#include <iostream>                                         /* allows to perform standard input and output operations */
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
//#include <string>                                         /* String function definitions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <termios.h>                                        /* POSIX terminal control definitions */
#include <ctype.h>                                        /* isxxx() */
#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* Custom message /encoders */

const char* serialport="/dev/ttyAMA0";                      /* defines used serialport */
int serialport_bps=B38400;                                  /* defines baudrate od serialport */
int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
unsigned char speed_l=128, speed_r=128;                               /* speed to set for MD49 */
bool cmd_vel_received=true;
double vr = 0.0;
double vl = 0.0;
double max_vr = 0.2;
double max_vl = 0.2;
double min_vr = 0.2;
double min_vl = 0.2;
double base_width = 0.4;                                    /* Base width in meters */

int filedesc;                                               // File descriptor of serial port we will talk to
int fd;                                                     /* serial port file descriptor */
unsigned char serialBuffer[16];                             /* Serial buffer to store uart data */
struct termios orig;                                        // Port options


int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void read_MD49_Data (void);
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r);


void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){

//    ROS_DEBUG("geometry_msgs/Twist received: linear.x= %f angular.z= %f", vel_cmd.linear.x, vel_cmd.angular.z);

/*
    //ANFANG Alternative
    if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){vl=0;vr=0;}
    else if(vel_cmd.linear.x == 0){
        // turning
        vr = vel_cmd.angular.z * base_width / 2.0;
        vl = (-1) * vr;
    }
    else if(vel_cmd.angular.z == 0){
        // forward / backward
        vl = vr = vel_cmd.linear.x;
    }
    else{
        // moving doing arcs
        vl = vel_cmd.linear.x - vel_cmd.angular.z * base_width / 2.0;
        if (vl > max_vl) {vl=max_vl;}
        if (vl < min_vl) {vl=min_vl;}
        vr = vel_cmd.linear.x + vel_cmd.angular.z * base_width / 2.0;
        if (vr > max_vr) {vr=max_vr;}
        if (vr < min_vr) {vr=min_vr;}
    }
    //ENDE Alternative
*/

    if (vel_cmd.linear.x>0){
        speed_l = 255;
        speed_r = 255;
        //serialBuffer[0] = 88;					// 88 =X Steuerbyte um Commands an MD49 zu senden
        //serialBuffer[1] = 115;					// 115=s Steuerbyte setSpeed
        //serialBuffer[2] = 255;					// speed1
        //serialBuffer[3] = 255;					// speed2
        //writeBytes(fd, 4);
    }
    if (vel_cmd.linear.x<0){
        speed_l = 0;
        speed_r = 0;
    }
    if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){
        speed_l = 128;
        speed_r = 128;
    }
    if (vel_cmd.angular.z>0){
        speed_l = 0;
        speed_r = 255;
    }
    if (vel_cmd.angular.z<0){
        speed_l = 255;
        speed_r = 0;
    }

    cmd_vel_received=true;
}


int main( int argc, char* argv[] ){

    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 100, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",100);

    // Open serial port
    // ****************
    filedesc = openSerialPort("/dev/ttyAMA0", B38400);
    if (filedesc == -1) exit(1);
    usleep(10000);                                      // Sleep for UART to power up and set options
    ROS_DEBUG("Serial Port opened \n");

    // Set nodes looprate 10Hz
    // ***********************
    ros::Rate loop_rate(5);

    while( n.ok() )
    {
        ros::spinOnce();
        // Read encoder and other data from MD49
            // *************************************
            read_MD49_Data();

            // Set speed left and right for MD49
            // ********************************
            if (cmd_vel_received==true) {
                set_MD49_speed(speed_l,speed_r);
                //set_MD49_speed(speed_l,speed_r);
                cmd_vel_received=false;
            }
            //set_MD49_speed(speed_l,speed_r);

            // Publish encoder values to topic /encoders (custom message)
            // ********************************************************************
            base_controller::encoders encoders;
            encoders.encoder_l=EncoderL;
            encoders.encoder_r=EncoderR;
            encoders_pub.publish(encoders);

            // Loop
            // ****

            loop_rate.sleep();
    }// end.mainloop

    return 1;
} // end.main

int openSerialPort(const char * device, int bps){
   struct termios neu;
   char buf[128];

   //fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
   fd = open(device, O_RDWR | O_NOCTTY);

   if (fd == -1)
   {
      sprintf(buf, "openSerialPort %s error", device);
      perror(buf);
   }
   else
   {
      tcgetattr(fd, &orig); 						/* save current serial settings */
      tcgetattr(fd, &neu);
      cfmakeraw(&neu);
      //fprintf(stderr, "speed=%d\n", bps);
      cfsetispeed(&neu, bps);
      cfsetospeed(&neu, bps);
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd, TCSANOW, &neu); 				/* set new serial settings */
      fcntl (fd, F_SETFL, O_RDWR);
   }
   return fd;
}

void writeBytes(int descriptor, int count) {
    if ((write(descriptor, serialBuffer, count)) == -1) {	// Send data out
        perror("Error writing");
        close(descriptor);				// Close port if there is an error
        exit(1);
    }
    //write(fd,serialBuffer, count);

}

void readBytes(int descriptor, int count) {
    if (read(descriptor, serialBuffer, count) == -1) {	// Read back data into buf[]
        perror("Error reading ");
        close(descriptor);				// Close port if there is an error
        exit(1);
    }
}

void read_MD49_Data (void){
    serialBuffer[0] = 82;							// 82=R Steuerbyte um alle Daten vom MD49 zu lesen
    writeBytes(fd, 1);
    //Daten lesen und in Array schreiben
    readBytes(fd, 18);






    printf("\033[2J");        /*  clear the screen  */
    printf("\033[H");         /*  position cursor at top-left corner */
    printf ("MD49-Data read from AVR-Master: \n");
    printf("====================================================== \n");
    //printf("Encoder1 Byte1: %i ",serialBuffer[0]);
    //printf("Byte2: %i ",serialBuffer[1]);
    //printf("Byte3: % i ",serialBuffer[2]);
    //printf("Byte4: %i \n",serialBuffer[3]);
    //printf("Encoder2 Byte1: %i ",serialBuffer[4]);
    //printf("Byte2: %i ",serialBuffer[5]);
    //printf("Byte3: %i ",serialBuffer[6]);
    //printf("Byte4: %i \n",serialBuffer[7]);
    printf("EncoderL: %i ",EncoderL);
    printf("EncoderR: %i \n",EncoderR);
    printf("====================================================== \n");
    printf("Speed1: %i ",serialBuffer[8]);
    printf("Speed2: %i \n",serialBuffer[9]);
    printf("Volts: %i \n",serialBuffer[10]);
    printf("Current1: %i ",serialBuffer[11]);
    printf("Current2: %i \n",serialBuffer[12]);
    printf("Error: %i \n",serialBuffer[13]);
    printf("Acceleration: %i \n",serialBuffer[14]);
    printf("Mode: %i \n",serialBuffer[15]);
    printf("Regulator: %i \n",serialBuffer[16]);
    printf("Timeout: %i \n",serialBuffer[17]);

   // printf("vl= %f \n", vl);
  //  printf("vr= %f \n", vr);
    //EncoderL = serialBuffer[0] << 24;                        // Put together first encoder value
    //EncoderL |= (serialBuffer[1] << 16);
    //EncoderL |= (serialBuffer[2] << 8);
    //EncoderL |= (serialBuffer[3]);
    //EncoderR = serialBuffer[4] << 24;                        // Put together second encoder value
    //EncoderR |= (serialBuffer[5] << 16);
    //EncoderR |= (serialBuffer[6] << 8);
    //EncoderR |= (serialBuffer[7]);
}

void set_MD49_speed (unsigned char speed_l, unsigned char speed_r){
    serialBuffer[0] = 88;					// 88 =X Steuerbyte um Commands an MD49 zu senden
    serialBuffer[1] = 115;					// 115=s Steuerbyte setSpeed
    serialBuffer[2] = speed_l;					// speed1
    serialBuffer[3] = speed_r;					// speed2
    writeBytes(fd, 4);
}
