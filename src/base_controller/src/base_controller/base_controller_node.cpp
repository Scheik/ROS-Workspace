#include <iostream>
#include <stdio.h>   								/* Standard input/output definitions */
#include <stdint.h>   								/* Standard input/output definitions */
#include <stdlib.h>  								/* exit */
#include <string.h>  								/* String function definitions */
#include <unistd.h>  								/* UNIX standard function definitions */
#include <fcntl.h>  	 							/* File control definitions */
#include <errno.h>   								/* Error number definitions */
#include <termios.h> 								/* POSIX terminal control definitions */
#include <ctype.h>   								/* isxxx() */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/JointState.h>
//#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int16.h"

int16_t EncoderL;
int16_t EncoderR;

double width_robot = 0.4;
double vl = 0.0;
double vr = 0.0;
ros::Time last_time;
double right_enc = 0.0;
double left_enc = 0.0;
double right_enc_old = 0.0;
double left_enc_old = 0.0;
double distance_left = 0.0;
double distance_right = 0.0;
double ticks_per_meter = 100;
double x = 0.0;
double y = 0.0;
double th = 0.0;
geometry_msgs::Quaternion odom_quat;

struct termios orig;
int filedesc;
int fd;
unsigned char serialBuffer[16];						// Serial buffer to store data for I/O

int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void read_MD49_Data (void);



void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){ 
    //ROS_INFO("I heard: [%f]", vel_cmd.linear.y);
    //std::cout << "Twist Received " << std::endl;

    //ANFANG Alternative
    double vel_linear_x = vel_cmd.linear.x;
    double vel_angular_z = vel_cmd.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    if(vel_linear_x == 0){
        // turning
        right_vel = vel_angular_z * width_robot / 2.0;
        left_vel = (-1) * right_vel;
    }
    else if(vel_angular_z == 0){
        // forward / backward
        left_vel = right_vel = vel_linear_x;
    }
    else{
        // moving doing arcs
        left_vel = vel_linear_x - vel_angular_z * width_robot / 2.0;
        right_vel = vel_linear_x + vel_angular_z * width_robot / 2.0;
    }
    vl = left_vel;
    vr = right_vel;
    //ENDE Alternative


    if (vel_cmd.linear.x>0){
        serialBuffer[0] = 88;							// 88 =X Steuerbyte um Commands an MD49 zu senden
        serialBuffer[1] = 115;							// 115=s Steuerbyte setSpeed
        serialBuffer[2] = 255;					// speed1
        serialBuffer[3] = 255;					// speed2
        writeBytes(fd, 4);
    }
    if (vel_cmd.linear.x<0){
        serialBuffer[0] = 88;							// 88 =X Steuerbyte um Commands an MD49 zu senden
        serialBuffer[1] = 115;							// 115=s Steuerbyte setSpeed
        serialBuffer[2] = 0;					// speed1
        serialBuffer[3] = 0;					// speed2
        writeBytes(fd, 4);
    }
    if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){
        serialBuffer[0] = 88;							// 88 =X Steuerbyte um Commands an MD49 zu senden
        serialBuffer[1] = 115;							// 115=s Steuerbyte setSpeed
        serialBuffer[2] = 128;					// speed1
        serialBuffer[3] = 128;					// speed2
        writeBytes(fd, 4);
    }
    if (vel_cmd.angular.z>0){
        serialBuffer[0] = 88;							// 88 =X Steuerbyte um Commands an MD49 zu senden
        serialBuffer[1] = 115;							// 115=s Steuerbyte setSpeed
        serialBuffer[2] = 0;					// speed1
        serialBuffer[3] = 255;					// speed2
        writeBytes(fd, 4);
    }
    if (vel_cmd.angular.z<0){
        serialBuffer[0] = 88;							// 88 =X Steuerbyte um Commands an MD49 zu senden
        serialBuffer[1] = 115;							// 115=s Steuerbyte setSpeed
        serialBuffer[2] = 255;					// speed1
        serialBuffer[3] = 0;					// speed2
        writeBytes(fd, 4);
    }
}


int main( int argc, char* argv[] ){
    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    // Node subscribes to topic /cmd_vel (geometry_msgs/Twist)
    ros::Subscriber sub = n.subscribe("/cmd_vel", 100, cmd_vel_callback);
    // Node publishes to topics /lwheel and /rwheel
    ros::Publisher lwheel_pub = n.advertise<std_msgs::Int16>("lwheel", 100);
    ros::Publisher rwheel_pub = n.advertise<std_msgs::Int16>("rwheel", 100);


    filedesc = openSerialPort("/dev/ttyAMA0", B38400);
    if (filedesc == -1) exit(1);
    usleep(40000);// Sleep for UART to power up and set options

    //ROS_INFO_STREAM("serial Port opened \n");
    ros::Rate loop_rate(10);

    while( n.ok() )
    {
            read_MD49_Data();
            std_msgs::Int16 lwheel;
            std_msgs::Int16 rwheel;
            lwheel.data = EncoderL;
            rwheel.data=EncoderR;
            lwheel_pub.publish(lwheel);
            rwheel_pub.publish(rwheel);

            ros::spinOnce();
            loop_rate.sleep();
    }

    return 1;
}

int openSerialPort(const char * device, int bps){
   struct termios neu;
   char buf[128];

   fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

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
    EncoderL = serialBuffer[0] << 24;                        // Put together first encoder value
    EncoderL |= (serialBuffer[1] << 16);
    EncoderL |= (serialBuffer[2] << 8);
    EncoderL |= (serialBuffer[3]);
    EncoderR = serialBuffer[4] << 24;                        // Put together second encoder value
    EncoderR |= (serialBuffer[5] << 16);
    EncoderR |= (serialBuffer[6] << 8);
    EncoderR |= (serialBuffer[7]);


    printf("\033[2J");        /*  clear the screen  */
    printf("\033[H");         /*  position cursor at top-left corner */
    printf ("MD49-Data read from AVR-Master: \n");
    printf("====================================================== \n");
    printf("Encoder1 Byte1: %i ",serialBuffer[0]);
    printf("Byte2: %i ",serialBuffer[1]);
    printf("Byte3: % i ",serialBuffer[2]);
    printf("Byte4: %i \n",serialBuffer[3]);
    printf("Encoder2 Byte1: %i ",serialBuffer[4]);
    printf("Byte2: %i ",serialBuffer[5]);
    printf("Byte3: %i ",serialBuffer[6]);
    printf("Byte4: %i \n",serialBuffer[7]);
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

    printf("vl= %f \n", vl);
    printf("vr= %f \n", vr);


}
