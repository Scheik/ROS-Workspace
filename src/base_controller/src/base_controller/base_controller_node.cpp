//#include <iostream>                                         /* allows to perform standard input and output operations */
//#include <stdio.h>                                          /* Standard input/output definitions */
//#include <stdint.h>                                         /* Standard input/output definitions */
//#include <stdlib.h>                                         /* defines several general purpose functions */
//#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
//#include <ctype.h>                                          /* isxxx() */
#include <termios.h>                                        /* POSIX terminal control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* Custom message /encoders */
#include <base_controller/md49data.h>                       /* Custom message /encoders */
#include <serialport/serialport.h>

const char* serialport_name="/dev/ttyAMA0";                   /* defines used serialport on BPi. Use "/dev/ttyAMA0" for RPi*/
int serialport_bps=38400;                                  /* defines used baudrate on serialport */
int fd;                                                     /* serial port file descriptor */
struct termios orig;                                        // backuped port options
unsigned char speed_l=128, speed_r=128;                     // speed to set for MD49
unsigned char last_speed_l=128, last_speed_r=128;           // speed to set for MD49
double vr = 0.0;
double vl = 0.0;
double max_vr = 0.2;
double max_vl = 0.2;
double min_vr = 0.2;
double min_vl = 0.2;
double base_width = 0.4;                                    /* Base width in meters */
unsigned char serialBuffer[18];                             /* Serial buffer to store uart data */

void read_MD49_Data (void);
void set_MD49_speed (void);
int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);

base_controller::encoders encoders;
base_controller::md49data md49data;
cereal::CerealPort device;
char reply[8];
#define TIMEOUT 1000

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){
    if (vel_cmd.linear.x>0){
        speed_l = 255;
        speed_r = 255;
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
    ROS_INFO("base_controller: Received /cmd_vel message; requested speed_l=%i, speed_r=%i",speed_l,speed_r);

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
}


int main( int argc, char* argv[] ){

    // Init node
    // *********
    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",10);
    ros::Publisher md49data_pub = n.advertise<base_controller::md49data>("md49data",10);
    ros::Rate loop_rate(10);
    ROS_INFO("base_controller running...");
    ROS_INFO("=============================");
    ROS_INFO("Subscribing to topic /cmd_vel");
    ROS_INFO("Publishing to topic /encoders");
    ROS_INFO("Publishing to topic /md49data");

/*
    // Open serialport
    // ***************
    fd = openSerialPort(serialport_name, serialport_bps);
    if (fd == -1){
        ROS_FATAL("Could not open serialport at %s with %i",serialport_name,serialport_bps);
        exit(1);
    }
    ROS_INFO("Opend serialport %s with %i Bps",serialport_name,serialport_bps);
    usleep(10000); // Sleep for UART to power up and set options
*/
    // Open serialport
    // ***************
    try{ device.open(serialport_name, serialport_bps); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");


    // Init MD49 defaults
    // ******************
    ROS_INFO("base_controller: Init MD49:");
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x37;					// Command to enable md49 regulator
    writeBytes(fd, 2);
*/
    const char md49_enable_regulator[] = {0x00,0x37};        // Command to enable md49 regulator
    device.write(md49_enable_regulator,2);
    md49data.regulator=1;
    ROS_INFO("Set MD49 Regulator=Enabled");
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x39;					// Command to enable md49 timeout
    writeBytes(fd, 2);
*/
    const char md49_enable_timeout[] = {0x00,0x39};        // Command to enable md49 timeout
    device.write(md49_enable_timeout,2);
    md49data.timeout=1;
    ROS_INFO("Set MD49 Timeout=Enabled");
    // Read actual acceleration
    // as serial data from MD49
    // ************************
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x2A;					// Command to return acceleration
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_acceleration[] = {0x00,0x2A};        // Command to read md49 set acceleration
    device.write(md49_read_acceleration,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    ROS_INFO("Got this reply (Acc.): %i", reply[0]);
    md49data.acceleration=reply[0];
    // Read actual mode
    // as serial data from MD49
    // ************************
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x2B;					// Command to return mode
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_mode[] = {0x00,0x2B};        // Command to read md49 set acceleration
    device.write(md49_read_mode,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    ROS_INFO("Got this reply (Mode): %i", reply[0]);
    md49data.mode=reply[0];

    // Mainloop
    // ********
    while(n.ok())
    {
        // set speed on MD49 via UART as set through /cmd_vel
        // if speed_l or speed_r changed since last cycle
        // **************************************************
        if ((speed_l!=last_speed_l) || (speed_r!=last_speed_r)){
            set_MD49_speed();
            last_speed_l=speed_l;
            last_speed_r=speed_r;
        }
        // Read encoder- and other data from MD49 via UART
        // ***********************************************
        read_MD49_Data();
        // Publish encoder values as read to topic /encoders
        // *************************************************
        encoders_pub.publish(encoders);
        // Publish MD49 data as read to topic /md49data
        // ********************************************
        md49data_pub.publish(md49data);
        // ********************************************
        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
    return 1;
} // end.main



void read_MD49_Data (void){
    // Read encoder values as serial data from MD49
    // ********************************************
/*    serialBuffer[0] = 0;
    serialBuffer[1] = 0x25;					// Command to return encoder values
    writeBytes(fd, 2);
    readBytes(fd, 8);
*/
    const char md49_read_encoders[] = {0x00,0x25};
    device.write(md49_read_encoders,2);
    // Get the reply, the last value is the timeout in ms
    try{ device.read(reply, 8, TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    //ROS_INFO("Got this reply: %i,%i,%i,%i,%i,%i,%i,%i", reply[0], reply[1], reply[2],reply[3], reply[4], reply[5], reply[6], reply[7]);

    // Set all values of custom message /encoders,
    // *******************************************
    encoders.encoder_l = reply[0] << 24;                       // Put together first encoder value
    encoders.encoder_l |= (reply[1] << 16);
    encoders.encoder_l |= (reply[2] << 8);
    encoders.encoder_l |= (reply[3]);
    encoders.encoder_r = reply[4] << 24;                       // Put together second encoder value
    encoders.encoder_r |= (reply[5] << 16);
    encoders.encoder_r |= (reply[6] << 8);
    encoders.encoder_r |= (reply[7]);
    encoders.encoderbyte1l=reply[0];
    encoders.encoderbyte2l=reply[1];
    encoders.encoderbyte3l=reply[2];
    encoders.encoderbyte4l=reply[3];
    encoders.encoderbyte1r=reply[4];
    encoders.encoderbyte2r=reply[5];
    encoders.encoderbyte3r=reply[6];
    encoders.encoderbyte4r=reply[7];

    // Read actual volts
    // as serial data from MD49
    // ************************
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x26;					// Command to return volts value
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_volts[] = {0x00,0x26};        // Command to read md49 set acceleration
    device.write(md49_read_volts,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    //ROS_INFO("Got this reply (volts): %i", reply[0]);
    md49data.volts=reply[0];
    // Read actual current_l and current_r
    // as serial data from MD49
    // ***********************************
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x27;					// Command to return current_l value
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_current_l[] = {0x00,0x27};        // Command to read md49 set acceleration
    device.write(md49_read_current_l,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    //ROS_INFO("Got this reply (current_l): %i", reply[0]);
    md49data.current_l =reply[0];
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x28;					// Command to return volts value
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_current_r[] = {0x00,0x28};        // Command to read md49 set acceleration
    device.write(md49_read_current_r,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    //ROS_INFO("Got this reply (current_r): %i", reply[0]);
    md49data.current_r =reply[0];
    // Read actual error
    // as serial data from MD49
    // ************************
/*  serialBuffer[0] = 0;
    serialBuffer[1] = 0x2D;					// Command to return error value
    writeBytes(fd, 2);
    readBytes(fd, 1);
*/
    const char md49_read_error[] = {0x00,0x28};        // Command to read md49 set acceleration
    device.write(md49_read_error,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e)
    {
        ROS_ERROR("Timeout!");
    }
    //ROS_INFO("Got this reply (error): %i", reply[0]);
    md49data.error=reply[0];

/*    // Output MD49 data on screen
    // **************************
    printf("\033[2J");                                      //  clear the screen
    printf("\033[H");                                       //  position cursor at top-left corner
    printf ("MD49-Data read from AVR-Master: \n");
    printf("========================================\n");
    printf("Encoder1 Byte1: %i ",encoders.encoderbyte1l);
    printf("Byte2: %i ",encoders.encoderbyte2l);
    printf("Byte3: % i ",encoders.encoderbyte3l);
    printf("Byte4: %i \n",encoders.encoderbyte4l);
    printf("Encoder2 Byte1: %i ",encoders.encoderbyte1r);
    printf("Byte2: %i ",encoders.encoderbyte2r);
    printf("Byte3: %i ",encoders.encoderbyte3r);
    printf("Byte4: %i \n",encoders.encoderbyte4r);
    printf("EncoderL: %i ",encoders.encoder_l);
    printf("EncoderR: %i \n",encoders.encoder_r);
    printf("========================================\n");
    printf("SpeedL: %i ",md49data.speed_l);
    printf("SpeedR: %i \n",md49data.speed_r);
    printf("Volts: %i \n",md49data.volts);
    printf("CurrentL: %i ",md49data.current_l);
    printf("CurrentR: %i \n",md49data.current_r);
    printf("Error: %i \n",md49data.error);
    printf("Acceleration: %i \n",md49data.acceleration);
    printf("Mode: %i \n",md49data.mode);
    printf("Regulator: %i \n",md49data.regulator);
    printf("Timeout: %i \n",md49data.timeout);
*/
}

void set_MD49_speed(void){
    // set serial command for speed_l
    // *******************************
 /* serialBuffer[0] = 0;
    serialBuffer[1] = 0x31;					// Command to set motor speed
    serialBuffer[2] = speed_l;				// Speed to be set
*/
    const char md49_set_speed_l[]={0x00,0x31,speed_l};
    device.write(md49_set_speed_l,3);
    // set serial command for speed_r
    // ********************************
/*    serialBuffer[3] = 0;
    serialBuffer[4] = 0x32;
    serialBuffer[5] = speed_r;
*/
    const char md49_set_speed_r[]={0x00,0x32,speed_r};
    device.write(md49_set_speed_r,3);
    // send serialBuffer via UART
    // **************************
    //writeBytes(fd, 6);
    md49data.speed_l=speed_l;
    md49data.speed_r=speed_r;
}

// Open serialport
// ***************
int openSerialPort(const char * device, int bps){

   struct termios neu;
   //char buf[128];

   //fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
   fd = open(device, O_RDWR | O_NOCTTY);

   if (fd == -1)
   {
      ROS_WARN("openSerialPort %s error", device);
      //sprintf(buf, "openSerialPort %s error", device);
      //perror(buf);
   }
   else
   {
      tcgetattr(fd, &orig);                                 /* save current serial settings */
      tcgetattr(fd, &neu);
      cfmakeraw(&neu);
      cfsetispeed(&neu, bps);
      cfsetospeed(&neu, bps);
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd, TCSANOW, &neu);                         /* set new serial settings */
      fcntl (fd, F_SETFL, O_RDWR);
   }
   return fd;
}

// Write bytes serial to UART
// **************************
void writeBytes(int descriptor, int count) {
    if ((write(descriptor, serialBuffer, count)) == -1) {   // Send data out
        ROS_WARN("Error: writing at serialport");
        //perror("Error writing");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }

}

// Read bytes serial from UART
// ***************************
void readBytes(int descriptor, int count) {
    if (read(descriptor, serialBuffer, count) == -1) {      // Read back data into buf[]
        ROS_WARN("Error: reading from serialport");
        //perror("Error reading ");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }
}


