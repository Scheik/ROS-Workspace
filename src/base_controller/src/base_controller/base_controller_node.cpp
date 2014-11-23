#include <iostream>
#include <stdio.h>   								/* Standard input/output definitions */
#include <stdint.h>   								/* Standard input/output definitions */
#include <stdlib.h>  								/* exit */
//#include <string>  								/* String function definitions */
#include <unistd.h>  								/* UNIX standard function definitions */
#include <fcntl.h>  	 							/* File control definitions */
#include <errno.h>   								/* Error number definitions */
#include <termios.h> 								/* POSIX terminal control definitions */
#include <ctype.h>   								/* isxxx() */

#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <geometry_msgs/Twist.h>                    /* Message keeps encoder-values as Vector3.x (left) and .y (right) */
#include <base_controller/encoders.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


const char* serialport="/dev/ttyAMA0";
int serialport_bps=B38400;
int16_t EncoderL;
int16_t EncoderR;
int16_t previous_EncoderL;
int16_t previous_EncoderR;
double deltaLeft;
double deltaRight;
double meter_per_tick = 0.001;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx;
double vy;
double vth;
double Kettenabstand = 0.4;
double Antriebsrolle_Radius=0.03;
double vl = 0.0;
double vr = 0.0;

ros::Time current_time_encoder, last_time_encoder;
geometry_msgs::Quaternion odom_quat;

struct termios orig;
int filedesc;
int fd;                                             // serial port file descriptor
unsigned char serialBuffer[16];						// Serial buffer to store data for I/O

int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void read_MD49_Data (void);



void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){ 
    ROS_DEBUG("geometry_msgs/Twist received: linear.x= %f angular.z= %f", vel_cmd.linear.x, vel_cmd.angular.z);

    //ANFANG Alternative
    double vel_linear_x = vel_cmd.linear.x;
    double vel_angular_z = vel_cmd.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    if(vel_linear_x == 0){
        // turning
        right_vel = vel_angular_z * Kettenabstand / 2.0;
        left_vel = (-1) * right_vel;
    }
    else if(vel_angular_z == 0){
        // forward / backward
        left_vel = right_vel = vel_linear_x;
    }
    else{
        // moving doing arcs
        left_vel = vel_linear_x - vel_angular_z * Kettenabstand / 2.0;
        right_vel = vel_linear_x + vel_angular_z * Kettenabstand / 2.0;
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
    ros::Subscriber sub = n.subscribe("/cmd_vel", 100, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<geometry_msgs::Vector3>("encoders", 100);
    //ros::Publisher encoder_l_pub = n.advertise<std_msgs::Int16>("encoder_l", 100);
    //ros::Publisher encoder_r_pub = n.advertise<std_msgs::Int16>("encoder_r", 100);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    filedesc = openSerialPort("/dev/ttyAMA0", B38400);
    if (filedesc == -1) exit(1);
    usleep(40000);// Sleep for UART to power up and set options
    ROS_DEBUG("Serial Port opened \n");

    //ros::Time current_time, last_time;
    current_time_encoder = ros::Time::now();
    last_time_encoder = ros::Time::now();
    ros::Rate loop_rate(10);

    while( n.ok() )
    {    
            // Read encoder and other data from MD49
            // *************************************
            read_MD49_Data();

            current_time_encoder = ros::Time::now();

            // calculate odomety
            // *****************
            deltaLeft = EncoderL - previous_EncoderL;
            deltaRight = EncoderR - previous_EncoderR;

            vx = deltaLeft * meter_per_tick;
            vy = deltaRight * meter_per_tick;

            previous_EncoderL = EncoderL;
            previous_EncoderR = EncoderR;
            //last_time_encoder = current_time_encoder;
            //compute odometry in a typical way given the velocities of the robot
            double dt = (current_time_encoder - last_time_encoder).toSec();
            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time_encoder;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time_encoder;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            //publish the message
            odom_pub.publish(odom);

            last_time_encoder = current_time_encoder;

            // Publish EncoderL and EncoderR to topics encoder_l and encoder_r
            // ***************************************************************
            // std_msgs::Int16 encoder_l;
            // std_msgs::Int16 encoder_r;
            // encoder_l.data = EncoderL;
            // encoder_r.data=EncoderR;
            // encoder_l_pub.publish(encoder_l);
            // encoder_r_pub.publish(encoder_r);

            geometry_msgs::Vector3 encoders;
            encoders.x=EncoderL;
            encoders.y=EncoderR;
            encoders_pub.publish(encoders);


            // Loop
            // ****
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

    //printf("vl= %f \n", vl);
    //printf("vr= %f \n", vr);


}
