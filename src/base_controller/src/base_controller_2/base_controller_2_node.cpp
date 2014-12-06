#include <iostream>                                         /* allows to perform standard input and output operations */
#include <fstream>
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <ctype.h>                                          /* isxxx() */
#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* Custom message /encoders */
#include <base_controller/md49data.h>                       /* Custom message /encoders */

#include <serialport/serialport.h>
#define REPLY_SIZE 18
#define TIMEOUT 1000

int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
char reply[REPLY_SIZE];
unsigned char speed_l=128, speed_r=128;                     /* speed to set for MD49 */
unsigned char last_speed_l=128, last_speed_r=128;           /* speed to set for MD49 */
double vr = 0.0;
double vl = 0.0;
double max_vr = 0.2;
double max_vl = 0.2;
double min_vr = 0.2;
double min_vl = 0.2;
double base_width = 0.4;                                    /* Base width in meters */

//unsigned char serialBuffer[18];                             /* Serial buffer to store uart data */
void read_MD49_Data (void);
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r);
char* itoa(int value, char* result, int base);

using namespace std;
cereal::CerealPort device;
base_controller::encoders encoders;
base_controller::md49data md49data;

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
        if ((speed_l != last_speed_l) || (speed_r != last_speed_r)){
            //set_MD49_speed(speed_l,speed_r);
            last_speed_l=speed_l;
            last_speed_r=speed_r;
        }

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

    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",10);
    ros::Publisher md49data_pub = n.advertise<base_controller::md49data>("md49data",10);

    // Init node
    // *********
    ros::Rate loop_rate(10);
    ROS_INFO("base_controller running...");
    ROS_INFO("=============================");
    ROS_INFO("Subscribing to topic /cmd_vel");
    ROS_INFO("Publishing to topic /encoders");
    ROS_INFO("Publishing to topic /md49data");
    sleep(2);

    // Open serial port
    // ****************
    try{ device.open("/dev/ttyAMA0", 38400); }
    catch(cereal::Exception& e)
    {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");


    while(n.ok())
    {
        // Read encoder and other data from MD49
        // (data is read from serial_controller_node
        //  and avaiable through md49_data.txt)
        // *****************************************
        read_MD49_Data();

        // Publish encoder values to topic /encoders (custom message)
        // **********************************************************       
        encoders.encoder_l=EncoderL;
        encoders.encoder_r=EncoderR;
        encoders_pub.publish(encoders);

        // Publish MD49 data to topic /md49data (custom message)
        // *****************************************************        
        md49data.speed_l = reply[8];
        md49data.speed_r = reply[9];
        md49data.volt = reply[10];
        md49data.current_l = reply[11];
        md49data.current_r = reply[12];
        md49data.error = reply[13];
        md49data.acceleration = reply[14];
        md49data.mode = reply[15];
        md49data.regulator = reply[16];
        md49data.timeout = reply[17];
        md49data_pub.publish(md49data);

        // ****
        ros::spinOnce();
        loop_rate.sleep();

    }// end.mainloop

    return 1;
} // end.main


void read_MD49_Data (void){

    // Send 'R' over the serial port
    device.write("R");
    // Get the reply, the last value is the timeout in ms
    try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
      ROS_ERROR("Timeout on serialport! No data read");
    }
    //ROS_INFO("Received MD49 data");

    // Put toghether new encodervalues
    // *******************************
    EncoderL = reply[0] << 24;                        // Put together first encoder value
    EncoderL |= (reply[1] << 16);
    EncoderL |= (reply[2] << 8);
    EncoderL |= (reply[3]);
    EncoderR = reply[4] << 24;                        // Put together second encoder value
    EncoderR |= (reply[5] << 16);
    EncoderR |= (reply[6] << 8);
    EncoderR |= (reply[7]);

/*
    // Output MD49 data on screen
    // **************************
    printf("\033[2J");                                      //  clear the screen
    printf("\033[H");                                       //  position cursor at top-left corner
    printf ("MD49-Data read from AVR-Master: \n");
    printf("========================================\n");
    printf("Encoder1 Byte1: %i ",reply[0]);
    printf("Byte2: %i ",reply[1]);
    printf("Byte3: % i ",reply[2]);
    printf("Byte4: %i \n",reply[3]);
    printf("Encoder2 Byte1: %i ",reply[4]);
    printf("Byte2: %i ",reply[5]);
    printf("Byte3: %i ",reply[6]);
    printf("Byte4: %i \n",reply[7]);
    printf("EncoderL: %i ",EncoderL);
    printf("EncoderR: %i \n",EncoderR);
    printf("========================================\n");
    printf("Speed1: %i ",reply[8]);
    printf("Speed2: %i \n",reply[9]);
    printf("Volts: %i \n",reply[10]);
    printf("Current1: %i ",reply[11]);
    printf("Current2: %i \n",reply[12]);
    printf("Error: %i \n",reply[13]);
    printf("Acceleration: %i \n",reply[14]);
    printf("Mode: %i \n",reply[15]);
    printf("Regulator: %i \n",reply[16]);
    printf("Timeout: %i \n",reply[17]);
*/

}

void set_MD49_speed (unsigned char speed_l, unsigned char speed_r){



/*
    char buffer[33];
    ofstream myfile;
    myfile.open ("md49_commands.txt");
    //myfile << "Writing this to a file.\n";
    if (speed_l==0){
        myfile << "000";
        myfile << "\n";
    }
    else if (speed_l<10){
        myfile << "00";
        myfile << itoa(speed_l,buffer,10);
        myfile << "\n";
    }
    else if (speed_l<100){
        myfile << "0";
        myfile << itoa(speed_l,buffer,10);
        myfile << "\n";
    }
    else{
        myfile << itoa(speed_l,buffer,10);
        myfile << "\n";
    }

    if (speed_r==0){
        myfile << "000";
        myfile << "\n";
    }
    else if (speed_r<10){
        myfile << "00";
        myfile << itoa(speed_r,buffer,10);
        myfile << "\n";
    }
    else if (speed_r<100){
        myfile << "0";
        myfile << itoa(speed_r,buffer,10);
        myfile << "\n";
    }
    else{
        myfile << itoa(speed_r,buffer,10);
        myfile << "\n";
    }
    myfile.close();
*/

}

char* itoa(int value, char* result, int base) {
        // check that the base if valid
        if (base < 2 || base > 36) { *result = '\0'; return result; }

        char* ptr = result, *ptr1 = result, tmp_char;
        int tmp_value;

        do {
            tmp_value = value;
            value /= base;
            *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        } while ( value );

        // Apply negative sign
        if (tmp_value < 0) *ptr++ = '-';
        *ptr-- = '\0';
        while(ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr--= *ptr1;
            *ptr1++ = tmp_char;
        }
        return result;
    }
