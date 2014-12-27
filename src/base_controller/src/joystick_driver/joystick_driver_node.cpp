// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Standard Libraries
#include <stdlib.h>
#include <math.h>
// Own Libraries

// Variables
int16_t Joy_Button_Up_pressed=0;
int16_t Joy_Button_Down_pressed=0;
int16_t Joy_Button_Left_pressed=0;
int16_t Joy_Button_Right_pressed=0;
int16_t Joy_Button_Up_pressed_last=0;
int16_t Joy_Button_Down_pressed_last=0;
int16_t Joy_Button_Left_pressed_last=0;
int16_t Joy_Button_Right_pressed_last=0;

float Joy_AnalogLeft_Up_pressed;
float Joy_AnalogLeft_Down_pressed;
float Joy_AnalogLeft_Left_pressed;
float Joy_AnalogLeft_Right_pressed;
float Joy_AnalogLeft_Up_pressed_last=0;
float Joy_AnalogLeft_Down_pressed_last=0;
float Joy_AnalogLeft_Left_pressed_last=0;
float Joy_AnalogLeft_Right_pressed_last=0;

double linear_x=0;
double angular_z=0;

// Defines
#define Joystick_Buttons_Left_Right msg->axes[4]
#define Joystick_Buttons_Up_Down msg->axes[5]
#define Joystick_AnalogLeft_Left_Right msg->axes[0]
#define Joystick_AnalogLeft_Up_Down msg->axes[1]

// Callback Functions
void Joy_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
    //for (unsigned i = 0; i < msg->axes.size(); ++i) {
    //  ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
    //}

    // Joystick Buttons pressed?
    if (Joystick_Buttons_Left_Right==1) {
        Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;
        angular_z=1.0;
    }
    if (Joystick_Buttons_Left_Right==-1) {
        Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;
        angular_z=-1.0;
    }
    if (Joystick_Buttons_Up_Down==1) {
        Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;
        linear_x=0.2;
    }
    if (Joystick_Buttons_Up_Down==-1) {
        Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;
        linear_x=-0.2;
    }
    if (Joystick_Buttons_Up_Down==0) {
        Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;
        Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;
        linear_x=0;
    }
    if (Joystick_Buttons_Left_Right==0) {
        Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;
        Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;
        angular_z=0;
    }

}

void onExit( void )
{
    // Run cleanup code here!
    ROS_INFO("joystick_driver: Exit node");
}

int main(int argc, char **argv){

    atexit(onExit);

    ros::init(argc, argv, "joystick_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 100, Joy_Callback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(50);                                                  //Sets the loop to publish at a rate of 10Hz
  geometry_msgs::Twist cmd_vel_msg;                                         //Declares the message to be sent

  while (n.ok()){

      if ((cmd_vel_msg.linear.x != linear_x) || (cmd_vel_msg.angular.z != angular_z)){
          cmd_vel_msg.linear.x=linear_x;
          cmd_vel_msg.angular.z=angular_z;
          cmd_vel_pub.publish(cmd_vel_msg);
          ROS_INFO("linear.x = %f angular.z = %f",cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
      }


    ros::spinOnce();
    loop_rate.sleep();
  }//end.mainloop

  return 0;
}//end.main

