// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Standard Libraries
#include <stdlib.h>
#include <math.h>

float linear_x_button=0;
float angular_z_button=0;
float linear_x_stick=0;
float angular_z_stick=0;

bool stick_used=false;
bool button_used=false;

// Defines
#define Joystick_Stick_Left_Right msg->axes[4]
#define Joystick_Stick_Up_Down msg->axes[5]
#define Joystick_Button_Left_Right msg->axes[0]
#define Joystick_Button_Up_Down msg->axes[1]

// Callback Functions
void Joy_Callback (const sensor_msgs::Joy::ConstPtr& msg)
{
//  for (unsigned i = 0; i < msg->axes.size(); ++i) {
//      ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
//  }

    // Test if joystick buttons are pressed?
    if (Joystick_Button_Left_Right>0) {
        angular_z_button=Joystick_Button_Left_Right;
        button_used = true;
    }
    if (Joystick_Button_Left_Right<0) {
        angular_z_button=Joystick_Button_Left_Right;
        button_used = true;
    }
    if (Joystick_Button_Up_Down>0) {
        linear_x_button=Joystick_Button_Up_Down/5;
        button_used = true;
    }
    if (Joystick_Button_Up_Down<0) {
        linear_x_button=Joystick_Button_Up_Down/5;
        button_used = true;
    }
    if (Joystick_Button_Up_Down==0) {
        linear_x_button=Joystick_Button_Up_Down;
        button_used = true;
    }
    if (Joystick_Button_Left_Right==0) {
        angular_z_button=Joystick_Button_Left_Right;
        button_used = true;
    }
    if ((Joystick_Button_Left_Right==0) && (Joystick_Button_Up_Down==0))
    {
        button_used=false;
    }

    // Test if joystick stick is pressed?
    if (Joystick_Stick_Left_Right>0) {
        angular_z_stick=Joystick_Stick_Left_Right;
        stick_used = true;
    }
    if (Joystick_Stick_Left_Right<0) {
        angular_z_stick=Joystick_Stick_Left_Right;
        stick_used = true;
    }
    if (Joystick_Stick_Up_Down>0) {
        linear_x_stick=Joystick_Stick_Up_Down/5;
        stick_used = true;
    }
    if (Joystick_Stick_Up_Down<0) {
        linear_x_stick=Joystick_Stick_Up_Down/5;
        stick_used = true;
    }
    if (Joystick_Stick_Up_Down==0) {
        linear_x_stick=Joystick_Stick_Up_Down;
        stick_used = true;
    }
    if (Joystick_Stick_Left_Right==0) {
        angular_z_stick=Joystick_Stick_Left_Right;
        stick_used = true;
    }
     if ((Joystick_Stick_Left_Right==0) && (Joystick_Stick_Up_Down==0))
     {
         stick_used=false;
     }


}

void onExit( void )
{
    // Run cleanup code here!
    ROS_INFO("joystick_teleop: Exit node");
}

int main(int argc, char **argv)
{
  atexit(onExit);
  ros::init(argc, argv, "joystick_teleop");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 100, Joy_Callback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(20);                                                  //Sets the loop to publish at a rate of 10Hz
  geometry_msgs::Twist cmd_vel_msg;                                         //Declares the message to be sent
  bool publish_cmd_vel=false;

  while (ros::ok()){

    if (button_used==true && stick_used==false){
      cmd_vel_msg.linear.x=linear_x_button;
      cmd_vel_msg.angular.z=angular_z_button;
      cmd_vel_pub.publish(cmd_vel_msg);
      ROS_INFO("linear.x = %f angular.z = %f",cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
      publish_cmd_vel = true;
    }
    if (stick_used==true && button_used==false){
      cmd_vel_msg.linear.x=linear_x_stick;
      cmd_vel_msg.angular.z=angular_z_stick;
      cmd_vel_pub.publish(cmd_vel_msg);
      ROS_INFO("linear.x = %f angular.z = %f",cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
      publish_cmd_vel = true;
    }
    // If nor button or stick is pressed, send null-message to cmd_vel once
    if ((stick_used==false && button_used==false) && publish_cmd_vel==true){
        cmd_vel_msg.linear.x=0;
        cmd_vel_msg.angular.z =0;
        cmd_vel_pub.publish(cmd_vel_msg);
        ROS_INFO("linear.x = %f angular.z = %f",cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
        publish_cmd_vel = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }//end.mainloop
  return 0;
}//end.main

