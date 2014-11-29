// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Standard Libraries
#include <stdlib.h>
#include <math.h>
// Own Libraries

// Variables
int16_t Joy_Button_Up_pressed;
int16_t Joy_Button_Down_pressed;
int16_t Joy_Button_Left_pressed;
int16_t Joy_Button_Right_pressed;
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

    // Joystick Analogstick left moved?
    if (Joystick_AnalogLeft_Left_Right>0) {Joy_AnalogLeft_Left_pressed=Joystick_AnalogLeft_Left_Right;};
    if (Joystick_AnalogLeft_Left_Right<0) {Joy_AnalogLeft_Right_pressed=Joystick_AnalogLeft_Left_Right;};
    if (Joystick_AnalogLeft_Up_Down>0) {Joy_AnalogLeft_Up_pressed=Joystick_AnalogLeft_Up_Down;};
    if (Joystick_AnalogLeft_Up_Down<0) {Joy_AnalogLeft_Down_pressed=Joystick_AnalogLeft_Up_Down;};
    if (Joystick_AnalogLeft_Up_Down==0) {Joy_AnalogLeft_Down_pressed=Joystick_AnalogLeft_Up_Down;Joy_AnalogLeft_Up_pressed=Joystick_AnalogLeft_Up_Down;};
    if (Joystick_AnalogLeft_Left_Right==0) {Joy_AnalogLeft_Left_pressed=Joystick_AnalogLeft_Left_Right;Joy_AnalogLeft_Right_pressed=Joystick_AnalogLeft_Left_Right;};


    // Joystick Buttons pressed?
    if (Joystick_Buttons_Left_Right==1) {Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;};
    if (Joystick_Buttons_Left_Right==-1) {Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;};
    if (Joystick_Buttons_Up_Down==1) {Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;};
    if (Joystick_Buttons_Up_Down==-1) {Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;};
    if (Joystick_Buttons_Up_Down==0) {Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;};
    if (Joystick_Buttons_Left_Right==0) {Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;};
}

int main(int argc, char **argv){
  ros::init(argc, argv, "joystick_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1000, Joy_Callback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  //Sets the loop to publish at a rate of 10Hz
  ros::Rate loop_rate(1);
  //Declares the message to be sent
  geometry_msgs::Twist cmd_vel_msg;
  while (n.ok()){


/*
      // PUBLISH to /cmd_vel if Joystick corresponding Buttons are pressed
      if (Joy_Button_Up_pressed==1){
          ROS_INFO("Button Up pressed");
          cmd_vel_msg.linear.x=0.2;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Up_pressed_last=Joy_Button_Up_pressed;
      }
      else if (Joy_Button_Down_pressed==-1){
          ROS_INFO("Button Down pressed");
          cmd_vel_msg.linear.x=-0.2;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Down_pressed_last=Joy_Button_Down_pressed;
      }
      else if (Joy_Button_Left_pressed==1){
          ROS_INFO("Button Left pressed");
          cmd_vel_msg.angular.z=1.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Left_pressed_last=Joy_Button_Left_pressed;
      }
      else if (Joy_Button_Right_pressed==-1){
          ROS_INFO("Button Right pressed");
          cmd_vel_msg.angular.z=-1.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Right_pressed_last=Joy_Button_Right_pressed;
      }
      else if (Joy_Button_Up_pressed==0){
          ROS_INFO("Button Up released");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Up_pressed_last=Joy_Button_Up_pressed;
      }
      else if (Joy_Button_Down_pressed==0){
          ROS_INFO("Button Down released");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Down_pressed_last=Joy_Button_Down_pressed;
      }
      else if (Joy_Button_Left_pressed==0){
          ROS_INFO("Button Left released");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
         // Joy_Button_Left_pressed_last=Joy_Button_Left_pressed;
      }
      else if (Joy_Button_Right_pressed==0){
          ROS_INFO("Button Right released");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          //Joy_Button_Right_pressed_last=Joy_Button_Right_pressed;
      }
*/



      // PUBLISH to /cmd_vel if Joystick corresponding Buttons are pressed
      if (Joy_Button_Up_pressed==1 && Joy_Button_Up_pressed!=Joy_Button_Up_pressed_last){
          ROS_INFO("Button Up pressed");
          cmd_vel_msg.linear.x=0.2;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Up_pressed_last=Joy_Button_Up_pressed;
      }
      else if (Joy_Button_Down_pressed==-1 && Joy_Button_Down_pressed!=Joy_Button_Down_pressed_last){
          ROS_INFO("Button Down pressed");
          cmd_vel_msg.linear.x=-0.2;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Down_pressed_last=Joy_Button_Down_pressed;
      }
      else if (Joy_Button_Left_pressed==1 && Joy_Button_Left_pressed!=Joy_Button_Left_pressed_last){
          ROS_INFO("Button Left pressed");
          cmd_vel_msg.angular.z=1.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Left_pressed_last=Joy_Button_Left_pressed;
      }
      else if (Joy_Button_Right_pressed==-1 && Joy_Button_Right_pressed!=Joy_Button_Right_pressed_last){
          ROS_INFO("Button Right pressed");
          cmd_vel_msg.angular.z=-1.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Right_pressed_last=Joy_Button_Right_pressed;
      }
      else if (Joy_Button_Up_pressed==0 && Joy_Button_Up_pressed!=Joy_Button_Up_pressed_last){
          ROS_INFO("Button Up released");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Up_pressed_last=Joy_Button_Up_pressed;
      }
      else if (Joy_Button_Down_pressed==0 && Joy_Button_Down_pressed!=Joy_Button_Down_pressed_last){
          ROS_INFO("Button Down released");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Down_pressed_last=Joy_Button_Down_pressed;
      }
      else if (Joy_Button_Left_pressed==0 && Joy_Button_Left_pressed!=Joy_Button_Left_pressed_last){
          ROS_INFO("Button Left released");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Left_pressed_last=Joy_Button_Left_pressed;
      }
      else if (Joy_Button_Right_pressed==0 && Joy_Button_Right_pressed!=Joy_Button_Right_pressed_last){
          ROS_INFO("Button Right released");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_Button_Right_pressed_last=Joy_Button_Right_pressed;
      }



/*
      // PUBLISH to /cmd_vel if AnalogeLeft is moved
      if (Joy_AnalogLeft_Up_pressed>0 && Joy_AnalogLeft_Up_pressed!=Joy_AnalogLeft_Up_pressed_last){
          ROS_INFO("AnalogLeft Up changed");
          cmd_vel_msg.linear.x=floor((Joy_AnalogLeft_Up_pressed*0.2)*100+0.5)/100;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Up_pressed_last=Joy_AnalogLeft_Up_pressed;
      }
      else if (Joy_AnalogLeft_Down_pressed<0 && Joy_AnalogLeft_Down_pressed!=Joy_AnalogLeft_Down_pressed_last){
          ROS_INFO("AnalogLeft Down changed");
          cmd_vel_msg.linear.x=floor((Joy_AnalogLeft_Down_pressed*0.2)*100+0.5)/100;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Down_pressed_last=Joy_AnalogLeft_Down_pressed;
      }
      else if (Joy_AnalogLeft_Left_pressed>0 && Joy_AnalogLeft_Left_pressed!=Joy_AnalogLeft_Left_pressed_last){
          ROS_INFO("AnalogLeft Left changed");
          cmd_vel_msg.angular.z=floor((Joy_AnalogLeft_Left_pressed*1.0)*100+0.5)/100;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Left_pressed_last=Joy_AnalogLeft_Left_pressed;
      }
      else if (Joy_AnalogLeft_Right_pressed<0 && Joy_AnalogLeft_Right_pressed!=Joy_AnalogLeft_Right_pressed_last){
          ROS_INFO("AnalogLeft Right changed");
          cmd_vel_msg.angular.z=floor((Joy_AnalogLeft_Right_pressed*1.0)*100+0.5)/100;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Right_pressed_last=Joy_AnalogLeft_Right_pressed;
      }
      else if (Joy_AnalogLeft_Up_pressed==0 && Joy_AnalogLeft_Up_pressed!=Joy_AnalogLeft_Up_pressed_last){
          ROS_INFO("AnalogLeft Up changed");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Up_pressed_last=Joy_AnalogLeft_Up_pressed;
      }
      else if (Joy_AnalogLeft_Down_pressed==0 && Joy_AnalogLeft_Down_pressed!=Joy_AnalogLeft_Down_pressed_last){
          ROS_INFO("AnalogLeft Down changed");
          cmd_vel_msg.linear.x=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Down_pressed_last=Joy_AnalogLeft_Down_pressed;
      }
      else if (Joy_AnalogLeft_Left_pressed==0 && Joy_AnalogLeft_Left_pressed!=Joy_AnalogLeft_Left_pressed_last){
          ROS_INFO("AnalogLeft Left changed");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Left_pressed_last=Joy_AnalogLeft_Left_pressed;
      }
      else if (Joy_AnalogLeft_Right_pressed==0 && Joy_AnalogLeft_Right_pressed!=Joy_AnalogLeft_Right_pressed_last){
          ROS_INFO("AnalogLeft Right changed");
          cmd_vel_msg.angular.z=0.0;
          cmd_vel_pub.publish(cmd_vel_msg);
          Joy_AnalogLeft_Right_pressed_last=Joy_AnalogLeft_Right_pressed;
      }
*/
    ros::spinOnce();
    loop_rate.sleep();
  }//end.mainloop
  return 0;
}//end.main

