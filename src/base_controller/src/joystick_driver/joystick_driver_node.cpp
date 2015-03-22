// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Standard Libraries
#include <stdlib.h>
#include <math.h>

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
/*    for (unsigned i = 0; i < msg->axes.size(); ++i) {
      ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
    }
*/

    // Joystick Buttons pressed?
    if (Joystick_Buttons_Left_Right==1) {
        angular_z=Joystick_Buttons_Left_Right;
    }
    if (Joystick_Buttons_Left_Right==-1) {
        angular_z=Joystick_Buttons_Left_Right;
    }
    if (Joystick_Buttons_Up_Down==1) {
        linear_x=Joystick_Buttons_Up_Down;
    }
    if (Joystick_Buttons_Up_Down==-1) {
        linear_x=Joystick_Buttons_Up_Down;
    }
    if (Joystick_Buttons_Up_Down==0) {
        linear_x=Joystick_Buttons_Up_Down;
    }
    if (Joystick_Buttons_Left_Right==0) {
        angular_z=Joystick_Buttons_Up_Down;
    }

    // Joystick Analog pressed?
    if (Joystick_AnalogLeft_Left_Right>0) {
        angular_z=Joystick_AnalogLeft_Left_Right;
    }
    if (Joystick_AnalogLeft_Left_Right<0) {
        angular_z=Joystick_AnalogLeft_Left_Right;
    }
    if (Joystick_AnalogLeft_Up_Down>0) {
        linear_x=Joystick_AnalogLeft_Up_Down;
    }
    if (Joystick_AnalogLeft_Up_Down<0) {
        linear_x=Joystick_AnalogLeft_Up_Down;
    }
    if (Joystick_AnalogLeft_Up_Down==0) {
        linear_x=Joystick_AnalogLeft_Up_Down;
    }
    if (Joystick_AnalogLeft_Left_Right==0) {
        angular_z=Joystick_AnalogLeft_Left_Right;
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

