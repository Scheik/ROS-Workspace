#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int16_t Joy_Button_Up_pressed;
int16_t Joy_Button_Down_pressed;
int16_t Joy_Button_Left_pressed;
int16_t Joy_Button_Right_pressed;
bool Joy_Set=false;

#define Joystick_Buttons_Left_Right msg->axes[4]
#define Joystick_Buttons_Up_Down msg->axes[5]

void myCallback (const sensor_msgs::Joy::ConstPtr& msg)
{
  //for (unsigned i = 0; i < msg->axes.size(); ++i) {
    //ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
  //}

  // Joystick Button pressed?
  if (Joystick_Buttons_Left_Right==1) {Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;};
  if (Joystick_Buttons_Left_Right==-1) {Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;};
  if (Joystick_Buttons_Up_Down==1) {Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;};
  if (Joystick_Buttons_Up_Down==-1) {Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;};
  if (Joystick_Buttons_Up_Down==0) {Joy_Button_Down_pressed=Joystick_Buttons_Up_Down;};
  if (Joystick_Buttons_Left_Right==0) {Joy_Button_Left_pressed=Joystick_Buttons_Left_Right;};
  if (Joystick_Buttons_Up_Down==0) {Joy_Button_Up_pressed=Joystick_Buttons_Up_Down;};
  if (Joystick_Buttons_Left_Right==0) {Joy_Button_Right_pressed=Joystick_Buttons_Left_Right;};
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joystick_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1000, myCallback);
   //to the /cmd_vel topic, with a queue size of 100
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate loop_rate(10);
  //Declares the message to be sent
  geometry_msgs::Twist cmd_vel_msg;

  while (n.ok()){

    //
    //cmd_vel_msg.linear.x =2;
    //cmd_vel_pub.publish(cmd_vel_msg);
      if (Joy_Button_Up_pressed==1){
          ROS_INFO("Up");
          cmd_vel_msg.linear.x=2.0;
          cmd_vel_pub.publish(cmd_vel_msg);
      }
      else if (Joy_Button_Down_pressed==-1){
          ROS_INFO("Down");
          cmd_vel_msg.linear.x=-2.0;
          cmd_vel_pub.publish(cmd_vel_msg);
      }
      else if (Joy_Button_Left_pressed==1){
          ROS_INFO("Left");
          cmd_vel_msg.angular.z=2.0;
          cmd_vel_pub.publish(cmd_vel_msg);
      }
      else if (Joy_Button_Right_pressed==-1){
          ROS_INFO("Right");
          cmd_vel_msg.angular.z=-2.0;
          cmd_vel_pub.publish(cmd_vel_msg);
      }
      else {cmd_vel_msg.linear.x=0;cmd_vel_msg.angular.z=0;cmd_vel_pub.publish(cmd_vel_msg);};

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

