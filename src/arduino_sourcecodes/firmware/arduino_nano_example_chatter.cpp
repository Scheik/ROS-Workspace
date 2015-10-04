#include <ros.h>
#include <std_msgs/String.h>
#include <Arduino.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher arduino_nano_example_chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(arduino_nano_example_chatter);
}

void loop()
{
  str_msg.data = hello;
  arduino_nano_example_chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
