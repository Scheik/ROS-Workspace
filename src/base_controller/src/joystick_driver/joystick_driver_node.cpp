#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

void myCallback (const sensor_msgs::Joy::ConstPtr& msg)
{
  for (unsigned i = 0; i < msg->axes.size(); ++i) {
    ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1000, myCallback);
  ros::spin();

  return 0;
}

