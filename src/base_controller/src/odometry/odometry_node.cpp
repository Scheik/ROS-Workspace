#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main( int argc, char* argv[] ){
    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("/encoder_l", 100, cmd_vel_callback);

}
