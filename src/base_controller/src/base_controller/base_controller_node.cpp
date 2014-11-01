#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{ 
    ROS_INFO("I heard: [%f]", vel_cmd.linear.y);
    std::cout << "Twist Received " << std::endl;
}


int main( int argc, char* argv[] )
{
ros::init(argc, argv, "toeminator_publisher" );

ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmd_vel_callback);

while( n.ok() ) 
{
    ros::spin();
}

return 1;
}
