#include <ros/ros.h>
#include <cereal_port/CerealPort.h>

#define REPLY_SIZE 8
#define TIMEOUT 1000

// This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_node");
    ros::NodeHandle n;

    cereal::CerealPort device;
    char reply[REPLY_SIZE];

    // Change the next line according to your port name and baud rate
    try{ device.open("/dev/ttyUSB0", 9600); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");

    ros::Rate r(1);
    while(ros::ok())
    {
        // Send 'R' over the serial port
        device.write("R");

        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        ROS_INFO("Got this reply: %s", reply);

        ros::spinOnce();
        r.sleep();
    }   
}
