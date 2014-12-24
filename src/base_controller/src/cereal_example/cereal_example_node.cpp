#include <ros/ros.h>
#include <serialport/serialport.h>

#include <sstream>
#include <string>

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
    try{ device.open("/dev/ttyAMA0", 38400); }
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
        //unsigned char zeichen="0";
        device.write("0");
        device.write("%");
        //ss("");
        //ss << 0x25;
        //device.write(ss.str());
        //device.write("%");

        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        ROS_INFO("Got this reply: %i,%i,%i,%i,%i,%i,%i,%i", reply[0], reply[1], reply[2],reply[3], reply[4], reply[5], reply[6], reply[7]);

        ros::spinOnce();
        r.sleep();
    }   
}
