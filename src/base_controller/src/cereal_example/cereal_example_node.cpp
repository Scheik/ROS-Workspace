#include <ros/ros.h>
#include <serialport/serialport.h>

#include <sstream>
#include <string>

#define REPLY_SIZE 8
#define TIMEOUT 1000

char* itoa(int value, char* result, int base);

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
        //const char send[] = "\x00\x25";
        //or
        const char send[] = {0x00,0x25};
        device.write(send,2);
        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        ROS_INFO("Got this reply: %i,%i,%i,%i,%i,%i,%i,%i", reply[0], reply[1], reply[2],reply[3], reply[4], reply[5], reply[6], reply[7]);

        const char send1[]={0x00,0x31,128};
        device.write(send1,3);

        ros::spinOnce();
        r.sleep();
    }   
}

char* itoa(int value, char* result, int base) {
        // check that the base if valid
        if (base < 2 || base > 36) { *result = '\0'; return result; }

        char* ptr = result, *ptr1 = result, tmp_char;
        int tmp_value;

        do {
            tmp_value = value;
            value /= base;
            *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        } while ( value );

        // Apply negative sign
        if (tmp_value < 0) *ptr++ = '-';
        *ptr-- = '\0';
        while(ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr--= *ptr1;
            *ptr1++ = tmp_char;
        }
        return result;
}
