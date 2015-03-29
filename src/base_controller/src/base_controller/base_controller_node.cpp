#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* custom message /encoders */
#include <base_controller/md49data.h>                       /* custom message /encoders */
#include <serialport/serialport.h>                          // library for serial communications via UART

#define TIMEOUT 1000                                        // timeout for reading serialport in ms
#define ON true
#define OFF false
#define OR ||
#define NOT !=

void md49_set_speed (void);
void md49_set_mode(int mode);
void md49_set_acceleration(int acceleration);
void md49_enable_timeout(void);
void md49_enable_regulator(void);
void md49_disable_timeout(void);
void md49_disable_regulator(void);
void md49_get_acceleration(void);
void md49_get_mode(void);
void md49_get_encoders(void);
void md49_get_speed(void);
void md49_get_currents(void);
void md49_get_error(void);
void md49_get_volts(void);

base_controller::encoders encoders;                         // topic /encoders
base_controller::md49data md49data;                         // topic /md49data
cereal::CerealPort device;                                  // serialport
std::string serialport;                                     // keeps used serialport on Pi, is read from parameters server
int serialport_bps;                                         // keeps used baudrate, is read from parameters server
int md49_mode;                                              // keeps MD49 Mode, is read from parameters server
int md49_acceleration;                                      // keeps MD49 Acceleration,  is read from parameters server
bool md49_timeout;
bool md49_regulator;
int speed_l, speed_r;                                       // default speed_l and speed_r for MD49
int last_speed_l=128, last_speed_r=128;                     // buffers last set speed_l and speed_r
char reply[8];                                              // max buffersize serial input

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){
    if (vel_cmd.linear.x != 0){
        speed_l = 128+(635*vel_cmd.linear.x);
        speed_r = 128+(635*vel_cmd.linear.x);
    }
    if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){
        speed_l = 128;
        speed_r = 128;
    }
    if (vel_cmd.angular.z != 0){
        speed_l = 128 - (127*vel_cmd.angular.z);
        speed_r = 128 + (127*vel_cmd.angular.z);
        //speed_l = 0;
        //speed_r = 255;
    }
    //if (vel_cmd.angular.z<0){
    //    speed_l = 255;
    //    speed_r = 0;
    //}
    ROS_INFO("base_controller: Received /cmd_vel message. Requested speed_l=%i, speed_r=%i",speed_l,speed_r);

    /*
    double vr = 0.0;
    double vl = 0.0;
    double max_vr = 0.2;
    double max_vl = 0.2;
    double min_vr = 0.2;
    double min_vl = 0.2;
    double base_width = 0.4;                                // Base width in meters
    //ANFANG Alternative
    if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){vl=0;vr=0;}
    else if(vel_cmd.linear.x == 0){
        // turning
        vr = vel_cmd.angular.z * base_width / 2.0;
        vl = (-1) * vr;
    }
    else if(vel_cmd.angular.z == 0){
        // forward / backward
        vl = vr = vel_cmd.linear.x;
    }
    else{
        // moving doing arcs
        vl = vel_cmd.linear.x - vel_cmd.angular.z * base_width / 2.0;
        if (vl > max_vl) {vl=max_vl;}
        if (vl < min_vl) {vl=min_vl;}
        vr = vel_cmd.linear.x + vel_cmd.angular.z * base_width / 2.0;
        if (vr > max_vr) {vr=max_vr;}
        if (vr < min_vr) {vr=min_vr;}
    }
    //ENDE Alternative
    */
}

int main( int argc, char* argv[] ){

    // Init node
    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",10);
    ros::Publisher md49data_pub = n.advertise<base_controller::md49data>("md49data",10);
    ros::Rate loop_rate(10);
    n.param<std::string>("serialport/name", serialport, "/dev/ttyAMA0");       // Get serialportname from ROS Parameter sevice, default is ttyAMA0 (RPis GPIO UART)
    n.param("serialport/bps", serialport_bps, 38400);                          // Get serialport bps from ROS Parameter sevice, default is 38400Bps
    n.param("md49/mode", md49_mode, 0);                                        // Get MD49 Mode from ROS Parameter sevice, default is Mode=0
    n.param("md49/acceleration", md49_acceleration, 5);                        // Get MD49 Acceleration from ROS Parameter sevice, default is Acceleration=0
    n.param("md49/regulator", md49_regulator, ON);                             // Get MD49 Regulator from ROS Parameter sevice, default is Regulator=ON
    n.param("md49/timeout", md49_timeout, ON);                                 // Get MD49 Timeout from ROS Parameter sevice, default is Timeout=ON
    n.param("md49/speed_l", speed_l, 128);                                     // Get MD49 speed_l from ROS Parameter sevice, default is spee_l=128
    n.param("md49/speed_r", speed_r, 128);                                     // Get MD49 speed_r from ROS Parameter sevice, default is spee_r=128
    ROS_INFO("base_controller: base_controller running...");

    // Open serialport
    try{ device.open(serialport.c_str(), serialport_bps); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("base_controller: Failed to open serialport %s!",serialport.c_str());
        ROS_BREAK();
    }
    ROS_INFO("base_controller: Opened Serialport at %s with %i bps.",serialport.c_str(),serialport_bps);

    // Set MD49 defaults
    md49_set_speed();
    md49_enable_timeout();
    md49_enable_regulator();
    md49_set_mode(md49_mode);
    md49_set_acceleration(md49_acceleration);

    // Mainloop
    while(n.ok())
    {
        // set speed on MD49 via UART as set through /cmd_vel
        // if speed_l or speed_r changed since last cycle
        if ((speed_l NOT last_speed_l) OR (speed_r NOT last_speed_r)){
            md49_set_speed();
        }
        // Read encoder- data from MD49 via UART
        md49_get_encoders();
        // Publish encoder values as read to topic /encoders
        encoders_pub.publish(encoders);
        // Read other- data from MD49 via UART
        md49_get_speed();
        md49_get_volts();
        md49_get_currents();
        //md49_get_acceleration();
        //md49_get_mode();
        md49_get_error();
        // Publish MD49 data as read to topic /md49data
        md49data_pub.publish(md49data);
        // Loop
        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
    return 1;
} // end.main

void md49_set_speed(void){
    // set and send serial command for speed_l
    const char md49_set_speed_l[]={0x00,0x31,speed_l};
    device.write(md49_set_speed_l,3);
    // set and send serial command for speed_r
    const char md49_set_speed_r[]={0x00,0x32,speed_r};
    device.write(md49_set_speed_r,3);
    //Alter speed_l and speed_r in message for topic /md49data
    md49data.speed_l=speed_l;
    md49data.speed_r=speed_r;
    last_speed_l=speed_l; last_speed_r=speed_r;
    ROS_INFO("base_controller: Set speed_l=%i and speed_r=%i on MD49", speed_l,speed_r);
}

void md49_set_mode(int mode){
    const char md49_set_mode[]={0x00,0x34,mode};
    device.write(md49_set_mode,3);
    ROS_INFO("base_controller: Set mode=%i on MD49", mode);
    md49data.mode=mode;
}

void md49_set_acceleration(int acceleration){
    const char md49_set_acceleration[]={0x00,0x33,acceleration};
    device.write(md49_set_acceleration,3);
    ROS_INFO("base_controller: Set acceleration=%i on MD49", acceleration);
    md49data.acceleration=acceleration;
}

void md49_enable_timeout(void){
    const char md49_enable_timeout[] = {0x00,0x39};        // Command to enable md49 timeout
    device.write(md49_enable_timeout,2);
    md49data.timeout=1;
    ROS_INFO("base_controller: Enabled timeout on MD49");
}

void md49_enable_regulator(void){
    const char md49_enable_regulator[] = {0x00,0x37};        // Command to enable md49 regulator
    device.write(md49_enable_regulator,2);
    md49data.regulator=1;
    ROS_INFO("base_controller: Enabled regulator on MD49");
}

void md49_disable_timeout(void){
    const char md49_disable_timeout[] = {0x00,0x38};        // Command to enable md49 regulator
    device.write(md49_disable_timeout,2);
    md49data.timeout=0;
    ROS_INFO("base_controller: Disabled timeout on MD49");
}

void md49_disable_regulator(void){
    const char md49_disable_regulator[] = {0x00,0x36};        // Command to enable md49 timeout
    device.write(md49_disable_regulator,2);
    md49data.regulator=0;
    ROS_INFO("base_controller: Disabled regulator on MD49");
}

void md49_get_acceleration(void){
    const char md49_get_acceleration[] = {0x00,0x2A};        // Command to read md49 set acceleration
    device.write(md49_get_acceleration,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 acceleration!");
    }
    //ROS_INFO("base_controller: MD49 Acceleration= %i", reply[0]);
    md49data.acceleration=reply[0];
}

void md49_get_mode(void){
    const char md49_get_mode[] = {0x00,0x2B};        // Command to read md49 set acceleration
    device.write(md49_get_mode,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 Mode!");
    }
    //ROS_INFO("base_controller: MD49 Mode= %i", reply[0]);
    md49data.mode=reply[0];
}

void md49_get_speed(void){
    const char md49_get_speed_l[] = {0x00,0x21};        // Command to read md49 set acceleration
    device.write(md49_get_speed_l,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 speed_l!");
    }
    md49data.speed_l=reply[0];
    const char md49_get_speed_r[] = {0x00,0x22};        // Command to read md49 set acceleration
    device.write(md49_get_speed_r,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, acceleration
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 speed_r!");
    }
    md49data.speed_r=reply[0];
    //ROS_INFO("base_controller: MD49 speed_l= %i, speed_r= %i", speed_l,speed_r);
}

void md49_get_encoders(void){
    // Read encoder values data from MD49
    // **********************************
    const char md49_get_encoders[] = {0x00,0x25};
    device.write(md49_get_encoders,2);
    // Get the reply, the last value is the timeout in ms
    try{ device.read(reply, 8, TIMEOUT); }
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 encodervalues!");
    }
    // Set all values of custom message /encoders,
    // *******************************************
    encoders.encoder_l = reply[0] << 24;                       // Put together first encoder value
    encoders.encoder_l |= (reply[1] << 16);
    encoders.encoder_l |= (reply[2] << 8);
    encoders.encoder_l |= (reply[3]);
    encoders.encoder_r = reply[4] << 24;                       // Put together second encoder value
    encoders.encoder_r |= (reply[5] << 16);
    encoders.encoder_r |= (reply[6] << 8);
    encoders.encoder_r |= (reply[7]);
    encoders.encoderbyte1l=reply[0];
    encoders.encoderbyte2l=reply[1];
    encoders.encoderbyte3l=reply[2];
    encoders.encoderbyte4l=reply[3];
    encoders.encoderbyte1r=reply[4];
    encoders.encoderbyte2r=reply[5];
    encoders.encoderbyte3r=reply[6];
    encoders.encoderbyte4r=reply[7];
    //ROS_INFO("Got this reply: %i,%i,%i,%i,%i,%i,%i,%i", reply[0], reply[1], reply[2],reply[3], reply[4], reply[5], reply[6], reply[7]);
}

void md49_get_volts(void){
    // Read actual volts
    // as serial data from MD49
    // ************************
    const char md49_get_volts[] = {0x00,0x26};        // Command to read md49 set volts
    device.write(md49_get_volts,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, volts
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 volts!");
    }
    //ROS_INFO("Got this reply (volts): %i", reply[0]);
    md49data.volts=reply[0];
}

void md49_get_currents(void){
    // Read actual current_l and current_r
    // as serial data from MD49
    // ***********************************
    const char md49_get_current_l[] = {0x00,0x27};        // Command to read md49 set current_l
    device.write(md49_get_current_l,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, current_l
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 current_l!");
    }
    //ROS_INFO("Got this reply (current_l): %i", reply[0]);
    md49data.current_l =reply[0];
    const char md49_get_current_r[] = {0x00,0x28};        // Command to read md49 set current_r
    device.write(md49_get_current_r,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, current_r
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 current_r!");
    }
    //ROS_INFO("Got this reply (current_r): %i", reply[0]);
    md49data.current_r =reply[0];
}

void md49_get_error(void){
    // Read actual error
    // as serial data from MD49
    // ************************
    const char md49_get_error[] = {0x00,0x2D};        // Command to read md49 set error
    device.write(md49_get_error,2);
    try{ device.read(reply, 1, TIMEOUT); }                             // get answer, error
    catch(cereal::TimeoutException& e){
        ROS_ERROR("base_controller: Timeout reading MD49 errorbyte!");
    }
    //ROS_INFO("Got this reply (error): %i", reply[0]);
    md49data.error=reply[0];
}

void md49_reset_encoders(void){
    const char md49_reset_encoders[] = {0x00,0x35};        // Command to enable md49 timeout
    device.write(md49_reset_encoders,2);
    ROS_INFO("base_controller: Reset encoders on MD49");
}
