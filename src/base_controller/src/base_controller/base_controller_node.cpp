/**
 * @file    base_controller_node.cpp
 * @author  Fabian Prinzing <scheik.todeswache@googlemail.com>
 * @version v2.0.0
 *
 * @section LICENSE
 *
 * Copyright (C) 2015, Fabian Prinzing. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Fabian Prinzing "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * Details
 */

#include <ros/ros.h>                                                                            /**<  Include ROS functionality */
#include <geometry_msgs/Twist.h>                                                                /**<  ROS Twist message */
#include <serialport/serialport.h>                                                              /**<  library for serial communications via UART*/
#include <custom_messages/md49_data.h>                                                          /**<  custom message /md49_data */
#include <custom_messages/md49_encoders.h>                                                      /**<  custom message /md49_encoders */

#define TIMEOUT 1000                                                                            /**<  timeout for reading serialport in ms */

cereal::CerealPort device;                                                                      /**<  serialport */
std::string serialport;                                                                         /**<  used serialport on pcDuino, is read from parameters server */
int serialport_bps;                                                                             /**<  used baudrate, is read from parameters server */
char reply[8];                                                                                  /**<  max buffersize serial input */


/**
 * @brief The SubscribeAndPublish class
 */
class SubscribeAndPublish
{

private:

    int requested_speed_l, requested_speed_r;                                                   /**<  requested speed_l and speed_r for MD49 computed from cmd_vel */
    int actual_speed_l, actual_speed_r;                                                         /**<  buffers actual set speed_l and speed_r for MD49 */

public:

    ros::NodeHandle n;
    ros::Subscriber vel_cmd_sub;
    ros::Publisher md49_encoders_pub;
    ros::Publisher md49_data_pub;

    /**
     * @brief SubscribeAndPublish constructor
     */
    SubscribeAndPublish()
    {
        vel_cmd_sub = n.subscribe("/cmd_vel", 10, &SubscribeAndPublish::cmd_vel_callback, this);
        md49_encoders_pub = n.advertise<custom_messages::md49_encoders>("md49_encoders",10);
        md49_data_pub = n.advertise<custom_messages::md49_data>("md49_data",10);
    }//end.constructor SubscribeAndPublish
    /**
     * @brief cmd_vel_callback
     * @param vel_cmd
     */
    void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
    {
        // Drive For- or Backward:
        if (vel_cmd.linear.x != 0){
            requested_speed_l = 128+(635*vel_cmd.linear.x);
            requested_speed_r = 128+(635*vel_cmd.linear.x);
        }
        // Drive stopped:
        if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){
            requested_speed_l = 128;
            requested_speed_r = 128;
        }
        // Turn clock- or counterclockwise:
        if (vel_cmd.angular.z != 0){
            requested_speed_l = 128 - (127*vel_cmd.angular.z);
            requested_speed_r = 128 + (127*vel_cmd.angular.z);
        }
        ROS_INFO("base_controller: Received /cmd_vel message. Requested speed_l=%i, speed_r=%i",requested_speed_l,requested_speed_r);
    }//end.cmd_vel_callback

    int get_requested_speed_l()
    {
        return requested_speed_l;
    }
    int get_requested_speed_r()
    {
        return requested_speed_r;
    }
    void set_requested_speed_l(int speed_l)
    {
        requested_speed_l=speed_l;
    }
    void set_requested_speed_r(int speed_r)
    {
        requested_speed_r=speed_r;
    }
    int get_actual_speed_l()
    {
        return actual_speed_l;
    }
    int get_actual_speed_r()
    {
        return actual_speed_r;
    }
    void set_actual_speed_l(int speed_l)
    {
        actual_speed_l=speed_l;
    }
    void set_actual_speed_r(int speed_r)
    {
        actual_speed_r=speed_r;
    }
};// End of class SubscribeAndPublish


/**
 * @brief The MD49 class
 */
class MD49
{
public:

    custom_messages::md49_data md49_data;                                                           /**<  genrate message /md49_data of type md49_data from package custom_messages */
    custom_messages::md49_encoders md49_encoders;                                                   /**<  genrate message /md49_encoders of type md49_encoders from package custom_messages */

    /**
     * @brief MD49
     * @param speed_l
     * @param speed_r
     * @param mode
     * @param acceleration
     * @param timeout
     * @param regulator
     */
    MD49(int initial_speed_l, int initial_speed_r, int initial_mode, int initial_acceleration, bool initial_timeout, bool initial_regulator)
    {
        set_speed(initial_speed_l,initial_speed_r);
        set_mode(initial_mode);
        set_acceleration(initial_acceleration);
        if (initial_timeout==true)
        {
            enable_timeout();
        }
        else if (initial_timeout==false)
        {
            disable_timeout();
        }
        if (initial_regulator==true)
        {
            enable_regulator();
        }
        else if (initial_regulator==false)
        {
            disable_regulator();
        }
    }//end.constructor MD49

    /**
     * @brief init
     * @param speed_l
     * @param speed_r
     * @param mode
     * @param acceleration
     * @param timeout
     * @param regulator
     */
    void init(int speed_l, int speed_r, int mode, int acceleration, bool timeout, bool regulator)
    {
        set_speed(speed_l,speed_r);
        set_mode(mode);
        set_acceleration(acceleration);
        if (timeout==true)
        {
            enable_timeout();
        }
        else if (timeout==false)
        {
            disable_timeout();
        }
        if (regulator==true)
        {
            enable_regulator();
        }
        else if (regulator==false)
        {
            disable_regulator();
        }
    }//end.init
    /**
     * @brief set_speed
     * @param speed_l
     * @param speed_r
     */
    void set_speed(int speed_l, int speed_r)
    {
        // set and send serial command for speed_l
        const char md49_set_speed_l[]={0x00,0x31,speed_l};
        device.write(md49_set_speed_l,3);
        // set and send serial command for speed_r
        const char md49_set_speed_r[]={0x00,0x32,speed_r};
        device.write(md49_set_speed_r,3);
        ROS_INFO("base_controller: Set speed_l=%i and speed_r=%i on MD49", speed_l,speed_r);
    }//end.set_speed
    /**
     * @brief set_mode
     * @param mode
     */
    void set_mode(int mode)
    {
        const char md49_set_mode[]={0x00,0x34,mode};
        device.write(md49_set_mode,3);
        ROS_INFO("base_controller: Set mode=%i on MD49", mode);
    }//end.set_mode
    /**
     * @brief set_acceleration
     * @param acceleration
     */
    void set_acceleration(int acceleration)
    {
        const char md49_set_acceleration[]={0x00,0x33,acceleration};
        device.write(md49_set_acceleration,3);
        ROS_INFO("base_controller: Set acceleration=%i on MD49", acceleration);
    }//end.set_acceleration
    /**
     * @brief enable_timeout
     */
    void enable_timeout(void){
        const char md49_enable_timeout[] = {0x00,0x39};             // put together command to enable md49 timeout
        device.write(md49_enable_timeout,2);
        md49_data.timeout=1;
        ROS_INFO("base_controller: Enabled timeout on MD49");
    }//end.enable_timeout
    /**
     * @brief enable_regulator
     */
    void enable_regulator(void){
        const char md49_enable_regulator[] = {0x00,0x37};           // put together command to enable md49 regulator
        device.write(md49_enable_regulator,2);
        md49_data.regulator=1;
        ROS_INFO("base_controller: Enabled regulator on MD49");
    }//end.enable_regulator
    /**
     * @brief disable_timeout
     */
    void disable_timeout(void){
        const char md49_disable_timeout[] = {0x00,0x38};            // put together command to enable md49 regulator
        device.write(md49_disable_timeout,2);
        md49_data.timeout=0;
        ROS_INFO("base_controller: Disabled timeout on MD49");
    }//end.disable_timeout
    /**
     * @brief disable_regulator
     */
    void disable_regulator(void){
        const char md49_disable_regulator[] = {0x00,0x36};          // put together command to enable md49 timeout
        device.write(md49_disable_regulator,2);
        md49_data.regulator=0;
        ROS_INFO("base_controller: Disabled regulator on MD49");
    }//end.disable_regulator
    /**
     * @brief get_acceleration
     * @return
     */
    int get_acceleration(){
        const char md49_get_acceleration[] = {0x00,0x2A};           // put together command to read md49 set acceleration
        device.write(md49_get_acceleration,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 acceleration!");
        }
        //ROS_INFO("base_controller: MD49 Acceleration= %i", reply[0]);
        return reply[0];
    }//end.get_acceleration
    /**
     * @brief get_mode
     * @return
     */
    int get_mode(){
        const char md49_get_mode[] = {0x00,0x2B};                   // put together command to read md49 set acceleration
        device.write(md49_get_mode,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 Mode!");
        }
        //ROS_INFO("base_controller: MD49 Mode= %i", reply[0]);
        return reply[0];
    }//end.get_mode
    /**
     * @brief get_speed_l
     * @return
     */
    int get_speed_l(){
        const char md49_get_speed_l[] = {0x00,0x21};                // put together command to read md49 set acceleration
        device.write(md49_get_speed_l,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 speed_l!");
        }
        return reply[0];
        //ROS_INFO("base_controller: MD49 speed_l= %i, speed_r= %i", speed_l,speed_r);
    }//end.get_speed_l
    /**
     * @brief get_speed_r
     * @return
     */
    int get_speed_r(){
        const char md49_get_speed_r[] = {0x00,0x22};                // put together command to read md49 set acceleration
        device.write(md49_get_speed_r,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 speed_r!");
        }
        return reply[0];
        //ROS_INFO("base_controller: MD49 speed_l= %i, speed_r= %i", speed_l,speed_r);
    }//end.get_speed_r
    /**
     * @brief get_encoders
     */
    void get_encoders(void){
        const char md49_get_encoders[] = {0x00,0x25};               // put together command to read md49 encoders
        device.write(md49_get_encoders,2);
        // ******************************************************
        // * Get the reply, the last value is the timeout in ms *
        // ******************************************************
        try{ device.read(reply, 8, TIMEOUT); }
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 encodervalues!");
        }
        // ***************************************************
        // * Set all values of custom message /md49_encoders *
        // ***************************************************
        md49_encoders.encoder_l = reply[0] << 24;                   // put together first encoder value
        md49_encoders.encoder_l |= (reply[1] << 16);
        md49_encoders.encoder_l |= (reply[2] << 8);
        md49_encoders.encoder_l |= (reply[3]);
        md49_encoders.encoder_r = reply[4] << 24;                   // put together second encoder value
        md49_encoders.encoder_r |= (reply[5] << 16);
        md49_encoders.encoder_r |= (reply[6] << 8);
        md49_encoders.encoder_r |= (reply[7]);
        md49_encoders.encoderbyte1l=reply[0];
        md49_encoders.encoderbyte2l=reply[1];
        md49_encoders.encoderbyte3l=reply[2];
        md49_encoders.encoderbyte4l=reply[3];
        md49_encoders.encoderbyte1r=reply[4];
        md49_encoders.encoderbyte2r=reply[5];
        md49_encoders.encoderbyte3r=reply[6];
        md49_encoders.encoderbyte4r=reply[7];
        //ROS_INFO("Got this reply: %i,%i,%i,%i,%i,%i,%i,%i", reply[0], reply[1], reply[2],reply[3], reply[4], reply[5], reply[6], reply[7]);
    }//end.get_encoders
    /**
     * @brief get_volts
     * @return
     */
    int get_volts(){
        const char md49_get_volts[] = {0x00,0x26};                  // put together command to read md49 volts
        device.write(md49_get_volts,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, volts
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 volts!");
        }
        //ROS_INFO("Got this reply (volts): %i", reply[0]);
        return reply[0];
    }//end.get_volts
    /**
     * @brief get_current_l
     * @return
     */
    int get_current_l(){
        const char md49_get_current_l[] = {0x00,0x27};              // put together command to read md49 current_l
        device.write(md49_get_current_l,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, current_l
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 current_l!");
        }
        //ROS_INFO("Got this reply (current_l): %i", reply[0]);
        return reply[0];
    }//end.get_current_l
    /**
     * @brief get_current_r
     * @return
     */
    int get_current_r(){
        const char md49_get_current_r[] = {0x00,0x28};              // put together command to read md49 current_r
        device.write(md49_get_current_r,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, current_r
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 current_r!");
        }
        //ROS_INFO("Got this reply (current_r): %i", reply[0]);
        return reply[0];
    }//end.get_current_r
    /**
     * @brief get_error
     * @return
     */
    int get_error(){
        const char md49_get_error[] = {0x00,0x2D};                  // put together command to read md49 error
        device.write(md49_get_error,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, error
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 errorbyte!");
        }
        //ROS_INFO("Got this reply (error): %i", reply[0]);
        return reply[0];
    }//end.get_error
    /**
     * @brief reset_encoders
     */
    void reset_encoders(void){
        const char md49_reset_encoders[] = {0x00,0x35};             // put together command to reset md49 encoders
        device.write(md49_reset_encoders,2);
        ROS_INFO("base_controller: Reset encoders on MD49");
    }//end.reset_encoders
};//End of class MD49


/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "base_controller" );                                                          /**< Init as ROS node */
    ros::Rate loop_rate(10);                                                                            /**< Set ROS loop rate */

    // **********************************************************************
    // * Create instance mySubscribeAndPublish of class SubscribeAndPublish *
    // **********************************************************************
    SubscribeAndPublish mySubscribeAndPublish;                                                          /**< Generate instance mySubsribeAndPublish of class SubscribeAndPublish */

    int initial_mode_from_parameter_service;                                                            /**< Stores initial mode to be read from parameter service */
    int initial_acceleration_from_parameter_service;                                                    /**< Stores initial acceleration to be read from parameter service */
    bool initial_timeout_from_parameter_service;                                                        /**< Stores initial timeout state to be read from parameter service */
    bool initial_regulator_from_parameter_service;                                                      /**< Stores initial regulator state to be read from parameter service */
    int initial_speed_l_from_parameter_service;                                                         /**< Stores initial speed_l to be read from parameter service */
    int initial_speed_r_from_parameter_service;                                                         /**< Stores initial speed_r to be read from parameter service */

    // ********************************************
    // * Load parameters from config/.yaml- files *
    // ********************************************
    mySubscribeAndPublish.n.param<std::string>("serialport/name", serialport, "/dev/ttyS0");            /**< Get serialportname from ROS Parameter sevice, default is ttyS0 (pcDuinos GPIO UART) */
    mySubscribeAndPublish.n.param("serialport/bps", serialport_bps, 38400);                             /**< Get serialport bps from ROS Parameter sevice, default is 38400Bps */
    mySubscribeAndPublish.n.param("md49/mode", initial_mode_from_parameter_service, 0);                 /**< Get MD49 Mode from ROS Parameter sevice, default is Mode=0 */
    mySubscribeAndPublish.n.param("md49/acceleration", initial_acceleration_from_parameter_service, 5); /**< Get MD49 Acceleration from ROS Parameter sevice, default is Acceleration=0 */
    mySubscribeAndPublish.n.param("md49/regulator", initial_timeout_from_parameter_service, true);      /**< Get MD49 Regulator from ROS Parameter sevice, default is Regulator=ON */
    mySubscribeAndPublish.n.param("md49/timeout", initial_regulator_from_parameter_service, true);      /**< Get MD49 Timeout from ROS Parameter sevice, default is Timeout=ON */
    mySubscribeAndPublish.n.param("md49/speed_l", initial_speed_l_from_parameter_service, 128);        /**< Get MD49 speed_l from ROS Parameter sevice, default is spee_l=128 */
    mySubscribeAndPublish.n.param("md49/speed_r", initial_speed_r_from_parameter_service, 128);        /**< Get MD49 speed_r from ROS Parameter sevice, default is spee_r=128 */

    // *******************
    // * Open serialport *
    // *******************
    try{ device.open(serialport.c_str(), serialport_bps); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("base_controller: Failed to open serialport %s!",serialport.c_str());
        ROS_BREAK();
    }
    ROS_INFO("base_controller: Opened Serialport at %s with %i bps.",serialport.c_str(),serialport_bps);

    // *********************************
    // * Create instance myMD49 of class MD49 and set initial settings for MD49 within constructor of class *
    // *********************************
    MD49 myMD49(initial_speed_l_from_parameter_service,initial_speed_r_from_parameter_service,initial_mode_from_parameter_service,initial_acceleration_from_parameter_service,initial_timeout_from_parameter_service,initial_regulator_from_parameter_service);                                                                                        /**< Generate instance myMD49 of class MD49 */

    // *********************************
    // * Set initial settings for MD49 *
    // *********************************
    //myMD49.init(myMD49.get_initial_speed_l(),myMD49.get_initial_speed_r(),myMD49.get_initial_mode(),myMD49.get_initial_acceleration(),myMD49.get_initial_timeout(),myMD49.get_initial_regulator());

    // ************
    // * Mainloop *
    // ************
    while(mySubscribeAndPublish.n.ok())
    {
        // *****************************************************************************************************
        // * set speed on MD49 via UART as set through /cmd_vel if speed_l or speed_r changed since last cycle *
        // *****************************************************************************************************
        if ((mySubscribeAndPublish.get_requested_speed_l() != mySubscribeAndPublish.get_actual_speed_l()) || (mySubscribeAndPublish.get_requested_speed_r() != mySubscribeAndPublish.get_actual_speed_r()))
        {
            myMD49.set_speed(mySubscribeAndPublish.get_requested_speed_l(),mySubscribeAndPublish.get_requested_speed_r());
            mySubscribeAndPublish.set_actual_speed_l(mySubscribeAndPublish.get_requested_speed_l());
            mySubscribeAndPublish.set_actual_speed_r(mySubscribeAndPublish.get_requested_speed_r());
        }
        // ****************************************************************************************************
        // * Read encoder- data from MD49 via UART and publish encoder values as read to topic /md49_encoders *
        // ****************************************************************************************************
        myMD49.get_encoders();
        mySubscribeAndPublish.md49_encoders_pub.publish(myMD49.md49_encoders);
        // *****************************************************************************************
        // * Read other- data from MD49 via UART and publish MD49 data as read to topic /md49_data *
        // *****************************************************************************************
        myMD49.md49_data.speed_l=myMD49.get_speed_l();
        myMD49.md49_data.speed_r=myMD49.get_speed_r();
        myMD49.md49_data.volts=myMD49.get_volts();
        myMD49.md49_data.current_l=myMD49.get_current_l();
        myMD49.md49_data.current_r=myMD49.get_current_r();
        myMD49.md49_data.acceleration=myMD49.get_acceleration();
        myMD49.md49_data.mode=myMD49.get_mode();
        myMD49.md49_data.error=myMD49.get_error();
        mySubscribeAndPublish.md49_data_pub.publish(myMD49.md49_data);
        // ********
        // * Loop *
        // ********
        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
    return 1;
} // end.main


