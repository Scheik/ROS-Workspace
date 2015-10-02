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

#include <ros/ros.h>                                                                            /**<  ROS */
#include <geometry_msgs/Twist.h>                                                                /**<  ROS Twist message */
#include <serialport/serialport.h>                                                              /**<  library for serial communications via UART*/
#include <custom_messages/md49_data.h>                                                          /**<  custom message /md49_data */
#include <custom_messages/md49_encoders.h>                                                      /**<  custom message /md49_encoders */
#include "base_controller/base_controller_class.h"

#define TIMEOUT 1000                                                                            /**<  timeout for reading serialport in ms */                                                                               /**<  max buffersize serial input */

class BaseController
{
public:

    ros::NodeHandle n;

    /**
     * @brief BaseController
     */
    BaseController()
      {
        //Topics to publish
        md49_encoders_pub = n.advertise<custom_messages::md49_encoders>("md49_encoders",10);
        md49_data_pub = n.advertise<custom_messages::md49_data>("md49_data",10);
        //Topic to subscribe
        sub_cmd_vel = n.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, this);
        // Read initial parameters from parameter service
        n.param<std::string>("serialport/name", serialport, "/dev/ttyS0");                      // Get serialportname from ROS Parameter sevice, default is ttyS0 (pcDuinos GPIO UART)
        n.param("serialport/bps", serialport_bps, 38400);                                       // Get serialport bps from ROS Parameter sevice, default is 38400Bps
        n.param("md49/mode", initial_md49_mode, 0);                                             // Get MD49 Mode from ROS Parameter sevice, default is Mode=0
        n.param("md49/acceleration", initial_md49_acceleration, 5);                             // Get MD49 Acceleration from ROS Parameter sevice, default is Acceleration=0
        n.param("md49/regulator", initial_md49_regulator, true);                                // Get MD49 Regulator from ROS Parameter sevice, default is Regulator=ON
        n.param("md49/timeout", initial_md49_timeout, true);                                    // Get MD49 Timeout from ROS Parameter sevice, default is Timeout=ON
        n.param("md49/speed_l", requested_speed_l, 128);                                        // Get MD49 speed_l from ROS Parameter sevice, default is spee_l=128
        n.param("md49/speed_r",  requested_speed_r, 128);                                       // Get MD49 speed_r from ROS Parameter sevice, default is spee_r=128
        actual_speed_l=requested_speed_l;
        actual_speed_r=requested_speed_r;
      }

    /**
     * @brief cmd_vel_callback
     * @param vel_cmd
     */
    void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){
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
    }

    void open_serialport()
    {
        try{ device.open(serialport.c_str(), serialport_bps); }
        catch(cereal::Exception& e)
        {
            ROS_FATAL("base_controller: Failed to open serialport %s!",serialport.c_str());
            ROS_BREAK();
        }
        ROS_INFO("base_controller: Opened Serialport at %s with %i bps.",serialport.c_str(),serialport_bps);
    }
    /**
     * @brief read_encoders
     */
    void publish_encoders()
    {
        get_encoders();
        md49_encoders_pub.publish(md49_encoders);
    }

    void publish_md49_data()
    {
        md49_data.speed_l=get_speed_l();
        md49_data.speed_r=get_speed_r();
        md49_data.volts=get_volts();
        md49_data.current_l=get_current_l();
        md49_data.current_r=get_current_r();
        md49_data.acceleration=get_acceleration();
        md49_data.mode=get_mode();
        md49_data.error=get_error();
        md49_data_pub.publish(md49_data);
    }

    /**
     * @brief init_md49
     * @param speed_l
     * @param speed_r
     * @param mode
     * @param acceleration
     * @param timeout
     * @param regulator
     */
    void init_md49(int speed_l, int speed_r, int mode, int acceleration, bool timeout, bool regulator)
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
    }
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

    }
    /**
     * @brief set_mode
     * @param mode
     */
    void set_mode(int mode)
    {
        const char md49_set_mode[]={0x00,0x34,mode};
        device.write(md49_set_mode,3);
        ROS_INFO("base_controller: Set mode=%i on MD49", mode);
    }
    /**
     * @brief set_acceleration
     * @param acceleration
     */
    void set_acceleration(int acceleration)
    {
        const char md49_set_acceleration[]={0x00,0x33,acceleration};
        device.write(md49_set_acceleration,3);
        ROS_INFO("base_controller: Set acceleration=%i on MD49", acceleration);
    }
    /**
     * @brief enable_timeout
     */
    void enable_timeout(void){
        const char md49_enable_timeout[] = {0x00,0x39};             // put together command to enable md49 timeout
        device.write(md49_enable_timeout,2);
        md49_data.timeout=1;
        ROS_INFO("base_controller: Enabled timeout on MD49");
    }
    /**
     * @brief enable_regulator
     */
    void enable_regulator(void){
        const char md49_enable_regulator[] = {0x00,0x37};           // put together command to enable md49 regulator
        device.write(md49_enable_regulator,2);
        md49_data.regulator=1;
        ROS_INFO("base_controller: Enabled regulator on MD49");
    }
    /**
     * @brief disable_timeout
     */
    void disable_timeout(void){
        const char md49_disable_timeout[] = {0x00,0x38};            // put together command to enable md49 regulator
        device.write(md49_disable_timeout,2);
        md49_data.timeout=0;
        ROS_INFO("base_controller: Disabled timeout on MD49");
    }
    /**
     * @brief disable_regulator
     */
    void disable_regulator(void){
        const char md49_disable_regulator[] = {0x00,0x36};          // put together command to enable md49 timeout
        device.write(md49_disable_regulator,2);
        md49_data.regulator=0;
        ROS_INFO("base_controller: Disabled regulator on MD49");
    }
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
    }
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
    }
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
    }
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
    }
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
    }
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
    }
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
    }
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
    }
    /**
     * @brief get_error
     * @return
     */
    int get_error(){
        // ************************
        const char md49_get_error[] = {0x00,0x2D};                  // put together command to read md49 error
        device.write(md49_get_error,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, error
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 errorbyte!");
        }
        //ROS_INFO("Got this reply (error): %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief reset_encoders
     */
    void reset_encoders(void){
        const char md49_reset_encoders[] = {0x00,0x35};             // put together command to reset md49 encoders
        device.write(md49_reset_encoders,2);
        ROS_INFO("base_controller: Reset encoders on MD49");
    }

    /**
     * @brief get_requested_speed_l
     * @return
     */
    int get_requested_speed_l()
    {
        return requested_speed_l;
    }
    /**
     * @brief get_requested_speed_r
     * @return
     */
    int get_requested_speed_r()
    {
        return requested_speed_r;
    }
    /**
     * @brief set_requested_speed_l
     * @param speed_l
     */
    void set_requested_speed_l(int speed_l)
    {
        requested_speed_l=speed_l;
    }
    /**
     * @brief set_requested_speed_r
     * @param speed_r
     */
    void set_requested_speed_r(int speed_r)
    {
        requested_speed_r=speed_r;
    }
    /**
     * @brief get_actual_speed_l
     * @return
     */
    int get_actual_speed_l()
    {
        return actual_speed_l;
    }
    /**
     * @brief get_actual_speed_r
     * @return
     */
    int get_actual_speed_r()
    {
        return actual_speed_r;
    }
    /**
     * @brief set_actual_speed_l
     * @param speed_l
     */
    void set_actual_speed_l(int speed_l)
    {
        actual_speed_l=speed_l;
    }
    /**
     * @brief set_actual_speed_r
     * @param speed_r
     */
    void set_actual_speed_r(int speed_r)
    {
        actual_speed_r=speed_r;
    }
    /**
     * @brief get_initial_md49_mode
     * @return
     */
    int get_initial_md49_mode()
    {
        return initial_md49_mode;
    }/**
     * @brief get_initial_md49_acceleration
     * @return
     */
    int get_initial_md49_acceleration()
    {
        return initial_md49_acceleration;
    }
    /**
     * @brief get_initial_md49_timeout
     * @return
     */
    int get_initial_md49_timeout()
    {
        return initial_md49_timeout;
    }
    /**
     * @brief get_initial_md49_regulator
     * @return
     */
    int get_initial_md49_regulator()
    {
        return initial_md49_regulator;
    }

private:

    cereal::CerealPort device;                                                                      /**<  serialport */
    char reply[8];
    int requested_speed_l, requested_speed_r;                                                       /**<  requested speed_l and speed_r for MD49 */
    int actual_speed_l, actual_speed_r;                                                             /**<  buffers actual set speed_l and speed_r */
    int initial_md49_mode;                                                                          /**<  MD49 Mode, is read from parameters server */
    int initial_md49_acceleration;                                                                  /**<  MD49 Acceleration,  is read from parameters server */
    bool initial_md49_timeout;                                                                      /**<  MD49 Timeout-Mode, is read from parameters server */
    bool initial_md49_regulator;                                                                    /**<  MD40 Regulator-Mode , is read from parameters server */
    std::string serialport;                                                                         /**<  used serialport on pcDuino, is read from parameters server */
    int serialport_bps;                                                                             /**<  used baudrate, is read from parameters server */
    //ros::NodeHandle n;
    ros::Subscriber sub_cmd_vel;
    custom_messages::md49_data md49_data;                                                           /**<  topic /md49_data */
    custom_messages::md49_encoders md49_encoders;                                                   /**<  topic /md49_encoders */
    ros::Publisher md49_encoders_pub;
    ros::Publisher md49_data_pub;
}; //End of class BaseController

int main( int argc, char* argv[] ){

    // *****************
    // * Init ROS node *
    // *****************
    ros::init(argc, argv, "base_controller" );                                                      /**< Create an object of class SubscribeAndPublish */
    BaseController myBaseController;
    ros::Rate loop_rate(10);
    ROS_INFO("base_controller: base_controller running...");

    // *******************
    // * Open serialport *
    // *******************
    myBaseController.open_serialport();

    // *****************************
    // * Set initial MD49 settings *
    // *****************************
    myBaseController.init_md49(myBaseController.get_requested_speed_l(), myBaseController.get_requested_speed_r(), myBaseController.get_initial_md49_mode(), \
                               myBaseController.get_initial_md49_acceleration(), myBaseController.get_initial_md49_timeout(), myBaseController.get_initial_md49_regulator());

    // ************
    // * Mainloop *
    // ************
    while(myBaseController.n.ok())
    {
        // set speed on MD49 via UART as set through /cmd_vel if speed_l or speed_r changed since last cycle
        if ((myBaseController.get_requested_speed_l() != myBaseController.get_actual_speed_l()) || (myBaseController.get_requested_speed_r() != myBaseController.get_actual_speed_r()))
        {
            myBaseController.set_speed(myBaseController.get_requested_speed_l(),myBaseController.get_requested_speed_r());
            myBaseController.set_actual_speed_l(myBaseController.get_requested_speed_l());
            myBaseController.set_actual_speed_r(myBaseController.get_requested_speed_r());
            ROS_INFO("base_controller: Set speed_l=%i and speed_r=%i on MD49", myBaseController.get_requested_speed_l(), myBaseController.get_requested_speed_r());
        }
        // Read encoder- data from MD49 via UART and publish encoder values as read to topic /md49_encoders
        myBaseController.publish_encoders();
        // Read other- data from MD49 via UART and publish MD49 data as read to topic /md49_data
        myBaseController.publish_md49_data();
        // Loop
        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
    return 1;
} // end.main

