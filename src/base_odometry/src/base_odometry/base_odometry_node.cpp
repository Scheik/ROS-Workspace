#include "ros/ros.h"
#include <custom_messages/md49_encoders.h>
//#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>


//int16_t EncoderL;
//int16_t EncoderR;
/*
int32_t previous_EncoderL;
int32_t previous_EncoderR;
double deltaLeft;
double deltaRight;
double ticks_meter =4800;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double v_linear=0.0;
double v_angular=0.0;
double distance_traveled=0.0;
double base_width=0.4;
double vx;
double vy;
double vth;
ros::Time current_time_encoder, last_time_encoder;
*/


int32_t ticks_per_meter;
double base_width;
int32_t encoder_min;
int32_t encoder_max;
int32_t encoder_low_wrap= (encoder_max - encoder_min) * 0.3 +encoder_min;
int32_t encoder_high_wrap= (encoder_max - encoder_min) * 0.7 +encoder_min;
ros::Time current_time, last_time;

// wheel encoder readings
int32_t prev_lencoder=0;
int32_t prev_rencoder=0;
int32_t encR;
int32_t encL;
double delta_left;
double delta_right;
// position in xy plane
double self_x=0;
double self_y=0;
double self_th=0;
// speeds in x/rotation
double dx=0;
double dr=0;



void encoderdata_callback(const custom_messages::md49_encoders& md49_encoders){

    delta_left = md49_encoders.encoder_l - prev_lencoder ;
    prev_lencoder=md49_encoders.encoder_l;

    delta_right = md49_encoders.encoder_r-prev_rencoder;
    prev_rencoder=md49_encoders.encoder_r;
}

int main( int argc, char* argv[] ){

    ros::init(argc, argv, "base_odometry" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/md49_encoders", 100, encoderdata_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    n.param("base_odometry_parameters/ticks_per_meter", ticks_per_meter,4900);
    n.param("base_odometry_parameters/encoder_min", encoder_min,-2147483647);
    n.param("base_odometry_parameters/encoder_max", encoder_max,2147483647);
    n.param("base_odometry_parameters/base_width", base_width,0.5);

    ROS_INFO("Node base_odometry started");

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);
    while(n.ok()){

        ros::spinOnce();
        double x, y, th, d, d_left, d_right;
        current_time = ros::Time::now();
        double elapsed = (current_time - last_time).toSec();

        // ********************
        // * compute odometry *
        // ********************
        d_left = delta_left/ticks_per_meter;
        d_right=delta_right/ticks_per_meter;
        //distance traveled as average of both wheels
        d = (d_left + d_right)/2;
        th = (d_right - d_left)/ base_width;
        //calculate velocities
        dx = d / elapsed;
        dr = th / elapsed;
        //calculate distance traveled and final position
        if (d != 0)
        {
            //calculate distance traveled
            x = cos(th) * d;
            y = -sin(th) * d;
            //calculate final position
            self_x = self_x + (cos(self_th) * x - sin(self_th) * y);
            self_y = self_y + (sin(self_th) * x + cos(self_th) * y);
        }
        if (th != 0)
        {
            self_th = self_th + th;
        }

        //****************************
        //* publish odom information *
        //****************************
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(self_th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = self_x;
        odom_trans.transform.translation.y = self_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = self_x;
        odom.pose.pose.position.y = self_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = dr;
        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        loop_rate.sleep();
    }// end.mainloop
}// end.main
