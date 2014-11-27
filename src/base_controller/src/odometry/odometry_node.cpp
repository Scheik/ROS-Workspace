#include "ros/ros.h"
#include <base_controller/encoders.h>               /* Custom message /encoders */
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//int16_t EncoderL;
//int16_t EncoderR;
int16_t previous_EncoderL;
int16_t previous_EncoderR;
double deltaLeft;
double deltaRight;
double meter_per_tick = 0.0008;
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
//ros::Time current_time_encoder, last_time_encoder;

void encoderdata_callback(const base_controller::encoders& encoders){
    //EncoderL=encoders.encoder_l;
    //EncoderR=encoders.encoder_r;
    //current_time_encoder = ros::Time::now();
    // calculate odomety
    // *****************
    deltaLeft = encoders.encoder_l - previous_EncoderL;
    deltaRight = encoders.encoder_r - previous_EncoderR;

    vx = deltaLeft * meter_per_tick;
    vy = deltaRight * meter_per_tick;

    previous_EncoderL = encoders.encoder_l;
    previous_EncoderR = encoders.encoder_r;
    //last_time_encoder = current_time_encoder;
}

int main( int argc, char* argv[] ){
    ros::init(argc, argv, "odometry" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/encoders", 100, encoderdata_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    ros::Rate loop_rate(10);
    while(n.ok()){

        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //distance_traveled=(vx+vy)/2;
        //v_linear = distance_traveled/dt;       // linear velocity
        //th = ( vy - vx ) / base_width;
        //v_angular = th/dt;                      //angular velocity



        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
