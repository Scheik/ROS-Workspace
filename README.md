ROS-Groovy-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with nodes: "base_controller", "serial_controller", "joystick_driver", "odometry"


This Robot Hardware is build around a Raspberry Pi 
and some AVR ATMegas. 

EMG-49 Drives are driven with MD49 (both from Devantech).
ROS Distribution on RPi and Ubuntu-Workstation is Groovy.

Install package base_controller on RPi and Ubuntu- Workstation.

INSTALL INTO CATKIN_WORKSPACE:
---------------------------

$ cd ~

$ git clone https://github.com/scheik/ROS-Groovy-Workspace.git

$ cd ~/ROS-Groovy-Workspace

$ catkin_make

On RPi
=============

cd ROS-Groovy-Workspace

source devel/setup.bash

Run node serial_controller: 
---------------------------

rosrun base_controller serial_controller

This node connects to AVR-Master via UART/RS232 and is working as an Interface to all settings & data of/from robots hardware.

Run node base_controller: 
-----------------------------

rosrun base_controller base_controller

Publishes /md49data (custom Message with all settings and parameters from robots drive

Publishes /encoders (custom Message with encodervalues from left and right drive)

Subscribes /cmd_vel (geometry_msgs/Twist)

On Ubuntu-Workstation
=====================

comming soon...
