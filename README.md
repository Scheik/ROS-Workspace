ROS-Groovy-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with nodes: "base_controller", "md49_console"
Package "robot_teleop" with nodes: "joystick_driver"

The Robot Hardware is build around Raspberry Pi 
and AVR-Controllers. 
EMG-49 Drives are driven with MD49.
ROS Distribution on RPi is Groovy.

INSTALL INTO CATKIN_WORKSPACE:
==

$ cd ~

$ git clone https://github.com/scheik/ROS-Groovy-Workspace.git

$ cd ~/ROS-Groovy-Workspace

$ catkin_make
