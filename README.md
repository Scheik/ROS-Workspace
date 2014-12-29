ROS-Groovy-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with nodes: 
"base_controller", "joystick_driver", "odometry"


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
