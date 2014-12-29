ROS-Groovy-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with nodes: 
"base_controller", "joystick_driver", "odometry", "sqlite_connector"


This Robot Hardware is build around a Raspberry Pi 
and some AVR ATMegas. 

Robots drives are two EMG-49 geared Motors with wheelencoders, connected to MD49 Driverboard (both from Devantech) with serial connection to RPi.

ROS Distribution on RPi and Ubuntu-Workstation is Groovy.

INSTALL INTO CATKIN_WORKSPACE:
---------------------------

$ cd ~

$ git clone https://github.com/scheik/ROS-Groovy-Workspace.git

$ cd ~/ROS-Groovy-Workspace

$ catkin_make
