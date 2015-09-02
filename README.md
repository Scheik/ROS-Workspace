ROS-Groovy-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with nodes: 
"base_controller", "joystick_driver", "odometry", "sqlite_connector"


This Robot Hardware is build around a pcDuino . 

Robots drives are two EMG-49 geared Motors with wheelencoders, connected to MD49 Driverboard (both from Devantech) with serial connection to pcDuino.

ROS Distribution on pcDuino and Ubuntu-Workstation is Indigo.

INSTALL INTO CATKIN_WORKSPACE:
---------------------------

$ cd ~

$ git clone https://github.com/scheik/ROS-Workspace.git

$ cd ~/ROS-Workspace

$ catkin_make
