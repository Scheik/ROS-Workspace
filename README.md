ROS-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

Package "base_controller" with node: 
 "base_controller"
 
Package "joystick_driver" with node:
 "joystick_driver"
 
 Package "base_odometry" with node:
  "base_odometry"
 
Package "sqlite_connector" with node: 
 "sqlite_connector"

--------------------------

This Robot Hardware is build around an embedded Linux PC (pcDuino). 

Robots drives are two EMG-49 geared Motors with wheelencoders, connected to MD49 Driverboard (both from Devantech) with serial connection to pcDuino.

ROS Distribution on pcDuino and Ubuntu-Workstation is Indigo.

robotOS and workstationOS is Ubuntu 14

INSTALL INTO CATKIN_WORKSPACE:
---------------------------

$ cd ~

$ git clone https://github.com/scheik/ROS-Workspace.git

$ cd ~/ROS-Workspace

$ catkin_make

---------------------------
The complete documentation of the project can be found on

www.the-starbearer.de
