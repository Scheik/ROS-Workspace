ROS-Workspace
========================
Includes the following ROS Packages for my mobile Robot:

 Package "base_odometry" with node: 
 "base_odometry"
 
 Package "base_controller" with node: 
 "base_controller"
 
 Package "robot_teleop" with nodes:
 "joystick_teleop", 
 "keyboard_teleop"

 Package "robot_urdf" for visualization
 of robot in rviz
 
 Package "robot_2dnav" for implementation
 of the navigation stack

 Package "robot_maps" with maps
 to use with the navigation stack
 
 Package "sqlite_connector" with node: 
 "sqlite_connector"
 
 Package "arduino_examples" to demonstrate
 usage of Arduinos as ROS- Nodes
 
 Package "md49_messages" implementing 
 custom ROS messages
 
 Package "serialport" as C++ function
 library for serialports

--------------------------

![alt tag](http://url/to/img.png)

This Robot Hardware is build around an embedded Linux PC (pcDuino). 

Robots drives are two EMG-49 geared Motors with wheelencoders, connected to MD49 Driverboard (both from Devantech) with serial connection to pcDuino.

ROS Distribution on pcDuino and Ubuntu-Workstation is Indigo.

robots OS and workstations OS is both Ubuntu 14

INSTALL INTO CATKIN_WORKSPACE:
---------------------------

$ cd ~

$ git clone https://github.com/scheik/ROS-Workspace.git

$ cd ~/ROS-Workspace

$ rosdep install --from-paths . --ignore-src --rosdistro indigo

$ catkin_make

---------------------------
The complete documentation of the project can be found on

www.the-starbearer.de
