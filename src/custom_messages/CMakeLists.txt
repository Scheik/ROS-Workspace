cmake_minimum_required(VERSION 2.4.6)
project(custom_messages)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

#generate messages in the msg folder
add_message_files(
  FILES 
  md49_encoders.msg
  md49_data.msg
)

#generate added messages with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
