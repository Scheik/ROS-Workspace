cmake_minimum_required(VERSION 2.8.3)
project(arduino_examples)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware chatter ALL)
rosserial_add_client_target(firmware chatter-upload)

rosserial_add_client_target(firmware chatter2 ALL)
rosserial_add_client_target(firmware chatter2-upload)

