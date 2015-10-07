cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(arduino_example_chatter
  SRCS arduino_example_chatter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nano328
  PORT /dev/ttyUSB0
)

generate_arduino_firmware(arduino_example_subscriber
  SRCS arduino_example_subscriber.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nano328
  PORT /dev/ttyUSB0
)