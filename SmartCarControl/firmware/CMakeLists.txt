
cmake_minimum_required(VERSION 2.8.3)

include_directories(../include ${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON -DDAY)

generate_arduino_firmware(firmware
        SRCS ../src/SmartCar.cpp chatter.cpp ${ROS_LIB_DIR}/time.cpp
        BOARD uno
        PORT /dev/ttyACM0
        )

