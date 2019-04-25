cmake_minimum_required(VERSION 2.8.3)
project(SmartCarControl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
        FILES
        front.msg
)

add_service_files(
        FILES
        distance.srv
        switchmode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES SmartCarControl
   CATKIN_DEPENDS roscpp rosserial_arduino rosserial_client std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ROS_LIB_DIR}
)

rosserial_generate_ros_lib(
        PACKAGE rosserial_arduino
        SCRIPT make_libraries.py
)

rosserial_configure_client(
        DIRECTORY firmware
        TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware firmware ALL)
rosserial_add_client_target(firmware firmware-upload)


add_executable(smartcar_monitor src/SmartCarMonitor.cpp)
add_dependencies(smartcar_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smartcar_monitor ${catkin_LIBRARIES})
