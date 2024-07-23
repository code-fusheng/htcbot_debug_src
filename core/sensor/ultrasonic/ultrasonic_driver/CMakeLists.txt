 cmake_minimum_required(VERSION 2.8.3)
project(ultrasonic_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_filters
  dynamic_reconfigure
  htcbot_common
  htcbot_msgs
  message_generation
  message_runtime
)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/dyp_ultrasonic.cfg
)

# add_message_files()
add_message_files(
  FILES
  UltraSonicDetect.msg
)

# add_service_files()
# add_action_files()

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)

if("${ROS_DISTRO}" STREQUAL "melodic")
  message(STATUS "Detected ROS Distro: ${ROS_DISTRO}")
endif()