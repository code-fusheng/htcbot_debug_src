cmake_minimum_required(VERSION 2.8.3)
project(system_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  dynamic_reconfigure
  htcbot_common
  htcbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/status_control.cfg
)

catkin_package(
  # CATKIN_DEPENDS std_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)