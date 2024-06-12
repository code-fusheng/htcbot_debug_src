cmake_minimum_required(VERSION 3.0.2)
project(turn_htcbot)

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
)

# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/auto_start.cfg
)

catkin_package(
  # CATKIN_DEPENDS std_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)