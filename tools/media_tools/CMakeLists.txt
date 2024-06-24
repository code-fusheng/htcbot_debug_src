 cmake_minimum_required(VERSION 2.8.3)
project(media_tools)

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
  sound_play
)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/camera_play.cfg
  cfg/voice_play.cfg
)

# add_message_files()
# add_service_files()
# add_action_files()
# generate_message()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)

if("${ROS_DISTRO}" STREQUAL "melodic")
  message(STATUS "Detected ROS Distro: ${ROS_DISTRO}")
endif()