cmake_minimum_required(VERSION 2.8.3)
project(htcbot_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  htcbot_msgs
  message_filters
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS std_msgs  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


