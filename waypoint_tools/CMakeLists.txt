cmake_minimum_required(VERSION 2.8.3)
project(waypoint_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  htcbot_msgs
  message_filters
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/waypoint_saver.cfg
  cfg/waypoint_loader.cfg
)

catkin_package(
  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_saver_node
  nodes/waypoint_saver/waypoint_saver.cpp nodes/waypoint_saver/waypoint_saver_node.cpp
)
add_dependencies(waypoint_saver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(waypoint_saver_node
  ${catkin_LIBRARIES}
)

