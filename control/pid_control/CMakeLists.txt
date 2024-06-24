 cmake_minimum_required(VERSION 2.8.3)
project(pid_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_filters
  dynamic_reconfigure
)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/pid_control.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_control_node
  src/pid_control.cpp src/pid_control_node.cpp
)
add_dependencies(pid_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pid_control_node
  ${catkin_LIBRARIES}
)