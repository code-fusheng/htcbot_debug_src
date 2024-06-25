 cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

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
  can_msgs
)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
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

add_executable(pure_pursuit_node 
               src/pure_pursuit_node.cpp
               src/pure_pursuit.cpp 
               src/pure_pursuit_core.cpp
               src/utils.cpp
               src/pure_pursuit_viz.cpp 
               )

target_link_libraries(pure_pursuit_node
                     ${catkin_LIBRARIES})
target_include_directories(pure_pursuit_node PRIVATE
                           ${catkin_INCLUDE_DIRS})
add_dependencies(pure_pursuit_node ${catkin_EXPORTED_TARGETS})