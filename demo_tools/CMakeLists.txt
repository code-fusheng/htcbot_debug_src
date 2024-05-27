 cmake_minimum_required(VERSION 2.8.3)
project(demo_tools)

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

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/cpp_demo.cfg
  cfg/py_demo.cfg
  cfg/cpp_base.cfg
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

add_executable(cpp_demo_node
  nodes/cpp_demo/cpp_demo.cpp nodes/cpp_demo/cpp_demo_node.cpp
)
add_dependencies(cpp_demo_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cpp_demo_node
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(cpp_base_node
  nodes/cpp_base/cpp_base.cpp nodes/cpp_base/cpp_base_node.cpp
)
add_dependencies(cpp_base_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cpp_base_node
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
)