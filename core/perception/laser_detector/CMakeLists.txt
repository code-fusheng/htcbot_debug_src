cmake_minimum_required(VERSION 2.8.3)
project(laser_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  tf
  htcbot_msgs
  message_filters
  dynamic_reconfigure
  htcbot_common
  htcbot_msgs
)

find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/laser_detector.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs htcbot_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

add_executable(laser_detector_node
  src/laser_detector.cpp src/laser_detector_node.cpp
)
add_dependencies(laser_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(laser_detector_node
  ${htcbot_common_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)



