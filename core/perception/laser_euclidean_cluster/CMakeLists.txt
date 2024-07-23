 cmake_minimum_required(VERSION 2.8.3)
project(laser_euclidean_cluster)

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
  jsk_recognition_msgs
  cv_bridge
  pcl_ros
  autoware_msgs
)

find_package(PCL 1.7 REQUIRED)

find_package(OpenMP)
find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/laser_euclidean_cluster.cfg
)

# add_message_files()
# add_service_files()
# add_action_files()
# generate_message()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs  pcl_ros htcbot_msgs jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)

if("${ROS_DISTRO}" STREQUAL "melodic")
  message(STATUS "Detected ROS Distro: ${ROS_DISTRO}")
endif()

add_executable(${PROJECT_NAME}_node
  src/laser_euclidean_cluster.cpp 
  src/laser_euclidean_cluster_node.cpp
  src/cluster.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${htcbot_common_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${catkin_LIBRARIES}
)