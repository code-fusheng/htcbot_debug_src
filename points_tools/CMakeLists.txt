cmake_minimum_required(VERSION 3.0.2)
project(points_tools)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
  velodyne_pointcloud
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/plane_ground_filter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf tf2 tf2_eigen tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

if("${ROS_DISTRO}" STREQUAL "melodic")
    
endif()

add_executable(ring_ground_filter
            nodes/ring_ground_filter/ring_ground_filter.cpp
            )
target_include_directories(ring_ground_filter PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            )
target_link_libraries(ring_ground_filter
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${Qt5Core_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )
add_dependencies(ring_ground_filter ${catkin_EXPORTED_TARGETS})

add_executable(plane_ground_filter_node
  nodes/plane_ground_filter/plane_ground_filter.cpp nodes/plane_ground_filter/plane_ground_filter_node.cpp
)
add_dependencies(plane_ground_filter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(plane_ground_filter_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)