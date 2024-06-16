cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  htcbot_msgs
  htcbot_common
  dynamic_reconfigure
)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/map_loader.cfg
)

catkin_package(
  CATKIN_DEPENDS std_msgs  
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${htcbot_common_INCLUDE_DIRS}
)

add_executable(map_loader_node
  nodes/map_loader/map_loader.cpp nodes/map_loader/map_loader_node.cpp
)
add_dependencies(map_loader_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(map_loader_node
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcd_map_grid_node
  nodes/pcd_map_grid/pcd_map_grid.cpp nodes/pcd_map_grid/pcd_map_grid_node.cpp
)
add_dependencies(pcd_map_grid_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pcd_map_grid_node
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
