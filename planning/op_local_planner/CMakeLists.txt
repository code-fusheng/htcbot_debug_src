 cmake_minimum_required(VERSION 2.8.3)
project(op_local_planner)

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
  pcl_conversions
  pcl_ros
  can_msgs
)

# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/op_local_planner.cfg
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

add_executable(op_common_params_node nodes/op_common_params/op_common_params.cpp )
target_link_libraries(op_common_params_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(op_trajectory_generator_node
               nodes/op_trajectory_generator/op_trajectory_generator_node.cpp
               nodes/op_trajectory_generator/op_trajectory_generator.cpp
               src/op_utils.cpp)
target_link_libraries(op_trajectory_generator_node 
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(op_trajectory_evaluator_node 
               nodes/op_trajectory_evaluator/op_trajectory_evaluator_node.cpp
               nodes/op_trajectory_evaluator/op_trajectory_evaluator.cpp
               src/op_utils.cpp)
target_link_libraries(op_trajectory_evaluator_node 
  ${htcbot_common_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(
    op_common_params_node 
    op_trajectory_generator_node
    op_trajectory_evaluator_node
    ${catkin_EXPORTED_TARGETS}
)