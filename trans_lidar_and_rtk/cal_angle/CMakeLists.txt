cmake_minimum_required(VERSION 3.0.2)
project(cal_angle)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)
find_package(PCL 1.8 REQUIRED COMPONENTS
  common
  io
  visualization  
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)



add_executable(cal_angle src/cal_angle.cpp)


add_dependencies(cal_angle ${catkin_EXPORTED_TARGETS})

target_link_libraries(cal_angle
  ${catkin_LIBRARIES}  # 链接catkin提供的库
  ${PCL_LIBRARIES}     # 链接PCL库
)

# 包含PCL头文件
target_include_directories(cal_angle PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


