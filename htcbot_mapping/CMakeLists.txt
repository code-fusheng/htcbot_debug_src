cmake_minimum_required(VERSION 3.0.2)
project(htcbot_mapping)

#find_package(CUDA)
find_package(PCL REQUIRED)
IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  pcl_ros
  sensor_msgs
  pcl_conversions
  htcbot_msgs
  ndt_cpu
  pcl_omp_registration
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ndt_mapping_pro.cfg
)

catkin_package(
  CATKIN_DEPENDS std_msgs  
  DEPENDS PCL
  ndt_cpu
  pcl_omp_registration
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ndt_mapping_pro_node nodes/ndt_mapping_pro/ndt_mapping_pro_node.cpp nodes/ndt_mapping_pro/ndt_mapping_pro.cpp)
target_link_libraries(ndt_mapping_pro_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ndt_mapping_pro_node ${catkin_EXPORTED_TARGETS})


