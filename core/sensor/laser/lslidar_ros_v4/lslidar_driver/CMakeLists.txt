cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver_v4)

if(NOT DEFINED ENV{LS_V} OR "$ENV{LS_V}" STREQUAL "4")
message(STATUS "Compiling lslidar_driver_v4")
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
set(libpcap_LIBRARIES -lpcap)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    pcl_ros
    roscpp
    sensor_msgs
    tf
    dynamic_reconfigure
    lslidar_msgs_v4
    nodelet
)


find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
        pcl_conversions
        rospy
		roscpp
		pluginlib
		lslidar_msgs_v4
	    std_msgs
        genmsg
        message_generation
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(
		include
		${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)


catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
		message_runtime std_msgs
		roscpp sensor_msgs pluginlib nodelet
		pcl_ros pcl_conversions
		lslidar_msgs_v4
		DEPENDS
		Boost
    )
add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

add_library(lslidar_input_v4 src/input.cc)
target_link_libraries(lslidar_input_v4
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES})

add_library(lslidar_driver src/lslidar_driver.cpp)

target_link_libraries(lslidar_driver
		lslidar_input_v4
		${catkin_LIBRARIES})

# build the nodelet version
add_library(lslidar_driver_nodelet src/lslidar_driver_nodelet.cc src/lslidar_driver.cpp)
target_link_libraries(lslidar_driver_nodelet
		lslidar_input_v4
		${catkin_LIBRARIES}
		)

add_executable(lslidar_driver_node src/lslidar_driver_node.cpp)

if(catkin_EXPORTED_TARGETS)
	add_dependencies(lslidar_input_v4 ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(lslidar_driver_node
		lslidar_driver
		lslidar_input_v4
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
		)


install(TARGETS lslidar_input_v4 lslidar_driver lslidar_driver_nodelet lslidar_driver_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)


install(DIRECTORY launch rviz
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)
install(FILES
		nodelet_lslidar.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

else()
message(STATUS "Skipping lslidar_driver_v4")
endif()

