cmake_minimum_required(VERSION 2.8.3)

project(htcbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  message_runtime
  jsk_recognition_msgs
)

# PS: 创建自定义消息时,如果存在自身嵌套引用,消息路径必须在msg下
add_message_files(
        FILES
        # switch
        GnssModeSwitch.msg
        # conf
        MapPathConf.msg
        ModeSwitch.msg
        GridMapBuild.msg
        MappingConf.msg
        # obj
        DetectedObject.msg
        DetectedObjectArray.msg
        Lane.msg
        LaneArray.msg
        Waypoint.msg
        LaserDetect.msg
        # autoware
        CloudCluster.msg
        CloudClusterArray.msg
        # new conf
        ConfObstacleDetection.msg
        ConfPurePursuit.msg
        ConfEuclideanCluster.msg
        ConfOpLocalPlanner.msg
        ConfVehicle.msg
        ConfNdtMapping.msg
        ConfNdtLocalizer.msg
        ControlCommand.msg
        ControlCommandStamped.msg
        SimpleObstacleDist.msg
        UserCmd.msg
        VideoStreamControl.msg
        # new status
        StatusHtcbotModule.msg
        StatusHtcbotSensor.msg
        StatusHtcbotModuleArray.msg
        StatusHtcbotSensorArray.msg
        StatusHtcbotSystem.msg
        # new Conf
        conf/ConfWaypointSaver.msg
)

add_service_files(
        FILES
        SwitchStatusSrv.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)



