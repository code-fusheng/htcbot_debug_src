#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: code-fusheng
Date: 2024-05-10 15:04:52
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-05 12:37:38
Description: 
'''

import rospy
import csv
import math
import os
import time
from enum import Enum

from dynamic_reconfigure.server import Server
from system_control.cfg import StatusControlConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse
from htcbot_msgs.msg import StatusHtcbotSensor, StatusHtcbotSensorArray
from htcbot_msgs.msg import StatusHtcbotModule, StatusHtcbotModuleArray
from htcbot_msgs.msg import StatusHtcbotSystem
from htcbot_msgs.msg import UserCmd
from htcbot_msgs.msg import SimpleObstacleDist
from htcbot_msgs.msg import LaserDetect
from can_msgs.msg import vehicle_status, battery
from std_msgs.msg import String

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix
from ultrasonic_driver.msg import UltraSonicDetect

class MODULE_TYPE(Enum):
    UNKNOWN = 0    
    MAP = 1
    LOCALIZER = 2
    SENSOR = 3
    WAYPOINT = 4
    PLANNER = 5
    CONTROL = 6
    GATEWAY = 7

class SENSOR_TYPE(Enum):
    UNKNOWN = 0
    GNSS = 1               # GNSS 惯导
    LASER = 2              # 激光雷达
    ULTRASONIC = 3         # 超声波雷达
    CAMERA = 4             # 摄像头
    DEPTH_CAMERA = 5       # 深度相机
    IMU = 6                # IMU
    VEHICLE = 7            # 底盘
    OTHER = 8              # 其他

class STATUS_TYPE(Enum):
    NONE = 0                # 默认
    READY = 1               # 就绪
    WARN = 2                # 警告
    DANGER = 3              # 危险
    EXC = 4                 # 异常
    EXPIRED = 5             # 过期

class SECURITY_LEVEL(Enum):
    NONE = 0                # 无
    NORMAL = 1              # 一般/正常
    STRICT = 2              # 严格

class StatusControlNode:

    def __init__(self):
        rospy.init_node("status_control_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False

        self.use_gnss = True
        self.use_laser = True
        self.use_ultrasonic = True
        self.use_camera = False
        self.use_imu = False
        self.use_vehicle = True

        # 车辆信息
        self.cur_speed = None   # m/s
        self.cur_steer = None   # angle

        # 状态
        self.laser_raw_status = None
        self.laser_obs_status = None

        # 雷达检测更新状态
        self.front_obs_dist = None
        self.front_obs_update_time = None
        self.back_obs_dist = None
        self.back_obs_update_time = None

        self.switch_status = 1
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        # self.status_htcbot_sensor = None
        self.sensor_status_array = StatusHtcbotSensorArray()
        self.module_status_array = StatusHtcbotModuleArray()

        self.security_level = str(SECURITY_LEVEL.NORMAL.value)

    def run(self):
        self.init()
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # rospy.loginfo("[status_control_node] ===> Running...")
            self.check_and_pub_status()
            rate.sleep()

    def init(self):

        # self.init_status_array()
        self.init_sensor_status()
        self.init_module_status()

        self.low_battery_warn = rospy.get_param("~low_battery_warn", default=30)
        self.low_battery_danger = rospy.get_param("~low_battery_danger", default=20)

        ultrosonic1_limit = rospy.get_param("~ultrosonic_detect_distance_1", default=350)
        ultrosonic2_limit = rospy.get_param("~ultrosonic_detect_distance_2", default=350)
        ultrosonic3_limit = rospy.get_param("~ultrosonic_detect_distance_3", default=350)
        ultrosonic4_limit = rospy.get_param("~ultrosonic_detect_distance_4", default=350)

        self.ultrasonic_distance_limit = [ultrosonic1_limit, ultrosonic2_limit, ultrosonic3_limit, ultrosonic4_limit]

        self.laser_stop_front = rospy.get_param("~laser_stop_front", default=0.5)
        self.laser_stop_back = rospy.get_param("~laser_stop_back", default=0)
        self.laser_stop_left = rospy.get_param("~laser_stop_left", default=0.45)
        self.laser_stop_right = rospy.get_param("~laser_stop_right", default=0.45)
        self.laser_error_points = rospy.get_param("~laser_error_points", default=20000)

        # self.sub_pointcloud = rospy.Subscriber("rslidar_points", PointCloud2, self.callback_pointcloud, queue_size=10)
        self.sub_ultrasonic = rospy.Subscriber("ultrasonic_detection", UltraSonicDetect, self.callback_ultrasonic, queue_size=5)
        self.sub_laser_detection = rospy.Subscriber("laser_detection", LaserDetect, self.callback_laser_detection, queue_size=10)
        self.sub_gnss = rospy.Subscriber("fix", NavSatFix, self.callback_gnss, queue_size=5)
        self.sub_user_cmd = rospy.Subscriber("user_cmd", UserCmd, self.callback_user_cmd, queue_size=5)
        self.sub_vehicle_status = rospy.Subscriber("vehicle_status", vehicle_status, self.callback_vehicle_status, queue_size=5)
        self.sub_battery_status = rospy.Subscriber("battery_status", battery, self.callback_battery_status, queue_size=5)

        self.sub_module_status = rospy.Subscriber("/htcbot/module_status", StatusHtcbotModule, self.callback_module_status, queue_size=10)
        self.sub_sensor_status = rospy.Subscriber("/htcbot/sensor_status", StatusHtcbotSensor, self.callback_sensor_status, queue_size=10)

        self.pub_sensor_status = rospy.Publisher("/htcbot/sensor_status", StatusHtcbotSensor, queue_size=10)
        self.pub_module_status = rospy.Publisher("/htcbot/module_status", StatusHtcbotModule, queue_size=10)

        self.pub_system_status = rospy.Publisher("/htcbot/system_status", StatusHtcbotSystem, queue_size=10)

        self.pub_sensor_status_array = rospy.Publisher("/htcbot/sensor_status_array", StatusHtcbotSensorArray, queue_size=10)
        self.pub_module_status_array = rospy.Publisher("/htcbot/module_status_array", StatusHtcbotModuleArray, queue_size=10)
        self.pub_voice_scene = rospy.Publisher("/htcbot/voice_play_scene", String, queue_size=1)

        self.server = Server(StatusControlConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        self.security_level = config.security_level
        self.low_battery_warn = config.low_battery_warn
        self.low_battery_danger = config.low_battery_danger
        self.laser_stop_front = config.laser_stop_front
        self.laser_stop_back = config.laser_stop_back
        self.laser_stop_left = config.laser_stop_left
        self.laser_stop_right = config.laser_stop_right
        return config

    def switch_status_callback(self, req):
        # rospy.loginfo("[sensor_control_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def callback_user_cmd(self, msg):
        status_htcbot_module = StatusHtcbotModule()
        status_htcbot_module.header.stamp = rospy.Time().now()
        status_htcbot_module.module_type = int(MODULE_TYPE.CONTROL.value)
        if msg.data == msg.NORMAL_PAUSE:
            status_htcbot_module.module_status = int(STATUS_TYPE.DANGER.value)
        else:
            status_htcbot_module.module_status = int(STATUS_TYPE.READY.value)
        self.pub_module_status.publish(status_htcbot_module)

    # def callback_pointcloud(self, msg):
    #     status_htcbot_sensor = StatusHtcbotSensor()
    #     status_htcbot_sensor.header.stamp = rospy.Time().now()
    #     status_htcbot_sensor.sensor_type = int(SENSOR_TYPE.LASER.value)
    #     status_htcbot_sensor.sensor_status = int(STATUS_TYPE.READY.value)

    #     # 判断 pointcloud 点云数目
    #     if msg.width <= self.laser_error_points:
    #         status_htcbot_sensor.sensor_status = int(STATUS_TYPE.EXC.value)
    #         status_htcbot_sensor.reason = "Laser Points Width {} Too Low".format(msg.width)

    #     self.pub_sensor_status.publish(status_htcbot_sensor)
        
    def callback_laser_detection(self, msg):
        status_htcbot_sensor = StatusHtcbotSensor()
        status_htcbot_sensor.header.stamp = rospy.Time().now()
        status_htcbot_sensor.sensor_type = int(SENSOR_TYPE.LASER.value)
        status_htcbot_sensor.sensor_status = int(STATUS_TYPE.READY.value)
        status_htcbot_sensor.reason = ""

        if msg.front_distance <= self.laser_stop_front:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value) 
            status_htcbot_sensor.reason = "Front {} Danger".format(format(msg.front_distance, ".2f"))
        if msg.back_distance <= self.laser_stop_back:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value) 
            status_htcbot_sensor.reason = "Back {} Danger".format(format(msg.back_distance, ".2f"))
        if msg.left_distance <= self.laser_stop_left:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value) 
            status_htcbot_sensor.reason = "Left {} Danger".format(format(msg.left_distance, ".2f"))
        if msg.right_distance <= self.laser_stop_right:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value) 
            status_htcbot_sensor.reason = "Right {} Danger".format(format(msg.right_distance, ".2f"))
        self.pub_sensor_status.publish(status_htcbot_sensor)

    def callback_gnss(self, msg):
        status_htcbot_sensor = StatusHtcbotSensor()
        status_htcbot_sensor.header.stamp = rospy.Time().now()
        status_htcbot_sensor.sensor_type = int(SENSOR_TYPE.GNSS.value)
        status_htcbot_sensor.reason = ""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.EXC.value)
            status_htcbot_sensor.reason = "Gps Data Is Nan"
        else:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.READY.value)
        self.pub_sensor_status.publish(status_htcbot_sensor)

    def callback_ultrasonic(self, msg):
        ultrasonic_state = [STATUS_TYPE.NONE, STATUS_TYPE.NONE, STATUS_TYPE.NONE, STATUS_TYPE.NONE]
        status_htcbot_sensor = StatusHtcbotSensor()
        status_htcbot_sensor.header.stamp = rospy.Time().now()
        status_htcbot_sensor.sensor_type = int(SENSOR_TYPE.ULTRASONIC.value)
        for i in range(0, msg.nums):
            id = i + 1
            dist = msg.distance[i]
            if (dist == 0):
                ultrasonic_state[i] = STATUS_TYPE.WARN
            elif (dist <= self.ultrasonic_distance_limit[i]):
                ultrasonic_state[i] = STATUS_TYPE.DANGER
                status_htcbot_sensor.reason = "Ultrasonic Dist {} Is Danger".format(dist)
            else:
                ultrasonic_state[i] = STATUS_TYPE.READY
                status_htcbot_sensor.reason = ""
        # handle ultrasonic_state 
        all_ready = all(state == STATUS_TYPE.READY for state in ultrasonic_state)
        danger_detected = any(state == STATUS_TYPE.DANGER for state in ultrasonic_state)
        if all_ready:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.READY.value)
        elif danger_detected:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value)
        else:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.WARN.value)
        self.pub_sensor_status.publish(status_htcbot_sensor)

    def callback_vehicle_status(self, msg):
        self.cur_speed = msg.cur_speed
        self.cur_steer = msg.cur_steer

        voice_scene_str = String()
        if msg.shift_level == 3:
            voice_scene_str.data = "1007"
        elif msg.cur_steer >= 7.5:
            voice_scene_str.data = "1006"
        elif msg.cur_steer <= -7.5:
            voice_scene_str.data = "1005"
        if msg.cur_speed > 0.01:
            self.pub_voice_scene.publish(voice_scene_str)

    def callback_battery_status(self, msg):
        status_htcbot_sensor = StatusHtcbotSensor()
        status_htcbot_sensor.header.stamp = rospy.Time().now()
        status_htcbot_sensor.sensor_type = int(SENSOR_TYPE.VEHICLE.value)
        if msg.capacity <= self.low_battery_danger:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.DANGER.value)
            status_htcbot_sensor.reason = "Battery {} Is Too Low".format(msg.capacity)
        elif msg.capacity <= self.low_battery_warn:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.WARN.value)
            status_htcbot_sensor.reason = "Battery {} Is Low".format(msg.capacity)
        else:
            status_htcbot_sensor.sensor_status = int(STATUS_TYPE.READY.value)
            status_htcbot_sensor.reason = str(msg.capacity)
        self.pub_sensor_status.publish(status_htcbot_sensor)

    def callback_sensor_status(self, msg):
        if msg.sensor_type == int(SENSOR_TYPE.GNSS.value):
            self.sensor_status_array.gnss_status = msg
        elif msg.sensor_type == int(SENSOR_TYPE.LASER.value):
            self.sensor_status_array.laser_status = msg
        elif msg.sensor_type == int(SENSOR_TYPE.ULTRASONIC.value):
            self.sensor_status_array.ultrasonic_status = msg
        elif msg.sensor_type == int(SENSOR_TYPE.CAMERA.value):
            self.sensor_status_array.camera_status = msg
        elif msg.sensor_type == int(SENSOR_TYPE.IMU.value):
            self.sensor_status_array.imu_status = msg
        elif msg.sensor_type == int(SENSOR_TYPE.VEHICLE.value):
            self.sensor_status_array.vehicle_status = msg
        else:
            pass

    def callback_module_status(self, msg):
        # rospy.loginfo("[module_control_node] ===> module.type: %d, module.status: %d", msg.module_type, msg.module_status)
        if msg.module_type == int(MODULE_TYPE.MAP.value):
            self.module_status_array.map_status = msg
        elif msg.module_type == int(MODULE_TYPE.LOCALIZER.value):
            self.module_status_array.localizer_status = msg
        elif msg.module_type == int(MODULE_TYPE.SENSOR.value):
            self.module_status_array.sensor_status = msg
        elif msg.module_type == int(MODULE_TYPE.WAYPOINT.value):
            self.module_status_array.waypoint_status = msg
        elif msg.module_type == int(MODULE_TYPE.PLANNER.value):
            self.module_status_array.planner_status = msg
        elif msg.module_type == int(MODULE_TYPE.CONTROL.value):
            self.module_status_array.control_status = msg
        elif msg.module_type == int(MODULE_TYPE.GATEWAY.value):
            self.module_status_array.gateway_status = msg
        else:
            pass

    def init_status_array(self):
        # 初始化模块状态数组
        self.module_status_array.status_array = [{'module_type': i+1, 'module_status': 0, 'reason': ""} for i in range(7)]
        # 初始化传感器状态数组
        self.sensor_status_array.status_array = [{'sensor_type': i+1, 'sensor_status': 0, 'reason': ""} for i in range(8)]

    def init_sensor_status(self):
        self.sensor_status_array.gnss_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.GNSS.value), sensor_status=int(STATUS_TYPE.NONE.value))
        self.sensor_status_array.laser_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.LASER.value), sensor_status=int(STATUS_TYPE.NONE.value))
        self.sensor_status_array.ultrasonic_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.ULTRASONIC.value), sensor_status=int(STATUS_TYPE.NONE.value))
        self.sensor_status_array.camera_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.CAMERA.value), sensor_status=int(STATUS_TYPE.NONE.value))
        self.sensor_status_array.imu_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.IMU.value), sensor_status=int(STATUS_TYPE.NONE.value))
        self.sensor_status_array.vehicle_status = StatusHtcbotSensor(sensor_type=int(SENSOR_TYPE.VEHICLE.value), sensor_status=int(STATUS_TYPE.NONE.value))  

    def init_module_status(self):
        self.module_status_array.map_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.MAP.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.localizer_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.LOCALIZER.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.sensor_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.SENSOR.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.waypoint_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.WAYPOINT.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.planner_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.PLANNER.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.control_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.CONTROL.value), module_status=int(STATUS_TYPE.NONE.value))
        self.module_status_array.control_status = StatusHtcbotModule(module_type=int(MODULE_TYPE.GATEWAY.value), module_status=int(STATUS_TYPE.NONE.value))

    def check_and_pub_status(self):
        # True ｜ False 
        system_status = True
        # module
        if self.module_status_array.map_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.module_status_array.localizer_status.module_status not in [int(STATUS_TYPE.READY.value), int(STATUS_TYPE.WARN.value)]:
            system_status = False
        if self.module_status_array.sensor_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.module_status_array.waypoint_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.module_status_array.planner_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.module_status_array.control_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.module_status_array.gateway_status.module_status != int(STATUS_TYPE.READY.value):
            system_status = False

        # sensor
        if self.use_gnss and self.sensor_status_array.gnss_status.sensor_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.use_laser and self.sensor_status_array.laser_status.sensor_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.use_ultrasonic and self.sensor_status_array.ultrasonic_status.sensor_status not in [int(STATUS_TYPE.READY.value), int(STATUS_TYPE.WARN.value)]:
            system_status = False
        if self.use_camera and self.sensor_status_array.camera_status.sensor_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.use_imu and self.sensor_status_array.imu_status.sensor_status != int(STATUS_TYPE.READY.value):
            system_status = False
        if self.use_vehicle and self.sensor_status_array.vehicle_status.sensor_status != int(STATUS_TYPE.READY.value):
            system_status = False

        # 根据系统安全级别控制
        # NONE 级别无安全限制
        # NORMAL 一般级别仅考虑障碍物
        # S    严格级别考虑整车模块系统情况 
        if self.security_level == str(SECURITY_LEVEL.NONE.value):
            system_status = True

        # 发布系统最终状态
        system_status_ = StatusHtcbotSystem()
        system_status_.status = system_status
        system_status_.reason = ""
        self.pub_system_status.publish(system_status_)
        
        self.module_status_array.header.stamp = rospy.Time().now()
        self.module_status_array.status_array = []
        self.module_status_array.status_array.append(self.module_status_array.map_status)
        self.module_status_array.status_array.append(self.module_status_array.localizer_status)
        self.module_status_array.status_array.append(self.module_status_array.sensor_status)
        self.module_status_array.status_array.append(self.module_status_array.waypoint_status)
        self.module_status_array.status_array.append(self.module_status_array.planner_status)
        self.module_status_array.status_array.append(self.module_status_array.control_status)
        self.module_status_array.status_array.append(self.module_status_array.gateway_status)
        self.pub_module_status_array.publish(self.module_status_array)

        self.sensor_status_array.header.stamp = rospy.Time().now()
        self.sensor_status_array.status_array = []
        self.sensor_status_array.status_array.append(self.sensor_status_array.gnss_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.laser_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.ultrasonic_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.camera_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.imu_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.vehicle_status)
        self.sensor_status_array.status_array.append(self.sensor_status_array.other_status)
        self.pub_sensor_status_array.publish(self.sensor_status_array)

        # pub vehicle        

if __name__ == '__main__':
    try:
        node = StatusControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass