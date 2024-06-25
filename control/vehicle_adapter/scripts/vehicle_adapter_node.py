#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-21 19:08:36
Description: 
'''

import rospy
import csv
import math
import os
import time

from dynamic_reconfigure.server import Server
from vehicle_adapter.cfg import VehicleAdapterConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest
from htcbot_msgs.msg import StatusHtcbotSystem
from can_msgs.msg import ecu
from std_msgs.msg import String

class VehicleAdapterNode:

    def __init__(self):
        rospy.init_node("vehicle_adapter_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.enable_smooth_steer = True
        self.enable_smooth_speed = True
        self.switch_status = 1
        self.server = Server(VehicleAdapterConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        self.system_status = False
        self.updated_time = None
        self.pre_pre_speed = 0.0
        self.pre_speed = 0.0
        self.pre_pre_steer = 0.0
        self.pre_steer = 0.0
        self.bias = 0
        self.steer_left_ratio = 1
        self.steer_right_ratio = 1
        self.max_speed_limit = 1.0
        self.min_speed_limit = 0.5

    def run(self):
        self.init()
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            now = rospy.Time().now()
            time_diff = (now - self.updated_time).to_sec()
            if time_diff >= 1.0:
                self.system_status = False
            # rospy.loginfo("[vehicle_adapter_node] ===> Running... system_status: %d", self.system_status)
            rate.sleep()

    def init(self):
        self.updated_time = rospy.Time().now()
        self.enable_smooth_steer = rospy.get_param("~enable_smooth_steer", default=True)
        self.enable_smooth_speed = rospy.get_param("~enable_smooth_speed", default=True)
        self.max_speed_limit = rospy.get_param("~max_speed_limit", default=4.0)
        self.min_speed_limit = rospy.get_param("~min_speed_limit", default=0.2)

        self.sub_system_status = rospy.Subscriber("/htcbot/system_status", StatusHtcbotSystem, self.callback_system_status, queue_size=10)
        self.sub_control = rospy.Subscriber("/pure_pursuit/ecu", ecu, self.callback_pure_pursuit_ecu, queue_size=10)
        
        self.pub_control = rospy.Publisher("/ecu", ecu, queue_size=10)
        self.pub_voice_scene = rospy.Publisher("/htcbot/voice_play_scene", String, queue_size=1)
        
        pass  # Add any initialization steps here

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[vehicle_adapter_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def callback_system_status(self, msg):
        self.system_status = msg.status
        self.updated_time = rospy.Time().now()

    def callback_pure_pursuit_ecu(self, msg):
        # 获取当前时间
        now = rospy.Time().now()
        _ecu_msg = ecu()
        cur_speed = msg.motor
        cur_steer = msg.steer + self.bias
        # 前一次状态
        previous_status = False
        # 更新状态时间校验
        time_diff = (now - self.updated_time).to_sec()
        if time_diff <= 1.0:
            current_status = self.system_status
        else:
            current_status = False
        # 检查系统是否处理 False
        if not current_status:
            _ecu_msg.shift = _ecu_msg.SHIFT_D
            _ecu_msg.brake = True
            _ecu_msg.motor = 0
            _ecu_msg.steer = 0
            self.pub_control.publish(_ecu_msg)
            return
        
        # 非刹车状态 对转向进行平滑
        if self.enable_smooth_steer:
            if cur_steer > 0:
                smooth_steer = cur_steer * self.steer_left_ratio
            else:
                smooth_steer = cur_steer * self.steer_right_ratio
            cur_steer = smooth_steer
        # 平滑速度
        if self.enable_smooth_speed:
            smooth_speed = (self.pre_speed + cur_speed) / 2.0
            smooth_speed = min(self.max_speed_limit, smooth_speed)    # 取其小
            
            # 转向限制速度
            if abs(cur_steer) >= 30:
                smooth_speed = self.min_speed_limit
            else:
                smooth_speed = cur_speed - (cur_speed - self.min_speed_limit) * (abs(cur_steer / 30))
            cur_speed = smooth_speed 

        # 根据障碍物调整速度 => 前方障碍物检测
        # 

        if cur_speed > self.max_speed_limit:
            cur_speed = self.max_speed_limit
        if cur_speed != 0 and cur_speed < self.min_speed_limit:
            cur_speed = self.min_speed_limit

        _ecu_msg.brake = False
        _ecu_msg.motor = cur_speed
        _ecu_msg.steer = cur_steer
        _ecu_msg.shift = msg.shift

        self.pub_control.publish(_ecu_msg)
        self.pre_pre_speed = self.pre_speed
        self.pre_speed = cur_speed
        self.pre_pre_steer = self.pre_steer
        self.pre_steer = cur_steer

        voice_scene_str = String()
        voice_scene_str.data = "1005"
        # self.pub_voice_scene.publish(voice_scene_str)

if __name__ == '__main__':
    try:
        node = VehicleAdapterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass