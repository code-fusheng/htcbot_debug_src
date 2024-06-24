#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-22 15:55:45
Description: 
'''

import rospy
import csv
import math
import os
import time
from enum import Enum

from dynamic_reconfigure.server import Server
from media_tools.cfg import VoicePlayConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest
from std_msgs.msg import String
from sound_play.msg import SoundRequest

class VOICE_SCENES(Enum):
    
    UNKNOWN = 0  
    AUTO_RUNNING = 1001        # 自动运行
    FRONT_TRUN_LEFT = 1002     # 前方左转
    FRONT_TRUN_RIGHT = 1003    # 前方右转
    FRONT_STRAIGHT = 1004      # 前方直行
    TRUN_LEFT_WARN = 1005      # 左转,请注意!
    TRUN_RIGHT_WARN = 1006     # 右转,请注意!
    TRUN_BACK_WARN = 1007      # 倒车,请注意!
    
    WARN_BE_AVOID = 2001       # 危险,请避让!
    WARN_BE_FAR = 2002         # 危险,请远离!
    WARN_BE_NOT_FOLLOW = 2003  # 危险,请勿跟车!

    STATUS_READY = 3001      # 状态就绪
    STATUS_EXC = 3002        # 状态异常 
    
    SENSOR_READY = 3100         # 传感器状态就绪

    SENSOR_LASER_READY = 3120   # 雷达就绪
    SENSOR_LASER_WARN = 3121    # 雷达警告
    SENSOR_LASER_EXC = 3122     # 雷达异常
    
    SENSOR_VEHICLE_READY = 3170             # 底盘就绪
    SENSOR_VEHICLE_BATTERY_WARN = 3171      # 电池警告
    SENSOR_VEHICLE_BATTERY_DANGER = 3172    # 电池危险

    MODULE_READY = 3200         # 模块状态就绪
    
    MODULE_LOCALIZER_READY = 3220    # 定位就绪
    MODULE_LOCALIZER_WARN = 3221     # 定位质量差
    MODULE_LOCALIZER_EXC_TRY_RECOVER = 3222      # 定位异常 尝试恢复
    MODULE_LOCALIZER_GPS_OFFSET_TOO_FAR_WARN = 3223      # GPS 定位差

class VoicePlayNode:

    def __init__(self):
        rospy.init_node("voice_play_node", anonymous=False, log_level=rospy.INFO)
        self.voice_scenes_to_wav = {}
        self.is_debug = False
        self.switch_status = 0
        self.server = Server(VoicePlayConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)

    def run(self):
        self.init()
        rospy.loginfo("[voice_play_node] ===> started")
        rospy.spin()

    def init(self):
        self.voice_scenes_to_wav = {
            VOICE_SCENES.UNKNOWN: "unknown.wav",
            VOICE_SCENES.AUTO_RUNNING: "auto_running.wav",
            VOICE_SCENES.FRONT_TRUN_LEFT: "front_turn_left.wav",
            VOICE_SCENES.FRONT_TRUN_RIGHT: "front_turn_right.wav",
            VOICE_SCENES.FRONT_STRAIGHT: "front_straight.wav",
            VOICE_SCENES.TRUN_LEFT_WARN: "tts_turn_left_warn_1.wav",
            VOICE_SCENES.TRUN_RIGHT_WARN: "tts_turn_right_warn_1.wav",
            VOICE_SCENES.TRUN_BACK_WARN: "tts_back_warn_1.wav",
            VOICE_SCENES.WARN_BE_AVOID: "warn_be_avoid.wav",
            VOICE_SCENES.WARN_BE_FAR: "warn_be_far.wav",
            VOICE_SCENES.WARN_BE_NOT_FOLLOW: "warn_be_not_follow.wav",
            VOICE_SCENES.STATUS_READY: "status_ready.wav",
            VOICE_SCENES.STATUS_EXC: "status_exc.wav",
            VOICE_SCENES.SENSOR_READY: "sensor_ready.wav",
            VOICE_SCENES.SENSOR_LASER_READY: "sensor_laser_ready.wav",
            VOICE_SCENES.SENSOR_LASER_WARN: "sensor_laser_warn.wav",
            VOICE_SCENES.SENSOR_LASER_EXC: "sensor_laser_exc.wav",
            VOICE_SCENES.SENSOR_VEHICLE_READY: "sensor_vehicle_ready.wav",
            VOICE_SCENES.SENSOR_VEHICLE_BATTERY_WARN: "sensor_vehicle_battery_warn.wav",
            VOICE_SCENES.SENSOR_VEHICLE_BATTERY_DANGER: "sensor_vehicle_battery_danger.wav",
            VOICE_SCENES.MODULE_READY: "module_ready.wav",
            VOICE_SCENES.MODULE_LOCALIZER_READY: "module_localizer_ready.wav",
            VOICE_SCENES.MODULE_LOCALIZER_WARN: "module_localizer_warn.wav",
            VOICE_SCENES.MODULE_LOCALIZER_EXC_TRY_RECOVER: "module_localizer_exc_try_recover.wav",
            VOICE_SCENES.MODULE_LOCALIZER_GPS_OFFSET_TOO_FAR_WARN: "gps_offset_too_far_warn.wav"
        }
        self.local_voice_dir = rospy.get_param("~local_voice_dir", default="")
        self.voice_volume = rospy.get_param("~voice_volume", default=1.0)
        self.sub_voice_play = rospy.Subscriber("/htcbot/voice_play_source", String, self.callback_voice_play_source, queue_size=1)
        self.sub_voice_play_scene = rospy.Subscriber("/htcbot/voice_play_scene", String, self.callback_voice_play_scene, queue_size=1)
        self.pub_sound_play = rospy.Publisher("robotsound", SoundRequest, queue_size=5)

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[voice_play_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def callback_voice_play_source(self, msg):
        voice_source_file = self.local_voice_dir + msg.data
        rospy.loginfo("[voice_play_node] ===> voice source: %s", voice_source_file)
        sound_msg = SoundRequest()  # 创建SoundRequest消息实例
        sound_msg.sound = SoundRequest.PLAY_FILE  # 设置播放文件的命令
        sound_msg.command = SoundRequest.PLAY_ONCE
        sound_msg.arg = voice_source_file  # 设置音频文件路径
        sound_msg.volume = 1.0  # 设置音量
        self.pub_sound_play.publish(sound_msg)  # 发布消息
        # play

    def callback_voice_play_scene(self, msg):
        print(msg.data)
        scene_value = int(msg.data)
        print(scene_value)
        scene = VOICE_SCENES(scene_value)
        voice_source = self.voice_scenes_to_wav.get(scene)
        print(voice_source)
        voice_source_file = self.local_voice_dir +voice_source
        rospy.loginfo("[voice_play_node] ===> play scenes voice source: %s", voice_source_file)
        sound_msg = SoundRequest()  # 创建SoundRequest消息实例
        sound_msg.sound = SoundRequest.PLAY_FILE  # 设置播放文件的命令
        sound_msg.command = SoundRequest.PLAY_ONCE
        sound_msg.arg = voice_source_file  # 设置音频文件路径
        sound_msg.volume = 1.0  # 设置音量
        self.pub_sound_play.publish(sound_msg)  # 发布消息
        # time.sleep(2)

if __name__ == '__main__':
    try:
        node = VoicePlayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass