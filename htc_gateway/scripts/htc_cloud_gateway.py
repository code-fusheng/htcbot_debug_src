#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-20 16:31:14
Description: 
'''

import rospy
import csv
import math
import os
import time
import tf
import json
from threading import Thread
import paho.mqtt.client as mqtt
from enum import Enum
from dynamic_reconfigure.server import Server
from htc_gateway.cfg import HtcCloudGatewayConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse
from htcbot_msgs.msg import UserCmd, VideoStreamControl
from htcbot_msgs.msg import StatusHtcbotModule
from can_msgs.msg import battery, vehicle_status
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from apscheduler.schedulers.background import BackgroundScheduler

class MODULE_TYPE(Enum):
    UNKNOWN = 0    
    MAP = 1
    LOCALIZER = 2
    SENSOR = 3
    WAYPOINT = 4
    PLANNER = 5
    CONTROL = 6
    GATEWAY = 7

class STATUS_TYPE(Enum):
    NONE = 0                # 默认
    READY = 1               # 就绪
    WARN = 2                # 警告
    DANGER = 3              # 危险
    EXC = 4                 # 异常
    EXPIRED = 5             # 过期

class TaskStatus:
    WAIT  = 0x01
    START = 0x02
    STOP  = 0x03
    PAUSE = 0x04
    CONTINUE = 0x05

class HtcCloudGatewayNode:

    def __init__(self):
        rospy.init_node("htc_cloud_gateway", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 1
        self.server = Server(HtcCloudGatewayConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        self.current_pose = PoseStamped()
        self.gps_fix = NavSatFix()
        self.battery_status = battery()
        self.vehicle_status = vehicle_status()
        self.scheduler = BackgroundScheduler()
        self.job_heartbeat_id = None
        self.module_status = StatusHtcbotModule()
        self.task_start_pose = None
        self.task_start_time = None
        self.task_key = None

    def run(self):
        self.init()   
        time.sleep(10)
        self.init_mqtt()  
        while not rospy.is_shutdown():  # 添加条件
            if not self.mqtt_client.is_connected():
                try:
                    self.mqtt_client.connect(self.broker, self.port, 60)
                    # 启动MQTT客户端循环
                    mqtt_thread = Thread(target=self.mqtt_client.loop_forever)
                    mqtt_thread.start()
                    break
                except Exception as e:
                    print("[htc_cloud_gateway] ===> Error while connecting to mqtt server")
                    time.sleep(5)  
            rospy.loginfo("[htc_cloud_gateway] ===> Gateway started")
            rospy.spin()
            self.mqtt_client.loop_stop()
            mqtt_thread.join()
            self.scheduler.remove_job(self.job_heartbeat_id)

    def init(self):
        self.client_id = rospy.get_param("/htc_cloud_gateway_node/mqtt_cloud_cid", "HTCBOT_A000*")
        self.broker = rospy.get_param("/htc_cloud_gateway_node/mqtt_cloud_broker", "118.190.156.22")
        self.port   = rospy.get_param("/htc_cloud_gateway_node/mqtt_cloud_port", 1883)
        self.username = rospy.get_param("/htc_cloud_gateway_node/mqtt_cloud_username", "htcbot")
        self.password = rospy.get_param("/htc_cloud_gateway_node/mqtt_cloud_password", "htcbot123456")
        self.vin = rospy.get_param("/htc_cloud_gateway_node/htcbot_vin", "HTCBOT_A000*")
        self.vtype = rospy.get_param("/htc_cloud_gateway_node/htcbot_vtype", "none")
        
        # 心跳消息
        self.pub_heartbeat_topic          = "/patrol_robot/up/device/car/heartbeat"
        self.heartbeat_relay = 1
        self.pub_response_topic           = "/patrol_robot/up/device/car/location/reply"
        self.sub_trigger_poseupdate_topic = "/patrol_robot/down/device/car/{}/location".format(self.vin)
        self.sub_task_control_topic       = "/patrol_robot/down/device/car/{}/task/control".format(self.vin)
        self.pub_task_status_topic        = "/patrol_robot/up/device/car/task/control"
        self.sub_video_stream_ctl_topic   = "/patrol_robot/down/device/car/{}/video/play".format(self.vin)

        self.pub_user_cmd = rospy.Publisher("user_cmd", UserCmd, queue_size=1)

        self.pub_module_status = rospy.Publisher("/htcbot/module_status", StatusHtcbotModule, queue_size=10)

        self.sub_current_pose   = rospy.Subscriber("/current_pose", PoseStamped, self.callback_current_pose)
        self.sub_battery_status = rospy.Subscriber("/battery_status", battery, self.callback_battery_status)
        self.sub_gps_fix        = rospy.Subscriber("/fix", NavSatFix, self.callback_gps_fix)
        self.sub_vehicle_status = rospy.Subscriber("/vehicle_status", vehicle_status, self.callback_vehicle_status)

        self.job_heartbeat_id = self.scheduler.add_job(self.send_heartbreat, 'interval', seconds=self.heartbeat_relay).id
        self.scheduler.start()

    def init_mqtt(self):
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.message_callback_add(self.sub_trigger_poseupdate_topic, self.on_message_upstream_pose)
        self.mqtt_client.message_callback_add(self.sub_task_control_topic, self.on_message_task_control)
        self.mqtt_client.message_callback_add(self.sub_video_stream_ctl_topic, self.on_message_videostream_control)
        self.mqtt_client.username_pw_set(self.username, self.password)

    def on_connect_mqtt(self, client, userdata, flags, rc):
        rospy.loginfo("[htc_cloud_gateway] ===> mqtt Connected with result code "+str(rc))
        # 0: 连接成功
        # 1: 连接被拒绝，不支持的协议版本
        # 2: 连接被拒绝，不合格的client identifier
        # 3: 连接被拒绝，服务器不可用
        # 4: 连接被拒绝，错误的用户名或密码
        # 5: 连接被拒绝，未授权
        self.mqtt_client.subscribe(self.sub_trigger_poseupdate_topic)
        self.mqtt_client.subscribe(self.sub_task_control_topic)
        self.mqtt_client.subscribe(self.sub_video_stream_ctl_topic)

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[htc_cloud_gateway] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def callback_current_pose(self, msg):
        self.current_pose = msg
        if self.task_start_pose:
            if (rospy.Time().now() - self.task_start_time).to_sec() < 10:
                return
            if math.fabs(self.current_pose.pose.position.x - self.task_start_pose.pose.position.x) < 1.0 and math.fabs(self.current_pose.pose.position.y - self.task_start_pose.pose.position.y) < 1.0 :
                user_cmd_msg = UserCmd()
                user_cmd_msg.data = user_cmd_msg.NORMAL_PAUSE
                self.pub_user_cmd.publish(user_cmd_msg)
                self.task_status = TaskStatus.STOP
                self.report_task_status()

    def callback_battery_status(self, msg):
        self.battery_status = msg

    def callback_gps_fix(self, msg):
        self.gps_fix = msg

    def callback_vehicle_status(self, msg):
        self.vehicle_status = msg

    def send_heartbreat(self):
        if not self.switch_status:
            return
        rospy.logdebug("[htc_cloud_gateway] ===> Thread send heartbeat job start")
        self.module_status.header.stamp = rospy.Time().now()
        self.module_status.module_type = int(MODULE_TYPE.GATEWAY.value)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (self.current_pose.pose.orientation.x,
             self.current_pose.pose.orientation.y,
             self.current_pose.pose.orientation.z,
             self.current_pose.pose.orientation.w,)
        )
        pub_msg = {
                    "timestamp": time.time(),
                    "vin":  self.vin,
                    "vtype": self.vtype,
                    "gps": {
                        "latitude":  self.gps_fix.latitude,
                        "longitude": self.gps_fix.longitude,
                        "altitude":  self.gps_fix.altitude
                    },
                    "position": {
                        "x": self.current_pose.pose.position.x,
                        "y": self.current_pose.pose.position.y,
                        "angle": math.degrees(yaw),
                        "speed": self.vehicle_status.cur_speed
                    },
                    "battery_capacity": self.battery_status.capacity,
                }
        if self.mqtt_client.is_connected:
            self.mqtt_client.publish(self.pub_heartbeat_topic, json.dumps(pub_msg), qos=0, retain=False)
            self.module_status.module_status = int(STATUS_TYPE.READY.value)
        else:
            rospy.logwarn("[htc_cloud_gateway] ===> mqtt server not connected!")
            self.module_status.module_status = int(STATUS_TYPE.EXC.value)
        self.pub_module_status.publish(self.module_status)

    def on_message_upstream_pose(self, client, userdata, msg):
        rospy.loginfo("[htc_cloud_gateway] Received message on topic1: "+str(msg.payload))
        line = str(msg.payload.decode("utf-8"))
        data = json.loads(line)
        if not "key" in data.keys():
            print("No key is found")
            return
        key = data['key']
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (self.current_pose.pose.orientation.x,
             self.current_pose.pose.orientation.y,
             self.current_pose.pose.orientation.z,
             self.current_pose.pose.orientation.w,)
        )
        pub_msg = {
                    "timestamp": time.time(),
                    "vin": self.vin,
                    "vtype": self.vtype,
                    "key": key,
                    "gps": {
                        "latitude":  self.gps_fix.latitude,
                        "longitude": self.gps_fix.longitude,
                        "altitude":  self.gps_fix.altitude
                    },
                    "position": {
                        "x": self.current_pose.pose.position.x,
                        "y": self.current_pose.pose.position.y,
                        "angle": math.degrees(yaw),
                        "speed": self.vehicle_status.cur_speed
                    },
                    "battery_capacity": self.battery_status.capacity,
                }
        self.mqtt_client.publish(self.pub_response_topic, json.dumps(pub_msg), qos=1, retain=False)
    
    def on_message_task_control(self, client, userdata, msg):
        rospy.loginfo("[gateway] Received task control message: "+str(msg.payload))
        line = str(msg.payload.decode("utf-8"))
        data = json.loads(line)
        if not "workTask" in data.keys():
            self.task_key = data["workTask"]
        if not "taskStatus" in data.keys():
            rospy.loginfo("[gateway] No taskStatus control command is found, full message is: {}".format(data))
            return
        cmd = int(data["taskStatus"])
        if cmd == TaskStatus.START:
            self.task_start_pose = self.current_pose
            self.task_start_time = rospy.Time().now()
        if cmd == TaskStatus.START or cmd == TaskStatus.CONTINUE:
            user_cmd_msg = UserCmd()
            user_cmd_msg.data = user_cmd_msg.NORMAL_RUN
            self.pub_user_cmd.publish(user_cmd_msg)
            self.start_time = rospy.Time.now()
        elif cmd == TaskStatus.PAUSE or cmd == TaskStatus.STOP:
            user_cmd_msg = UserCmd()
            user_cmd_msg.data = user_cmd_msg.NORMAL_PAUSE
            self.pub_user_cmd.publish(user_cmd_msg)
        else:
            rospy.logwarn("[gateway] Unrecognized cmd msg: {}".format(data))
        self.task_status = cmd
        self.report_task_status()

    def on_message_videostream_control(self, client, userdata, msg):
        rospy.loginfo("[gateway] Received video control control message: "+str(msg.payload))
        line = str(msg.payload.decode("utf-8"))
        data = json.loads(line)
        for _key in ["playStatus", "deviceIp", "inUrl", "outUrl"]:
            if not _key in data.keys():
                rospy.loginfo("[gateway] Param '{}' is found, full message is: {}".format(_key, data))
                return

        rosmsg_vs_ctl = VideoStreamControl()

        cmd = int(data["playStatus"])
        if cmd == 1: # start streaming
            rosmsg_vs_ctl.command = rosmsg_vs_ctl.COMMAND_SEND
        elif cmd == -1:  # stop
            rosmsg_vs_ctl.command = rosmsg_vs_ctl.COMMAND_STOP
        else:
            rospy.logwarn("[gateway] Control command '{}' is invalid. Data: {}".format(cmd, data))
            return

        rosmsg_vs_ctl.source_url = data["inUrl"]
        rosmsg_vs_ctl.target_url = data["outUrl"]
        rosmsg_vs_ctl.device_ip = data["deviceIp"]
        self.pub_vs_control.publish(rosmsg_vs_ctl)

    def report_task_status(self):
        pub_msg = {
                    "workTask": self.task_key,
                    "taskStatus": self.task_status,
                    "vin": self.vin,
                    "timestamp": time.time(),
                  }
        self.mqtt_client.publish(self.pub_task_status_topic, json.dumps(pub_msg), qos=1, retain=False)

if __name__ == '__main__':
    try:
        node = HtcCloudGatewayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass