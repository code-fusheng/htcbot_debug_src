#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Author: code-fusheng 2561035977@qq.com
Date: 2024-05-30 22:25:27
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-13 14:34:26
FilePath: /src/htc_gateway/scripts/htc_cloud_websocket.py
# pip3 install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
# pip3 install scikit-build
'''

import rospy
import csv
import math
import os
import time
import threading
import asyncio
import websockets
import cv2
import numpy as np

from dynamic_reconfigure.server import Server
from htc_gateway.cfg import HtcCloudWebSocketConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class HtcCloudWebSocketNode:

    def __init__(self):
        rospy.init_node("htc_cloud_websocket_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 0
        self.server = Server(HtcCloudWebSocketConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        self.device_ports = ["/dev/camera_front"]
        self.image_subs = []

        
        self.websocket_uri = "ws://124.223.72.28:10240/robot/push/websocket"
        self.websocket_thread = threading.Thread(target=self.start_websocket_loop)
        self.websocket_thread.start()

    def run(self):
        self.init()
        # rate = rospy.Rate(1)  # 1 Hz
        # while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            # rospy.loginfo("[htc_cloud_websocket_node] ===> Running...")
            # rate.sleep()
        rospy.spin()

    def init(self):
        # init connect websocket
        self.init_subscribers()
        # init sub
        pass  # Add any initialization steps here

    def init_subscribers(self):
        for i in range(len(self.device_ports)):
            topic_name = "/camera_" + str(i) + "/image_raw"
            self.image_subs.append(rospy.Subscriber(topic_name, Image, self.image_callback, callback_args=i))

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[htc_cloud_websocket_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def image_callback(self, msg, camera_index):
        rospy.loginfo(f"Received image from camera {camera_index}")
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Encode image as JPEG
        _, jpeg_image = cv2.imencode('.jpg', cv_image)
        # Convert to bytes
        image_bytes = jpeg_image.tobytes()
        # Send image bytes via WebSocket
        asyncio.run_coroutine_threadsafe(self.send_image(image_bytes), self.websocket_loop)

    def start_websocket_loop(self):
        self.websocket_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.websocket_loop)
        self.websocket_loop.run_until_complete(self.connect_to_websocket())

    async def connect_to_websocket(self):
        async with websockets.connect(self.websocket_uri) as websocket:
            self.websocket = websocket
            while not rospy.is_shutdown():
                await asyncio.sleep(1)

    async def send_image(self, image_bytes):
        if self.websocket.open:
            await self.websocket.send(image_bytes)
        else:
            rospy.logwarn("WebSocket connection is not open")

if __name__ == '__main__':
    try:
        node = HtcCloudWebSocketNode()
        node.run()
    except rospy.ROSInterruptException:
        pass