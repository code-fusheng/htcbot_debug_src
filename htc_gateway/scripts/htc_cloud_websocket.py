#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Author: code-fusheng 2561035977@qq.com
Date: 2024-05-30 22:25:27
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-13 18:31:55
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
from websockets.extensions.permessage_deflate import ClientPerMessageDeflateFactory
import cv2
import numpy as np
import struct
import zlib

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
        
        self.image_subs = []

        self.front_camera_topic = "/camera/color/image_raw/compressed"
        self.front_image_sub = None
        self.back_camera_topic = ""
        self.back_image_sub = None
        self.left_camera_topic = "/camera_0/image_raw"
        self.left_image_sub = None
        self.right_camera_topic = "/camera_1/image_raw"
        self.right_image_sub = None

        self.websocket = None

        self.last_image_time = 0
        self.image_frequency = 5
        self.image_interval = 1.0 / self.image_frequency
        self.image_quality = 50

        self.websocket_ip = "124.223.72.28"
        self.websocket_uri = "ws://124.223.72.28:10240/robot/push/websocket?sid=car&vin=HTCBOT_A0003"
        # self.websocket_thread = threading.Thread(target=self.start_websocket_loop)
        # self.websocket_thread.start()

        self.check_connection_thread = threading.Thread(target=self.start_websocket_loop)
        self.check_connection_thread.start()

        self.frames = []

        self.front_switch = False
        self.back_switch = False
        self.left_switch = False
        self.right_switch = False

        # self.bridge = CvBridge()

    def run(self):
        self.init()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            rate.sleep()

    def init(self):
        # init connect websocket
        self.init_subscribers()
        # init sub
        pass  # Add any initialization steps here

    def init_subscribers(self):
        # for i in range(len(self.device_ports)):
        #     topic_name = "/camera_" + str(i) + "/image_raw"
        #     self.image_subs.append(rospy.Subscriber(topic_name, Image, self.image_callback, callback_args=i))
        # self.back_image_sub = rospy.Subscriber(self.back_camera_topic, Image, self.image_callback, queue_size=1, callback_args="Back")
        pass

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        self.image_frequency = config.image_frequency
        self.image_quality = config.image_quality
        self.front_camera_topic = config.front_camera_topic
        self.back_camera_topic = config.back_camera_topic
        self.left_camera_topic = config.left_camera_topic
        self.right_camera_topic = config.right_camera_topic
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[htc_cloud_websocket_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def image_callback(self, msg, camera_index):
        rospy.loginfo(f"Received image from camera {camera_index}")
        current_time = time.time()
        if current_time - self.last_image_time < self.image_interval:
            return
        self.last_image_time = current_time
        # Convert ROS Image message to OpenCV image
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Encode image as JPEG
        np_arr = np.frombuffer(msg.data, np.uint8)
        # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        try:
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Failed to decode image: {e}")
            return
        _, jpeg_image = cv2.imencode('.jpg', cv_image)

        # Convert to bytes
        
        image_bytes = jpeg_image.tobytes()
        image_size_kb = len(image_bytes) / 1024
        rospy.loginfo(f"image size: {image_size_kb:.2f} KB")
        # Send image bytes via WebSocket
        asyncio.run_coroutine_threadsafe(self.send_image(image_bytes), self.websocket_loop)

    def compressed_image_callback(self, msg, camera_index):
        current_time = time.time()
        if current_time - self.last_image_time < self.image_interval:
            return
        self.last_image_time = current_time
        rospy.loginfo(f"Received compressed image from camera {camera_index}")
        # Convert ROS CompressedImage message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_image = cv2.resize(cv_image, (640, 480))
        # Encode image as JPEG
        _, jpeg_image = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality])
        # Convert to bytes
        image_bytes = jpeg_image.tobytes()
        image_size_kb = len(image_bytes) / 1024
        rospy.loginfo(f"Compressed image size: {image_size_kb:.2f} KB")
        # compressed_data = zlib.compress(image_bytes)
        asyncio.run_coroutine_threadsafe(self.send_image(image_bytes), self.websocket_loop)
        # self.frames.append(image_bytes)

    def start_websocket_loop(self):
        self.websocket_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.websocket_loop)
        self.websocket_loop.run_until_complete(self.check_websocket_connection())
        self.websocket_loop.run_forever()

    async def check_websocket_connection(self):
        while not rospy.is_shutdown():
            print(self.websocket)
            if self.websocket is None:
                rospy.logwarn("WebSocket connection lost, attempting to reconnect...")
                await self.connect_to_websocket()
            else:
                rospy.loginfo("WebSocket connected")
            await asyncio.sleep(1)

    async def connect_to_websocket(self):
        try:
            self.websocket = await websockets.connect(self.websocket_uri)
            rospy.loginfo("WebSocket connection established")
            asyncio.ensure_future(self.receive_messages())  # 启动接收消息的任务
            # asyncio.ensure_future(self.send_images_packs())
        except Exception as e:
            rospy.logerr(f"Failed to connect to WebSocket: {e}")

    async def receive_messages(self):
        try:
            async for message in self.websocket:
                rospy.loginfo(f"Received message: {message}")
                        # 取消所有订阅器的订阅
                for sub in [self.front_image_sub, self.back_image_sub, self.left_image_sub, self.right_image_sub]:
                    if sub:
                        sub.unregister()
                # 根据消息订阅新的话题
                try:
                    if message == "Front":
                        self.front_image_sub = rospy.Subscriber(self.front_camera_topic, CompressedImage, self.compressed_image_callback, queue_size=1, callback_args="Front")
                        rospy.loginfo("Subscribed to Front camera")
                    elif message == "Back":
                        self.back_image_sub = rospy.Subscriber(self.back_camera_topic, Image, self.image_callback, queue_size=1, callback_args="Back")
                        rospy.loginfo("Subscribed to Back camera")
                    elif message == "Left":
                        self.left_image_sub = rospy.Subscriber(self.left_camera_topic, Image, self.image_callback, queue_size=1, callback_args="Left")
                        rospy.loginfo("Subscribed to Left camera")
                    elif message == "Right":
                        self.right_image_sub = rospy.Subscriber(self.right_camera_topic, Image, self.image_callback, queue_size=1, callback_args="Right")
                        rospy.loginfo("Subscribed to Right camera")
                    else:
                        rospy.logwarn(f"message: {message}")
                except Exception as sub_create_e:
                    rospy.logerr(f"Error creating subscriber: {sub_create_e}")
        except websockets.ConnectionClosed:
            rospy.logwarn("WebSocket connection closed during message reception")
        except Exception as e:
            rospy.logerr(f"Error receiving WebSocket messages: {e}")

    async def send_images_packs(self):
        while True:
            await asyncio.sleep(1)
            if self.frames:
                # Pack all frames in one message, each frame prefixed with its length
                packed_frames = b''.join([struct.pack('!I', len(frame)) + frame for frame in self.frames])
                await self.send_image(packed_frames)
                self.frames.clear()

    async def send_image(self, image_bytes):
        if self.websocket and self.websocket.open:
            try:
                await self.websocket.send(image_bytes)
            except websockets.ConnectionClosed:
                rospy.logwarn("WebSocket connection is not open")
            except Exception as e:
                rospy.logerr(f"Error sending image: {e}")

if __name__ == '__main__':
    try:
        node = HtcCloudWebSocketNode()
        node.run()
    except rospy.ROSInterruptException:
        pass