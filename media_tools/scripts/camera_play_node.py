#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-13 11:20:52
Description: 
'''

import rospy
import csv
import math
import os
import time
import cv2
import numpy as np

from dynamic_reconfigure.server import Server
from media_tools.cfg import CameraPlayConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class CameraPlayNode:

    def __init__(self):
        rospy.init_node("camera_play_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 1
        self.server = Server(CameraPlayConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        self.is_pub_topic = True
        self.width = 480
        self.height = 480
        self.image_pubs = []
        self.compressed_image_pubs = []
        self.device_ports = ["/dev/camera_left", "/dev/camera_right"]
        # self.device_ports = ["/dev/camera_front"]
        # self.device_ports = [0, 2]
        self.device_angle = [90, -90]
        self.frame_rate = 30
        self.jpeg_quality = 50
        self.bridge = CvBridge()

    def run(self):
        self.init()
        current_rate = self.frame_rate
        rate = rospy.Rate(int(current_rate))  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            if current_rate != self.frame_rate:
                rate = rospy.Rate(int(self.frame_rate))
                current_rate = self.frame_rate
            try:
                if self.switch_status:
                    for i, cap in enumerate(self.video_caps):
                        # 判断是否打开
                        if not cap.isOpened():
                            print("Error Open Device Port:%s", self.device_ports[i])
                            continue
                        ret, frame = cap.read()
                        if not ret:
                            continue
                        if self.is_pub_topic:
                            # Convert OpenCV image to ROS image
                            # 旋转图像
                            angle = self.device_angle[i]
                            if angle != 0:
                                _height, _width = frame.shape[:2]
                                _center = (_width // 2, _height // 2)
                                # 获取旋转矩阵，这里不进行尺寸调整
                                rotation_matrix = cv2.getRotationMatrix2D(_center, angle, 1.0)
                                frame = cv2.warpAffine(frame, rotation_matrix, (_width, _height), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

                            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                            # Set ROS image header timestamp
                            ros_image.header.stamp = rospy.Time.now()
                            ros_image.header.frame_id = "camera"
                            # Publish ROS image message
                            self.image_pubs[i].publish(ros_image)
            except CvBridgeError as e:
                print(e)
            except KeyboardInterrupt:
                print("KeyboardInterrupt received, shutting down...")
                self.shutdown()
            except Exception as e:
                print(e)
                self.shutdown()
            # Perform your main operations here
            # rospy.loginfo("[camera_play_node] ===> Running...")
            rate.sleep()

    def init_subscribers(self):
        pass

    def init_publishers(self):
        for i in range(len(self.device_ports)):
            topic_name = "/camera_"+str(i)+"/image_raw"
            topic_name_compressed = "/camera_"+str(i)+"/image_compressed"
            self.image_pubs.append(rospy.Publisher(topic_name, Image, queue_size=120))
            self.compressed_image_pubs.append(rospy.Publisher(topic_name_compressed, CompressedImage, queue_size=120))

    def init(self):
        self.init_publishers()
        self.init_subscribers()
        self.video_caps = [cv2.VideoCapture(url, cv2.CAP_V4L2) for url in self.device_ports]

    def shutdown(self):
        # 关闭所有的VideoCapture资源
        for cap in self.video_caps:
            cap.release()
        cv2.destroyAllWindows()

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        self.frame_rate = config.frame_rate
        self.device_ports = [str(port) for port in config.device_ports.split(',')]
        self.device_angle = [int(angle) for angle in config.device_angle.split(',')]
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[camera_play_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res

if __name__ == '__main__':
    try:
        node = CameraPlayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass