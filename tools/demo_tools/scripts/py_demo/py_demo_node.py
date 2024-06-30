#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-15 14:11:45
Description: 
'''

import sys
import rospy
import csv
import math
import os
import time

from dynamic_reconfigure.server import Server
from demo_tools.cfg import PyDemoConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

class PyDemoNode:

    def __init__(self):
        rospy.init_node("py_demo_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 0
        self.server = Server(PyDemoConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)

        # 初始化 PyQt 应用和窗口
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("PyQt Window in ROS Node")
        self.layout = QVBoxLayout()
        self.label = QLabel("Switch Status: 0")
        self.layout.addWidget(self.label)
        self.window.setLayout(self.layout)
        self.window.resize(250, 150)

    def run(self):
        self.init()
        # self.window.show()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            # rospy.loginfo("[py_demo_node] ===> Running...")
            # self.label.setText(f"Switch Status: {self.switch_status}")
            rate.sleep()

        # 运行 PyQt 应用
        # sys.exit(self.app.exec_())

    def init(self):
        pass  # Add any initialization steps here

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[py_demo_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res

if __name__ == '__main__':
    try:
        node = PyDemoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass