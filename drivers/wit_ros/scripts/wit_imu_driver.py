#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import time
import serial
import device_model

class WitImuDriver:

    def __init__(self):
        rospy.init_node("wit_imu_driver", anonymous=False, log_level=rospy.INFO)
        self.serial = None
        self.device = None
        self.is_debug = False
        self.port = "/dev/imu_wit9073"
        self.baud = 2000000
        self.imu_topic = ""

    def run(self):
        self.init()
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            # rospy.loginfo("[py_demo_node] ===> Running...")
            if self.device.isOpen:
                # pass
                print(
                "ID:{}  {}{}  AccX:{}  AAAAAbbbccY:{}  AccZ:{}  AsX:{}  AsY:{}  AsZ:{}  AngX:{}  AngY:{}  AngZ:{}  Hx:{}  Hy:{}  Hy:{}"
                .format(self.device.get("CanID"), self.device.get("canmode_2"), self.device.get("canmode_1"), self.device.get("AccX"), self.device.get("AccY"),
                        self.device.get("AccZ"), self.device.get("AsX"), self.device.get("AsY"), self.device.get("AsZ"), self.device.get("AngX"),
                        self.device.get("AngY"), self.device.get("AngZ"), self.device.get("HX"), self.device.get("HY"), self.device.get("HZ")))
            else:
                pass
                # self.open_device()
            # ... 以及其他您需要的传感器数据  
            rate.sleep()
        if self.device.isOpen:
            self.device.closeDevice()

    def init(self):
        self.port = rospy.get_param("~port", "/dev/imu_wit9073")
        self.baud = rospy.get_param("~baud", 2000000)
        self.imu_topic = rospy.get_param("~imu_topic", "imu_raw")
        self.open_device()
        pass  # Add any initialization steps here
    
    def open_device(self):
        try:
            self.device = device_model.DeviceModel("HWT9073", self.port, self.baud, 250)
            self.device.openDevice()
            print(self.device.isOpen)
            time.sleep(0.1)
        except Exception as e:
            print(e)
            rospy.loginfo("wit_imu_driver device open failed")
            exit()

    def open_com(self):
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=2)
            if self.serial.isOpen():
                rospy.loginfo("wit_imu_driver com port opened")
            else:
                self.serial.open()
                rospy.loginfo("wit_imu_driver com port open")
        except Exception as e:
            print(e)
            rospy.loginfo("wit_imu_driver com open failed")
            exit()

if __name__ == '__main__':
    try:
        node = WitImuDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass