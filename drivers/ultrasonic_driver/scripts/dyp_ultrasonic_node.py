#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# must python3

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-05 12:15:07
Description: 
# pip3 install pyserial
# pip3 install crcmod
'''

import rospy
import csv
import math
import os
import time
import struct
import crcmod
import serial
import serial.tools.list_ports

from dynamic_reconfigure.server import Server
from ultrasonic_driver.cfg import DypUltrasonicConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse
from ultrasonic_driver.msg import UltraSonicDetect

class DypUltrasonicNode:

    def __init__(self):
        rospy.init_node("dyp_ultrasonic_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 0
        self.server = Server(DypUltrasonicConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)
        self.serial_ = None
        self.serial_port = "/dev/ttyUSB0"
        # self.serial_port = "/dev/cu.usbserial-B0019JD8"
        self.baud_rate = 9600
        self.parity = 'N'
        self.stopbits = 1
        self.bytesize = 8
        self.timeout = 0.3
        self.device_address = 0x01


    def run(self):
        self.init()
        rate = rospy.Rate(5)  # 1 Hz
        self.connect_serial()
        while not rospy.is_shutdown():  # 添加条件
            if hasattr(self.serial_, 'isOpen') and self.serial_.isOpen():
                self.read_and_pub_detection()
            else:
                self.connect_serial()
            rate.sleep()

    def init(self):

        self.serial_port = rospy.get_param('~serial_port','/dev/ttyUSB0')
        self.frame_id = rospy.get_param('~frame_id', 'ultrasonic_link')

        self.pub_ultrasonic_detection = rospy.Publisher("/ultrasonic_detection", UltraSonicDetect, queue_size=10)

        ports = list(serial.tools.list_ports.comports())
        for port, desc, hwid  in ports:
            print(port, desc, hwid)

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[dyp_ultrasonic_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def connect_serial(self):
        try:
            self.serial_ = serial.Serial(port=self.serial_port,
                                    baudrate = self.baud_rate,
                                    timeout = self.timeout,
                                    bytesize = self.bytesize,
                                    parity = self.parity,
                                    stopbits = self.stopbits
                                    )
        except Exception as e:
            rospy.logerr("[dyp_ultrasonic_node] ===> Open serial failed, exit. {}".format(e))
            print(e)

    def calculate_crc16(self, data):
        crc16 = crcmod.predefined.Crc('modbus')
        crc16.update(data)
        crc_value = crc16.crcValue
        # 将 CRC 校验值的字节顺序反转
        crc_bytes = crc_value.to_bytes(2, byteorder='big')
        crc_bytes_reversed = crc_bytes[::-1]
        return crc_bytes_reversed   

    def read_and_pub_detection(self):
        try:
            # 记录发送命令前的时间
            start_time = time.time()
            # 构造要发送的数据
            command = struct.pack('>BBHH', self.device_address, 0x03, 0x0106, 0x0004)  # 主机发送的命令
            crc = self.calculate_crc16(command)
            data_to_send = command + crc  # 添加CRC校验值
            hex_data = ' '.join(format(byte, '02X') for byte in data_to_send)
            # print("send data:", hex_data)
            # 发送数据
            self.serial_.write(data_to_send)
            # 读取从机响应数据
            response = self.serial_.read(13)  # 读取从机响应数据
            if len(response) != 13:
                raise ValueError(f"Received unexpected response length: {len(response)}")
            hex_response = ' '.join(format(byte, '02X') for byte in response)
            # print("receive data:", hex_response)
            # 计算发送命令到接收响应所花费的时间
            end_time = time.time()
            elapsed_time = end_time - start_time
            # print("ms:", elapsed_time, "s")
            # 提取探头数量
            num_sensors = response[2] // 2  # 除以2是因为每个测量值占两个字节
            detect_msg = UltraSonicDetect()
            detect_msg.header.stamp = rospy.Time().now()
            detect_msg.nums = num_sensors
            # 提取每个探头的测量值
            for i in range(num_sensors):
                # 提取测量值的起始索引
                start_index = 3 + i * 2
                # 提取测量值的高位字节和低位字节
                high_byte = response[start_index]
                low_byte = response[start_index + 1]
                # 计算测量值
                measurement_value = (high_byte << 8) | low_byte
                # print("ultrasonic:", i+1, "distance:", measurement_value)
                detect_msg.distance.append(measurement_value)
            self.pub_ultrasonic_detection.publish(detect_msg)
        except Exception as e:
            rospy.logerr("[dyp_ultrasonic_node] ===> Read Data Error{}".format(e))

if __name__ == '__main__':
    try:
        node = DypUltrasonicNode()
        node.run()
    except rospy.ROSInterruptException:
        pass