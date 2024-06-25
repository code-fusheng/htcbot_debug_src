#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import time
import serial
import device_model
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped

class WitImuDriver:

    def __init__(self):
        rospy.init_node("wit_imu_driver", anonymous=False, log_level=rospy.INFO)
        self.serial = None
        self.device = None
        self.is_debug = False
        self.port = "/dev/imu_wit9073"
        self.baud = 2000000
        self.imu_topic = "imu_raw"

        self.prev_time = rospy.Time.now()

        # ============== data ============ #
        # IMU 数据相关
        # # 角度
        self.angle_roll = 0.0
        self.angle_pitch = 0.0
        self.angle_yaw = 0.0
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        # 磁场
        self.h_x = None
        self.h_y = None
        self.h_z = None

        # ============== odom ============ # 

        # 互补滤波参数
        self.alpha = 0.98  # 互补滤波系数
        self.dt = 0.1  # 时间步长，假设为0.1秒
       # 里程信息
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def run(self):
        self.init()
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            # rospy.loginfo("[py_demo_node] ===> Running...")
            if self.device.isOpen:
                # pass
                # print(self.device.isOpen)
                # print(
                # "ID:{}  {}{}  AccX:{}  AAAAAbbbccY:{}  AccZ:{}  AsX:{}  AsY:{}  AsZ:{}  AngX:{}  AngY:{}  AngZ:{}  Hx:{}  Hy:{}  Hy:{}"
                # .format(self.device.get("CanID"), self.device.get("canmode_2"), self.device.get("canmode_1"), self.device.get("AccX"), self.device.get("AccY"),
                #         self.device.get("AccZ"), self.device.get("AsX"), self.device.get("AsY"), self.device.get("AsZ"), self.device.get("AngX"),
                #         self.device.get("AngY"), self.device.get("AngZ"), self.device.get("HX"), self.device.get("HY"), self.device.get("HZ")))
                # print("AngX:{} AngY:{} AngZ:{}".format(self.device.get("AngX"), self.device.get("AngY"), self.device.get("AngZ")))
                print("HX:{} HY:{} HZ:{}".format(self.device.get("HX"), self.device.get("HY"), self.device.get("HZ")))
                # print("AccX:{} AccY:{} AccZ:{}".format(self.device.get("AccX"), self.device.get("AccY"), self.device.get("AccZ")))
                self.build_base_data()
                self.build_raw_data()
                # self.build_odom_data()
                # self.build_rpy_data()
                self.build_heading_data()
            else:
                # pass
                print("will be open status:{}".format(self.device.isOpen))
                self.open_device()
                print("has be opened status:{}".format(self.device.isOpen))
            # ... 以及其他您需要的传感器数据  
            rate.sleep()
        if self.device.isOpen:
            print("will be close status:{}".format(self.device.isOpen))
            self.device.closeDevice()
            print("has be closed status:{}".format(self.device.isOpen))

    def init(self):
        self.port = rospy.get_param("~port", "/dev/imu_wit9073")
        self.baud = rospy.get_param("~baud", 2000000)
        self.imu_topic = rospy.get_param("~imu_topic", "imu_raw")

        self.imu_raw_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self.imu_rpy_pub = rospy.Publisher("imu_rpy", Quaternion, queue_size=10)
        self.imu_pose_pub = rospy.Publisher('imu_pose', PoseStamped, queue_size=10)
        self.imu_odom_pub = rospy.Publisher("imu_odom", Odometry, queue_size=10)
        self.open_device()
        pass  # Add any initialization steps here
    
    def open_device(self):
        try:
            self.device = device_model.DeviceModel("HWT-9073", self.port, self.baud, 250)
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
            if self.serial.is_open:
                rospy.loginfo("wit_imu_driver com port opened")
            else:
                self.serial.open()
                rospy.loginfo("wit_imu_driver com port open")
        except Exception as e:
            print(e)
            rospy.loginfo("wit_imu_driver com open failed")
            exit()

    def build_base_data(self):
        self.angle_roll = self.device.get("AngX")
        self.angle_pitch = self.device.get("AngY")
        self.angle_yaw = self.device.get("AngZ")
        self.angular_velocity[0] = self.device.get("AsX")
        self.angular_velocity[1] = self.device.get("AsY")
        self.angular_velocity[2] = self.device.get("AsZ")
        self.linear_acceleration[0] = self.device.get("AccX")
        self.linear_acceleration[1] = self.device.get("AccY")
        self.linear_acceleration[2] = self.device.get("AccZ")
        self.h_x = self.device.get("HX")
        self.h_y = self.device.get("HY")

    def build_raw_data(self):
        # 创建一个Imu消息并填充数据  
        imu_msg = Imu()  
        imu_msg.header.stamp = rospy.Time.now()  
        imu_msg.header.frame_id = "imu_link"  
        imu_msg.linear_acceleration.x = self.linear_acceleration[0] 
        imu_msg.linear_acceleration.y = self.linear_acceleration[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2]
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        roll = self.angle_roll
        pitch = self.angle_pitch
        yaw = self.angle_yaw
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_raw_pub.publish(imu_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()  
        pose_msg.header.frame_id = "imu_link" 
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.imu_pose_pub.publish(pose_msg)

    def build_rpy_data(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        q_msg = Quaternion()
        # 加速度计 计算角度
        accel_x = self.device.get("AccX") 
        accel_y = self.device.get("AccY") 
        accel_z = self.device.get("AccZ")
        # 通过角速度求解角度
        roll_accel = math.atan2(accel_y, accel_z)
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2))
        # 陀螺仪积分
        gyro_x = self.device.get("AsX")  
        gyro_y = self.device.get("AsY") 

        self.angle_roll += gyro_x * dt
        self.angle_pitch += gyro_y * dt

        # 互补滤波
        self.angle_roll = self.alpha * self.angle_roll + (1 - self.alpha) * roll_accel
        self.angle_pitch = self.alpha * self.angle_pitch + (1 - self.alpha) * pitch_accel

        quaternion = self.euler_to_quaternion(self.angle_roll, self.angle_pitch, 0)
        q = Quaternion()
        q.x = quaternion[0]
        q.y = quaternion[1]
        q.z = quaternion[2]
        q.w = quaternion[3]
        # self.imu_rpy_pub.publish(q)

    def build_odom_data(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        # 计算里程信息
        if self.is_static():
            # 静止状态，使用角速度进行互补滤波
            self.current_theta += self.angular_velocity[2] * dt
        else:
            # 动态状态，使用加速度进行互补滤波
            self.current_x += self.linear_acceleration[0] * dt
            self.current_y += self.linear_acceleration[1] * dt
            self.current_theta += self.angular_velocity[2] * dt
        # 发布里程计信息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"

        # 设置位置信息
        odom_msg.pose.pose.position.x = self.current_x
        odom_msg.pose.pose.position.y = self.current_y
        odom_msg.pose.pose.position.z = 0.0

        # 设置姿态信息
        quaternion = self.euler_to_quaternion(self.angle_roll, self.angle_pitch, 0)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # 设置速度信息
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = self.linear_acceleration[0]  # 假设 x 方向为前进方向
        odom_msg.twist.twist.linear.y = self.linear_acceleration[1]
        odom_msg.twist.twist.linear.z = self.linear_acceleration[2]
        odom_msg.twist.twist.angular.x = self.angular_velocity[0]
        odom_msg.twist.twist.angular.y = self.angular_velocity[1]
        odom_msg.twist.twist.angular.z = self.angular_velocity[2]

        self.imu_odom_pub.publish(odom_msg)

    def build_heading_data(self):
        if self.h_x is not None and self.h_y is not None:
            heading = math.atan2(self.h_y, self.h_x)
            if heading < 0:
                heading += 2 * math.pi
            if heading > 2 * math.pi:
                heading -= 2 * math.pi
            heading_degrees = math.degrees(heading)
        else:
            heading_degrees = None
        print(heading_degrees)
        return heading_degrees

    def is_static(self):
        # 判断是否为静止状态，这里使用角速度的绝对值进行判断
        threshold = 0.1  # 可根据实际情况调整阈值
        return abs(self.angular_velocity[0]) < threshold and abs(self.angular_velocity[1]) < threshold and abs(self.angular_velocity[2]) < threshold
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        # 将角度转换为弧度
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

if __name__ == '__main__':
    try:
        node = WitImuDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass