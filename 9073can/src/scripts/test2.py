#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time
import device_model


import rospy  

from sensor_msgs.msg import Imu  

#====================================================================================================================
# device = device_model.DeviceModel("测试设备", "/dev/imu",2000000, 250)
# # device = device_model.DeviceModel("测试设备", "COM5", 115200, 250)
# # 开启设备 Turn on the device
# device.openDevice()
# time.sleep(0.1)

# # 读取回传速率
# # device.readReg(0x03)
# # time.sleep(1)

# # 设置20hz回传速率
# # device.writeReg(0x03, 7)

# # 设置透传模式
# # device.setET()

# # 数据展示
# while True:
#     print(
#         "ID:{}  {}{}  AccX:{}  AAAAAAAAAAAAAAAAAAAAAAAAccY:{}  AccZ:{}  AsX:{}  AsY:{}  AsZ:{}  AngX:{}  AngY:{}  AngZ:{}  Hx:{}  Hy:{}  Hy:{}"
#         .format(device.get("CanID"), device.get("canmode_2"), device.get("canmode_1"), device.get("AccX"), device.get("AccY"),
#                 device.get("AccZ"), device.get("AsX"), device.get("AsY"), device.get("AsZ"), device.get("AngX"),
#                 device.get("AngY"), device.get("AngZ"), device.get("HX"), device.get("HY"), device.get("HZ")))
#     time.sleep(0.1)
#====================================================================================================================

def sensor_data_publisher():  
    # 初始化节点  
    rospy.init_node('sensor_data_publisher', anonymous=True)  
  
    # 创建一个Publisher，发布名为'imu_data'的sensor_msgs/Imu话题  
    pub = rospy.Publisher('imu_raw', Imu, queue_size=10)  
  
    # 拿到设备模型  
    device = device_model.DeviceModel("测试设备", "/dev/imu", 2000000, 250)  
    # device = device_model.DeviceModel("测试设备", "/dev/ttyUSB3", 2000000, 250)  
  
    # 开启设备  
    device.openDevice()  
    time.sleep(0.1)  
  
    # 数据展示并发布  
    rate = rospy.Rate(1) # 10hz  
    while not rospy.is_shutdown():  
        # 获取传感器数据  
        can_id = device.get("CanID")  
        acc_x = device.get("AccX")  
        acc_y = device.get("AccY")  
        acc_z = device.get("AccZ")  
        print(
        "ID:{}  {}{}  AccX:{}  AAAAAbbbccY:{}  AccZ:{}  AsX:{}  AsY:{}  AsZ:{}  AngX:{}  AngY:{}  AngZ:{}  Hx:{}  Hy:{}  Hy:{}"
        .format(device.get("CanID"), device.get("canmode_2"), device.get("canmode_1"), device.get("AccX"), device.get("AccY"),
                device.get("AccZ"), device.get("AsX"), device.get("AsY"), device.get("AsZ"), device.get("AngX"),
                device.get("AngY"), device.get("AngZ"), device.get("HX"), device.get("HY"), device.get("HZ")))
        # ... 以及其他您需要的传感器数据  
  
        # 创建一个Imu消息并填充数据  
        imu_msg = Imu()  
        imu_msg.header.stamp = rospy.Time.now()  
        imu_msg.header.frame_id = "imu_frame"  
        imu_msg.linear_acceleration.x = acc_x  
        imu_msg.linear_acceleration.y = acc_y  
        imu_msg.linear_acceleration.z = acc_z  

        imu_msg.angular_velocity.x = device.get("AsX")  
        imu_msg.angular_velocity.y = device.get("AsY")  
        imu_msg.angular_velocity.z = device.get("AsZ")  


        imu_msg.orientation.x =device.get("AngX")  
        imu_msg.orientation.y = device.get("AngY")  
        imu_msg.orientation.z =device.get("AngZ")  
        imu_msg.orientation.w = 0.0



       
        # ... 设置其他IMU字段，如angular_velocity  
  
        # 发布消息  
        pub.publish(imu_msg)  
  
        # 控制发布频率  
        rate.sleep()  
  
    # 关闭设备  
    device.closeDevice()  
  
if __name__ == '__main__':  
    try:  
        sensor_data_publisher()  
    except rospy.ROSInterruptException:  

        pass

