# 维特智能驱动手册

https://pan.baidu.com/s/1PVfmRWPc4259qZ87InQbGQ?pwd=t6ab 密码空格

### 设备编号

- HWT-9073
  Bus 003 Device 011: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_wit9073"
```

mac
Port: /dev/cu.usbserial-14330, Description: USB Serial, Hardware ID: USB VID:PID=1A86:7523 LOCATION=20-3.3

### Input

### Output

```
self.imu_raw_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
self.imu_rpy_pub = rospy.Publisher("imu_rpy", Quaternion, queue_size=10)
self.imu_pose_pub = rospy.Publisher('imu_pose', PoseStamped, queue_size=10)
self.imu_odom_pub = rospy.Publisher("imu_odom", Odometry, queue_size=10)
```
