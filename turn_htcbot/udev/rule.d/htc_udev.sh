# Bus 001 Device 022: ID 05a3:9230 ARC International
###
 # @Author: code-fusheng
 # @Date: 2024-05-15 15:25:02
 # @LastEditors: code-fusheng 2561035977@qq.com
 # @LastEditTime: 2024-05-18 12:45:38
 # @Description: 
### 
# touch /etc/udev/rules.d/htcbot.rules

# 超声波 485
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", GROUP:="dialout",  SYMLINK+="ultrasonic_front"
# Dove-485
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", MODE:="0777", GROUP:="dialout",  SYMLINK+="gps"
# D300
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="gps"

# 摄像头
# 同型号相机的命名通过USB插口区分
udevadm info -a -n /dev/video0 | grep KERNELS   # PS KERNELS 找设备的物理口+拓展坞口
SUBSYSTEM=="video[0-9]*", ATTR{idVendor}=="05a3", ATTR{idProduct}=="9230", MODE="0777"
KERNEL=="video0", KERNELS=="1-1", MODE:="0777", SYMLINK+="camera_left"
KERNEL=="video2", KERNELS=="1-2", MODE:="0777", SYMLINK+="camera_right"

SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="intel_d435i"

# 需要物理拔插设备
sudo service udev reload
sleep 2
service udev restart
# 
python >> import cv2
python >> cap = cv2.VideoCapture("/dev/camera_left", cv2.CAP_V4L2)