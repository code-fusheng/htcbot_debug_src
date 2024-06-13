<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-06-12 09:55:36
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-06-13 11:02:07
 * @FilePath: /src/media_tools/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

# 同型号相机的命名通过 USB 插口区分

udevadm info -a -n /dev/video0 | grep KERNELS # PS KERNELS 找设备的物理口+拓展坞口
SUBSYSTEM=="video[0-9]\*", ATTR{idVendor}=="05a3", ATTR{idProduct}=="9230", MODE="0777"
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

#

v4l2-ctl --device=/dev/video<ID> -L
