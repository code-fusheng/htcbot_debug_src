## 系统环境安装手册

### 初始化系统环境

> 网络优化

```
# Github 仓库代理
https://ghproxy.com/
# 科学订阅地址
https://sub.wl-sub1.com/api/v1/client/subscribe?token=1ddb6feb800a114b7bdb3afc43373ddf
# pip 源
pip3 install pyproj -i https://pypi.tuna.tsinghua.edu.cn/simple
https://mirror.tuna.tsinghua.edu.cn/help/ubuntu/
```

> 安装 ssh

```shell
# 安装 ssh
sudo apt install -y openssh-server
sudo vim /etc/ssh/sshd_config
sudo service ssh restart
# ~/.ssh/authorized_keys
```

### 安装 ROS

```shell
wget http://fishros.com/install -O fishros && . fishros
```

### Python

```
sudo apt-get update
sudo apt --fix-broken install
sudo apt install python3.8
sudo apt-get install python3-setuptools
sudo apt-get install python3-pip
pip install virtualenv
sudo apt-get install python3.8 python3.8-venv python3.8-dev


python<version> -m venv <virtual-environment-name>
eg:
 mkdir projectA
 cd projectA
 python3 -m venv lpr_py
 # python3.8 -m venv lpr_py38
 source lpr_py/bin/activate
 deactivate
```

### 工程部署

```shell
sudo apt install -y git
git clone -b master https://github.com/code-fusheng/htc-robot-ros_src.git
git submodule update --init --recursive
```

### 安装基础依赖

```shell
sudo apt install -y vim \
cutecom \
ros-$ROS_DISTRO-audio-common \
git \
terminator \
libmetis-dev \
libpcap-dev \
ros-$ROS_DISTRO-serial \
ros-$ROS_DISTRO-tf2-sensor-msgs \
ros-$ROS_DISTRO-costmap-converter \
ros-$ROS_DISTRO-mbf-costmap-core \
ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-nmea-msgs \
ros-$ROS_DISTRO-gps-common \
ros-$ROS_DISTRO-mbf-msgs \
ros-$ROS_DISTRO-gmapping \
ros-$ROS_DISTRO-hector-mapping \
ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-cartographer-ros \
pcl-tools \
ros-$ROS_DISTRO-bfl \
ros-$ROS_DISTRO-gps-common \
ros-$ROS_DISTRO-rqt-top \
ros-$ROS_DISTRO-python-qt-binding \
ros-$ROS_DISTRO-rosbag \
ros-$ROS_DISTRO-rqt-common-plugins \
ros-$ROS_DISTRO-message-filters
```

###

### GPS

sudo apt-get install libgps-dev
pip install pyproj

git clone https://github.com/nobleo/rviz_satellite
sudo apt-get install ros-melodic-mapviz

```
$SET CASTER rtk.ntrip.qxwz.com,8002,qxygel001,1cef2a4,RTCM32_GGB,*AA
```

### Video

```
# sudo apt-get install ros-$ROS_DISTRO-usb-cam \
sudo apt-get install cheese
cheese

sudo apt install v4l-utils libv4l-dev

v4l2-ctl -d 1 --all

PS: audio-common 音频处理

pip install -i https://pypi.tuna.tsinghua.edu.cn/simple opencv-python==4.2.0.32

sudo apt-get install ros-$ROS_DISTRO-cv-bridge \
ros-$ROS_DISTRO-image-transport

pip install opencv-python

pip install pyaudio

# 本机视频流服务
docker pull bluenviron/mediamtx:latest
docker run --rm -it \
-e MTX_PROTOCOLS=tcp \
-e MTX_WEBRTCADDITIONALHOSTS=192.168.x.x \
-p 8554:8554 \
-p 1935:1935 \
-p 8888:8888 \
-p 8889:8889 \
-p 8890:8890/udp \
-p 8189:8189/udp \
bluenviron/mediamtx


```

### MQTT

```
pip install paho-mqtt
pip install apscheduler
# apscheduler 高版本存在 python 2.7 用不了 datetime.datetime
pip install apscheduler==3.7.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Mapping 建图相关

```
https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
sudo apt install ros-$ROS_DISTRO-hector-mapping
```

### Gazebo

```
sudo apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs \
ros-$ROS_DISTRO-gazebo-msgs \
ros-$ROS_DISTRO-gazebo-plugins \
ros-$ROS_DISTRO-gazebo-ros-control \
ros-$ROS_DISTRO-velodyne-simulator
```

### 安装 autoware 移植基础依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-autoware-msgs \
ros-$ROS_DISTRO-autoware-config-msgs
ros-$ROS_DISTRO-rqt-runtime-monitor

```

### IMU 标定

```shell
sudo apt-get install libdw-dev

sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev
libgoogle-glog-dev libgtest-dev

git clone https://github.com/ceres-solver/ceres-solver.git
mkdir build
cd build
cmake ..
sudo make install
catkin_make -DCATKIN_WHITELIST_PACKAGES=code_utils
catkin_make -DCATKIN_WHITELIST_PACKAGES=imu_utils

```

### GNSS

```
sudo apt install -y ros-$ROS_DISTRO-nmea-msgs
# 参考 nmea_navsat_drive
```

### 安装相机依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-libuvc-camera \
ros-$ROS_DISTRO-libuvc-ros \
ros-$ROS_DISTRO-libuvc \
```

> 编译 libuvc（noetic 环境）

PS: noetic 环境下 libuvc 需要拉源码进行编译 [https://github.com/libuvc/libuvc](https://github.com/libuvc/libuvc)

```shell
git clone https://ghproxy.com/https://github.com/libuvc/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
# ros_astra_camera 功能包设置 libuvc 环境
find_package(libuvc QUIET)
if(EXISTS "/usr/local/include/libuvc")
  set(libuvc_INCLUDE_DIRS "/usr/local/include/libuvc")
  set(libuvc_LIBRARIES "/usr/local/lib/libuvc.so")
endif()
```

```shell
# 安装依赖
sudo apt install -y ros-$ROS_DISTRO-ddynamic-reconfigure \
libudev-dev \
pkg-config \
libgtk-3-dev \
libusb-1.0-0-dev \
libglfw3-dev \
libssl-dev
# 下载 librealsense 源码
git clone https://github.com/IntelRealSense/librealsense.git
# git clone https://ghproxy.com/https://github.com/IntelRealSense/librealsense.git
cd librealsense
# 安装权限脚本
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
# 编译
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=ture
make sudo make install
sudo make install
# 建议上科学
git clone https://gitcode.net/mirrors/curl/curl.git
git clone https://github.com/curl/curl.git
cd build/third-party

#
git clone https://github.com/IntelRealSense/realsense-ros.git
git checkout 2.3.2
git checkout -b 2.3.2 2.3.2
#
roslaunch realsense2_camera rs_camera.launch
```

```shell
# 打开imu
<arg name="enable_gyro"         default="true"/>
<arg name="enable_accel"        default="true"/>
# 联合方式copy或linear_interpolation
<arg name="unite_imu_method"          default="linear_interpolation"/>
# 时间戳同步
<arg name="enable_sync"               default="true"/>
```

### 安装激光雷达

sudo apt install -y ros-$ROS_DISTRO-velodyne-description \
ros-$ROS_DISTRO-joint-state-publisher-gui

```shell

```

```shell
# Wireshark 抓包调试雷达
```

### 安装 rtabmap

```shell
sudo apt install -y ros-$ROS_DISTRO-rtabmap*
```

### 安装 Rtapmap-Ros

```shell
# 编译 rtabmap_ros
git clone https://github.com/introlab/rtabmap_ros.git
# 不同的版本环境需要切换分支
git checkout noetic-devel
git checkout melodic-devel
```

```
git submodule add https://github.com/pal-robotics/ddynamic_reconfigure.git drivers/ddynamic_reconfigure
git submodule add https://github.com/code-fusheng/realsense-ros.git drivers/realsense-ros
```

### 安装 Lio-Sam

```shell
# 基础依赖
sudo apt-get install -y ros-$ROS_DISTRO-navigation \
ros-$ROS_DISTRO-robot-localization \
ros-$ROS_DISTRO-robot-state-publisher \
ros-$ROS_DISTRO-fake-localization \
libmetis-dev \
libtbb-dev
# gtsam method:1 (可能需要科学) # noetic 见下方问题处理
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

# 拉取 lio-sam & 编译
git clone https://github.com/TixiaoShan/LIO-SAM.git
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES=lio_sam
```

> lio-sam 编译问题(noetic)

[https://github.com/TixiaoShan/LIO-SAM/issues/206](https://github.com/TixiaoShan/LIO-SAM/issues/206)

```shell
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
# 修改
#include <opencv/cv.h> => <opencv2/opencv.hpp>
#                      => <opencv2/imgproc.hpp>
#
# 修改 set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
# 移位置
#include <pcl/kdtree/kdtree_flann.h>
```

### 安装 LeGo-Loam

```shell
sudo apt-get install -y libmetis-dev
catkin_make -DCATKIN_WHITELIST_PACKAGES=cloud_msgs
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES=lego_loam
#include <opencv2/opencv.hpp>
```

### 安装 Cartographer-Ros

cartographer_ros

https://google-cartographer-ros.readthedocs.io/en/latest/

```shell
$ sudo apt-get update
# >= noetic
$ sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
# melodic
$ sudo apt-get install -y python-wstool python-rosdep ninja-build stow
# abseil 冲突问题
$ sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
$ git clone https://ghproxy.com/https://github.com/abseil/abseil-cpp.git
$ cd abseil-cpp
$ mkdir build & cd build
$ cmake -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
-DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl

```

### 问题处理

#### 1. 20.04 缺少 wifi 适配器 & 18.04 缺少 wifi 适配器

lspci -v

sudo apt install git
sudo apt install build-essential
sudo apt install dkms
git clone https://ghproxy.com/https://github.com/tomaspinho/rtl8821ce.git
cd rtl8821ce

> Intel AX201 => iwlwifi

sudo apt install git linux-headers-generic build-essential
git clone https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git

#### 1. noetic 缺少 bfl

sudo apt -y install liborocos-bfl-dev

#### melodic 缺少 sdl

sudo apt-get install libsdl-image1.2-dev libsdl-dev

### 编译运行

```shell
catkin_make -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;cloud_msgs;automotive_msgs;path_msgs;smartcar_msgs;lb_cloud_msgs;rtk_cloud_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release
```

### QT 问题

```
add_library(runtime_control SHARED IMPORTED)
set_target_properties(runtime_control PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libruntime_control.so)

add_library(status_dashboard SHARED IMPORTED)
set_target_properties(status_dashboard PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libstatus_dashboard.so)
```

### wxPython

```
https://pypi.org/project/wxPython/4.0.7.post2/#files
```

### VM gazebo 启动参数问题

```
https://blog.csdn.net/coolwaterld/article/details/72467942
```

> gazebo 缺少模型库

git clone https://ghproxy.com/https://github.com/osrf/gazebo_models.git
https://github.com/osrf/gazebo_models.git

### LCM

```
sudo apt-get install build-essential autoconf automake autopoint libglib2.0-dev libtool openjdk-8-jdk python-dev
```

### NTP 时间同步

```
sudo apt-get install ntp
sudo apt-get install ntpstat
# 查看同步情况
ntpstat
ntpq -p

```

### opencv

```
pip install torch torchvision torchaudio -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install numpy
pip install matplotlib
pip install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install torch
pip install opencv-python --prefer-binary -i https://pypi.tuna.tsinghua.edu.cn/simple

pip install --upgrade pip setuptools wheel
```

```
sudo apt-get install -y \
ros-$ROS_DISTRO-automotive-navigation-msgs \
ros-$ROS_DISTRO-image-view2 \
ros-$ROS_DISTRO-uvc-camera \
ros-$ROS_DISTRO-velocity-controllers \
ros-$ROS_DISTRO-jsk-topic-tools \
ros-$ROS_DISTRO-effort-controllers \
ros-$ROS_DISTRO-gps-common \
ros-$ROS_DISTRO-jsk-rviz-plugins \
ros-$ROS_DISTRO-carla-msgs \
ros-$ROS_DISTRO-velodyne-description \
ros-$ROS_DISTRO-nmea-msgs \
ros-$ROS_DISTRO-jsk-recognition-msgs \
ros-$ROS_DISTRO-velodyne-pointcloud \
ros-$ROS_DISTRO-gscam \
ros-$ROS_DISTRO-lgsvl-msgs \
ros-$ROS_DISTRO-velodyne \
ros-$ROS_DISTRO-qpoases-vendor \
ros-$ROS_DISTRO-automotive-platform-msgs \
ros-$ROS_DISTRO-sound-play \
ros-$ROS_DISTRO-grid-map-ros \
ros-$ROS_DISTRO-geodesy \
ros-$ROS_DISTRO-jsk-topic-tools \
ros-$ROS_DISTRO-velodyne-gazebo-plugins \
ros-$ROS_DISTRO-rosbridge-server \
ros-$ROS_DISTRO-imu-tools \
ros-$ROS_DISTRO-nmea-navsat-driver \
libglew-dev \
ros-$ROS_DISTRO-lanelet2*
```

```
https://wiki.ros.org/ROS/Tutorials#ROS_Tutorials
```
