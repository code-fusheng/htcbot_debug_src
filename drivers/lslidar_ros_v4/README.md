# LSLIDAR_CX_V4.2.4_230705_ROS使用说明

## 1.工程介绍
​		LSLIDAR_CX_V4.2.3_230705_ROS为linux环境下雷达ros驱动，适用于C1, C1Plus, C8,C16,C32(32度，70度和90度)  4.0版本雷达，程序在ubuntu 20.04 ros noetic，ubuntu18.04 ros melodic以及ubuntu16.04 ros kinetic下测试通过。

## 2.依赖

1.ubuntu20.04 ros noetic/ubuntu18.04 ros melodic/ubuntu16.04 ros kinetic

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
~~~



## 3.编译运行

### 3.1 编译

~~~bash
mkdir -p ~/lidar_ws/src
#将驱动压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
# 单线雷达C1/C1Plus
roslaunch lslidar_driver lslidar_c1.launch
# 单线雷达C8
roslaunch lslidar_driver lslidar_c8.launch
# 单线雷达C16
roslaunch lslidar_driver lslidar_c16.launch
# 单线雷达C32
roslaunch lslidar_driver lslidar_c32.launch
~~~

运行多个雷达：

~~~bash
roslaunch lslidar_driver lslidar_double.launch
~~~

## 4.参数介绍

lslidar_c32.launch文件内容如下，每个参数含义见注释说明。

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />           #雷达ip
  <arg name="msop_port" default="2368"/>                     #数据包端口
  <arg name="difop_port" default="2369"/>                    # 设备包端口
  <arg name="use_gps_ts" default="false" />                  #是否gps时间同步 
  <arg name="pcl_type" default="false" />                    # pcl点云类型
  <arg name="lidar_type" default="c32"/>                     #雷达类型  
  <arg name="c32_type" default="c32_32"/>                    #<!--c32_32: 垂直角度是的30度c32   c32_70: 垂直角度是的70度c32  c32_90: 垂直角度是的90度c32/-->
  <arg name="packet_rate" default="1695.0"/>                 #雷达每秒钟发送的数据包的个数，此参数在读取pcap包的时候有用

  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/123.pcap" /-->   #pcap包路径，加载pcap包时打开此注释
    <param name="packet_rate" value="$(arg packet_rate)"/>           
    <param name="device_ip" value="$(arg device_ip)" />
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    <param name="lidar_type" value="$(arg lidar_type)"/>     #雷达类型  单线:C1 8线：C8， 16线:C16, 32线: C32
    <param name="c32_type" value="$(arg c32_type)"/>         # 32线雷达垂直角度 c32_32: 垂直角度是的30度c32   c32_70: 垂直角度是的70度c32  c32_90: 垂直角度是的90度c32     
    <param name="add_multicast" value="false"/>               #是否添加组播
    <param name="group_ip" value="224.1.1.2"/>                #组播的ip
    <param name="use_gps_ts" value="$(arg use_gps_ts)"/>       
    <param name="min_range" value="0.15"/>                    # 雷达的最小测量距离
    <param name="max_range" value="150.0"/>                   # 雷达的最大测量距离
    <param name="frame_id" value="laser_link"/>               # 雷达点云坐标系
    <param name="distance_unit" value="0.40"/>                #雷达距离分辨率
    <param name="angle_disable_min" value="0"/>               #雷达裁剪角度开始值 ，单位0.01°
    <param name="angle_disable_max" value="0"/>               #雷达裁剪角度结束值，单位0.01°
    <param name="horizontal_angle_resolution" value="0.18"/>   # 水平角度分辨率 10Hz:0.18  20Hz:0.36 5Hz: 0.09 
    <param name="scan_num" value="10"/>                       #laserscan线号
    <param name="read_once" value="false"/>                   #是否重复播放pcap包，  false: 重复播放  true:只播放一次
    <param name="publish_scan" value="false"/>                #是否发布scan
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  # 发布点云话题的名称
    <param name="coordinate_opt" value="false"/>              #默认false  雷达零度角对应点云方向
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_driver)/rviz_cfg/lslidar.rviz"/>
  
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100" /-->
</launch>
~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  <param name="add_multicast" value="false"/>               #是否添加组播
  <param name="group_ip" value="224.1.1.2"/>                #组播的ip
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 把录制好的pcap文件，拷贝到cx_4.0_ws/src/lslidar_ros/lslidar_driver/pcap文件夹下。（cx_4.0_ws是ros工作空间,根据实际工作空间修改）

- 修改launch文件以下参数

  ~~~shell
  #取消注释
      <param name="pcap" value="$(find lslidar_driver)/pcap/123.pcap" />   #pcap包路径，加载pcap包时打开此注释
  ~~~

###  pcl点云类型：

- 修改launch文件以下参数

  ~~~shell
  <param name="pcl_type" value="$(arg pcl_type)"/>         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~



### 修改雷达授时方式：

source devel/setup.bash

GPS授时：

~~~bash
rosservice call /time_service "time_service_mode: 'gps'
ntp_ip: ''" 
~~~

PTP授时：

~~~bash
rosservice call /time_service "time_service_mode: 'ptp'
ntp_ip: ''" 
~~~

NTP授时：

~~~bash
rosservice call /time_service "time_service_mode: 'ntp'
ntp_ip: '192.168.1.102'" 
~~~



### 雷达上下电(雷达依然转动，只发设备包，不发送数据包)：

source devel/setup.bash

上电：

~~~bash
rosservice call /lslidar_control "laser_control: 1"
~~~

下电：

~~~bash
rosservice call /lslidar_control "laser_control: 0"
~~~



### 雷达转动/停止转动(电机停转)：

source devel/setup.bash

转动：

~~~bash
rosservice call /motor_control "motor_control: 1"
~~~

停止转动：

~~~bash
rosservice call /motor_control "motor_control: 0"
~~~



### 设置雷达转速：

source devel/setup.bash

可选频率  5Hz/10Hz/20Hz

~~~bash
rosservice call /set_motor_speed "motor_speed: 20"
~~~



### 设置雷达数据包端口

source devel/setup.bash

~~~bash
rosservice call /set_data_port "data_port: 2368"  #范围[1025,65535]
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达设备包端口

source devel/setup.bash

~~~bash
rosservice call /set_dev_port "dev_port: 2369"  #范围[1025,65535]
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达ip

source devel/setup.bash

~~~bash
rosservice call /set_data_ip "data_ip: '192.168.1.200'"
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达目的ip

source devel/setup.bash

~~~bash
rosservice call /set_destination_ip "destination_ip: '192.168.1.102'"
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.1.4_220425_ROS

Modify:  original version

Date    : 2022-04-25

--------------------------------------------------------------------

Original version : LSLIDAR_CX_V4.1.5_220620_ROS

Modify:  c8雷达，光学垂直角度修改。

Date    : 2022-06-20

------------------------------------------------------------

Original version : LSLIDAR_CX_V4.2.0_221028_ROS

Modify:  1.新增对4.0版本单线雷达， C32 70度和90度雷达的支持

2.统一laserscan和pointcloud2坐标系

Date    : 2022-10-28

------------------

Original version : LSLIDAR_CX_V4.2.1_221227_ROS

Modify:  1.scan话题新增强度信息

2.fpga升级，C32 90度修改计算公式

3.ROS驱动新增修改授时方式的功能

4.新增雷达上下电,修改雷达ip，端口，转速等功能。

5.修复ntp授时解析问题。

Date    : 2022-12-27

-----

Original version : LSLIDAR_CX_V4.2.2_230322_ROS

Modify:  1.增加使用时长提示

2.新增驱动版本提示

Date    : 2023-03-22

------------

Original version : LSLIDAR_CX_V4.2.3_230403_ROS

Modify:  1.fpga变更，修改C32W的计算公式

Date    : 2023-04-03

------

Original version : LSLIDAR_CX_V4.2.4_230705_ROS

Modify:  1.修复雷达切换成低功耗模式后，不能切换成正常模式的问题

​               2.兼任C1Plus 型号雷达

Date    : 2023-07-05







