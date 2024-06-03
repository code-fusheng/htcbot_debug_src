<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-05-27 10:01:42
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-29 15:02:11
 * @FilePath: /src/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

https://sub.wl-sub1.com/api/v1/client/subscribe?token=1ddb6feb800a114b7bdb3afc43373ddf

xx

# htcbot_debug_src

catkin_make -DCATKIN_WHITELIST_PACKAGES="htcbot_msgs;can_msgs;mmware_msgs;rtk_cloud_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release

sudo apt-get install ros-$ROS_DISTRO-rosbridge-server

### MQTT

```
pip install paho-mqtt
pip install apscheduler
# apscheduler 高版本存在 python 2.7 用不了 datetime.datetime
pip install apscheduler==3.7.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

###5.29
一、laser_detector功能包：
1.加入强度滤波、半径滤波、离群滤波，默认值已调试好
2.输出话题加入距离、角度、高度三个指标,删除前后左右距离
3.增加最近点的点云话题以便后续继续调试

二、plane_fit_ground_filter功能包：
1.新增传感器高度标定功能，原理：原点到拟合地面的距离。
（文件结构还比较乱，该功能还没专门独立分出来）

###5.31
一、laser_detector功能包：
1.在选取检测区域时将车身侧面区域单独分离出来，做权重更大的离群滤波处理，再进入后续处理。（为了减小后续滤波的难度和耗时）
2.各滤波的默认阈值已调整好，跑bag包的时候已经无近点噪声，并且能较好地保留障碍物轮廓
二、plane_fit_ground_filter功能包：
1.调试了高度标定的参数，精度一般，需要在静止、空旷场景下进一步调试。

