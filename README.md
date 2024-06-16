<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-05-27 10:01:42
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-06-05 10:48:07
 * @FilePath: /src/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

https://sub.wl-sub1.com/api/v1/client/subscribe?token=1ddb6feb800a114b7bdb3afc43373ddf

xx
yy

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

```
巡检机器人实验工程开发日志 htcbot_debug_ws

commit 51a791b5d6b86036a0f519f2070ab1fea7607c88 (HEAD -> dev, origin/dev_mapping, origin/dev, dev_mapping)
Author: code-fusheng <2561035977@qq.com>
Date:   Sun Jun 16 18:45:45 2024 +0800
feature : 新增建图 Pre 兼容UTM
更新内容:
- 新增离线建图功能模块
- 调整航迹点记录从原始GPS轨迹为UTM轨迹
```
