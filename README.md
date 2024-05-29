<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-05-27 10:01:42
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-29 10:05:59
 * @FilePath: /src/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

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
