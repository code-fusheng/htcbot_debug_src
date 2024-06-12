#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-04-29 23:10:26
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-14 12:45:46
Description: 
'''

import rospy
import os
import time
from dynamic_reconfigure.server import Server
from turn_htcbot.cfg import AutoStartConfig
from htcbot_msgs.msg import MapPathConf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest

class AutoStartNode:

    def __init__(self):
        rospy.init_node("auto_start_node", anonymous=True)
        self.is_debug = False
        self.server = Server(AutoStartConfig, self.dynamic_reconfigure_callback)
        self.laser_euclidean_cluster_client = rospy.ServiceProxy('/laser_euclidean_cluster_node/set_switch_status', SwitchStatusSrv)

    def run(self):
        self.init()
        time.sleep(5)
        self.auto_start_system()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            if self.is_debug:
                rospy.loginfo("[auto_start_node] ===> Running...")
                pass
            rate.sleep()

    def init(self):
        # init params

        # 发布场景配置话题
        self.parent_path_value = rospy.get_param("/htcbot/scenes/parent_path", "htcbot")
        self.child_path_value = rospy.get_param("/htcbot/scenes/child_path", "t_0")
        self.base_dir = "/home/data"
        self.map_static_str = ""
        self.map_dynamic_str = ""
        self.pathes_str = ""
        self.config_str = ""

        self.map_path_conf_pub = rospy.Publisher("htcbot/map_path_conf", MapPathConf, queue_size=10)
        self.init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)

    def auto_start_system(self):
        # pub screen setting
        rospy.loginfo("[auto_start_node] ===> Begin Auto Start System...")
        self.setting_and_pub_map_path_conf()
        # pub 位姿?
        time.sleep(5)
        self.setting_and_pub_init_current_pose()
        time.sleep(5)
        self.setting_and_call_up_perception()
        # pub user cmd 
        pass
    
    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config
    
    def setting_and_pub_map_path_conf(self):
        rospy.logwarn("[auto_start_node] ===> starting setting and pub map path conf")
        dir_str = os.path.join(self.base_dir, self.parent_path_value, self.child_path_value)
        for subdir in ["lidar_mode/grid_map/2d_map", "lidar_mode/pcd_map/static_map", "lidar_mode/pcd_map/dynamic_map", "lidar_mode/pathes", "lidar_mode/config"]:
            full_subdir_path = os.path.join(dir_str, subdir)
            if not os.path.exists(full_subdir_path):
                os.makedirs(full_subdir_path)
        # 设置各路径变量
        self.map_grid_str = os.path.join(dir_str, "lidar_mode/grid_map/2d_map")
        self.map_static_str = os.path.join(dir_str, "lidar_mode/pcd_map/static_map")
        self.map_dynamic_str = os.path.join(dir_str, "lidar_mode/pcd_map/dynamic_map")
        self.pathes_str = os.path.join(dir_str, "lidar_mode/pathes")
        self.config_str = os.path.join(dir_str, "lidar_mode/config")
        # 创建ROS消息并发布
        conf_msg = MapPathConf( # 这里应该是ROS消息类的实例化，但Python中不直接使用这种语法，需根据ROS实际库如rospy来调整
            map_grid_path = self.map_grid_str,
            map_static_path = self.map_static_str,
            map_dynamic_path = self.map_dynamic_str,
            route_path = self.pathes_str,
            conf_path = self.config_str
        )
        self.map_path_conf_pub.publish(conf_msg)

    def setting_and_pub_init_current_pose(self):
        rospy.logwarn("[auto_start_node] ===> auto starting setting and pub init current pose")
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "world"
        init_pose.header.stamp = rospy.Time().now()
        init_pose.pose.pose.position.x = float(0)
        init_pose.pose.pose.position.y = float(0)
        init_pose.pose.pose.position.z = float(0)
        init_pose.pose.pose.orientation.x = float(0)
        init_pose.pose.pose.orientation.y = float(0)
        init_pose.pose.pose.orientation.z = float(0)
        init_pose.pose.pose.orientation.w = float(1)
        self.init_pose_pub.publish(init_pose)

    def setting_and_call_up_perception(self):
        rospy.logwarn("[auto_start_node] ===> auto starting setting and call up perception")
        rospy.wait_for_service("/laser_euclidean_cluster_node/set_switch_status")
        try:
            request = SwitchStatusSrvRequest(switch_to=1)
            response = self.laser_euclidean_cluster_client(request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        rospy.loginfo("Service responded with: %s", response)

if __name__ == '__main__':
    try:
        node = AutoStartNode()
        node.run()
    except rospy.ROSInterruptException:
        pass