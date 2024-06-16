#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: code-fusheng
Date: 2024-05-10 00:58:00
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-05-14 13:48:31
Description: 
# 计划轨迹发布增设航迹点 Target Stations Point
'''

import rospy
import csv
import math
import os
import time
from enum import Enum

from dynamic_reconfigure.server import Server
from waypoint_tools.cfg import WaypointLoaderConfig
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse
from htcbot_msgs.msg import StatusHtcbotModule

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from htcbot_msgs.msg import Lane, Waypoint
from htcbot_msgs.msg import MapPathConf

class MODULE_TYPE(Enum):
    UNKNOWN = 0    
    MAP = 1
    LOCALIZER = 2
    SENSOR = 3
    WAYPOINT = 4
    PLANNER = 5
    CONTROL = 6

class STATUS_TYPE(Enum):
    NONE = 0                # 默认
    READY = 1               # 就绪
    WARN = 2                # 警告
    DANGER = 3              # 危险
    EXC = 4                 # 异常
    EXPIRED = 5             # 过期

class WaypointLoaderNode:

    def __init__(self):
        rospy.init_node("waypoint_loader_node", anonymous=False, log_level=rospy.DEBUG)
        self.is_debug = False
        self.switch_status = 0
        self.waypoint_dir = ""   # 轨迹文件路径
        self.current_pose_topic = ""
        self.waypoint_path = []
        self.current_pose = None
        self.f_map_path_set = False
        self.waypoint_status = StatusHtcbotModule()
        
        self.server = Server(WaypointLoaderConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)

    def run(self):
        self.init()
        self.rate = rospy.Rate(1.0/self.publish_time_interval)
        time.sleep(5)
        while not rospy.is_shutdown():  # 添加条件
            # Perform your main operations here
            # rospy.loginfo("[waypoint_loader_node] ===> Running...")
            self.publish_global_path()
            self.rate.sleep()

    def init(self):
        
        self.waypoint_status.module_type = int(MODULE_TYPE.WAYPOINT.value)
        self.waypoint_status.module_status = int(STATUS_TYPE.NONE.value)

        self.publish_time_interval = rospy.get_param("~publish_time_interval", default=10)
        self.current_pose_topic = rospy.get_param("~current_pose_topic", default="current_pose_truth")

        self.sub_current_pose = rospy.Subscriber(self.current_pose_topic, PoseStamped, self.callback_current_pose, queue_size=5)
        self.sub_map_path_conf = rospy.Subscriber("/htcbot/map_path_conf", MapPathConf, self.callback_map_path_conf, queue_size=5)

        self.pub_default_path = rospy.Publisher("/default_path", MarkerArray, queue_size=10)
        self.pub_global_path = rospy.Publisher("/global_path", Lane, queue_size=5)
        self.pub_global_path_rviz = rospy.Publisher("/global_path_rviz", Marker, queue_size=5)
        self.pub_waypoint_status = rospy.Publisher("/htcbot/module_status", StatusHtcbotModule, queue_size=5)

        pass  # Add any initialization steps here

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[waypoint_loader_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = switch_status
        return res
    
    def callback_map_path_conf(self, msg):
        rospy.loginfo("[waypoint_loader_node] ===> Recivced Msg Map Path Conf RoutePath:{}".format(msg.route_path))
        self.waypoint_dir = msg.route_path
        if not os.path.exists(self.waypoint_dir):
            rospy.logger("[waypoint_loader_node] ===> Waypoint Path Dir ({}) Not Exist!".format(self.waypoint_dir))
        files = os.listdir(self.waypoint_dir)
        for file_name in files:
            if file_name.startswith("lane"):
                self.load_waypoint(os.path.join(self.waypoint_dir, file_name)) 
        self.publish_default_path()

    def callback_current_pose(self, msg):
        self.current_pose = msg

    def load_waypoint(self, waypoint_file):
        with open(waypoint_file, 'r') as f:
            csv_reader = csv.reader(f)
            t = list(csv_reader)
            point_num = len(t)
            for i in range(5, point_num):
                _current_pose = t[i]
                p = Waypoint()
                p.pose.pose.position.x = float(_current_pose[0])
                p.pose.pose.position.y = float(_current_pose[1])
                p.pose.pose.position.z = float(_current_pose[2])
                p.pose.pose.orientation.x = float(_current_pose[3])
                p.pose.pose.orientation.y = float(_current_pose[4])
                p.pose.pose.orientation.z = float(_current_pose[5])
                p.pose.pose.orientation.w = float(_current_pose[6])
                self.waypoint_path.append(p)

    def publish_default_path(self):
        default_path = MarkerArray()
        if len(self.waypoint_path) != 0:
            _pre_pose = None
            for i in range(len(self.waypoint_path)):
                _pose = self.waypoint_path[i].pose.pose
                if _pre_pose is None or self.distance_of_two_point(_pre_pose.position, _pose.position) > 1.0:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time().now()
                    marker.ns = "default_path"
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.scale.x = 0.5
                    marker.scale.y = 0.3
                    marker.scale.z = 0.3
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0
                    marker.color.a = 1
                    marker.id = i
                    marker.pose = _pose
                    _pre_pose = _pose
                    default_path.markers.append(marker)
            self.pub_default_path.publish(default_path)

    def publish_global_path(self):
        msg_global_path = Lane()
        if len(self.waypoint_path) == 0:
            self.waypoint_status.module_status = int(STATUS_TYPE.EXC.value)
        else:
            if self.current_pose is not None:
                closet_index = self.find_closet_index()
                msg_global_path.waypoints.extend(self.waypoint_path[closet_index:-1])
                msg_global_path.waypoints.extend(self.waypoint_path[0:closet_index])
            else:
                msg_global_path.waypoints = self.waypoint_path
            msg_global_path.header.frame_id = "map"
            msg_global_path.header.stamp = rospy.Time().now()
            if len(msg_global_path.waypoints) > 1000:
                msg_global_path.waypoints = msg_global_path.waypoints[0:1000]  # 只截取前1000个点
            else:
                msg_global_path.waypoints = msg_global_path.waypoints[0:len(msg_global_path.waypoints)/2]
            self.pub_global_path.publish(msg_global_path)
            rospy.loginfo("[waypoint_loader_node] ===> Publish Global Path Success")
            self.publish_global_path_rviz(msg_global_path)
            self.waypoint_status.module_status = int(STATUS_TYPE.READY.value)
        self.pub_waypoint_status.publish(self.waypoint_status)

    def publish_global_path_rviz(self, msg_global_path):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time().now()
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = "global_path"
        marker.scale.x = 1.0
        marker.color.a = 0.7  # Alpha (透明度)
        marker.color.g = 1.0
        marker.points = [p.pose.pose.position for p in msg_global_path.waypoints]
        self.pub_global_path_rviz.publish(marker)

    def find_closet_index(self):
        min_dist = 99999999
        closet_index = 0
        for i in range(len(self.waypoint_path)):
            dist = self.distance_of_two_point(self.current_pose.pose.position, self.waypoint_path[i].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                closet_index = i
        return closet_index

    def distance_of_two_point(self, p1, p2):
        return math.sqrt(math.pow(p2.x - p1.x, 2) + math.pow(p2.y - p1.y, 2))

if __name__ == '__main__':
    try:
        node = WaypointLoaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass