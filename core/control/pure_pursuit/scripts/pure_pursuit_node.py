#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import math
import os
import time

from dynamic_reconfigure.server import Server
from pure_pursuit.cfg import PurePursuitConfig
from geometry_msgs.msg import PoseStamped, Twist
from can_msgs.msg import ecu
from htcbot_msgs.msg import Lane, Waypoint
from htcbot_msgs.srv import SwitchStatusSrv, SwitchStatusSrvResponse, SwitchStatusSrvRequest

class PurePursuit:

    def __init__(self):
        rospy.init_node("pure_pursuit_node", anonymous=False, log_level=rospy.INFO)
        self.is_debug = False
        self.switch_status = 1

        # 跟踪模块输入
        self.final_lane_topic = ""  # 轨迹话题名称
        self.waypoints = []         # 当前轨迹点
        self.current_pose = None    # 当前位姿
        # 跟踪算法参数
        self.lookahead_distance = 5.0       # 预喵距离
        self.end_waypoint_threshold = 2     # 定义临近终点的航迹点数
        
        # 
        self.vehicle_info = None    # 车辆信息
        self.wheel_base = 0.5       # 前后轮轴距

        self.lookahead_index = None

        self.server = Server(PurePursuitConfig, self.dynamic_reconfigure_callback)
        self.srv_switch_status = rospy.Service("~set_switch_status", SwitchStatusSrv, self.switch_status_callback)

    def run(self):
        self.init()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():  # 添加条件
            if self.switch_status:
                # rospy.loginfo("[pure_pursuit_node] ===> Running...")
                if self.current_pose is None or len(self.waypoints) == 0:
                    rospy.logwarn("[pure_pursuit_node] ===> current_pose is None and waypoints:{}".format(len(self.waypoints)))
                    self.publish_brake_cmd()
                else:
                    closest_index = self.find_closest_waypoint()
                    if closest_index == -1:
                        self.publish_brake_cmd()
                        continue
                    lookahead_waypoint = self.find_lookahead_waypoint(closest_index)
                    # 计算曲率
                    curvature = self.calcu_curvature(lookahead_waypoint)
                    # 构建控制指令
                    self.publish_control_cmd(curvature)
            rate.sleep()

    def init(self):
        self.lookahead_distance = rospy.get_param("~lookahead_distance", default=5.0)
        self.final_lane_topic = rospy.get_param("~final_lane_topic", default="global_path")
        # sub
        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.callback_current_pose)
        self.sub_lane = rospy.Subscriber(self.final_lane_topic, Lane, self.callback_lane)
        # pub
        self.pub_pp_ecu = rospy.Publisher("/pure_pursuit/ecu", ecu, queue_size=10)
        self.pub_pp_cmd_vel = rospy.Publisher("/pure_pursuit/cmd_vel", Twist, queue_size=10)
        pass  # Add any initialization steps here

    def dynamic_reconfigure_callback(self, config, level):
        self.is_debug = config.is_debug
        return config

    def switch_status_callback(self, req):
        rospy.loginfo("[pure_pursuit_node] ===> srv switch status callback req.switch_to: %d", req.switch_to)
        self.switch_status = req.switch_to
        res = SwitchStatusSrvResponse()
        res.switch_status = self.switch_status
        return res
    
    def callback_current_pose(self, msg):
        self.current_pose = msg
    
    def callback_lane(self, msg):
        # TODO? 优化处理轨迹
        self.waypoints = msg.waypoints

    # 通过当前位置查找最近航迹点
    def find_closest_waypoint(self):
        closest_index = -1
        min_distance = float('inf')
        for i, waypoint in enumerate(self.waypoints):
            distance = self.calcu_distance_tow_pose(self.current_pose.pose.position, waypoint.pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        # 判断是否临近终点
        if len(self.waypoints) - closest_index <= self.end_waypoint_threshold:
            closest_index = -1  # 临近终点时，将索引设置为-1
        return closest_index 
    
    # 通过最近轨迹点结合预喵距离查询预喵点
    def find_lookahead_waypoint(self, closest_index):
        for i in range(closest_index, len(self.waypoints)):
            distance = self.calcu_distance_tow_pose(self.current_pose.pose.position, self.waypoints[i].pose.pose.position)
            if distance > self.lookahead_distance:
                self.lookahead_index = i
                return self.waypoints[i]
        # TODO 临近终点
        self.lookahead_index = len(self.waypoints)
        return self.waypoints[-1]
    
    # 计算曲率
    def calcu_curvature(self, lookahead_waypoint):
        if lookahead_waypoint == None:
            return None
        lookahead_point = lookahead_waypoint.pose.pose.position
        car_position = self.current_pose.pose.position
        car_orientation = self.current_pose.pose.orientation
        # 计算预喵点相对于车辆的坐标
        dx = lookahead_point.x - car_position.x
        dy = lookahead_point.y - car_position.y

        yaw = 2 * math.atan2(car_orientation.z, car_orientation.w)
        lookahead_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        lookahead_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # 曲率
        denominator = lookahead_x**2 + lookahead_y**2
        numerator = 2 * lookahead_y
        if denominator != 0:
            curvature = numerator / denominator
        else:
            curvature = 0.0
        return curvature

    def publish_control_cmd(self, curvature):
        ecu_ctl = ecu()
        ecu_ctl.header.stamp = rospy.Time.now()
        steer = math.atan(self.wheel_base * curvature) if curvature else 0.0
        # 弧度转角度
        steer = steer * 180.0 /math.pi
        # 
        ecu_ctl.steer = steer if curvature else 0.0
        # ecu_ctl.shift = ecu.SHIFT_R 倒车
        ecu_ctl.shift = ecu.SHIFT_D
        # 判断是否刹车
        ecu_ctl.brake = False
        ecu_ctl.motor = self.computure_cmd_velocity() if curvature else 0.0
        
        self.pub_pp_ecu.publish(ecu_ctl)

    def publish_brake_cmd(self):
        ecu_ctl = ecu()
        ecu_ctl.header.stamp = rospy.Time.now()
        ecu_ctl.brake = True
        ecu_ctl.shift = ecu.SHIFT_UNKNOWN
        ecu_ctl.motor = 0.0
        ecu_ctl.steer = 0.0
        self.pub_pp_ecu.publish(ecu_ctl)

    def computure_cmd_velocity(self):
        const_velocity = 2
        # TODO 完善不同场景速度控制
        return const_velocity

    def calcu_distance_tow_pose(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

if __name__ == '__main__':
    try:
        node = PurePursuit()
        node.run()
    except rospy.ROSInterruptException:
        pass