#!/usr/bin/env python
#-*-coding:utf-8-*-

'''
Author: code-fusheng
Date: 2024-04-22 12:49:17
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-06-12 11:28:13
Description: 
pip install pyproj
'''

import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Transformer
from htcbot_msgs.msg import ModeSwitch, GnssModeSwitch, MapPathConf, MappingConf
import math
from math import radians, sin, cos, sqrt, atan2
import csv
import os
import time

class Transfix:

    def __init__(self):

        rospy.init_node("trans_fix_node")
        self.mode_switch = False
        self.transformer = None
        # 39.133853256 , 117.357401963
        # 武汉市 30.593099,     114.305392
        # 30.769732, Longitude: 114.205022

        self.base_lat = None
        self.base_lon = None
        self.utm_offset_x = 0
        self.utm_offset_y = 0

        self.orientation_ready = False
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32650") 
        self.is_first = True

        self.pub_gps = rospy.get_param("~pub_gps", default=True)
        self.pub_utm = rospy.get_param("~pub_utm", default=False)
        self.pub_enu = rospy.get_param("~pub_enu", default=False)

        rospy.Subscriber('/fix', NavSatFix, self.fix_callback, queue_size=10)
        rospy.Subscriber("/htcbot/gnss_mode_switch", GnssModeSwitch, self.switch_callback)
        rospy.Subscriber("/htcbot/mapping_conf", MappingConf, self.conf_mapping_callback)
        rospy.Subscriber("/htcbot/map_path_conf", MapPathConf, self.conf_path_callback)

        self.config_file = ""
        self.route_path = ""
        self.lane_path_points = []
        self.gps_path_points = []
        self.utm_path_points = []
        self.is_load_path_points = False

        self.prev_utm_yaw = 0

        self.prev_utm_pose = None
        self.utm_pose = None
        self.utm_pose_pub = rospy.Publisher('/gnss/utm_pose', PoseStamped, queue_size=5)
        self.utm_pose_path = Path()
        self.utm_pose_path_pub = rospy.Publisher('gnss/utm_path', Path, queue_size=5)

        self.prev_gps_pose = None
        self.gps_pose = None
        self.gps_pose_pub = rospy.Publisher('/gnss/gps_pose', PoseStamped, queue_size=5)
        self.prev_enu_pose = None
        self.enu_pose = None
        self.enu_pose_pub = rospy.Publisher('/gnss/enu_pose', PoseStamped, queue_size=5)
        
        self.current_pose_fix = None
        self.current_pose_fix_pub = rospy.Publisher('/gnss/current_pose/fix', PoseStamped, queue_size=1)
        self.current_pose_fix_marker_pub = rospy.Publisher('gnss_fix_pose_marker', Marker, queue_size=10)
        self.current_pose_calcu_marker_pub =rospy.Publisher("gnss_calcu_pose_marker", Marker, queue_size=10)

        self.br = tf2_ros.TransformBroadcaster()

    def run(self):
        # 地理坐标系 https://developers.arcgis.com/javascript/3/jshelp/gcs.htm
        # 投影坐标系 https://developers.arcgis.com/javascript/3/jshelp/pcs.htm
        # 天津市 WGS_1984_UTM_Zone_50N 32650
        rospy.spin()

    def switch_callback(self, switch_msgs):
        self.mode_switch = switch_msgs.switch_to

    def conf_mapping_callback(self, msgs):
        if msgs.mapping_state == 1:
            self.mode_switch = True
        else: 
            self.mode_switch = False

    def conf_path_callback(self, conf_msgs):
        self.config_file = conf_msgs.conf_path + "/base_point.txt"
        self.route_path = conf_msgs.route_path
        self.lane_path_points, self.gps_path_points, self.utm_path_points = self.load_route_path(self.route_path)
        print("base size: , gps size: ", len(self.lane_path_points), len(self.gps_path_points))
        if len(self.lane_path_points) > 0:
            self.is_load_path_points = True
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as file:
                content = file.read()
                self.base_lat, self.base_lon = map(float, content.split(','))
                self.utm_offset_x, self.utm_offset_y = self.transformer.transform(self.base_lat, self.base_lon)

    def fix_callback(self, fix_msg):
        if self.pub_gps:
            self.trans_fix_2_gps(fix_msg)
        if self.mode_switch is False:
            return
        lat = fix_msg.latitude
        lon = fix_msg.longitude
        if math.isnan(lat) or math.isnan(lon):
            return
        if self.base_lat == None or self.base_lat == None:
            self.base_lat = lat
            self.base_lon = lon
            with open(self.config_file, 'w') as file:
                file.write(self.base_lat+","+self.base_lon)
        if self.pub_utm:
            self.trans_fix_2_utm(fix_msg)
        if self.pub_enu:
            self.trans_fix_2_enu(fix_msg)

    def trans_fix_2_gps(self, fix_msg):
        self.gps_pose = PoseStamped()
        self.gps_pose.header.frame_id = "world"
        self.gps_pose.header.stamp = fix_msg.header.stamp
        self.gps_pose.pose.position.x = fix_msg.latitude
        self.gps_pose.pose.position.y = fix_msg.longitude
        self.gps_pose.pose.position.z = fix_msg.altitude
        if self.prev_gps_pose:
            pass
        (q_x, q_y, q_z, q_w) = quaternion_from_euler(0, 0, 0)
        quaternion_msg = Quaternion(q_x, q_y, q_z, q_w) 
        self.gps_pose.pose.orientation = quaternion_msg
        self.gps_pose_pub.publish(self.gps_pose)
        self.prev_gps_pose = self.gps_pose

        # 是否开启gps位姿辅助修正
        if self.is_load_path_points:
            nearest_index, nearest_point, min_distance = self.search_nearest_point(fix_msg.latitude, fix_msg.longitude, self.gps_path_points)
            # print("[tran_fix] ===> search_nearest_point index: x: y: distance: ", nearest_index, nearest_point[0], nearest_point[1] , min_distance)
            cur_x, cur_y = self.calu_pose_for_gps(fix_msg.latitude, fix_msg.longitude, self.gps_path_points, self.lane_path_points)
            # print("[tran_fix] ===> calu_pose_for_gps: cur_x: cur_y:", cur_x, cur_y)
            if nearest_index is not None:
                pose_fix = self.lane_path_points[nearest_index]
                # print("======> pose_fix: ", pose_fix)
                self.current_pose_fix = PoseStamped()
                self.current_pose_fix.header.frame_id = "world"
                self.current_pose_fix.header.stamp = fix_msg.header.stamp
                self.current_pose_fix.pose.position.x = float(pose_fix[0])
                self.current_pose_fix.pose.position.y = float(pose_fix[1])
                self.current_pose_fix.pose.position.z = float(pose_fix[2])
                q = Quaternion(float(pose_fix[3]), float(pose_fix[4]), float(pose_fix[5]), float(pose_fix[6]))  
                self.current_pose_fix.pose.orientation = q
                self.current_pose_fix_pub.publish(self.current_pose_fix)

                marker = Marker()
                marker.header.frame_id = "world"  # 使用与路径相同的坐标系
                marker.header.stamp = rospy.Time.now()
                marker.ns = "gnss_fix_marker"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(pose_fix[0])
                marker.pose.position.y = float(pose_fix[1])
                marker.pose.position.z = float(pose_fix[2])
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0  # 设置marker的大小
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.0  # 设置marker的颜色（这里是红色）
                marker.color.g = 1.0
                marker.color.b = 0.0
                self.current_pose_fix_marker_pub.publish(marker)

                marker.pose.position.x = float(cur_x)
                marker.pose.position.y = float(cur_y)
                marker.pose.position.z = float(pose_fix[2])
                marker.scale.x = 1.0  # 设置marker的大小
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 1.0  # 设置marker的颜色（这里是红色）
                marker.color.g = 1.0
                marker.color.b = 0.0
                self.current_pose_calcu_marker_pub.publish(marker)
                
    def trans_fix_2_utm(self, fix_msg):
        lat = fix_msg.latitude
        lon = fix_msg.longitude
        x, y = self.transformer.transform(lat, lon)
        if x == "nan" or y == "nan":
            return
        if self.utm_offset_x == 0 or self.utm_offset_y == 0:
            self.utm_offset_x = x
            self.utm_offset_y = y
        x -= self.utm_offset_x
        y -= self.utm_offset_y
        rospy.loginfo("(Latitude: %f, Longitude: %f) => (UTM X: %f, UTM Y: %f)", lat, lon, x, y)
        # 将UTM坐标发布到路径消息
        self.utm_pose = PoseStamped()
        self.utm_pose.header.frame_id = "world"
        self.utm_pose.header.stamp = fix_msg.header.stamp
        self.utm_pose.pose.position.x = x
        self.utm_pose.pose.position.y = y
        self.utm_pose.pose.position.z = 0

        if self.is_first:
            self.prev_utm_pose = self.utm_pose
            self.is_first = False
            pass

        # 计算当前位置与上一位置之间的距离，用于判断是否有位置更新
        distance = math.sqrt(pow(self.utm_pose.pose.position.y - self.prev_utm_pose.pose.position.y, 2) +
                                pow(self.utm_pose.pose.position.x - self.prev_utm_pose.pose.position.x, 2))
            
        if distance > 0.2:
            yaw = math.atan2(self.utm_pose.pose.position.y - self.prev_utm_pose.pose.position.y, self.utm_pose.pose.position.x - self.prev_utm_pose.pose.position.x)
            # print("gps trans yaw :", yaw)
            yaw_degrees = math.degrees(yaw)
            # print("gps trans yaw_degrees: ", yaw_degrees)
            (q_x, q_y, q_z, q_w) = quaternion_from_euler(0, 0, yaw)
            quaternion_msg = Quaternion(q_x, q_y, q_z, q_w)  
            self.orientation_ready = True
        else:
            self.orientation_ready = False

        if self.orientation_ready:
            self.utm_pose.pose.orientation = quaternion_msg
            self.prev_utm_yaw = yaw
        else:
            (q_x, q_y, q_z, q_w) = quaternion_from_euler(0, 0, self.prev_utm_yaw)
            quaternion_msg = Quaternion(q_x, q_y, q_z, q_w)  
            self.utm_pose.pose.orientation = quaternion_msg
        self.prev_utm_pose = self.utm_pose
        self.utm_pose_pub.publish(self.utm_pose)

        # 将 utm 添加直 path
        if True:
            self.utm_pose_path.header.frame_id = "world"
            self.utm_pose_path.header.stamp = rospy.Time.now()
            self.utm_pose_path.poses.append(self.utm_pose)
            self.utm_pose_path_pub.publish(self.utm_pose_path)

        # angle_rad = np.arctan2(x, y)
        # angle_deg = np.degrees(angle_rad)

    def trans_fix_2_enu(self, fix_msg):
        lat = fix_msg.latitude
        lon = fix_msg.longitude
        alt = fix_msg.altitude
        if self.base_lat is None or self.base_lon is None:
            rospy.logwarn("Base latitude or longitude is not set. Skipping ENU calculation.")
            return
        x_base, y_base = self.transformer.transform(self.base_lat, self.base_lon)
        x, y = self.transformer.transform(lat, lon)
        if x == "nan" or y == "nan":
            rospy.logwarn("Invalid UTM coordinates. Skipping ENU calculation.")
            return
        delta_x = x - x_base
        delta_y = y - y_base
        enu_yaw = math.atan2(delta_y, delta_x)
        enu_x = delta_y * cos(enu_yaw) + delta_x * sin(enu_yaw)  # 使用 delta_y
        enu_y = -delta_y * sin(enu_yaw) + delta_x * cos(enu_yaw)  # 使用 delta_x
        enu_z = alt  # Assuming altitude is already in the correct reference frame
        rospy.loginfo("(Latitude: %f, Longitude: %f) => (ENU X: %f, UTM Y: %f)", lat, lon, enu_x, enu_y)
        enu_pose = PoseStamped()
        enu_pose.header.frame_id = "gps"
        enu_pose.header.stamp = fix_msg.header.stamp
        enu_pose.pose.position.x = enu_x
        enu_pose.pose.position.y = enu_y
        enu_pose.pose.position.z = enu_z
        enu_pose.pose.orientation = self.prev_utm_pose.pose.orientation

        self.enu_pose_pub.publish(enu_pose)

    def load_route_path(self, path):
        lane_points = []
        gps_points = []
        utm_points = []
        files = os.listdir(path)
        print(len(files))
        if len(files) == 0:
            self.is_load_path_points = False
            return lane_points, gps_points, utm_points
        for file_name in files:
            file_path = os.path.join(self.route_path, file_name)
            with open(file_path, 'r') as f:
                points = []
                csv_reader = csv.reader(f)
                rows = list(csv_reader)
                row_num = len(rows)
                # print("len:", row_num)
                for i in range(5, row_num):
                    point = rows[i]
                    points.append(point)
            if file_name.startswith("lane"):
                lane_points.extend(points)
            elif file_name.startswith("gps"):
                gps_points.extend(points)
            else:
                pass
        return lane_points, gps_points, utm_points

    # 通过当前GPS 估算 当前 x y 坐标
    def calu_pose_for_gps(self, cur_lat, cur_lon, gps_points, base_points):
        min_distance = float('inf')
        nearest_point = None
        nearest_index = None
        prev_nearest_point = None
        prev_nearest_index = None

        nearest_base_points = None
        prev_nearest_base_points = None

        # 先找到最近的点
        for i, point in enumerate(gps_points):
            # print("===> point:", point)
            distance = self.distance_of_two_point_4_gps(cur_lat, cur_lon, point[0], point[1])
            if distance < min_distance:
                min_distance = distance
                nearest_point = point
                nearest_index = i
        # 找到上一个GPS点
        if (nearest_index != 0):
            prev_nearest_point = gps_points[i-1]
            prev_nearest_index = i - 1
        else:
            prev_nearest_point = gps_points[i]
            prev_nearest_index = i

        # 找本地轨迹点
        nearest_base_points = base_points[nearest_index]
        prev_nearest_base_points = base_points[prev_nearest_index]
        

        # 转换 GPS 到 UTM
        utm_x1, utm_y1 = self.transformer.transform(prev_nearest_point[0], prev_nearest_point[1])
        utm_x2, utm_y2 = self.transformer.transform(nearest_point[0], nearest_point[1])

        x1 = prev_nearest_base_points[0]
        y1 = prev_nearest_base_points[1]
        x2 = nearest_base_points[0]
        y2 = nearest_base_points[1]

        # 已知 cur_lat, cur_lon
        cur_utm_x, cur_utm_y = self.transformer.transform(cur_lat, cur_lon)
        # 计算 cur_x cur_y
        # 计算当前GPS点在直角坐标系中的坐标
        t = self.calculate_t(cur_utm_x, cur_utm_y, utm_x1, utm_y1, utm_x2, utm_y2)
        cur_x = self.interpolate(prev_nearest_base_points[0], nearest_base_points[0], t)
        cur_y = self.interpolate(prev_nearest_base_points[1], nearest_base_points[1], t)
        return cur_x, cur_y

    def calculate_t(self, x, y, x1, y1, x2, y2):
        x, y, x1, y1, x2, y2 = float(x), float(y), float(x1), float(y1), float(x2), float(y2)
        # 计算线段的长度
        length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        # 计算当前点到第一个点的距离比例
        t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / (length ** 2)
        return t

    def interpolate(self, x1, x2, t):
        x1, x2 = float(x1), float(x2)
        # 线性插值公式
        return x1 + (x2 - x1) * t

    def search_nearest_point(self, p_lat, p_lon, points):
        start_time = time.time()
        min_distance = float('inf')
        nearest_point = None
        nearest_index = None

        for i, point in enumerate(points):
            # print("===> point:", point)
            distance = self.distance_of_two_point_4_gps(p_lat, p_lon, point[0], point[1])
            if distance < min_distance:
                min_distance = distance
                nearest_point = point
                nearest_index = i
        end_time = time.time()
        elapsed_time = end_time - start_time
        # print("ms:", elapsed_time, "s")
        # print("index: x: y: distance: ", nearest_index, nearest_point[0], nearest_point[1] , min_distance)
        return nearest_index, nearest_point, min_distance

    def distance_of_two_point_4_gps(self, lat1, lon1, lat2, lon2):
        # print("lat1:  lon1: lat2: lon2:", lat1, lon1, lat2, lon2)
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(float(lat2))
        lon2 = radians(float(lon2))
        # 地球半径（单位：米）
        R = 6371000.0
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c
        return distance

if __name__ == '__main__':
    app = Transfix()
    app.run()