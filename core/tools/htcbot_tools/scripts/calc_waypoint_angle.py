#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pip3 install * -i https://pypi.tuna.tsinghua.edu.cn/simple

import numpy as np
import pandas as pd

class CalcWaypointAngle:

    def __init__(self) -> None:
        self.source_lane_file_path = "/home/data/test/t1/lidar_mode/pathes/lane_1.csv"
        self.target_lane_file_path = "/home/data/test/t1/lidar_mode/pathes/utm_lane_1.csv"
        self.sample_interval = 10
        self.start_lane = 10
        self.end_lane = 2000

    def run(self):
        # 加载轨迹
        source_vectors = self.read_waypoint_vectors(self.source_lane_file_path, self.start_lane, self.end_lane, self.sample_interval)
        target_vectors = self.read_waypoint_vectors(self.target_lane_file_path, self.start_lane, self.end_lane, self.sample_interval)
        rotation_matrix, average_angle = self.calc_average_rotation_angle(source_vectors, target_vectors)
        print("Rotation Matrix:")
        print(rotation_matrix)
        print("Average Rotation Angle: {:.2f} degrees".format(average_angle))

    def read_waypoint_vectors(self, file_path, start_line, end_line, sample_interval):
        # 读取 CSV 文件并从 start_index 开始
        df = pd.read_csv(file_path, header=None, skiprows=start_line)
        total_lines = df.shape[0]
        print("calc_waypoint_angle ===> file:{} --- lanes:{}".format(file_path, total_lines))
        if end_line > (total_lines - start_line + 1):
            # raise ValueError(f"File Lane Is Less Than End Line!")
            end_line = total_lines - start_line + 1
            print("calc_waypoint_angle ===> change end_line to:{}".format(end_line))
        # 选取指定范围内的行并且按照间隔进行采样
        df = df.iloc[:end_line - start_line + 1:sample_interval]
        vectors = df[[0, 1]].values
        return vectors
    
    def calc_average_rotation_angle(self, source_vectors, target_vectors):
        if source_vectors.shape != target_vectors.shape:
            raise ValueError("The Size Of Tow Vector List Does Not Match.")
        total_angle = 0.0
        total_radians = 0.0
        total_weight = 0.0
        count = 0

        for i in range(len(source_vectors)):
            source_vector = source_vectors[i]
            target_vector = target_vectors[i]

            norm_source_vector = source_vector / np.linalg.norm(source_vector)
            norm_target_vector = target_vector / np.linalg.norm(target_vector)

            dot_project = np.dot(norm_source_vector, norm_target_vector)
            cos_angle = np.clip(dot_project, -1.0, 1.0)
            angle_radians = np.arccos(cos_angle)
            angle_degrees = np.degrees(angle_radians)

            tenth_percent = int(len(source_vector) * 0.15)
            weight = 0.8 if i < tenth_percent else 1.0

            total_angle += angle_degrees * weight
            total_radians += angle_radians * weight
            total_weight += weight

            count += 1
        
        average_angle = (total_angle / total_weight) if total_weight > 0 else 0.0
        average_radians = (total_radians / total_weight) if total_weight > 0 else 0.0

        rotation_matrix = np.array([
            [np.cos(average_radians), -np.sin(average_radians)],
            [np.sin(average_radians), np.cos(average_radians)]
        ])

        return rotation_matrix, average_angle


if __name__ == '__main__':
    app = CalcWaypointAngle()
    app.run()