#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pip3 install open3d -i https://pypi.tuna.tsinghua.edu.cn/simple

import os
import time
import open3d as o3d
import numpy as np

class RotationPcdMap:

    def __init__(self) -> None:
        self.file_path = "/home/data/test/t1/lidar_mode/pcd_map/static_map/origin.pcd"
        self.output_file_path = "/home/data/test/t1/lidar_mode/pcd_map/static_map/origin_trans.pcd"
        # 37.8536
        self.rotation_angle_degrees = -40.26
        pass

    def run(self):
        # 加载 pcd 文件
        original_pcd = o3d.io.read_point_cloud(self.file_path)
        # 复制原始点云以进行变换
        transformed_pcd = original_pcd.voxel_down_sample(voxel_size=0.02).translate((0, 0, 0))
        # 创建旋转矩阵
        rotation_angle_radians = np.deg2rad(self.rotation_angle_degrees)
        rotation_matrix = transformed_pcd.get_rotation_matrix_from_axis_angle([0, 0, rotation_angle_radians])
        # 应用旋转变换
        transformed_pcd = transformed_pcd.rotate(rotation_matrix, center=(0, 0, 0))
        o3d.io.write_point_cloud(self.output_file_path, transformed_pcd)

        # 创建两个可视化窗口
        original_pcd.paint_uniform_color([1, 0, 0])  # 原始点云显示为红色
        transformed_pcd.paint_uniform_color([0, 1, 0])  # 变换后的点云显示为绿色

        # 可视化原始点云和变换后的点云
        o3d.visualization.draw_geometries([original_pcd, transformed_pcd], window_name="Original and Transformed Point Cloud Viewer")


if __name__ == '__main__':
    app = RotationPcdMap()
    app.run()