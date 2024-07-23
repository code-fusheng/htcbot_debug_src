/*
 * @Author: code-fusheng
 * @Date: 2024-04-25 09:56:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-29 10:36:12
 * @Description:
 */
#ifndef LASER_DETECTOR_H
#define LASER_DETECTOR_H

#include <htcbot_common.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detector/LaserDetectorConfig.h>

#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>

#include <iostream>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// 障碍物距离信息
#include <htcbot_msgs/LaserDetect.h>

using namespace HtcbotCommonNS;
namespace LaserDetectorNS
{

    class LaserDetector
    {

    public:
        LaserDetector();
        ~LaserDetector();
        void run();
        void init();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::ServiceServer srv_switch_status_;

        // 声明动态重配置服务器
        dynamic_reconfigure::Server<laser_detector::LaserDetectorConfig> server_;

        ros::Publisher pub_roi_pointcloud_;
        ros::Publisher pub_detect_pointcloud_;
        ros::Publisher pub_right_close_cloud_;
        ros::Publisher pub_left_close_cloud_;
        ros::Publisher pub_front_close_cloud_;
        ros::Publisher pub_back_close_cloud_;
        ros::Publisher pub_laser_detection_;
        ros::Subscriber sub_pointcloud_;

        bool is_debug_;
        int switch_status_;

        // 雷达过滤距离
        double laser_remove_front_;
        double laser_remove_back_;
        double laser_remove_left_;
        double laser_remove_right_;
        // 雷达检测距离
        double laser_detect_front_;
        double laser_detect_back_;
        double laser_detect_left_;
        double laser_detect_right_;
        double laser_detect_top_;
        double laser_detect_bottom_;

        // 离群滤波加权
        double statistical_nums_plus_;
        double statistical_thresh_plus_;

        // 半径滤波
        bool enable_radius_outlier_filter_;
        double radius_outlier_filter_radius_;
        double radius_outlier_filter_nums_;
        // 离群滤波
        bool enable_statistical_outlier_filter_;
        double statistical_outlier_filter_nums_;
        double statistical_outlier_filter_thresh_;
        // 强度滤波
        bool enable_intensity_outlier_filter_;
        double intensity_thresh_;

        void dynamicReconfigureCallback(laser_detector::LaserDetectorConfig &config, uint32_t level);
        bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);
        void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &input);
    };

}

#endif // LASER_DETECTOR_H
