/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-23 01:10:15
 * @Description:
 */
#include "laser_detector.h"

namespace LaserDetectorNS
{

    LaserDetector::LaserDetector() : nh_private_("~")
    {
        is_debug_ = 0;
        switch_status_ = 0;
    }

    LaserDetector::~LaserDetector()
    {
    }

    void LaserDetector::run()
    {
        init();
        ros::spin();
    }

    void LaserDetector::init()
    {

        nh_private_.param<double>("laser_remove_front", laser_remove_front_, 0.05);
        nh_private_.param<double>("laser_remove_back", laser_remove_back_, -0.75);
        nh_private_.param<double>("laser_remove_left", laser_remove_left_, 0.3);
        nh_private_.param<double>("laser_remove_right", laser_remove_right_, -0.3);

        nh_private_.param<double>("laser_detect_front", laser_detect_front_, 2);
        nh_private_.param<double>("laser_detect_back", laser_detect_back_, -2.5);
        nh_private_.param<double>("laser_detect_left", laser_detect_left_, 2.0);
        nh_private_.param<double>("laser_detect_right", laser_detect_right_, -2.0);
        nh_private_.param<double>("laser_detect_top", laser_detect_top_, 2.0);
        nh_private_.param<double>("laser_detect_bottom", laser_detect_bottom_, -1.5);

        server_.setCallback(boost::bind(&LaserDetector::dynamicReconfigureCallback, this, _1, _2));
        srv_switch_status_ = nh_private_.advertiseService("set_switch_status", &LaserDetector::setSwitchStatusCallback, this);

        sub_pointcloud_ = nh_.subscribe("rslidar_points", 10, &LaserDetector::callbackPointCloud, this);
        pub_roi_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("roi_points", 10);
        pub_laser_detection_ = nh_.advertise<htcbot_msgs::LaserDetect>("laser_detection", 10);

        pub_right_close_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("right_close_points", 10);
        pub_left_close_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("left_close_points", 10);
        pub_front_close_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("front_close_points", 10);
        pub_back_close_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("back_close_points", 10);
    }

    void LaserDetector::dynamicReconfigureCallback(laser_detector::LaserDetectorConfig &config, uint32_t level)
    {
        is_debug_ = config.is_debug;
        laser_remove_front_ = config.laser_remove_front;
        laser_remove_back_ = config.laser_remove_back;
        laser_remove_left_ = config.laser_remove_left;
        laser_remove_right_ = config.laser_remove_right;

        laser_detect_front_ = config.laser_detect_front;
        laser_detect_back_ = config.laser_detect_back;
        laser_detect_left_ = config.laser_detect_left;
        laser_detect_right_ = config.laser_detect_right;
        laser_detect_top_ = config.laser_detect_top;
        laser_detect_bottom_ = config.laser_detect_bottom;

        statistical_nums_plus_ = config.statistical_nums_plus;
        statistical_thresh_plus_ = config.statistical_thresh_plus;

        enable_radius_outlier_filter_ = config.enable_radius_outlier_filter;
        radius_outlier_filter_radius_ = config.radius_outlier_filter_radius;
        radius_outlier_filter_nums_ = config.radius_outlier_filter_nums;

        enable_statistical_outlier_filter_ = config.enable_statistical_outlier_filter;
        statistical_outlier_filter_nums_ = config.statistical_outlier_filter_nums;
        statistical_outlier_filter_thresh_ = config.statistical_outlier_filter_thresh;

        enable_intensity_outlier_filter_ = config.enable_intensity_outlier_filter;
        intensity_thresh_ = config.intensity_thresh;
    }

    bool LaserDetector::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res)
    {
        ROS_INFO("[laser_detector_node] ===> req: %d", req.switch_to);
        switch_status_ = req.switch_to;
        res.switch_status = switch_status_;
        return true;
    }

    void LaserDetector::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        // 将ROS的PointCloud2消息转换为PCL的PointCloud
        pcl::PointCloud<pcl::PointXYZI> rawCloud;
        pcl::fromROSMsg(*input, rawCloud);

        pcl::PointCloud<pcl::PointXYZI> roiCloud;

        pcl::PointCloud<pcl::PointXYZI> tempRoiCloud;
        pcl::PointCloud<pcl::PointXYZI> sideRoiCloud;

        pcl::PointCloud<pcl::PointXYZI> vehicleRoiCloud;
        pcl::PointCloud<pcl::PointXYZI> intensityfilterCloud;
        pcl::PointCloud<pcl::PointXYZI> radiusfilteredCloud;
        pcl::PointCloud<pcl::PointXYZI> statisticalfilteredCloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierFilter;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statisticalOutlierRemoval;

        pcl::PointCloud<pcl::PointXYZI> frontClosePoint;
        pcl::PointCloud<pcl::PointXYZI> backClosePoint;
        pcl::PointCloud<pcl::PointXYZI> leftClosePoint;
        pcl::PointCloud<pcl::PointXYZI> rightClosePoint;

        float min_distance_front = laser_detect_front_;
        float min_distance_back = laser_detect_back_;
        float min_distance_left = laser_detect_left_;
        float min_distance_right = laser_detect_right_;

        float angle, height, angle_in_degrees;
        double closest_distance = 10000.0;

        // 遍历点云的每一个点
        for (const auto &point : rawCloud.points)
        {
            // 过滤车身点
            bool is_outside_vehicle =
                point.x > laser_remove_front_ ||
                point.x < laser_remove_back_ ||
                point.y > laser_remove_left_ ||
                point.y < laser_remove_right_;

            // 检测范围的条件
            bool is_within_detection_area =
                point.x < laser_detect_front_ &&
                point.x > laser_detect_back_ &&
                point.y < laser_detect_left_ &&
                point.y > laser_detect_right_ &&
                point.z < laser_detect_top_ &&
                point.z > laser_detect_bottom_;
            // ROS_INFO("is_outside_vehicle %d, point (%f, %f, %f)", is_outside_vehicle, point.x, point.y, point.z);

            // 车身侧面区域
            bool is_within_vehicle =
                point.x < laser_remove_front_ ||
                point.x > laser_remove_back_ ||
                point.y < laser_remove_left_ + 0.5 ||
                point.y > laser_remove_right_ - 0.5;

            // 如果该点在车身和侧面区域之外并且在检测范围内，则添加到结果点云中
            if (is_outside_vehicle && is_within_detection_area && !is_within_vehicle)
            {
                tempRoiCloud.points.push_back(point);
            }
            // 如果该点在车身侧面，先添加到点云中进行一轮离群过滤
            if (is_outside_vehicle && is_within_detection_area && is_within_vehicle)
            {
                if (enable_intensity_outlier_filter_)
                {
                    if (point.intensity > intensity_thresh_)
                    {
                        sideRoiCloud.points.push_back(point);
                    }
                }
                else
                {
                    sideRoiCloud.points.push_back(point);
                }
            }
        }

        // 对车身侧面区域的运动畸变噪声进行离群过滤
        if (enable_statistical_outlier_filter_)
        {
            statisticalOutlierRemoval.setInputCloud(sideRoiCloud.makeShared());
            statisticalOutlierRemoval.setMeanK(statistical_outlier_filter_nums_ + statistical_nums_plus_); // 对车身侧面噪声，加大过滤权重
            statisticalOutlierRemoval.setStddevMulThresh(statistical_outlier_filter_thresh_ + statistical_thresh_plus_);
            statisticalOutlierRemoval.filter(vehicleRoiCloud);
        }
        else
        {
            vehicleRoiCloud = sideRoiCloud;
        }

        tempRoiCloud.points.insert(tempRoiCloud.points.end(), vehicleRoiCloud.points.begin(), vehicleRoiCloud.points.end());

        // 半径滤波
        // 过滤阈值较大，处理大部分噪声
        if (enable_radius_outlier_filter_)
        {
            radiusOutlierFilter.setInputCloud(tempRoiCloud.makeShared());
            radiusOutlierFilter.setRadiusSearch(radius_outlier_filter_radius_);       // 搜索半径
            radiusOutlierFilter.setMinNeighborsInRadius(radius_outlier_filter_nums_); // 设置半径内最小邻居点数
            radiusOutlierFilter.filter(radiusfilteredCloud);
            // ROS_INFO("[laser_detector_node] ===> radiusfilteredCloud.size: %zu", radiusfilteredCloud.points.size());
        }
        else
        {
            radiusfilteredCloud = tempRoiCloud;
        }

        roiCloud = tempRoiCloud;

        for (const auto &point : roiCloud.points)
        {
            // // 计算各个方向的最近距离
            if (point.x > 0 && point.x < min_distance_front && point.y > laser_remove_right_ && point.y < laser_remove_left_)
            {
                min_distance_front = point.x; // 前方向
                frontClosePoint.clear();
                frontClosePoint.points.push_back(point); // 将最近点添加到点云
            }
            if (point.x < 0 && point.x > min_distance_back && point.y > laser_remove_right_ && point.y < laser_remove_left_)
            {
                min_distance_back = point.x; // 后方向
                backClosePoint.clear();
                backClosePoint.points.push_back(point); // 将最近点添加到点云
            }
            if (point.y > 0 && point.y < min_distance_left && point.x > laser_remove_back_ && point.x < laser_remove_front_)
            {
                min_distance_left = point.y; // 左方向
                leftClosePoint.clear();
                leftClosePoint.points.push_back(point); // 将最近点添加到点云
            }
            if (point.y < 0 && point.y > min_distance_right && point.x > laser_remove_back_ && point.x < laser_remove_front_)
            {
                min_distance_right = point.y; // 右方向
                rightClosePoint.clear();
                rightClosePoint.points.push_back(point); // 将最近点添加到点云
            }
        }

        sensor_msgs::PointCloud2 right_close_point_msg;
        pcl::toROSMsg(rightClosePoint, right_close_point_msg);
        right_close_point_msg.header = input->header;
        sensor_msgs::PointCloud2 left_close_point_msg;
        pcl::toROSMsg(leftClosePoint, left_close_point_msg);
        left_close_point_msg.header = input->header;
        sensor_msgs::PointCloud2 front_close_point_msg;
        pcl::toROSMsg(frontClosePoint, front_close_point_msg);
        front_close_point_msg.header = input->header;
        sensor_msgs::PointCloud2 back_close_point_msg;
        pcl::toROSMsg(backClosePoint, back_close_point_msg);
        back_close_point_msg.header = input->header;
        // 发布最近点点云
        pub_right_close_cloud_.publish(right_close_point_msg);
        pub_left_close_cloud_.publish(left_close_point_msg);
        pub_front_close_cloud_.publish(front_close_point_msg);
        pub_back_close_cloud_.publish(back_close_point_msg);

        // 将结果点云从PCL格式转换为ROS消息
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(roiCloud, output);
        output.header = input->header; // 保留输入点云的header信息
        // 发布过滤后的点云（假设有一个publisher_来发布点云消息）
        pub_roi_pointcloud_.publish(output);

        htcbot_msgs::LaserDetect laser_detect_msg;
        laser_detect_msg.front_distance = std::abs(min_distance_front);
        laser_detect_msg.back_distance = std::abs(min_distance_back);
        laser_detect_msg.left_distance = std::abs(min_distance_left);
        laser_detect_msg.right_distance = std::abs(min_distance_right);
        pub_laser_detection_.publish(laser_detect_msg);

        ros::Time end = ros::Time::now();
        // std::cout << ". -> total used time: " << (end - input->header.stamp).toSec() << "s" << std::endl;
    }

}