/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-23 01:10:15
 * @Description: 
 */
#include "laser_detector.h"

namespace LaserDetectorNS {

LaserDetector::LaserDetector() : nh_private_("~") {
    is_debug_ = 0;
    switch_status_ = 0;
}

LaserDetector::~LaserDetector() {
}

void LaserDetector::run() {
    init();
    ros::spin();
}

void LaserDetector::init() {

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
}

void LaserDetector::dynamicReconfigureCallback(laser_detector::LaserDetectorConfig &config, uint32_t level) {
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
}

bool LaserDetector::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res) {
    ROS_INFO("[laser_detector_node] ===> req: %d", req.switch_to); 
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_; 
    return true;                      
}

void LaserDetector::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input) {
    // 将ROS的PointCloud2消息转换为PCL的PointCloud
    pcl::PointCloud<pcl::PointXYZ> rawCloud;
    pcl::fromROSMsg(*input, rawCloud);

    pcl::PointCloud<pcl::PointXYZ> roiCloud;

    float min_distance_front = laser_detect_front_;
    float min_distance_back = laser_detect_back_;
    float min_distance_left = laser_detect_left_;
    float min_distance_right = laser_detect_right_;

    // 遍历点云的每一个点
    for (const auto& point : rawCloud.points) {
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

        // 如果该点在车身之外并且在检测范围内，则添加到结果点云中
        if (is_outside_vehicle && is_within_detection_area) {
            roiCloud.points.push_back(point);    
            // // 计算各个方向的最近距离
            if (point.x > 0 && point.x < min_distance_front && point.y > laser_remove_right_ && point.y < laser_remove_left_) {
                min_distance_front = point.x;  // 前方向
            }
            if (point.x < 0 && point.x > min_distance_back && point.y > laser_remove_right_ && point.y < laser_remove_left_) {
                min_distance_back = point.x;  // 后方向
            }
            if (point.y > 0 && point.y < min_distance_left && point.x > laser_remove_back_ && point.x < laser_remove_front_) {
                min_distance_left = point.y;  // 左方向
            }
            if (point.y < 0 && point.y > min_distance_right && point.x > laser_remove_back_ && point.x < laser_remove_front_) {
                min_distance_right = point.y;  // 右方向
            }
        }
    }

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

}

}