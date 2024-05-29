/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-29 11:10:23
 * @Description: 
 */
#include "laser_detector.h"

namespace LaserDetectorNS {

LaserDetector::LaserDetector() : nh_private_("~") {
    is_debug_ = 0;
    switch_status_ = 0;
    enable_radius_outlier_filter_ = 0;
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
    pub_raw_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("firstfilterCloud", 10);
    pub_detect_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("detect_points", 10);
    pub_close_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("close_points", 10);
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
    
    enable_radius_outlier_filter_ = config.enable_radius_outlier_filter;
    radius_outlier_filter_radius_ = config.radius_outlier_filter_radius;
    radius_outlier_filter_nums_ = config.radius_outlier_filter_nums;

    enable_statistical_outlier_filter_ = config.enable_statistical_outlier_filter;
    statistical_outlier_filter_nums_ = config.statistical_outlier_filter_nums;
    statistical_outlier_filter_thresh_ = config.statistical_outlier_filter_thresh;

    enable_intensity_outlier_filter_ = config.enable_intensity_outlier_filter;
    intensity_thresh_ = config.intensity_thresh;
}

bool LaserDetector::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res) {
    ROS_INFO("[laser_detector_node] ===> req: %d", req.switch_to); 
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_; 
    return true;                      
}
//反射强度滤波、半径滤波、离群点滤波
   void LaserDetector::doFilter(){
    pcl::PointCloud<pcl::PointXYZI> firstfilterCloud; 
    pcl::PointCloud<pcl::PointXYZI> filteredCloud; 
    pcl::PointCloud<pcl::PointXYZI> roiCloud; 
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statisticalOutlierRemoval;
   if(enable_intensity_outlier_filter_){
     for (const auto& point :roiCloud.points)
     {
         if (point.intensity >intensity_thresh_){
            continue;
        }else{
         firstfilterCloud.points.push_back(point);
        }
     }
   }else{
    firstfilterCloud=roiCloud;
   }

    if (enable_radius_outlier_filter_) {
        radiusOutlierFilter.setInputCloud(firstfilterCloud.makeShared()); // 设置RadiusOutlierRemoval的输入点云
        radiusOutlierFilter.setRadiusSearch(radius_outlier_filter_radius_); // 搜索半径
        radiusOutlierFilter.setMinNeighborsInRadius(radius_outlier_filter_nums_); // 设置半径内最小邻居点数
        radiusOutlierFilter.filter(filteredCloud);
    } else {
        filteredCloud = firstfilterCloud;
    }
	
if(enable_statistical_outlier_filter_){
	statisticalOutlierRemoval.setInputCloud(firstfilterCloud.makeShared());//设置待滤波的点云
	statisticalOutlierRemoval.setMeanK(statistical_outlier_filter_nums_);//设置在进行统计时考虑查询点邻居点数
	statisticalOutlierRemoval.setStddevMulThresh(statistical_outlier_filter_thresh_);//设置判断是否为离群点的阈值
	statisticalOutlierRemoval.filter(filteredCloud);//将滤波结果保存在cloud_filtered中
}else{
    filteredCloud = firstfilterCloud;
}
    }

void LaserDetector::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input) {
    // 将ROS的PointCloud2消息转换为PCL的PointCloud
    pcl::PointCloud<pcl::PointXYZI>rawCloud;
    pcl::fromROSMsg(*input, rawCloud); // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZI> firstfilterCloud; 
    pcl::PointCloud<pcl::PointXYZI> filteredCloud; 
    pcl::PointCloud<pcl::PointXYZI> detectCloud; 
    pcl::PointCloud<pcl::PointXYZI> roiCloud; 
    pcl::PointCloud<pcl::PointXYZI> CLOSECloud;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statisticalOutlierRemoval;
    float min_distance_front = laser_detect_front_;
    float min_distance_back = laser_detect_back_;
    float min_distance_left = laser_detect_left_;
    float min_distance_right = laser_detect_right_;
    float angle,height,angle_in_degrees;
    double closest_distance = 10000.0; 
    pcl::PointXYZI closest_point; // 存储最近点的坐标

    // 遍历点云的每一个点
    for (const auto& point :rawCloud.points) {
       
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
        }
    }
    LaserDetector::doFilter();
     for (int i = 0; i < filteredCloud.points.size(); i++){
            detectCloud.points.push_back(filteredCloud.points[i]);
                        // 计算与原点的距离
            double distance = std::sqrt(
                filteredCloud.points[i].x * filteredCloud.points[i].x +
                filteredCloud.points[i].y * filteredCloud.points[i].y +
                filteredCloud.points[i].z * filteredCloud.points[i].z
            );

            // 如果找到更近的点，则更新最近点的坐标和距离
            if (distance < closest_distance) {
                closest_point = filteredCloud.points[i];
                // 计算高度和角度
                height = closest_point.z;
                angle = atan2(closest_point.y, closest_point.x);
                if (angle < 0) {
                     angle += 2 * M_PI; // 将角度转换到0到2PI范围内
                    }
                    angle_in_degrees = angle * (180.0 / M_PI);
                closest_distance = distance;
                CLOSECloud.clear(); 
                CLOSECloud.points.push_back(filteredCloud.points[i]);//将最近点添加到点云
            }


    }
    // 将结果点云从PCL格式转换为ROS消息
    sensor_msgs::PointCloud2 roi;
    sensor_msgs::PointCloud2 firstfilter;
    sensor_msgs::PointCloud2 detect;
    sensor_msgs::PointCloud2  CLOSECloud_msg;
    pcl::toROSMsg(roiCloud, roi);
    pcl::toROSMsg(firstfilterCloud, firstfilter);
    pcl::toROSMsg(detectCloud, detect);
    pcl::toROSMsg( CLOSECloud,  CLOSECloud_msg);
    roi.header = input->header; 
    firstfilter.header = input->header; 
    detect.header = input->header; // 保留输入点云的header信息
    CLOSECloud_msg.header = input->header;
    pub_roi_pointcloud_.publish(roi);
    pub_raw_pointcloud_.publish(firstfilter);
    pub_detect_pointcloud_.publish(detect);
    pub_close_cloud_.publish( CLOSECloud_msg); // 发布最近点点云
    htcbot_msgs::LaserDetect laser_detect_msg; 
    laser_detect_msg.distance = closest_distance; // 使用找到的最近距离
    laser_detect_msg.height=height;
    laser_detect_msg.direction = angle_in_degrees;
    pub_laser_detection_.publish(laser_detect_msg);

}


}