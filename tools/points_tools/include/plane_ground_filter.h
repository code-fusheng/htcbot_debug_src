/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:45
 * @LastEditors: code-fusheng 2561035977@qq.com
 */

#ifndef PLANE_GROUND_FILTER_H
#define PLANE_GROUND_FILTER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <points_tools/PlaneGroundFilterConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

// 矩阵运算
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace PlaneGroundFilterNS
{

    class PlaneGroundFilter
    {

    public:
        PlaneGroundFilter();
        ~PlaneGroundFilter();
        void run();
        void init();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber sub_pointcloud_;
        ros::Publisher pub_ground_points_;
        ros::Publisher pub_no_ground_points_;
        ros::Publisher pub_all_points_;

        // 声明动态重配置服务器
        dynamic_reconfigure::Server<points_tools::PlaneGroundFilterConfig> server_;

        // 算法核心参数
        int sensor_model_;
        double sensor_height_;
        double clip_height_;
        double min_distance_;
        double max_distance_;

        int num_seg_ = 1;
        int num_iter_;
        int num_lpr_;
        double th_seeds_;
        double th_dist_;

        float d_;
        float th_dist_d_;
        MatrixXf normal_;

        std::string ground_topic_;
        std::string no_ground_topic_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr seeds_points_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_points_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_points_;

        void dynamicReconfigureCallback(points_tools::PlaneGroundFilterConfig &config, uint32_t level);

        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input);
        void extractIntialSeeds(const pcl::PointCloud<pcl::PointXYZI> &sorted_points);
        void estimatePlane();
    };

}

#endif // PLANE_GROUND_FILTER_H
