/*
 * @Author: code-fusheng
 * @Date: 2024-04-25 09:56:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 14:31:31
 * @Description: 
 */
#ifndef LASER_EUCLIDEAN_CLUSTER_H
#define LASER_EUCLIDEAN_CLUSTER_H

#include <htcbot_common.h>
#include <iostream>
#include "cluster.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <laser_euclidean_cluster/LaserEuclideanClusterConfig.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "autoware_msgs/Centroids.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>
#include "opencv2/core/core.hpp"
//#include "precomp.hpp"
#include <opencv2/opencv.hpp>

#include <htcbot_msgs/StatusHtcbotModule.h>
#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>

#include <htcbot_msgs/DetectedObject.h>
#include <htcbot_msgs/DetectedObjectArray.h>
#include <htcbot_msgs/CloudCluster.h>
#include <htcbot_msgs/CloudClusterArray.h>
#include <htcbot_msgs/ConfEuclideanCluster.h>

using namespace cv;
namespace LaserEuclideanClusterNS {

class LaserEuclideanCluster {

public:

    LaserEuclideanCluster();
    ~LaserEuclideanCluster();
    void run();
    void init();

private:

    struct Detected_Obj
    {
        jsk_recognition_msgs::BoundingBox bounding_box_;
        std_msgs::Header header;
        geometry_msgs::Pose pose;
        geometry_msgs::Vector3 dimensions;
        std_msgs::Float32 value;
        std_msgs::UInt32 label;

        pcl::PointXYZ min_point_;
        pcl::PointXYZ max_point_;
        pcl::PointXYZ centroid_;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceServer srv_switch_status_;

    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_point_cloud_pro_;
    ros::Subscriber sub_euclidean_cluster_conf_;

    ros::Publisher pub_bounding_boxs_;
    ros::Publisher pub_bounding_nums;
    ros::Publisher pub_polyon;

    ros::Publisher pub_detected_obj_array;
    ros::Publisher pub_centroid_;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_module_status_;
    htcbot_msgs::StatusHtcbotModule module_status_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<laser_euclidean_cluster::LaserEuclideanClusterConfig> server_;

    bool is_debug_;
    int switch_status_;
    std_msgs::Header point_cloud_header_;
    std::string in_cloud_topic_;
    double remove_points_upto_;
    //  点云降采样
    bool downsample_cloud_;
    double leaf_size_;
    double clip_min_height_;
    double clip_max_height_;
    bool keep_lanes_;
    double keep_lane_left_distance_;
    double keep_lane_right_distance_;
    bool remove_ground_;
    bool use_diffnormals_;
    double cluster_merge_threshold_;
    double clustering_distance_;
    int cluster_size_min_;
    int cluster_size_max_;
    bool use_multiple_thres_;
    std::vector<double> clustering_distances_;
    std::vector<double> clustering_ranges_;
    std::vector<cv::Scalar> colors_;
    bool pose_estimation_;

    // 创建TF变换相关的对象
    tf::StampedTransform transform;
    tf::TransformListener listener;

    void dynamicReconfigureCallback(laser_euclidean_cluster::LaserEuclideanClusterConfig &config, uint32_t level);
    bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);

    void callback_pointcloud(const sensor_msgs::PointCloud2ConstPtr& in_pointcloud_ptr);

    void removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size);

    void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height, float in_max_height);

    void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                        float in_left_lane_threshold,
                        float in_right_lane_threshold);

    void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, 
                    float in_max_height,
                    float in_floor_max_angle);

    void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                        autoware_msgs::Centroids &in_out_centroids, htcbot_msgs::CloudClusterArray &in_out_clusters);

    std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                            autoware_msgs::Centroids &in_out_centroids,
                                            double in_max_cluster_distance);

    void checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                        float in_merge_threshold);

    void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                        std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                        double in_merge_threshold);

    void mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                    std::vector<size_t> in_merge_indices, const size_t &current_index,
                    std::vector<bool> &in_out_merged_clusters);

    void publishColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);

    void publishDetectedObjects(const htcbot_msgs::CloudClusterArray &in_clusters);

    void publishCentroids(const autoware_msgs::Centroids &in_centroids);

    void downsamplePoints( const Mat& src, Mat& dst, size_t count );

    void generateColors( std::vector<Scalar>& colors, size_t count, size_t factor);

};

}

#endif //LASER_EUCLIDEAN_CLUSTER_H