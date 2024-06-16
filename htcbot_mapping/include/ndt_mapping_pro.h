/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 12:42:06
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 14:12:14
 * @Description: 
 */

#ifndef NDT_MAPPING_PRO_H
#define NDT_MAPPING_PRO_H

#include "ndt_common.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <htcbot_mapping/ndt_mapping_proConfig.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <ndt_cpu/NormalDistributionsTransform.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/ConfNdtMapping.h>
#include <htcbot_msgs/MappingConf.h>

using namespace NdtCommonNS;

namespace NdtMappingProNS
{

class NdtMappingPro {
    
public:

    NdtMappingPro();
    ~NdtMappingPro();
    void run();
    void init();

private:

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<htcbot_mapping::ndt_mapping_proConfig> server_;

    ros::Subscriber sub_map_path_conf_;
    ros::Subscriber sub_ndt_mapping_conf_;
    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_mapping_conf_;

    ros::Publisher pub_ndt_map_;
    ros::Publisher pub_current_pose_;
    ros::Publisher pub_mix_current_pose_;

    MethodType method_type_ = MethodType::PCL_GENERIC;

    Pose init_pose_;    // 初始位姿
    Pose current_pose_;     // 当前位姿
    Pose previous_pose_;
    Pose current_pose_imu_, current_pose_odom_; 
    Pose added_pose_;
    Pose guess_pose_, ndt_pose_, localizer_pose_;

    double diff_ , diff_x_, diff_y_, diff_z_, diff_roll_, diff_pitch_, diff_yaw_;
    double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
    double current_velocity_x_, current_velocity_y_, current_velocity_z_;

    ros::Time current_scan_time_;
    ros::Time previous_scan_time_;
    ros::Duration scan_duration_;
    double delay_time_;
    double align_time_;
    std::chrono::time_point<std::chrono::system_clock> align_start_, align_end_;

    pcl::PointCloud<pcl::PointXYZI> map_;    // 点云地图
    pcl::PointCloud<pcl::PointXYZI> map_filtered_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;   // 降采样


    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt_;

    nav_msgs::Path history_trajectory_;

    int switch_to_exp_;

    bool is_debug_;
    bool is_pub_map_;
    double queue_size_;

    // Default values
    double voxel_leaf_size_;    // NDT降采样
    int max_iter_;          // Maximum iterations
    double step_size_;      // Step size
    float ndt_res_;         // Resolution
    double trans_eps_;      // Transformation epsilon

    bool is_filter_add_map_;    // 是否过滤后添加至地图
    double filter_res_;         // 过滤降采样

    double min_scan_range_;
    double max_scan_range_;
    double min_scan_height_;
    double max_scan_height_;
    double min_add_shift_;      // 最小地图更新步长
    bool is_limit_flat_;
    bool is_sync_update_map_;
    bool is_remapping_;

    double align_error_threshold_;

    bool is_pub_pgm_;
    bool is_filter_pass_through_;
    double filter_high_;
    double filter_low_;
    bool is_filter_radius_outlier_;
    double filter_radius_;
    int filter_thre_count_;

    // NDT Res
    double fitness_score_;
    bool has_converged_;
    int final_num_iteration_;
    double transformation_probability_;

    std::string map_path_;
    bool f_set_map_path_;
    bool f_initial_scan_loaded_;    // 是否加载首帧(初始)地图

    // Data Input

    std::string lidar_frame_;
    std::string pointcloud_topic_;
    std::string base_frame_;

    bool use_imu_;
    bool use_odom_;
    bool use_gnss_;
    bool use_vehicle_;
    bool use_vo_;

    std::string imu_topic_;
    std::string odom_topic_;
    std::string gnss_topic_;
    std::string vehicle_topic_;

    tf::TransformListener transform_listener_;
    tf::StampedTransform lidar_base_tf_;
    bool f_lidar_base_tf_;
    
    double tf_lidar_x_;
    double tf_lidar_y_;
    double tf_lidar_z_;
    double tf_lidar_roll_;
    double tf_lidar_pitch_;
    double tf_lidar_yaw_;

    Eigen::Matrix4f tf_btol_, tf_ltob_;

    void initPose();

    void dynamicReconfigureCallback(htcbot_mapping::ndt_mapping_proConfig &config, uint32_t level);

    void callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg);
    void callbackConfNdtMapping(const htcbot_msgs::ConfNdtMapping::ConstPtr &msg);
    void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input);
    void callbackConfMapping(const htcbot_msgs::MappingConf::ConstPtr& msg);

    bool findTansform4Source2Target(tf::StampedTransform& transform, const std::string source_frame, const std::string target_frame);
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
    void doFilterMap();
    void doSaveMap2Pcd(const std::string filename);

};

}



#endif  // NDT_MAPPING_PRO_H