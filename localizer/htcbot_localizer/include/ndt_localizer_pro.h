/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 12:42:06
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-22 16:00:31
 * @Description: 
 */

#ifndef NDT_LOCALIZER_PRO_H
#define NDT_LOCALIZER_PRO_H

#include "ndt_common.h"
#include <htcbot_common.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <htcbot_localizer/ndt_localizer_proConfig.h>

#include <ros/callback_queue.h>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <can_msgs/vehicle_status.h>

#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/ConfNdtLocalizer.h>
#include <htcbot_msgs/MappingConf.h>
#include <htcbot_msgs/StatusHtcbotModule.h>
#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>

using namespace NdtCommonNS;
using namespace HtcbotCommonNS;

namespace NdtLocalizerProNS
{

class NdtLocalizerPro {
    
public:

    NdtLocalizerPro();
    ~NdtLocalizerPro();
    void run();
    void init();

private:

    ros::NodeHandle nh;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_map_;
    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_initialpose_;
    ros::Subscriber sub_vehicle_status_;
    ros::Subscriber sub_gnss_current_pose_fix_;

    ros::Publisher pub_ndt_pose_;           // 匹配位姿
    ros::Publisher pub_predict_pose_;       // 预测位姿
    ros::Publisher pub_current_pose_;       // 当前位姿
    ros::Publisher pub_current_pose_truth_; // 真实当前位姿
    ros::Publisher pub_time_ndt_localizer_; // 

    ros::Publisher pub_estimated_vel_mps_;  // 估计速度 m/s
    ros::Publisher pub_estimated_vel_kmps_; // 估计速度 km/s
    ros::Publisher pub_system_status_;
    ros::Publisher pub_voice_play_scene_;   // 

    ros::ServiceServer srv_switch_status_;

    htcbot_msgs::StatusHtcbotModule localizer_status_;
    std_msgs::String voice_scene_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<htcbot_localizer::ndt_localizer_proConfig> server_;

    ros::CallbackQueue map_callback_queue_;
    ros::AsyncSpinner* map_spinner_;
    pthread_mutex_t mutex_;

    MethodType method_type_ = MethodType::PCL_GENERIC;

    // 创建一个 TransformBroadcaster 用于发布坐标变换
    tf::TransformBroadcaster br_;
    tf::Transform transform_;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt_;

    pcl::PointCloud<pcl::PointXYZ> map_;
    pcl::PointCloud<pcl::PointXYZ> global_map_;
    pcl::PointCloud<pcl::PointXYZ> local_map_;

    Pose init_pose_;        // 初始位姿
    Pose current_pose_, previous_pose_;     // 当前位姿
    Pose final_pose_;       // 最终(确定)位姿
    Pose ndt_pose_;
    Pose localizer_pose_;
    Pose trans_current_pose_;   // 
    // 预测位姿
    Pose predict_pose_, predict_pose_imu_, predict_pose_odom_, predict_pose_gnss_, predict_pose_vo_;
    // 修正位姿
    Pose fix_pose_gnss_;

    // predict
    bool use_imu_;
    bool use_odom_;
    bool use_gnss_;
    bool use_vehicle_;

    // fix
    double align_error_threshold_;
    bool enable_auto_fix_;    // auto fix
    bool use_gnss_fix_;
    bool use_vehicle_fix_; 
    bool localizer_truth_;
    double distance_pose_to_fix_threshold_;
    double distance_gps_to_gps_threshold_;

    double prev_vehicle_steer_;
    double prev_vehicle_speed_;
    double current_vehicle_steer_;
    double current_vehicle_speed_;

    std::string offset_ = "linear";
    double offset_x_, offset_y_, offset_z_, offset_roll_, offset_pitch_, offset_yaw_;
    double offset_odom_x_, offset_odom_y_, offset_odom_z_, offset_odom_roll_, offset_odom_pitch_, offset_odom_yaw_;
    double offset_imu_x_, offset_imu_y_, offset_imu_z_, offset_imu_roll_, offset_imu_pitch_, offset_imu_yaw_;

    double diff_;
    double diff_x_, diff_y_, diff_z_, diff_yaw_;

    double current_velocity_, previous_velocity_;
    double current_velocity_x_, previous_velocity_x_;
    double current_velocity_y_, previous_velocity_y_;
    double current_velocity_z_, previous_velocity_z_;
    double current_velocity_yaw_, previous_velocity_yaw_;
    
    double previous_previous_velocity_;
    double current_velocity_smooth_;

    double angular_velocity_;

    double current_accel_, previous_accel_;  // [m/s^2]
    double current_accel_x_;
    double current_accel_y_;
    double current_accel_z_;
    double current_accel_yaw_;
    
    int switch_status_;
    bool mode_switch_;
    bool is_debug_;
    bool enable_auto_init_;

    double voxel_leaf_size_;
    double map_voxel_leaf_size_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter_;
    pcl::VoxelGrid<pcl::PointXYZ> map_voxel_grid_filter_;

    // NDT align params
    int max_iter_;          // Maximum iterations
    double step_size_;      // Step size
    float ndt_res_;         // Resolution
    double trans_eps_;      // Transformation epsilon

    bool f_map_loaded_;
    bool f_init_pos_set_;

    int queue_size_;
    std::string pointcloud_topic_;
    std::string scan_topic_;
    std::string map_topic_;
    std::string lidar_frame_;

    ros::Time current_scan_time_;
    ros::Time previous_scan_time_;
    double diff_time_;
    double delay_time_;
    double delay_time_error_threshold_;
    int error_offset_num_;          // 错误定位次数
    int offset_count_threshold_;
    double error_offset_time_;      // 错误定位时间
    double offset_time_threshold_;  // 偏移时间修正阈值

    int try_fix_count_;             // 偏移修正尝试次数

    // 匹配与预测差值
    double predict_pose_distance_;
    double predict_pose_error_threshold_;
    bool f_use_predict_pose_;

    double align_time_, get_fitness_score_time_;
    std::chrono::time_point<std::chrono::system_clock> align_start_, 
        align_end_, 
        get_fitness_score_start_,
        get_fitness_score_end_;
    
    std::chrono::time_point<std::chrono::system_clock> matching_start_, matching_end_;
    double exe_time_;
    bool has_converged_;
    int iteration_;
    double fitness_score_;
    double trans_probability_;

    // 地图序号
    unsigned int points_map_num_;

    double tf_lidar_x_;
    double tf_lidar_y_;
    double tf_lidar_z_;
    double tf_lidar_roll_;
    double tf_lidar_pitch_;
    double tf_lidar_yaw_;

    Eigen::Matrix4f tf_btol_, tf_ltob_;

    void dynamicReconfigureCallback(htcbot_localizer::ndt_localizer_proConfig &config, uint32_t level);
    bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);
    void callbackMap(const sensor_msgs::PointCloud2::ConstPtr& input);
    void callbackPoints(const sensor_msgs::PointCloud2::ConstPtr& input);
    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
    void callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr& msg);
    void callbackGnssCurrentPoseFix(const geometry_msgs::PoseStamped::ConstPtr& input);

    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
    Pose convertPoseIntoRelativeCoordinate(const Pose &target_pose, const Pose &reference_pose);
    
};

}



#endif  // NDT_LOCALIZER_PRO_H