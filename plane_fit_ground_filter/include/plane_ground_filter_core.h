#pragma once

#include <ros/ros.h>
#pragma once

#include <ros/ros.h>

// 禁用PCL预编译库，使用PointXYZIR点类型
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
// 矩阵运算
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

namespace velodyne_pointcloud
{
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                // 四元浮点数XYZ
    float intensity;                /// 激光强度
    uint16_t ring;                  /// 激光环数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保正确对齐
  } EIGEN_ALIGN16;

};

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

// 用于存储聚类点
namespace plane_ground_filter
{
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                // 四元浮点数XYZ
    float intensity;                /// 激光强度读数
    uint16_t ring;                  /// 激光环数
    uint16_t label;                 ///< 点标签
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保正确对齐
  } EIGEN_ALIGN16;

};

#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter
{

private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
  std::string point_topic_;

  int sensor_model_;
  double sensor_height_, clip_height_, min_distance_, max_distance_;
  int num_seg_ = 1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;

  float d_, th_dist_d_;
  MatrixXf normal_;

  pcl::PointCloud<VPoint>::Ptr g_seeds_pc;
  pcl::PointCloud<VPoint>::Ptr g_ground_pc;
  pcl::PointCloud<VPoint>::Ptr g_not_ground_pc;
  pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);
  void post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void clip_above(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);

  void clip_to_circle(const pcl::PointCloud<VPoint>::Ptr in,
                      const pcl::PointCloud<VPoint>::Ptr out);

public:
  PlaneGroundFilter(ros::NodeHandle &nh);
  ~PlaneGroundFilter();
  void Spin();
};
