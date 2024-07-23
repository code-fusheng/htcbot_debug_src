/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 17:01:53
 * @Description:
 */
#include "laser_euclidean_cluster.h"

using namespace HtcbotCommonNS;
namespace LaserEuclideanClusterNS
{

  LaserEuclideanCluster::LaserEuclideanCluster() : nh_private_("~")
  {
    is_debug_ = 0;
    switch_status_ = 0;
    // 障碍物检测的模块状态由障碍物检测情况决定
    module_status_.module_type = static_cast<int>(HtcbotCommonNS::MODULE_TYPE::SENSOR);
    module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
  }

  LaserEuclideanCluster::~LaserEuclideanCluster()
  {
  }

  void LaserEuclideanCluster::run()
  {
    init();
    while (ros::ok())
    {
      // ROS_DEBUG_STREAM("[laser_euclidean_cluster_node] ===> status:" << switch_status_ << "...");
      ros::spin();
    }
  }

  void LaserEuclideanCluster::init()
  {

    server_.setCallback(boost::bind(&LaserEuclideanCluster::dynamicReconfigureCallback, this, _1, _2));
    srv_switch_status_ = nh_.advertiseService("/laser_euclidean_cluster_node/set_switch_status", &LaserEuclideanCluster::setSwitchStatusCallback, this);

    nh_private_.param<std::string>("in_cloud_topic", in_cloud_topic_, "/points_raw");

    nh_private_.param<double>("remove_points_upto", remove_points_upto_, 0.0);
    nh_private_.param<bool>("downsample_cloud", downsample_cloud_, false);
    nh_private_.param<double>("leaf_size", leaf_size_, 0.1);
    nh_private_.param<double>("clip_min_height", clip_min_height_, -0.65);
    nh_private_.param<double>("clip_max_height", clip_max_height_, 0.5);

    nh_private_.param<bool>("keep_lanes", keep_lanes_, false);
    nh_private_.param<double>("keep_lane_left_distance", keep_lane_left_distance_, 5.0);
    nh_private_.param<double>("keep_lane_right_distance", keep_lane_right_distance_, 5.0);
    nh_private_.param<int>("cluster_size_min", cluster_size_min_, 20);
    nh_private_.param<int>("cluster_size_max", cluster_size_max_, 100000);
    nh_private_.param<bool>("remove_ground", remove_ground_, false);
    nh_private_.param<bool>("use_diffnormals", use_diffnormals_, false);
    nh_private_.param<double>("cluster_merge_threshold", cluster_merge_threshold_, 1.5);
    nh_private_.param<double>("clustering_distance", clustering_distance_, 0.75);
    nh_private_.param<bool>("use_multiple_thres", use_multiple_thres_, true);
    nh_private_.param<bool>("pose_estimation", pose_estimation_, false);
    nh_private_.param<int>("switch_status", switch_status_, 0);

    clustering_distances_ = {0.3, 0.6, 0.9, 2.0, 2.5};
    clustering_ranges_ = {15, 30, 45, 60};

    // 生成颜色映射
    generateColors(colors_, 255, 100);

    pub_module_status_ = nh_.advertise<htcbot_msgs::StatusHtcbotModule>("/htcbot/module_status", 10);
    // 发布检测到的边界框
    pub_bounding_boxs_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
    // 发布检测到的物体数量
    pub_bounding_nums = nh_.advertise<std_msgs::Int16>("/detected_bounding_nums", 8);
    pub_polyon = nh_.advertise<geometry_msgs::PolygonStamped>("/detection/debug_polygon", 10);
    // 发布整个检测到的物体数组
    pub_detected_obj_array = nh_.advertise<htcbot_msgs::DetectedObjectArray>("/detected_objects", 10);

    // 发布聚类中心的话题
    pub_centroid_ = nh_.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);
    pub_cluster_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);

    if (switch_status_)
    {
      sub_point_cloud_pro_ = nh_.subscribe(in_cloud_topic_, 5, &LaserEuclideanCluster::callback_pointcloud, this);
    }
  }

  void LaserEuclideanCluster::dynamicReconfigureCallback(laser_euclidean_cluster::LaserEuclideanClusterConfig &config, uint32_t level)
  {
    is_debug_ = config.is_debug;
  }

  bool LaserEuclideanCluster::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res)
  {
    ROS_INFO("[laser_euclidean_cluster_node] ===> req: %d", req.switch_to);
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_;
    if (switch_status_)
    {
      sub_point_cloud_pro_ = nh_.subscribe(in_cloud_topic_, 5, &LaserEuclideanCluster::callback_pointcloud, this);
    }
    else
    {
      sub_point_cloud_pro_.shutdown();
    }
    return true;
  }

  /**
   * code-fusheng : htcbot
   * 专业版聚类检测 2024-3-26
   */
  void LaserEuclideanCluster::callback_pointcloud(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
  {
    // 创建一系列点云相关的消息变量
    // 当前传入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 点云消息的头部信息
    point_cloud_header_ = in_sensor_cloud->header;

    autoware_msgs::Centroids centroids;
    htcbot_msgs::CloudClusterArray cloud_clusters;

    // 将 ROS 中输入的点云消息转换为 PCL 格式的点云数据
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
    // 移除点云的距离阈值
    if (remove_points_upto_ > 0.0)
    {
      // 移除距离雷达太近的点
      removePointsUpTo(current_sensor_cloud_ptr, removed_points_cloud_ptr, remove_points_upto_);
    }
    else
    {
      removed_points_cloud_ptr = current_sensor_cloud_ptr;
    }
    // 点云降采样
    if (downsample_cloud_)
    {
      downsampleCloud(removed_points_cloud_ptr, downsampled_cloud_ptr, leaf_size_);
    }
    else
    {
      downsampled_cloud_ptr = removed_points_cloud_ptr;
    }
    // 点云高度剪裁
    clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, clip_min_height_, clip_max_height_);

    // 保留指定车道宽度内的点
    if (keep_lanes_)
      keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, keep_lane_left_distance_, keep_lane_right_distance_);
    else
      inlanes_cloud_ptr = clipped_cloud_ptr;

    // 移除地面点
    if (remove_ground_)
    {
      removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr, 0.2, 0.1);
    }
    else
    {
      nofloor_cloud_ptr = inlanes_cloud_ptr;
    }
    diffnormals_cloud_ptr = nofloor_cloud_ptr;
    // 处理后的点云进行距离分割，得到彩色聚类点云
    segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, centroids,
                      cloud_clusters);
    // 发布彩色点云
    publishColorCloud(colored_clustered_cloud_ptr);
    // 发布质心
    // pub_centroid_
    centroids.header = point_cloud_header_;
    publishCentroids(centroids);

    cloud_clusters.header = point_cloud_header_;

    publishDetectedObjects(cloud_clusters);
    // 发布模块状态
    pub_module_status_.publish(module_status_);
  }

  void LaserEuclideanCluster::removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
  {
    out_cloud_ptr->points.clear();
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
      if (origin_distance > in_distance)
      {
        out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
      }
    }
  }

  void LaserEuclideanCluster::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2)
  {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
  }

  void LaserEuclideanCluster::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height = -1.3, float in_max_height = 0.5)
  {
    out_cloud_ptr->points.clear();
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      if (in_cloud_ptr->points[i].z >= in_min_height && in_cloud_ptr->points[i].z <= in_max_height)
      {
        out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
      }
    }
  }

  void LaserEuclideanCluster::keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5,
                                             float in_right_lane_threshold = 1.5)
  {
    pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
      {
        far_indices->indices.push_back(i);
      }
    }
    out_cloud_ptr->points.clear();
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(far_indices);
    extract.setNegative(true); // true removes the indices, false leaves only the indices
    extract.filter(*out_cloud_ptr);
  }

  void LaserEuclideanCluster::removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr,
                                          float in_max_height = 0.2,
                                          float in_floor_max_angle = 0.1)
  {

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(in_floor_max_angle);

    seg.setDistanceThreshold(in_max_height); // floor distance
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // REMOVE THE FLOOR FROM THE CLOUD
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true); // true removes the indices, false leaves only the indices
    extract.filter(*out_nofloor_cloud_ptr);

    // EXTRACT THE FLOOR FROM THE CLOUD
    extract.setNegative(false); // true removes the indices, false leaves only the indices
    extract.filter(*out_onlyfloor_cloud_ptr);
  }

  void LaserEuclideanCluster::publishColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = point_cloud_header_;
    pub_cluster_cloud_.publish(cloud_msg);
  }

  void LaserEuclideanCluster::publishCentroids(const autoware_msgs::Centroids &in_centroids)
  {
    pub_centroid_.publish(in_centroids);
  }

  void LaserEuclideanCluster::publishDetectedObjects(const htcbot_msgs::CloudClusterArray &in_clusters)
  {
    htcbot_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_clusters.header;

    for (size_t i = 0; i < in_clusters.clusters.size(); i++)
    {
      htcbot_msgs::DetectedObject detected_object;
      detected_object.header = in_clusters.header;
      detected_object.label = "unknown";
      detected_object.score = 1;
      detected_object.space_frame = in_clusters.header.frame_id;
      detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
      detected_object.dimensions = in_clusters.clusters[i].dimensions;
      detected_object.pointcloud = in_clusters.clusters[i].cloud;
      detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
      detected_object.valid = true;
      detected_objects.objects.push_back(detected_object);
    }
    pub_detected_obj_array.publish(detected_objects);
  }

  void LaserEuclideanCluster::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                                autoware_msgs::Centroids &in_out_centroids, htcbot_msgs::CloudClusterArray &in_out_clusters)
  {
    // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
    // entire pc)
    // in this way, the points farther in the pc will also be clustered

    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6

    std::vector<ClusterPtr> all_clusters;

    if (!use_multiple_thres_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

      for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
      {
        pcl::PointXYZ current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;

        cloud_ptr->points.push_back(current_point);
      }
      all_clusters =
          clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, clustering_distance_);
      ROS_DEBUG("[euclidean_cluster false-multiple] all_clusters: %zu", all_clusters.size());
    }
    else
    {
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
      for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_segments_array[i] = tmp_cloud;
      }

      for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
      {
        pcl::PointXYZ current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));
        // 按距离点云分段
        if (origin_distance < clustering_ranges_[0])
        {
          cloud_segments_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < clustering_ranges_[1])
        {
          cloud_segments_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < clustering_ranges_[2])
        {
          cloud_segments_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < clustering_ranges_[3])
        {
          cloud_segments_array[3]->points.push_back(current_point);
        }
        else
        {
          cloud_segments_array[4]->points.push_back(current_point);
        }
      }
      // 给分段点云 聚类
      std::vector<ClusterPtr> local_clusters;
      for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
      {
        local_clusters = clusterAndColor(
            cloud_segments_array[i], out_cloud_ptr, in_out_centroids, clustering_distances_[i]);
        all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
        ROS_DEBUG("[euclidean_cluster true-multiple] all_clusters: %zu", all_clusters.size());
      }
    }

    // Clusters can be merged or checked in here
    //....
    // check for mergable clusters
    std::vector<ClusterPtr> mid_clusters;
    std::vector<ClusterPtr> final_clusters;
    // 根据两个聚类质心距离判断是否要合并 这是一个嵌套函数
    if (all_clusters.size() > 0)
      checkAllForMerge(all_clusters, mid_clusters, cluster_merge_threshold_);
    else
      mid_clusters = all_clusters;

    if (mid_clusters.size() > 0)
      checkAllForMerge(mid_clusters, final_clusters, cluster_merge_threshold_);
    else
      final_clusters = mid_clusters;

    if (final_clusters.size() == 0)
    {
      module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
      module_status_.reason = "Not Found Clusters!";
    }
    // ROS_INFO("[euclidean_cluster] final_clusters: %d, all_clusters: %d, mid_clusters: %d",final_clusters.size(),  all_clusters.size(), mid_clusters.size());

    // Get final PointCloud to be published

    // add
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (unsigned int i = 0; i < final_clusters.size(); i++)
    {
      *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

      jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
      geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();

      pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
      pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
      pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
      geometry_msgs::Point centroid;
      centroid.x = center_point.x;
      centroid.y = center_point.y;
      centroid.z = center_point.z;
      bounding_box.header = point_cloud_header_;
      polygon.header = point_cloud_header_;
      if (final_clusters[i]->IsValid())
      {

        in_out_centroids.points.push_back(centroid);

        htcbot_msgs::CloudCluster cloud_cluster;
        final_clusters[i]->ToROSMessage(point_cloud_header_, cloud_cluster);
        in_out_clusters.clusters.push_back(cloud_cluster);
      }

      // add
      bbox_array.boxes.push_back(bounding_box);
    }
    // add
    bbox_array.header = point_cloud_header_;
    pub_bounding_boxs_.publish(bbox_array);
  }

  /**
   * 点云聚类 将输入的三维点云分割成不同的聚类，并提取每个聚类的一些属性，如中心点、尺寸、边界框等
   */
  std::vector<ClusterPtr> LaserEuclideanCluster::clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                                                 autoware_msgs::Centroids &in_out_centroids,
                                                                 double in_max_cluster_distance = 0.5)
  {
    // 创建KD树以进行高效的最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 创建一个2D点云，通过复制输入点云实现
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
    // 将2D点云展平，将z坐标设置为0
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
      // 放到 pcl::EuclideanClusterExtraction 是一个已经平面化的二维点云，这种做法能够带来速度的提升
      cloud_2d->points[i].z = 0;
    }
    // 如果2D点云中有点，则设置KD树的输入云
    if (cloud_2d->points.size() > 0)
      tree->setInputCloud(cloud_2d);
    // 存储局部簇的点云索引的向量
    std::vector<pcl::PointIndices> cluster_indices;
    // perform clustering on 2d cloud  创建欧几里得聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(in_max_cluster_distance); //
    ec.setMinClusterSize(cluster_size_min_);
    ec.setMaxClusterSize(cluster_size_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);
    // use indices on 3d cloud
    //---  3. Color clustered points
    /////////////////////////////////
    unsigned int k = 0;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<ClusterPtr> clusters;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
    // cluster
    //   ROS_INFO("[euclidean_cluster#clusterAndColor] ===> cluster_indices Size: %lu", cluster_indices.size());
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      ClusterPtr cluster(new Cluster());
      // cluster->SetCloud2(in_cloud_ptr, it->indices, point_cloud_header_, k, (int) );
      cluster->SetCloud(in_cloud_ptr, it->indices, point_cloud_header_, k, (int)colors_[k].val[0],
                        (int)colors_[k].val[1],
                        (int)colors_[k].val[2], "", pose_estimation_);
      clusters.push_back(cluster);

      k++;
    }
    // std::cout << "Clusters: " << k << std::endl;
    return clusters;
  }

  void LaserEuclideanCluster::checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                                               float in_merge_threshold)
  {
    // std::cout << "checkAllForMerge" << std::endl;
    std::vector<bool> visited_clusters(in_clusters.size(), false);
    std::vector<bool> merged_clusters(in_clusters.size(), false);
    size_t current_index = 0;
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
      if (!visited_clusters[i])
      {
        visited_clusters[i] = true;
        std::vector<size_t> merge_indices;
        checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
        mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
      }
    }
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
      // check for clusters not merged, add them to the output
      if (!merged_clusters[i])
      {
        out_clusters.push_back(in_clusters[i]);
      }
    }

    // ClusterPtr cluster(new Cluster());
  }

  void LaserEuclideanCluster::checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                                                std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                                                double in_merge_threshold)
  {
    // std::cout << "checkClusterMerge" << std::endl;
    // 返回一个质心坐标
    pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
      if (i != in_cluster_id && !in_out_visited_clusters[i])
      {
        pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
        double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
        if (distance <= in_merge_threshold)
        {
          in_out_visited_clusters[i] = true;
          out_merge_indices.push_back(i);
          // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
          LaserEuclideanCluster::checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
        }
      }
    }
  }

  void LaserEuclideanCluster::mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                                            std::vector<size_t> in_merge_indices, const size_t &current_index,
                                            std::vector<bool> &in_out_merged_clusters)
  {
    // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
    pcl::PointCloud<pcl::PointXYZ> mono_cloud;
    ClusterPtr merged_cluster(new Cluster());
    for (size_t i = 0; i < in_merge_indices.size(); i++)
    {
      sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
      in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    std::vector<int> indices(sum_cloud.points.size(), 0);
    for (size_t i = 0; i < sum_cloud.points.size(); i++)
    {
      indices[i] = i;
    }

    if (sum_cloud.points.size() > 0)
    {
      pcl::copyPointCloud(sum_cloud, mono_cloud);
      merged_cluster->SetCloud(mono_cloud.makeShared(), indices, point_cloud_header_, current_index,
                               (int)colors_[current_index].val[0], (int)colors_[current_index].val[1],
                               (int)colors_[current_index].val[2], "", pose_estimation_);
      out_clusters.push_back(merged_cluster);
    }
  }

  void LaserEuclideanCluster::downsamplePoints(const Mat &src, Mat &dst, size_t count)
  {
    CV_Assert(count >= 2);
    CV_Assert(src.cols == 1 || src.rows == 1);
    CV_Assert(src.total() >= count);
    CV_Assert(src.type() == CV_8UC3);

    dst.create(1, (int)count, CV_8UC3);
    // TODO: optimize by exploiting symmetry in the distance matrix
    Mat dists((int)src.total(), (int)src.total(), CV_32FC1, Scalar(0));
    if (dists.empty())
      std::cerr << "Such big matrix cann't be created." << std::endl;

    for (int i = 0; i < dists.rows; i++)
    {
      for (int j = i; j < dists.cols; j++)
      {
        float dist = (float)norm(src.at<Point3_<uchar>>(i) - src.at<Point3_<uchar>>(j));
        dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
      }
    }

    double maxVal;
    Point maxLoc;
    minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

    dst.at<Point3_<uchar>>(0) = src.at<Point3_<uchar>>(maxLoc.x);
    dst.at<Point3_<uchar>>(1) = src.at<Point3_<uchar>>(maxLoc.y);

    Mat activedDists(0, dists.cols, dists.type());
    Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
    activedDists.push_back(dists.row(maxLoc.y));
    candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

    for (size_t i = 2; i < count; i++)
    {
      activedDists.push_back(dists.row(maxLoc.x));
      candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

      Mat minDists;
      reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
      minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
      dst.at<Point3_<uchar>>((int)i) = src.at<Point3_<uchar>>(maxLoc.x);
    }
  }

  void LaserEuclideanCluster::generateColors(std::vector<Scalar> &colors, size_t count, size_t factor = 100)
  {
    if (count < 1)
      return;

    colors.resize(count);

    if (count == 1)
    {
      colors[0] = Scalar(0, 0, 255); // red
      return;
    }
    if (count == 2)
    {
      colors[0] = Scalar(0, 0, 255); // red
      colors[1] = Scalar(0, 255, 0); // green
      return;
    }

    // Generate a set of colors in RGB space. A size of the set is severel times (=factor) larger then
    // the needed count of colors.
    Mat bgr(1, (int)(count * factor), CV_8UC3);
    randu(bgr, 0, 256);

    // Convert the colors set to Lab space.
    // Distances between colors in this space correspond a human perception.
    Mat lab;
    cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    // Subsample colors from the generated set so that
    // to maximize the minimum distances between each other.
    // Douglas-Peucker algorithm is used for this.
    Mat lab_subset;
    downsamplePoints(lab, lab_subset, count);

    // Convert subsampled colors back to RGB
    Mat bgr_subset;
    cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);

    CV_Assert(bgr_subset.total() == count);
    for (size_t i = 0; i < count; i++)
    {
      Point3_<uchar> c = bgr_subset.at<Point3_<uchar>>((int)i);
      colors[i] = Scalar(c.x, c.y, c.z);
    }
  }

}