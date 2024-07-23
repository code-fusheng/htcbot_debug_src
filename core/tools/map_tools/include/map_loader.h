/*
 * @Author: code-fusheng
 * @Date: 2024-04-19 13:45:23
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 23:16:35
 * @Description: 
 */
#ifndef PCD_MAP_LOADER_H
#define PCD_MAP_LOADER_H

#include <map_common.h>
#include <htcbot_common.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <map_tools/MapLoaderConfig.h>

#include <queue>
#include <thread>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/StatusHtcbotModule.h>


using namespace MapCommonNS;
using namespace HtcbotCommonNS;

namespace MapLoaderNS {

class MapLoader {

public:

    MapLoader();
    ~MapLoader();
    void run();
    void init();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_map_path_conf_;
    ros::Subscriber sub_initialpose_;
    ros::Subscriber sub_current_pose_;

    ros::Publisher pub_dynamic_points_map_;
    ros::Publisher pub_static_points_map_;
    ros::Publisher pub_compress_points_map_;
    ros::Publisher pub_2d_grid_map_;
    ros::Publisher pub_module_status_;
    ros::Publisher pub_occ_grid_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<map_tools::MapLoaderConfig> server_;

    htcbot_msgs::StatusHtcbotModule map_status_;

    double margin_;
    double update_interval_;    // 更新时间间隔
    double update_offset_;      // 更新位移

    bool is_debug_;
    bool is_pub_pgm_;
    double map_resolution_;
    bool is_filter_pass_through_;
    double filter_high_;
    double filter_low_;
    bool is_filter_radius_outlier_;
    double filter_radius_;
    int filter_thre_count_;
    bool is_filter_voxel_leaf_;         // 是否过滤后添加至地图
    double filter_leaf_size_;            // 过滤降采样

    // std::string occ_topic_;
    std::string static_topic_;
    std::string dynamic_topic_;

    ros::Time last_update_time_;

    std::string area_list_filename_;
    std::string dir_static_map_, dir_dynamic_map_;

    MapCommonNS::AreaList default_area_list_;
    MapCommonNS::AreaList cached_area_list_;

    sensor_msgs::PointCloud2 current_dynamic_pcd_map_msg_;

    pcl::PointCloud<pcl::PointXYZI> pcd_map_;    // 点云地图
    pcl::PointCloud<pcl::PointXYZI> pcd_map_filtered_;
    
    void dynamicReconfigureCallback(map_tools::MapLoaderConfig &config, uint32_t level);

    void callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg);
    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &input);
    void callbackCurrentPose(const geometry_msgs::PoseStamped &input);


    void doFilterMap();
    bool publishStaticPcdMap();
    bool publishDynamicPcdMap();
    void convertPointCloudToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud, nav_msgs::OccupancyGrid &out_occ_msg);
    MapCommonNS::AreaList loadAreaList(const std::string &path);
    MapCommonNS::AreaList loadNeedAreaList(const geometry_msgs::Point &point);
    sensor_msgs::PointCloud2 loadPcdByPoint(const geometry_msgs::Point &point);
    bool isInArea(double x, double y, const MapCommonNS::Area &area, double m);
    int isInCachedAreaList(MapCommonNS::Area need_area);
};

}

#endif  // PCD_MAP_LOADER_H

