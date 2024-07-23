
#ifndef WAYPOINT_SAVER_H
#define WAYPOINT_SAVER_H

#include <waypoint_common.h>

#include <ros/ros.h>
#include <ros/timer.h>

#include <dynamic_reconfigure/server.h>
#include <waypoint_tools/WaypointSaverConfig.h>

#include <iostream>
#include <fstream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/ConfWaypointSaver.h>
#include <htcbot_msgs/ModeSwitch.h>

namespace WaypointSaverNS
{

    class WaypointSaver
    {

    public:
        WaypointSaver();
        ~WaypointSaver();
        void run();
        void init();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber sub_map_path_conf_;
        ros::Subscriber sub_waypoint_saver_conf_;
        ros::Subscriber sub_current_pose_base_;
        ros::Subscriber sub_current_pose_gps_;
        ros::Subscriber sub_current_pose_enu_;
        ros::Subscriber sub_current_pose_utm_;
        ros::Subscriber sub_vehicle_status_;
        ros::Subscriber sub_mode_switch_; // temp

        ros::Publisher pub_waypoints_base_;
        ros::Publisher pub_waypoints_gps_;
        ros::Publisher pub_waypoints_enu_;
        ros::Publisher pub_waypoints_utm_;

        // 声明动态重配置服务器
        dynamic_reconfigure::Server<waypoint_tools::WaypointSaverConfig> server_;

        // subscriber
        // 近似时间
        // typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MixPoseSync;
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_current_pose_base_sync_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_current_pose_gps_sync_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_current_pose_utm_sync_;
        // message_filters::Synchronizer<MixPoseSync> sync_mix_pose_;

        // params
        bool is_debug_;
        int switch_to_exp_;
        bool is_filter_;
        double filter_distance_;

        bool save_base_;
        bool save_gps_;
        bool save_enu_;
        bool save_utm_;

        // pre defind

        std::string waypoints_saver_dir_;
        bool f_set_saver_dir_;

        bool f_first_waypoint_;
        int pre_node_id_;
        int node_id_;
        int next_node_id_;

        std::string base_waypoints_filename_;
        std::string gps_waypoints_filename_;
        std::string utm_waypoints_filename_;
        std::string enu_waypoints_filename_;

        geometry_msgs::PoseStamped last_base_waypoint_;
        nav_msgs::Path base_waypoints_;

        geometry_msgs::PoseStamped last_gps_waypoint_;
        nav_msgs::Path gps_waypoints_;

        geometry_msgs::PoseStamped last_utm_waypoint_;
        nav_msgs::Path utm_waypoints_;

        void dynamicReconfigureCallback(waypoint_tools::WaypointSaverConfig &config, uint32_t level);

        void callbackModeSwitch(const htcbot_msgs::ModeSwitch::ConstPtr &msg);
        void callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg);
        void callbackWaypointSaverConf(const htcbot_msgs::ConfWaypointSaver::ConstPtr &msg);
        void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &pose_base_msg);
        void callbackMixPose(const geometry_msgs::PoseStamped::ConstPtr &pose_base_msg,
                             const geometry_msgs::PoseStamped::ConstPtr &pose_gps_msg,
                             const geometry_msgs::PoseStamped::ConstPtr &pose_utm_msg);
        void saveWaypointToPath();
        void writePathToCsv(const std::string &filename, const nav_msgs::Path &path);
    };

}

#endif // WAYPOINT_SAVER_H