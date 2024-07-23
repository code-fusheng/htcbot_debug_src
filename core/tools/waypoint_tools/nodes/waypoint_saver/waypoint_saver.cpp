/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-26 00:28:21
 * @Description:
 */
#include "waypoint_saver.h"

using namespace WaypointCommonNS;

namespace WaypointSaverNS
{

    WaypointSaver::WaypointSaver() : nh_private_("~")
    {
        is_debug_ = 0;
        f_set_saver_dir_ = false;
        f_first_waypoint_ = true;
        switch_to_exp_ = 0;
        base_waypoints_filename_ = "lane_1.csv";
        gps_waypoints_filename_ = "gps_lane_1.csv";
        utm_waypoints_filename_ = "utm_lane_1.csv";
        pre_node_id_ = 1;
        node_id_ = 1;
        next_node_id_ = 2;
    }

    WaypointSaver::~WaypointSaver()
    {
    }

    void WaypointSaver::run()
    {
        init();
    }

    void WaypointSaver::init()
    {
        nh_private_.param<int>("/waypoint_saver_node/switch_to_exp", switch_to_exp_, 0);
        nh_private_.param<bool>("/waypoint_saver_node/is_filter", is_filter_, true);
        nh_private_.param<double>("/waypoint_saver_node/filter_distance", filter_distance_, 0.1);
        nh_private_.param<bool>("/waypoint_saver_node/save_base", save_base_, true);
        nh_private_.param<bool>("/waypoint_saver_node/save_gps", save_gps_, true);
        nh_private_.param<bool>("/waypoint_saver_node/save_enu", save_enu_, false);
        nh_private_.param<bool>("/waypoint_saver_node/save_utm", save_utm_, false);

        server_.setCallback(boost::bind(&WaypointSaver::dynamicReconfigureCallback, this, _1, _2));

        sub_map_path_conf_ = nh_.subscribe("/htcbot/map_path_conf", 10, &WaypointSaver::callbackMapPathConf, this);
        sub_waypoint_saver_conf_ = nh_.subscribe("/htcbot/waypoint_saver/conf", 10, &WaypointSaver::callbackWaypointSaverConf, this);
        // old
        sub_mode_switch_ = nh_.subscribe("/htcbot/mode_switch", 10, &WaypointSaver::callbackModeSwitch, this);

        pub_waypoints_base_ = nh_.advertise<nav_msgs::Path>("/waypoint_saver/base_waypoints", 10);
        pub_waypoints_gps_ = nh_.advertise<nav_msgs::Path>("/waypoint_saver/gps_waypoints", 10);
        pub_waypoints_utm_ = nh_.advertise<nav_msgs::Path>("/waypoint_saver/utm_waypoints", 10);

        base_waypoints_.header.frame_id = "map";
        gps_waypoints_.header.frame_id = "map";
        utm_waypoints_.header.frame_id = "map";

        // sub_current_pose_base_sync_.subscribe(nh_, "/current_pose", 10);
        // sub_current_pose_gps_sync_.subscribe(nh_, "/gnss/gps_pose", 10);

        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> ApproSynch;
        message_filters::Synchronizer<ApproSynch> appro_synch(ApproSynch(100), sub_current_pose_base_sync_, sub_current_pose_gps_sync_, sub_current_pose_utm_sync_);
        appro_synch.registerCallback(&WaypointSaver::callbackMixPose, this);

        ros::spin();
    }

    void WaypointSaver::dynamicReconfigureCallback(waypoint_tools::WaypointSaverConfig &config, uint32_t level)
    {
        is_debug_ = config.is_debug;
    }

    void WaypointSaver::callbackModeSwitch(const htcbot_msgs::ModeSwitch::ConstPtr &msg)
    {
        if (msg->mode == htcbot_msgs::ModeSwitch::POSE_RECORD)
        {
            switch_to_exp_ = msg->switch_to;
            nh_private_.setParam("/waypoint_saver_node/switch_to_exp", msg->switch_to);
            if (switch_to_exp_ == 1)
            {
                if (save_base_ && save_gps_ && save_utm_)
                {
                    // 近似时间
                    // ROS_ERROR("[waypoint_saver] ==> callbackModeSwitch");
                    sub_current_pose_base_sync_.subscribe(nh_, "/current_pose", 100);
                    sub_current_pose_gps_sync_.subscribe(nh_, "/gnss/gps_pose", 100);
                    sub_current_pose_utm_sync_.subscribe(nh_, "/gnss/utm_pose", 100);
                }
                else if (save_base_)
                {
                    sub_current_pose_base_ = nh_.subscribe("/current_pose", 10, &WaypointSaver::callbackCurrentPose, this);
                }
                else if (save_gps_)
                {
                    // sub_current_pose_gps_ = nh_.subscribe("/gnss/gps_pose", 50, &WaypointSaver::callbackGpsPose, this);
                }
                else if (save_utm_)
                {
                    // sub_current_pose_utm_ = nh_.subscribe("/gnss/utm_pose", 50, &WaypointSaver::callbackUtmPose, this);
                }
            }
            else
            {
                //  保存
                sub_current_pose_base_.shutdown();
                ROS_INFO("[waypoint_saver] ==> waypoints saver final size: %zu", base_waypoints_.poses.size());
                saveWaypointToPath();
            }
        }
    }

    void WaypointSaver::callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg)
    {
        ROS_INFO("[waypoint_saver] ==> route_path: %s", msg->route_path.c_str());
        waypoints_saver_dir_ = msg->route_path;
        f_set_saver_dir_ = true;
    }

    void WaypointSaver::callbackWaypointSaverConf(const htcbot_msgs::ConfWaypointSaver::ConstPtr &msg)
    {
        switch_to_exp_ = msg->switch_to_exp;
        is_filter_ = msg->is_filter;
        filter_distance_ = msg->filter_distance;
        save_base_ = msg->save_base;
        save_gps_ = msg->save_gps;
        save_enu_ = msg->save_enu;
        save_utm_ = msg->save_utm;
        nh_private_.setParam("/waypoint_saver_node/switch_to_exp", msg->switch_to_exp);
        if (msg->switch_to_exp == 1)
        {
            sub_current_pose_base_ = nh_.subscribe("/current_pose", 10, &WaypointSaver::callbackCurrentPose, this);
        }
        else
        {
            //  保存
            sub_current_pose_base_.shutdown();
        }
    }

    void WaypointSaver::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &pose_base_msg)
    {
        if (!f_set_saver_dir_)
        {
            return;
        }
        if (f_first_waypoint_)
        {
            base_waypoints_.poses.push_back(*pose_base_msg);
            last_base_waypoint_ = *pose_base_msg;
            f_first_waypoint_ = false;
        }
        else
        {
            if (is_filter_ && filter_distance_ != 0)
            {
                double dist = double(distance2points(last_base_waypoint_.pose.position, pose_base_msg->pose.position));
                if (dist >= filter_distance_)
                {
                    base_waypoints_.poses.push_back(*pose_base_msg);
                    last_base_waypoint_ = *pose_base_msg;
                }
            }
        }
        ROS_INFO("[waypoint_saver] ==> waypoints size: %zu", base_waypoints_.poses.size());
        pub_waypoints_base_.publish(base_waypoints_);
    }

    void WaypointSaver::callbackMixPose(const geometry_msgs::PoseStamped::ConstPtr &pose_base_msg,
                                        const geometry_msgs::PoseStamped::ConstPtr &pose_gps_msg,
                                        const geometry_msgs::PoseStamped::ConstPtr &pose_utm_msg)
    {
        ros::Duration time_diff = pose_gps_msg->header.stamp - pose_base_msg->header.stamp;
        if (!f_set_saver_dir_)
        {
            return;
        }
        if (f_first_waypoint_)
        {
            base_waypoints_.poses.push_back(*pose_base_msg);
            last_base_waypoint_ = *pose_base_msg;
            gps_waypoints_.poses.push_back(*pose_gps_msg);
            last_gps_waypoint_ = *pose_gps_msg;
            utm_waypoints_.poses.push_back(*pose_utm_msg);
            last_utm_waypoint_ = *pose_utm_msg;
            f_first_waypoint_ = false;
        }
        else
        {
            if (is_filter_ && filter_distance_ != 0)
            {
                double dist = double(distance2points(last_base_waypoint_.pose.position, pose_base_msg->pose.position));
                // TODO 增加 GPS dist 判断
                if (dist >= filter_distance_)
                {
                    base_waypoints_.poses.push_back(*pose_base_msg);
                    last_base_waypoint_ = *pose_base_msg;
                    gps_waypoints_.poses.push_back(*pose_gps_msg);
                    last_gps_waypoint_ = *pose_gps_msg;
                    utm_waypoints_.poses.push_back(*pose_utm_msg);
                    last_utm_waypoint_ = *pose_utm_msg;
                }
            }
        }
        pub_waypoints_base_.publish(base_waypoints_);
        pub_waypoints_gps_.publish(gps_waypoints_);
        pub_waypoints_utm_.publish(utm_waypoints_);
        if (is_debug_)
        {
            std::cout << "waypoint_saver-----------------------------------------------------------------" << std::endl;
            std::cout << "Base Pose Time: " << pose_base_msg->header.stamp.toSec() << std::endl;
            std::cout << "Gps Pose Time: " << pose_gps_msg->header.stamp.toSec() << std::endl;
            std::cout << "Utm Pose Time: " << pose_utm_msg->header.stamp.toSec() << std::endl;
            std::cout << "Time Difference: " << time_diff.toSec() << std::endl;
            std::cout << "Base Waypoints Size: " << base_waypoints_.poses.size() << std::endl;
            std::cout << "Gps Waypoints Size: " << gps_waypoints_.poses.size() << std::endl;
            std::cout << "Utm Waypoints Size: " << utm_waypoints_.poses.size() << std::endl;
            std::cout << "waypoint_saver-----------------------------------------------------------------" << std::endl;
        }
    }

    void WaypointSaver::saveWaypointToPath()
    {
        if (save_base_)
        {
            std::string full_path = waypoints_saver_dir_ + "/" + base_waypoints_filename_;
            ROS_INFO("[waypoint_saver] ==> Saving BASE Waypoints Size = %zu", base_waypoints_.poses.size());
            writePathToCsv(full_path, base_waypoints_);
        }
        if (save_gps_)
        {
            std::string full_path = waypoints_saver_dir_ + "/" + gps_waypoints_filename_;
            ROS_INFO("[waypoint_saver] ==> Saving GPS Waypoints Size = %zu", gps_waypoints_.poses.size());
            writePathToCsv(full_path, gps_waypoints_);
        }
        if (save_utm_)
        {
            std::string full_path = waypoints_saver_dir_ + "/" + utm_waypoints_filename_;
            ROS_INFO("[waypoint_saver] ==> Saving GPS(UTM) Waypoints Size = %zu", utm_waypoints_.poses.size());
            writePathToCsv(full_path, utm_waypoints_);
        }
    }

    void WaypointSaver::writePathToCsv(const std::string &filename, const nav_msgs::Path &path)
    {
        if (path.poses.empty())
        {
            ROS_WARN("[waypoint_saver] ==> Cannot Save The Path, Because The Path Size Is 0");
            return;
        }
        std::ofstream file(filename);
        if (!file.is_open())
        {
            ROS_ERROR_STREAM("[waypoint_saver] ==> Failed To Open File: " << filename);
            return;
        }
        file << 1 << std::endl;                 // id
        file << path.poses.size() << std::endl; // length
        file << 1 << std::endl;                 // reverse
        file << 1 << std::endl;                 // pre_node_id
        file << 2 << std::endl;                 // next_node_id

        for (const auto &pose_stamped : path.poses)
        {
            const auto &pose = pose_stamped.pose;

            file << std::fixed << std::setprecision(7) << pose.position.x << ", ";
            file << std::fixed << std::setprecision(7) << pose.position.y << ", ";
            file << std::fixed << std::setprecision(7) << pose.position.z << ", ";
            file << std::fixed << std::setprecision(7) << pose.orientation.x << ", ";
            file << std::fixed << std::setprecision(7) << pose.orientation.y << ", ";
            file << std::fixed << std::setprecision(7) << pose.orientation.z << ", ";
            file << std::fixed << std::setprecision(7) << pose.orientation.w << std::endl;
        }

        file.close();
        ROS_INFO_STREAM("[waypoint_saver] ==> Saved " << path.poses.size() << " Points To " << filename);
    }

}