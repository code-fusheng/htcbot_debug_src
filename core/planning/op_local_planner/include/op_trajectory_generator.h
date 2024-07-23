/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-04-02 14:48:42
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 15:28:47
 * @FilePath: /src/core/planning/op_local_planner/include/op_trajectory_generator.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef OP_TRAJECTORY_GENERATOR_H
#define OP_TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include "op_common.h"
#include "op_utils.h"

#include <htcbot_msgs/LaneArray.h>
#include <htcbot_msgs/ConfOpLocalPlanner.h>
#include <htcbot_msgs/ConfVehicle.h>

#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>


using namespace OpCommonNS;
using namespace OpUtilsNS;

namespace OpTrajectoryGeneratorNS {

class OpTrajectoryGenerator {

public:
    OpTrajectoryGenerator();
    ~OpTrajectoryGenerator();
    void run();
    void init();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceServer srv_switch_status_;

    OpCommonNS::WayPoint op_current_pose_;
    OpCommonNS::WayPoint op_init_pose_;

    OpCommonNS::PlanningParams op_planner_params_;

    OpCommonNS::VehicleState op_vehicle_state_;

    std::vector<std::vector<OpCommonNS::WayPoint>> op_global_paths_;
    std::vector<std::vector<OpCommonNS::WayPoint>> op_global_paths_sections_;

    std::vector<OpCommonNS::WayPoint> op_center_trajectory_smoothed_;

    std::vector<std::vector<std::vector<OpCommonNS::WayPoint>>> op_rollouts_;

    int switch_status_;
    bool mode_switch_;
    bool f_initialpose_;
    bool f_current_pose_;
    bool f_vehicle_status_;
    ros::Time truth_pose_update_time_;

    ros::Subscriber sub_initialpose_;
	ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_global_planner_path_;
    ros::Subscriber sub_op_local_planner_conf_;

    ros::Publisher pub_central_path_section_;
    ros::Publisher pub_local_trajectories_;
    ros::Publisher pub_local_trajectories_rviz_;

    void callbackConf(const htcbot_msgs::ConfOpLocalPlanner::ConstPtr &msg);
    bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);
    void callbackInitPose();

    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void callbackGlobalPlannerPath(const htcbot_msgs::Lane::ConstPtr& msg);

};
}

#endif  // OP_TRAJECTORY_GENERATOR_H


