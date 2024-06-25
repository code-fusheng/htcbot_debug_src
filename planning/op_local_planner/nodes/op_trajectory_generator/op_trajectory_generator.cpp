#include "op_trajectory_generator.h"

using namespace OpCommonNS;
using namespace OpUtilsNS;

namespace OpTrajectoryGeneratorNS
{
    
OpTrajectoryGenerator::OpTrajectoryGenerator() : nh_private_("~")
{
    mode_switch_ = true;
    switch_status_ = 0;
    f_initialpose_ = false;
    f_current_pose_ = false;
    init();

}

OpTrajectoryGenerator::~OpTrajectoryGenerator() {}

void OpTrajectoryGenerator::init() {

    nh_private_.param<int>("switch_status", switch_status_, 0);
    srv_switch_status_ = nh_.advertiseService("/op_trajectory_generator_node/set_switch_status", &OpTrajectoryGenerator::setSwitchStatusCallback, this);
    OpUtilsNS::GetPlanningParams(nh_, op_planner_params_);

    pub_central_path_section_ = nh_.advertise<htcbot_msgs::Lane>("central_path_section", 1);
    pub_local_trajectories_ = nh_.advertise<htcbot_msgs::LaneArray>("local_trajectories", 1);
    pub_local_trajectories_rviz_ = nh_.advertise<visualization_msgs::MarkerArray>("local_trajectories_rviz", 1);

    sub_op_local_planner_conf_ = nh_.subscribe("/htcbot/op_local_planner/conf", 1, &OpTrajectoryGenerator::callbackConf, this);
    sub_current_pose_ = nh_.subscribe("/current_pose_truth", 10, &OpTrajectoryGenerator::callbackCurrentPose, this); 
    sub_global_planner_path_ = nh_.subscribe("/global_path", 1, &OpTrajectoryGenerator::callbackGlobalPlannerPath, this);
    // TODO sub vehicle_status

}

void OpTrajectoryGenerator::run() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if ((ros::Time::now() - truth_pose_update_time_).toSec() > 0.5) {
            f_current_pose_ = false;
        }
        if (switch_status_ && f_current_pose_ && op_global_paths_.size() > 0) {
            // ROS_INFO("[op_trajectory_generator] op_global_paths_.size %d", op_global_paths_.size());
            // 清空存储全局路径部分的容器
            op_global_paths_sections_.clear();
            // 对每个全局路径进行处理
            for (size_t i = 0; i < op_global_paths_.size(); i++) {
                op_center_trajectory_smoothed_.clear();
                // 提取全局路径中的局部路径段
                OpUtilsNS::ExtractPartFromPointToDistance(op_global_paths_[i], 
                                          op_current_pose_, 
                                          op_planner_params_.microPlanDistance,
                                          op_planner_params_.pathDensity, 
                                          op_center_trajectory_smoothed_);
                op_global_paths_sections_.push_back(op_center_trajectory_smoothed_);
                // ROS_WARN("[op_trajectory_generator] op_center_trajectory_smoothed_ size() %d", op_center_trajectory_smoothed_.size());
            }
			// // 定义一个用于调试的存储采样点的容器
			std::vector<OpCommonNS::WayPoint> sampled_points_debug;
            OpUtilsNS::GenerateRunoffTrajectory(op_global_paths_sections_,
                op_current_pose_,
                1,
                op_planner_params_.microPlanDistance,
                op_planner_params_.carTipMargin,
                op_planner_params_.rollInMargin,
                op_planner_params_.rollInSpeedFactor,
                op_planner_params_.pathDensity,
                op_planner_params_.rollOutDensity,
                op_planner_params_.rollOutNumber,
                op_planner_params_.smoothingDataWeight,
                op_planner_params_.smoothingSmoothWeight,
                op_planner_params_.smoothingToleranceError,
                op_rollouts_,
                sampled_points_debug
            );
            // 发布本地轨迹
            htcbot_msgs::LaneArray local_lanes;
            // 遍历生成的轨迹，将其转换为Autoware中的Lane消息，并添加到local_lanes中
			for(unsigned int i=0; i < op_rollouts_.size(); i++)
			{
				for(unsigned int j=0; j < op_rollouts_.at(i).size(); j++)
				{
					htcbot_msgs::Lane local_lane;
					ConvertLocalLane2MsgLane(op_rollouts_.at(i).at(j), local_lane);
					local_lane.cost = 0;
					local_lane.is_blocked = false;
					local_lane.lane_index = i;
                    local_lane.lane_id = i;
					local_lanes.lanes.push_back(local_lane);
				}
			}
			// 发布局部轨迹消息
			pub_local_trajectories_.publish(local_lanes);
            // 定义存储可视化轨迹的消息
		    visualization_msgs::MarkerArray marker_rollouts;
            TrajectoriesToMarkers(op_rollouts_, marker_rollouts);
            pub_local_trajectories_rviz_.publish(marker_rollouts);

            htcbot_msgs::Lane central_path_section;
            for (size_t m = 0; m < op_global_paths_sections_.size(); m++) {
                for (size_t im = 0; im < op_global_paths_sections_[m].size(); im++) {
                    htcbot_msgs::Waypoint wp;
                    wp.pose.pose.position.x = op_global_paths_sections_[m][im].pos.x;
                    wp.pose.pose.position.y = op_global_paths_sections_[m][im].pos.y;
                    wp.pose.pose.position.z = op_global_paths_sections_[m][im].pos.z;
                    wp.yaw = op_global_paths_sections_[m][im].pos.yaw;
                    central_path_section.waypoints.push_back(wp);
                }
            }
            pub_central_path_section_.publish(central_path_section);
        }
        loop_rate.sleep();
    }
}

bool OpTrajectoryGenerator::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res) {
    ROS_INFO("[py_demo_node] ===> req: %d", req.switch_to); 
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_; 
    return true;                      
}

void OpTrajectoryGenerator::callbackConf(const htcbot_msgs::ConfOpLocalPlanner::ConstPtr &msg) {
    if (mode_switch_ != msg->switch_to) {
        mode_switch_ = msg->switch_to;   
    }
    if (mode_switch_) {
        mode_switch_ = msg->switch_to;
        op_planner_params_.carTipMargin = msg->sampling_tip_margin;
        op_planner_params_.rollInMargin = msg->sampling_out_margin;
        op_planner_params_.rollInSpeedFactor = msg->sampling_speed_factor;
        op_planner_params_.enableHeadingSmoothing = msg->enable_heading_smoothing;
        
        op_planner_params_.microPlanDistance = msg->max_local_plan_distance;
        op_planner_params_.horizontalSafetyDistance = msg->horizontal_safety_distance;
        op_planner_params_.verticalSafetyDistance = msg->vertical_safety_distance;

        op_planner_params_.rollOutDensity = msg->rollouts_density;
        op_planner_params_.rollOutNumber = msg->rollouts_number;

        OpUtilsNS::UpdatePlanningParams(nh_, op_planner_params_);
    }
    nh_.setParam("/op_common_params_node/modeSwitch", mode_switch_);
}

void OpTrajectoryGenerator::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    op_current_pose_ = OpCommonNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    f_current_pose_ = true;
    truth_pose_update_time_ = ros::Time::now();
}

void OpTrajectoryGenerator::callbackGlobalPlannerPath(const htcbot_msgs::Lane::ConstPtr& msg) {
    // check global path waypoint
    if (msg->waypoints.size() > 0) {
        // 清除之前保存的全局路径
        op_global_paths_.clear();
        // 创建一个用于保存单个路径的std::vector对象
        std::vector<OpCommonNS::WayPoint> single_path;
        OpUtilsNS::ConvertMsgLane2LocalLane(*msg, single_path);
        OpUtilsNS::CalcAngleAndCost(single_path);
        op_global_paths_.push_back(single_path);
    }
}

} // namespace name
