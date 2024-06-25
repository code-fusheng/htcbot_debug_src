/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-04-02 14:49:43
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-21 12:56:00
 * @FilePath: /src/core/planning/op_local_planner/include/op_trajectory_evaluator.h
 */
#ifndef OP_TRAJECTORY_EVALUATOR_H
#define OP_TRAJECTORY_EVALUATOR_H

#include "op_common.h"
#include <htcbot_common.h>
#include "op_utils.h"

#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <op_local_planner/OpLocalPlannerConfig.h>

#include <can_msgs/vehicle_status.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <htcbot_msgs/StatusHtcbotModule.h>
#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>
#include <htcbot_msgs/LaneArray.h>
#include <htcbot_msgs/ConfOpLocalPlanner.h>
#include <htcbot_msgs/ConfVehicle.h>
#include <htcbot_msgs/DetectedObject.h>
#include <htcbot_msgs/DetectedObjectArray.h>

using namespace OpCommonNS;
using namespace HtcbotCommonNS;
using namespace OpUtilsNS;


namespace OpTrajectoryEvaluatorNS
{

class OpTrajectoryEvaluator {

public:
    OpTrajectoryEvaluator();
    ~OpTrajectoryEvaluator();
    void run();
    void init();

private:

    ros::NodeHandle nh;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_module_status_;

    ros::ServiceServer srv_switch_status_;

    htcbot_msgs::StatusHtcbotModule module_status_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<op_local_planner::OpLocalPlannerConfig> server_;

    std::string lidar_frame;
    std::string map_frame;
    tf::TransformListener transform_listener;
    tf::StampedTransform lidar_map_tf;
    tf::StampedTransform base_map_tf;

    OpCommonNS::WayPoint op_current_pose_;
    std::vector<OpCommonNS::DetectedObject> op_detected_objects_;
    std::vector<std::vector<OpCommonNS::WayPoint> > op_generated_rollouts_;
    std::vector<OpCommonNS::WayPoint> op_central_path_section_;
    std::vector<OpCommonNS::TrajectoryCost> op_trajectory_costs_;   
    std::vector<OpCommonNS::WayPoint> op_all_contour_points_;   // 存储所有轮廓点的向量
    OpCommonNS::PolygonShape op_safety_border_;
  	OpCommonNS::PlanningParams op_planner_params_;
    OpCommonNS::CAR_BASIC_INFO op_car_info_;
    OpCommonNS::VehicleState op_vehicle_status_;

  	visualization_msgs::MarkerArray op_collisions_dummy_;
	visualization_msgs::MarkerArray op_collisions_actual_;
    geometry_msgs::PolygonStamped safety_area_;

    int op_prev_cost_index_;
	int op_prev_index_;
    double op_weight_priority_;
	double op_weight_transition_;
	double op_weight_long_;
	double op_weight_lat_;
	double op_weight_lane_change_;
	double op_lateral_skip_distance_;
	double op_collision_time_diff_;
    
    int switch_status_;
    bool is_debug_;
    bool mode_switch_;
    bool f_current_pose_;
    bool f_detected_objects_;
    bool f_lidar_map_tf_;
    bool f_generated_rollouts_;
    ros::Time truth_pose_update_time_;

	ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_local_trajectories_;
    ros::Subscriber sub_detected_objects_;
    ros::Subscriber sub_op_local_planner_conf_;
    ros::Subscriber sub_central_path_section_;
    ros::Subscriber sub_vehicle_status_;

    ros::Publisher pub_trajectory_cost_;
    ros::Publisher pub_trajectory_best_;
    ros::Publisher pub_trajectory_best_rviz_;
	ros::Publisher pub_local_weighted_trajectories_rviz_;
	ros::Publisher pub_local_weighted_trajectories_;
    ros::Publisher pub_collision_points_rviz_;
    ros::Publisher pub_safety_border_rviz_;
    ros::Publisher pub_safety_area_rviz_;
    ros::Publisher pub_central_path_section_rviz_;
    
    void dynamicReconfigureCallback(op_local_planner::OpLocalPlannerConfig &config, uint32_t level);

    void callbackConf(const htcbot_msgs::ConfOpLocalPlanner::ConstPtr &msg);
    bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);
    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackLocalTrajectories(const htcbot_msgs::LaneArray::ConstPtr& msg);
	void callbackDetectedObjects(const htcbot_msgs::DetectedObjectArray::ConstPtr& msg);
    void callbackCentralPathSection(const htcbot_msgs::Lane::ConstPtr& msg);
    void callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr& msg);

    void setDetectedObject(const htcbot_msgs::DetectedObject& det_obj, OpCommonNS::DetectedObject& obj);
    tf::StampedTransform findTransform4Lidar2Map(const std::string& map_frame, const std::string lidar_frame);
    tf::StampedTransform findTansform4Source2Target(const std::string& source_frame, const std::string target_frame);
    OpCommonNS::GPSPoint transformPoint(const OpCommonNS::GPSPoint& in_point_pos, const tf::StampedTransform& in_transform);

    // 计算轨迹代价函数（静态）
    OpCommonNS::TrajectoryCost doEvaluatorTrajectoryCostStatic(const vector<vector<OpCommonNS::WayPoint> >& rollOuts,
            const vector<OpCommonNS::WayPoint>& totalPaths, 
            const OpCommonNS::WayPoint& currPose,
            const OpCommonNS::PlanningParams& params, 
            const OpCommonNS::CAR_BASIC_INFO& carInfo, 
            const OpCommonNS::VehicleState& vehicleState,
            const std::vector<OpCommonNS::DetectedObject>& obj_list, 
            const int& iCurrentIndex = -1);

    void calculateLateralAndLongitudinalCostsStaticPro(vector<OpCommonNS::TrajectoryCost>& trajectoryCosts,
		const vector<vector<OpCommonNS::WayPoint> >& rollOuts, 
        const vector<OpCommonNS::WayPoint>& centerPath,
		const OpCommonNS::WayPoint& currPose, 
        vector<OpCommonNS::WayPoint>& contourPoints, 
        const OpCommonNS::PlanningParams& params,
		const OpCommonNS::CAR_BASIC_INFO& carInfo, 
        const OpCommonNS::VehicleState& vehicleState);

    void calculateLateralAndLongitudinalCostsStatic(vector<OpCommonNS::TrajectoryCost>& trajectoryCosts,
		const vector<vector<OpCommonNS::WayPoint> >& rollOuts, 
        const vector<OpCommonNS::WayPoint>& totalPaths,
		const OpCommonNS::WayPoint& currPose, 
        vector<OpCommonNS::WayPoint>& contourPoints, 
        const OpCommonNS::PlanningParams& params,
		const OpCommonNS::CAR_BASIC_INFO& carInfo, 
        const OpCommonNS::VehicleState& vehicleState);

    void normalizeCosts(vector<OpCommonNS::TrajectoryCost>& trajectoryCosts);
    void visualContourPoints(const std::vector<OpCommonNS::WayPoint>& in_points);

    void visualSafetyArea(const OpCommonNS::PolygonShape& safety_border);

};

}

#endif  // OP_TRAJECTORY_EVALUATOR_H