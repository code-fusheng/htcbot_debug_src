#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS includes
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/service.h>
#include <queue>
#include <string>

#include <htcbot_msgs/ModeSwitch.h>
#include <htcbot_msgs/Lane.h>
#include <htcbot_msgs/LaneArray.h>
#include <htcbot_msgs/Waypoint.h>
#include <htcbot_msgs/ConfPurePursuit.h>
#include <htcbot_msgs/ControlCommandStamped.h>


namespace PurePursuitNS {
class PurePursuit {
   private:
    // handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher pub_ctl, pub_target;
    ros::Publisher pub_path;
    ros::Publisher pub_control;
    ros::Publisher pub_pure_pursuit_finished;
    ros::Publisher pub_reversed_current_pose;
    ros::Publisher pub_closet_index;

    ros::Publisher pub_trajectory_circle;

    // subscriber
    ros::Subscriber sub_avoid_type_switch;
    ros::Subscriber sub_currentpose, sub_lane, sub_speed;
    ros::Subscriber sub_safe_waypoints;  // For A* 局部路径规划
    ros::Subscriber sub_best_local_trajectory;  // For 基于采样的open planner局部路径规划


    ros::Publisher pubNextWaypoint_;
    ros::Publisher pubNextTarget_;
    ros::Publisher pubSearchCircle_;
    ros::Publisher pubTrajectoryCircle_;
    ros::Subscriber confPurePursuitSub_;    // htcbot 参数配置

    ros::ServiceServer ppt_service, avoid_service;

    // constant
    const int LOOP_RATE_;  // processing frequency
    const double curvature_MIN_;
    double const_velocity_;
    double const_lookahead_distance_;  // meter

    // variables
    bool is_linear_interpolation_;
    bool is_waypoint_set_;
    bool is_pose_set_;
    bool is_const_lookahead_dis_;
    bool is_const_speed_command_;
    double command_linear_velocity_;
    int next_waypoint_number_;
    int end_waypoint_index;
    // bool use_point_speed_limit;  // 使用点上的速度限制
    geometry_msgs::Point next_target_position_;

    geometry_msgs::Pose current_pose_;

    double cur_speed = 0.5;
    ros::Time last_cpose_update_time = ros::Time::now();

    std::vector<htcbot_msgs::Waypoint> current_waypoints_;
    double wheel_base_;
    double lookahead_distance_;
    double lookahead_distance_ratio_;  // lookahead_distance_ = cur_speed * lookahead_distance_ratio_, longer lookahead_distance_ = more stable
    double minimum_lookahead_distance_;

    bool is_last_point;

    bool is_in_cross;
    bool is_not_lane;
    double cross_lookahead_distance;
    double lane_lookahead_distance;

    int pre_index;
    int search_start_index;
    int clearest_points_index;

    float search_radius;
    bool cross_in;
    bool cross_out;
    bool almost_reach;
    int lock_index;
    int save_index;
    int current_pose_closet_index;

    double lane_speed_limit;
    double cross_speed_limit;

    double pre_pre_steer = 0;
    double pre_steer = 0;

    std::string current_pose_type = "null";
    std::string automotive_mode = "";
    std::string input_current_pose_topic = "";
    double stop_distance = 1.0;
    double step_size = 0.1;

    bool is_target_front;

    bool current_avoid_type_switch = htcbot_msgs::ModeSwitch::ON;

    bool is_debug = false;

    // PS: code-fusheng 20240410
    htcbot_msgs::Waypoint next_waypoint_;
    int next_traj_index_;
    bool f_is_blocked_;

   public:
    PurePursuit();

    ~PurePursuit();

    void run();

    // callbacks
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

    void callbackFromWayPoints(const htcbot_msgs::Lane &msg);
    void callbackFromSafeWayPoints(const nav_msgs::PathConstPtr &msg);
    void callbackFromBestLocalTrajectory(const htcbot_msgs::Lane &msg);

    void callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg);

    void __handle_followed_path(htcbot_msgs::Lane msg);

    void handle_avoid_type_switch(const htcbot_msgs::ModeSwitchConstPtr &msg);

    void callbackPurePursuitConf(const htcbot_msgs::ConfPurePursuit::ConstPtr &msg);

    // initializer
    void initForROS();

    void reset();

    // functions

    void visualInRviz();

    bool computeCurvature(double *output_curvature);

    double calcCurvature(geometry_msgs::Point target);

    bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target);

    void getNextWaypoint();

    void publishControlCommandStamped(const bool &can_get_curvature, const double &curvature);

    void publishBrakeCommand();

    double computeLookaheadDistance() const;

    double computeCommandVelocity();

    geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c);

    tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);

    tf::Vector3 point2vector(geometry_msgs::Point point);

    bool do_ending_process(can_msgs::ecu msg_ecu);

    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c);

    double deg2rad(double deg) { return deg * M_PI / 180; }

    std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target, geometry_msgs::Pose current_pose);

    double calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose);

    visualization_msgs::Marker displayNextWaypoint(htcbot_msgs::Waypoint waypoint);

    visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target);

    visualization_msgs::Marker displaySearchRadius(geometry_msgs::Point current_pose, double search_radius);

    visualization_msgs::Marker displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array);

    htcbot_msgs::Waypoint getPoseOfNextWaypoint();

    geometry_msgs::Point getPoseOfNextTarget() const
    {
        return next_target_position_;
    }

    geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree);

    geometry_msgs::Pose getCurrentPose() const
    {
        return current_pose_;
    }

    double getLookaheadDistance() const
    {
        return lookahead_distance_;
    }

};

}  // namespace PurePursuitNS

#endif  // PURE_PURSUIT_H
