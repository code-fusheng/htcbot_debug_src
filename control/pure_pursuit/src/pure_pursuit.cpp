#include "pure_pursuit.h"

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <cmath>
#include <std_msgs/Int32.h>

namespace PurePursuitNS {
// Constructor
PurePursuit::PurePursuit()
    : private_nh_("~"),
      LOOP_RATE_(25),
      curvature_MIN_(1 / 9e10),
      is_waypoint_set_(false),
      is_pose_set_(false),
      command_linear_velocity_(0),
      next_waypoint_number_(-1),
      end_waypoint_index(-1),
      lookahead_distance_(0),
      is_last_point(false),
      is_in_cross(false),
      is_not_lane(true),
      pre_index(0),
      search_start_index(0),
      clearest_points_index(-1),
      search_radius(2),
      cross_in(false),
      cross_out(false),
      almost_reach(false),
      lock_index(-1),
      save_index(-1),
      current_pose_closet_index(0),
      next_traj_index_(-1),
      f_is_blocked_(false),
      is_target_front(true) {
    // 设置了订阅，发布的话题消息， 读取了launch文件中的配置变量
    initForROS();
}

// Destructor
PurePursuit::~PurePursuit() {}

void PurePursuit::initForROS() {
    // ros parameter settings
    private_nh_.param("is_debug", is_debug, bool(false));
    // 是否线性插值
    private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(false));
    // 车身轴距
    private_nh_.param("wheel_base", wheel_base_, double(0.5));
    // 前视距离比率
    private_nh_.param("lookahead_distance_ratio", lookahead_distance_ratio_, double(6.0));  //假设速度的单位是m/s, 1M/S的预瞄距离是4m
    // 最小前视距离
    private_nh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, double(2));
    // 常规前视距离
    private_nh_.param("const_lookahead_distance", const_lookahead_distance_, double(5));
    // 是否定量前视距离
    private_nh_.param("is_const_lookahead_dis", is_const_lookahead_dis_, true);
    // 是否定量速度指令
    private_nh_.param("is_const_speed_command", is_const_speed_command_, false);
    // cross 路线前视距离
    private_nh_.param("cross_lookahead_distance", cross_lookahead_distance, double(2));
    // lane 路线前视距离
    private_nh_.param("lane_lookahead_distance", lane_lookahead_distance, double(9));
    // 恒定速度
    private_nh_.param("const_velocity", const_velocity_, double(5.4));  // km/h
    // lane 限速
    private_nh_.param("lane_speed_limit", lane_speed_limit, double(3.6));
    // cross 限速
    private_nh_.param("cross_speed_limit", cross_speed_limit, double(3.6 * 0.5));
    // 停止距离
    private_nh_.param("stop_distance", stop_distance, double(0.5));
    nh_.param("automotive_mode", automotive_mode, std::string(""));

    input_current_pose_topic = "/current_pose";

    // setup publisher
    // pub_ctl = nh_.advertise<geometry_msgs::Twist>("/ecu_raw", 10);
    // 目标航迹点
    pub_target = nh_.advertise<visualization_msgs::MarkerArray>("/pure_pursuit/target_waypoint", 10);
    // 遵循的路径
    pub_path = nh_.advertise<nav_msgs::Path>("/pure_pursuit/followed_path", 10);
    // 发布的控制指令
    pub_control = nh_.advertise<can_msgs::ecu>("/pure_pursuit/ecu", 5);
    // 任务是否完成
    pub_pure_pursuit_finished = nh_.advertise<std_msgs::Bool>("/pure_pursuit/task_finished", 5);
    pub_reversed_current_pose = nh_.advertise<geometry_msgs::PoseStamped>("/ppt/current_pose_reversed", 5);

    // code add 20240207
    pub_trajectory_circle = nh_.advertise<visualization_msgs::Marker>("/trajectory_circle_mark", 0);

    sub_avoid_type_switch = nh_.subscribe("/htcbot/mode_switch", 10, &PurePursuit::handle_avoid_type_switch, this);

    // 全局路径
    sub_lane = nh_.subscribe("/global_path", 10, &PurePursuit::callbackFromWayPoints, this);
    // 当前位姿
    sub_currentpose = nh_.subscribe(input_current_pose_topic, 10, &PurePursuit::callbackFromCurrentPose, this);
    // 车辆状态信息
    sub_speed = nh_.subscribe("/vehicle_status", 10, &PurePursuit::callbackFromCurrentVelocity, this);

    // rviz 的下一个路径点标志
    pubNextWaypoint_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
    // rviz 的下一个目标点标志
    // pubNextTarget_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
    // 搜索区域
    pubSearchCircle_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
    // 运动轨迹示意
    pubTrajectoryCircle_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);

    confPurePursuitSub_ = nh_.subscribe("/htcbot/pure_pursuit/conf", 10, &PurePursuit::callbackPurePursuitConf, this);
    // 开启主动避障, 将订阅局部路径规划的输出
    sub_safe_waypoints = nh_.subscribe("safety_waypoints", 10, &PurePursuit::callbackFromSafeWayPoints, this);
    sub_best_local_trajectory = nh_.subscribe("best_local_trajectorie", 10, &PurePursuit::callbackFromBestLocalTrajectory, this);

}

// 主体循环代码
void PurePursuit::run() {
    // ROS_INFO_STREAM("pure pursuit start");
    ros::Rate loop_rate(LOOP_RATE_);

    while (ros::ok()) {
        ros::spinOnce();
        // 是否发布当前位姿 以及 是否设置航迹点
        if (!is_pose_set_ || !is_waypoint_set_ || f_is_blocked_) {
            static ros::Time last_record = ros::Time::now();
            if ((ros::Time::now() - last_record).toSec() > 1.0) {
                // ROS_WARN("[waypoint_follower]Necessary topics are not subscribed yet, is_pose_set_ : %d, is_waypoint_set_ : %d", is_pose_set_, is_waypoint_set_);
                last_record = ros::Time::now();
            }
            // 发布刹车命令
            publishBrakeCommand();  // if there is no task (no pose set or no global path set), send brake command
        } else {
            if ((ros::Time::now() - last_cpose_update_time).toSec() > 0.5) {
                // ROS_WARN("[waypoint_follower] long time after last current pose update");
                // loop_rate.sleep();
                continue;
            }
            // ROS_INFO("[waypoint_follwer] do follower");
            lookahead_distance_ = computeLookaheadDistance();
            double curvature = 0;
            bool can_get_curvature = computeCurvature(&curvature);
            // ROS_INFO("[waypoint_follower] next_waypoint_number_ %d", next_waypoint_number_);
            publishControlCommandStamped(can_get_curvature, curvature);
            // show closet point and target point
            // visualInRviz();
            // is_pose_set_ = false;
            pubNextWaypoint_.publish(displayNextWaypoint(getPoseOfNextWaypoint()));
            // pubNextTarget_.publish(displayNextTarget(getPoseOfNextTarget()));
            pubSearchCircle_.publish(displaySearchRadius(getCurrentPose().position, getLookaheadDistance()));
            pubTrajectoryCircle_.publish(displayTrajectoryCircle(generateTrajectoryCircle(getPoseOfNextTarget(), getCurrentPose())));
        }
        // ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_WARN("[pure_pursuit_node] terminated.");
}

void PurePursuit::reset() {
    this->is_waypoint_set_ = false;
    this->is_target_front = true;
}

void PurePursuit::handle_avoid_type_switch(const htcbot_msgs::ModeSwitchConstPtr &msg) 
{
    ROS_INFO("[pure_pursuit] ==> mode: %s switch: %s", msg->mode.c_str(), msg->switch_to ? "true" : "false");
    if (msg->mode != htcbot_msgs::ModeSwitch::AVOID_TYPE) {
        ROS_INFO("[pure_pursuit] ==> xxx");
        return;
    }
    if (msg->switch_to == htcbot_msgs::ModeSwitch::ON && !current_avoid_type_switch) {
        current_avoid_type_switch = msg->switch_to;
        ROS_INFO("[pure_pursuit] Set avoid AUTO");        
        // 开启主动避障, 将订阅局部路径规划的输出
        sub_safe_waypoints = nh_.subscribe("safety_waypoints", 10, &PurePursuit::callbackFromSafeWayPoints, this);
        sub_best_local_trajectory = nh_.subscribe("best_local_trajectorie", 10, &PurePursuit::callbackFromBestLocalTrajectory, this);
    } else if (msg->switch_to == htcbot_msgs::ModeSwitch::OFF && current_avoid_type_switch) {
        current_avoid_type_switch = msg->switch_to;
        ROS_INFO("[pure_pursuit] Set avoid STOP");
        // 关闭主动避障, 只按照global_path的路径行驶
        sub_safe_waypoints.shutdown();
        sub_best_local_trajectory.shutdown();
    } else {
        ROS_WARN("[pure_pursuit] Set avoid UNKNOW");
    }
} 

void PurePursuit::publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) {
    can_msgs::ecu ecu_ctl;

    // ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;

    double steer = atan(wheel_base_ * curvature);

    // 平滑处理
    steer = (steer + pre_steer + pre_pre_steer) / 3.0;

    pre_pre_steer = pre_steer;
    pre_steer = steer;
    // steer = steer / 0.06 * 30;  // 转化弧度为角度
    steer = steer * 180.0 / 3.141592654;

    // 如果获取失败则置为0
    ecu_ctl.steer = can_get_curvature ? steer : 0;

    ecu_ctl.shift = ecu_ctl.SHIFT_D;
    ecu_ctl.brake = false;

    ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;
    // static bool enter_ending_process = false;

    // if (getPlaneDistance(current_pose_.position, current_waypoints_.at(current_waypoints_.size() - 1).pose.pose.position) < 3.0) {
    // ROS_INFO("current_pose_closet_index=%d, current_waypoints_.size()=%d", current_pose_closet_index, current_waypoints_.size());
    if (current_pose_closet_index >= end_waypoint_index - 5) {  // TODO:: 增加考虑刹车距离, end_waytpoint_index应减去 (刹车距离/点密度)
        // ecu_ctl.shift = ecu_ctl.SHIFT_D;
        // if (ecu_ctl.motor > 0.01) {
        //     ecu_ctl.brake = true;
        //     std::cout << "reach last point" << std::endl;
        // }
        // ecu_ctl.motor = 0;
        // ecu_ctl.steer = 0;
        // std_msgs::Bool finished;
        // finished.data = true;
        // pub_pure_pursuit_finished.publish(finished);
        do_ending_process(ecu_ctl);
        return;
    }

    if (!this->is_target_front){
        ecu_ctl.shift = ecu_ctl.SHIFT_R;
    }
    ecu_ctl.header.stamp = ros::Time::now();
    pub_control.publish(ecu_ctl);
}

void PurePursuit::publishBrakeCommand() {
    can_msgs::ecu ecu_ctl;
    ecu_ctl.header.stamp = ros::Time::now();
    ecu_ctl.brake = true;
    ecu_ctl.shift = 0;
    ecu_ctl.motor = 0;
    ecu_ctl.steer = 0;
    pub_control.publish(ecu_ctl);
}

bool PurePursuit::do_ending_process(can_msgs::ecu msg_ecu) {
    ROS_INFO("[waypoint_follower] entering ending process");

    msg_ecu.header.stamp = ros::Time::now();
    msg_ecu.brake = true;
    msg_ecu.motor = 0.0;
    msg_ecu.steer = 0.0;
    msg_ecu.shift = msg_ecu.SHIFT_N;
    pub_control.publish(msg_ecu);

    // TODO:: reset pure_pursuit_node
    is_waypoint_set_ = false;

    std_msgs::Bool finished;
    finished.data = true;
    pub_pure_pursuit_finished.publish(finished);
    ROS_INFO("[waypoint_follower] Task finished! Waitting for new task...");
    return true;
}

double PurePursuit::computeLookaheadDistance() const {
    if (is_const_lookahead_dis_ == true) {
        if (is_in_cross || is_not_lane) {
            return cross_lookahead_distance;
        } else {
            return lane_lookahead_distance;
        }
    }
    // else
    // {
    //   return 5;
    // }

    // 通过速度乘系数得到的前视距离不小于最小前视距离，不大于当前速度×7
    double maximum_lookahead_distance = cur_speed * 7;
    double ld = cur_speed * lookahead_distance_ratio_;
    return ld < (minimum_lookahead_distance_) ? minimum_lookahead_distance_ : (ld > maximum_lookahead_distance) ? maximum_lookahead_distance : ld;
}

double PurePursuit::computeCommandVelocity() {
    if (is_const_speed_command_ == true) return const_velocity_;
    // 判断是否在直线上，修改前进速度
    if (is_in_cross || is_not_lane) {
        command_linear_velocity_ = cross_speed_limit;  // 弯道处的速度
    } else {
        command_linear_velocity_ = lane_speed_limit;  // 直线的速度
    }
    // printf("--------------speed: %f\n", command_linear_velocity_);
    return command_linear_velocity_;
}

// 获取当前位置
void PurePursuit::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg) {
    // 读取了当前的车辆pose消息，里面有xyz和四元数
    last_cpose_update_time = ros::Time::now();
    current_pose_ = msg->pose;
    if ( !this->is_target_front ){
        // yaw反向180度
        // TODO::
        //     1. yaw旋转180度
        //     2. 发布ecu的时候, 如果非istarget_front, 则设置shift=R,
        //     3. fcontroller逻辑处理
        //     4. 测试
        double yaw, pitch, roll;
        tf2::Matrix3x3 mat(tf2::Quaternion(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        ));
        mat.getEulerYPR(yaw, pitch, roll);
        // ROS_INFO("RPY = %.2f, %.2f, %.2f", roll, pitch, yaw);

        yaw += 3.141592654;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        current_pose_.orientation = quat;

        // geometry_msgs::PoseStamped current_pose_reversed;
        // current_pose_reversed.header.frame_id = "map";
        // current_pose_reversed.pose = current_pose_;
        // this->pub_reversed_current_pose.publish(current_pose_reversed);
    }
    is_pose_set_ = true;
    if (!is_waypoint_set_) return;

    float min_dist = 9999999999.9;
    int search_start = ((current_pose_closet_index-200) > 0) ? (current_pose_closet_index-200) : 0;
    int search_end = ((search_start+400) < current_waypoints_.size()) ? (search_start+400) : current_waypoints_.size();
    int temp_closet_index = 0;

    bool closet_index_got = false;
    bool is_second_search = false;
    while ( !closet_index_got ) {
        for (int i = search_start; i < search_end; i++){
            htcbot_msgs::Waypoint p = current_waypoints_[i];
            float cur_distance = getPlaneDistance(p.pose.pose.position, current_pose_.position);
            if ( cur_distance < min_dist ){
                min_dist = cur_distance;
                temp_closet_index = i;
            }
        }

        // 临时方案：要求车辆距离轨迹线不能超过1m
        // if (min_dist > 1) {
        //     if (!is_second_search) {
        //         is_second_search = true;
        //         search_start = 0;
        //         search_end = current_waypoints_.size();
        //         min_dist = 9999999999.9;
        //         continue;
        //     }else{
        //         closet_index_got = false;
        //         break;
        //     }
        // }

        closet_index_got = true;
    }

    if (closet_index_got){
        current_pose_closet_index = temp_closet_index;
    }else{
        // 车辆距离路径过远, 丢失, 终止
        current_pose_closet_index = current_waypoints_.size()-1;
    }
    // ROS_INFO("path_size: %d, search_index: %d, search_end: %d, min_dist: %d, closet_index: %d", current_waypoints_.size(), search_start, search_end, min_dist, current_pose_closet_index);
}

// 获取当前速度
void PurePursuit::callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg) { cur_speed = msg.cur_speed; }

void PurePursuit::callbackFromSafeWayPoints(const nav_msgs::PathConstPtr &msg) {
    htcbot_msgs::Lane smartcar_lane;
    for (const auto p: msg->poses) {
        htcbot_msgs::Waypoint w;
        w.pose.pose = p.pose;
        smartcar_lane.waypoints.push_back(w);
    }
    this->__handle_followed_path(smartcar_lane);
}

void PurePursuit::callbackPurePursuitConf(const htcbot_msgs::ConfPurePursuit::ConstPtr &msg) {

    lookahead_distance_ = msg->const_lookahead_distance;
    lane_lookahead_distance = msg->lane_lookahead_distance;
    cross_lookahead_distance = msg->cross_lookahead_distance;
    lookahead_distance_ratio_ = msg->lookahead_distance_ratio;
    minimum_lookahead_distance_ = msg->minimum_lookahead_distance;

    // 更新参数
    private_nh_.setParam("const_lookahead_distance", lookahead_distance_);
    private_nh_.setParam("lane_lookahead_distance", lane_lookahead_distance);
    private_nh_.setParam("cross_lookahead_distance", cross_lookahead_distance);
    private_nh_.setParam("lookahead_distance_ratio", lookahead_distance_ratio_);
    private_nh_.setParam("minimum_lookahead_distance", minimum_lookahead_distance_);

}

void PurePursuit::callbackFromBestLocalTrajectory(const htcbot_msgs::Lane &msg) {
    this->__handle_followed_path(msg);
}

void PurePursuit::callbackFromWayPoints(const htcbot_msgs::Lane &msg) {

    this->__handle_followed_path(msg);
}

void PurePursuit::__handle_followed_path(htcbot_msgs::Lane msg) {
    if (msg.is_blocked) {
        f_is_blocked_ = true;
        return;
    } else if (msg.waypoints.size() < 2) {
        // 如果路径上的路径点数量小于2，不执行后续操作
        f_is_blocked_ = true;
        return;
    } 
    f_is_blocked_ = false;
    next_traj_index_ = msg.lane_index;
    // 设置结束路径点的索引为路径上的最后一个点
    end_waypoint_index = msg.waypoints.size();
    // 计算整个路径的长度
    float _total_distance = 0.0;
    for (int i = 1; i < msg.waypoints.size(); i++) {
        // 使用 getPlaneDistance 函数计算两点之间的平面距离并累加
        _total_distance += getPlaneDistance(msg.waypoints[i-1].pose.pose.position, msg.waypoints[i].pose.pose.position);
    }
    // 根据路径长度设置步长
    this->step_size = _total_distance / msg.waypoints.size();

    // 延长尾部, 避免终点震荡
    htcbot_msgs::Lane lane_extended;
    lane_extended.waypoints = msg.waypoints;
    // 如果路径上的路径点数量大于5，执行尾部延长操作
    if (lane_extended.waypoints.size() > 5) {
        // 获取路径上的倒数第五个点和最后一个点
        htcbot_msgs::Waypoint st = lane_extended.waypoints[lane_extended.waypoints.size() - 5];
        htcbot_msgs::Waypoint end = lane_extended.waypoints[lane_extended.waypoints.size()-1];
        // 计算倒数第五个点到最后一个点的方向角度
        double rad = atan2(end.pose.pose.position.y - st.pose.pose.position.y, end.pose.pose.position.x - st.pose.pose.position.x);
        // 在路径上添加一系列点，延长路径
        for (int i = 1; i < 100; i++){
            htcbot_msgs::Waypoint t;
            t.pose.pose.position.x = end.pose.pose.position.x + i*0.1*cos(rad);
            t.pose.pose.position.y = end.pose.pose.position.y + i*0.1*sin(rad);
            t.speed_limit = end.speed_limit;
            t.lane_id = msg.lane_index;
            t.lane_index = msg.lane_index;
            lane_extended.waypoints.push_back(t);
        }
    }
    // 将延长后的路径作为当前路径
    current_waypoints_ = lane_extended.waypoints;
    // 输出路径长度信息
    // ROS_WARN("[pure_pursuit] Total trajectory index = %d, length = %.2f m, waypoints = %d", next_traj_index_, _total_distance, current_waypoints_.size());
    // 设置路径已被设定的标志为true
    is_waypoint_set_ = true;
    // 初始化一些变量
    is_last_point = false;
    next_waypoint_number_ = -1;
    is_in_cross = false;
    pre_index = 0;
    search_start_index = 0;
    clearest_points_index = -1;
    // 设置搜索半径
    search_radius = 2;  // 搜索半径
    cross_in = false;
    cross_out = false;
    almost_reach = false;
    lock_index = -1;
    save_index = -1;
    is_not_lane = true;
    // 初始化当前位姿最近的路径点索引
    current_pose_closet_index = 0;
}

}  // namespace PurePursuitNS
