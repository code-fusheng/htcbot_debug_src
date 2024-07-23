#include "op_trajectory_evaluator.h"

using namespace OpCommonNS;
using namespace OpUtilsNS;

namespace OpTrajectoryEvaluatorNS
{

    OpTrajectoryEvaluator::OpTrajectoryEvaluator() : nh_private_("~")
    {
        mode_switch_ = true;
        switch_status_ = 0;
        op_prev_index_ = -1;
        op_prev_cost_index_ = -1;
        op_weight_priority_ = 0.9;
        op_weight_transition_ = 0.9;
        op_weight_long_ = 1.0;
        op_weight_lat_ = 1.2;
        op_weight_lane_change_ = 0.0;
        op_lateral_skip_distance_ = 50;
        op_collision_time_diff_ = 6.0;

        module_status_.module_type = static_cast<int>(HtcbotCommonNS::MODULE_TYPE::PLANNER);
        module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::NONE);
    }

    OpTrajectoryEvaluator::~OpTrajectoryEvaluator() {}

    void OpTrajectoryEvaluator::init()
    {

        nh_private_.param<int>("switch_status", switch_status_, 0);
        srv_switch_status_ = nh_private_.advertiseService("set_switch_status", &OpTrajectoryEvaluator::setSwitchStatusCallback, this);

        OpUtilsNS::GetPlanningParams(nh, op_planner_params_);

        // 车辆尺寸相关的参数
        nh_private_.param<double>("width", op_car_info_.width, 0.6);                       // 宽度
        nh_private_.param<double>("length", op_car_info_.length, 0.9);                     // 车长度
        nh_private_.param<double>("wheel_base", op_car_info_.wheel_base, 0.50);            // 轮距
        nh_private_.param<double>("max_steer_angle", op_car_info_.max_steer_angle, 0.524); // 最大转向角度

        lidar_frame = "rslidar";

        // 接收车辆当前姿态信息
        sub_current_pose_ = nh.subscribe("/current_pose_truth", 1, &OpTrajectoryEvaluator::callbackCurrentPose, this);
        // 接收局部轨迹信息
        sub_local_trajectories_ = nh.subscribe("/local_trajectories", 1, &OpTrajectoryEvaluator::callbackLocalTrajectories, this);
        // 订阅检测障碍物
        sub_detected_objects_ = nh.subscribe("/detected_objects", 1, &OpTrajectoryEvaluator::callbackDetectedObjects, this);

        sub_central_path_section_ = nh.subscribe("/central_path_section", 1, &OpTrajectoryEvaluator::callbackCentralPathSection, this);

        sub_op_local_planner_conf_ = nh.subscribe("/htcbot/op_local_planner/conf", 1, &OpTrajectoryEvaluator::callbackConf, this);

        sub_vehicle_status_ = nh.subscribe("/vehicle_status", 1, &OpTrajectoryEvaluator::callbackVehicleStatus, this);

        pub_local_weighted_trajectories_rviz_ = nh.advertise<visualization_msgs::MarkerArray>("local_weighted_trajectories_rviz", 1);
        pub_local_weighted_trajectories_ = nh.advertise<htcbot_msgs::LaneArray>("local_weighted_trajectories", 1);
        pub_trajectory_cost_ = nh.advertise<htcbot_msgs::Lane>("best_local_trajectorie_cost", 1);
        pub_trajectory_best_ = nh.advertise<htcbot_msgs::Lane>("best_local_trajectorie", 1);
        pub_trajectory_best_rviz_ = nh.advertise<visualization_msgs::Marker>("local_trajectory_best_rviz", 1);

        pub_collision_points_rviz_ = nh.advertise<visualization_msgs::MarkerArray>("local_collision_points_rviz", 1);
        pub_safety_border_rviz_ = nh.advertise<visualization_msgs::Marker>("safety_border", 1);
        pub_safety_area_rviz_ = nh.advertise<geometry_msgs::PolygonStamped>("safety_area", 1);

        pub_central_path_section_rviz_ = nh.advertise<visualization_msgs::Marker>("central_path_section_rviz", 1);
        pub_module_status_ = nh.advertise<htcbot_msgs::StatusHtcbotModule>("/htcbot/module_status", 10);
        OpUtilsNS::InitCollisionPointsMarkers(50, op_collisions_dummy_);

        server_.setCallback(boost::bind(&OpTrajectoryEvaluator::dynamicReconfigureCallback, this, _1, _2));
    }

    void OpTrajectoryEvaluator::run()
    {
        init();
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            // 声明并初始化一个轨迹成本对象 tc，用于存储当前轨迹的成本评估结果
            OpCommonNS::TrajectoryCost tc;
            if ((ros::Time::now() - truth_pose_update_time_).toSec() > 0.5)
            {
                f_current_pose_ = false;
            }
            if (f_current_pose_ && f_generated_rollouts_ && op_central_path_section_.size() > 0 && f_lidar_map_tf_)
            {
                htcbot_msgs::Lane best_lane;
                visualization_msgs::Marker best_traj;
                tc = doEvaluatorTrajectoryCostStatic(op_generated_rollouts_,
                                                     op_central_path_section_,
                                                     op_current_pose_,
                                                     op_planner_params_,
                                                     op_car_info_,
                                                     op_vehicle_status_,
                                                     op_detected_objects_);
                // 将最近物体的距离存储到消息中
                best_lane.closest_object_distance = tc.closest_obj_distance;
                // 将最近物体的速度存储到消息中
                best_lane.closest_object_velocity = tc.closest_obj_velocity;
                // 将轨迹的成本存储到消息中
                best_lane.cost = tc.cost;
                best_lane.center_cost = tc.priority_cost;
                best_lane.transition_cost = tc.transition_cost;
                best_lane.lateral_cost = tc.lateral_cost;
                best_lane.long_cost = tc.longitudinal_cost;
                // 将轨迹是否被阻塞的信息存储到消息中
                best_lane.is_blocked = tc.bBlocked;
                // 将轨迹的索引存储到消息中
                best_lane.lane_index = tc.index;
                best_lane.lane_id = tc.index;
                // 发布轨迹成本消息
                pub_trajectory_cost_.publish(best_lane);
                // TODO 还存在问题
                // ROS_ERROR("[op_trajectory_evaluator] best_lane.index %d", tc.index);
                if (tc.index != -1)
                {
                    module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
                    OpUtilsNS::ConvertLocalLane2MsgLane(op_generated_rollouts_.at(tc.index), best_lane);
                    // 发布最佳轨迹可视化
                    OpUtilsNS::TrajectoryToMarker(op_generated_rollouts_.at(tc.index), best_traj);
                }
                else
                {
                    best_lane.lane_index = -1;
                    best_lane.lane_id = -1;
                    best_lane.waypoints.clear();
                    best_traj.points.clear();
                    module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::DANGER);
                }
                pub_trajectory_best_.publish(best_lane);
                pub_trajectory_best_rviz_.publish(best_traj);
            }
            else
            {
                module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::EXC);
            }
            // 如果轨迹成本列表的大小与生成的轨迹列表的大小相同，则将生成的轨迹列表转换为 htcbot 车道消息
            if (op_trajectory_costs_.size() == op_generated_rollouts_.size())
            {
                htcbot_msgs::LaneArray local_lanes;
                for (size_t i = 0; i < op_generated_rollouts_.size(); i++)
                {
                    htcbot_msgs::Lane lane;
                    OpUtilsNS::ConvertLocalLane2MsgLane(op_generated_rollouts_.at(i), lane);
                    lane.closest_object_distance = op_trajectory_costs_.at(i).closest_obj_distance;
                    lane.closest_object_velocity = op_trajectory_costs_.at(i).closest_obj_velocity;
                    lane.cost = op_trajectory_costs_.at(i).cost;
                    lane.is_blocked = op_trajectory_costs_.at(i).bBlocked;
                    lane.lane_index = i;
                    local_lanes.lanes.push_back(lane);
                }
                pub_local_weighted_trajectories_.publish(local_lanes);
            }
            else
            {
                module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::EXC);
            }
            // 可视化 轨迹代价列表的大小大于 0
            if (op_trajectory_costs_.size() > 0)
            {
                visualization_msgs::MarkerArray all_traj_cost_msgs;
                OpUtilsNS::TrajectoriesToColoredMarkers(op_generated_rollouts_,
                                                        op_trajectory_costs_,
                                                        all_traj_cost_msgs);
                pub_local_weighted_trajectories_rviz_.publish(all_traj_cost_msgs);
                // 将碰撞点信息转换为可视化标记，存储到 m_CollisionsActual 中
                OpUtilsNS::ConvertCollisionPointsMarkers(op_all_contour_points_, op_collisions_actual_, op_collisions_dummy_);
                pub_collision_points_rviz_.publish(op_collisions_actual_);
                // 安全区域
                visualSafetyArea(op_safety_border_);
            }
            else
            {
                module_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::EXC);
            }
            pub_module_status_.publish(module_status_);
            loop_rate.sleep();
        }
    }

    void OpTrajectoryEvaluator::dynamicReconfigureCallback(op_local_planner::OpLocalPlannerConfig &config, uint32_t level)
    {
        is_debug_ = config.is_debug;
        op_planner_params_.minObstacleEvaluateDistance = config.min_obstacle_evaluate_distance;
        op_planner_params_.minBackObstacleEvaluateDistance = config.min_back_obstacle_evaluate_distance;
        std::cout << "UPDATE-----------------------------------------------------------------START" << std::endl;
        std::cout << "is_debug: " << is_debug_ << std::endl;
        std::cout << "minObstacleEvaluateDistance: " << op_planner_params_.minObstacleEvaluateDistance << std::endl;
        std::cout << "minBackObstacleEvaluateDistance: " << op_planner_params_.minBackObstacleEvaluateDistance << std::endl;
        std::cout << "UPDATE-----------------------------------------------------------------END" << std::endl;
    }

    bool OpTrajectoryEvaluator::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res)
    {
        ROS_INFO("[op_trajector_evaluator_node] ===> req: %d", req.switch_to);
        switch_status_ = req.switch_to;
        res.switch_status = switch_status_;
        return true;
    }

    void OpTrajectoryEvaluator::callbackConf(const htcbot_msgs::ConfOpLocalPlanner::ConstPtr &msg)
    {
        if (mode_switch_ != msg->switch_to)
        {
            mode_switch_ = msg->switch_to;
        }
        if (mode_switch_)
        {
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

            op_planner_params_.minObstacleEvaluateDistance = msg->min_obstacle_evaluate_distance;

            OpUtilsNS::UpdatePlanningParams(nh, op_planner_params_);
        }
        nh.setParam("/op_common_params_node/modeSwitch", mode_switch_);
    }

    void OpTrajectoryEvaluator::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        op_current_pose_ = OpCommonNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
        f_current_pose_ = true;
        truth_pose_update_time_ = ros::Time::now();
    }

    void OpTrajectoryEvaluator::callbackLocalTrajectories(const htcbot_msgs::LaneArray::ConstPtr &msg)
    {
        // 检查接收到的跟踪轨迹数组是否非空
        if (msg->lanes.size() > 0)
        {
            op_generated_rollouts_.clear();
            // 遍历接收到的每个跟踪轨迹
            for (size_t i = 0; i < msg->lanes.size(); i++)
            {
                // 存储单条跟踪轨迹的容器
                std::vector<OpCommonNS::WayPoint> rollout_single_path;
                OpUtilsNS::ConvertMsgLane2LocalLane(msg->lanes.at(i), rollout_single_path);
                op_generated_rollouts_.push_back(rollout_single_path);
            }
            // // 检查路径是否有效且生成的轨迹列表不为空
            // && op_generated_rollouts_.at(0).size() > 0
            if (op_generated_rollouts_.size() > 0)
            {
                f_generated_rollouts_ = true;
            }
        }
    }

    void OpTrajectoryEvaluator::callbackDetectedObjects(const htcbot_msgs::DetectedObjectArray::ConstPtr &msg)
    {
        op_detected_objects_.clear();
        f_detected_objects_ = true;
        OpCommonNS::DetectedObject obj;
        // 初始化一个标志变量，用于标记是否存在相似的障碍物
        bool f_similar_obj = false;
        // 遍历接收到的障碍物数组
        for (int i = 0; i < msg->objects.size(); i++)
        {
            setDetectedObject(msg->objects.at(i), obj);
            op_detected_objects_.push_back(obj);
        }
        // ROS_INFO("[op_trajectory_evaluator] op_detected_objects_ size %d", op_detected_objects_.size());
    }

    void OpTrajectoryEvaluator::callbackCentralPathSection(const htcbot_msgs::Lane::ConstPtr &msg)
    {
        // 清空之前存储的全局轨迹片段
        op_central_path_section_.clear();
        // 遍历接收到的全局轨迹中的每个路径点
        for (int i = 0; i < msg->waypoints.size(); i++)
        {
            OpCommonNS::WayPoint p;
            // 获取路径点的位置信息
            p.pos.x = msg->waypoints[i].pose.pose.position.x;
            p.pos.y = msg->waypoints[i].pose.pose.position.y;
            p.pos.z = msg->waypoints[i].pose.pose.position.z;
            // 获取路径点的偏航角信息
            p.pos.yaw = msg->waypoints[i].yaw;
            // 将构建好的路径点添加到当前跟踪的全局轨迹片段的容器中
            op_central_path_section_.push_back(p);
            // 发布
            visualization_msgs::Marker central_path_section_msgs;
            OpUtilsNS::TrajectoryToMarker(op_central_path_section_, central_path_section_msgs);
            pub_central_path_section_rviz_.publish(central_path_section_msgs);
        }
    }

    void OpTrajectoryEvaluator::callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr &msg)
    {
        op_vehicle_status_.speed = msg->cur_speed;
        op_vehicle_status_.steer = msg->cur_steer;
    }

    void OpTrajectoryEvaluator::setDetectedObject(const htcbot_msgs::DetectedObject &det_obj, OpCommonNS::DetectedObject &out_obj)
    {
        // 设置障碍物的开始时间为当前时间
        out_obj.start_time = ros::Time::now();
        // 设置障碍物的ID。
        out_obj.id = det_obj.id;
        // 设置障碍物的标签
        out_obj.label = det_obj.label;
        // 设置障碍物中心的位置
        out_obj.center.pos.x = det_obj.pose.position.x;
        out_obj.center.pos.y = det_obj.pose.position.y;
        out_obj.center.pos.z = det_obj.pose.position.z;
        lidar_map_tf = findTransform4Lidar2Map("world", lidar_frame);
        // 将障碍物中心位置转换到地图坐标系下 TODO yaw 变换?
        out_obj.center.pos = transformPoint(out_obj.center.pos, lidar_map_tf);
        OpCommonNS::GPSPoint p;
        out_obj.l = 0.1;         // 初始化障碍物的长度。
        out_obj.w = 0.1;         // 初始化障碍物的宽度
        out_obj.h = 0.1;         // 初始化障碍物的高度
        out_obj.contour.clear(); // 清空障碍物的轮廓
        // 遍历障碍物的凸多边形轮廓
        for (int k = 0; k < det_obj.convex_hull.polygon.points.size(); k++)
        {
            // 获取凸多边形轮廓的每个点的坐标
            p.x = det_obj.convex_hull.polygon.points[k].x;
            p.y = det_obj.convex_hull.polygon.points[k].y;
            p.z = det_obj.convex_hull.polygon.points[k].z;
            // 将每个点的坐标执行坐标变换，转换到地图坐标系下
            p = transformPoint(p, lidar_map_tf);
            // 更新障碍物的长度、宽度和高度
            out_obj.l = (out_obj.l < 2 * (p.x - out_obj.center.pos.x) ? 2 * (p.x - out_obj.center.pos.x) : out_obj.l);
            out_obj.w = (out_obj.w < 2 * (p.y - out_obj.center.pos.y) ? 2 * (p.y - out_obj.center.pos.y) : out_obj.w);
            out_obj.h = (out_obj.h < 2 * (p.z - out_obj.center.pos.z) ? 2 * (p.z - out_obj.center.pos.z) : out_obj.h);
            // 将变换后的点添加到障碍物的轮廓中
            out_obj.contour.push_back(p);
        }
    }

    tf::StampedTransform OpTrajectoryEvaluator::findTansform4Source2Target(const std::string &source_frame, const std::string target_frame)
    {
        tf::StampedTransform transform;
        static ros::Time pre_find_time = ros::Time::now();
        try
        {
            transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("[op_trajectory_evalutor] : find tf failed");
        }
        return transform;
    }

    tf::StampedTransform OpTrajectoryEvaluator::findTransform4Lidar2Map(const std::string &map_frame, const std::string lidar_frame)
    {
        f_lidar_map_tf_ = false;
        tf::StampedTransform transform;
        static ros::Time pre_find_time = ros::Time::now();
        try
        {
            transform_listener.lookupTransform(map_frame, lidar_frame, ros::Time(0), transform);
            f_lidar_map_tf_ = true;
        }
        catch (tf::TransformException ex)
        {
            ros::Time t = ros::Time::now();
            if ((t - pre_find_time).toSec() > 1.0)
            {
                ROS_INFO("[op_trajectory_evalutor] : find tf failed");
                pre_find_time = t;
            }
        }
        return transform;
    }

    OpCommonNS::GPSPoint OpTrajectoryEvaluator::transformPoint(const OpCommonNS::GPSPoint &in_point_pos,
                                                               const tf::StampedTransform &in_transform)
    {
        tf::Vector3 tf_point(in_point_pos.x, in_point_pos.y, in_point_pos.z);
        tf::Vector3 tf_point_transformed = in_transform * tf_point;
        return OpCommonNS::GPSPoint(tf_point_transformed.x(), tf_point_transformed.y(), tf_point_transformed.z(), in_point_pos.yaw);
    }

    OpCommonNS::TrajectoryCost OpTrajectoryEvaluator::doEvaluatorTrajectoryCostStatic(
        const vector<vector<OpCommonNS::WayPoint>> &rollOuts,
        const vector<OpCommonNS::WayPoint> &totalPaths,
        const OpCommonNS::WayPoint &currPose,
        const OpCommonNS::PlanningParams &params,
        const OpCommonNS::CAR_BASIC_INFO &carInfo,
        const OpCommonNS::VehicleState &vehicleState,
        const std::vector<OpCommonNS::DetectedObject> &obj_list,
        const int &iCurrentIndex)
    {
        // 初始化最佳轨迹成本对象
        OpCommonNS::TrajectoryCost best_trajectory_cost;
        // 设置其初始值，表示车辆目前处于被阻挡状态
        best_trajectory_cost.bBlocked = true;
        best_trajectory_cost.closest_obj_distance = params.horizonDistance;
        best_trajectory_cost.closest_obj_velocity = 0;
        best_trajectory_cost.index = -1;
        // 计算当前车辆相对于全局路径的位置索引
        OpCommonNS::RelativeInfo car_relative_info;
        OpUtilsNS::GetRelativeInfo(totalPaths, currPose, car_relative_info);
        // 根据相对位置信息设置当前轨迹的索引
        int currIndex = params.rollOutNumber / 2 + floor(car_relative_info.perp_distance / params.rollOutDensity);
        if (currIndex < 0)
            currIndex = 0;
        else if (currIndex > params.rollOutNumber)
            currIndex = params.rollOutNumber;
        op_trajectory_costs_.clear();
        // 计算沿着中心轨迹的代价，初始化 op_trajectory_costs_ 向量
        if (rollOuts.size() > 0)
        {
            OpCommonNS::TrajectoryCost tc;
            int centralIndex = params.rollOutNumber / 2;
            tc.lane_index = 0;
            for (int i = 0; i < rollOuts.size(); i++)
            {
                // 遍历生成的所有轨迹，为每条轨迹计算相对于中心轨迹的相对索引、距离中心轨迹的横向距离、优先级成本等信息
                tc.index = i;
                // 相对索引位置等于当前索引 - 中间索引的一半
                tc.relative_index = i - centralIndex;
                tc.distance_from_center = params.rollOutDensity * tc.relative_index;
                // 计算权重 代表中间的局部轨迹优先级是最高的，在没有障碍物的情况下，有限选择中间的局部轨迹
                tc.priority_cost = fabs(tc.distance_from_center);
                tc.closest_obj_distance = params.horizonDistance;
                tc.longitudinal_cost = 0;
                tc.lateral_cost = 0;
                // TODO laneChangeCost 暂未计算
                if (rollOuts.at(i).size() > 0)
                    tc.lane_change_cost = rollOuts.at(i).at(0).laneChangeCost;
                op_trajectory_costs_.push_back(tc);
            }
        }
        // 计算轨迹切换代价 遍历所有轨迹，计算每个轨迹相对于中心轨迹的索引、距离中心的偏移、纵向和横向代价
        for (int ki = 0; ki < op_trajectory_costs_.size(); ki++)
            op_trajectory_costs_.at(ki).transition_cost = fabs(params.rollOutDensity * (currIndex - ki));

        OpCommonNS::WayPoint obj_p;
        // 遍历所有检测到的物体，提取物体的边界轮廓点，并存储到 op_all_contour_points_
        op_all_contour_points_.clear();
        for (int kk = 0; kk < obj_list.size(); kk++)
        {
            for (int m = 0; m < obj_list.at(kk).contour.size(); m++)
            {
                obj_p.pos = obj_list.at(kk).contour.at(m);
                obj_p.v = obj_list.at(kk).center.v;
                obj_p.id = kk;
                obj_p.laneId = kk;
                obj_p.cost = sqrt(obj_list.at(kk).w * obj_list.at(kk).w + obj_list.at(kk).l * obj_list.at(kk).l);
                op_all_contour_points_.push_back(obj_p);
            }
        }
        // 计算静态轨迹的纵向和横向成本 考虑车辆与周围物体的碰撞风险和舒适性
        calculateLateralAndLongitudinalCostsStaticPro(op_trajectory_costs_, rollOuts, totalPaths, currPose, op_all_contour_points_, params, carInfo, vehicleState);
        // 归一化轨迹成本
        normalizeCosts(op_trajectory_costs_);

        int smallestIndex = -1;
        double smallestCost = DBL_MAX;
        double smallestDistance = DBL_MAX;
        double velo_of_next = 0;

        // 选择最佳轨迹 选择成本最小且未被阻挡的轨迹作为最佳轨迹
        for (unsigned int ic = 0; ic < op_trajectory_costs_.size(); ic++)
        {
            if (!op_trajectory_costs_.at(ic).bBlocked && op_trajectory_costs_.at(ic).cost < smallestCost)
            {
                smallestCost = op_trajectory_costs_.at(ic).cost;
                smallestIndex = ic;
            }
            if (op_trajectory_costs_.at(ic).closest_obj_distance < smallestDistance)
            {
                smallestDistance = op_trajectory_costs_.at(ic).closest_obj_distance;
                velo_of_next = op_trajectory_costs_.at(ic).closest_obj_velocity;
            }
        }
        if (smallestIndex == -1)
        {
            best_trajectory_cost.bBlocked = true;
            best_trajectory_cost.lane_index = 0;
            best_trajectory_cost.index = op_prev_cost_index_;
            best_trajectory_cost.closest_obj_distance = smallestDistance;
            best_trajectory_cost.closest_obj_velocity = velo_of_next;
        }
        else if (smallestIndex >= 0)
        {
            best_trajectory_cost = op_trajectory_costs_.at(smallestIndex);
        }
        op_prev_index_ = currIndex;
        return best_trajectory_cost;
    }

    /**
     * Pro
     */
    void OpTrajectoryEvaluator::calculateLateralAndLongitudinalCostsStaticPro(
        vector<OpCommonNS::TrajectoryCost> &trajectoryCosts,
        const vector<vector<OpCommonNS::WayPoint>> &rollOuts,
        const vector<OpCommonNS::WayPoint> &centerPath,
        const OpCommonNS::WayPoint &currPose,
        vector<OpCommonNS::WayPoint> &contourPoints,
        const OpCommonNS::PlanningParams &params,
        const OpCommonNS::CAR_BASIC_INFO &carInfo,
        const OpCommonNS::VehicleState &vehicleState)
    {
        // 计算横向临界距离 车身/2 + 水平安全距离
        double critical_lateral_distance = carInfo.width / 2.0 + params.horizontalSafetyDistance;
        // 计算前方纵向临界距离 轴距/2 + 车身/2 + 垂直安全距离
        double critical_long_front_distance = carInfo.wheel_base / 2.0 + carInfo.length / 2.0 + params.verticalSafetyDistance;
        // 计算后方纵向临界距离 车身/2 + 垂直安全距离 - 轴距/2
        double critical_long_back_distance = carInfo.length / 2.0 - carInfo.wheel_base / 2.0 + params.verticalSafetyDistance;
        // 创建一个矩阵 用于表示车辆位置的逆旋转矩阵 + 顺时针旋转90度
        OpCommonNS::Mat3 invRoatationMat(currPose.pos.yaw - M_PI / 2);
        // 创建一个矩阵 用于表示车辆位置的逆平移矩阵
        OpCommonNS::Mat3 invTranslationMat(currPose.pos.x, currPose.pos.y);

        OpCommonNS::GPSPoint bottom_left(-critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
        OpCommonNS::GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
        OpCommonNS::GPSPoint top_left(-critical_lateral_distance, critical_long_front_distance, currPose.pos.z, 0);
        OpCommonNS::GPSPoint top_right(critical_lateral_distance, critical_long_front_distance, currPose.pos.z, 0);

        bottom_left = invRoatationMat * bottom_left;
        bottom_left = invTranslationMat * bottom_left;

        bottom_right = invRoatationMat * bottom_right;
        bottom_right = invTranslationMat * bottom_right;

        top_left = invRoatationMat * top_left;
        top_left = invTranslationMat * top_left;

        top_right = invRoatationMat * top_right;
        top_right = invTranslationMat * top_right;

        op_safety_border_.points.clear();
        op_safety_border_.points.push_back(bottom_left);
        op_safety_border_.points.push_back(bottom_right);
        op_safety_border_.points.push_back(top_right);
        op_safety_border_.points.push_back(top_left);

        int iCostIndex = 0;
        tf::StampedTransform tf_ = findTansform4Source2Target("world", "rslidar");
        if (rollOuts.size() > 0 && rollOuts.at(0).size() > 0)
        {
            int skip_id = -1;
            // 获取 车辆 相对于全局路径的相关信息
            OpCommonNS::RelativeInfo car_info;
            OpUtilsNS::GetRelativeInfo(centerPath, currPose, car_info);
            for (size_t ri = 0; ri < rollOuts.size(); ri++)
            {
                int skip_id = -1;
                double hori_distance = DBL_MAX, vert_distance = 0;
                for (size_t rj = 0; rj < rollOuts.at(ri).size(); rj++)
                {
                    for (size_t cp_i = 0; cp_i < contourPoints.size(); cp_i++)
                    {
                        // 根据运动方向标记障碍物点
                        if (vehicleState.speed >= 0)
                        {
                            // 前进
                            GPSPoint pos = transformPoint(contourPoints.at(cp_i).pos, tf_);
                            // TODO car_lenght
                            if (pos.x < params.minBackObstacleEvaluateDistance)
                            {
                                contourPoints.at(cp_i).level = -1;
                                continue;
                            }
                            // 路线规划距离
                            if (pos.x > params.minObstacleEvaluateDistance)
                            {
                                contourPoints.at(cp_i).level = 1;
                                continue;
                            }
                        }
                        // Horizontal portrait
                        // 计算轨迹线 到 障碍物点 最小横向距离
                        double t_hori_distance = std::sqrt(std::pow(contourPoints.at(cp_i).pos.x - rollOuts.at(ri).at(rj).pos.x, 2) + std::pow(contourPoints.at(cp_i).pos.y - rollOuts.at(ri).at(rj).pos.y, 2));
                        if (t_hori_distance <= critical_lateral_distance)
                        {
                            // ROS_ERROR("[op_traj_eval pro] ===> iCostIndex: %d, lateral_distance: %f, t_hori_distance: %f, hori_distance: %f", iCostIndex, critical_lateral_distance, t_hori_distance, hori_distance);
                            contourPoints.at(cp_i).level = 2;
                            trajectoryCosts.at(iCostIndex).bBlocked = true;
                        }
                        hori_distance = std::min(hori_distance, t_hori_distance);
                    }
                    // 如果纵向距离不为零，更新轨迹的纵向成本
                    // hori_distance = hori_distance - critical_lateral_distance;
                    if (hori_distance != 0)
                        trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0 / fabs(hori_distance);
                    // 如果当前障碍物点在纵向上比当前轨迹记录的最近障碍物点更近，则更新轨迹的最近障碍物信息
                    if (hori_distance >= -critical_long_front_distance && hori_distance < trajectoryCosts.at(iCostIndex).closest_obj_distance)
                    {
                        trajectoryCosts.at(iCostIndex).closest_obj_distance = hori_distance;
                        trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0 / fabs(hori_distance);
                        // trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(cp_i).v;
                    }
                    // if (rj > 0) {
                    //     min_vert_distance += std::sqrt(std::pow(rollouts[i][j].pos.x - rollouts[i][j-1].pos.x,2) + std::pow(rollouts[i][j].pos.y - rollouts[i][j-1].pos.y, 2));
                    // }
                }
                iCostIndex++;
            }
        }
    }

    /**
     * totalPaths 中心轨迹
     */
    void OpTrajectoryEvaluator::calculateLateralAndLongitudinalCostsStatic(vector<OpCommonNS::TrajectoryCost> &trajectoryCosts,
                                                                           const vector<vector<OpCommonNS::WayPoint>> &rollOuts,
                                                                           const vector<OpCommonNS::WayPoint> &totalPaths,
                                                                           const OpCommonNS::WayPoint &currPose,
                                                                           vector<OpCommonNS::WayPoint> &contourPoints,
                                                                           const OpCommonNS::PlanningParams &params,
                                                                           const OpCommonNS::CAR_BASIC_INFO &carInfo,
                                                                           const OpCommonNS::VehicleState &vehicleState)
    {
        // 计算横向临界距离 车身/2 + 水平安全距离
        double critical_lateral_distance = carInfo.width / 2.0 + params.horizontalSafetyDistance;
        // 计算前方纵向临界距离 轴距/2 + 车身/2 + 垂直安全距离
        double critical_long_front_distance = carInfo.wheel_base / 2.0 + carInfo.length / 2.0 + params.verticalSafetyDistance;
        // 计算后方纵向临界距离 车身/2 + 垂直安全距离 - 轴距/2
        double critical_long_back_distance = carInfo.length / 2.0 - carInfo.wheel_base / 2.0 + params.verticalSafetyDistance;

        // 创建一个矩阵 用于表示车辆位置的逆旋转矩阵 + 顺时针旋转90度
        OpCommonNS::Mat3 invRotationMat(currPose.pos.yaw - M_PI / 2);
        // 创建一个矩阵 用于表示车辆位置的逆平移矩阵
        OpCommonNS::Mat3 invTranslationMat(currPose.pos.x, currPose.pos.y);

        // 计算车辆转弯时侧向滑动的距离
        double corner_slide_distance = critical_lateral_distance / 2.0;
        // 计算将侧滑距离转化为角度的比例 侧滑距离/最大转向角度
        double ratio_to_angle = corner_slide_distance / carInfo.max_steer_angle;
        // 计算实际侧向滑动的距离 PS:这里 car 的状态不准确
        // double slide_distance = vehicleState.steer * ratio_to_angle;
        double slide_distance = 0;

        // 创建车辆尾部左右侧的GPS坐标点
        OpCommonNS::GPSPoint bottom_left(-critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
        OpCommonNS::GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
        // 类似创建了车辆头部 左侧、右侧 和 车身两个边角(考虑侧向滑动)的GPS坐标点
        // OpCommonNS::GPSPoint top_right_car(critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0,  currPose.pos.z, 0);
        // OpCommonNS::GPSPoint top_left_car(-critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0, currPose.pos.z, 0);

        OpCommonNS::GPSPoint top_right(critical_lateral_distance - slide_distance, critical_long_front_distance, currPose.pos.z, 0);
        OpCommonNS::GPSPoint top_left(-critical_lateral_distance - slide_distance, critical_long_front_distance, currPose.pos.z, 0);

        // 将尾部左侧的GPS坐标点变换到车辆当前位置和姿态下
        bottom_left = invRotationMat * bottom_left;
        bottom_left = invTranslationMat * bottom_left;

        top_right = invRotationMat * top_right;
        top_right = invTranslationMat * top_right;

        bottom_right = invRotationMat * bottom_right;
        bottom_right = invTranslationMat * bottom_right;

        top_left = invRotationMat * top_left;
        top_left = invTranslationMat * top_left;

        // top_right_car = invRotationMat * top_right_car;
        // top_right_car = invTranslationMat * top_right_car;

        // top_left_car = invRotationMat * top_left_car;
        // top_left_car = invTranslationMat * top_left_car;

        // 发布安全区域polygen
        // 将四个GPS点进行坐标变换，转换到车辆坐标系下
        op_safety_border_.points.clear();
        op_safety_border_.points.push_back(bottom_left);
        op_safety_border_.points.push_back(bottom_right);
        // op_safety_border_.points.push_back(top_right_car);
        op_safety_border_.points.push_back(top_right);
        op_safety_border_.points.push_back(top_left);
        // op_safety_border_.points.push_back(top_left_car);

        // 初始化轨迹代价索引
        int iCostIndex = 0;
        // 如果存在备选轨迹数据

        tf::StampedTransform tf_ = findTansform4Source2Target("world", "rslidar");
        if (rollOuts.size() > 0 && rollOuts.at(0).size() > 0)
        {
            // 获取 车辆 相对于全局路径的相关信息
            OpCommonNS::RelativeInfo car_info;
            OpUtilsNS::GetRelativeInfo(totalPaths, currPose, car_info);
            // 遍历轨迹路线
            for (size_t it = 0; it < rollOuts.size(); it++)
            {
                int skip_id = -1;
                for (size_t icon = 0; icon < contourPoints.size(); icon++)
                {
                    if (skip_id == contourPoints.at(icon).id)
                        continue;
                    // 根据前进方向排除点
                    if (vehicleState.speed >= 0)
                    {
                        // 前进
                        GPSPoint pos = transformPoint(contourPoints.at(icon).pos, tf_);
                        if (pos.x < 0)
                        {
                            contourPoints.at(icon).level = -1;
                            continue;
                        }
                        if (pos.x > 5.0)
                        {
                            contourPoints.at(icon).level = -1;
                            continue;
                        }
                    }
                    // 获取 障碍物轮廓点 相对于中心路径的信息
                    OpCommonNS::RelativeInfo obj_info;
                    // 获取障碍物点相对于中心轨迹的位置
                    OpUtilsNS::GetRelativeInfo(totalPaths, contourPoints.at(icon), obj_info);

                    // cout << "contourPoints: " << icon
                    //         << ", Size: " << contourPoints.size()
                    //         << ", id: " << contourPoints.at(icon).id
                    // 		<< ", perp_distance: " << obj_info.perp_distance
                    // 		<< ", to_front_distance: " << obj_info.to_front_distance
                    // 		<< ", from_back_distance: " << obj_info.from_back_distance
                    // 		<< ", iFront: " << obj_info.iFront
                    //         << ", iBack: " << obj_info.iBack
                    // 		<< ", iGlobalPath: " << obj_info.iGlobalPath
                    // 		<< ", angle_diff: " << obj_info.angle_diff
                    //         << ", after_angle:" << obj_info.after_angle
                    // 		<< endl;

                    // 计算当前障碍物点到车辆位置的沿着中心轨迹的纵向距离
                    double longitudinalDist = OpUtilsNS::GetExactDistanceOnTrajectory(totalPaths, car_info, obj_info);
                    // 处理车辆前方第一个全局路径点的特殊情况
                    if (obj_info.iFront == 0 && longitudinalDist > 0)
                        longitudinalDist = -longitudinalDist;
                    // 计算车辆和轮廓点之间的直线距离
                    double direct_distance = hypot(obj_info.perp_point.pos.y - contourPoints.at(icon).pos.y, obj_info.perp_point.pos.x - contourPoints.at(icon).pos.x);
                    // 如果轮廓点速度小于最小速度且直线距离大于一定值 跳过
                    // if(contourPoints.at(icon).v < params.minSpeed && direct_distance > (op_lateral_skip_distance_+contourPoints.at(icon).cost))
                    // {
                    // 	skip_id = contourPoints.at(icon).id;
                    // 	continue;
                    // }
                    // 初始化纵向距离的百分比
                    double close_in_percentage = 1;
                    // 获取轨迹代价对象中的距离中心的距离
                    double distance_from_center = trajectoryCosts.at(iCostIndex).distance_from_center;
                    // 根据百分比调整距离中心的值
                    if (close_in_percentage < 1)
                        distance_from_center = distance_from_center - distance_from_center * (1.0 - close_in_percentage);
                    // 计算横向距离
                    double lateralDist = fabs(obj_info.perp_distance - distance_from_center);
                    // 如果纵向距离不在有效范围内 则跳过
                    if (longitudinalDist < -carInfo.length || longitudinalDist > params.minFollowingDistance || lateralDist > op_lateral_skip_distance_)
                    {
                        continue;
                    }
                    // 调整longitudinalDist，为了考虑车辆前端的安全距离
                    longitudinalDist = longitudinalDist - critical_long_front_distance;
                    // 检查障碍物点是否在安全区域内，如果是，则将对应轨迹的bBlocked标志设置为true，表示该轨迹受到阻挡
                    // 检查横向和纵向距离是否满足一定条件，如果满足，则同样将bBlocked标志设置为true。
                    if (lateralDist <= critical_lateral_distance)
                        contourPoints.at(icon).level = 1;
                    // 	trajectoryCosts.at(iCostIndex).bBlocked = true;
                    //     contourPoints.at(icon).bBlocked = true;
                    //     ROS_ERROR("[op_traj_eval] ===> bBlocked for dist lateralDist:%d longitudinalDist:%d", lateralDist, longitudinalDist);
                    // 如果横向距离不为零，更新轨迹的横向成本
                    if (op_safety_border_.PointInsidePolygon(op_safety_border_, contourPoints.at(icon).pos) == true)
                        // ROS_ERROR("[op_traj_eval] ===> bBlocked for safety_border");
                        // trajectoryCosts.at(iCostIndex).bBlocked = true;
                        // contourPoints.at(icon).bBlocked = false;
                        contourPoints.at(icon).level = 2;
                    if (lateralDist != 0)
                        trajectoryCosts.at(iCostIndex).lateral_cost += 1.0 / lateralDist;
                    // 如果纵向距离不为零，更新轨迹的纵向成本
                    if (longitudinalDist != 0)
                        trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0 / fabs(longitudinalDist);
                    // 如果当前障碍物点在纵向上比当前轨迹记录的最近障碍物点更近，则更新轨迹的最近障碍物信息
                    if (longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectoryCosts.at(iCostIndex).closest_obj_distance)
                    {
                        trajectoryCosts.at(iCostIndex).closest_obj_distance = longitudinalDist;
                        trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(icon).v;
                    }
                }
                iCostIndex++;
            }
        }
    }

    void OpTrajectoryEvaluator::normalizeCosts(vector<OpCommonNS::TrajectoryCost> &trajectoryCosts)
    {
        // Normalize costs
        double totalPriorities = 0;
        double totalChange = 0;
        double totalLateralCosts = 0;
        double totalLongitudinalCosts = 0;
        double transitionCosts = 0;

        for (unsigned int ic = 0; ic < trajectoryCosts.size(); ic++)
        {
            totalPriorities += trajectoryCosts.at(ic).priority_cost;
            transitionCosts += trajectoryCosts.at(ic).transition_cost;
        }

        for (unsigned int ic = 0; ic < trajectoryCosts.size(); ic++)
        {
            totalChange += trajectoryCosts.at(ic).lane_change_cost;
            totalLateralCosts += trajectoryCosts.at(ic).lateral_cost;
            totalLongitudinalCosts += trajectoryCosts.at(ic).longitudinal_cost;
        }

        for (unsigned int ic = 0; ic < trajectoryCosts.size(); ic++)
        {
            if (totalPriorities != 0)
                trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
            else
                trajectoryCosts.at(ic).priority_cost = 0;

            if (transitionCosts != 0)
                trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / transitionCosts;
            else
                trajectoryCosts.at(ic).transition_cost = 0;

            if (totalChange != 0)
                trajectoryCosts.at(ic).lane_change_cost = trajectoryCosts.at(ic).lane_change_cost / totalChange;
            else
                trajectoryCosts.at(ic).lane_change_cost = 0;

            if (totalLateralCosts != 0)
                trajectoryCosts.at(ic).lateral_cost = trajectoryCosts.at(ic).lateral_cost / totalLateralCosts;
            else
                trajectoryCosts.at(ic).lateral_cost = 0;

            if (totalLongitudinalCosts != 0)
                trajectoryCosts.at(ic).longitudinal_cost = trajectoryCosts.at(ic).longitudinal_cost / totalLongitudinalCosts;
            else
                trajectoryCosts.at(ic).longitudinal_cost = 0;

            trajectoryCosts.at(ic).cost = (op_weight_priority_ * trajectoryCosts.at(ic).priority_cost + op_weight_transition_ * trajectoryCosts.at(ic).transition_cost + op_weight_lat_ * trajectoryCosts.at(ic).lateral_cost + op_weight_long_ * trajectoryCosts.at(ic).longitudinal_cost) / 4.0;

            // cout << "Index: " << ic
            // 				<< ", Priority: " << trajectoryCosts.at(ic).priority_cost
            // 				<< ", Transition: " << trajectoryCosts.at(ic).transition_cost
            // 				<< ", Lat: " << trajectoryCosts.at(ic).lateral_cost
            // 				<< ", Long: " << trajectoryCosts.at(ic).longitudinal_cost
            // 				<< ", Change: " << trajectoryCosts.at(ic).lane_change_cost
            // 				<< ", Avg: " << trajectoryCosts.at(ic).cost
            //                 << ", bBlocked:" << trajectoryCosts.at(ic).bBlocked
            // 				<< endl;
        }
    }

    void OpTrajectoryEvaluator::visualContourPoints(const std::vector<OpCommonNS::WayPoint> &in_points)
    {
        visualization_msgs::MarkerArray points_array;
        visualization_msgs::Marker one_point;

        one_point.header.frame_id = "map";
        one_point.header.stamp = ros::Time();
        one_point.ns = "contour_points";
        one_point.type = visualization_msgs::Marker::SPHERE;
        one_point.action = visualization_msgs::Marker::ADD;
        one_point.frame_locked = false;
        one_point.color.a = 0.6;
        one_point.color.r = 0.3;
        one_point.color.g = 0.6;
        one_point.color.b = 0.6;
        one_point.scale.x = 0.2;
        one_point.scale.y = 0.2;
        one_point.scale.z = 0.2;
        one_point.lifetime = ros::Duration(0.1);
        one_point.frame_locked = false;

        for (size_t i = 0; i < in_points.size(); i++)
        {
            one_point.id = i;
            one_point.pose.position.x = in_points[i].pos.x;
            one_point.pose.position.y = in_points[i].pos.y;
            one_point.pose.position.z = in_points[i].pos.z;
            points_array.markers.push_back(one_point);
        }
        pub_collision_points_rviz_.publish(points_array);
    }

    void OpTrajectoryEvaluator::visualSafetyArea(const OpCommonNS::PolygonShape &safety_border)
    {
        geometry_msgs::PolygonStamped safety_area;
        safety_area.header.frame_id = "map";
        for (size_t i = 0; i < safety_border.points.size(); i++)
        {
            geometry_msgs::Point32 safety_point;
            safety_point.x = safety_border.points.at(i).x;
            safety_point.y = safety_border.points.at(i).y;
            safety_point.z = safety_border.points.at(i).z;
            safety_area.polygon.points.push_back(safety_point);
        }
        pub_safety_area_rviz_.publish(safety_area);
    }

} // OpTrajectoryEvaluatorNS