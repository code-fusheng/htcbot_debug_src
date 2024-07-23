#include <op_common.h>
#include <op_utils.h>

using namespace OpCommonNS;
using namespace std;

namespace OpUtilsNS
{

    void GetPlanningParams(ros::NodeHandle &_nh, OpCommonNS::PlanningParams &planningParams)
    {
        _nh.param<double>("/op_common_params_node/rollOutDensity", planningParams.rollOutDensity, 0.2);
        _nh.param<int>("/op_common_params_node/rollOutNumber", planningParams.rollOutNumber, 6);
        _nh.param<double>("/op_common_params_node/pathDensity", planningParams.pathDensity, 0.25);
        _nh.param<double>("/op_common_params_node/maxVelocity", planningParams.maxSpeed, 5.0);
        _nh.param<double>("/op_common_params_node/minVelocity", planningParams.minSpeed, 0);
        _nh.param<double>("/op_common_params_node/microPlanDistance", planningParams.microPlanDistance, 30);
        _nh.param<double>("/op_common_params_node/smoothingDataWeight", planningParams.smoothingDataWeight, 0.47);
        _nh.param<double>("/op_common_params_node/smoothingSmoothWeight", planningParams.smoothingSmoothWeight, 0.2);
        _nh.param<double>("/op_common_params_node/smoothingToleranceError", planningParams.smoothingSmoothWeight, 0.05);

        _nh.param<double>("/op_common_params_node/horizontalSafetyDistance", planningParams.horizontalSafetyDistance, 0.5);
        _nh.param<double>("/op_common_params_node/verticalSafetyDistance", planningParams.verticalSafetyDistance, 0.5);

        _nh.param<bool>("/op_trajectory_evaluator_node/enablePrediction", planningParams.enableObjectsPrediction, false);
        _nh.param<double>("/op_trajectory_evaluator_node/minObstacleEvaluateDistance", planningParams.minObstacleEvaluateDistance, 5.0);

        _nh.param<double>("/op_trajectory_generator_node/carTipMargin", planningParams.carTipMargin, 4);
        _nh.param<double>("/op_trajectory_generator_node/rollInMargin", planningParams.rollInMargin, 12);
        _nh.param<double>("/op_trajectory_generator_node/rollInSpeedFactor", planningParams.rollInSpeedFactor, 0.25);

        _nh.param<bool>("/op_trajectory_generator_node/enableHeadingSmoothing", planningParams.enableHeadingSmoothing, true);
    }

    void UpdatePlanningParams(ros::NodeHandle &_nh, OpCommonNS::PlanningParams &planningParams)
    {
        _nh.setParam("/op_common_params_node/rollOutDensity", planningParams.rollOutDensity);
        _nh.setParam("/op_common_params_node/rollOutNumber", planningParams.rollOutNumber);
        _nh.setParam("/op_common_params_node/pathDensity", planningParams.pathDensity);
        _nh.setParam("/op_common_params_node/maxVelocity", planningParams.maxSpeed);
        _nh.setParam("/op_common_params_node/minVelocity", planningParams.minSpeed);
        _nh.setParam("/op_common_params_node/microPlanDistance", planningParams.microPlanDistance);
        _nh.setParam("/op_common_params_node/smoothingDataWeight", planningParams.smoothingDataWeight);
        _nh.setParam("/op_common_params_node/smoothingSmoothWeight", planningParams.smoothingSmoothWeight);
        _nh.setParam("/op_common_params_node/horizontalSafetyDistance", planningParams.horizontalSafetyDistance);
        _nh.setParam("/op_common_params_node/verticalSafetyDistance", planningParams.verticalSafetyDistance);

        _nh.setParam("/op_trajectory_generator_node/carTipMargin", planningParams.carTipMargin);
        _nh.setParam("/op_trajectory_generator_node/rollInMargin", planningParams.rollInMargin);
        _nh.setParam("/op_trajectory_generator_node/rollInSpeedFactor", planningParams.rollInSpeedFactor);
        _nh.setParam("/op_trajectory_generator_node/enableHeadingSmoothing", planningParams.enableHeadingSmoothing);

        _nh.setParam("/op_trajectory_evaluator_node/minObstacleEvaluateDistance", planningParams.minObstacleEvaluateDistance);
    }

    void ConvertMsgLane2LocalLane(const htcbot_msgs::Lane &msg_path, std::vector<OpCommonNS::WayPoint> &local_path)
    {
        // 清空传入的本地路径 local_path
        local_path.clear();
        // 遍历传入的全局路径消息中的每个路径点
        for (size_t i = 0; i < msg_path.waypoints.size(); i++)
        {
            OpCommonNS::WayPoint wp;
            // 将全局路径消息中的位置信息（x、y、z坐标）赋值给 wp 的位置信息
            wp.pos.x = msg_path.waypoints.at(i).pose.pose.position.x;
            wp.pos.y = msg_path.waypoints.at(i).pose.pose.position.y;
            wp.pos.z = msg_path.waypoints.at(i).pose.pose.position.z;
            // 使用tf::getYaw函数获取全局路径消息中的姿态信息的偏航角，并赋值给 wp 的偏航角
            wp.pos.yaw = tf::getYaw(msg_path.waypoints.at(i).pose.pose.orientation);
            wp.v = msg_path.waypoints.at(i).speed_limit;
            wp.laneId = msg_path.waypoints.at(i).lane_id;
            // wp.cost = trajectory.waypoints.at(i).cost;
            local_path.push_back(wp);
        }
    }

    void ConvertLocalLane2MsgLane(const std::vector<OpCommonNS::WayPoint> &local_path, htcbot_msgs::Lane &msg_lane)
    {
        msg_lane.waypoints.clear();
        for (int i = 0; i < local_path.size(); i++)
        {
            // 路径上的轨迹点信息
            htcbot_msgs::Waypoint wp;
            wp.pose.pose.position.x = local_path.at(i).pos.x;
            wp.pose.pose.position.y = local_path.at(i).pos.y;
            wp.pose.pose.position.z = local_path.at(i).pos.z;
            wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(FixNegativeAngle(local_path.at(i).pos.yaw));
            msg_lane.waypoints.push_back(wp);
        }
    }

    void SafetyRectangleToMarkers(const std::vector<OpCommonNS::GPSPoint> &safety_rect,
                                  visualization_msgs::Marker &marker)
    {
        visualization_msgs::Marker lane_waypoint_marker;
        lane_waypoint_marker.header.frame_id = "map";
        lane_waypoint_marker.header.stamp = ros::Time();
        lane_waypoint_marker.ns = "global_lane_array_marker";
        lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
        lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
        lane_waypoint_marker.scale.x = 0.2;
        lane_waypoint_marker.scale.y = 0.2;
        // lane_waypoint_marker.scale.z = 0.1;
        lane_waypoint_marker.frame_locked = false;
        lane_waypoint_marker.color.r = 0.0;
        lane_waypoint_marker.color.g = 1.0;
        lane_waypoint_marker.color.b = 0.0;
        lane_waypoint_marker.color.a = 0.6;

        for (unsigned int i = 0; i < safety_rect.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = safety_rect.at(i).x;
            p.y = safety_rect.at(i).y;
            p.z = safety_rect.at(i).z;

            lane_waypoint_marker.points.push_back(p);
        }
        if (safety_rect.size() > 0)
        {
            geometry_msgs::Point p;
            p.x = safety_rect.at(0).x;
            p.y = safety_rect.at(0).y;
            p.z = safety_rect.at(0).z;
            lane_waypoint_marker.points.push_back(p);
        }
        marker = lane_waypoint_marker;
    }

    void TrajectoriesToColoredMarkers(const std::vector<std::vector<OpCommonNS::WayPoint>> &paths,
                                      const std::vector<OpCommonNS::TrajectoryCost> &traj_costs,
                                      visualization_msgs::MarkerArray &markerArray,
                                      const int &iClosest)
    {
        visualization_msgs::Marker lane_waypoint_marker;
        lane_waypoint_marker.header.frame_id = "map";
        lane_waypoint_marker.header.stamp = ros::Time();
        lane_waypoint_marker.ns = "local_lane_array_marker";
        lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
        lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
        lane_waypoint_marker.scale.x = 0.1;
        lane_waypoint_marker.scale.y = 0.1;
        // lane_waypoint_marker.scale.z = 0.1;
        lane_waypoint_marker.color.a = 0.9;
        lane_waypoint_marker.color.r = 1.0;
        lane_waypoint_marker.color.g = 1.0;
        lane_waypoint_marker.color.b = 1.0;
        lane_waypoint_marker.frame_locked = false;

        int count = 0;
        for (unsigned int i = 0; i < paths.size(); i++)
        {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = count;

            for (unsigned int j = 0; j < paths.at(i).size(); j++)
            {
                geometry_msgs::Point point;

                point.x = paths.at(i).at(j).pos.x;
                point.y = paths.at(i).at(j).pos.y;
                point.z = paths.at(i).at(j).pos.z;

                lane_waypoint_marker.points.push_back(point);
            }

            lane_waypoint_marker.color.b = 0;

            if (traj_costs.size() == paths.size())
            {
                float norm_cost = traj_costs.at(i).cost * paths.size();
                if (norm_cost <= 1.0)
                {
                    lane_waypoint_marker.color.r = norm_cost;
                    lane_waypoint_marker.color.g = 1.0;
                }
                else if (norm_cost > 1.0)
                {
                    lane_waypoint_marker.color.r = 1.0;
                    lane_waypoint_marker.color.g = 2.0 - norm_cost;
                }
            }
            else
            {
                lane_waypoint_marker.color.r = 1.0;
                lane_waypoint_marker.color.g = 0.0;
            }

            if (traj_costs.at(i).bBlocked)
            {
                lane_waypoint_marker.color.r = 1.0;
                lane_waypoint_marker.color.g = 0.0;
                lane_waypoint_marker.color.b = 0.0;
            }

            if (i == iClosest)
            {
                lane_waypoint_marker.color.r = 1.0;
                lane_waypoint_marker.color.g = 0.0;
                lane_waypoint_marker.color.b = 1.0;
            }

            markerArray.markers.push_back(lane_waypoint_marker);
            count++;
        }
    }

    void TrajectoryToMarker(const std::vector<OpCommonNS::WayPoint> &path, visualization_msgs::Marker &lane_marker)
    {
        lane_marker.header.frame_id = "map";
        lane_marker.header.stamp = ros::Time();
        lane_marker.ns = "lane_marker";
        lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
        lane_marker.action = visualization_msgs::Marker::ADD;
        lane_marker.frame_locked = false;
        lane_marker.scale.x = 0.1;
        lane_marker.points.clear();
        for (size_t k = 0; k < path.size(); k++)
        {
            geometry_msgs::Point wp;
            wp.x = path[k].pos.x;
            wp.y = path[k].pos.y;
            wp.z = path[k].pos.z;
            lane_marker.points.push_back(wp);
        }
        lane_marker.color.r = 1.0; // Red
        lane_marker.color.g = 1.0; // Green
        lane_marker.color.b = 1.0; // Blue
        lane_marker.color.a = 1.0; // Alpha (Opaque)
    }

    void TrajectoriesToMarkers(const std::vector<std::vector<std::vector<OpCommonNS::WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray)
    {
        visualization_msgs::Marker lane_waypoint_marker;
        lane_waypoint_marker.header.frame_id = "map";
        lane_waypoint_marker.header.stamp = ros::Time();
        lane_waypoint_marker.ns = "local_lane_array_marker";
        lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
        lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
        lane_waypoint_marker.scale.x = 0.1;
        lane_waypoint_marker.scale.y = 0.1;
        // lane_waypoint_marker.scale.z = 0.1;
        lane_waypoint_marker.frame_locked = false;
        std_msgs::ColorRGBA total_color, curr_color;
        int count = 0;
        for (unsigned int il = 0; il < paths.size(); il++)
        {
            for (unsigned int i = 0; i < paths.at(il).size(); i++)
            {
                lane_waypoint_marker.points.clear();
                lane_waypoint_marker.id = count;
                for (unsigned int j = 0; j < paths.at(il).at(i).size(); j++)
                {
                    geometry_msgs::Point point;
                    point.x = paths.at(il).at(i).at(j).pos.x;
                    point.y = paths.at(il).at(i).at(j).pos.y;
                    point.z = paths.at(il).at(i).at(j).pos.z;
                    lane_waypoint_marker.points.push_back(point);
                }
                lane_waypoint_marker.color.a = 0.9;
                lane_waypoint_marker.color.r = 0.0;
                lane_waypoint_marker.color.g = 1.0;
                lane_waypoint_marker.color.b = 0.0;
                markerArray.markers.push_back(lane_waypoint_marker);
                count++;
            }
        }
    }

    double CalcAngleAndCost(std::vector<OpCommonNS::WayPoint> &path)
    {
        // 如果路径中的点数小于2，直接返回0 因为至少需要两个点才能计算航向角和代价
        if (path.size() < 2)
            return 0;
        if (path.size() == 2)
        {
            // 如果路径中有两个点，分别计算第一个点和第二个点的航向角，并设置代价。最后返回第二个点的累积代价
            path[0].pos.yaw = FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
            path[0].cost = 0;
            path[1].pos.yaw = path[0].pos.yaw;
            path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
            return path[1].cost;
        }

        // 对于路径中的第一个点，计算其航向角并将代价设置为0
        path[0].pos.yaw = FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;

        // 对于路径中的中间点，循环计算每个点的航向角和代价，代价为前一个点的代价加上当前点与前一个点的距离
        for (int j = 1; j < path.size() - 1; j++)
        {
            path[j].pos.yaw = FixNegativeAngle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
            path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
        }

        // 对于路径中的最后一个点，设置其航向角为前一个点的航向角，并计算代价为前一个点的代价加上当前点与前一个点的距离
        int j = (int)path.size() - 1;
        path[j].pos.yaw = path[j - 1].pos.yaw;
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

        // 返回路径中最后一个点的累积代价
        return path[j].cost;
    }

    // 用于从给定路径中提取一个部分，使得路径从当前位置开始，沿着车辆的方向延伸一定距离
    void ExtractPartFromPointToDistance(const std::vector<OpCommonNS::WayPoint> &originalPath,
                                        const OpCommonNS::WayPoint &pos,
                                        const double &minDistance,
                                        const double &pathDensity, std::vector<OpCommonNS::WayPoint> &extractedPath)
    {
        // 如果原始路径点少于两个，直接返回
        if (originalPath.size() < 2)
            return;
        // 清空提取的路径
        extractedPath.clear();
        // 获取距离当前位置最近的路径点的索引
        int close_index = GetClosestNextPointIndex(originalPath, pos);
        //   ROS_INFO("[op_trajectory_generator] close_index %d", close_index);
        double dis = 0;
        // 如果索引超过原始路径的最大索引，则将索引设置为原始路径的倒数第二个索引
        if (close_index >= originalPath.size() - 1)
            close_index = originalPath.size() - 2;

        // 从最近的点开始向前提取路径
        for (int i = close_index; i >= 0; i--)
        {
            extractedPath.insert(extractedPath.begin(), originalPath[i]);
            if (i < originalPath.size())
                dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
            // 如果累计距离超过2，停止提取
            if (dis > 2)
                break;
        }
        // 重置距离计数
        dis = 0;
        // 从最近的点的下一个点开始向后提取路径
        for (int i = close_index + 1; i < (int)originalPath.size(); i++)
        {
            extractedPath.push_back(originalPath[i]);
            if (i > 0)
                dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
            // 如果累计距离超过最小规划距离，停止提取
            if (dis > minDistance)
                break;
        }
        // 如果提取后的路径点数小于2，则输出错误信息并返回
        if (extractedPath.size() < 2)
        {
            std::cout << std::endl
                      << "[loacal_planner_node] Extracted Rollout Path is too Small, Size = " << extractedPath.size() << std::endl;
            return;
        }
        // 对提取后的路径进行密度调整
        FixPathDensity(extractedPath, pathDensity);
        // SmoothPath()
        CalcAngleAndCost(extractedPath);
    }

    // 获取轨迹上距离当前位置最近的轨迹点（前方）
    int GetClosestNextPointIndex(const std::vector<OpCommonNS::WayPoint> &trajectory,
                                 const OpCommonNS::WayPoint &curr_pos,
                                 const int &prevIndex)
    {
        if (trajectory.size() < 2 || prevIndex < 0)
            return 0;
        double dis = 0, min_dis = DBL_MAX;
        int min_index = prevIndex;

        for (size_t i = prevIndex; i < trajectory.size(); i++)
        {
            dis = distance2pointsSqr(trajectory[i].pos, curr_pos.pos);
            double angle_diff = AngleBetweenTwoAnglesPositive(trajectory[i].pos.yaw, curr_pos.pos.yaw) * RAD2DEG;
            if (dis < min_dis && angle_diff < 45)
            {
                min_index = i;
                min_dis = dis;
            }
        }

        if (min_index < (int)trajectory.size() - 2)
        {
            OpCommonNS::GPSPoint closest, next;
            closest = trajectory[min_index].pos;
            next = trajectory[min_index + 1].pos;
            OpCommonNS::GPSPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
            double length1 = pointNorm(v_1);
            OpCommonNS::GPSPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
            double length2 = pointNorm(v_2);
            double angle = FixNegativeAngle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
            if (angle <= M_PI_2)
                min_index = min_index + 1;
        }
        return min_index;
    }

    void FixPathDensity(std::vector<OpCommonNS::WayPoint> &path, const double &pathDensity)
    {
        // 如果路径为空或者路径密度为零，则直接返回
        if (path.size() == 0 || pathDensity == 0)
            return;
        // 初始化距离和角度
        double dis = 0, ang = 0;
        // 设置路径密度的边缘值
        double margin = pathDensity * 0.01;
        // 初始化剩余距离为零
        double remaining = 0;
        // 初始化点数
        int nPoints = 0;
        // 存储修正后的路径的容器
        std::vector<OpCommonNS::WayPoint> fixedPath;
        // 将原始路径的第一个点添加到修正后的路径中
        fixedPath.push_back(path[0]);
        // 初始化起始点索引和下一个点索引
        size_t start = 0, next = 1;
        // 遍历路径中的每个点
        while (next < path.size())
        {
            // 计算两点之间的距离，并加上之前剩余的距离
            dis += hypot(path[next].pos.x - path[next - 1].pos.x, path[next].pos.y - path[next - 1].pos.y) + remaining;
            // 计算两点之间的角度
            ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);
            // 如果距离小于目标密度减去边缘值，将下一个点添加到修正后的路径中，并更新索引
            if (dis < pathDensity - margin)
            {
                next++;
                remaining = 0;
                // 如果距离大于目标密度加上边缘值，进行插值，将插值点添加到修正后的路径中，并更新索引
            }
            else if (dis > (pathDensity + margin))
            {
                OpCommonNS::WayPoint point_start = path[start];
                nPoints = dis / pathDensity;
                for (int j = 0; j < nPoints; j++)
                {
                    point_start.pos.x = point_start.pos.x + pathDensity * cos(ang);
                    point_start.pos.y = point_start.pos.y + pathDensity * sin(ang);
                    fixedPath.push_back(point_start);
                }
                remaining = dis - nPoints * pathDensity;
                start++;
                path[start].pos = point_start.pos;
                dis = 0;
                next++;
                // 如果距离在目标密度加减边缘值之间，将下一个点添加到修正后的路径中，并更新索引
            }
            else
            {
                dis = 0;
                remaining = 0;
                fixedPath.push_back(path[next]);
                next++;
                start = next - 1;
            }
        }
        // 更新原始路径为修正后的路径
        path = fixedPath;
    }

    double AngleBetweenTwoAnglesPositive(const double &yaw1, const double &yaw2)
    {
        double diff = yaw1 - yaw2;
        if (diff < 0)
            diff = yaw2 - yaw1;

        if (diff > M_PI)
            diff = 2.0 * M_PI - diff;

        return diff;
    }

    double FixNegativeAngle(const double &a)
    {
        double angle = 0;
        if (a < -2.0 * M_PI || a >= 2.0 * M_PI)
        {
            angle = fmod(a, 2.0 * M_PI);
        }
        else
        {
            angle = a;
        }
        if (angle < 0)
        {
            angle = 2.0 * M_PI + angle;
        }
        return angle;
    }

    double SplitPositiveAngle(const double &a)
    {
        double angle = a;

        if (a < -2.0 * M_PI || a >= 2.0 * M_PI)
        {
            angle = fmod(a, 2.0 * M_PI);
        }

        if (angle >= M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        else if (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    // 滚动轨迹生成
    void GenerateRunoffTrajectory(
        const std::vector<std::vector<OpCommonNS::WayPoint>> &referencePaths,
        const OpCommonNS::WayPoint &carPos,
        const double &speed,
        const double &microPlanDistance,
        const double &carTipMargin,
        const double &rollInMargin,
        const double &rollInSpeedFactor,
        const double &pathDensity,
        const double &rollOutDensity,
        const int &rollOutNumber,
        const double &SmoothDataWeight,
        const double &smoothingSmoothWeight,
        const double &smoothingToleranceError,
        std::vector<std::vector<std::vector<OpCommonNS::WayPoint>>> &rollOutsPaths,
        std::vector<OpCommonNS::WayPoint> &sampledPoints)
    {
        // 如果参考路径为空或者微观规划的距离小于等于零，则直接返回
        if (referencePaths.size() == 0)
            return;
        if (microPlanDistance <= 0)
            return;
        // 清空输出的候选路径和采样点
        rollOutsPaths.clear();
        sampledPoints.clear();

        // 遍历每个参考路径
        for (unsigned int i = 0; i < referencePaths.size(); i++)
        {
            // 存储局部备选路径的容器
            std::vector<std::vector<OpCommonNS::WayPoint>> local_rollOutPaths;
            // 记录备选路径的起始和结束索引以及每个结束点到最后一个点的距离
            int s_index = 0, e_index = 0;
            std::vector<double> e_distances;
            // 如果参考路径不为空，则计算备选路径
            if (referencePaths.at(i).size() > 0)
            {
                CalculateRollInTrajectories(carPos,
                                            speed,
                                            referencePaths.at(i),
                                            s_index, e_index, e_distances,
                                            local_rollOutPaths,
                                            microPlanDistance,
                                            carTipMargin,
                                            rollInMargin,
                                            rollInSpeedFactor,
                                            pathDensity,
                                            rollOutDensity,
                                            rollOutNumber,
                                            SmoothDataWeight, smoothingSmoothWeight, smoothingToleranceError,
                                            sampledPoints);
            }
            else
            {
                for (int j = 0; j < rollOutNumber + 1; j++)
                {
                    local_rollOutPaths.push_back(vector<OpCommonNS::WayPoint>());
                }
            }
            // 将当前参考路径的备选路径添加到输出容器中
            rollOutsPaths.push_back(local_rollOutPaths);
        }
    }

    void CalculateRollInTrajectories(const OpCommonNS::WayPoint &carPos,
                                     const double &speed,
                                     const std::vector<OpCommonNS::WayPoint> &originalCenter,
                                     int &start_index,
                                     int &end_index,
                                     std::vector<double> &end_laterals,
                                     std::vector<std::vector<OpCommonNS::WayPoint>> &rollInPaths,
                                     const double &microPlanDistance,
                                     const double &carTipMargin,
                                     const double &rollInMargin,
                                     const double &rollInSpeedFactor,
                                     const double &pathDensity,
                                     const double &rollOutDensity,
                                     const int &rollOutNumber,
                                     const double &smoothingDataWeight,
                                     const double &smoothingSmoothWeight,
                                     const double &smoothingToleranceError,
                                     std::vector<OpCommonNS::WayPoint> &sampledPoints)
    {
        OpCommonNS::WayPoint p;
        // 计算车辆前端范围的索引限制
        int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
        if (iLimitIndex >= originalCenter.size())
            iLimitIndex = originalCenter.size() - 1;

        // Get Closest Index
        //  获取车辆当前位置到中心轨迹的相对信息
        OpCommonNS::RelativeInfo info;
        GetRelativeInfo(originalCenter, carPos, info);

        double remaining_distance = 0;
        int close_index = info.iBack;
        for (unsigned int i = close_index; i < originalCenter.size() - 1; i++)
        {
            if (i > 0)
                remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
        }

        double initial_roll_in_distance = info.perp_distance; // GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

        std::vector<OpCommonNS::WayPoint> RollOutStratPath;

        // 计算起始索引
        double d_limit = 0;
        unsigned int far_index = close_index;

        double start_distance = rollInSpeedFactor * speed + rollInMargin;
        if (start_distance > remaining_distance)
            start_distance = remaining_distance;

        // 计算实际计算的起始索引
        d_limit = 0;
        for (unsigned int i = close_index; i < originalCenter.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
            if (d_limit >= start_distance)
            {
                far_index = i;
                break;
            }
        }

        int centralTrajectoryIndex = rollOutNumber / 2;
        std::vector<double> end_distance_list;
        for (int i = 0; i < rollOutNumber + 1; i++)
        {
            double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
            end_distance_list.push_back(end_roll_in_distance);
        }

        start_index = close_index;
        end_index = far_index; // end_index是第二个阶段结尾的点坐标
        end_laterals = end_distance_list;

        // calculate the actual calculation starting index
        d_limit = 0;
        unsigned int smoothing_start_index = start_index;
        unsigned int smoothing_end_index = end_index;

        // 插入路径的点并计算横向偏移
        for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
            if (d_limit > carTipMargin)
                break;
            smoothing_start_index++; // 这个是一个阶段结尾的点下标
        }

        d_limit = 0;
        for (unsigned int i = end_index; i < originalCenter.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
            if (d_limit > carTipMargin)
                break;

            smoothing_end_index++;
        }

        int nSteps = end_index - smoothing_start_index;

        std::vector<double> inc_list;
        rollInPaths.clear();
        std::vector<double> inc_list_inc;
        for (int i = 0; i < rollOutNumber + 1; i++)
        {
            double diff = end_laterals.at(i) - initial_roll_in_distance;
            inc_list.push_back(diff / (double)nSteps);
            rollInPaths.push_back(std::vector<OpCommonNS::WayPoint>());
            inc_list_inc.push_back(0);
        }

        std::vector<std::vector<OpCommonNS::WayPoint>> execluded_from_smoothing;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
            execluded_from_smoothing.push_back(std::vector<OpCommonNS::WayPoint>());

        // fs: 插入 carTip 部分轨迹
        for (unsigned int j = start_index; j < smoothing_start_index; j++)
        {
            p = originalCenter.at(j);
            double original_speed = p.v;
            for (unsigned int i = 0; i < rollOutNumber + 1; i++)
            {
                p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2);
                p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2);

                if (i != centralTrajectoryIndex)
                    p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
                else
                    p.v = original_speed;

                if (j < iLimitIndex)
                    execluded_from_smoothing.at(i).push_back(p);
                else
                    rollInPaths.at(i).push_back(p);

                sampledPoints.push_back(p);
            }
        }

        // 插入 rollin 部分
        for (unsigned int j = smoothing_start_index; j < end_index; j++)
        {
            p = originalCenter.at(j);
            double original_speed = p.v;
            for (unsigned int i = 0; i < rollOutNumber + 1; i++)
            {
                inc_list_inc[i] += inc_list[i];
                double d = inc_list_inc[i];
                p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2) - d * cos(p.pos.yaw + M_PI_2);
                p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2) - d * sin(p.pos.yaw + M_PI_2);

                if (i != centralTrajectoryIndex)
                    p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
                else
                    p.v = original_speed;

                rollInPaths.at(i).push_back(p);

                sampledPoints.push_back(p);
            }
        }

        // Insert last strait points to make better smoothing
        for (unsigned int j = end_index; j < smoothing_end_index; j++)
        {
            p = originalCenter.at(j);
            double original_speed = p.v;
            for (unsigned int i = 0; i < rollOutNumber + 1; i++)
            {
                double d = end_laterals.at(i);
                p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
                p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);
                if (i != centralTrajectoryIndex)
                    p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
                else
                    p.v = original_speed;
                rollInPaths.at(i).push_back(p);
                sampledPoints.push_back(p);
            }
        }

        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
            rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

        d_limit = 0;
        for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
        {
            if (j > 0)
                d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

            if (d_limit > microPlanDistance)
                break;
            p = originalCenter.at(j);
            double original_speed = p.v;
            for (unsigned int i = 0; i < rollInPaths.size(); i++)
            {
                double d = end_laterals.at(i);
                p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
                p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);

                if (i != centralTrajectoryIndex)
                    p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
                else
                    p.v = original_speed;

                rollInPaths.at(i).push_back(p);

                sampledPoints.push_back(p);
            }
        }

        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            // 轨迹平滑处理
            SmoothPath(rollInPaths.at(i), smoothingDataWeight, smoothingSmoothWeight, smoothingToleranceError);
        }
    }

    // 平滑生成的曲线
    void SmoothPath(std::vector<OpCommonNS::WayPoint> &path, double weight_data,
                    double weight_smooth, double tolerance)
    {
        if (path.size() <= 2)
            return;

        const std::vector<OpCommonNS::WayPoint> &path_in = path;
        std::vector<OpCommonNS::WayPoint> smoothPath_out = path_in;

        double change = tolerance;
        double xtemp, ytemp;
        int nIterations = 0;

        int size = path_in.size();

        while (change >= tolerance)
        {
            change = 0.0;
            for (int i = 1; i < size - 1; i++)
            {
                xtemp = smoothPath_out[i].pos.x;
                ytemp = smoothPath_out[i].pos.y;

                smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
                smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

                smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
                smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

                change += fabs(xtemp - smoothPath_out[i].pos.x);
                change += fabs(ytemp - smoothPath_out[i].pos.y);
            }
            nIterations++;
        }
        path = smoothPath_out;
    }

    /**
     * 计算车辆到某一个轨迹上点的相对位置
     * PS: 计算点与轨迹的相对位置
     */
    bool GetRelativeInfo(const std::vector<OpCommonNS::WayPoint> &trajectory,
                         const OpCommonNS::WayPoint &p,
                         OpCommonNS::RelativeInfo &info,
                         const int &prevIndex)
    {
        // 检查给定路径的点数量，如果小于2个，则无法进行相对位置计算
        if (trajectory.size() < 2)
            return false;
        OpCommonNS::WayPoint p0, p1;
        // 如果路径只有2个点，直接取第一个点和第二个点的中点作为 p0 和 p1
        if (trajectory.size() == 2)
        {
            p0 = trajectory[0];
            p1 = OpCommonNS::WayPoint(
                (p0.pos.x + trajectory[1].pos.x) / 2.0,
                (p0.pos.y + trajectory[1].pos.y) / 2.0,
                (p0.pos.z + trajectory[1].pos.z) / 2.0, p0.pos.yaw);
            info.iBack = 0;
            info.iFront = 1;
        }
        else
        {
            // 根据车辆当前位置获取最近的下一个路径点索引，并计算 p0 和 p1
            // 获取相邻路径点
            info.iFront = GetClosestNextPointIndex(trajectory, p, prevIndex);
            if (info.iFront > 0)
                info.iBack = info.iFront - 1;
            else
                info.iBack = 0;

            if (info.iFront == 0)
            {
                p0 = trajectory[info.iFront];
                p1 = trajectory[info.iFront + 1];
            }
            else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
            {
                p0 = trajectory[info.iFront - 1];
                p1 = trajectory[info.iFront];
            }
            else
            {
                p0 = trajectory[info.iFront - 1];
                p1 = OpCommonNS::WayPoint(
                    (p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                    (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                    (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0, p0.pos.yaw);
            }
        }
        // 坐标系变换
        OpCommonNS::WayPoint prevWP = p0;
        OpCommonNS::Mat3 rotationMat(-p1.pos.yaw);
        OpCommonNS::Mat3 translationMat(-p.pos.x, -p.pos.y);
        OpCommonNS::Mat3 invRotationMat(p1.pos.yaw);
        OpCommonNS::Mat3 invTranslationMat(p.pos.x, p.pos.y);

        p0.pos = translationMat * p0.pos;
        p0.pos = rotationMat * p0.pos;

        p1.pos = translationMat * p1.pos;
        p1.pos = rotationMat * p1.pos;

        double k = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
        // 计算横向偏移量 info.perp_distance，表示车辆当前位置到路径的垂直距离
        info.perp_distance = p1.pos.y - k * p1.pos.x;

        if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
            info.perp_distance = 0;
        // 计算到下一个路径点的距离 info.to_front_distance
        info.to_front_distance = fabs(p1.pos.x);

        info.perp_point = p1;
        info.perp_point.pos.x = 0;
        info.perp_point.pos.y = info.perp_distance;

        info.perp_point.pos = invRotationMat * info.perp_point.pos;
        info.perp_point.pos = invTranslationMat * info.perp_point.pos;
        // 计算车辆当前位置到路径上最近点的距离
        info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
        // 计算车辆当前朝向与路径朝向的角度差
        info.angle_diff = AngleBetweenTwoAnglesPositive(p1.pos.yaw, p.pos.yaw) * RAD2DEG;

        info.direct_distance = hypot(p1.pos.y - p.pos.y, p1.pos.x - p.pos.x);
        return true;
    }

    double GetExactDistanceOnTrajectory(const std::vector<OpCommonNS::WayPoint> &trajectory, const OpCommonNS::RelativeInfo &p1, const OpCommonNS::RelativeInfo &p2)
    {
        if (trajectory.size() == 0)
            return 0;

        if (p2.iFront == p1.iFront && p2.iBack == p1.iBack)
        {
            return p2.to_front_distance - p1.to_front_distance;
        }
        else if (p2.iBack >= p1.iFront)
        {
            double d_on_path = p1.to_front_distance + p2.from_back_distance;
            for (int i = p1.iFront; i < p2.iBack; i++)
                d_on_path += hypot(trajectory.at(i + 1).pos.y - trajectory.at(i).pos.y, trajectory.at(i + 1).pos.x - trajectory.at(i).pos.x);

            return d_on_path;
        }
        else if (p2.iFront <= p1.iBack)
        {
            double d_on_path = p1.from_back_distance + p2.to_front_distance;
            for (int i = p2.iFront; i < p1.iBack; i++)
                d_on_path += hypot(trajectory.at(i + 1).pos.y - trajectory.at(i).pos.y, trajectory.at(i + 1).pos.x - trajectory.at(i).pos.x);

            return -d_on_path;
        }
        else
        {
            return 0;
        }
    }

    void ConvertCollisionPointsMarkers(const std::vector<OpCommonNS::WayPoint> &col_points, visualization_msgs::MarkerArray &collision_markers, visualization_msgs::MarkerArray &collision_markers_d)
    {
        collision_markers = collision_markers_d;
        for (unsigned int i = 0; i < col_points.size(); i++)
        {
            // 根据bBlocked字段确定标记颜色
            // col_points.at(i).level
            double r, g, b;
            switch (col_points.at(i).level)
            {
            case -1:
                r = 0.5;
                g = 0.5;
                b = 0.5; // 灰色
                break;
            case 0:
                r = 0.0;
                g = 1.0;
                b = 0.0; // 绿色
                break;
            case 1:
                // 设置为黄色
                r = 1.0;
                g = 1.0;
                b = 0.0;
                break;
            case 2:
                // 设置为红色
                r = 1.0;
                g = 0.0;
                b = 0.0;
                break;
            default:
                // 如果level不在0、1、2范围内，默认设置为蓝色
                r = 0.0;
                g = 0.0;
                b = 1.0;
                break;
            }
            // double r = col_points.at(i).bBlocked ? 1.0 : 0.0;  // 如果bBlocked为true，则为红色，否则为绿色
            // double g = col_points.at(i).bBlocked ? 0.0 : 1.0;  // 如果bBlocked为true，则为绿色，否则为红色
            // double b = 0;
            visualization_msgs::Marker mkr = CreateGenMarker(col_points.at(i).pos.x, col_points.at(i).pos.y, col_points.at(i).pos.z, 0, r, g, b, 0.2, i, "collision_points_rviz", visualization_msgs::Marker::SPHERE);

            if (i < collision_markers.markers.size())
                collision_markers.markers.at(i) = mkr;
            else
                collision_markers.markers.push_back(mkr);
        }
    }

    void InitCollisionPointsMarkers(const int &nMarkers, visualization_msgs::MarkerArray &col_points)
    {
        col_points.markers.clear();
        for (int i = 0; i < nMarkers; i++)
        {
            visualization_msgs::Marker mkr = CreateGenMarker(0, 0, 0, 0, 1, 1, 1, 0.2, i, "collision_points_rviz", visualization_msgs::Marker::SPHERE);
            col_points.markers.push_back(mkr);
        }
    }

    visualization_msgs::Marker CreateGenMarker(const double &x, const double &y, const double &z, const double &a,
                                               const double &r, const double &g, const double &b, const double &scale, const int &id, const std::string &ns, const int &type)
    {
        visualization_msgs::Marker mkr;
        mkr.header.frame_id = "map";
        mkr.header.stamp = ros::Time();
        mkr.ns = ns;
        mkr.type = type;
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.scale.x = scale;
        mkr.scale.y = scale;
        mkr.scale.z = scale;
        mkr.color.a = 0.8;
        mkr.color.r = r;
        mkr.color.g = g;
        mkr.color.b = b;
        mkr.pose.position.x = x;
        mkr.pose.position.y = y;
        mkr.pose.position.z = z;
        mkr.pose.orientation = tf::createQuaternionMsgFromYaw(a);
        mkr.id = id;
        return mkr;
    }

}