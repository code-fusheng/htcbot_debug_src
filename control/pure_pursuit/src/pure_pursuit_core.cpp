#include <ros/ros.h>

#include "pure_pursuit.h"

namespace PurePursuitNS {

// 计算曲率
bool PurePursuit::computeCurvature(double* output_curvature) {
    // search next waypoint
    // 获取了大于前视距离的最邻近点，保存在了next_waypoint_number_
    // 如果没有可以用于跟踪的目标路径点,那么会返回-1
    getNextWaypoint();
    if (next_waypoint_number_ == -1) {
        ROS_ERROR("lost next waypoint");
        return false;
    }

    // if is_linear_interpolation_ is false or next waypoint is first or last
    if (!is_linear_interpolation_ || next_waypoint_number_ == 0 || next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1))) {
        next_target_position_ = current_waypoints_.at(next_waypoint_number_).pose.pose.position;
        *output_curvature = calcCurvature(next_target_position_);
        return true;
    }

    // linear interpolation and calculate angular velocity
    bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);
    if (!interpolation) {
        ROS_INFO_STREAM("lost target! ");
        return false;
    }
    *output_curvature = calcCurvature(next_target_position_);
    return true;
}

void PurePursuit::getNextWaypoint() {
    // std::cout << "getNextWaypoint start" << std::endl;
    const int path_size = static_cast<int>(current_waypoints_.size());

    // if waypoints are not given, do nothing.
    if (path_size == 0) {
        ROS_WARN("getNextWaypoint path_size==0! Has global path be setted?");
        next_waypoint_number_ = -1;
        return;
    }

    // // look for closet point
    // double cur_dist = getPlaneDistance(current_waypoints_.at(0).pose.pose.position, current_pose_.position);
    // double pre_dist = cur_dist + 0.1;
    // int start_index = 0;
    // for (int i = 0; i < path_size; i++) {
    //     cur_dist = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
    //     if (cur_dist > pre_dist) {
    //         start_index = i;
    //         break;
    //     }
    //     pre_dist = cur_dist;
    // }
    // std::cout << "getNextWaypoint get start index: " << start_index << std::endl;
    // if(start_index == -1){
    //     ROS_ERROR("[waypoint_follower] Can't find closet waypoint!");
    //     return;
    // }


    // int start_index = 0;
    // double _dist = getPlaneDistance(current_waypoints_.at(0).pose.pose.position, current_pose_.position)+1;
    // int end = (next_waypoint_number_ > 10 ? next_waypoint_number_ : path_size);
    // for(int i = pre_index;i < end; i++){
    //     double cur_dist = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
    //     if(cur_dist < _dist){
    //         _dist = cur_dist;
    //         start_index = i;
    //     }
    // }
    // pre_index = std::max(start_index - 10, 0);

    // look for the next waypoint.
    for (int i = current_pose_closet_index; i < path_size; i++) {
        // if search waypoint is the last
        if (i == (path_size - 1)) {
            // ROS_INFO("search waypoint is the last");
            next_waypoint_number_ = i;
            return;
        }

        // if there exists an effective waypoint
        double _distance = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
        if ( _distance > lookahead_distance_) {
            next_waypoint_number_ = i;
            // std::cout << "getNextWaypoint get next waypoint" << next_waypoint_number_ << std::endl;
            return;
        }
    }

    // if this program reaches here , it means we lost the waypoint!
    next_waypoint_number_ = -1;
    ROS_WARN("[waypoint follower] Lost followed point!");
    return;
}

// linear interpolation of next target
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) {
    const double ERROR = pow(10, -5);  // 0.00001

    int path_size = static_cast<int>(current_waypoints_.size());
    if (next_waypoint == path_size - 1) {
        *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
        return true;
    }
    double search_radius = lookahead_distance_;
    geometry_msgs::Point zero_p;
    geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
    geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(x1 - x2" ,c = "(y1-y2)*x1 + (x2-x1)*y1"
    // 这一步就是根据当前计算出来的预瞄点和前一个点，得到一个直线表达式
    double a = 0;
    double b = 0;
    double c = 0;
    double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    if (!get_linear_flag) return false;

    // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
    // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
    // distance between target 1 and target2 in 2-D
    //    | a * x0 + b * y0 + c |
    // d = -------------------------------
    //          √( a~2 + b~2)
    // 这一步就是计算当前位置到前面拟合出来的直线的“点到直线的距离”
    double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

    // ROS_INFO("a : %lf ", a);
    // ROS_INFO("b : %lf ", b);
    // ROS_INFO("c : %lf ", c);
    // ROS_INFO("distance : %lf ", d);
    if (d > search_radius) return false;

    // unit vector of point 'start' to point 'end'
    // 求取这两个点的向量
    tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
    // Normalize this vector x^2 + y^2 + z^2 = 1.
    // 就是三维矢量的方向不变，但是模长变为1,变成单位向量了
    tf::Vector3 unit_v = v.normalize();

    // normal unit vectors of v
    // 将上面的单位向量分别顺时针和逆时针旋转90度
    tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
    tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise -90 degree

    // the foot of a perpendicular line
    // 现在的h1的坐标就是 当前车辆位置到前面拟合直线的垂足的坐标
    // 他的做法是将车辆当前坐标朝着垂足的方向移动了垂线的距离（当前位置到拟合直线的距离）
    geometry_msgs::Point h1;
    h1.x = current_pose_.position.x + d * unit_w1.getX();
    h1.y = current_pose_.position.y + d * unit_w1.getY();
    h1.z = current_pose_.position.z;

    geometry_msgs::Point h2;
    h2.x = current_pose_.position.x + d * unit_w2.getX();
    h2.y = current_pose_.position.y + d * unit_w2.getY();
    h2.z = current_pose_.position.z;

    // 为什么要写两个垂足，这是因为拟合直线的斜率不一样，情况是不同的，
    // 如果拟合直线的斜率是负，那么h1垂足是在直线上，保留，h2舍弃
    // 如果直线的的斜率是正的，那么h2垂足是在直线上，保留，h1舍弃
    // check which of two foot of a perpendicular line is on the line equation
    geometry_msgs::Point h;
    if (fabs(a * h1.x + b * h1.y + c) < ERROR) {
        h = h1;
        //   ROS_INFO("use h1");
    } else if (fabs(a * h2.x + b * h2.y + c) < ERROR) {
        //   ROS_INFO("use h2");
        h = h2;
    } else {
        return false;
    }

    // get intersection[s]
    // 这里是以车辆当前位置画一个圈，圈的半径就是前视距离
    // 如果计算出来的 当前位置到拟合直线的距离 大于这个圈，即拟合直线和圈没有交点，那么情况错误，函数直接返回false
    // 如果有一个交点，那么这个交点就是我们的预瞄点，如果有两个交点，那么就选取教前方那个点。
    // if there is a intersection
    if (d == search_radius) {
        *next_target = h;
        return true;
    } else {
        // if there are two intersection
        // get intersection in front of vehicle
        double s = sqrt(pow(search_radius, 2) - pow(d, 2));
        geometry_msgs::Point target1;
        target1.x = h.x + s * unit_v.getX();
        target1.y = h.y + s * unit_v.getY();
        target1.z = current_pose_.position.z;

        geometry_msgs::Point target2;
        target2.x = h.x - s * unit_v.getX();
        target2.y = h.y - s * unit_v.getY();
        target2.z = current_pose_.position.z;

        // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
        // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
        // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

        // check intersection is between end and start
        double interval = getPlaneDistance(end, start);
        if (getPlaneDistance(target1, end) < interval) {
            // ROS_INFO("result : target1");
            *next_target = target1;
            return true;
        } else if (getPlaneDistance(target2, end) < interval) {
            // ROS_INFO("result : target2");
            *next_target = target2;
            return true;
        } else {
            // ROS_INFO("result : false ");
            return false;
        }
    }
}

double PurePursuit::calcCurvature(geometry_msgs::Point target) {
    // 计算曲率
    // 曲率 = 2 * el / ld^2  el是当前位置和目标预瞄点的横向误差
    double curvature;
    double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
    double numerator = -2 * calcRelativeCoordinate(target, current_pose_).y;

    if (denominator != 0)
        curvature = numerator / denominator;
    else {
        if (numerator > 0)
            curvature = curvature_MIN_;
        else
            curvature = -curvature_MIN_;
    }
    return curvature;
}

geometry_msgs::Point PurePursuit::rotatePoint(geometry_msgs::Point point, double degree)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}

}  // namespace PurePursuitNS