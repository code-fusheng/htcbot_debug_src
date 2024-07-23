#include <ros/ros.h>

#include "pure_pursuit.h"

namespace PurePursuitNS {

double PurePursuit::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2) {
    // distance between target 1 and target2 in 2-D
    tf::Vector3 v1 = point2vector(target1);
    v1.setZ(0);
    tf::Vector3 v2 = point2vector(target2);
    v2.setZ(0);
    return tf::tfDistance(v1, v2);
}

double PurePursuit::getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c) {
    double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

    return d;
}

tf::Vector3 PurePursuit::rotateUnitVector(tf::Vector3 unit_vector, double degree) {
    tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(), sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
    tf::Vector3 unit_w1 = w1.normalize();
    return unit_w1;
}

tf::Vector3 PurePursuit::point2vector(geometry_msgs::Point point) {
    tf::Vector3 vector(point.x, point.y, point.z);
    return vector;
}

bool PurePursuit::getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c) {
    //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
    double sub_x = fabs(start.x - end.x);
    double sub_y = fabs(start.y - end.y);
    double error = pow(10, -5);  // 0.00001

    if (sub_x < error && sub_y < error) {
        ROS_INFO("two points are the same point!!");
        return false;
    }

    *a = end.y - start.y;
    *b = (-1) * (end.x - start.x);
    *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

    return true;
}

geometry_msgs::Point PurePursuit::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
    // calculation relative coordinate of point from current_pose frame
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);

    return tf_point_msg;
}

geometry_msgs::Point PurePursuit::calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = inverse * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

// 
double PurePursuit::calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose)
{
  double radius;
  double denominator = 2 * calcRelativeCoordinate(target, current_pose).y;
  double numerator = pow(getPlaneDistance(target, current_pose.position), 2);

  if (denominator != 0)
    radius = numerator / denominator;
  else
    radius = 0;

  // ROS_INFO("radius : %lf", radius);
  return radius;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> PurePursuit::generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose)
{
  std::vector<geometry_msgs::Point> traj_circle_array;
  double radius = calcRadius(target, current_pose);
  double range = M_PI / 8;
  double increment = 0.01;

  for (double i = 0; i < range; i += increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  // reverse vector
  std::reverse(traj_circle_array.begin(), traj_circle_array.end());

  for (double i = 0; i > (-1) * range; i -= increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  return traj_circle_array;
}

htcbot_msgs::Waypoint PurePursuit::getPoseOfNextWaypoint()
{
    next_waypoint_ = current_waypoints_.at(next_waypoint_number_);
    return next_waypoint_;
}

}  // namespace PurePursuitNS