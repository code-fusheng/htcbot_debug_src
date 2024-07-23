#ifndef OP_UTILS_H
#define OP_UTILS_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include <htcbot_msgs/Lane.h>
#include <htcbot_msgs/Waypoint.h>
#include <htcbot_msgs/DetectedObject.h>

using namespace std;
using namespace OpCommonNS;

namespace OpUtilsNS {

#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)
#define RAD2DEG 180. / M_PI
#define LANE_CHANGE_SPEED_FACTOR 0.5

void GetPlanningParams(ros::NodeHandle& _nh, OpCommonNS::PlanningParams& planningParams);

void UpdatePlanningParams(ros::NodeHandle& _nh, OpCommonNS::PlanningParams& planningParams);

void ConvertMsgLane2LocalLane(const htcbot_msgs::Lane& msg_path, std::vector<OpCommonNS::WayPoint>& path);

void ConvertLocalLane2MsgLane(const std::vector<OpCommonNS::WayPoint>& path, htcbot_msgs::Lane& msg_lane);

void SafetyRectangleToMarkers(const std::vector<OpCommonNS::GPSPoint>& safety_rect,
		visualization_msgs::Marker& marker);

void TrajectoriesToColoredMarkers(const std::vector<std::vector<OpCommonNS::WayPoint> >& paths, 
    const std::vector<OpCommonNS::TrajectoryCost>& traj_costs, 
    visualization_msgs::MarkerArray& markerArray, 
    const int& iClosest = -1);

void TrajectoryToMarker(const std::vector<OpCommonNS::WayPoint>& path, visualization_msgs::Marker& marker);

void TrajectoriesToMarkers(const std::vector<std::vector<std::vector<OpCommonNS::WayPoint> > >& paths, visualization_msgs::MarkerArray& markerArray);

double CalcAngleAndCost(std::vector<OpCommonNS::WayPoint>& path);

double FixNegativeAngle(const double& a);

double SplitPositiveAngle(const double& a);

// 用于从给定路径中提取一个部分，使得路径从当前位置开始，沿着车辆的方向延伸一定距离
void ExtractPartFromPointToDistance(const std::vector<OpCommonNS::WayPoint>& originalPath, 
    const OpCommonNS::WayPoint& pos, 
    const double& minDistance,
    const double& pathDensity, std::vector<OpCommonNS::WayPoint>& extractedPath);

int GetClosestNextPointIndex(const std::vector<OpCommonNS::WayPoint>& trajectory,
    const OpCommonNS::WayPoint& curr_pos,
    const int& prevIndex = 0);

void FixPathDensity(std::vector<OpCommonNS::WayPoint>& path, const double& pathDensity);

double AngleBetweenTwoAnglesPositive(const double& yaw1, const double& yaw2);

void GenerateRunoffTrajectory(
    const std::vector<std::vector<OpCommonNS::WayPoint>>& referencePaths,
    const OpCommonNS::WayPoint& carPos, 
    const double& speed,
    const double& microPlanDistance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& smoothingSmoothWeight,
    const double& smoothingToleranceError,
	std::vector<std::vector<std::vector<OpCommonNS::WayPoint>>>& rollOutsPaths,
	std::vector<OpCommonNS::WayPoint>& sampledPoints);

void CalculateRollInTrajectories(const OpCommonNS::WayPoint& carPos,
    const double& speed,
    const std::vector<OpCommonNS::WayPoint>& originalCenter,
    int& start_index,
    int& end_index,
    std::vector<double>& end_laterals,
    std::vector<std::vector<OpCommonNS::WayPoint>>& rollInPaths,
    const double& microPlanDistance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& smoothingDataWeight,
    const double& smoothingSmoothWeight,
    const double& smoothingToleranceError,
    std::vector<OpCommonNS::WayPoint>& sampledPoints);

bool GetRelativeInfo(const std::vector<OpCommonNS::WayPoint>& trajectory, 
    const OpCommonNS::WayPoint& p, 
    OpCommonNS::RelativeInfo& info, 
    const int& prevIndex = 0);

double GetExactDistanceOnTrajectory(const std::vector<OpCommonNS::WayPoint>& trajectory, const OpCommonNS::RelativeInfo& p1,const OpCommonNS::RelativeInfo& p2);

void SmoothPath(std::vector<OpCommonNS::WayPoint>& path, double weight_data,
    double weight_smooth, double tolerance);

void ConvertCollisionPointsMarkers(const std::vector<OpCommonNS::WayPoint>& col_pointss, visualization_msgs::MarkerArray& collision_markers, visualization_msgs::MarkerArray& collision_markers_d);

void InitCollisionPointsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& col_points);

visualization_msgs::Marker CreateGenMarker(const double& x, const double& y, const double& z,const double& a,
		const double& r, const double& g, const double& b, const double& scale, const int& id, const std::string& ns, const int& type);

}

#endif //OP_UTILS_H