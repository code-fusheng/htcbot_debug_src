/*
 * @Author: code-fusheng
 * @Date: 2024-04-02 18:10:11
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-21 12:55:14
 * @Description: 
 */
#ifndef OP_COMMON_H
#define OP_COMMON_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

namespace OpCommonNS {

enum POSITION_TYPE {
    FRONT, BACK, LEFT, RIGHT, NONE
};

#define MIN(x,y) (x <= y ? x : y)
#define MAX(x,y) (x >= y ? x : y)

class GPSPoint {
public:
    double x;
    double y;
    double z;
    double yaw;

    GPSPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        yaw = 0;
    }

    GPSPoint(const double& x, const double& y, const double& z, const double& yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
    }

    std::string ToString()
    {
        std::stringstream str;
        str.precision(12);
        str << "X:" << x << ", Y:" << y << ", Z:" << z << ", Yaw:" << yaw << std::endl;
        return str.str();
    }
};

class Rotation {
public:
    double x;
    double y;
    double z;
    double w;

    Rotation()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

class WayPoint {
public:
    GPSPoint pos;   // 包含位置信息的 GPSPoint 对象
    Rotation rot;   // 包含旋转信息的 Rotation 对象
    double v;       // 速度信息
    double cost;    // 成本信息
    int id;         // ID
    int laneId;     // 车道编号信息
    double 	laneChangeCost;
    bool bBlocked;
    int level;      // 0 defult 1 warn 2 danger -1 none

    WayPoint()
    {
        v = 0;
        cost = 0;
        laneId = -1;
        laneChangeCost = 0;
        bBlocked = false;
        level = 0;
    }

    WayPoint(const double& x, const double& y, const double& z, const double& a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.yaw = a;

        v = 0;
        cost = 0;
        laneId = -1;
        laneChangeCost = 0;
        bBlocked = false;
        level = 0;
    }
};

class Mat3 {
    double m[3][3];

public:
    Mat3()
    {
        //initialize Identity by default
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = 0;

        m[0][0] = m[1][1] = m[2][2] = 1;
    }

    Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
    {
        m[0][0] = (mirrorX == true) ? -1 : 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = (mirrorY == true) ? -1 : 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double transX, double transY)
    {
        m[0][0] = 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double rotation_angle)
    {
        double c = cos(rotation_angle);
        double s = sin(rotation_angle);
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = 0;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = 0;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(GPSPoint rotationCenter)
    {
        double c = cos(rotationCenter.yaw);
        double s = sin(rotationCenter.yaw);
        double u = rotationCenter.x;
        double v = rotationCenter.y;
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = -u * c + v * s + u;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = -u * s - v * c + v;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    GPSPoint operator*(GPSPoint v)
    {
        GPSPoint _v = v;
        v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
        v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
        return v;
    }
};

/**
 * 表示车辆相对位置信息的数据结构
*/
class RelativeInfo {
public:
    double perp_distance;       // 车辆当前位置到路径的垂直距离 中心偏移量
    double to_front_distance;   // 车辆当前位置到路径上最近点的距离(前方)
    double from_back_distance;  // 车辆当前位置到路径上最近点的距离(后方)
    int iFront;                 // 路径上最近的下一个点的索引
    int iBack;                  // 路径上最近的前一个点的索引
    int iGlobalPath;            // 车辆当前所在的全局路径的索引
    WayPoint perp_point;        // 路径上垂直投影点的坐标
    double angle_diff;          // 车辆当前朝向与路径朝向的角度差 单位为度
    bool bBefore;               // 车辆当前位置是否在路径上最近点的前方
	bool bAfter;                // 车辆当前位置是否在路径上最近点的后方
	double after_angle;         // 车辆当前位置到路径上最近点的后方方向的角度差
    double direct_distance;

    RelativeInfo()
    {
		after_angle = 0;
		bBefore = false;
		bAfter = false;
		perp_distance = 0;
		to_front_distance = 0;
		from_back_distance = 0;
		iFront = 0;
		iBack = 0;
		iGlobalPath = 0;
		angle_diff = 0;
        direct_distance = 0;
    }
};

class VehicleState
{
public:
	double speed;
	double steer;

	VehicleState()
	{
		speed = 0;
		steer = 0;
	}

};

class DetectedObject {
public:
    int id;
    std::string label;
    WayPoint center;
    std::vector<GPSPoint> contour;
    double w;
    double l;
    double h;

    ros::Time start_time;

    DetectedObject()
    {
        id = 0;
        w = 0;
        l = 0;
        h = 0;
    }
};

// 路径轨迹的成本 的结构体
class TrajectoryCost
{
public:
	int index;                              // 轨迹在整体轨迹集合中的索引
	int relative_index;                     // 轨迹相对于当前位置的相对索引
	double distance_from_center;            // 轨迹到车辆中心的距离
	double priority_cost;                   // 0 to 1 优先级(距离中心轨迹)成本，范围在0到1之间
	double transition_cost;                 // 0 to 1 过渡成本，范围在0到1之间
	double closest_obj_cost;                // 0 to 1
	double cost;                            // 总成本
	double closest_obj_distance;            // 最近物体的距离 
	double closest_obj_velocity;            // 最近物体的速度

	int lane_index;
	double lane_change_cost;
	double lateral_cost;                    // 横向成本
	double longitudinal_cost;               // 纵向成本
	bool bBlocked;                          // 是否被阻塞
	std::vector<std::pair<int, double> > lateral_costs;


	TrajectoryCost()
	{
		lane_index = -1;
		index = -1;
		relative_index = -100;
		closest_obj_velocity = 0;
		priority_cost = 0;
		transition_cost = 0;
		closest_obj_cost = 0;
		distance_from_center = 0;
		cost = 0;
		closest_obj_distance = -1;
		lane_change_cost = 0;
		lateral_cost = 0;
		longitudinal_cost = 0;
		bBlocked = false;
	}

	std::string ToString()
	{
		std::ostringstream str;
		str.precision(4);
		str << "LI   : " << lane_index;
		str << ", In : " << relative_index;
		str << ", Co : " << cost;
		str << ", Pr : " << priority_cost;
		str << ", Tr : " << transition_cost;
		str << ", La : " << lateral_cost;
		str << ", Lo : " << longitudinal_cost;
		str << ", Ln : " << lane_change_cost;
		str << ", Bl : " << bBlocked;
		str << "\n";
		for (unsigned int i=0; i<lateral_costs.size(); i++ )
		{
			str << " - (" << lateral_costs.at(i).first << ", " << lateral_costs.at(i).second << ")";
		}

		return str.str();

	}
};

/**
 * 射线法（Ray Casting Algorithm）来判断点是否在多边形内部
 * 首先，初始化一个计数器 counter 为 0。
 * 然后，遍历多边形的所有边，依次计算每条边与射线的交点个数。
 * 如果交点个数为奇数，则点在多边形内部；如果交点个数为偶数，则点在多边形外部。
 * 返回值为 1 表示点在多边形内部，返回值为 0 表示点在多边形外部。
*/
class PolygonShape
{
public:
	std::vector<GPSPoint> points;

	inline int PointInsidePolygon(const PolygonShape& polygon,const GPSPoint& p)
	{
		int counter = 0;
		  int i;
		  double xinters;
		  GPSPoint p1,p2;
		  int N = polygon.points.size();
		  if(N <=0 ) return -1;

		  p1 = polygon.points.at(0);
		  for (i=1;i<=N;i++)
		  {
		    p2 = polygon.points.at(i % N);

		    if (p.y > MIN(p1.y,p2.y))
		    {
		      if (p.y <= MAX(p1.y,p2.y))
		      {
		        if (p.x <= MAX(p1.x,p2.x))
		        {
		          if (p1.y != p2.y)
		          {
		            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
		            if (p1.x == p2.x || p.x <= xinters)
		              counter++;
		          }
		        }
		      }
		    }
		    p1 = p2;
		  }

		  if (counter % 2 == 0)
		    return 0;
		  else
		    return 1;
	}
};

class PlanningParams
{
public:

	double 	maxSpeed;
	double 	minSpeed;

	double 	carTipMargin;
	double 	rollInMargin;
	double 	rollInSpeedFactor;
	bool 	enableHeadingSmoothing;

	double 	rollOutDensity;
	int 	rollOutNumber;
	double 	pathDensity;                // 路径点的密度

	double 	planningDistance;
	double 	microPlanDistance;          // 局部轨迹生成的长度
	double 	horizonDistance;
    double 	minFollowingDistance;

	double 	smoothingDataWeight;
	double 	smoothingSmoothWeight;
	double 	smoothingToleranceError;


	double  verticalSafetyDistance;
	double  horizontalSafetyDistance;

    bool    enableObjectsPrediction;

    double  minObstacleEvaluateDistance;    // front
    double minBackObstacleEvaluateDistance;

	PlanningParams()
	{
        maxSpeed 						= 5;
		minSpeed 						= 0;
		carTipMargin					= 4.0;
		rollInMargin					= 12.0;
		rollInSpeedFactor				= 0.25;
		rollOutDensity					= 0.5;
		rollOutNumber					= 4;
		pathDensity						= 0.25;
		microPlanDistance 				= 30;
        horizonDistance					= 120;
    	minFollowingDistance			= 35;
		smoothingDataWeight				= 0.47;
		smoothingSmoothWeight			= 0.2;
		smoothingToleranceError			= 0.05;

	    verticalSafetyDistance 			= 0.0;
		horizontalSafetyDistance		= 0.0;

        minObstacleEvaluateDistance     = 2.0;

    	enableHeadingSmoothing			= false;
        enableObjectsPrediction         = false;

	}
};

class CAR_BASIC_INFO {
public:
    double wheel_base;
    double length;
    double width;
    double max_steer_angle;

    CAR_BASIC_INFO()
    {
        wheel_base = 2.7;
        length = 4.3;
        width = 1.82;
    	max_steer_angle	= 0.524;    // 30 × (π / 180)
    }
};

} // namespace OpCommonNS

#endif //OP_COMMON_H
