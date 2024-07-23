/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:56
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-24 15:29:24
 * @Description: 
 */
#ifndef WAYPOINT_COMMON_H
#define WAYPOINT_COMMON_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>
#include <thread>
#include <cmath>

using namespace std;

namespace WaypointCommonNS {

#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))

}

#endif //WAYPOINT_COMMON_H
