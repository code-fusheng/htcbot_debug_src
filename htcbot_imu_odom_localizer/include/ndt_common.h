/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 13:25:53
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 15:44:13
 * @Description: ndt_common
 */

#ifndef NDT_COMMON_H
#define NDT_COMMON_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>
#include <thread>

namespace NdtCommonNS {

#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))

enum class MethodType
{
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
};

enum class MappingStateExp
{
    NONE = 0,
    START = 1,
    FINISH = -1,
    STOP = 2,
};

class Pose {
public:
    double x;     // x
    double y;     // y
    double z;     // z
    double roll;  // | x
    double pitch; // | y
    double yaw;   // | z

    Pose()
    {
        x = 0;
        y = 0;
        z = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
};

}


#endif //NDT_COMMON_H
