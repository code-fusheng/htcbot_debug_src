/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:45
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-26 22:35:10
 * @Description: cpp 基础 ROS 工程
 */

#ifndef CPP_BASE_H
#define CPP_BASE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <demo_tools/CppBaseConfig.h>

namespace CppBaseNS {

class CppBase {

public:

    CppBase();
    ~CppBase();
    void run();
    void init();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<demo_tools::CppBaseConfig> server_;

    void dynamicReconfigureCallback(demo_tools::CppBaseConfig &config, uint32_t level);
};

}

#endif //CPP_BASE_H


