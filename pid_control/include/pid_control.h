/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:45
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-26 22:49:56
 * @Description: cpp 基础 ROS 工程
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <pid_control/PidControlConfig.h>

namespace PidControlNS {

class PidControl {

public:

    PidControl();
    ~PidControl();
    void run();
    void init();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // 声明动态重配置服务器
    dynamic_reconfigure::Server<pid_control::PidControlConfig> server_;

    void dynamicReconfigureCallback(pid_control::PidControlConfig &config, uint32_t level);
};

}

#endif //PID_CONTROL_H


