/*
 * @Author: code-fusheng
 * @Date: 2024-04-25 09:56:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-08 20:02:32
 * @Description:
 */
#ifndef CPP_DEMO_H
#define CPP_DEMO_H

#include <cpp_common.h>
#include <htcbot_common.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <demo_tools/cpp_demoConfig.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <htcbot_msgs/SwitchStatusSrv.h>
#include <htcbot_msgs/SwitchStatusSrvRequest.h>
#include <htcbot_msgs/SwitchStatusSrvResponse.h>

using namespace HtcbotCommonNS;
namespace CppDemoNS
{

    class CppDemo
    {

    public:
        CppDemo();
        ~CppDemo();
        void run();
        void init();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::ServiceServer srv_switch_status_;

        // 声明动态重配置服务器
        dynamic_reconfigure::Server<demo_tools::cpp_demoConfig> server_;

        bool is_debug_;
        int switch_status_;

        void dynamicReconfigureCallback(demo_tools::cpp_demoConfig &config, uint32_t level);
        bool setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res);
    };

}

#endif // CPP_DEMO_H