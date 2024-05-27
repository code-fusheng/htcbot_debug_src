/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-08 20:23:15
 * @Description: 
 */
#include "cpp_demo.h"

using namespace CppCommonNS;

namespace CppDemoNS {

CppDemo::CppDemo() : nh_private_("~") {
    is_debug_ = 0;
    switch_status_ = 0;
}

CppDemo::~CppDemo() {
}

void CppDemo::run() {
    init();
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO_STREAM("[py_demo_node] ===> status:" << switch_status_ << "...");
        loop_rate.sleep(); // 控制循环的频率
    }
}

void CppDemo::init() {
    server_.setCallback(boost::bind(&CppDemo::dynamicReconfigureCallback, this, _1, _2)); 
    srv_switch_status_ = nh_private_.advertiseService("set_switch_status", &CppDemo::setSwitchStatusCallback, this);
}

void CppDemo::dynamicReconfigureCallback(demo_tools::cpp_demoConfig &config, uint32_t level) {
    is_debug_ = config.is_debug;
}

bool CppDemo::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res) {
    ROS_INFO("[py_demo_node] ===> req: %d", req.switch_to); 
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_; 
    return true;                      
}

}