/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-26 22:50:52
 * @Description: 
 */

#include "pid_control.h"

namespace PidControlNS {

PidControl::PidControl() : nh_private_("~") {
}

PidControl::~PidControl() {
}

void PidControl::run() {
    init();
    ros::spin();
}

void PidControl::init() {
    server_.setCallback(boost::bind(&PidControl::dynamicReconfigureCallback, this, _1, _2)); 
}

void PidControl::dynamicReconfigureCallback(pid_control::PidControlConfig &config, uint32_t level) {
}

}