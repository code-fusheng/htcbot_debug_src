/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-27 10:58:30
 * @Description: 
 */

#include "cpp_base.h"

namespace CppBaseNS {

CppBase::CppBase() : nh_private_("~") {
}

CppBase::~CppBase() {
}

void CppBase::run() {
    init();
    ros::spin();
}

void CppBase::init() {
    server_.setCallback(boost::bind(&CppBase::dynamicReconfigureCallback, this, _1, _2)); 
}

void CppBase::dynamicReconfigureCallback(demo_tools::CppBaseConfig &config, uint32_t level) {
}

}