/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 12:45:11
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-17 18:59:20
 * @Description: 
 */
#include <ros/ros.h>
#include <iostream>
#include "ndt_localizer_pro.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ndt_localizer_pro_node");
	NdtLocalizerProNS::NdtLocalizerPro node;
	node.run();
	return 0;
}