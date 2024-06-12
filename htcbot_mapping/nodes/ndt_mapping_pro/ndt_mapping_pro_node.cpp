/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 12:45:11
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-14 13:56:13
 * @Description: 
 */
#include <ros/ros.h>
#include <iostream>
#include "ndt_mapping_pro.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ndt_mapping_pro_node");
	NdtMappingProNS::NdtMappingPro node;
	node.run();
	return 0;
}