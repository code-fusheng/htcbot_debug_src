/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:30
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-06-06 11:08:29
 * @Description:
 */

#include <ros/ros.h>
#include "plane_ground_filter.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane_ground_filter_node");
	PlaneGroundFilterNS::PlaneGroundFilter node;
	node.run();
	return 0;
}