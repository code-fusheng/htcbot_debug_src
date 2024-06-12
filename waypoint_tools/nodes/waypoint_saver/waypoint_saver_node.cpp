/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-24 13:03:51
 * @Description: 
 */
#include <ros/ros.h>
#include "waypoint_saver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_saver_node");
	WaypointSaverNS::WaypointSaver node;
	node.run();
	return 0;
}