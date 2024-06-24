/*
 * @Author: code-fusheng
 * @Date: 2024-04-19 13:45:04
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 15:02:29
 * @Description: 
 */
#include <ros/ros.h>
#include <iostream>
#include "map_loader.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_loader_node");
	MapLoaderNS::MapLoader node;
	node.run();
	return 0;
}