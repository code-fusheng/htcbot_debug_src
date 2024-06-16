/*
 * @Author: code-fusheng
 * @Date: 2024-04-22 20:28:03
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-22 20:31:37
 * @Description: 
 */
#include <ros/ros.h>
#include "pcd_map_grid.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_map_grid_node");
	PcdMapGridNS::PcdMapGrid node;
	node.run();
	return 0;
}