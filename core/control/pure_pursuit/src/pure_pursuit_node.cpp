/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:59:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-10 14:10:01
 * @Description: 
 */
#include <ros/ros.h>
#include "pure_pursuit.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pure_pursuit_node");
	PurePursuitNS::PurePursuit node;
	node.run();
	return 0;
}