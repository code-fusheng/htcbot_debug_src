/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:59:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-24 13:03:17
 * @Description: 
 */
#include <ros/ros.h>
#include "cpp_demo.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cpp_demo_node");
	CppDemoNS::CppDemo node;
	node.run();
	return 0;
}