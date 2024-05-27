/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:30
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-26 22:38:56
 * @Description: 
 */

#include <ros/ros.h>
#include "cpp_base.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cpp_base_node");
	CppBaseNS::CppBase node;
	node.run();
	return 0;
}