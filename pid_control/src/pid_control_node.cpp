/*
 * @Author: code-fusheng
 * @Date: 2024-05-26 22:28:30
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-26 22:49:24
 * @Description: 
 */

#include <ros/ros.h>
#include "pid_control.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_control_node");
	PidControlNS::PidControl node;
	node.run();
	return 0;
}