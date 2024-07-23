/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:59:24
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-22 17:08:03
 * @Description: 
 */
#include <ros/ros.h>
#include "laser_detector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_detector_node");
	LaserDetectorNS::LaserDetector node;
	node.run();
	return 0;
}