/*
 * @Author: code-fusheng
 * @Date: 2024-05-10 11:09:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-10 11:22:33
 * @Description: 
 */
#include <ros/ros.h>
#include "laser_euclidean_cluster.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_euclidean_cluster_node");
	LaserEuclideanClusterNS::LaserEuclideanCluster node;
	node.run();
	return 0;
}