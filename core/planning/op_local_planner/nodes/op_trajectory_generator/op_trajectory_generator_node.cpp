#include <ros/ros.h>
#include <iostream>
#include "op_trajectory_generator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_trajectory_generator_node");
	OpTrajectoryGeneratorNS::OpTrajectoryGenerator node;
	node.run();
	return 0;
}
