/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-04-04 16:15:53
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-05 00:42:30
 * @FilePath: /src/core/planning/op_local_planner/nodes/op_trajectory_evaluator/op_trajectory_evaluator_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <iostream>
#include "op_trajectory_evaluator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_trajectory_evaluator_node");
	OpTrajectoryEvaluatorNS::OpTrajectoryEvaluator node;
	node.run();
	return 0;
}
