#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_common_params_node");

	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}