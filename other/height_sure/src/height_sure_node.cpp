
#include "height_sure_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "height_sure");

    ros::NodeHandle nh("~");

    heightsure core(nh);
    return 0;
}