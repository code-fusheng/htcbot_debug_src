/*
 * @Author: code-fusheng
 * @Date: 2024-04-19 14:11:58
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-22 21:17:58
 * @Description: 
 */
#ifndef MAP_COMMON_H
#define MAP_COMMON_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace MapCommonNS {

class Area
{
public:
	std::string filename;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
	sensor_msgs::PointCloud2 points;

    Area() {
        filename = "";
    	x_min = 0;
	    y_min = 0;
	    z_min = 0;
	    x_max = 0;
	    y_max = 0;
	    z_max = 0;
    }

};

typedef std::vector<Area> AreaList;

class PcdGrid
{
public:
    std::string filename;
    std::string name;
    int grid_id;
    int grid_id_x;
    int grid_id_y;
    int lower_bound_x;
    int lower_bound_y;
    int upper_bound_x;
    int upper_bound_y;
    pcl::PointCloud<pcl::PointXYZI> cloud;
};


}


#endif //MAP_COMMON_H