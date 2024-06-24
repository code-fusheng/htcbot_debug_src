/*
 * @Author: code-fusheng
 * @Date: 2024-04-22 20:28:28
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 16:59:54
 * @Description: 
 */
#ifndef PCD_MAP_GRID_H
#define PCD_MAP_GRID_H

#include <map_common.h>

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <string>
#include <vector>
#include <htcbot_msgs/GridMapBuild.h>


using namespace MapCommonNS;

namespace PcdMapGridNS {

class PcdMapGrid {

public:

    PcdMapGrid();
    ~PcdMapGrid();
    void run();
    void init();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string in_folder_;
    bool f_in_folder_;
    std::string out_folder_;
    bool f_out_folder_;
    bool f_grid_size_;
    bool f_voxel_size_;
    double grid_size_, voxel_size_;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;

    std::string area_list_filename_;

    std::string dir_static_map_, dir_dynamic_map_;

    ros::Subscriber sub_grid_map_build_;

    void callbackPcdMapGrid(const htcbot_msgs::GridMapBuild::ConstPtr &msg);

    void setInFolder(const std::string &folder);
    void setOutFolder(const std::string &folder);
    bool doGridPcdMapProcess();
    bool endWith(const std::string &original, const std::string &pattern);
    bool startWith(const std::string &original, const std::string &pattern);
    std::vector<std::string> readAllFiles();
    bool writeCsv(const std::vector<PcdGrid> &grids);

};

}

#endif  // PCD_MAP_GRID_H