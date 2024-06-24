#include "pcd_map_grid.h"

using namespace MapCommonNS;

namespace PcdMapGridNS {

PcdMapGrid::PcdMapGrid() : nh_private_("~") {
    f_in_folder_ = false;
    f_out_folder_ = false;
    area_list_filename_ = "pcd_info.csv";
}

PcdMapGrid::~PcdMapGrid() {
}

void PcdMapGrid::run() {
    init();
	while (ros::ok()) {
        ros::spin();
    }
}

void PcdMapGrid::init() {
    sub_grid_map_build_ = nh_.subscribe("/htcbot/grid_map_build", 3, &PcdMapGrid::callbackPcdMapGrid, this);
}

void PcdMapGrid::callbackPcdMapGrid(const htcbot_msgs::GridMapBuild::ConstPtr &msg) {
    this->setInFolder(msg->input_dir); // or input file
    this->setOutFolder(msg->output_dir);
    grid_size_ = msg->side_length;
    voxel_size_ = msg->voxel_size;
    voxel_grid_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    f_voxel_size_ = true;
    doGridPcdMapProcess();
}

void PcdMapGrid::setInFolder(const std::string &folder) {
    in_folder_ = folder;
    f_in_folder_ = true;
    if( endWith(in_folder_, ".pcd") || endWith(in_folder_, ".PCD") || startWith(in_folder_, "static") ) {
        return;
    }
    if (!endWith(in_folder_, "/"))
        in_folder_ = in_folder_ + "/";
    return;
}

void PcdMapGrid::setOutFolder(const std::string &folder) {
    out_folder_ = folder;
    f_out_folder_ = true;
    if (!endWith(out_folder_, "/"))
        out_folder_ = out_folder_ + "/";
    return;
}

bool PcdMapGrid::doGridPcdMapProcess() {
    if (!f_in_folder_ || !f_out_folder_) {
        std::cout << "confirm in_folder | out_folder first!" << std::endl;
        return false;
    }
    // 读取原始点云
    std::vector<std::string> files = readAllFiles();
    if (files.size() == 0) {
        std::cout << "No pcd file found" << std::endl;
        return false;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    for (size_t i = 0; i < files.size(); i++) {
        tmp->clear();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(files[i], *tmp) == -1) {
        std::cout << "Failed to load " << files[i] << "." << std::endl;
        }
        *map += *tmp;
        std::cout << "Finished to load " << files[i] << " " << std::endl;
    }
    std::cout << "Finished to load all PCDs: " << map->size() << " points."
                << std::endl;
    double min_x = 10000000000.0;
    double max_x = -10000000000.0;
    double min_y = 10000000000.0;
    double max_y = -10000000000.0;
    // Search minimum and maximum points along x and y axis.
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator p = map->begin(); p != map->end(); p++) {
        if (p->x < min_x) {
        min_x = p->x;
        }
        if (p->x > max_x) {
        max_x = p->x;
        }
        if (p->y < min_y) {
        min_y = p->y;
        }
        if (p->y > max_y) {
        max_y = p->y;
        }
    }

    // Find minimum and maximum boundary
    int min_x_b = grid_size_ * static_cast<int>(floor(min_x / grid_size_));
    int max_x_b = grid_size_ * static_cast<int>(floor(max_x / grid_size_) + 1);
    int min_y_b = grid_size_ * static_cast<int>(floor(min_y / grid_size_));
    int max_y_b = grid_size_ * static_cast<int>(floor(max_y / grid_size_) + 1);

    // Number of grid along x and y axis
    int div_x = (max_x_b - min_x_b) / grid_size_;
    int div_y = (max_y_b - min_y_b) / grid_size_;
    int grid_num = div_x * div_y;

    std::vector<PcdGrid> grids(grid_num);
    for (int y = 0; y < div_y; y++) {
        for (int x = 0; x < div_x; x++) {
        int id = div_x * y + x;
        grids[id].grid_id = id;
        grids[id].grid_id_x = x;
        grids[id].grid_id_y = y;
        grids[id].lower_bound_x = min_x_b + grid_size_ * x;
        grids[id].lower_bound_y = min_y_b + grid_size_ * y;
        grids[id].upper_bound_x = min_x_b + grid_size_ * (x + 1);
        grids[id].upper_bound_y = min_y_b + grid_size_ * (y + 1);
        grids[id].filename = out_folder_ + std::to_string(int(grid_size_)) + "_" +
                            std::to_string(grids[id].lower_bound_x) + "_" +
                            std::to_string(grids[id].lower_bound_y) + ".pcd";
        grids[id].name = std::to_string(int(grid_size_)) + "_" +
                        std::to_string(grids[id].lower_bound_x) + "_" +
                        std::to_string(grids[id].lower_bound_y) + ".pcd";
        }
    }

    // Assign all points to appropriate grid according to their x/y value
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator p = map->points.begin(); 
            p != map->points.end(); p++) {
        int idx = static_cast<int>(
            floor((p->x - static_cast<float>(min_x_b)) / grid_size_));
        int idy = static_cast<int>(
            floor((p->y - static_cast<float>(min_y_b)) / grid_size_));
        int id = idy * div_x + idx;

        pcl::PointXYZI tmp = *p;
        grids[id].cloud.push_back(tmp);
    }

    int points_num = 0;
    for (int i = 0; i < grid_num; i++) {
        if (grids[i].cloud.points.size() > 0) {
            try {
                if (this->f_voxel_size_) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cur(new pcl::PointCloud<pcl::PointXYZI>());
                    for (auto p : grids[i].cloud) {
                        pcl::PointXYZI t = p;
                        cur->push_back(t);
                    }
                    voxel_grid_filter_.setInputCloud(cur);
                    voxel_grid_filter_.filter(*scan_ptr);
                    pcl::io::savePCDFileBinary(grids[i].filename, *scan_ptr);
                    std::cout << "Wrote " << scan_ptr->points.size() << " points to "
                                << grids[i].filename << "." << std::endl;
                    points_num += scan_ptr->points.size();
                } else {
                    pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
                    std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                                << grids[i].filename << "." << std::endl;
                    points_num += grids[i].cloud.points.size();
                }
            } catch (...) {
                std::cout << "IO exception" << std::endl;
            }
        }
    }
    writeCsv(grids);
    std::cout << "Total points num: " << points_num << " points." << std::endl;
    return true;
}

bool PcdMapGrid::endWith(const std::string &original, const std::string &pattern) {
  return original.length() >= pattern.length() &&
         original.substr(original.length() - pattern.length()) == pattern;
}

bool PcdMapGrid::startWith(const std::string &original, const std::string &pattern) {
    return original.find(pattern) == 0;
}

std::vector<std::string> PcdMapGrid::readAllFiles() {
  // for single pcd file
  if (endWith(in_folder_, ".pcd") || endWith(in_folder_, ".PCD") || startWith(in_folder_, "static")) {
    return std::vector<std::string>{in_folder_};
  }

  // for pcd files folder
  DIR *dir;
  struct dirent *ptr;
  char base[1000];
  if ((dir = opendir(in_folder_.c_str())) == NULL) {
    perror("Open dir error...");
    std::cout << "Check: " << in_folder_ << std::endl;
    exit(1);
  }
  std::vector<std::string> files;
  while ((ptr = readdir(dir)) != NULL) {
    if (ptr->d_type == 8) // 文件
    {
      std::string name = ptr->d_name;
      if (endWith(name, ".pcd") || endWith(name, ".PCD") || startWith(in_folder_, "static")) {
        // std::cout << _in_folder + name << std::endl;
        files.push_back(in_folder_ + name); // 记录所有pcd文件名
      }
    }
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
  return files;
}

bool PcdMapGrid::writeCsv(const std::vector<PcdGrid> &grids) {
    std::string whole_file_name = out_folder_ + area_list_filename_;
    std::cout << "csv file name = " << whole_file_name << std::endl;
    std::ofstream ofs(whole_file_name.c_str());
    int grid_num = grids.size();
    for (int i = 0; i < grid_num; i++) {
        if (grids[i].cloud.points.size() > 0) {
        ofs << grids[i].name << "," << grids[i].lower_bound_x << ","
            << grids[i].lower_bound_y << "," << 0.0 << ","
            << grids[i].upper_bound_x << "," << grids[i].upper_bound_y << ","
            << 0.0 << std::endl;
        }
    }
    return true;
}

}