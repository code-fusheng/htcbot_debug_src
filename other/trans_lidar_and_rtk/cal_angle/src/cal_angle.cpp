#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

std::vector<Eigen::Vector2d> readVectorsFromFile(const std::string& file_path, int start_line, int end_line, int sample_interval) {
    std::vector<Eigen::Vector2d> vectors;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return vectors;
    }

    std::string line;
    int current_line = 0;
    // 跳过文件开始的行，直到开始读取的起始行
    while (current_line < start_line) {
        std::getline(file, line);
        ++current_line;
    }

    // 按照采样间隔读取指定范围内的行
    while (current_line <= end_line) {
        if (current_line >= start_line && current_line % sample_interval == 0) {
            std::getline(file, line);
            if (!line.empty()) {
                std::stringstream ss(line);
                std::string x_str, y_str;
                // 读取第一列和第二列的值
                if (std::getline(ss, x_str, ',') && std::getline(ss, y_str)) {
                    double x, y;
                    try {
                        x = std::stod(x_str);
                        y = std::stod(y_str);
                        vectors.emplace_back(x, y);  // 使用emplace_back来构造Eigen::Vector2d
                    } catch (const std::invalid_argument& e) {
                        std::cerr << "Error converting string to double at line " << current_line << ": " << e.what() << std::endl;
                    }
                }
            }
        }
        ++current_line;
    }

    file.close();
    return vectors;
}

int main() {
    // 定义CSV文件路径
    std::string file_path1 = "/home/htc/trans_ws/src/csv/2105/lidar_mode/pathes/gps_lane_1.csv";
    std::string file_path2 = "/home/htc/trans_ws/src/csv/2105/lidar_mode/pathes/lane_1.csv";
    int sample_interval =1; // 采样一次
    auto vectors1 = readVectorsFromFile(file_path1, 10, 1400, sample_interval);
    auto vectors2 = readVectorsFromFile(file_path2, 10, 1400, sample_interval);

    if (vectors1.size() != vectors2.size()) {
        std::cerr << "The size of the two vector lists does not match." << std::endl;
        return 1;
    }
    double total_angle = 0.0;
    double total_radians=0.0;
    double total_weight = 0.0;
    int count = 0;

    for (size_t i = 0; i < vectors1.size(); ++i) {
        Eigen::Vector2d vectorBefore = vectors1[i];
        Eigen::Vector2d vectorAfter = vectors2[i];

        Eigen::Vector2d normVectorBefore = vectorBefore.normalized();
        Eigen::Vector2d normVectorAfter = vectorAfter.normalized();

        double dotProduct = normVectorBefore.dot(normVectorAfter);
        double cosAngle = std::max(std::min(dotProduct, 1.0), -1.0);
        double angle_radians = std::acos(cosAngle);
        double angle_degrees = angle_radians * (180.0 / M_PI);

        // 计算前10%的数据点的数量
        int tenth_percent = static_cast<int>(vectors1.size() * 0.15);
        double weight = (i < tenth_percent) ? 0.8 : 1.0;

        // 累加加权角度和权重
        total_angle += angle_degrees * weight;
        total_radians+=angle_radians* weight;
        total_weight += weight;
        ++count;
    }
  // 计算平均旋转角度
    double average_angle = (total_weight > 0) ? (total_angle / total_weight) : 0.0;
    double average_radians = (total_weight > 0) ? (total_radians / total_weight) : 0.0;
    // 使用角度构造旋转矩阵
    Eigen::Rotation2Dd rotation(average_radians);
    Eigen::Matrix2d rotMatrix = rotation.toRotationMatrix();

// 打印旋转矩阵和角度
        ROS_INFO_STREAM("Rotation Matrix:");
        ROS_INFO_STREAM(rotMatrix);
    // 打印加权平均角度
    ROS_INFO_STREAM(" average rotation angle: " << average_angle << " degrees");

    return 0;
}