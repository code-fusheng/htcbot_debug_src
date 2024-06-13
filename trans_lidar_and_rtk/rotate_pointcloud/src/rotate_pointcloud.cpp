#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
  

int main(int argc, char **argv)
{
    // ROS初始化
    ros::init(argc, argv, "rotate_pointcloud");
    ros::NodeHandle nh;

    // 加载点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/home/htc/Downloads/htc_robot_debug/src/trans_lidar_and_rtk/rotate_pointcloud/2105/lidar_mode/pcd_map/static_map/gps_lane_1 - Cloud.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }

    // 创建旋转矩阵
    double rotation_angle = 37.8536 * M_PI / 180.0; // 旋转度
     Eigen::Affine3d rotation_matrix = Eigen::AngleAxisd(rotation_angle, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, 0);
    


    // 应用旋转变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix);
// 保存变换后的点云到PCD文件
if (pcl::io::savePCDFileBinary("transformed.pcd", *cloud_transformed) == -1)
{
    PCL_ERROR("Couldn't write transformed point cloud to PCD file.\n");
    return (-1);
}
    // 创建两个可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer_original(new pcl::visualization::PCLVisualizer("Original Point Cloud Viewer"));
    pcl::visualization::PCLVisualizer::Ptr viewer_transformed(new pcl::visualization::PCLVisualizer("Transformed Point Cloud Viewer"));

    // 可视化原始点云和变换后的点云
    viewer_original->addPointCloud(cloud, "original_cloud");
    viewer_original->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer_original->addCoordinateSystem(1.0);
    viewer_original->initCameraParameters();

    viewer_transformed->addPointCloud(cloud_transformed, "transformed_cloud");
    viewer_transformed->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    viewer_transformed->addCoordinateSystem(1.0);
    viewer_transformed->initCameraParameters();

    // 运行两个viewer
    while (!viewer_original->wasStopped() && !viewer_transformed->wasStopped())
    {
        viewer_original->spinOnce(100);
        viewer_transformed->spinOnce(100);
        ros::spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }

    return 0;
}