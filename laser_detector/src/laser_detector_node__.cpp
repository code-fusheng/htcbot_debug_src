#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// 障碍物距离信息
#include <htcbot_msgs/SimpleObstacleDist.h>


class LaserDetector {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher pub_obstacle, pub_transformed_cloud, pub_ros_cloud;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_pointcloud;

    // 用于将激光扫描数据投影为点云数据
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    float bound_left, bound_right, bound_back, bound_front;
    float bound_top, bound_bottom;
    int th_num_obstacle;
    bool debug;
    int direction;
    std::string scan_frame, scan_topic, pointcloud_topic;

    // 处理接收到的激光扫描数据
    void cb_scan(const sensor_msgs::LaserScanConstPtr& input) {
        std::vector<float> x_distance;
        sensor_msgs::PointCloud2 cloud;
        // 将激光扫描线转换为点云数据类型
        projector_.transformLaserScanToPointCloud(scan_frame, *input, cloud, tfListener_);
        if (debug) {
            pub_transformed_cloud.publish(cloud);
        }

        // 将ROS格式的点云数据转换为PCL格式
        pcl::PointCloud<pcl::PointXYZ> rawCloud;
        pcl::fromROSMsg(cloud, rawCloud);
        
        pcl::PointCloud<pcl::PointXYZ> roiCloud;
        int cnt = 0;
        float distance = 0.0;
        float closest = 0.05;  // 5 cm
        // 遍历每个点
        for (int i = 0; i < rawCloud.points.size(); i++) {
            // closest points filter
            // 过滤掉最近距离内的点。
            if (-closest < rawCloud.points[i].x && rawCloud.points[i].x < closest && -closest < rawCloud.points[i].y && rawCloud.points[i].y < closest)
                continue;

            if (bound_back < rawCloud.points[i].x && rawCloud.points[i].x < bound_front && bound_left < rawCloud.points[i].y && rawCloud.points[i].y < bound_right) {
                // 将合格的点保存到x_distance中
                x_distance.push_back(rawCloud.points[i].x);
                if (debug) {
                    roiCloud.points.push_back(rawCloud.points[i]);
                }
                if ( debug ) ROS_INFO("rawCloud.points[i].x, y = %.2f, %.2f", rawCloud.points[i].x, rawCloud.points[i].y);
            }
        }

        if ( debug ) {
            sensor_msgs::PointCloud2 roiCloud_msg;
            pcl::toROSMsg(roiCloud, roiCloud_msg);
            roiCloud_msg.header.frame_id = input->header.frame_id;
            pub_ros_cloud.publish(roiCloud_msg);
        }

        htcbot_msgs::SimpleObstacleDist msg;
        msg.direction = this->direction;
        if (x_distance.size() < th_num_obstacle){
            msg.distance = 100000.0;
            pub_obstacle.publish(msg);
            return;
        }

        // 点按照x坐标进行排序，并计算障碍物距离，最后发布障碍物消息
        std::sort(x_distance.begin(), x_distance.end());
        
        double dist = 0.0;
        for (int i = 0; i < th_num_obstacle; i++) {
            dist += x_distance[i];
        }
        msg.distance = dist / th_num_obstacle;
        pub_obstacle.publish(msg);
    }

    void cb_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& input) {
        // 处理多线点云
        std::vector<float> x_distance;

        pcl::PointCloud<pcl::PointXYZ> rawCloud;
        pcl::fromROSMsg(*input, rawCloud);

        pcl::PointCloud<pcl::PointXYZ> roiCloud;
        int cnt = 0;
        float distance = 0.0;
        float closest = 0.02;  // 5 cm

        for (int i = 0; i < rawCloud.points.size(); i++) {
            // closest points filter
            // 过滤掉最近距离内的点。
            if (-closest < rawCloud.points[i].x && rawCloud.points[i].x < closest && -closest < rawCloud.points[i].y && rawCloud.points[i].y < closest)
                continue;
            // 过滤掉车体范围的点
            if (bound_back < rawCloud.points[i].x && rawCloud.points[i].x < bound_front &&
                bound_left < rawCloud.points[i].y && rawCloud.points[i].y < bound_right &&
                bound_bottom < rawCloud.points[i].z && rawCloud.points[i].z < bound_top) {
                // 将合格的点保存到x_distance中
                x_distance.push_back(rawCloud.points[i].x);
                if (debug) {
                    roiCloud.points.push_back(rawCloud.points[i]);
                }
                if ( debug ) ROS_INFO("rawCloud.points[i].x, y = %.2f, %.2f", rawCloud.points[i].x, rawCloud.points[i].y);
            }
        }
        if ( debug ) {
            sensor_msgs::PointCloud2 roiCloud_msg;
            pcl::toROSMsg(roiCloud, roiCloud_msg);
            roiCloud_msg.header.frame_id = input->header.frame_id;
            pub_ros_cloud.publish(roiCloud_msg);
        }

        htcbot_msgs::SimpleObstacleDist msg;
        msg.direction = this->direction;
        if (x_distance.size() < th_num_obstacle){
            msg.distance = 100000.0;
            pub_obstacle.publish(msg);
            return;
        }

        // 点按照x坐标进行排序，并计算障碍物距离，最后发布障碍物消息
        std::sort(x_distance.begin(), x_distance.end());
        
        double dist = 0.0;
        for (int i = 0; i < th_num_obstacle; i++) {
            dist += x_distance[i];
        }
        msg.distance = dist / th_num_obstacle;
        pub_obstacle.publish(msg);
    }

public:
    LaserDetector(): pnh("~"){}
    ~LaserDetector(){}

    void run() {
        pnh.getParam("bound_left", bound_left);
        pnh.getParam("bound_right", bound_right);
        pnh.getParam("bound_back", bound_back);
        pnh.getParam("bound_front", bound_front);
        pnh.getParam("bound_top", bound_top);
        pnh.getParam("bound_bottom", bound_bottom);
        pnh.getParam("nums_of_obstacle", th_num_obstacle);
        pnh.getParam("debug", debug);
        pnh.getParam("direction", direction);
        pnh.getParam("scan_frame", scan_frame);
        pnh.getParam("scan_topic", scan_topic);

        nh.param<std::string>("pointcloud_toopic", pointcloud_topic, "/rslidar_points");

        tfListener_.setExtrapolationLimit(ros::Duration(0.1));

        pub_obstacle = nh.advertise<htcbot_msgs::SimpleObstacleDist>("/detection/simple_obstacle_detection", 5);
        pub_transformed_cloud = nh.advertise<sensor_msgs::PointCloud2>("/LaserDetector/transformed_cloud", 5);
        pub_ros_cloud = nh.advertise<sensor_msgs::PointCloud2>("/LaserDetector/roi_cloud", 5);

        // sub_scan = nh.subscribe(scan_topic, 5, &LaserDetector::cb_scan, this);
        sub_pointcloud = nh.subscribe(pointcloud_topic, 5, &LaserDetector::cb_pointcloud, this);

        ros::spin();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "euclidean_cluster", ros::init_options::AnonymousName);

    LaserDetector detector;
    detector.run();

    return 0;
}
