/*
 * @Author: code-fusheng
 * @Date: 2024-04-24 12:49:35
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-06-06 17:10:29
 * @Description:
 */

#include "plane_ground_filter.h"

namespace PlaneGroundFilterNS
{

    // 比较z轴高度用
    bool pointCompareZ(pcl::PointXYZI a, pcl::PointXYZI b)
    {
        return a.z < b.z;
    }

    PlaneGroundFilter::PlaneGroundFilter() : nh_private_("~")
    {
    }

    PlaneGroundFilter::~PlaneGroundFilter()
    {
    }

    void PlaneGroundFilter::run()
    {
        init();
        ros::spin();
    }

    void PlaneGroundFilter::init()
    {
        nh_private_.param<std::string>("no_ground_topic", no_ground_topic_, "no_ground_points");
        nh_private_.param<std::string>("ground_topic", ground_topic_, "ground_points");
        nh_private_.param<double>("sensor_height", sensor_height_, 0.625);
        nh_private_.param<int>("sensor_model", sensor_model_, 16);
        nh_private_.param<int>("num_iter", num_iter_, 30);
        nh_private_.param<int>("num_lpr", num_lpr_, 20);
        nh_private_.param<double>("th_seeds", th_seeds_, 0.08);
        nh_private_.param<double>("th_dist", th_dist_, 0.08);

        server_.setCallback(boost::bind(&PlaneGroundFilter::dynamicReconfigureCallback, this, _1, _2));

        sub_pointcloud_ = nh_.subscribe("/rslidar_points", 10, &PlaneGroundFilter::pointcloudCallback, this);

        pub_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_, 10);
        pub_no_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 10);

        seeds_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        ground_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        no_ground_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        all_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    void PlaneGroundFilter::dynamicReconfigureCallback(points_tools::PlaneGroundFilterConfig &config, uint32_t level)
    {
        sensor_height_ = config.sensor_height;
        sensor_model_ = config.sensor_model;
        num_iter_ = config.num_iter;
        num_lpr_ = config.num_lpr;
        th_seeds_ = config.th_seeds;
        th_dist_ = config.th_dist;
    }

    void PlaneGroundFilter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_in;
        pcl::fromROSMsg(*input, cloud_in);
        pcl::PointCloud<pcl::PointXYZI> cloud_in_ori;
        pcl::fromROSMsg(*input, cloud_in_ori);

        pcl::PointXYZI point; // 定义点
        // 接收点云,将点云的标签都设置为0，经过后续的计算，如果点云是地面点，将标签设置为1
        for (size_t i = 0; i < cloud_in.points.size(); i++)
        {
            point.x = cloud_in.points[i].x;
            point.y = cloud_in.points[i].y;
            point.z = cloud_in.points[i].z;
            point.intensity = cloud_in.points[i].intensity;
            // point.ring = cloud_in.points[i].ring;
            all_points_->points.push_back(point);
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices); // 移除无效点

        std::sort(cloud_in.points.begin(), cloud_in.end(), pointCompareZ); // 按Z轴进行排序

        pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_in.points.begin();

        for (int i = 0; i < cloud_in.points.size(); i++)
        {
            if (cloud_in.points[i].z < -1.5 * sensor_height_)
            {
                it++;
            }
            else
            {
                break;
            }
        }
        cloud_in.points.erase(cloud_in.points.begin(), it); // 低于1.5倍传感器高度的点云被滤除了

        // 提取地面种子点
        extractIntialSeeds(cloud_in);
        ground_points_ = seeds_points_;

        for (int i = 0; i < num_iter_; i++)
        {
            estimatePlane(); // 拟合平面
            ground_points_->clear();
            no_ground_points_->clear();
            MatrixXf points(cloud_in_ori.points.size(), 3); // 点云xyz矩阵
            int j = 0;
            for (auto p : cloud_in_ori.points)
            {
                points.row(j++) << p.x, p.y, p.z;
            }
            VectorXf result = points * normal_;     // 点和法向量的点积，即点到平面的距离
            for (int r = 0; r < result.rows(); r++) //
            {
                if (result[r] < th_dist_d_)
                {
                    // 小于阈值高度的设为地面点
                    ground_points_->points.push_back(cloud_in_ori[r]);
                }
                else
                {
                    // 大于阈值高度的设为非地面点
                    no_ground_points_->points.push_back(cloud_in_ori[r]);
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_no_ground(new pcl::PointCloud<pcl::PointXYZI>(*no_ground_points_));

        // 发布地面点的话题
        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_points_, ground_msg);
        ground_msg.header.stamp = input->header.stamp;
        ground_msg.header.frame_id = input->header.frame_id;
        pub_ground_points_.publish(ground_msg);

        // 发布非地面点的话题
        sensor_msgs::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*final_no_ground, no_ground_msg);
        no_ground_msg.header.stamp = input->header.stamp;
        no_ground_msg.header.frame_id = input->header.frame_id;
        pub_no_ground_points_.publish(no_ground_msg);
    }

    void PlaneGroundFilter::extractIntialSeeds(const pcl::PointCloud<pcl::PointXYZI> &sorted_points)
    {
        double sum = 0;
        int cnt = 0;
        for (int i = 0; i < sorted_points.points.size() && cnt < num_lpr_; i++)
        {
            sum += sorted_points.points[i].z;
            cnt++;
        }
        double lpr_height = cnt != 0 ? sum / cnt : 0; // 计算平均高度，同时避免分母为0
        seeds_points_->clear();

        for (int i = 0; i < sorted_points.points.size(); i++)
        {
            if (sorted_points.points[i].z < lpr_height + th_seeds_)
            {
                seeds_points_->points.push_back(sorted_points.points[i]); // 将低于平均高度的种子点都当作下一步平面点
            }
        }
    }

    void PlaneGroundFilter::estimatePlane()
    {
        Eigen::Matrix3f cov;                                                        // 协方差
        Eigen::Vector4f pointcloud_mean;                                            // 点云三维平均值
        pcl::computeMeanAndCovarianceMatrix(*ground_points_, cov, pointcloud_mean); // 计算协方差
        JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);    // SVD分解
        normal_ = (svd.matrixU().col(2));
        Eigen::Vector3f seeds_mean = pointcloud_mean.head<3>(); // 点云三维平均值的矩阵
        d_ = -(normal_.transpose() * seeds_mean)(0, 0);
        th_dist_d_ = th_dist_ - d_;
        float distance_to_plane = std::abs(d_); // 根据法线和平均点计算平面方程的常数项                                         // 选择SVD分解后的左奇异矩阵的第三列向量（方差最小列），作为法线的方向
    }

}