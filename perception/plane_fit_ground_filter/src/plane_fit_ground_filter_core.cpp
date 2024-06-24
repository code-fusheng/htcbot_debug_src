#include "plane_ground_filter_core.h"

PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    sub_point_cloud_ = nh.subscribe("/rslidar_points", 10, &PlaneGroundFilter::point_cb, this); // 订阅激光雷达的话题
    // init publisher
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("clip_height", clip_height_);
    ROS_INFO("clip_height: %f", clip_height_);
    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    nh.getParam("min_distance", min_distance_);
    ROS_INFO("min_distance: %f", min_distance_);
    nh.getParam("max_distance", max_distance_);
    ROS_INFO("max_distance: %f", max_distance_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_seeds", th_seeds_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);

    ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {}
bool point_cmp(VPoint a, VPoint b)
{ // 比较z轴高度用
    return a.z < b.z;
}
void PlaneGroundFilter::Spin()
{
}
// 去除过高的点云
void PlaneGroundFilter::clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                                   const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper; // 创建用于提取点的对象

    cliper.setInputCloud(in);                      // 设置将要处理的点云
    pcl::PointIndices indices;                     // 用于存储要移除点索引的结构体
#pragma omp for                                    // 加速并行处理
    for (size_t i = 0; i < in->points.size(); i++) // 检查每个点的z坐标是否高于clip_height_，如果是，则添加到索引列表中
    {
        if (in->points[i].z > clip_height_)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices)); // 将收集到的索引设置为ExtractIndices对象的索引
    cliper.setNegative(true);                                          // true是移除
    cliper.filter(*out);                                               // 执行过滤，根据设置的索引移除点云中的点，结果存储在输出点云out中
}

// 移除太近的点
void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                                            const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y); // 点到原点的距离

        if ((distance < min_distance_) || (distance > max_distance_))
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); // true 是移除
    cliper.filter(*out);
}

void PlaneGroundFilter::estimate_plane_(void)
{

    Eigen::Matrix3f cov;                                             // 协方差
    Eigen::Vector4f pc_mean;                                         // 点云三维平均值
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean); // 计算协方差

    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU); // SVD分解

    normal_ = (svd.matrixU().col(2)); // 选择SVD分解后的左奇异矩阵的第三列向量（方差最小列），作为法线的方向

    Eigen::Vector3f seeds_mean = pc_mean.head<3>(); // 点云三维平均值的矩阵

    d_ = -(normal_.transpose() * seeds_mean)(0, 0); // 根据法线和平均点计算平面方程的常数项

    th_dist_d_ = th_dist_ - d_;
    float distance_to_plane = std::abs(d_);
}

void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted) // 选择最初的种子点
{

    double sum = 0;
    int cnt = 0;
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // 计算平均高度，同时避免分母为0
    g_seeds_pc->clear();                          // 更新

    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]); // 将低于平均高度的种子点都当作下一步平面点
        }
    }
}

void PlaneGroundFilter::post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::PointCloud<VPoint>::Ptr cliped_pc_ptr(new pcl::PointCloud<VPoint>);
    clip_above(in, cliped_pc_ptr);
    pcl::PointCloud<VPoint>::Ptr remove_close(new pcl::PointCloud<VPoint>);
    remove_close_far_pt(cliped_pc_ptr, out);
}

void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);
    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn_org);

    SLRPointXYZIRL point; // 定义点
    // 接收点云,将点云的标签都设置为0，经过后续的计算，如果点云是地面点，将标签设置为1
    for (size_t i = 0; i < laserCloudIn.points.size(); i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u; // 0 means uncluster
        g_all_pc->points.push_back(point);
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices); // 移除无效点
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp); // 按Z轴进行排序

    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it); // 低于1.5倍传感器高度的点云被滤除了

    // 4. 提取地面种子点
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    // 5. 循环
    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_(); // 拟合平面
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        MatrixXf points(laserCloudIn_org.points.size(), 3); // 点云xyz矩阵
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }

        VectorXf result = points * normal_; // 点和法向量的点积，即点到平面的距离

        for (int r = 0; r < result.rows(); r++) //
        {
            if (result[r] < th_dist_d_)
            {
                g_all_pc->points[r].label = 1u; // 小于阈值高度的设为地面点
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                g_all_pc->points[r].label = 0u; // 大于阈值高度的设为非地面点
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }
    pcl::PointCloud<VPoint>::Ptr final_no_ground(new pcl::PointCloud<VPoint>);
    post_process(g_not_ground_pc, final_no_ground);

    // 发布地面点的话题
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_ground_.publish(ground_msg);

    // 发布非地面点的话题
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // 发布所有话题
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();
}