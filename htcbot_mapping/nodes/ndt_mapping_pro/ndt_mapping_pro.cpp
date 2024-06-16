/*
 * @Author: code-fusheng
 * @Date: 2024-04-14 12:45:52
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 15:12:58
 * @Description: 
 */
#include "ndt_mapping_pro.h"

using namespace NdtCommonNS;

namespace NdtMappingProNS
{
    
NdtMappingPro::NdtMappingPro()
{
    f_set_map_path_ = false;
    f_initial_scan_loaded_ = false;
    switch_to_exp_ = 0;
    is_pub_map_ = true;
}

NdtMappingPro::~NdtMappingPro() {}

void NdtMappingPro::init() {
    
    switch_to_exp_ = 0;
    nh_private.param<double>("/ndt_mapping_pro_node/voxel_leaf_size", voxel_leaf_size_, 1.0);
    int method_type_tmp = 1;
    nh_private.param<int>("/ndt_mapping_pro_node/method_type", method_type_tmp, 1);
    method_type_ = static_cast<NdtCommonNS::MethodType>(method_type_tmp);
    nh_private.param<int>("/ndt_mapping_pro_node/max_iter", max_iter_, 30);
    nh_private.param<double>("/ndt_mapping_pro_node/step_size", step_size_, 0.1);
    nh_private.param<float>("/ndt_mapping_pro_node/ndt_res", ndt_res_, 1.0);
    nh_private.param<double>("/ndt_mapping_pro_node/trans_eps", trans_eps_, 0.01);
    nh_private.param<bool>("/ndt_mapping_pro_node/is_filter_add_map", is_filter_add_map_, false);
    nh_private.param<double>("/ndt_mapping_pro_node/filter_res", filter_res_, 0.2);
    nh_private.param<double>("/ndt_mapping_pro_node/min_add_shift", min_add_shift_, 1.0);
    nh_private.param<bool>("/ndt_mapping_pro_node/is_limit_flat", is_limit_flat_, false);
    nh_private.param<double>("/ndt_mapping_pro_node/align_error_threshold", align_error_threshold_, 0.8);

    nh_private.param<bool>("/ndt_mapping_pro_node/is_pub_pgm", is_pub_pgm_, false);
    nh_private.param<bool>("/ndt_mapping_pro_node/is_filter_pass_through", is_filter_pass_through_, false);
    nh_private.param<double>("/ndt_mapping_pro_node/filter_high", filter_high_, 2.0);
    nh_private.param<double>("/ndt_mapping_pro_node/filter_low", filter_low_, -2.0);
    nh_private.param<bool>("/ndt_mapping_pro_node/is_filter_radius_outlier", is_filter_radius_outlier_, false);
    nh_private.param<double>("/ndt_mapping_pro_node/filter_radius", filter_radius_, 1.0);
    nh_private.param<int>("/ndt_mapping_pro_node/filter_thre_count", filter_thre_count_, 5);

    nh_private.param<double>("/ndt_mapping_pro_node/min_scan_range", min_scan_range_, 0.6);
    nh_private.param<double>("/ndt_mapping_pro_node/max_scan_range", max_scan_range_, 200.0);
    nh_private.param<double>("/ndt_mapping_pro_node/min_scan_height", min_scan_height_, 0.0);
    nh_private.param<double>("/ndt_mapping_pro_node/max_scan_height", max_scan_height_, 200.0);

    nh_private.param<bool>("/ndt_mapping_pro_node/use_imu", use_imu_, false);
    nh_private.param<bool>("/ndt_mapping_pro_node/use_odom", use_odom_, false);
    nh_private.param<bool>("/ndt_mapping_pro_node/use_gnss", use_gnss_, false);
    nh_private.param<bool>("/ndt_mapping_pro_node/use_vehicle", use_vehicle_, false);

    nh_private.param<std::string>("/ndt_mapping_pro_node/lidar_frame", lidar_frame_, "rslidar");
    nh_private.param<std::string>("/ndt_mapping_pro_node/pointcloud_topic", pointcloud_topic_, "rslidar_points");
    
    std::cout << "-*-*-*- NDT Mapping Pro Conf Strat -*-*-*-" << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size_ << std::endl;
    std::cout << "max_iter: " << max_iter_ << std::endl;
    std::cout << "step_size: " << step_size_ << std::endl;
    std::cout << "ndt_res: " << ndt_res_ << std::endl;
    std::cout << "trans_eps: " << trans_eps_ << std::endl;
    std::cout << "is_filter_add_map: " << is_filter_add_map_ << std::endl;
    std::cout << "filter_res: " << filter_res_ << std::endl;
    std::cout << "min_add_shift: " << min_add_shift_ << std::endl;
    std::cout << "min_scan_range: " << min_scan_range_ << std::endl;
    std::cout << "max_scan_range: " << max_scan_range_ << std::endl;
    std::cout << "min_scan_height: " << min_scan_height_ << std::endl;
    std::cout << "max_scan_height: " << max_scan_height_ << std::endl;
    std::cout << "use_imu: " << use_imu_ << std::endl;
    std::cout << "use_odom: " << use_odom_ << std::endl;
    std::cout << "use_gnss: " << use_gnss_ << std::endl;
    std::cout << "use_vehicle: " << use_vehicle_ << std::endl;
    std::cout << "-*-*-*- NDT Mapping Pro Conf End -*-*-*-" << std::endl;

    lidar_frame_ = "rslidar";
    pointcloud_topic_ = "rslidar_points";
    base_frame_ = "base_link";
    tf::StampedTransform lidar_base_tf;
    f_lidar_base_tf_ = findTansform4Source2Target(lidar_base_tf, lidar_frame_, base_frame_);
    ROS_WARN("[ndt_mapping_pro] : f_lidar_base_tf_: %s", f_lidar_base_tf_ ? "true" : "false");
    if (f_lidar_base_tf_) {
        tf::Vector3 origin = lidar_base_tf.getOrigin();
        double x = origin.getX();
        double y = origin.getY();
        double z = origin.getZ();
        // 获取旋转信息
        tf::Quaternion rotation = lidar_base_tf.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    } else {
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_x", tf_lidar_x_, 0.0);
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_y", tf_lidar_y_, 0.0);
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_z", tf_lidar_z_, 0.0);
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_roll", tf_lidar_roll_, 0.0);
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_pitch", tf_lidar_pitch_, 0.0);
        nh_private.param<double>("/ndt_mapping_pro_node/tf_lidar_yaw", tf_lidar_yaw_, 0.0);
    }
    std::cout << "(tf_lidar_x,tf_lidar_y,tf_lidar_z,tf_lidar_roll,tf_lidar_pitch,tf_lidar_yaw): (" 
            << tf_lidar_x_ << ", " << tf_lidar_y_ << ", " << tf_lidar_z_ << ", "
            << tf_lidar_roll_ << ", " << tf_lidar_pitch_ << ", " << tf_lidar_yaw_ << ")" << std::endl;

    // 初始化平移向量
    Eigen::Translation3f tl_btol(tf_lidar_x_, tf_lidar_y_, tf_lidar_z_);  
    // 初始化旋转向量，分别绕着 x、y、z 轴旋转
    Eigen::AngleAxisf rot_x_btol(tf_lidar_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_btol(tf_lidar_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(tf_lidar_yaw_, Eigen::Vector3f::UnitZ());

    tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    tf_ltob_ = tf_btol_.inverse();

    map_.header.frame_id = "map";
    history_trajectory_.header.frame_id = "map";

   // param 
    server_.setCallback(boost::bind(&NdtMappingPro::dynamicReconfigureCallback, this, _1, _2));

    sub_map_path_conf_ = nh.subscribe("/htcbot/map_path_conf", 10, &NdtMappingPro::callbackMapPathConf, this);
    // old temp
    sub_mapping_conf_ = nh.subscribe("/htcbot/mapping_conf", 10, &NdtMappingPro::callbackConfMapping, this);
    sub_ndt_mapping_conf_ = nh.subscribe("/htcbot/conf_ndt_mapping", 10, &NdtMappingPro::callbackConfNdtMapping, this);
    // sub_pointcloud_ = nh.subscribe(pointcloud_topic_, 100000, &NdtMappingPro::callbackPointCloud, this);

    // 地图消息发布
    pub_ndt_map_ = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    // 当前位姿消息发布
    pub_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
    // 发布用于记录轨迹的位姿
    pub_mix_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/mix/current_pose", 1000);

}

void NdtMappingPro::run() {
    init();
    initPose();
    while (ros::ok()) {
        ros::spin();
    }
}

void NdtMappingPro::initPose() {
    // previous: 前一帧点云车辆的位置
    previous_pose_.x = 0.0;
    previous_pose_.y = 0.0;
    previous_pose_.z = 0.0;
    previous_pose_.roll = 0.0;
    previous_pose_.pitch = 0.0;
    previous_pose_.yaw = 0.0;
    // ndt_pose_: NDT 配准算法得到的车辆位置
    ndt_pose_.x = 0.0;
    ndt_pose_.y = 0.0;
    ndt_pose_.z = 0.0;
    ndt_pose_.roll = 0.0;
    ndt_pose_.pitch = 0.0;
    ndt_pose_.yaw = 0.0;
    // current_pose: 当前帧点云车辆位置
    current_pose_.x = 0.0;
    current_pose_.y = 0.0;
    current_pose_.z = 0.0;
    current_pose_.roll = 0.0;
    current_pose_.pitch = 0.0;
    current_pose_.yaw = 0.0;
    // added: 用于计算地图更新的距离变化
    added_pose_.x = 0.0;
    added_pose_.y = 0.0;
    added_pose_.z = 0.0;
    added_pose_.roll = 0.0;
    added_pose_.pitch = 0.0;
    added_pose_.yaw = 0.0;
    // diff: 前后两次接收到传感器(IMU或者odom)消息时位姿的变化
    diff_ = 0;
    diff_x_ = 0.0;
    diff_y_ = 0.0;
    diff_z_ = 0.0;
    diff_yaw_ = 0.0;
    diff_roll_ = 0.0;
    diff_pitch_ = 0.0;

    current_velocity_x_ = 0.0;
    current_velocity_y_ = 0.0;
    current_velocity_z_ = 0.0;
}

void NdtMappingPro::dynamicReconfigureCallback(htcbot_mapping::ndt_mapping_proConfig &config, uint32_t level) {
    method_type_ = static_cast<MethodType>(config.method_type);
    is_debug_ = config.is_debug;
    is_pub_map_ = config.is_pub_map;
    queue_size_ = config.queue_size;
    voxel_leaf_size_ = config.voxel_leaf_size;
    max_iter_ = config.max_iter;
    step_size_ = config.step_size;
    trans_eps_ = config.trans_eps;
    ndt_res_ = config.ndt_res;
    is_filter_add_map_ = config.is_filter_add_map;
    filter_res_ = config.filter_res;
    min_scan_range_ = config.min_scan_range;
    max_scan_range_ = config.max_scan_range;
    min_scan_height_ = config.min_scan_height;
    max_scan_height_ = config.max_scan_height;
    min_add_shift_ = config.min_add_shift;
    is_limit_flat_ = config.is_limit_flat;
    is_sync_update_map_ = config.is_sync_update_map;
    is_remapping_ = config.is_remapping;
    align_error_threshold_ = config.align_error_threshold;

    is_pub_pgm_ = config.is_pub_pgm;
    is_filter_pass_through_ = config.is_filter_pass_through;
    filter_high_ = config.filter_high;
    filter_low_ = config.filter_low;
    is_filter_radius_outlier_ = config.is_filter_radius_outlier;
    filter_radius_ = config.filter_radius;
    filter_thre_count_ = config.filter_thre_count;

    std::cout << "UPDATE-----------------------------------------------------------------START" << std::endl;
    std::cout << "is_debug_: " << is_debug_ << std::endl;
    std::cout << "is_pub_map_: " << is_pub_map_ << std::endl;
    std::cout << "queue_size_: " << queue_size_ << std::endl;
    std::cout << "voxel_leaf_size_: " << voxel_leaf_size_ << std::endl;
    std::cout << "max_iter_: " << max_iter_ << std::endl;
    std::cout << "step_size_: " << step_size_ << std::endl;
    std::cout << "trans_eps_: " << trans_eps_ << std::endl;
    std::cout << "ndt_res_: " << ndt_res_ << std::endl;
    std::cout << "is_filter_add_map_: " << is_filter_add_map_ << std::endl;
    std::cout << "filter_res_: " << filter_res_ << std::endl;
    std::cout << "is_filter_pass_through_: " << is_filter_pass_through_ << std::endl;
    std::cout << "filter_high_: " << filter_high_ << std::endl;
    std::cout << "filter_low_: " << filter_low_ << std::endl;
    std::cout << "is_filter_radius_outlier_: " << is_filter_radius_outlier_ << std::endl;
    std::cout << "filter_radius_: " << filter_radius_ << std::endl;
    std::cout << "filter_thre_count_: " << filter_thre_count_ << std::endl;
    std::cout << "min_scan_range_: " << min_scan_range_ << std::endl;
    std::cout << "max_scan_range_: " << max_scan_range_ << std::endl;
    std::cout << "min_scan_height_: " << min_scan_height_ << std::endl;
    std::cout << "max_scan_height_: " << max_scan_height_ << std::endl;
    std::cout << "min_add_shift_: " << min_add_shift_ << std::endl;
    std::cout << "is_limit_flat_: " << is_limit_flat_ << std::endl;
    std::cout << "is_sync_update_map_: " << is_sync_update_map_ << std::endl;
    std::cout << "is_remapping_: " << is_remapping_ << std::endl;
    std::cout << "align_error_threshold_: " << align_error_threshold_ << std::endl;
    std::cout << "UPDATE-----------------------------------------------------------------END" << std::endl;
}

void NdtMappingPro::callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg) {
    ROS_INFO("[ndt_mapping_pro] ==> map_static_path: %s", msg->map_static_path.c_str());
    map_path_ = msg->map_static_path;
    f_set_map_path_ = true;
}

// Old
void NdtMappingPro::callbackConfMapping(const htcbot_msgs::MappingConf::ConstPtr& msg) {
    ROS_INFO("[ndt_mapping_pro] ==> callback mapping conf: %d", msg->mapping_state);
    map_path_ = msg->save_dir.c_str();
    voxel_leaf_size_ = msg->voxel_size;
    step_size_ = msg->step_size;
    min_add_shift_ = msg->add_shift;
    nh_private.setParam("/ndt_mapping_pro_node/switch_to_exp", msg->mapping_state);
    if (msg->mapping_state == 1) {
        switch_to_exp_ = 1;
        sub_pointcloud_ = nh.subscribe(pointcloud_topic_, queue_size_, &NdtMappingPro::callbackPointCloud, this);
    } else if (msg->mapping_state == 2) {
        switch_to_exp_ = 2;
        sub_pointcloud_.shutdown();
    } else if (msg->mapping_state == -1) {
         sub_pointcloud_.shutdown();
        switch_to_exp_ = -1;
        doFilterMap();
        doSaveMap2Pcd(map_path_);
    }
}

void NdtMappingPro::callbackConfNdtMapping(const htcbot_msgs::ConfNdtMapping::ConstPtr &msg) {
    // switch_to_exp_ = msg->switch_to_exp;
    method_type_ = static_cast<NdtCommonNS::MethodType>(msg->method_type);
    voxel_leaf_size_ = msg->voxel_leaf_size;
    step_size_ = msg->step_size;
    max_iter_ = msg->max_iter;
    ndt_res_ = msg->ndt_res;
    trans_eps_ = msg->trans_eps;
    is_limit_flat_ = msg->is_limit_flat;
    min_add_shift_ = msg->min_add_shift;
    is_filter_add_map_ = msg->is_filter_add_map;
    filter_res_ = msg->filter_res;
    is_sync_update_map_ = msg->is_sync_update_map;
    is_remapping_ = msg->is_remapping;
    min_scan_range_ = msg->min_scan_range;
    max_scan_range_ = msg->max_scan_range;
    min_scan_height_ = msg->min_scan_height;
    max_scan_height_ = msg->max_scan_height;
    use_odom_ = msg->use_odom;
    use_imu_ = msg->use_imu;
    use_gnss_ = msg->use_gnss;
    use_vo_ = msg->use_vo;

    nh_private.setParam("/ndt_mapping_pro_node/switch_to_exp", msg->switch_to_exp);
    nh_private.setParam("/ndt_mapping_pro_node/method_type", msg->method_type);
    nh_private.setParam("/ndt_mapping_pro_node/voxel_leaf_size", msg->voxel_leaf_size);
    nh_private.setParam("/ndt_mapping_pro_node/step_size", msg->step_size);
    nh_private.setParam("/ndt_mapping_pro_node/max_iter", msg->max_iter);
    nh_private.setParam("/ndt_mapping_pro_node/ndt_res", msg->ndt_res);
    nh_private.setParam("/ndt_mapping_pro_node/trans_eps", msg->trans_eps);
    nh_private.setParam("/ndt_mapping_pro_node/is_limit_flat", msg->is_limit_flat);
    nh_private.setParam("/ndt_mapping_pro_node/is_filter_add_map", msg->is_filter_add_map);
    nh_private.setParam("/ndt_mapping_pro_node/filter_res", msg->filter_res);
    nh_private.setParam("/ndt_mapping_pro_node/is_sync_update_map", msg->is_sync_update_map);
    nh_private.setParam("/ndt_mapping_pro_node/is_remapping", msg->is_remapping);
    nh_private.setParam("/ndt_mapping_pro_node/min_add_shift", msg->min_add_shift);
    nh_private.setParam("/ndt_mapping_pro_node/use_odom", msg->use_odom);
    nh_private.setParam("/ndt_mapping_pro_node/use_imu", msg->use_imu);
    nh_private.setParam("/ndt_mapping_pro_node/use_gnss", msg->use_gnss);
    nh_private.setParam("/ndt_mapping_pro_node/use_vo", msg->use_vo);
    nh_private.setParam("/ndt_mapping_pro_node/use_vehicle", msg->use_vehicle);
    nh_private.setParam("/ndt_mapping_pro_node/min_scan_range", msg->min_scan_range);
    nh_private.setParam("/ndt_mapping_pro_node/max_scan_range", msg->max_scan_range);
    nh_private.setParam("/ndt_mapping_pro_node/min_scan_height", msg->min_scan_height);
    nh_private.setParam("/ndt_mapping_pro_node/max_scan_height", msg->max_scan_height);
}

void NdtMappingPro::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input) {
    if (switch_to_exp_ != 1 || !f_set_map_path_) {
        ROS_WARN("[ndt_mapping_pro] ==> switch_to_exp_: %d f_set_map_path_: %s", switch_to_exp_, f_set_map_path_ ? "true" : "false");
        return;
    }
    delay_time_ = (ros::Time::now() - input->header.stamp).toSec();
    // r 表示激光点云到激光雷达的距离
    double r;
    pcl::PointXYZI p;
    // tmp 为原始点云转换的 PCL 点云数据
    // scan 为 tmp 过滤后的 PCL 点云数据
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_2_save_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q; 

    // 分别表示激光雷达与车体 相对于 map 的坐标系变换矩阵，并且初始化为 4 阶单位矩阵 
    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    tf::TransformBroadcaster br;
    tf::Transform transform;

    current_scan_time_ = input->header.stamp;

    // 将点云数据转换为 PCL 使用的数据类型
    pcl::fromROSMsg(*input, tmp);

    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        // 将 tmp 点云容器中的点进行逐一处理、去除不符合距离范围的点云数据
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;
        // 计算点雨激光雷达的欧式距离 r
        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        // 进行点云裁剪
        if (min_scan_height_ < p.z && p.z < max_scan_height_ && min_scan_range_ < r && r < max_scan_range_)
        {
            scan.push_back(p);
        }
    }

    // scan保存的是过滤的原始点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    // 点云降采样处理
    voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
                                  voxel_leaf_size_);
    voxel_grid_filter_.setInputCloud(scan_ptr);
    voxel_grid_filter_.filter(*filtered_scan_ptr);

    if (f_initial_scan_loaded_ == 0)
    { 
        // 将初始化点云加入至地图
        // 通过 tf_btol 变换矩阵 和 scan 点云数据 作为输入 将点云进行转化
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol_);
        // 将转换后的点云加入 map 进行拼接，实际上是作为第一帧点云图_像
        map_ += *transformed_scan_ptr;
        // 标记初始化载入状态 1: 成功
        f_initial_scan_loaded_ = 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));

    if (method_type_ == MethodType::PCL_GENERIC)
    {
        // 设置转换参数 Epsilon、最大步长、网格大小、最大迭代次数 以及设置输入数据为 已过滤点云 filtered_scan_ptr
        ndt_.setTransformationEpsilon(trans_eps_);
        ndt_.setStepSize(step_size_);
        ndt_.setResolution(ndt_res_);
        ndt_.setMaximumIterations(max_iter_);
        ndt_.setInputSource(filtered_scan_ptr);
    }
    // method_type == 1
    else if (method_type_ == MethodType::PCL_ANH)
    {
        anh_ndt_.setTransformationEpsilon(trans_eps_);
        anh_ndt_.setStepSize(step_size_);
        anh_ndt_.setResolution(ndt_res_);
        anh_ndt_.setMaximumIterations(max_iter_);
        anh_ndt_.setInputSource(filtered_scan_ptr);
    }
    bool is_first_map = true;
    if (is_first_map == true)
    {
        if (method_type_ == MethodType::PCL_GENERIC)
            ndt_.setInputTarget(map_ptr);
        else if (method_type_ == MethodType::PCL_ANH)
            anh_ndt_.setInputTarget(map_ptr);
        is_first_map = false;
    }

    guess_pose_.x = previous_pose_.x + diff_x_;
    guess_pose_.y = previous_pose_.y + diff_y_;
    guess_pose_.z = previous_pose_.z + diff_z_;
    guess_pose_.roll = previous_pose_.roll + diff_roll_;
    guess_pose_.pitch = previous_pose_.pitch + diff_pitch_;
    guess_pose_.yaw = previous_pose_.yaw + diff_yaw_;
    if (is_limit_flat_) {
        guess_pose_.roll = previous_pose_.roll;
        guess_pose_.pitch = previous_pose_.pitch;
    }

    // if (_use_imu == true) imu_calc(current_scan_time_);
    Pose guess_pose_for_ndt;
    guess_pose_for_ndt = guess_pose_;

    // 利用 guess_pose_for_ndt 位置的位姿旋转量 来初始化关于xyz轴的旋转向量
    Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
    // 利用 guess_pose_for_ndt 位置的三维坐标 来初始化平移向量
    Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

    Eigen::Matrix4f init_guess =
        (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 根据选择类型，进行 NDT 配准
    if (method_type_ == MethodType::PCL_GENERIC)
    {
        // 开始 NDT 配准，ndt.align 以 init_guess 为初值进行迭代优化 => 然后将配准结果保存在 output_cloud 点云中
        align_start_ = std::chrono::system_clock::now();
        ndt_.align(*output_cloud, init_guess);
        align_end_ = std::chrono::system_clock::now();
        // 计算目标点云与源点云之间的欧式距离平方和作为适应分数
        fitness_score_ = ndt_.getFitnessScore();
        // 得到最终的激光雷达相对于 map 坐标系的变换矩阵 t_localizer
        t_localizer = ndt_.getFinalTransformation();
        // 判断是否收敛
        has_converged_ = ndt_.hasConverged();
        // 得到最后的迭代次数
        final_num_iteration_ = ndt_.getFinalNumIteration();
        transformation_probability_ = ndt_.getTransformationProbability();
    }
    else if (method_type_ == MethodType::PCL_ANH)
    {
        align_start_ = std::chrono::system_clock::now();
        anh_ndt_.align(init_guess);
        align_end_ = std::chrono::system_clock::now();
        fitness_score_ = anh_ndt_.getFitnessScore();
        t_localizer = anh_ndt_.getFinalTransformation();
        has_converged_ = anh_ndt_.hasConverged();
        final_num_iteration_ = anh_ndt_.getFinalNumIteration();
    }

    // 计算匹配结果的姿态信息
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end_ - align_start_).count() / 1000.0;

    if (has_converged_) {
        ROS_INFO("[ndt_mapping_pro] ===> converged. matching score: %.3f, delay_time: %f ms", fitness_score_, delay_time_);
    } else {
        ROS_WARN("[ndt_mapping_pro] ===> not converged. matching score: %.3f, delay_time: %f ms", fitness_score_, delay_time_);
        // 可以根据需要处理未收敛的情况
    }

    if (is_limit_flat_) {
        t_localizer(2, 3) = tf_lidar_z_;
    }
    t_base_link = t_localizer * tf_ltob_;
    tf::Matrix3x3 mat_l, mat_b;

    mat_l.setValue(static_cast<double>(t_localizer(0, 0)),
                   static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)),
                   static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)),
                   static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)),
                   static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    mat_b.setValue(static_cast<double>(t_base_link(0, 0)),
                   static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)),
                   static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)),
                   static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)),
                   static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    localizer_pose_.x = t_localizer(0, 3);
    localizer_pose_.y = t_localizer(1, 3);
    localizer_pose_.z = t_localizer(2, 3);

    // 设置 localizer_pose 的旋转 rpy 角度
    mat_l.getRPY(localizer_pose_.roll, localizer_pose_.pitch, localizer_pose_.yaw, 1);

    // 更新 ndt_pose 获取 NDT 配准之后的位置
    ndt_pose_.x = t_base_link(0, 3);
    ndt_pose_.y = t_base_link(1, 3);
    ndt_pose_.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_pose_.roll, ndt_pose_.pitch, ndt_pose_.yaw, 1);

    // 将 NDT 配准之后的位置作为当前位置
    current_pose_.x = ndt_pose_.x;
    current_pose_.y = ndt_pose_.y;
    current_pose_.z = ndt_pose_.z;
    current_pose_.roll = ndt_pose_.roll;
    current_pose_.pitch = ndt_pose_.pitch;
    current_pose_.yaw = ndt_pose_.yaw;

    // 以当前位置作为坐标原点
    transform.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
    // 以当前位置旋转角度 rpy，设置旋转四元素 q
    q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    // 利用 q 来设置旋转
    transform.setRotation(q);
    // 发布坐标变换信息
    br.sendTransform(tf::StampedTransform(transform, current_scan_time_, "map", "base_link"));

    // 计算激光雷达扫描间隔时间
    scan_duration_ = current_scan_time_ - previous_scan_time_;
    double secs = scan_duration_.toSec();

    // 计算相邻帧位姿偏差
    diff_x_ = current_pose_.x - previous_pose_.x;
    diff_y_ = current_pose_.y - previous_pose_.y;
    diff_z_ = current_pose_.z - previous_pose_.z;
    diff_yaw_ = calcDiffForRadian(current_pose_.yaw, previous_pose_.yaw);
    diff_ = sqrt(diff_x_ * diff_x_ + diff_y_ * diff_y_ + diff_z_ * diff_z_);
    // 利用前后两帧扫描位置偏差与扫描时间间隔计算此时的瞬时速度
    current_velocity_x_ = diff_x_ / secs;
    current_velocity_y_ = diff_y_ / secs;
    current_velocity_z_ = diff_z_ / secs;

    // 最后将 current_pose 赋值前一帧位姿 previous_pos
    previous_pose_.x = current_pose_.x;
    previous_pose_.y = current_pose_.y;
    previous_pose_.z = current_pose_.z;
    previous_pose_.roll = current_pose_.roll;
    previous_pose_.pitch = current_pose_.pitch;
    previous_pose_.yaw = current_pose_.yaw;

    previous_scan_time_.sec = current_scan_time_.sec;
    previous_scan_time_.nsec = current_scan_time_.nsec;

    double shift = sqrt(pow(current_pose_.x - added_pose_.x, 2.0) + pow(current_pose_.y - added_pose_.y, 2.0));
    if (shift >= min_add_shift_)
    {   
        // 如果距离大于等于 min_add_scan_shift 则将经过坐标变换后得到的 *transformed_scan_ptr 加到 map 地图中完成拼接
        if (is_filter_add_map_ && fitness_score_ < align_error_threshold_) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_scan_2_save;
            voxel_grid_filter_scan_2_save.setLeafSize(filter_res_, filter_res_, filter_res_);
            voxel_grid_filter_scan_2_save.setInputCloud(scan_ptr);
            voxel_grid_filter_scan_2_save.filter(*filtered_scan_2_save_ptr);
            pcl::transformPointCloud(*filtered_scan_2_save_ptr, *transformed_scan_ptr, t_localizer);
        } else {
            pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
        }
        map_ += *transformed_scan_ptr;
        added_pose_.x = current_pose_.x;
        added_pose_.y = current_pose_.y;
        added_pose_.z = current_pose_.z;
        added_pose_.roll = current_pose_.roll;
        added_pose_.pitch = current_pose_.pitch;
        added_pose_.yaw = current_pose_.yaw;
    }

    if (is_pub_map_) {
        // 声明 ROS 可用的点云对象
        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
        pub_ndt_map_.publish(*map_msg_ptr);
    }

    geometry_msgs::PoseStamped current_pose_msg;
    q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time_;
    current_pose_msg.pose.position.x = current_pose_.x;
    current_pose_msg.pose.position.y = current_pose_.y;
    current_pose_msg.pose.position.z = current_pose_.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();
    pub_current_pose_.publish(current_pose_msg);
    
    // if (has_converged_) {
    //     pub_mix_current_pose_.publish(current_pose_msg);
    // }

    if (is_debug_) {
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "Sequence number: " << input->header.seq << std::endl;
        std::cout << "Number of tmp points: " << tmp.size() << " points." << std::endl;
        std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of filtered save scan points: " << filtered_scan_2_save_ptr->size() << " points." << std::endl;
        std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
        std::cout << "map: " << map_.points.size() << " points." << std::endl;
        std::cout << "NDT has converged: " << has_converged_ << std::endl;
        std::cout << "Fitness score: " << fitness_score_ << std::endl;
        std::cout << "Number of iteration: " << final_num_iteration_ << std::endl;
        std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
        std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
                    << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << t_localizer << std::endl;
        std::cout << "Align time: " << align_time_ << std::endl;
        std::cout << "shift: " << shift << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
    }

}

bool NdtMappingPro::findTansform4Source2Target(tf::StampedTransform& transform, const std::string source_frame, const std::string target_frame) {
    static ros::Time pre_find_time = ros::Time::now();
    try
    {
        transform_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        return true;
    } 
    catch(tf::TransformException ex)
    {
        ROS_ERROR("[ndt_mapping_pro] : find tf %s to %s failed: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

double NdtMappingPro::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

void NdtMappingPro::doFilterMap() {
    ROS_INFO("[ndt_mapping_pro] >> start to filter map");
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pass_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_radius_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    // PassThroughFilter
    if (is_filter_pass_through_) {
        pcl::PassThrough<pcl::PointXYZI> pass_through;
        pass_through.setInputCloud(map_ptr);
        pass_through.setFilterFieldName("z");
        pass_through.setFilterLimits(filter_low_, filter_high_);
        pass_through.setFilterLimitsNegative(false);
        pass_through.filter(*map_pass_filtered);
    } else {
        map_pass_filtered = map_ptr;
    }
    // RadiusOutlierFilter
    if (is_filter_radius_outlier_) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier;
        radius_outlier.setInputCloud(map_pass_filtered);
        radius_outlier.setRadiusSearch(filter_radius_);
        radius_outlier.setMinNeighborsInRadius(filter_thre_count_);
        radius_outlier.filter(*map_radius_filtered);
    } else {
        map_radius_filtered = map_pass_filtered;
    }
    // 
    if (is_filter_add_map_) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_save;
        voxel_grid_filter_save.setLeafSize(filter_res_, filter_res_, filter_res_);
        voxel_grid_filter_save.setInputCloud(map_radius_filtered);
        voxel_grid_filter_save.filter(*map_voxel_filtered);
    } else {
        map_voxel_filtered = map_radius_filtered;
    }
    // final
    map_filtered_ = *map_voxel_filtered;
}

void NdtMappingPro::doSaveMap2Pcd(const std::string filename) {
    ros::Time start = ros::Time::now();
    ROS_INFO("[ndt_mapping_pro] >> start to save map");
    std::cout << "filename: " << filename << std::endl;
    if (map_.size() < 10) {
        ROS_ERROR("[ndt_mapping_pro] No enough points in map, abort save map. size: %zu", map_.size());
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_filtered_));
    map_ptr->header.frame_id = "map";
    map_filtered_ptr->header.frame_id = "map";
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_filtered_ptr, *map_msg_ptr);
    pub_ndt_map_.publish(*map_msg_ptr);
    try {
        std::string orign_filename = filename + "/origin.pcd";
        pcl::io::savePCDFileBinary(orign_filename, *map_ptr);
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
    }
    ros::Time end = ros::Time::now();
    std::cout << "Saved " << map_ptr->points.size() << " data points to "
        << filename << ". -> total used time: " << end - start << "s"
        << std::endl;
    try {
        std::string static_filename = filename + "/static.pcd";
        pcl::io::savePCDFileBinary(static_filename, *map_filtered_ptr);
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
    }
    end = ros::Time::now();
    std::cout << "Saved " << map_filtered_ptr->points.size()
        << " data points to " << filename << "." << std::endl;
}



}