/*
 * @Author: code-fusheng
 * @Date: 2024-04-17 18:10:49
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-22 16:01:04
 * @Description: 
 */
#include "ndt_localizer_pro.h"
#include <iostream>



static sensor_msgs::Imu imu;


static    ros::Publisher  pub_imu_current_posess_;


static    ros::Publisher  pub_ndt_last_poses_linear;

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};




static pose previous_pose, guess_pose, guess_pose_odom, guess_pose_imu_odom, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;





 double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;


static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

using namespace NdtCommonNS;
using namespace HtcbotCommonNS;

namespace NdtLocalizerProNS
{
    
NdtLocalizerPro::NdtLocalizerPro() : nh_private_("~"), server_(nh_private_)


{


       velocity_.x = velocity_.y = velocity_.z = 0.0;
        position_.x = position_.y = position_.z = 0.0;
        last_time_ = ros::Time::now();


    localizer_truth_ = 1;
    switch_status_ = 0;
    is_debug_ = 0;
    enable_auto_init_ = 0;
    mode_switch_ = 1;
    points_map_num_ = 0;
    f_map_loaded_ = 0;
    f_init_pos_set_ = 0;
    f_use_predict_pose_ = 0;
    enable_auto_fix_ = 1;
    use_gnss_fix_ = 0;
    use_vehicle_fix_ = 0;
    offset_count_threshold_ = 1;
    align_error_threshold_ = 1;
    distance_pose_to_fix_threshold_ = 5.0;
    distance_gps_to_gps_threshold_ = 1.0;

    localizer_status_.module_type = static_cast<int>(HtcbotCommonNS::MODULE_TYPE::LOCALIZER);
    localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::NONE);

}

NdtLocalizerPro::~NdtLocalizerPro() {
    delete map_spinner_;
}

void NdtLocalizerPro::init() {

    pthread_mutex_init(&mutex_, NULL);

    prev_vehicle_steer_ = 0.0;
    prev_vehicle_speed_ = 0.0;
    current_vehicle_steer_ = 0.0;
    current_vehicle_speed_ = 0.0;

    current_velocity_x_ = 0.0;
    current_velocity_y_ = 0.0;
    current_velocity_z_ = 0.0;
    current_velocity_yaw_ = 0.0;
    current_velocity_smooth_ = 0.0;
    angular_velocity_ = 0.0;

    previous_velocity_ = 0.0;
    previous_velocity_x_ = 0.0;
    previous_velocity_y_ = 0.0;
    previous_velocity_z_ = 0.0;
    previous_velocity_yaw_ = 0.0;

    previous_previous_velocity_ = 0.0;

    current_accel_ = 0.0;
    previous_accel_ = 0.0;  // [m/s^2]
    current_accel_x_ = 0.0;
    current_accel_y_ = 0.0;
    current_accel_z_ = 0.0;
    current_accel_yaw_ = 0.0;

    init_pose_.x = 0.0;
    init_pose_.y = 0.0;
    init_pose_.z = 0.0;
    init_pose_.roll = 0.0;
    init_pose_.pitch = 0.0;
    init_pose_.yaw = 0.0;

    diff_ = 0.0;
    diff_x_ = 0.0;
    diff_y_ = 0.0;
    diff_z_ = 0.0;
    diff_yaw_ = 0.0;

    nh_private_.param<bool>("/ndt_localizer_pro_node/is_debug", is_debug_, 0);
    nh_private_.param<int>("/ndt_localizer_pro_node/switch_status", switch_status_, 0);
    nh_private_.param<bool>("/ndt_localizer_pro_node/enable_auto_init", enable_auto_init_, 0);

    int method_type_tmp = 1;
    nh_private_.param<int>("/ndt_localizer_pro_node/method_type", method_type_tmp, 1);
    method_type_ = static_cast<MethodType>(method_type_tmp);

    nh_private_.param<int>("/ndt_localizer_pro_node/queue_size", queue_size_, 1);
    nh_private_.param<std::string>("/ndt_localizer_pro_node/lidar_frame", lidar_frame_, "rslidar");
    nh_private_.param<std::string>("/ndt_localizer_pro_node/map_topic", map_topic_, "points_map");
    nh_private_.param<std::string>("/ndt_localizer_pro_node/pointcloud_topic", pointcloud_topic_, "rslidar_points");
    nh_private_.param<double>("/ndt_localizer_pro_node/predict_pose_error_threshold", predict_pose_error_threshold_, 0.5);
    nh_private_.param<std::string>("/ndt_localizer_pro_node/offset", offset_, "linear");
    nh_private_.param<double>("/ndt_localizer_pro_node/delay_time_error_threshold", delay_time_error_threshold_, 0.5);
    nh_private_.param<bool>("/ndt_localizer_pro_node/enable_auto_fix", enable_auto_fix_, 1);
    nh_private_.param<bool>("/ndt_localizer_pro_node/use_gnss_fix", use_gnss_fix_, 1);
    nh_private_.param<double>("/ndt_localizer_pro_node/distance_pose_to_fix_threshold", distance_pose_to_fix_threshold_, 5.0);
    
    nh_private_.param<int>("/ndt_localizer_pro_node/max_iter", max_iter_, 30);
    nh_private_.param<double>("/ndt_localizer_pro_node/step_size", step_size_, 0.1);
    nh_private_.param<float>("/ndt_localizer_pro_node/ndt_res", ndt_res_, 1.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/trans_eps", trans_eps_, 0.01);

    nh_private_.param<double>("/ndt_localizer_pro_node/voxel_leaf_size", voxel_leaf_size_, 0.2);
    nh_private_.param<double>("/ndt_localizer_pro_node/map_voxel_leaf_size_", map_voxel_leaf_size_, 0.2);

    voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    map_voxel_grid_filter_.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);

    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_x", tf_lidar_x_, 0.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_y", tf_lidar_y_, 0.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_z", tf_lidar_z_, 0.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_roll", tf_lidar_roll_, 0.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_pitch", tf_lidar_pitch_, 0.0);
    nh_private_.param<double>("/ndt_localizer_pro_node/tf_lidar_yaw", tf_lidar_yaw_, 0.0);

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

    map_spinner_ = new ros::AsyncSpinner(2, &map_callback_queue_);

    ros::SubscribeOptions map_ops = ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(map_topic_, 
        10, 
        boost::bind(&NdtLocalizerPro::callbackMap, this, _1), 
        ros::VoidPtr(), 
        &map_callback_queue_);
    sub_map_ = nh.subscribe(map_ops);

    // param 
    server_.setCallback(boost::bind(&NdtLocalizerPro::dynamicReconfigureCallback, this, _1, _2));

    // sub
    sub_pointcloud_ = nh.subscribe(pointcloud_topic_, queue_size_, &NdtLocalizerPro::callbackPoints, this);

    // sub_pointcloud_ = nh.subscribe("/rslidar_points", queue_size_, &NdtLocalizerPro::callbackPoints, this);

    //  sub_initialpose_ = nh.subscribe("initialpose", 10, &NdtLocalizerPro::callbackInitialPose, this);

    sub_initialpose_ = nh.subscribe("initialpose", 10, &NdtLocalizerPro::callbackInitialPose, this);
    // sub_map_ = nh.subscribe(map_topic_, 10, &NdtLocalizerPro::callbackMap, this);
    // 车辆状态信息
    sub_vehicle_status_ = nh.subscribe("/vehicle_status", 10, &NdtLocalizerPro::callbackVehicleStatus, this);
    sub_gnss_current_pose_fix_ = nh.subscribe("/gnss/current_pose/fix", 1, &NdtLocalizerPro::callbackGnssCurrentPoseFix, this);


    //    新增 处理  IMU  数据  并且 预测车辆初始位置姿态函数imu_callback


     sub_imu_data_ = nh.subscribe("/imu_raw", 1, &NdtLocalizerPro::imu_callback, this);


    // pub
    pub_system_status_ = nh.advertise<htcbot_msgs::StatusHtcbotModule>("/htcbot/module_status", 10);

    pub_ndt_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
    pub_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    pub_current_pose_truth_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose_truth", 10);
    pub_predict_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 10);

    pub_time_ndt_localizer_ = nh.advertise<std_msgs::Float32>("/time_ndt_localizer", 10);
    pub_estimated_vel_mps_ = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 10);
    pub_estimated_vel_kmps_ = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 10);

//======================================================================================================

//    新增 发布  IMU
     pub_imu_current_posess_ = nh.advertise<geometry_msgs::PoseStamped>("/imu_current_pose", 10);

     pub_ndt_last_poses_linear= nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_linear", 10);

//======================================================================================================


    srv_switch_status_ = nh_private_.advertiseService("set_switch_status", &NdtLocalizerPro::setSwitchStatusCallback, this);

    map_spinner_->start();

}

void NdtLocalizerPro::run() {
    init();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());
        loop_rate.sleep(); // 控制循环的频率
    }
}

void NdtLocalizerPro::dynamicReconfigureCallback(htcbot_localizer::ndt_localizer_proConfig &config, uint32_t level) {
    method_type_ = static_cast<MethodType>(config.method_type);
    is_debug_ = config.is_debug;
    queue_size_ = config.queue_size;
    voxel_leaf_size_ = config.voxel_leaf_size;
    map_voxel_leaf_size_ = config.map_voxel_leaf_size;
    max_iter_ = config.max_iter;
    step_size_ = config.step_size;
    trans_eps_ = config.trans_eps;
    offset_ = config.offset;
    predict_pose_error_threshold_ = config.predict_pose_error_threshold;
    delay_time_error_threshold_ = config.delay_time_error_threshold;
    enable_auto_fix_ = config.enable_auto_fix;
    use_gnss_fix_ = config.use_gnss_fix;
    use_vehicle_fix_ = config.use_vehicle_fix;
    offset_count_threshold_ = config.offset_count_threshold;
    align_error_threshold_ = config.align_error_threshold;
    distance_pose_to_fix_threshold_ = config.distance_pose_to_fix_threshold;
    distance_gps_to_gps_threshold_ = config.distance_gps_to_gps_threshold;
    std::cout << "UPDATE-----------------------------------------------------------------START" << std::endl;
    std::cout << "is_debug: " << is_debug_ << std::endl;
    std::cout << "method_type: " << config.method_type << std::endl;
    std::cout << "queue_size: " << queue_size_ << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size_ << std::endl;
    std::cout << "max_iter: " << max_iter_ << std::endl;
    std::cout << "step_size: " << step_size_ << std::endl;
    std::cout << "trans_eps: " << trans_eps_ << std::endl;
    std::cout << "offset: " << offset_ << std::endl;
    std::cout << "predict_pose_error_threshold: " << predict_pose_error_threshold_ << std::endl;
    std::cout << "delay_time_error_threshold: " << delay_time_error_threshold_ << std::endl;
    std::cout << "offset_count_threshold: " << offset_count_threshold_ << std::endl;
    std::cout << "enable_auto_fix: " << enable_auto_fix_ << std::endl;
    std::cout << "use_gnss_fix: " << use_gnss_fix_ << std::endl;
    std::cout << "use_vehicle_fix: " << use_vehicle_fix_ << std::endl;
    std::cout << "UPDATE-----------------------------------------------------------------END" << std::endl;
}

bool NdtLocalizerPro::setSwitchStatusCallback(htcbot_msgs::SwitchStatusSrv::Request &req, htcbot_msgs::SwitchStatusSrv::Response &res) {
    ROS_INFO("[ndt_localizer_pro] ===> req: %d", req.switch_to); 
    switch_status_ = req.switch_to;
    res.switch_status = switch_status_; 
    return true;                      
}

void NdtLocalizerPro::callbackMap(const sensor_msgs::PointCloud2::ConstPtr& input) {



    ROS_INFO("[ndt_localizer_pro] ==> update points_map, input points num = %d", input->width);
    if (points_map_num_ != input->width) {
        points_map_num_ = input->width;
        pcl::fromROSMsg(*input, map_);
        // TODO map => world 的 tf
        // ...



        pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_));
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (map_voxel_leaf_size_ > 0) {
            map_voxel_grid_filter_.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
            map_voxel_grid_filter_.setInputCloud(map_ptr);
            map_voxel_grid_filter_.filter(*filtered_map_ptr);
            ROS_ERROR("[ndt_localizer_pro] ===> filtered map size: %d", filtered_map_ptr->width);
            map_ptr = filtered_map_ptr;
        }
        // 准备 PCL 配准
        if (method_type_ == MethodType::PCL_GENERIC) {
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            new_ndt.setResolution(ndt_res_);
            new_ndt.setInputTarget(map_ptr);
            new_ndt.setMaximumIterations(max_iter_);
            new_ndt.setStepSize(step_size_);
            new_ndt.setTransformationEpsilon(trans_eps_);
            // Eigen::Matrix4f::Identity() 表示初始时不对输入点云进行任何变换
            new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());
            pthread_mutex_lock(&mutex_);
            ndt_ = new_ndt;
            pthread_mutex_unlock(&mutex_);
        } else if (method_type_ == MethodType::PCL_ANH) {
            cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
            new_anh_ndt.setResolution(ndt_res_);
            new_anh_ndt.setInputTarget(map_ptr);
            new_anh_ndt.setMaximumIterations(max_iter_);
            new_anh_ndt.setStepSize(step_size_);
            new_anh_ndt.setTransformationEpsilon(trans_eps_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointXYZ dummy_point;  // 虚拟点
            dummy_scan_ptr->push_back(dummy_point);
            new_anh_ndt.setInputSource(dummy_scan_ptr);
            new_anh_ndt.align(Eigen::Matrix4f::Identity());
            pthread_mutex_lock(&mutex_);
            anh_ndt_ = new_anh_ndt;
            pthread_mutex_unlock(&mutex_);
        }
        // #ifdef CUDA_FOUND
        f_map_loaded_ = 1;
    }   
}





void NdtLocalizerPro::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input) {



    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("world", input->header.frame_id, ros::Time(0), transform);
    } 
    catch(tf::TransformException ex)
    {
        ROS_ERROR("[ndt_mapping_pro] : find tf %s to %s failed: %s", "world", input->header.frame_id.c_str(), ex.what());
    }
    tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    current_pose_.x = input->pose.pose.position.x;
    current_pose_.y = input->pose.pose.position.y;
    current_pose_.z = input->pose.pose.position.z;
    m.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);

    previous_pose_.x = current_pose_.x;
    previous_pose_.y = current_pose_.y;
    previous_pose_.z = current_pose_.z;
    previous_pose_.roll = current_pose_.roll;
    previous_pose_.pitch = current_pose_.pitch;
    previous_pose_.yaw = current_pose_.yaw;

    current_velocity_x_ = 0.0;
    current_velocity_y_ = 0.0;
    current_velocity_z_ = 0.0;
    angular_velocity_ = 0.0;

    current_accel_ = 0.0;
    current_accel_x_ = 0.0;
    current_accel_y_ = 0.0;
    current_accel_z_ = 0.0;

    offset_x_ = 0.0;
    offset_y_ = 0.0;
    offset_z_ = 0.0;
    offset_yaw_ = 0.0;


    sum_poses_x =0.0;
    sum_poses_y =0.0;
    sum_poses_z =0.0;


    f_init_pos_set_ = 1;
     


}










void NdtLocalizerPro::callbackGnssCurrentPoseFix(const geometry_msgs::PoseStamped::ConstPtr& input) {



    tf::Quaternion q(
    input->pose.orientation.x, input->pose.orientation.y,
    input->pose.orientation.z, input->pose.orientation.w);
    tf::Matrix3x3 m(q);
    // 强制Z轴0
    fix_pose_gnss_.x = input->pose.position.x;
    fix_pose_gnss_.y = input->pose.position.y;
    fix_pose_gnss_.z = input->pose.position.z;
    m.getRPY(fix_pose_gnss_.roll, fix_pose_gnss_.pitch, fix_pose_gnss_.yaw);
}






static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}


static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}




void NdtLocalizerPro::imu_calc(ros::Time current_time,const sensor_msgs::Imu::ConstPtr &input)

// static void imu_calc(ros::Time current_time,const sensor_msgs::Imu::ConstPtr &input)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();







  double diff_imu_roll = (imu.angular_velocity.x * diff_time)*-1;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;


  offset_imu_x  =  0.0;
  offset_imu_y   = 0.0;
  offset_imu_z   =  0.0;

  
   offset_imu_roll =  0.0;
  offset_imu_pitch  = 0.0;
   offset_imu_yaw  =  0.0;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;



   current_velocity_imu_x =  0.0;
    current_velocity_imu_y  = 0.0;
  current_velocity_imu_z  =  0.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x =  previous_pose_.x + offset_imu_x;
  guess_pose_imu.y =previous_pose_.y+ offset_imu_y;
  guess_pose_imu.z = previous_pose_.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose_.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose_.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose_.yaw  + offset_imu_yaw;






  // std::cout << "---------------------------- current_velocity_imu_x------ current_velocity_imu_y---- current_velocity_imu_z----- ------" 
  // << " " <<  current_velocity_imu_x << " "  << current_velocity_imu_y << " " << current_velocity_imu_z<< std::endl;




     geometry_msgs::PoseStamped current_pose_msg;
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_time;
    current_pose_msg.pose.position.x = guess_pose_imu.x;
    current_pose_msg.pose.position.y = guess_pose_imu.y;
    current_pose_msg.pose.position.z = guess_pose_imu.z;

    double cy = cos(guess_pose_imu.yaw * 0.5);  
   double sy = sin(guess_pose_imu.yaw* 0.5);  
   double cp = cos( guess_pose_imu.pitch* 0.5);  
   double sp = sin( guess_pose_imu.pitch * 0.5);  
    double cr = cos( guess_pose_imu.roll  * 0.5);  
    double sr = sin( guess_pose_imu.roll  * 0.5);  
  
// 使用ZYX旋转顺序（Tait-Bryan angles的ZYX顺序）  
// 这里的四元数按q = w + xi + yj + zk的形式组织，其中w是实部，(x, y, z)是虚部  
// 在编程中，通常将四元数表示为(w, x, y, z)的向量或数组  

  tf::Quaternion q;  

q.setW(cy * cp * cr + sy * sp * sr);  

q.setX(cy * cp * sr - sy * sp * cr);  

q.setY(sy * cp * sr + cy * sp * cr);  

q.setZ(sy * cp * cr - cy * sp * sr);



    current_pose_msg.pose.orientation.x =q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();
    pub_imu_current_posess_.publish(current_pose_msg);

  previous_time = current_time;
}





void NdtLocalizerPro::imu_callback(const sensor_msgs::Imu::ConstPtr &input)


{


//   if (_imu_upside_down)
//     imuUpsideDown(input);



    current_time = input->header.stamp;
    static ros::Time previous_time = current_time;
     previous_times   = previous_time  ;


//    std::cout <<"========current_time ====previous_time==diff_time==="<<current_time << "  " <<previous_time << "  "<<  previous_times <<std::endl;

  // 计算前后两次接收到消息的微小时间差

  diff_time=(current_time - previous_time).toSec();




  double imu_roll, imu_pitch, imu_yaw;
  // 声明用于表示旋转的四元数
  tf::Quaternion imu_orientation;
  // 将 imu 采集的旋转四元数消息转化为 TF 类型的旋转四元数存入 imu_orientation
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  // 利用 imu_orientation 旋转变量 初始化一个 3*3 的旋转矩阵 然后通过 imu_roll, imu_pitch, imu_yaw 获取 imu 此时的旋转角度
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
  // 将角度转化为弧度
  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);




  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;





  // 将角度的变化转换为弧度
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);


  imu.header = input->header;
  // 获取 imu x 方向上的线性加速度
  imu.linear_acceleration.x = (input->linear_acceleration.x)*-1;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0)
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

//   // 利用 imu 计算位置初值 为 NDT 配准提供初始位置
  imu_calc(input->header.stamp,input);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

const char* methodTypeToString(MethodType methodType) {
    switch (methodType) {
        case MethodType::PCL_GENERIC: return "PCL_GENERIC";
        case MethodType::PCL_ANH: return "PCL_ANH";
        case MethodType::PCL_ANH_GPU: return "PCL_ANH_GPU";
        case MethodType::PCL_OPENMP: return "PCL_OPENMP";
        default: return "UNKNOWN_METHOD_TYPE";
    }
}





void NdtLocalizerPro::callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr &msg) 
{
    prev_vehicle_steer_ = current_vehicle_steer_;
    prev_vehicle_speed_ = current_vehicle_speed_;
    current_vehicle_steer_ = msg->cur_steer;
    current_vehicle_speed_ = msg->cur_speed;
}

void NdtLocalizerPro::callbackPoints(const sensor_msgs::PointCloud2::ConstPtr& input) {

 
    localizer_truth_ = 1;
    if (switch_status_ != 1 || f_map_loaded_ != 1 || f_init_pos_set_ != 1) {



       
        ROS_WARN("[ndt_localizer_pro] ==> switch_status_ = %d, f_map_loaded_ = %d, f_init_pos_set_ = %d", switch_status_, f_map_loaded_, f_init_pos_set_);
        return;
    }


    delay_time_ = (ros::Time::now() - input->header.stamp).toSec();
    if (delay_time_ >= delay_time_error_threshold_) {
        // TODO 刹车
        return;
    }
    // 默认是WARN 不可用
    localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::DANGER);
    // 记录匹配开始时间
    matching_start_ = std::chrono::system_clock::now();
    // 定义四元数变量，用于表示预测姿态
    tf::Quaternion predict_q, ndt_q, current_q, localizer_q;
    pcl::PointXYZ p;
    // 存储滤波后的点云数据
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    // 获取当前点云数据的时间戳 
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    // 从 ROS 消息中转换为 PCL 点云数据
    pcl::fromROSMsg(*input, *scan);
    voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter_.setInputCloud(scan);
    voxel_grid_filter_.filter(filtered_scan);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size();

    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());   // base_link
    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());  // localizer

    pthread_mutex_lock(&mutex_);



    // 根据不同的配准方法设置输入源
    if (method_type_ == MethodType::PCL_GENERIC)
      ndt_.setInputSource(filtered_scan_ptr);
    else if (method_type_ == MethodType::PCL_ANH)
      anh_ndt_.setInputSource(filtered_scan_ptr);




    
    // 计算预测姿态
    if (offset_ == "linear")
    
    {
            //    static ros::Time previous_time = current_time;
        
        


      offset_x_ = current_velocity_x_ * diff_time_;
      offset_y_ = current_velocity_y_ * diff_time_;
      offset_z_ = current_velocity_z_ * diff_time_;
      offset_yaw_ = angular_velocity_ * diff_time_;


    
    }
    // else if (offset_ == "quadratic")
    else if (offset_ == "zero")
    {

      offset_x_ = 0.0;
      offset_y_ = 0.0;
      offset_z_ = 0.0;
      offset_yaw_ = 0.0;
    }


//=======================加上这段代码后计算的   predict_pose_ 只考虑ndt  提供的位置姿态============================
      //  offset_x_ = 0.0;
      //  offset_y_ = 0.0;
      // offset_z_ = 0.0;
      // offset_yaw_ = 0.0;

//==========================================================================================================





    // 预测位姿 = 上次位姿加 + 位移
    predict_pose_.x = previous_pose_.x + offset_x_;
    predict_pose_.y = previous_pose_.y + offset_y_;
    predict_pose_.z = previous_pose_.z + offset_z_;
    predict_pose_.roll = previous_pose_.roll;
    predict_pose_.pitch = previous_pose_.pitch;
    predict_pose_.yaw = previous_pose_.yaw + offset_yaw_;



    geometry_msgs::PoseStamped  predict_pose_for_ndt_linear;



    predict_pose_for_ndt_linear.header.frame_id = "map";
    // current_pose_msg.header.stamp = current_time;
     predict_pose_for_ndt_linear.pose.position.x = predict_pose_.x ;
   predict_pose_for_ndt_linear.pose.position.y = predict_pose_.y;
  predict_pose_for_ndt_linear.pose.position.z = predict_pose_.z;

    double cy = cos(predict_pose_.yaw * 0.5);  
   double sy = sin(predict_pose_.yaw* 0.5);  
   double cp = cos( predict_pose_.pitch* 0.5);  
   double sp = sin( predict_pose_.pitch * 0.5);  
    double cr = cos( predict_pose_.roll  * 0.5);  
    double sr = sin( predict_pose_.roll  * 0.5);  

  tf::Quaternion q;  

q.setW(cy * cp * cr + sy * sp * sr);  

q.setX(cy * cp * sr - sy * sp * cr);  

q.setY(sy * cp * sr + cy * sp * cr);  

q.setZ(sy * cp * cr - cy * sp * sr);



    predict_pose_for_ndt_linear.pose.orientation.x =q.x();
    predict_pose_for_ndt_linear.pose.orientation.y = q.y();
    predict_pose_for_ndt_linear.pose.orientation.z = q.z();
    predict_pose_for_ndt_linear.pose.orientation.w = q.w();
    pub_ndt_last_poses_linear.publish( predict_pose_for_ndt_linear);




  




    
    Pose predict_pose_for_ndt;
    // TODO use_imu use_odom use_vo use_vehicle







    predict_pose_imu_.x   =   guess_pose_imu.x;
    predict_pose_imu_.y  =    guess_pose_imu.y;
    predict_pose_imu_.z   =    guess_pose_imu.z ;
    predict_pose_imu_.roll =   guess_pose_imu.roll;
    predict_pose_imu_.pitch  =  guess_pose_imu.pitch;
    predict_pose_imu_.yaw =     guess_pose_imu.yaw;




//==================================== predict_pose_：前后两帧估测的初始位姿========predict_pose_imu_ ： imu估测的初始位姿态=============================
    predict_pose_for_ndt = predict_pose_;
    //  predict_pose_for_ndt = predict_pose_imu_;



    
    Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);


   




 
    // 执行点云配准
    if (method_type_ == MethodType::PCL_GENERIC)
    {


      align_start_ = std::chrono::system_clock::now();
      ndt_.align(*output_cloud, init_guess);
      align_end_ = std::chrono::system_clock::now();
      has_converged_ = ndt_.hasConverged();
      t_base_link = ndt_.getFinalTransformation();
      iteration_ = ndt_.getFinalNumIteration();
      get_fitness_score_start_ = std::chrono::system_clock::now();
      fitness_score_ = ndt_.getFitnessScore();
      get_fitness_score_end_ = std::chrono::system_clock::now();
      trans_probability_ = ndt_.getTransformationProbability();
    }
    else if (method_type_ == MethodType::PCL_ANH)
    {


      align_start_ = std::chrono::system_clock::now();
      anh_ndt_.align(init_guess);
      align_end_ = std::chrono::system_clock::now();
      has_converged_ = anh_ndt_.hasConverged();
      t_base_link = anh_ndt_.getFinalTransformation();
      iteration_ = anh_ndt_.getFinalNumIteration();
      get_fitness_score_start_ = std::chrono::system_clock::now();
      fitness_score_ = anh_ndt_.getFitnessScore();
      get_fitness_score_end_ = std::chrono::system_clock::now();
      trans_probability_ = anh_ndt_.getTransformationProbability();
    }




  
        
    // 计算匹配结果的姿态信息
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end_ - align_start_).count() / 1000.0;

    pthread_mutex_unlock(&mutex_);

    t_localizer = t_base_link * tf_btol_.inverse();
    
    get_fitness_score_time_ = std::chrono::duration_cast<std::chrono::microseconds>(get_fitness_score_end_ - get_fitness_score_start_).count() / 1000.0;

    tf::Matrix3x3 mat_l, mat_b;

    mat_l.setValue(static_cast<double>(t_base_link(0, 0)),
                   static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)),
                   static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)),
                   static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)),
                   static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    mat_b.setValue(static_cast<double>(t_localizer(0, 0)),
                   static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)),
                   static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)),
                   static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)),
                   static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    localizer_pose_.x = t_base_link(0, 3);
    localizer_pose_.y = t_base_link(1, 3);
    localizer_pose_.z = t_base_link(2, 3);

    // 设置 localizer_pose 的旋转 rpy 角度
    mat_l.getRPY(localizer_pose_.roll, localizer_pose_.pitch, localizer_pose_.yaw, 1);

    // 更新 ndt_pose 获取 NDT 配准之后的位置
    ndt_pose_.x = t_localizer(0, 3);
    ndt_pose_.y = t_localizer(1, 3);
    ndt_pose_.z = t_localizer(2, 3);
    mat_b.getRPY(ndt_pose_.roll, ndt_pose_.pitch, ndt_pose_.yaw, 1);



    //  lei  da  dao  di  tu

    current_pose_.x = ndt_pose_.x;
    current_pose_.y = ndt_pose_.y;
    current_pose_.z = ndt_pose_.z;
    current_pose_.roll = ndt_pose_.roll;
    current_pose_.pitch = ndt_pose_.pitch;
    current_pose_.yaw = ndt_pose_.yaw;




//=======================================对比  三种方式下的 最终ndt匹配分数======================================================



  //  // 指定文件路径和名称  
  //   // std::string filename = "/home/lzl/scores_and_delays_imu.txt";  
  //       // std::string filename = "/home/lzl/scores_and_delays_velocity.txt";  
  //           std::string filename = "/home/lzl/scores_and_delays.txt"; 
  
  //   // 打开文件以写入数据（如果文件不存在，则会创建它）  
  //   std::ofstream file(filename, std::ios::out | std::ios::app);  
  
  //   // 检查文件是否成功打开  
  //   if (!file.is_open()) {  
  //       std::cerr << "无法打开文件: " << filename << std::endl;  
 
  //   }  
  
  //   // 将数据写入文件，每列之间使用制表符（\t）或逗号（,）分隔  
  //   // 这里使用制表符，因为它在文本文件中更易于阅读  
  //   // file << fitness_score_ << "\t" << delay_time_ << std::endl;  
  //       file << fitness_score_ << std::endl;  
  
  //   // 关闭文件  
  //   file.close();  
  
  //   // 输出到控制台以确认  
  //   std::cout << "数据已成功写入文件: " << filename << std::endl; 

//=============================================================================================

    bool ndt_source_too_low = false;
    // TODO 差值加考虑匹配分数
    if (has_converged_) {
        ROS_INFO("[ndt_localizer_pro] ===> converged. matching score: %.3f, delay_time_: %.4f", fitness_score_, delay_time_);
        localizer_truth_ = 1;
    } else {
        localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::DANGER);
        localizer_status_.reason = "Not Converged!!!";
        // ROS_WARN("[ndt_localizer_pro] ===> not converged. matching score: %.3f, delay_time_: %.4f", fitness_score_, delay_time_);
        localizer_truth_ = 0;
    }   





    if (fitness_score_ >= align_error_threshold_) {
        error_offset_num_ ++;

        // std::cout << "--------------------------------- error_offset_num_ ++;--------------------------------" << error_offset_num_ <<std::endl;
        ndt_source_too_low = true;
        ROS_WARN("[ndt_localizer_pro] ===> matching score: %.3f, error_offset_num_: %d", fitness_score_, error_offset_num_);
        if (use_gnss_fix_ && error_offset_num_ >= offset_count_threshold_) {
            double dist = double(distance2points(current_pose_, fix_pose_gnss_));
            if (dist >= distance_pose_to_fix_threshold_) {
                current_pose_.x = fix_pose_gnss_.x;
                current_pose_.y = fix_pose_gnss_.y;
                current_pose_.z = fix_pose_gnss_.z;
                current_pose_.roll = fix_pose_gnss_.roll;
                current_pose_.pitch = fix_pose_gnss_.pitch;
                current_pose_.yaw = fix_pose_gnss_.yaw;
                previous_pose_.x = current_pose_.x;
                previous_pose_.y = current_pose_.y;
                previous_pose_.z = current_pose_.z;
                previous_pose_.roll = current_pose_.roll;
                previous_pose_.pitch = current_pose_.pitch;
                previous_pose_.yaw = current_pose_.yaw;
                current_velocity_x_ = 0.0;
                current_velocity_y_ = 0.0;
                current_velocity_z_ = 0.0;
                angular_velocity_ = 0.0;

                current_accel_ = 0.0;
                current_accel_x_ = 0.0;
                current_accel_y_ = 0.0;
                current_accel_z_ = 0.0;

                offset_x_ = 0.0;
                offset_y_ = 0.0;
                offset_z_ = 0.0;
                offset_yaw_ = 0.0;
                
                error_offset_num_ = 0;

                localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::WARN);
                localizer_status_.reason = "By Gnss Fix";
            } else {
                localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::WARN);
                localizer_status_.reason = "Ndt Source Too Low But Gps Is Truth";
            }
        } else {
            localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::DANGER);
            localizer_status_.reason = "Localizer Score Too Low!";
            localizer_truth_ = 0;
        }
    } else {
        localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
    }
    // Calculate the difference between ndt_pose and predict_pose
    predict_pose_distance_ = sqrt(pow((ndt_pose_.x - predict_pose_for_ndt.x), 2) +
                              pow((ndt_pose_.y - predict_pose_for_ndt.y), 2) +
                              pow((ndt_pose_.z - predict_pose_for_ndt.z), 2));                     
    if (predict_pose_distance_ > predict_pose_error_threshold_) {
        f_use_predict_pose_ = 1;
    } else {
        f_use_predict_pose_ = 0;
    }
    // 判断是否使用预测位姿
    if (use_vehicle_fix_ && f_use_predict_pose_) {
        current_pose_.x = predict_pose_for_ndt.x;
        current_pose_.y = predict_pose_for_ndt.y;
        current_pose_.z = predict_pose_for_ndt.z;
        current_pose_.roll = predict_pose_for_ndt.roll;
        current_pose_.pitch = predict_pose_for_ndt.pitch;
        current_pose_.yaw = predict_pose_for_ndt.yaw;
        localizer_truth_ = 1;
    }

    // 在开启 gnss fix 情况下判定坐标误差是否过大
    if (use_gnss_fix_) {
        double dist = double(distance2points(current_pose_, fix_pose_gnss_));
        if (dist >= distance_pose_to_fix_threshold_) {
            ROS_WARN("[ndt_localozer_node] ==> pose to fix too far distance: %f", dist);
            localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::WARN);
            localizer_status_.reason = "Localizer Pose Too Far To Gps!";
            localizer_truth_ = false;
            voice_scene_.data = static_cast<int>(HtcbotCommonNS::VOICE_SCENES::MODULE_LOCALIZER_GPS_OFFSET_TOO_FAR_WARN);
            pub_voice_play_scene_.publish(voice_scene_);
        } 
        if (!localizer_truth_ && dist <= distance_gps_to_gps_threshold_) {
            localizer_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
            localizer_status_.reason = "Localizer Score Too Low But Gps Is Truth!";
            localizer_truth_ = true;
        }
    }
 
    // 计算速度和加速度 TODO 为啥不从地盘直接拿 直接拿速度积分出距离
    // 计算当前姿态与上一个姿态在 x、y、z 方向上的差值 diff_x、diff_y、diff_z，
    // 以及偏航角（yaw）的差值 diff_yaw 
    diff_x_ = current_pose_.x - previous_pose_.x;
    diff_y_ = current_pose_.y - previous_pose_.y;
    diff_z_ = current_pose_.z - previous_pose_.z;







    diff_yaw_ = calcDiffForRadian(current_pose_.yaw, previous_pose_.yaw);



    // 位移
    diff_ = sqrt(pow(diff_x_, 2) + pow(diff_y_, 2) + pow(diff_z_, 2));
    // 当前姿态转换为相对于上一个姿态的相对坐标系中的姿态
    trans_current_pose_ = convertPoseIntoRelativeCoordinate(current_pose_, previous_pose_);

    // 当前线速度
    current_velocity_ = (diff_time_ > 0) ? (diff_ / diff_time_) : 0;




    // 确定速度的正负 反应车辆运动方向 
    current_velocity_ =  (trans_current_pose_.x >= 0) ? current_velocity_ : -current_velocity_;


    current_velocity_x_ = (diff_time_ > 0) ? (diff_x_ / diff_time_) : 0;
    current_velocity_y_ = (diff_time_ > 0) ? (diff_y_ / diff_time_) : 0;
    current_velocity_z_ = (diff_time_ > 0) ? (diff_z_ / diff_time_) : 0;
    angular_velocity_ = (diff_time_ > 0) ? (diff_yaw_ / diff_time_) : 0;


    // std::cout << "----------current_velocity_ -------------- current_velocity_x_--------current_velocity_y_------current_velocity_z_---angular_velocity_ ----"
    //  <<"  "<<current_velocity_<<"   "<<current_velocity_x_<<"   "<< current_velocity_y_<<"   "<<current_velocity_z_<<"  "<< angular_velocity_ <<std::endl;


    // 当前速度平滑处理
    current_velocity_smooth_ = (current_velocity_ + previous_velocity_ + previous_previous_velocity_) / 3.0;
    // TODO 有无必要??
    if (std::fabs(current_velocity_smooth_) < 0.1)
    {
      current_velocity_smooth_ = 0.0;
    }

    // 计算加速度以及各个方向上的加速度
    current_accel_ = (diff_time_ > 0) ? ((current_velocity_ - previous_velocity_) / diff_time_) : 0;
    current_accel_x_ = (diff_time_ > 0) ? ((current_velocity_x_ - previous_velocity_x_) / diff_time_) : 0;
    current_accel_y_ = (diff_time_ > 0) ? ((current_velocity_y_ - previous_velocity_y_) / diff_time_) : 0;
    current_accel_z_ = (diff_time_ > 0) ? ((current_velocity_z_ - previous_velocity_z_) / diff_time_) : 0;

    // 发布估计速度 m/s
    std_msgs::Float32 estimated_vel_mps_msgs_;  
    estimated_vel_mps_msgs_.data = current_velocity_;
    pub_estimated_vel_mps_.publish(estimated_vel_mps_msgs_);

    // 发布估计速度 km/s
    std_msgs::Float32 estimated_vel_kmps_msgs_;
    estimated_vel_kmps_msgs_.data = current_velocity_ * 3.6;
    pub_estimated_vel_kmps_.publish(estimated_vel_kmps_msgs_);

    // Set values for publishing pose
    predict_q.setRPY(predict_pose_.roll, predict_pose_.pitch, predict_pose_.yaw);
    geometry_msgs::PoseStamped predict_pose_msg;
    predict_pose_msg.header.frame_id = "map";
    predict_pose_msg.header.stamp = current_scan_time_;
    predict_pose_msg.pose.position.x = predict_pose_.x;
    predict_pose_msg.pose.position.y = predict_pose_.y;
    predict_pose_msg.pose.position.z = predict_pose_.z;
    predict_pose_msg.pose.orientation.x = predict_q.x();
    predict_pose_msg.pose.orientation.y = predict_q.y();
    predict_pose_msg.pose.orientation.z = predict_q.z();
    predict_pose_msg.pose.orientation.w = predict_q.w();
    // pub_predict_pose_.publish(predict_pose_msg);

    geometry_msgs::PoseStamped ndt_pose_msg;
    ndt_pose_msg.header.frame_id = "map";
    ndt_pose_msg.header.stamp = current_scan_time_;
    ndt_pose_msg.pose.position.x = ndt_pose_.x;
    ndt_pose_msg.pose.position.y = ndt_pose_.y;
    ndt_pose_msg.pose.position.z = ndt_pose_.z;
    ndt_pose_msg.pose.orientation.x = ndt_q.x();
    ndt_pose_msg.pose.orientation.y = ndt_q.y();
    ndt_pose_msg.pose.orientation.z = ndt_q.z();
    ndt_pose_msg.pose.orientation.w = ndt_q.w();
    // pub_ndt_pose_.publish(ndt_pose_msg);

    current_q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    geometry_msgs::PoseStamped current_pose_msg;
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time_;
    current_pose_msg.pose.position.x = current_pose_.x;
    current_pose_msg.pose.position.y = current_pose_.y;
    current_pose_msg.pose.position.z = current_pose_.z;
    current_pose_msg.pose.orientation.x = current_q.x();
    current_pose_msg.pose.orientation.y = current_q.y();
    current_pose_msg.pose.orientation.z = current_q.z();
    current_pose_msg.pose.orientation.w = current_q.w();
    pub_current_pose_.publish(current_pose_msg);

    if (localizer_truth_) {
        pub_current_pose_truth_.publish(current_pose_msg);
    }

    // Send TF "/base_link" to "/map"
    transform_.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
    transform_.setRotation(current_q);
    br_.sendTransform(tf::StampedTransform(transform_, current_scan_time_, "map", "base_link"));
    matching_end_ = std::chrono::system_clock::now();
    exe_time_ = std::chrono::duration_cast<std::chrono::microseconds>(matching_end_ - matching_start_).count() / 1000.0;
    
    std_msgs::Float32 time_ndt_localizer_msgs;
    time_ndt_localizer_msgs.data = exe_time_;
    pub_time_ndt_localizer_.publish(time_ndt_localizer_msgs);

    // 发布模块状态
    pub_system_status_.publish(localizer_status_);

    if(is_debug_) {
        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "Sequence: " << input->header.seq << std::endl;
        std::cout << "Timestamp: " << input->header.stamp << std::endl;
        std::cout << "Frame ID: " << input->header.frame_id << std::endl;
        //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
        std::cout << "Predict Pose Distance: " << predict_pose_distance_ << std::endl;
        std::cout << "NDT has converged: " << has_converged_ << std::endl;
        std::cout << "Fitness Score: " << fitness_score_ << std::endl;
        std::cout << "Transformation Probability: " << trans_probability_ << std::endl;
        std::cout << "Execution Time: " << exe_time_ << " ms." << std::endl;
        std::cout << "Number of Iterations: " << iteration_ << std::endl;
        std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
        std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
                << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix: " << std::endl;
        std::cout << t_base_link << std::endl;
        std::cout << "Align time: " << align_time_ << std::endl;
        std::cout << "Get fitness score time: " << get_fitness_score_time_ << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
    }

    previous_pose_.x = current_pose_.x;
    previous_pose_.y = current_pose_.y;
    previous_pose_.z = current_pose_.z;
    previous_pose_.roll = current_pose_.roll;
    previous_pose_.pitch = current_pose_.pitch;
    previous_pose_.yaw = current_pose_.yaw;
    
    previous_scan_time_ = current_scan_time_;

    previous_previous_velocity_ = previous_velocity_;
    previous_velocity_ = current_velocity_;
    previous_velocity_x_ = current_velocity_x_;
    previous_velocity_y_ = current_velocity_y_;
    previous_velocity_z_ = current_velocity_z_;
    previous_accel_ = current_accel_;

}


double NdtLocalizerPro::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

/**
 * 将目标姿态（target_pose）转换为相对于参考姿态（reference_pose）的相对坐标系中的姿态
*/
Pose NdtLocalizerPro::convertPoseIntoRelativeCoordinate(const Pose &target_pose, const Pose &reference_pose) {
    // 目标姿态矩阵
    tf::Quaternion target_q;
    target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    tf::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
    tf::Transform target_tf(target_q, target_v);
    // 参考姿态矩阵
    tf::Quaternion reference_q;
    reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
    tf::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
    tf::Transform reference_tf(reference_q, reference_v);
    // trans_target_tf 从参考姿态坐标系到目标姿态坐标系的变换
    tf::Transform trans_target_tf = reference_tf.inverse() * target_tf;
    // 从变换矩阵中提取出相对坐标系中的位置和姿态角 存储到 trans_target_pose
    Pose trans_target_pose;
    trans_target_pose.x = trans_target_tf.getOrigin().getX();
    trans_target_pose.y = trans_target_tf.getOrigin().getY();
    trans_target_pose.z = trans_target_tf.getOrigin().getZ();
    tf::Matrix3x3 tmp_m(trans_target_tf.getRotation());
    tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

    return trans_target_pose;
}

}