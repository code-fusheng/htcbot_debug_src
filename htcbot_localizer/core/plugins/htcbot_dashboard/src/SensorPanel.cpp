/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-03-24 13:04:02
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 01:29:13
 * @FilePath: /src/core/plugins/htcbot_dashboard/src/SensorPanel.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "SensorPanel.h"

SensorPanel::SensorPanel(QWidget *parent) : QWidget(parent) {

    nh_ = ros::NodeHandle();

    QVBoxLayout *layout = new QVBoxLayout(this);

    // QPushButton *test_button = new QPushButton("测试按钮");
    // layout->addWidget(test_button);
    QGroupBox *vehicle_pose_box = new QGroupBox("车辆姿态");
    QVBoxLayout *vehicle_pose_layout = new QVBoxLayout(); 
    current_pose_x_label_ = new QLabel("X: N/A");
    current_pose_y_label_ = new QLabel("Y: N/A");
    current_pose_z_label_ = new QLabel("Z: N/A");
    current_pose_roll_label_ = new QLabel("Roll: N/A");
    current_pose_pitch_label_ = new QLabel("Pitch: N/A");
    current_pose_yaw_label_ = new QLabel("Yaw: N/A");

    vehicle_pose_layout->addWidget(current_pose_x_label_);
    vehicle_pose_layout->addWidget(current_pose_y_label_);
    vehicle_pose_layout->addWidget(current_pose_z_label_);
    vehicle_pose_layout->addWidget(current_pose_roll_label_);
    vehicle_pose_layout->addWidget(current_pose_pitch_label_);
    vehicle_pose_layout->addWidget(current_pose_yaw_label_);
    vehicle_pose_box->setLayout(vehicle_pose_layout);
    layout->addWidget(vehicle_pose_box);

    QGroupBox *vehicle_location_box = new QGroupBox("车辆定位");
    QVBoxLayout *vehicle_location_layout = new QVBoxLayout(); 
    latitude_label_ = new QLabel("纬度: N/A");
    longitude_label_ = new QLabel("经度: N/A");
    altitude_label_ = new QLabel("高度: N/A");
    vehicle_location_layout->addWidget(latitude_label_);
    vehicle_location_layout->addWidget(longitude_label_);
    vehicle_location_layout->addWidget(altitude_label_);
    vehicle_location_box->setLayout(vehicle_location_layout);
    layout->addWidget(vehicle_location_box);

    QGroupBox *vehicle_status_box = new QGroupBox("车辆状态");
    QVBoxLayout *vehicle_status_layout = new QVBoxLayout();
    speed_label_ = new QLabel("当前速度: N/A");
    steer_label_ = new QLabel("转向角度: N/A");
    odom_label_ = new QLabel("里程统计: N/A");
    shift_label_ = new QLabel("车辆档位: N/A");
    break_label_ = new QLabel("刹车状态: N/A");

    vehicle_status_layout->addWidget(speed_label_);
    vehicle_status_layout->addWidget(steer_label_);
    vehicle_status_layout->addWidget(odom_label_);
    vehicle_status_layout->addWidget(shift_label_);
    vehicle_status_layout->addWidget(break_label_);

    vehicle_status_box->setLayout(vehicle_status_layout);
    layout->addWidget(vehicle_status_box);
    
    QGroupBox *ultrasonic_info_box_ = new QGroupBox("超声波信息");
    QVBoxLayout *ultrasonic_info_layout = new QVBoxLayout();
    left_label_ = new QLabel("左前侧: N/A");
    left_top_label_ = new QLabel("左上角: N/A");
    right_top_label_ = new QLabel("右上角: N/A");
    right_label_ = new QLabel("右前侧: N/A");
    ultrasonic_info_layout->addWidget(left_label_);
    ultrasonic_info_layout->addWidget(left_top_label_);
    ultrasonic_info_layout->addWidget(right_top_label_);
    ultrasonic_info_layout->addWidget(right_label_);
    ultrasonic_info_box_->setLayout(ultrasonic_info_layout);
    layout->addWidget(ultrasonic_info_box_);

    QGroupBox *vehicle_cmd_box = new QGroupBox("车辆遥控");
    QVBoxLayout *vehicle_cmd_layout = new QVBoxLayout();
    vehicle_cmd_box->setLayout(vehicle_cmd_layout);
    // layout->addWidget(vehicle_cmd_box);

    sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &SensorPanel::callbackVehicleStatus, this);
    sub_current_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &SensorPanel::callbackCurrentPose, this);
    sub_gps_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/gnss/gps_pose", 1, &SensorPanel::callbackGpsPose, this);
    sub_ultrasonic_detect_ = nh_.subscribe("/ultrasonic_detection", 1, &SensorPanel::callbackUltrasonicDetection, this);
    // sub_localizer_system_status_ = nh_.subscribe("/htcbot/localizer_system_status", 1, &SensorPanel::callbackSystemStatus, this);
    // sub_map_system_status_ = nh_.subscribe("/htcbot/map_system_status", 1, &SensorPanel::callbackSystemStatus, this);

}

// void SensorPanel::callbackSystemStatus(const htcbot_msgs::StatusHtcbotSystem::ConstPtr& msg) {
//     if (msg->system_module == "LOCALIZER") {
//         localizer_status_label_->setText(QString("定位状态: %1").arg(QString::fromStdString(msg->system_status)));
//     } else if (msg->system_module == "MAP") {
//         map_status_label_->setText(QString("地图状态: %1").arg(QString::fromStdString(msg->system_status)));
//     }
// }

void SensorPanel::callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr& input) {
    speed_label_->setText(QString("当前速度: %1 m/s").arg(input->cur_speed));
    steer_label_->setText(QString("转向角度: %1 °").arg(input->cur_steer));
    odom_label_->setText(QString("里程统计: %1 km").arg(input->total_odometer));
    shift_label_->setText(QString("车辆档位: %1").arg(input->shift_level));
    break_label_->setText(QString("刹车状态: %1").arg(input->brake_level));
}

void SensorPanel::callbackUltrasonicDetection(const ultrasonic_driver::UltraSonicDetect::ConstPtr& input) {
    const size_t distance_array_size = sizeof(input->distance) / sizeof(input->distance[0]);
    for (size_t i = 0; i < distance_array_size; ++i) {
        QLabel* label;
        switch(i) {
            case 0:
                label = left_label_;
                break;
            case 1:
                label = left_top_label_;
                break;
            case 2:
                label = right_top_label_;
                break;
            case 3:
                label = right_label_;
                break;
            default:
                continue; // 超出索引范围，继续下一次循环
        }
        if (input->distance[i] == 0 || input->distance[i] >= 600) {
            label->setText(QString("探头-%1 %2: MAX").arg(i).arg(label->objectName()));
            label->setStyleSheet("color: green;");
        } else {
            label->setText(QString("探头-%1 %2: %3 mm").arg(i).arg(label->objectName()).arg(input->distance[i]));
            if (input->distance[i] > 250 && input->distance[i] < 300) {
                label->setStyleSheet("color: red;");
            } else {
                label->setStyleSheet(""); // 清除样式表，恢复默认颜色
            }
        }
    }
}

void SensorPanel::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& input) {
    current_pose_ = *input;
    tf::Quaternion q(
    input->pose.orientation.x, input->pose.orientation.y,
    input->pose.orientation.z, input->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll;  // | x
    double pitch; // | y
    double yaw;   // | z
    m.getRPY(roll, pitch, yaw);
    current_pose_x_label_->setText(QString("X: %1").arg(input->pose.position.x, 0, 'f', 2));
    current_pose_y_label_->setText(QString("Y: %1").arg(input->pose.position.y, 0, 'f', 2));
    current_pose_z_label_->setText(QString("Z: %1").arg(input->pose.position.z, 0, 'f', 2));
    current_pose_roll_label_->setText(QString("Roll: %1").arg(roll * RAD2DEG, 0, 'f', 2));
    current_pose_pitch_label_->setText(QString("Pitch: %1").arg(pitch * RAD2DEG, 0, 'f', 2));
    current_pose_yaw_label_->setText(QString("Yaw: %1").arg(yaw * RAD2DEG, 0, 'f', 2));
}

void SensorPanel::callbackGpsPose(const geometry_msgs::PoseStamped::ConstPtr& input) {
    gps_pose_ = *input;
    latitude_label_->setText(QString("Lat: %1").arg(input->pose.position.x, 0, 'f', 7));
    longitude_label_->setText(QString("Lon: %1").arg(input->pose.position.y, 0, 'f', 7));
    altitude_label_->setText(QString("Alt: %1").arg(input->pose.position.z, 0, 'f', 7));
}

