/*
 * @Author: code-fusheng
 * @Date: 2024-04-23 14:26:45
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-04-28 00:18:24
 * @Description: 
 */
// SensorPanel.h

#ifndef SENSOR_PANEL_H
#define SENSOR_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QGroupBox>
#include <QLabel>
#include <QDir>
#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDialogButtonBox>

#include <ros/ros.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <can_msgs/vehicle_status.h>
// #include <htcbot_msgs/StatusHtcbotSystem.h>
#include <ultrasonic_driver/UltraSonicDetect.h>

#define RAD2DEG 180. / M_PI

class SensorPanel : public QWidget {
    Q_OBJECT

public:
    explicit SensorPanel(QWidget *parent = nullptr);

private:

    ros::NodeHandle nh_;

    ros::Subscriber sub_vehicle_status_;
    ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_gps_pose_;
    ros::Subscriber sub_localizer_system_status_;
    ros::Subscriber sub_map_system_status_;
    ros::Subscriber sub_ultrasonic_detect_;

    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped gps_pose_;

    QLabel *current_pose_x_label_;
    QLabel *current_pose_y_label_;
    QLabel *current_pose_z_label_;
    QLabel *current_pose_roll_label_;
    QLabel *current_pose_pitch_label_;
    QLabel *current_pose_yaw_label_;

    QLabel *latitude_label_;
    QLabel *longitude_label_;
    QLabel *altitude_label_;

    QLabel *speed_label_;
    QLabel *steer_label_;
    QLabel *odom_label_;
    QLabel *shift_label_;
    QLabel *break_label_;

    QLabel *left_label_;
    QLabel *left_top_label_;
    QLabel *right_top_label_;
    QLabel *right_label_;

    QLabel *localizer_status_label_;
    QLabel *map_status_label_;
    QLabel *path_status_label_;
    QLabel *security_status_label_;

    void callbackVehicleStatus(const can_msgs::vehicle_status::ConstPtr& input);

    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& input);

    void callbackGpsPose(const geometry_msgs::PoseStamped::ConstPtr& input);

    void callbackUltrasonicDetection(const ultrasonic_driver::UltraSonicDetect::ConstPtr& input);

    // void callbackSystemStatus(const htcbot_msgs::StatusHtcbotSystem::ConstPtr& msg);


};

#endif // SENSOR_PANEL_H