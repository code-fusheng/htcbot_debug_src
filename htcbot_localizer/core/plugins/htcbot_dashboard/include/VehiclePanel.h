/*
 * @Author: code-fusheng
 * @Date: 2024-04-23 14:26:45
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 17:54:36
 * @Description: 
 */
// VehiclePanel.h

#ifndef VEHICLE_PANEL_H
#define VEHICLE_PANEL_H

#include <htcbot_common.h>

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
#include <QDebug>

#include <ros/ros.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <htcbot_msgs/StatusHtcbotSensor.h>
#include <htcbot_msgs/StatusHtcbotSensorArray.h>
#include <htcbot_msgs/StatusHtcbotModule.h>
#include <htcbot_msgs/StatusHtcbotModuleArray.h>
#include <htcbot_msgs/StatusHtcbotSystem.h>

using namespace HtcbotCommonNS;

#define RAD2DEG 180. / M_PI

class VehiclePanel : public QWidget {
    Q_OBJECT

public:

    explicit VehiclePanel(QWidget *parent = nullptr);

private:

    ros::NodeHandle nh_;

    ros::Subscriber sub_system_status_;
    ros::Subscriber sub_module_status_array_;
    ros::Subscriber sub_sensor_status_array_;

   // 维护系统模块状态数组
    htcbot_msgs::StatusHtcbotModuleArray module_status_array_;
    // 维护系统传感器状态数组
    htcbot_msgs::StatusHtcbotSensorArray sensor_status_array_;

    QLabel *system_status_label_;

    QLabel *module_map_label_;
    QLabel *module_localizer_label_;
    QLabel *module_sensor_label_;
    QLabel *module_waypoint_label_;
    QLabel *module_planner_label_;
    QLabel *module_control_label_;
    QLabel *module_gateway_label_;
    QLabel *module_other_label_;

    QLabel *sensor_gnss_label_;
    QLabel *sensor_laser_label_;
    QLabel *sensor_ultrasonic_label_;
    QLabel *sensor_camera_label_;
    QLabel *sensor_depth_camera_label_;
    QLabel *sensor_imu_label_;
    QLabel *sensor_vehicle_label_;
    QLabel *sensor_other_label_;

    void callbackSystemStatus(const htcbot_msgs::StatusHtcbotSystem::ConstPtr &msg);
    void callbackSensorStatusArray(const htcbot_msgs::StatusHtcbotSensorArray::ConstPtr &msg);
    void callbackModuleStatusArray(const htcbot_msgs::StatusHtcbotModuleArray::ConstPtr &msg);

    std::string statusToString(HtcbotCommonNS::STATUS_TYPE status_type);
    void setLabelColor(QLabel* label, HtcbotCommonNS::STATUS_TYPE status_type);

};

#endif // VEHICLE_PANEL_H