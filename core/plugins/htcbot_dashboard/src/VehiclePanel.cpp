/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2024-03-24 13:04:02
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 17:58:23
 * @FilePath: /src/core/plugins/htcbot_dashboard/src/VehiclePanel.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "VehiclePanel.h"

VehiclePanel::VehiclePanel(QWidget *parent) : QWidget(parent)
{

    nh_ = ros::NodeHandle();

    QVBoxLayout *layout = new QVBoxLayout(this);

    QGroupBox *system_box = new QGroupBox("系统状态");
    QVBoxLayout *system_layout = new QVBoxLayout();

    system_status_label_ = new QLabel("智驾系统: NONE");
    system_layout->addWidget(system_status_label_);
    system_box->setLayout(system_layout);
    layout->addWidget(system_box);

    QGroupBox *module_box = new QGroupBox("模块状态");
    QVBoxLayout *module_layout = new QVBoxLayout();

    module_map_label_ = new QLabel("地图模块: NONE");
    module_localizer_label_ = new QLabel("定位模块: NONE");
    module_sensor_label_ = new QLabel("感知模块: NONE");
    module_waypoint_label_ = new QLabel("轨迹模块: NONE");
    module_planner_label_ = new QLabel("规划模块: NONE");
    module_control_label_ = new QLabel("控制模块: NONE");
    module_gateway_label_ = new QLabel("网关模块: NONE");
    module_other_label_ = new QLabel("其他模块: NONE");
    module_layout->addWidget(module_map_label_);
    module_layout->addWidget(module_localizer_label_);
    module_layout->addWidget(module_sensor_label_);
    module_layout->addWidget(module_waypoint_label_);
    module_layout->addWidget(module_planner_label_);
    module_layout->addWidget(module_control_label_);
    module_layout->addWidget(module_gateway_label_);
    module_layout->addWidget(module_other_label_);
    module_box->setLayout(module_layout);
    layout->addWidget(module_box);

    QGroupBox *sensor_box = new QGroupBox("感知状态");
    QVBoxLayout *sensor_layout = new QVBoxLayout();

    sensor_gnss_label_ = new QLabel("GNSS: NONE");
    sensor_laser_label_ = new QLabel("激光雷达: NONE");
    sensor_ultrasonic_label_ = new QLabel("超声波: NONE");
    sensor_camera_label_ = new QLabel("摄像头: NONE");
    sensor_depth_camera_label_ = new QLabel("深度相机: NONE");
    sensor_imu_label_ = new QLabel("IMU: NONE");
    sensor_vehicle_label_ = new QLabel("底盘: NONE");
    sensor_other_label_ = new QLabel("其他: NONE");
    sensor_layout->addWidget(sensor_gnss_label_);
    sensor_layout->addWidget(sensor_laser_label_);
    sensor_layout->addWidget(sensor_ultrasonic_label_);
    sensor_layout->addWidget(sensor_camera_label_);
    sensor_layout->addWidget(sensor_depth_camera_label_);
    sensor_layout->addWidget(sensor_imu_label_);
    sensor_layout->addWidget(sensor_vehicle_label_);
    sensor_layout->addWidget(sensor_other_label_);
    sensor_box->setLayout(sensor_layout);
    layout->addWidget(sensor_box);

    sub_system_status_ = nh_.subscribe("/htcbot/system_status", 10, &VehiclePanel::callbackSystemStatus, this);
    sub_module_status_array_ = nh_.subscribe("/htcbot/module_status_array", 10, &VehiclePanel::callbackModuleStatusArray, this);
    sub_sensor_status_array_ = nh_.subscribe("/htcbot/sensor_status_array", 10, &VehiclePanel::callbackSensorStatusArray, this);
}

void VehiclePanel::callbackModuleStatusArray(const htcbot_msgs::StatusHtcbotModuleArray::ConstPtr &msg)
{
    for (const auto &_module_status : msg->status_array)
    {
        HtcbotCommonNS::MODULE_TYPE module_type = static_cast<HtcbotCommonNS::MODULE_TYPE>(_module_status.module_type);
        HtcbotCommonNS::STATUS_TYPE status_type = static_cast<HtcbotCommonNS::STATUS_TYPE>(_module_status.module_status);
        std::string reason = _module_status.reason;
        // 更新对应模块的状态标签文本
        QLabel *module_label = nullptr;
        switch (module_type)
        {
        case HtcbotCommonNS::MODULE_TYPE::MAP:
            module_label = module_map_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::LOCALIZER:
            module_label = module_localizer_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::SENSOR:
            module_label = module_sensor_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::WAYPOINT:
            module_label = module_waypoint_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::PLANNER:
            module_label = module_planner_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::CONTROL:
            module_label = module_control_label_;
            break;
        case HtcbotCommonNS::MODULE_TYPE::GATEWAY:
            module_label = module_gateway_label_;
            break;
        // 添加其他模块的更新代码
        default:
            break;
        }
        // 更新模块状态标签文本和颜色
        if (module_label != nullptr)
        {
            QString current_text = module_label->text();
            // qDebug() << "Current text: " << current_text;
            std::string text_str = current_text.toStdString();
            int pos = text_str.find(":");
            // qDebug() << "Position of colon: " << pos;
            if (pos != -1)
            {
                // 显示 reason
                text_str.replace(pos + 2, text_str.length() - pos - 2, statusToString(status_type) + "  " + reason);
                current_text = QString::fromStdString(text_str); // 将 std::string 转换回 QString
                module_label->setText(current_text);
                setLabelColor(module_label, status_type);
            }
        }
    }
}

void VehiclePanel::callbackSystemStatus(const htcbot_msgs::StatusHtcbotSystem::ConstPtr &msg)
{
    QString current_text = system_status_label_->text();
    std::string text_str = current_text.toStdString();
    int pos = text_str.find(":");
    if (pos != -1)
    {
        text_str.replace(pos + 2, text_str.length() - pos - 2, msg->status ? "Ture" : "False");
        current_text = QString::fromStdString(text_str); // 将 std::string 转换回 QString
        system_status_label_->setText(current_text);
        if (msg->status)
        {
            system_status_label_->setStyleSheet("color : green;");
        }
        else
        {
            system_status_label_->setStyleSheet("color : red;");
        }
    }
}

void VehiclePanel::callbackSensorStatusArray(const htcbot_msgs::StatusHtcbotSensorArray::ConstPtr &msg)
{
    for (const auto &_sensor_status : msg->status_array)
    {
        HtcbotCommonNS::SENSOR_TYPE sensor_type = static_cast<HtcbotCommonNS::SENSOR_TYPE>(_sensor_status.sensor_type);
        HtcbotCommonNS::STATUS_TYPE status_type = static_cast<HtcbotCommonNS::STATUS_TYPE>(_sensor_status.sensor_status);
        std::string reason = _sensor_status.reason;
        QLabel *sensor_label = nullptr;
        switch (sensor_type)
        {
        case HtcbotCommonNS::SENSOR_TYPE::GNSS:
            sensor_label = sensor_gnss_label_;
            break;
        case HtcbotCommonNS::SENSOR_TYPE::LASER:
            sensor_label = sensor_laser_label_;
            break;
        case HtcbotCommonNS::SENSOR_TYPE::ULTRASONIC:
            sensor_label = sensor_ultrasonic_label_;
            break;
        case HtcbotCommonNS::SENSOR_TYPE::IMU:
            sensor_label = sensor_imu_label_;
            break;
        case HtcbotCommonNS::SENSOR_TYPE::VEHICLE:
            sensor_label = sensor_vehicle_label_;
            break;
        case HtcbotCommonNS::SENSOR_TYPE::CAMERA:
            sensor_label = sensor_camera_label_;
            break;
        // 添加其他模块的更新代码
        default:
            break;
        }
        // 更新模块状态标签文本和颜色
        if (sensor_label != nullptr)
        {
            QString current_text = sensor_label->text();
            std::string text_str = current_text.toStdString();
            int pos = text_str.find(":");
            if (pos != -1)
            {
                text_str.replace(pos + 2, text_str.length() - pos - 2, statusToString(status_type) + "  " + reason);
                current_text = QString::fromStdString(text_str); // 将 std::string 转换回 QString
                sensor_label->setText(current_text);
                setLabelColor(sensor_label, status_type);
            }
        }
    }
}

std::string VehiclePanel::statusToString(HtcbotCommonNS::STATUS_TYPE status_type)
{
    switch (status_type)
    {
    case HtcbotCommonNS::STATUS_TYPE::NONE:
        return "NONE";
    case HtcbotCommonNS::STATUS_TYPE::READY:
        return "READY";
    case HtcbotCommonNS::STATUS_TYPE::WARN:
        return "WARN";
    case HtcbotCommonNS::STATUS_TYPE::DANGER:
        return "DANGER";
    case HtcbotCommonNS::STATUS_TYPE::EXC:
        return "EXC";
    case HtcbotCommonNS::STATUS_TYPE::EXPIRED:
        return "EXPIRED";

    default:
        return "UNKNOWN";
    }
}

void VehiclePanel::setLabelColor(QLabel *label, HtcbotCommonNS::STATUS_TYPE status_type)
{
    switch (status_type)
    {
    case HtcbotCommonNS::STATUS_TYPE::NONE:
        label->setStyleSheet("");
        break;
    case HtcbotCommonNS::STATUS_TYPE::READY:
        label->setStyleSheet("color : green;");
        break;
    case HtcbotCommonNS::STATUS_TYPE::WARN:
        label->setStyleSheet("color : orange;");
        break;
    case HtcbotCommonNS::STATUS_TYPE::DANGER:
        label->setStyleSheet("color : red;");
        break;
    case HtcbotCommonNS::STATUS_TYPE::EXC:
        label->setStyleSheet("color : red;");
        break;
    case HtcbotCommonNS::STATUS_TYPE::EXPIRED:
        label->setStyleSheet("color : gray;");
        break;
    default:
        label->setStyleSheet("color : gray;");
        break;
    }
}
