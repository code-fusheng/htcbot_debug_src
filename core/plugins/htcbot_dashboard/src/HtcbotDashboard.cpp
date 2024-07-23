/*
 * @Author: code-fusheng
 * @Date: 2024-04-27 23:42:31
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 16:27:17
 * @Description:
 */
#include "HtcbotDashboard.h"
#include "BasePanel.h"
#include "VehiclePanel.h"
#include "SensorPanel.h"

namespace htcbot_dashboard
{

    HtcbotDashboard::HtcbotDashboard(QWidget *parent)
        : rviz::Panel(parent)
    {

        QVBoxLayout *main_layout_ = new QVBoxLayout;

        // 创建tab切换面板
        tab_widget_ = new QTabWidget(this);

        switchVehiclePanel();
        switchSensorPanel();
        switchSystemPanel();
        switchBasePanel();

        main_layout_->addWidget(tab_widget_);
        setLayout(main_layout_);
    }

    void HtcbotDashboard::switchSensorPanel()
    {
        sensor_panel_ = new SensorPanel(tab_widget_);
        tab_widget_->addTab(sensor_panel_, "数据面板");
    }

    void HtcbotDashboard::switchVehiclePanel()
    {
        vehicle_panel_ = new VehiclePanel(tab_widget_);
        tab_widget_->addTab(vehicle_panel_, "车辆状态");
    }

    void HtcbotDashboard::switchSystemPanel()
    {
        QWidget *system_panel_ = new QWidget(tab_widget_);
        tab_widget_->addTab(system_panel_, "系统信息");
    }

    void HtcbotDashboard::switchBasePanel()
    {
    }

}

// 导出插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(htcbot_dashboard::HtcbotDashboard, rviz::Panel)