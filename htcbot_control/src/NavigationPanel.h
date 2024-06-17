// NavigationPanel.h

#ifndef NAVIGATION_PANEL_H
#define NAVIGATION_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QGroupBox>
#include <QLabel>
#include <QDir>
#include <QLineEdit>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <htcbot_msgs/ModeSwitch.h>
#include <htcbot_msgs/ConfObstacleDetection.h>

class NavigationPanel : public QWidget {
    Q_OBJECT

public:
    explicit NavigationPanel(QWidget *parent = nullptr);

private:

    ros::NodeHandle nh_;
    ros::Publisher localizer_gnss_switch_pub_;
    ros::Publisher conf_obs_stop_pub_;
    

    void navigateClicked();
    void pubLocalizerGnssSwitch();

    // 障碍检测配置重置处理
    void obsConfOpResetHandle();
    // 障碍检测配置处理
    void obsConfOpSetHandle();

    bool localizer_gnss_switch_value_ = true;

    QLineEdit *front_obs_stop_input_;
    QLineEdit *back_obs_stop_input_;
    QLineEdit *left_obs_stop_input_;
    QLineEdit *right_obs_stop_input_;

    double obs_stop_front_distance_value_;
    double obs_stop_back_distance_value_;
    double obs_stop_left_distance_value_;
    double obs_stop_right_distance_value_;

};

#endif // NAVIGATION_PANEL_H
