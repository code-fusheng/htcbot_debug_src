#include "NavigationPanel.h"

NavigationPanel::NavigationPanel(QWidget *parent) : QWidget(parent) {

    nh_ = ros::NodeHandle();
    nh_.getParam("/can_adapter_node/dist_front_stop", obs_stop_front_distance_value_);
    nh_.getParam("/can_adapter_node/dist_back_stop", obs_stop_back_distance_value_);

    // 创建导航面板的控件和逻辑
    QPushButton *navigate_button = new QPushButton("Navigate");
    connect(navigate_button, &QPushButton::clicked, this, &NavigationPanel::navigateClicked);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(navigate_button);

    // 车辆状态

    // 导航配置
    QGroupBox *navigation_conf_box = new QGroupBox("导航配置");
    QVBoxLayout *navigation_conf_layout = new QVBoxLayout();

    layout->addWidget(navigation_conf_box);

    // 定位配置
    QGroupBox *localizer_conf_box = new QGroupBox("定位配置");
    QVBoxLayout *localizer_conf_layout = new QVBoxLayout();
    // 卫星定位
    QPushButton *gnss_switch_button_ = new QPushButton("GNSS定位");
    localizer_conf_layout->addWidget(gnss_switch_button_);
    
    connect(gnss_switch_button_, &QPushButton::clicked, this, &NavigationPanel::pubLocalizerGnssSwitch);
    
    localizer_conf_box->setLayout(localizer_conf_layout);
    layout->addWidget(localizer_conf_box);

    // 预瞄配置
    QGroupBox *lookahead_conf_box = new QGroupBox("预瞄配置");
    QVBoxLayout *lookahead_conf_layout = new QVBoxLayout();

    layout->addWidget(lookahead_conf_box);

    // 停障配置
    QGroupBox *obstacle_conf_box = new QGroupBox("停障配置");
    QVBoxLayout *obstacle_conf_layout = new QVBoxLayout();

    // 前方停障
    QHBoxLayout *front_obs_stop_layout_ = new QHBoxLayout();
    QLabel *front_obs_stop_label_ = new QLabel("前方停障: ");
    front_obs_stop_input_ = new QLineEdit(this);
    front_obs_stop_input_->setText(QString::number(obs_stop_front_distance_value_));
    front_obs_stop_layout_->addWidget(front_obs_stop_label_);
    front_obs_stop_layout_->addWidget(front_obs_stop_input_);
    obstacle_conf_layout->addLayout(front_obs_stop_layout_);

    // 后方停障
    QHBoxLayout *back_obs_stop_layout_ = new QHBoxLayout();
    QLabel *back_obs_stop_label_ = new QLabel("后方停障: ");
    back_obs_stop_input_ = new QLineEdit(this);
    back_obs_stop_input_->setText(QString::number(obs_stop_back_distance_value_));
    back_obs_stop_layout_->addWidget(back_obs_stop_label_);
    back_obs_stop_layout_->addWidget(back_obs_stop_input_);
    obstacle_conf_layout->addLayout(back_obs_stop_layout_);

    // 左侧停障
    QHBoxLayout *left_obs_stop_layout_ = new QHBoxLayout();
    QLabel *left_obs_stop_label_ = new QLabel("左侧停障: ");
    left_obs_stop_input_ = new QLineEdit(this);
    left_obs_stop_input_->setText(QString::number(obs_stop_left_distance_value_));
    left_obs_stop_layout_->addWidget(left_obs_stop_label_);
    left_obs_stop_layout_->addWidget(left_obs_stop_input_);
    obstacle_conf_layout->addLayout(left_obs_stop_layout_);

    // 右侧停障
    QHBoxLayout *right_obs_stop_layout_ = new QHBoxLayout();
    QLabel *right_obs_stop_label_ = new QLabel("右侧停障: ");
    right_obs_stop_input_ = new QLineEdit(this);
    right_obs_stop_input_->setText(QString::number(obs_stop_right_distance_value_));
    right_obs_stop_layout_->addWidget(right_obs_stop_label_);
    right_obs_stop_layout_->addWidget(right_obs_stop_input_);
    obstacle_conf_layout->addLayout(right_obs_stop_layout_);

    QHBoxLayout *obs_conf_op_layout = new QHBoxLayout();
    QPushButton *obs_conf_reset_button = new QPushButton("重置");
    QPushButton *obs_conf_set_button = new QPushButton("确定");
    obs_conf_op_layout->addWidget(obs_conf_reset_button);
    obs_conf_op_layout->addWidget(obs_conf_set_button);
    obstacle_conf_layout->addLayout(obs_conf_op_layout);

    connect(obs_conf_reset_button, &QPushButton::clicked, this, &NavigationPanel::obsConfOpResetHandle);
    connect(obs_conf_set_button, &QPushButton::clicked, this, &NavigationPanel::obsConfOpSetHandle);

    obstacle_conf_box->setLayout(obstacle_conf_layout);

    layout->addWidget(obstacle_conf_box);

    // 导航任务

    localizer_gnss_switch_pub_ = nh_.advertise<htcbot_msgs::ModeSwitch>("htcbot/localizer_gnss_switch", 1);
    conf_obs_stop_pub_ = nh_.advertise<htcbot_msgs::ConfObstacleDetection>("htcbot/conf_obstacle_detection", 1);

}

void NavigationPanel::navigateClicked() {
    // 处理导航按钮点击事件
    // 可以发出信号，让其他类处理导航逻辑
    QMessageBox::information(this, "导航测试", "xxxxxx");
}

void NavigationPanel::pubLocalizerGnssSwitch() {
    localizer_gnss_switch_value_ = !localizer_gnss_switch_value_;
    QMessageBox::information(this, "当前GNSS定位开关", QString(localizer_gnss_switch_value_ ? "true" : "false"));
    htcbot_msgs::ModeSwitch switch_msg;
    switch_msg.mode = "Gnss_Localizer";
    switch_msg.switch_to = localizer_gnss_switch_value_;
    localizer_gnss_switch_pub_.publish(switch_msg);
}

void NavigationPanel::obsConfOpResetHandle() {
}

void NavigationPanel::obsConfOpSetHandle() {
    QMessageBox::StandardButton btn = QMessageBox::question(this, "Htcbot Control Panel!", "是否确认更新停障检测配置参数?", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if (QMessageBox::Yes == btn) {
        htcbot_msgs::ConfObstacleDetection conf_msg;
        obs_stop_front_distance_value_ = front_obs_stop_input_->text().toDouble();
        obs_stop_back_distance_value_ = back_obs_stop_input_->text().toDouble();
        obs_stop_left_distance_value_ = left_obs_stop_input_->text().toDouble();
        obs_stop_right_distance_value_ = right_obs_stop_input_->text().toDouble();
        conf_msg.obs_stop_front_distance = obs_stop_front_distance_value_;
        conf_msg.obs_stop_back_distance = obs_stop_back_distance_value_;
        conf_msg.obs_stop_left_distance = obs_stop_left_distance_value_;
        conf_msg.obs_stop_right_distance = obs_stop_right_distance_value_;
        conf_obs_stop_pub_.publish(conf_msg);
    }
}