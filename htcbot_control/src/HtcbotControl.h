#ifndef HTCBOT_CONTROL_H
#define HTCBOT_CONTROL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QDir>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QGroupBox>
#include <QTabWidget>
#include <QComboBox>
#include <QStringList>
#include <QTimer>

#include <std_msgs/String.h>

#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/ModeSwitch.h>
#include <htcbot_msgs/GridMapBuild.h>
#include <htcbot_msgs/MappingConf.h>

namespace htcbot_control
{
class HtcbotControl : public rviz::Panel {
  Q_OBJECT
  
public:
  HtcbotControl(QWidget *parent = 0);

public Q_SLOTS:
  void confirmTopic();
  
  void pubMappingSwitch();
  void pubLocalizerSwitch();
  void pubPoseRecordSwitch();
  void pubReMappingSwitch();
  void pubCloudGatewaySwitch();

  void pubRunningManualSwitch();
  void pubRunningAutoSwitch();
  void pubRunningAvoidSwitch();
  void pubRunningCycleSwitch();

  void pubBuildGridMap();
  void pubCarTaskStart();
  void pubCarTaskPuse();

private:

  ros::NodeHandle nh_;

  const QString base_dir = "/home/data";

  QString map_grid_str_;
  QString map_static_str_;
  QString map_dynamic_str_;
  QString pathes_str_;
  QString config_str_;

  QVBoxLayout *main_layout_;

  QGroupBox *map_box_;
  QVBoxLayout *map_layout_;

  QHBoxLayout *parent_layout_;
  QLabel *parent_path_label_;
  QLineEdit *parent_path_input_;
  // 场景名称
  QString parent_path_value_;

  QHBoxLayout *child_layout_;
  QLabel *child_path_label_;
  QLineEdit *child_path_input_;
  // 场景编号
  QString child_path_value_;


  QPushButton *reset_button_;
  QPushButton *confirm_button_;

  QPushButton *mapping_switch_button_;
  bool mapping_switch_value_ = false;
  int mapping_state_value_ = 0;

  QPushButton *localizer_switch_button_;
  bool localizer_switch_value_ = false;

  QPushButton *pose_record_switch_button_;
  bool pose_record_switch_value_ = false;

  QPushButton *re_mapping_switch_button_;
  bool re_mapping_switch_value_ = false;

  QPushButton *cloud_gateway_switch_button_;
  bool cloud_gateway_switch_value_ = false;

  QLineEdit *mapping_voxel_input_;
  QLineEdit *mapping_step_input_;
  QLineEdit *mapping_shift_input_; 

  // Grid Map Build
  QLineEdit *grid_voxel_input_;
  QLineEdit *grid_side_input_;

  QPushButton *manual_running_button_;
  bool manual_running_switch_value_ = false;
  QLabel *manual_running_label_;

  QPushButton *auto_running_button_;
  bool auto_running_switch_value_ = false;
  QLabel *auto_running_label_; 

  QPushButton *avoid_running_button_;
  bool avoid_running_switch_value_ = false;
  QLabel *avoid_running_label_;

  QPushButton *cycle_running_button_;
  bool cycle_running_switch_value_ = false;
  QLabel *cycle_running_label_;

  QPushButton *task_start_button_;
  QPushButton *task_puse_button_;
  QPushButton *task_end_button_;
  bool task_state_value_ = 0;

  ros::Publisher topic_publisher_;
  // 场景/地图路径配置发布者
  ros::Publisher map_path_conf_pub_;
  ros::Publisher mode_switch_pub_;
  ros::Publisher mapping_conf_pub_;
  ros::Publisher grid_map_build_pub_;

  QTabWidget *tab_widget_;
  QWidget *mapping_panel_;
  QWidget *navigation_panel_;
  QWidget *route_panel_;
  QWidget *home_panel_;
  QWidget *params_panel_;

  void pubMappingConf(const int state);

  void switchMappingPanel();
  void switchNavigationPanel();
  void switchRoutePanel();
  void switchHomePanel();
  void switchParamsPanel();

  bool checkDirValidate(const QString parent_text, const QString child_text);

};
}

#endif // HTCBOT_CONTROL_H
