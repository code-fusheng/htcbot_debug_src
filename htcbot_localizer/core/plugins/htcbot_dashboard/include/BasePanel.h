// BasePanel.h

#ifndef BASE_PANEL_H
#define BASE_PANEL_H

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
#include <QListWidget>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cstdlib>

class BasePanel : public QWidget {
    Q_OBJECT

public:
    explicit BasePanel(QWidget *parent = nullptr);

private:

    ros::NodeHandle nh_;
    ros::Publisher testPub_ = nh_.advertise<std_msgs::String>("htcbot/test", 1);;

    // 预瞄配置 

    QDialog *testDialog;

    bool testValue_;

    // 数据录制
    QDialog *recordDialog;

    rosbag::Bag *bagOut;

    QPushButton* start_button_;
    QPushButton* stop_button_;
    QListWidget* topic_list_;

    void openTestDialog();

    void openRosbagRecordDiglog();
    
};

#endif // BASE_PANEL_H