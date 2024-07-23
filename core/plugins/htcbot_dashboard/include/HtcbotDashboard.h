#ifndef HTCBOT_DASHBOARD_H
#define HTCBOT_DASHBOARD_H

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



namespace htcbot_dashboard
{

class HtcbotDashboard : public rviz::Panel {
  Q_OBJECT   

public:
  HtcbotDashboard(QWidget *parent = 0);

private:

    QTabWidget *tab_widget_;


    QWidget *system_panel_;
    QWidget *vehicle_panel_;
    QWidget *sensor_panel_;
    QWidget *base_panel_;

    void switchSystemPanel();
    void switchVehiclePanel();
    void switchSensorPanel();
    void switchBasePanel();

};

}

#endif // HTCBOT_DASHBOARD_H