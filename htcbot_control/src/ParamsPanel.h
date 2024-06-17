// ParamsPanel.h

#ifndef PARAMS_PANEL_H
#define PARAMS_PANEL_H

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
#include <QTimer>
#include <QSlider>
#include <QSpinBox>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <htcbot_msgs/ConfPurePursuit.h>
#include <htcbot_msgs/ConfEuclideanCluster.h>
#include <htcbot_msgs/ConfOpLocalPlanner.h>
#include <htcbot_msgs/ConfVehicle.h>
#include <htcbot_msgs/ConfNdtMapping.h>

struct OpLocalPlannerParams
{
    bool    modeSwitch;
	double 	maxSpeed;
	double 	minSpeed;
	double 	carTipMargin;
	double 	rollInMargin;
	double 	rollInSpeedFactor;
	bool 	enableHeadingSmoothing;

	double 	rollOutDensity;
	int 	rollOutNumber;
	double 	pathDensity;                // 路径点的密度
	double 	planningDistance;
	double 	microPlanDistance;
	double 	horizonDistance;
    double 	minFollowingDistance;
	double 	smoothingDataWeight;
	double 	smoothingSmoothWeight;
	double 	smoothingToleranceError;
	double  verticalSafetyDistance;
	double  horizontalSafetyDistance;
    double  minObstacleEvaluateDistance;
    bool    enableObjectsPrediction;
	bool 	enableLaneChange;

};

struct NdtMappingProParams
{
    /* data */
    int switchToExp;

    int methodType;
    double voxelLeafSize;
    double stepSize;
    int maxIter;
    double ndtRes;
    double transEps;
    bool isLimitFlat;
    bool isFilterAddMap;
    double filterRes;
    bool isSyncUpdateMap;
    bool isRemapping;
    double minAddShift;
    bool useOdom;
    bool useImu;
    bool useGnss;
    bool useVo;
    bool useVehicle;
    double minScanRange;
    double maxScanRange;
    double minScanHeight;
    double maxScanHeight;
};


class ParamsPanel : public QWidget {
    Q_OBJECT

public:
    explicit ParamsPanel(QWidget *parent = nullptr);

private:

    ros::NodeHandle nh_;
    ros::Publisher confPurePursuitPub_ = nh_.advertise<htcbot_msgs::ConfPurePursuit>("htcbot/pure_pursuit/conf", 1);
    ros::Publisher confEuclideanClusterPub_ = nh_.advertise<htcbot_msgs::ConfEuclideanCluster>("htcbot/euclidean_cluster/conf", 1);
    ros::Publisher confOpLocalPlannerPub_ = nh_.advertise<htcbot_msgs::ConfOpLocalPlanner>("htcbot/op_local_planner/conf", 1);
    ros::Publisher confNdtMappingPub_ = nh_.advertise<htcbot_msgs::ConfNdtMapping>("htcbot/ndt_mapping_pro/conf", 1);


    QTimer *updateTimer;

    QLabel *lookahead_status_label;
    QLabel *euclidean_cluster_status_label;
    QLabel *op_local_planner_status_label;
    QLabel *ndt_mapping_pro_status_label;

    // NDT 建图
    QDialog *ndtMappingProConfDialog;
    NdtMappingProParams *ndt_mapping_pro_params_;

    // 局部路径规划
    QDialog *opLocalPlannerConfDialog;
    OpLocalPlannerParams *op_local_planner_params_;

    // 预瞄配置 
    QDialog *lookaheadConfDialog;
    bool lookahead_switch_to_;
    bool is_const_lookahead_distanceValue_;
    double constLookaheadDistanceValue_;
    double crossLookaheadDistanceValue_;
    double laneLookaheadDistanceValue_;
    double lookaheadDistanceRatioValue_;
    double minimumLookaheadDistanceValue_;

    // 聚类检测
    QDialog *euclideanClusterConfDialog;
    bool euclidean_cluster_switch_to_;
    double remove_points_upto_;
    bool remove_ground_;
    bool use_multiple_thres_;
    bool downsample_cloud_;
    double leaf_size_;
    double clip_min_height_;
    double clip_max_height_;
    bool keep_lanes_;
    double keep_lane_left_distance_;
    double keep_lane_right_distance_;
    int cluster_size_min_;
    int cluster_size_max_;
    double cluster_merge_threshold_;
    double clustering_distance_;

    void openNdtMappingProDialog();

    void openLookaheadConfigDialog();

    void openEuclideanClusterConfigDialog();

    void openOpLocalPlannerConfigDialog();

    void updateSwitchStatus();
    
};

#endif // PARAMS_PANEL_H
