#include "ParamsPanel.h"

ParamsPanel::ParamsPanel(QWidget *parent) 
    : QWidget(parent), 
    updateTimer(new QTimer(this)),
    op_local_planner_params_(new OpLocalPlannerParams()),
    ndt_mapping_pro_params_(new NdtMappingProParams())
{

    nh_ = ros::NodeHandle();

    nh_.getParam("/pure_pursuit/mode_switch", lookahead_switch_to_);
    nh_.getParam("/euclidean_cluster_node/mode_switch", euclidean_cluster_switch_to_);
    nh_.getParam("/op_common_params_node/modeSwitch", op_local_planner_params_->modeSwitch);
    nh_.getParam("/ndt_mapping_pro_node/switch_to_exp", ndt_mapping_pro_params_->switchToExp);

    QVBoxLayout *layout = new QVBoxLayout(this);

    // NDT Pro 建图
    QHBoxLayout* ndt_mapping_pro_layout = new QHBoxLayout();
    QPushButton* ndt_mapping_pro_button = new QPushButton("NDT+建图");
    ndt_mapping_pro_status_label = new QLabel(QString::number(ndt_mapping_pro_params_->switchToExp));
    ndt_mapping_pro_layout->addWidget(ndt_mapping_pro_button);
    ndt_mapping_pro_layout->addWidget(ndt_mapping_pro_status_label);
    layout->addLayout(ndt_mapping_pro_layout);    

    // 局部规划
    QHBoxLayout* op_local_planner_layout = new QHBoxLayout();
    QPushButton* op_local_planner_button = new QPushButton("局部规划");
    op_local_planner_status_label = new QLabel(op_local_planner_params_->modeSwitch ? "ON" : "OFF");
    op_local_planner_layout->addWidget(op_local_planner_button);
    op_local_planner_layout->addWidget(op_local_planner_status_label);
    layout->addLayout(op_local_planner_layout);

    // 预瞄配置
    QHBoxLayout* lookahead_layout = new QHBoxLayout();
    QPushButton* lookahead_button = new QPushButton("预瞄配置");
    lookahead_status_label = new QLabel(lookahead_switch_to_ ? "ON" : "OFF");
    lookahead_layout->addWidget(lookahead_button);
    lookahead_layout->addWidget(lookahead_status_label);
    layout->addLayout(lookahead_layout);

    // 聚类检测
    QHBoxLayout *euclidean_cluster_layout = new QHBoxLayout();
    QPushButton *euclidean_cluster_button = new QPushButton("聚类检测");
    euclidean_cluster_status_label = new QLabel(euclidean_cluster_switch_to_ ? "ON" : "OFF");
    euclidean_cluster_layout->addWidget(euclidean_cluster_button);
    euclidean_cluster_layout->addWidget(euclidean_cluster_status_label);
    layout->addLayout(euclidean_cluster_layout);

    // 连接按钮信号槽
    connect(ndt_mapping_pro_button, &QPushButton::clicked, this, &ParamsPanel::openNdtMappingProDialog);
    connect(lookahead_button, &QPushButton::clicked, this, &ParamsPanel::openLookaheadConfigDialog);
    connect(euclidean_cluster_button, &QPushButton::clicked, this, &ParamsPanel::openEuclideanClusterConfigDialog);
    connect(op_local_planner_button, &QPushButton::clicked, this, &ParamsPanel::openOpLocalPlannerConfigDialog);

    // 添加定时器，定时更新开关状态
    connect(updateTimer, &QTimer::timeout, this, &ParamsPanel::updateSwitchStatus);
    updateTimer->start(1000); // 每秒更新一次，可根据需要调整间隔时间

}

// 新增槽函数用于定时刷新
void ParamsPanel::updateSwitchStatus()
{
    nh_.getParam("/pure_pursuit/mode_switch", lookahead_switch_to_);
    nh_.getParam("/euclidean_cluster_node/mode_switch", euclidean_cluster_switch_to_);
    nh_.getParam("/op_common_params_node/modeSwitch", op_local_planner_params_->modeSwitch);
    nh_.getParam("/ndt_mapping_pro_node/switch_to_exp", ndt_mapping_pro_params_->switchToExp);

    lookahead_status_label->setText("ON");
    euclidean_cluster_status_label->setText(euclidean_cluster_switch_to_ ? "ON" : "OFF");
    op_local_planner_status_label->setText(op_local_planner_params_->modeSwitch ? "ON" : "OFF");
    ndt_mapping_pro_status_label->setText(QString::number(ndt_mapping_pro_params_->switchToExp));
}

void ParamsPanel::openNdtMappingProDialog() {

    nh_.getParam("/ndt_mapping_pro_node/switch_to_exp", ndt_mapping_pro_params_->switchToExp);
    nh_.getParam("/ndt_mapping_pro_node/voxel_leaf_size", ndt_mapping_pro_params_->voxelLeafSize);
    nh_.getParam("/ndt_mapping_pro_node/method_type", ndt_mapping_pro_params_->methodType);
    nh_.getParam("/ndt_mapping_pro_node/max_iter", ndt_mapping_pro_params_->maxIter);
    nh_.getParam("/ndt_mapping_pro_node/step_size", ndt_mapping_pro_params_->stepSize);
    nh_.getParam("/ndt_mapping_pro_node/ndt_res", ndt_mapping_pro_params_->ndtRes);
    nh_.getParam("/ndt_mapping_pro_node/trans_eps", ndt_mapping_pro_params_->transEps);
    nh_.getParam("/ndt_mapping_pro_node/is_filter_add_map", ndt_mapping_pro_params_->isFilterAddMap);
    nh_.getParam("/ndt_mapping_pro_node/filter_res", ndt_mapping_pro_params_->filterRes);
    nh_.getParam("/ndt_mapping_pro_node/min_add_shift", ndt_mapping_pro_params_->minAddShift);
    nh_.getParam("/ndt_mapping_pro_node/is_limit_flat", ndt_mapping_pro_params_->isLimitFlat);

    nh_.getParam("/ndt_mapping_pro_node/min_scan_range", ndt_mapping_pro_params_->minScanRange);
    nh_.getParam("/ndt_mapping_pro_node/max_scan_range", ndt_mapping_pro_params_->maxScanRange);
    nh_.getParam("/ndt_mapping_pro_node/min_scan_height", ndt_mapping_pro_params_->minScanHeight);
    nh_.getParam("/ndt_mapping_pro_node/max_scan_height", ndt_mapping_pro_params_->maxScanHeight);

    nh_.getParam("/ndt_mapping_pro_node/use_imu", ndt_mapping_pro_params_->useImu);
    nh_.getParam("/ndt_mapping_pro_node/use_odom", ndt_mapping_pro_params_->useOdom);
    nh_.getParam("/ndt_mapping_pro_node/use_gnss", ndt_mapping_pro_params_->useGnss);
    nh_.getParam("/ndt_mapping_pro_node/use_vehicle", ndt_mapping_pro_params_->useVehicle);
    
    ndtMappingProConfDialog = new QDialog(); // 传入ros::NodeHandle实例
    ndtMappingProConfDialog->setWindowTitle("NDT+建图");
    ndtMappingProConfDialog->setFixedWidth(500);
    // 创建布局
    QFormLayout *form_layout = new QFormLayout(this);

    // 创建模块开关输入框
    // QLabel *switchToExpLabel = new QLabel("是否启用:");
    // QCheckBox *switchToExpCheckBox = new QCheckBox;
    // switchToExpCheckBox->setChecked(ndt_mapping_pro_params_->switchToExp);
    // form_layout->addRow(switchToExpLabel, switchToExpCheckBox);

    // 创建方法类型输入框
    QLabel *methodTypeLabel = new QLabel("方法类型:");
    QSpinBox *methodTypeSpin = new QSpinBox;
    methodTypeSpin->setValue(ndt_mapping_pro_params_->methodType);
    form_layout->addRow(methodTypeLabel, methodTypeSpin);

    // 创建双精度浮点数输入框
    QLabel *voxelLeafSizeLabel = new QLabel("体素叶大小:");
    QDoubleSpinBox *voxelLeafSizeSpin = new QDoubleSpinBox;
    voxelLeafSizeSpin->setValue(ndt_mapping_pro_params_->voxelLeafSize);
    form_layout->addRow(voxelLeafSizeLabel, voxelLeafSizeSpin);

    // 继续添加其他参数的输入框和标签
    // 最大迭代次数
    QLabel *maxIterLabel = new QLabel("最大迭代次数:");
    QSpinBox *maxIterSpin = new QSpinBox;
    maxIterSpin->setValue(ndt_mapping_pro_params_->maxIter);
    form_layout->addRow(maxIterLabel, maxIterSpin);

    // 步长大小
    QLabel *stepSizeLabel = new QLabel("步长大小:");
    QDoubleSpinBox *stepSizeSpin = new QDoubleSpinBox;
    stepSizeSpin->setValue(ndt_mapping_pro_params_->stepSize);
    form_layout->addRow(stepSizeLabel, stepSizeSpin);

    // 创建双精度浮点数输入框
    QLabel *ndtResLabel = new QLabel("NDT分辨率:");
    QDoubleSpinBox *ndtResSpin = new QDoubleSpinBox;
    ndtResSpin->setValue(ndt_mapping_pro_params_->ndtRes);
    form_layout->addRow(ndtResLabel, ndtResSpin);

    // 创建双精度浮点数输入框
    QLabel *transEpsLabel = new QLabel("变换容忍度:");
    QDoubleSpinBox *transEpsSpin = new QDoubleSpinBox;
    transEpsSpin->setValue(ndt_mapping_pro_params_->transEps);
    form_layout->addRow(transEpsLabel, transEpsSpin);

    // 创建布尔值输入框
    QLabel *isFilterAddMapLabel = new QLabel("是否过滤添加地图:");
    QCheckBox *isFilterAddMapCheckBox = new QCheckBox;
    isFilterAddMapCheckBox->setChecked(ndt_mapping_pro_params_->isFilterAddMap);
    form_layout->addRow(isFilterAddMapLabel, isFilterAddMapCheckBox);

    // 创建双精度浮点数输入框
    QLabel *filterResLabel = new QLabel("过滤器分辨率:");
    QDoubleSpinBox *filterResSpin = new QDoubleSpinBox;
    filterResSpin->setValue(ndt_mapping_pro_params_->filterRes);
    form_layout->addRow(filterResLabel, filterResSpin);

    QCheckBox *isLimitFlatCheckBox = new QCheckBox;
    isLimitFlatCheckBox->setChecked(ndt_mapping_pro_params_->isLimitFlat);
    form_layout->addRow("是否限制为平面:", isLimitFlatCheckBox);

    // 创建布尔值输入框
    QLabel *isSyncUpdateMapLabel = new QLabel("是否同步更新地图:");
    QCheckBox *isSyncUpdateMapCheckBox = new QCheckBox;
    isSyncUpdateMapCheckBox->setChecked(ndt_mapping_pro_params_->isSyncUpdateMap);
    form_layout->addRow(isSyncUpdateMapLabel, isSyncUpdateMapCheckBox);

    // 创建布尔值输入框
    QLabel *isRemappingLabel = new QLabel("是否重新建图:");
    QCheckBox *isRemappingCheckBox = new QCheckBox;
    isRemappingCheckBox->setChecked(ndt_mapping_pro_params_->isRemapping);
    form_layout->addRow(isRemappingLabel, isRemappingCheckBox);

    // 创建双精度浮点数输入框
    QLabel *minAddShiftLabel = new QLabel("最小更新位移:");
    QDoubleSpinBox *minAddShiftSpin = new QDoubleSpinBox;
    minAddShiftSpin->setValue(ndt_mapping_pro_params_->minAddShift);
    form_layout->addRow(minAddShiftLabel, minAddShiftSpin);

    // 创建布尔值输入框
    QLabel *useOdomLabel = new QLabel("是否使用里程计:");
    QCheckBox *useOdomCheckBox = new QCheckBox;
    useOdomCheckBox->setChecked(ndt_mapping_pro_params_->useOdom);
    form_layout->addRow(useOdomLabel, useOdomCheckBox);

    // 创建布尔值输入框
    QLabel *useImuLabel = new QLabel("是否使用IMU:");
    QCheckBox *useImuCheckBox = new QCheckBox;
    useImuCheckBox->setChecked(ndt_mapping_pro_params_->useImu);
    form_layout->addRow(useImuLabel, useImuCheckBox);

    // 创建布尔值输入框
    QLabel *useGnssLabel = new QLabel("是否使用GNSS:");
    QCheckBox *useGnssCheckBox = new QCheckBox;
    useGnssCheckBox->setChecked(ndt_mapping_pro_params_->useGnss);
    form_layout->addRow(useGnssLabel, useGnssCheckBox);

    // 创建布尔值输入框
    QLabel *useVoLabel = new QLabel("是否使用VO:");
    QCheckBox *useVoCheckBox = new QCheckBox;
    useVoCheckBox->setChecked(ndt_mapping_pro_params_->useVo);
    form_layout->addRow(useVoLabel, useVoCheckBox);

    // 创建是否使用车辆的复选框
    QCheckBox *useVehicleCheckBox = new QCheckBox;
    useVehicleCheckBox->setChecked(ndt_mapping_pro_params_->useVehicle);
    form_layout->addRow("是否使用Vehicle:", useVehicleCheckBox);

    // 创建双精度浮点数输入框
    QLabel *minScanRangeLabel = new QLabel("最小扫描范围:");
    QDoubleSpinBox *minScanRangeSpin = new QDoubleSpinBox;
    minScanRangeSpin->setMinimum(0);
    minScanRangeSpin->setValue(ndt_mapping_pro_params_->minScanRange);
    form_layout->addRow(minScanRangeLabel, minScanRangeSpin);

    // 创建双精度浮点数输入框
    QLabel *maxScanRangeLabel = new QLabel("最大扫描范围:");
    QDoubleSpinBox *maxScanRangeSpin = new QDoubleSpinBox;
    maxScanRangeSpin->setMaximum(999);
    maxScanRangeSpin->setValue(ndt_mapping_pro_params_->maxScanRange);
    form_layout->addRow(maxScanRangeLabel, maxScanRangeSpin);

    // 创建双精度浮点数输入框
    QLabel *minScanHeightLabel = new QLabel("最小扫描高度:");
    QDoubleSpinBox *minScanHeightSpin = new QDoubleSpinBox;
    minScanHeightSpin->setMinimum(-999.0);
    minScanHeightSpin->setValue(ndt_mapping_pro_params_->minScanHeight);
    form_layout->addRow(minScanHeightLabel, minScanHeightSpin);

    // 创建双精度浮点数输入框
    QLabel *maxScanHeightLabel = new QLabel("最大扫描高度:");
    QDoubleSpinBox *maxScanHeightSpin = new QDoubleSpinBox;
    maxScanHeightSpin->setMaximum(999);
    maxScanHeightSpin->setValue(ndt_mapping_pro_params_->maxScanHeight);
    form_layout->addRow(maxScanHeightLabel, maxScanHeightSpin);

    auto onAccepted = [=](){
        // 获取输入框的值并设置到对应的参数中
        ndt_mapping_pro_params_->methodType = methodTypeSpin->value();
        ndt_mapping_pro_params_->voxelLeafSize = voxelLeafSizeSpin->value();
        ndt_mapping_pro_params_->stepSize = stepSizeSpin->value();
        ndt_mapping_pro_params_->maxIter = maxIterSpin->value();
        ndt_mapping_pro_params_->ndtRes = ndtResSpin->value();
        ndt_mapping_pro_params_->transEps = transEpsSpin->value();
        ndt_mapping_pro_params_->isLimitFlat = isLimitFlatCheckBox->isChecked();
        ndt_mapping_pro_params_->isFilterAddMap = isFilterAddMapCheckBox->isChecked();
        ndt_mapping_pro_params_->filterRes = filterResSpin->value();
        ndt_mapping_pro_params_->isSyncUpdateMap = isSyncUpdateMapCheckBox->isChecked();
        ndt_mapping_pro_params_->isRemapping = isRemappingCheckBox->isChecked();
        ndt_mapping_pro_params_->minAddShift = minAddShiftSpin->value();
        ndt_mapping_pro_params_->useOdom = useOdomCheckBox->isChecked();
        ndt_mapping_pro_params_->useImu = useImuCheckBox->isChecked();
        ndt_mapping_pro_params_->useGnss = useGnssCheckBox->isChecked();
        ndt_mapping_pro_params_->useVo = useVoCheckBox->isChecked();
        ndt_mapping_pro_params_->useVehicle = useVehicleCheckBox->isChecked();
        ndt_mapping_pro_params_->minScanRange = minScanRangeSpin->value();
        ndt_mapping_pro_params_->maxScanRange = maxScanRangeSpin->value();
        ndt_mapping_pro_params_->minScanHeight = minScanHeightSpin->value();
        ndt_mapping_pro_params_->maxScanHeight = maxScanHeightSpin->value();

        // 创建消息并设置字段值
        htcbot_msgs::ConfNdtMapping conf_msg;
        conf_msg.method_type = ndt_mapping_pro_params_->methodType;
        conf_msg.voxel_leaf_size = ndt_mapping_pro_params_->voxelLeafSize;
        conf_msg.step_size = ndt_mapping_pro_params_->stepSize;
        conf_msg.max_iter = ndt_mapping_pro_params_->maxIter;
        conf_msg.ndt_res = ndt_mapping_pro_params_->ndtRes;
        conf_msg.trans_eps = ndt_mapping_pro_params_->transEps;
        conf_msg.is_limit_flat = ndt_mapping_pro_params_->isLimitFlat;
        conf_msg.is_filter_add_map = ndt_mapping_pro_params_->isFilterAddMap;
        conf_msg.filter_res = ndt_mapping_pro_params_->filterRes;
        conf_msg.is_sync_update_map = ndt_mapping_pro_params_->isSyncUpdateMap;
        conf_msg.is_remapping = ndt_mapping_pro_params_->isRemapping;
        conf_msg.min_add_shift = ndt_mapping_pro_params_->minAddShift;
        conf_msg.use_odom = ndt_mapping_pro_params_->useOdom;
        conf_msg.use_imu = ndt_mapping_pro_params_->useImu;
        conf_msg.use_gnss = ndt_mapping_pro_params_->useGnss;
        conf_msg.use_vo = ndt_mapping_pro_params_->useVo;
        conf_msg.use_vehicle = ndt_mapping_pro_params_->useVehicle;
        conf_msg.min_scan_range = ndt_mapping_pro_params_->minScanRange;
        conf_msg.max_scan_range = ndt_mapping_pro_params_->maxScanRange;
        conf_msg.min_scan_height = ndt_mapping_pro_params_->minScanHeight;
        conf_msg.max_scan_height = ndt_mapping_pro_params_->maxScanHeight;

        confNdtMappingPub_.publish(conf_msg);

        // 关闭对话框
        ndtMappingProConfDialog->accept();
    };

    // // 添加确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, onAccepted);
    connect(buttonBox, &QDialogButtonBox::rejected, ndtMappingProConfDialog, &QDialog::reject);

    form_layout->addWidget(buttonBox);

    ndtMappingProConfDialog->setLayout(form_layout);
    ndtMappingProConfDialog->show();


}

void ParamsPanel::openLookaheadConfigDialog()
{
    // 加载参数
    nh_.getParam("/pure_pursuit/const_lookahead_distance", constLookaheadDistanceValue_);
    nh_.getParam("/pure_pursuit/lane_lookahead_distance", crossLookaheadDistanceValue_);
    nh_.getParam("/pure_pursuit/cross_lookahead_distance", laneLookaheadDistanceValue_);
    nh_.getParam("/pure_pursuit/lookahead_distance_ratio", lookaheadDistanceRatioValue_);
    nh_.getParam("/pure_pursuit/minimum_lookahead_distance", minimumLookaheadDistanceValue_);

    // 创建并显示预瞄配置对话框
    lookaheadConfDialog = new QDialog(); // 传入ros::NodeHandle实例
    lookaheadConfDialog->setWindowTitle("预瞄配置");
    lookaheadConfDialog->setFixedWidth(500);
    // 创建布局
    QFormLayout *form_layout = new QFormLayout(this);

    // 创建固定预瞄距离的双精度输入框
    QLabel *constLookaheadDistanceLabel = new QLabel("固定预瞄距离(m):");
    QLineEdit *constLookaheadDistanceEdit = new QLineEdit;
    constLookaheadDistanceEdit->setText(QString::number(constLookaheadDistanceValue_));
    form_layout->addRow(constLookaheadDistanceLabel, constLookaheadDistanceEdit);
    // 创建直线瞄距离的双精度输入框
    QLabel *laneLookaheadDistanceLabel = new QLabel("直线预瞄距离(m):");
    QLineEdit *laneLookaheadDistanceEdit = new QLineEdit;
    laneLookaheadDistanceEdit->setText(QString::number(laneLookaheadDistanceValue_));
    form_layout->addRow(laneLookaheadDistanceLabel, laneLookaheadDistanceEdit);
    // 创建路口预瞄距离的双精度输入框
    QLabel *crossLookaheadDistanceLabel = new QLabel("路口预瞄距离(m):");
    QLineEdit *crossLookaheadDistanceEdit = new QLineEdit;
    crossLookaheadDistanceEdit->setText(QString::number(crossLookaheadDistanceValue_));
    form_layout->addRow(crossLookaheadDistanceLabel, crossLookaheadDistanceEdit);

    QLabel *lookaheadDistanceRatioLabel = new QLabel("预瞄距离速度比(d:v):");
    QLineEdit *lookaheadDistanceRatioEdit = new QLineEdit;
    lookaheadDistanceRatioEdit->setText(QString::number(lookaheadDistanceRatioValue_));
    form_layout->addRow(lookaheadDistanceRatioLabel, lookaheadDistanceRatioEdit);

    QLabel *minLookaheadDistanceLabel = new QLabel("最小预瞄距离(m):");
    QLineEdit *minLookaheadDistanceEdit = new QLineEdit;
    minLookaheadDistanceEdit->setText(QString::number(minimumLookaheadDistanceValue_));
    form_layout->addRow(minLookaheadDistanceLabel, minLookaheadDistanceEdit);

    // 创建一个lambda表达式用于处理确认操作
    auto onAccepted = [=](){
        constLookaheadDistanceValue_ = constLookaheadDistanceEdit->text().toDouble();
        crossLookaheadDistanceValue_ = crossLookaheadDistanceEdit->text().toDouble();
        laneLookaheadDistanceValue_ = laneLookaheadDistanceEdit->text().toDouble();
        lookaheadDistanceRatioValue_ = lookaheadDistanceRatioEdit->text().toDouble();
        minimumLookaheadDistanceValue_ = minLookaheadDistanceEdit->text().toDouble();

        // 在此处根据需要保存或使用这些参数，比如发布到ROS话题
        htcbot_msgs::ConfPurePursuit conf_msg;
        conf_msg.is_const_lookahead_distance = false;
        conf_msg.const_lookahead_distance = constLookaheadDistanceValue_;
        conf_msg.lane_lookahead_distance = laneLookaheadDistanceValue_;
        conf_msg.cross_lookahead_distance = crossLookaheadDistanceValue_;
        conf_msg.lookahead_distance_ratio = lookaheadDistanceRatioValue_;
        conf_msg.minimum_lookahead_distance = minimumLookaheadDistanceValue_;
        confPurePursuitPub_.publish(conf_msg);
        // 关闭对话框
        lookaheadConfDialog->accept();
    };

    // 添加确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, lookaheadConfDialog, onAccepted);
    connect(buttonBox, &QDialogButtonBox::rejected, lookaheadConfDialog, &QDialog::reject);

    form_layout->addWidget(buttonBox);

    lookaheadConfDialog->setLayout(form_layout);
    lookaheadConfDialog->show();
}

void ParamsPanel::openEuclideanClusterConfigDialog() {
    
    // 加载参数
    nh_.getParam("/euclidean_cluster_node/mode_switch", euclidean_cluster_switch_to_);
    nh_.getParam("/euclidean_cluster_node/remove_points_upto", remove_points_upto_);
    nh_.getParam("/euclidean_cluster_node/remove_ground", remove_ground_);
    nh_.getParam("/euclidean_cluster_node/use_multiple_thres", use_multiple_thres_);
    nh_.getParam("/euclidean_cluster_node/downsample_cloud", downsample_cloud_);
    nh_.getParam("/euclidean_cluster_node/leaf_size", leaf_size_);
    nh_.getParam("/euclidean_cluster_node/clip_min_height", clip_min_height_);
    nh_.getParam("/euclidean_cluster_node/clip_max_height", clip_max_height_);
    nh_.getParam("/euclidean_cluster_node/keep_lanes", keep_lanes_);
    nh_.getParam("/euclidean_cluster_node/keep_lane_left_distance", keep_lane_left_distance_);
    nh_.getParam("/euclidean_cluster_node/keep_lane_right_distance", keep_lane_right_distance_);
    nh_.getParam("/euclidean_cluster_node/cluster_size_min", cluster_size_min_);
    nh_.getParam("/euclidean_cluster_node/cluster_size_max", cluster_size_max_);
    nh_.getParam("/euclidean_cluster_node/cluster_merge_threshold", cluster_merge_threshold_);
    nh_.getParam("/euclidean_cluster_node/clustering_distance", clustering_distance_);

    // 创建并显示聚类检测配置对话框
    euclideanClusterConfDialog = new QDialog(); // 传入ros::NodeHandle实例
    euclideanClusterConfDialog->setWindowTitle("聚类检测");
    euclideanClusterConfDialog->setFixedWidth(500);
    // 创建布局
    QFormLayout *form_layout = new QFormLayout(euclideanClusterConfDialog);

    // 创建模块开关输入框
    QLabel *modeSwitchLabel = new QLabel("是否启用:");
    QCheckBox *modeSwitchCheckBox = new QCheckBox;
    modeSwitchCheckBox->setChecked(euclidean_cluster_switch_to_);
    form_layout->addRow(modeSwitchLabel, modeSwitchCheckBox);

    // 创建移除点参数输入框
    QLabel *removePointsLabel = new QLabel("移除点直到距离(m):");
    QLineEdit *removePointsEdit = new QLineEdit;
    removePointsEdit->setText(QString::number(remove_points_upto_));
    form_layout->addRow(removePointsLabel, removePointsEdit);

    // 创建移除地面参数输入框
    QLabel *removeGroundLabel = new QLabel("移除地面:");
    QCheckBox *removeGroundCheckBox = new QCheckBox;
    removeGroundCheckBox->setChecked(remove_ground_);
    form_layout->addRow(removeGroundLabel, removeGroundCheckBox);

    // 创建使用多阈值参数输入框
    QLabel *useMultipleThresLabel = new QLabel("使用多阈值:");
    QCheckBox *useMultipleThresCheckBox = new QCheckBox;
    useMultipleThresCheckBox->setChecked(use_multiple_thres_);
    form_layout->addRow(useMultipleThresLabel, useMultipleThresCheckBox);

    // 创建下采样参数输入框
    QLabel *downsampleCloudLabel = new QLabel("下采样点云:");
    QCheckBox *downsampleCloudCheckBox = new QCheckBox;
    downsampleCloudCheckBox->setChecked(downsample_cloud_);
    form_layout->addRow(downsampleCloudLabel, downsampleCloudCheckBox);

    // 创建叶子大小参数输入框
    QLabel *leafSizeLabel = new QLabel("叶子大小:");
    QLineEdit *leafSizeEdit = new QLineEdit;
    leafSizeEdit->setText(QString::number(leaf_size_));
    form_layout->addRow(leafSizeLabel, leafSizeEdit);

    // 创建高度剪裁参数输入框
    QLabel *clipMinHeightLabel = new QLabel("剪裁最小高度(m):");
    QLineEdit *clipMinHeightEdit = new QLineEdit;
    clipMinHeightEdit->setText(QString::number(clip_min_height_));
    form_layout->addRow(clipMinHeightLabel, clipMinHeightEdit);

    QLabel *clipMaxHeightLabel = new QLabel("剪裁最大高度(m):");
    QLineEdit *clipMaxHeightEdit = new QLineEdit;
    clipMaxHeightEdit->setText(QString::number(clip_max_height_));
    form_layout->addRow(clipMaxHeightLabel, clipMaxHeightEdit);

    // 创建保持车道参数输入框
    QLabel *keepLanesLabel = new QLabel("保持车道:");
    QCheckBox *keepLanesCheckBox = new QCheckBox;
    keepLanesCheckBox->setChecked(keep_lanes_);
    form_layout->addRow(keepLanesLabel, keepLanesCheckBox);

    QLabel *keepLaneLeftDistanceLabel = new QLabel("左侧车道保持距离(m):");
    QLineEdit *keepLaneLeftDistanceEdit = new QLineEdit;
    keepLaneLeftDistanceEdit->setText(QString::number(keep_lane_left_distance_));
    form_layout->addRow(keepLaneLeftDistanceLabel, keepLaneLeftDistanceEdit);

    QLabel *keepLaneRightDistanceLabel = new QLabel("右侧车道保持距离(m):");
    QLineEdit *keepLaneRightDistanceEdit = new QLineEdit;
    keepLaneRightDistanceEdit->setText(QString::number(keep_lane_right_distance_));
    form_layout->addRow(keepLaneRightDistanceLabel, keepLaneRightDistanceEdit);

    // 创建聚类最小和最大尺寸参数输入框
    QLabel *clusterSizeMinLabel = new QLabel("聚类最小尺寸:");
    QLineEdit *clusterSizeMinEdit = new QLineEdit;
    clusterSizeMinEdit->setText(QString::number(cluster_size_min_));
    form_layout->addRow(clusterSizeMinLabel, clusterSizeMinEdit);

    QLabel *clusterSizeMaxLabel = new QLabel("聚类最大尺寸:");
    QLineEdit *clusterSizeMaxEdit = new QLineEdit;
    clusterSizeMaxEdit->setText(QString::number(cluster_size_max_));
    form_layout->addRow(clusterSizeMaxLabel, clusterSizeMaxEdit);

    // 创建聚类合并阈值参数输入框
    QLabel *clusterMergeThresholdLabel = new QLabel("聚类合并阈值:");
    QLineEdit *clusterMergeThresholdEdit = new QLineEdit;
    clusterMergeThresholdEdit->setText(QString::number(cluster_merge_threshold_));
    form_layout->addRow(clusterMergeThresholdLabel, clusterMergeThresholdEdit);

    // 创建聚类距离参数输入框
    QLabel *clusteringDistanceLabel = new QLabel("聚类距离:");
    QLineEdit *clusteringDistanceEdit = new QLineEdit;
    clusteringDistanceEdit->setText(QString::number(clustering_distance_));
    form_layout->addRow(clusteringDistanceLabel, clusteringDistanceEdit);

    // 创建确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, euclideanClusterConfDialog, [=]() {
        // 更新参数
        euclidean_cluster_switch_to_ = modeSwitchCheckBox->isChecked();
        remove_points_upto_ = removePointsEdit->text().toDouble();
        remove_ground_ = removeGroundCheckBox->isChecked();
        use_multiple_thres_ = useMultipleThresCheckBox->isChecked();
        downsample_cloud_ = downsampleCloudCheckBox->isChecked();
        leaf_size_ = leafSizeEdit->text().toDouble();
        clip_min_height_ = clipMinHeightEdit->text().toDouble();
        clip_max_height_ = clipMaxHeightEdit->text().toDouble();
        keep_lanes_ = keepLanesCheckBox->isChecked();
        keep_lane_left_distance_ = keepLaneLeftDistanceEdit->text().toDouble();
        keep_lane_right_distance_ = keepLaneRightDistanceEdit->text().toDouble();
        cluster_size_min_ = clusterSizeMinEdit->text().toInt();
        cluster_size_max_ = clusterSizeMaxEdit->text().toInt();
        cluster_merge_threshold_ = clusterMergeThresholdEdit->text().toDouble();
        clustering_distance_ = clusteringDistanceEdit->text().toDouble();

        // 在此处根据需要保存或使用这些参数，比如发布到ROS话题
        htcbot_msgs::ConfEuclideanCluster conf_msg;
        conf_msg.switch_to = euclidean_cluster_switch_to_;
        conf_msg.remove_points_upto = remove_points_upto_;
        conf_msg.remove_ground = remove_ground_;
        conf_msg.use_multiple_thres = use_multiple_thres_;
        conf_msg.downsample_cloud = downsample_cloud_;
        conf_msg.leaf_size = leaf_size_;
        conf_msg.clip_min_height = clip_min_height_;
        conf_msg.clip_max_height = clip_max_height_;
        conf_msg.keep_lanes = keep_lanes_;
        conf_msg.keep_lane_left_distance = keep_lane_left_distance_;
        conf_msg.keep_lane_right_distance = keep_lane_right_distance_;
        conf_msg.cluster_size_min = cluster_size_min_;
        conf_msg.cluster_size_max = cluster_size_max_;
        conf_msg.cluster_merge_threshold = cluster_merge_threshold_;
        conf_msg.clustering_distance = clustering_distance_;
        confEuclideanClusterPub_.publish(conf_msg);
        // 发布到ROS话题...

        // 关闭对话框
        euclideanClusterConfDialog->accept();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, euclideanClusterConfDialog, &QDialog::reject);

    form_layout->addWidget(buttonBox);

    euclideanClusterConfDialog->setLayout(form_layout);
    euclideanClusterConfDialog->show();
}

void ParamsPanel::openOpLocalPlannerConfigDialog() {

    // 加载参数
    nh_.getParam("/op_common_params_node/modeSwitch", op_local_planner_params_->modeSwitch);
    nh_.getParam("/op_common_params_node/rollOutNumber", op_local_planner_params_->rollOutNumber);
    nh_.getParam("/op_common_params_node/rollOutDensity", op_local_planner_params_->rollOutDensity);
    nh_.getParam("/op_common_params_node/pathDensity", op_local_planner_params_->pathDensity);
    nh_.getParam("/op_common_params_node/planningDistance", op_local_planner_params_->planningDistance);
    nh_.getParam("/op_common_params_node/microPlanDistance", op_local_planner_params_->microPlanDistance);
    nh_.getParam("/op_common_params_node/horizonDistance", op_local_planner_params_->horizonDistance);
    nh_.getParam("/op_common_params_node/minFollowingDistance", op_local_planner_params_->minFollowingDistance);
    nh_.getParam("/op_common_params_node/smoothingDataWeight", op_local_planner_params_->smoothingDataWeight);
    nh_.getParam("/op_common_params_node/smoothingSmoothWeight", op_local_planner_params_->smoothingSmoothWeight);
    nh_.getParam("/op_common_params_node/smoothingToleranceError", op_local_planner_params_->smoothingToleranceError);
    nh_.getParam("/op_common_params_node/verticalSafetyDistance", op_local_planner_params_->verticalSafetyDistance);
    nh_.getParam("/op_common_params_node/horizontalSafetyDistance", op_local_planner_params_->horizontalSafetyDistance);
    nh_.getParam("/op_common_params_node/enableObjectsPrediction", op_local_planner_params_->enableObjectsPrediction);

    nh_.getParam("/op_trajectory_generator_node/carTipMargin", op_local_planner_params_->carTipMargin);
    nh_.getParam("/op_trajectory_generator_node/rollInMargin", op_local_planner_params_->rollInMargin);
    nh_.getParam("/op_trajectory_generator_node/rollInSpeedFactor", op_local_planner_params_->rollInSpeedFactor);
    nh_.getParam("/op_trajectory_generator_node/enableHeadingSmoothing", op_local_planner_params_->enableHeadingSmoothing);

    nh_.getParam("/op_trajectory_evaluator_node/minObstacleEvaluateDistance", op_local_planner_params_->minObstacleEvaluateDistance);

    opLocalPlannerConfDialog = new QDialog();
    opLocalPlannerConfDialog->setWindowTitle("局部规划");
    opLocalPlannerConfDialog->setFixedWidth(500);
    QFormLayout *form_layout = new QFormLayout(this);

    // 创建开关状态的复选框
    QCheckBox *switchToCheckBox = new QCheckBox;
    switchToCheckBox->setChecked(op_local_planner_params_->modeSwitch);
    form_layout->addRow("开关状态:", switchToCheckBox);

    // 创建各个浮点数输入框
    // QLineEdit *samplingTipMarginEdit = new QLineEdit;
    // samplingTipMarginEdit->setText(QString::number(op_local_planner_params_->carTipMargin));
    // form_layout->addRow("中心点到水平采样起点距离(m):", samplingTipMarginEdit);

    QSlider *samplingTipMarginSlider = new QSlider(Qt::Horizontal);
    samplingTipMarginSlider->setRange(0, 100); // 设置滑块的最小值和最大值
    samplingTipMarginSlider->setSingleStep(0.5);
    samplingTipMarginSlider->setValue(op_local_planner_params_->carTipMargin); // 设置滑块的初始值
    QLineEdit *samplingTipMarginEdit = new QLineEdit;
    samplingTipMarginEdit->setText(QString::number(op_local_planner_params_->carTipMargin));
    samplingTipMarginEdit->setFixedWidth(50);
    // 连接滑块值变化信号到标签更新槽函数
    connect(samplingTipMarginSlider, &QSlider::valueChanged, [=](int value) {
        samplingTipMarginEdit->setText(QString::number(value));
    });
    // 将滑块和文本编辑框添加到表单布局中的同一行
    QHBoxLayout *samplingTipMarginLayout = new QHBoxLayout;
    samplingTipMarginLayout->addWidget(samplingTipMarginSlider);
    samplingTipMarginLayout->addWidget(samplingTipMarginEdit);
    form_layout->addRow("中心点到水平点距离(m):", samplingTipMarginLayout);

    QLineEdit *samplingOutMarginEdit = new QLineEdit;
    samplingOutMarginEdit->setText(QString::number(op_local_planner_params_->rollInMargin));
    form_layout->addRow("水平点到平行点距离(m):", samplingOutMarginEdit);

    QLineEdit *samplingSpeedFactorEdit = new QLineEdit;
    samplingSpeedFactorEdit->setText(QString::number(op_local_planner_params_->rollInSpeedFactor));
    form_layout->addRow("水平速度系数:", samplingSpeedFactorEdit);

    QCheckBox *enableHeadingSmoothingCheckBox = new QCheckBox;
    enableHeadingSmoothingCheckBox->setChecked(op_local_planner_params_->enableHeadingSmoothing);
    form_layout->addRow("航向平滑:", enableHeadingSmoothingCheckBox);

    QLineEdit *maxLocalPlanDistanceEdit = new QLineEdit;
    maxLocalPlanDistanceEdit->setText(QString::number(op_local_planner_params_->microPlanDistance));
    form_layout->addRow("最远规划距离(m):", maxLocalPlanDistanceEdit);

    QLineEdit *horizonDistanceEdit = new QLineEdit;
    horizonDistanceEdit->setText(QString::number(op_local_planner_params_->horizonDistance));
    // form_layout->addRow("路径规划时前方距离(m):", horizonDistanceEdit);

    QLineEdit *rolloutsNumberEdit = new QLineEdit;
    rolloutsNumberEdit->setText(QString::number(op_local_planner_params_->rollOutNumber));
    form_layout->addRow("生成轨迹数:", rolloutsNumberEdit);

    QLineEdit *rolloutsDensityEdit = new QLineEdit;
    rolloutsDensityEdit->setText(QString::number(op_local_planner_params_->rollOutDensity));
    form_layout->addRow("轨迹间距(m):", rolloutsDensityEdit);

    QLineEdit *pathDensityEdit = new QLineEdit;
    pathDensityEdit->setText(QString::number(op_local_planner_params_->pathDensity));
    form_layout->addRow("路径点间距(m):", pathDensityEdit);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken); // 可以根据需要设置阴影效果

    // 将分割线添加到表单布局中
    form_layout->addRow(" ", line);

    QCheckBox *enableLaneChangeCheckBox = new QCheckBox;
    enableLaneChangeCheckBox->setChecked(op_local_planner_params_->enableLaneChange);
    form_layout->addRow("车道变更:", enableLaneChangeCheckBox);

    QLineEdit *horizontalSafetyDistanceEdit = new QLineEdit;
    horizontalSafetyDistanceEdit->setText(QString::number(op_local_planner_params_->horizontalSafetyDistance));
    form_layout->addRow("横向安全距离(m):", horizontalSafetyDistanceEdit);

    QLineEdit *verticalSafetyDistanceEdit = new QLineEdit;
    verticalSafetyDistanceEdit->setText(QString::number(op_local_planner_params_->verticalSafetyDistance));
    form_layout->addRow("纵向安全距离(m):", verticalSafetyDistanceEdit);

    QLineEdit *minFollowDistanceEdit = new QLineEdit;
    minFollowDistanceEdit->setText(QString::number(op_local_planner_params_->minFollowingDistance));
    form_layout->addRow("最小跟车距离(m):", minFollowDistanceEdit);

    QLineEdit *minObstacleEvaluateDistanceEdit = new QLineEdit;
    minObstacleEvaluateDistanceEdit->setText(QString::number(op_local_planner_params_->minObstacleEvaluateDistance));
    form_layout->addRow("障碍物评估最小距离(m):", minObstacleEvaluateDistanceEdit);

    // 创建一个lambda表达式用于处理确认操作
    auto onAccepted = [=](){

        op_local_planner_params_->modeSwitch = switchToCheckBox->isChecked();
        op_local_planner_params_->carTipMargin = samplingTipMarginEdit->text().toDouble();
        op_local_planner_params_->rollInMargin = samplingOutMarginEdit->text().toDouble();
        op_local_planner_params_->rollInSpeedFactor = samplingSpeedFactorEdit->text().toDouble();
        op_local_planner_params_->enableHeadingSmoothing = enableHeadingSmoothingCheckBox->isChecked();
        op_local_planner_params_->enableLaneChange = enableLaneChangeCheckBox->isChecked();
        op_local_planner_params_->microPlanDistance = maxLocalPlanDistanceEdit->text().toDouble();
        op_local_planner_params_->horizonDistance = horizonDistanceEdit->text().toDouble();
        op_local_planner_params_->rollOutNumber = rolloutsNumberEdit->text().toInt();
        op_local_planner_params_->rollOutDensity = rolloutsDensityEdit->text().toDouble();
        op_local_planner_params_->horizontalSafetyDistance = horizontalSafetyDistanceEdit->text().toDouble();
        op_local_planner_params_->verticalSafetyDistance = verticalSafetyDistanceEdit->text().toDouble();
        op_local_planner_params_->minFollowingDistance = minFollowDistanceEdit->text().toDouble();
        op_local_planner_params_->pathDensity = pathDensityEdit->text().toDouble();
        op_local_planner_params_->minObstacleEvaluateDistance = minObstacleEvaluateDistanceEdit->text().toDouble();

        htcbot_msgs::ConfOpLocalPlanner conf_msg;
        conf_msg.switch_to = op_local_planner_params_->modeSwitch;
        conf_msg.sampling_tip_margin = op_local_planner_params_->carTipMargin;
        conf_msg.sampling_out_margin = op_local_planner_params_->rollInMargin;
        conf_msg.sampling_speed_factor = op_local_planner_params_->rollInSpeedFactor;
        conf_msg.enable_heading_smoothing = op_local_planner_params_->enableHeadingSmoothing;
        conf_msg.enable_lane_change = op_local_planner_params_->enableLaneChange;
        conf_msg.max_local_plan_distance = op_local_planner_params_->microPlanDistance;
        conf_msg.horizon_distance = op_local_planner_params_->horizonDistance;
        conf_msg.rollouts_number = op_local_planner_params_->rollOutNumber;
        conf_msg.rollouts_density = op_local_planner_params_->rollOutDensity;
        conf_msg.horizontal_safety_distance = op_local_planner_params_->horizontalSafetyDistance;
        conf_msg.vertical_safety_distance = op_local_planner_params_->verticalSafetyDistance;
        conf_msg.min_follow_distance = op_local_planner_params_->minFollowingDistance;
        conf_msg.path_density = op_local_planner_params_->pathDensity;
        conf_msg.min_obstacle_evaluate_distance = op_local_planner_params_->minObstacleEvaluateDistance;

        confOpLocalPlannerPub_.publish(conf_msg);
        opLocalPlannerConfDialog->accept();
    };

    // 添加确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, opLocalPlannerConfDialog, onAccepted);
    connect(buttonBox, &QDialogButtonBox::rejected, opLocalPlannerConfDialog, &QDialog::reject);

    form_layout->addWidget(buttonBox);

    opLocalPlannerConfDialog->setLayout(form_layout);
    opLocalPlannerConfDialog->show();

}

