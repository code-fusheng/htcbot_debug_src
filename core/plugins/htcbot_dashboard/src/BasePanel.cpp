#include "BasePanel.h"

BasePanel::BasePanel(QWidget *parent) : QWidget(parent) {

    nh_ = ros::NodeHandle();

    QVBoxLayout *layout = new QVBoxLayout(this);

    QPushButton *test_button = new QPushButton("测试按钮");
    layout->addWidget(test_button);
    connect(test_button, &QPushButton::clicked, this, &BasePanel::openTestDialog);

    // 数据录制
    QPushButton *rosbag_record_button = new QPushButton("数据录制");
    // layout->addWidget(rosbag_record_button);
    connect(rosbag_record_button, &QPushButton::clicked, this, &BasePanel::openRosbagRecordDiglog);

}

void BasePanel::openTestDialog()
{

    // 加载参数
    nh_.getParam("/htcbot/test", testValue_);

    // 创建并显示预瞄配置对话框
    testDialog = new QDialog(); // 传入ros::NodeHandle实例
    testDialog->setWindowTitle("Test");
    // 创建布局
    QFormLayout *form_layout = new QFormLayout(this);

    QLabel *testLabel = new QLabel("Test:");
    QLineEdit *testEdit = new QLineEdit;
    testEdit->setText(QString::number(testValue_));
    form_layout->addRow(testLabel, testEdit);

    // 创建一个lambda表达式用于处理确认操作
    auto onAccepted = [=](){
        testValue_ = testEdit->text().toLong();

        // 在此处根据需要保存或使用这些参数，比如发布到ROS话题
        std_msgs::String test_msg;
        test_msg.data = testValue_;
        testPub_.publish(test_msg);
        // 关闭对话框
        testDialog->accept();
    };

    // 添加确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, testDialog, onAccepted);
    connect(buttonBox, &QDialogButtonBox::rejected, testDialog, &QDialog::reject);

    form_layout->addWidget(buttonBox);

    testDialog->setLayout(form_layout);
    testDialog->show();
}

void BasePanel::openRosbagRecordDiglog() 
{

    recordDialog = new QDialog(); // 传入ros::NodeHandle实例
    recordDialog->setWindowTitle("Rosbag Record");

    start_button_ = new QPushButton("Start Recording");
    stop_button_ = new QPushButton("Stop Recording");
    topic_list_ = new QListWidget();

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(new QLabel("Select topics to record:"));
    layout->addWidget(topic_list_);
  
    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addWidget(start_button_);
    button_layout->addWidget(stop_button_);
    layout->addLayout(button_layout);

    recordDialog->setLayout(layout);
    recordDialog->show();

    auto onStartRecord = [=](){
        // rosbag record /rslidar_points /imu_raw /scan /fix
        std::string cmd = "rosbag record /rslidar_points /imu_raw /scan /fix"; // 你要录制的话题
        int result = system(cmd.c_str());
    };

    auto onStopRecord = [=](){
        // 关闭对话框
        std::string cmd = "pkill -f 'rosbag record'";
        int result = system(cmd.c_str());
        recordDialog->accept();
    };

    // 添加确认和取消按钮
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(start_button_, &QPushButton::clicked, this, onStartRecord);
    connect(stop_button_, &QPushButton::clicked, this, onStopRecord);

}