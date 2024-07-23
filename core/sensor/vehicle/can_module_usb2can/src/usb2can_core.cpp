#include "usb2can_core.h"
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>

namespace USB2CAN
{
void *receive_func(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    std::string name = param->name;
    VCI_CAN_OBJ rec[3000]; // 接收缓存
    int reclen = 0;        // 获取收到的数据的帧数

    ros::NodeHandle nh_t;
    std::string battery_topic_;
    std::string vehicle_status_topic_;
    nh_t.param<std::string>("battery_topic", battery_topic_, "/battery");
    nh_t.param<std::string>("vehicle_status_topic", vehicle_status_topic_, "/vehicle_status");

    ros::Publisher pub_battery_status_ = param->battery_status_publisher;
    ros::Publisher pub_vehicle_sattus_ = param->vehicle_status_publisher;
    ros::Publisher pub_vehicle_status_low_rate = param->vehicle_status_low_rate_publisher;

    while (mrun_num & 0x0f)
    {
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, param->main_can_port, rec, 3000, 100)) > 0) //设备类型,设备索引,can通道索引,接收缓存索引,接收缓存大小,WaitTime(保留参数)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (rec[i].ID == 0x51){            // vehicle status
                    VehicleStatusMsg msg_factory(rec[i]);
                    can_msgs::vehicle_status msg_vehicle_status = msg_factory.getMessage();
                    msg_vehicle_status.Header.stamp = ros::Time::now();
                    msg_vehicle_status.left_wheel_speed = left_wheel_speed;
                    msg_vehicle_status.right_wheel_speed = right_wheel_speed;
                    pub_vehicle_sattus_.publish(msg_vehicle_status);
                }else if (rec[i].ID == 0x17904001){ // battery status
                    BatteryMsg msg_factory(rec[i]);
                    can_msgs::battery msg_battery_status = msg_factory.getMessage();
                    pub_battery_status_.publish(msg_battery_status);
                }else if(rec[i].ID == 0x178) {      // 左轮转速
                    WheelStatus msg_factory(rec[i], WHEEL_LEFT);
                    left_wheel_speed = msg_factory.get_rpm()*wheel_radius*2*3.141592654/60;
                }else if(rec[i].ID == 0x188) {      // 右轮转速
                    WheelStatus msg_factory(rec[i], WHEEL_RIGHT);
                    right_wheel_speed = msg_factory.get_rpm()*wheel_radius*2*3.141592654/60;
                }
            }
        }
    }
    printf("[can_module] Listener thread exit.\n");
    pthread_exit(0);
}

void *thread_debug(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    VCI_CAN_OBJ recv[3000]; // 接收缓存
    int reclen = 0;         // 获取收到的数据的帧数

    // 初始化CAN设备
    usleep(100000);
    if (VCI_InitCAN(VCI_USBCAN2, CAN_id, param->debug_can_port, &config) != 1) // params: 设备类型, 设备索引, can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Init CAN-" << param->debug_can_port + 1);
        VCI_CloseDevice(VCI_USBCAN2, CAN_id);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-" << param->debug_can_port + 1);
    }
    // 启动CAN设备, CAN_id = 0为默认,main_can_port = 0 对应CAN1
    usleep(100000);
    if (VCI_StartCAN(VCI_USBCAN2, CAN_id, param->debug_can_port) != 1) // params: 设备类型,设备索引,can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Start CAN-" << param->debug_can_port + 1);
        if (VCI_ResetCAN(VCI_USBCAN2, CAN_id, param->debug_can_port) != 1)
        {
            ROS_ERROR_STREAM("[can_module] Try to reset can but failed, CAN-: " << param->debug_can_port + 1);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            exit(1);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] reset can success");
        }
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-" << param->debug_can_port + 1);
    }
    while (mrun_num & 0x0f)
    {
        reclen = VCI_Receive(VCI_USBCAN2, 0, param->debug_can_port, recv, 3000, 100);
        if (reclen > 0)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (recv[i].ID == 0x51) // vehicle status
                {
                    VehicleStatusMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
                else if (recv[i].ID == 0x2AA) // battery status
                {
                    BatteryMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
                else if (recv[i].ID == 0x120)
                {
                    fprintf(stdout, "Recv ecu Msg->");
                    SendMsgBase msg_factory(recv[i]);
                    msg_factory.print();
                }
            }
        }
    }
    printf("[can_module] Listener-1 thread exit.\n"); //退出接收线程
    pthread_exit(0);
}

CAN_app::CAN_app() : p_nh("~")
{
    // initROS();
}

CAN_app::~CAN_app()
{
    switch (can_type)
    {
    case USB2CAN:
    {
        printf("[can module] try to ResetCAN...");
        usleep(100000);                                   //延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, CAN_id, main_can_port); //复位CAN通道。
        printf("[can module] ResetCan success");
        printf("[can module] try to CloseCAN...");
        usleep(100000);                  //延时100ms。
        VCI_CloseDevice(VCI_USBCAN2, 0); //关闭设备。
        printf("[can module] CloseCAN success! done");
        break;
    }
    case EthCAN_UDP:
    {
        for (std::thread& t: this->tasks) {
            t.join();
        }
        break;
    }
    default:
        break;
    }
}

// void CAN_app::initROS()
// {
//     p_nh.param<bool>("debug_mode", param_debug, false);
//     p_nh.param<int>("debug_can_id", debug_can_port, 1); // 默认读取can1数据来debug
//     p_nh.param<int>("main_can_id", main_can_port, 2);
//     p_nh.param<bool>("show_sending_msg", param_show_sending_msg, false);
//     std::string car_type_str;
//     p_nh.param<std::string>("car_type", car_type_str, "JD03");
//     convertStringToType(car_type_str);

//     main_can_port = main_can_port - 1;
//     debug_can_port = debug_can_port - 1;

//     sub_ecu = nh.subscribe("ecu", 10, &CAN_app::ecu_cb, this);
//     pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 100);
//     pub_battery_status = nh.advertise<can_msgs::battery>("battery_status", 100);

//     pre_steer = 0.;
// }

void CAN_app::convertStringToType(std::string str){
    if(str == "NWD"){
        car_type = NWD;
    }else if(str == "JD03"){
        car_type = JD03;
    }else if(str == "JD01"){
        car_type = JD01;
    }else if(str == "JD02"){
        car_type = JD02;
    }
}

void CAN_app::ecu_cb(const can_msgs::ecu::ConstPtr &msg)
{
    can_msgs::ecu _msg = *msg;
    // _msg.steer = _msg.steer * 3.14 / 180.0;
    // _msg.steer -= 13;
    //SendMsg sendMsg(_msg, pre_steer);
    SendMsgBase* sendMsg = nullptr;
    switch (car_type)
    {
    case NWD:
        sendMsg = new MsgNWD(_msg, pre_steer);
        break;
    case JD02:
        sendMsg = new MsgJD03(_msg, pre_steer);
        wheel_radius = 0.0505;  // 单位m
        break;
    case JD03:
        sendMsg = new MsgJD03(_msg, pre_steer);
        wheel_radius = 0.065;
        break;
    case JD01:
        sendMsg = new MsgJD01(_msg, pre_steer);
        break;
    default:
        ROS_WARN_STREAM("This driver only supports NWD/JD01/JD02/JD03. Confirm your vehicle type.");
        return;
    }

    // if(car_type == NWD){
    //     sendMsg = new MsgNWD(_msg, pre_steer);
    // }else if(car_type == JD03){
    //     sendMsg = new MsgJD03(_msg, pre_steer);
    // }else{
    //     ROS_WARN_STREAM("This driver only supports NWD series and JD03. Confirm your vehicle type.");
    //     return;
    // }

    pre_steer = _msg.steer;
    //VCI_CAN_OBJ sendData = sendMsg.getMessage();
    VCI_CAN_OBJ sendData = sendMsg->getMessage();
    // ROS_INFO_STREAM("set wheel angle to can, data = " << _msg.steer * 10);

    switch (can_type)
    {
    case USB2CAN:
        if (VCI_Transmit(VCI_USBCAN2, CAN_id, main_can_port, &sendData, 1) != 1)
        {
            // ROS_INFO("send can id: %02x", CAN_id);
            ROS_WARN_STREAM("[can_module] Transmit failed once.");
            ROS_INFO_STREAM("VCI_USBCAN2 : " << VCI_USBCAN2);
            ROS_INFO_STREAM("CAN_id     : " << CAN_id);
            ROS_INFO_STREAM("CAN_port   : " << main_can_port + 1);
            ROS_INFO_STREAM("sendData : ");
            // sendMsg.print(); // 以十六进制的形式打印ecu->can消息
        }
        break;
    case EthCAN_UDP:
    {
        // BYTE *d;
        // ROS_INFO("1");
        // memset(d, 0, sizeof(BYTE)*13);
        // ROS_INFO("2");
        BYTE d[13];
        d[0] = (BYTE)sendData.DataLen;
        d[1] = (BYTE)((0xff000000 & sendData.ID) >> 24);
        d[2] = (BYTE)((0xff0000 & sendData.ID) >> 16);
        d[3] = (BYTE)((0xff00 & sendData.ID) >> 8);
        d[4] = (BYTE)((0xff & sendData.ID));
        for (int i = 0; i < sendData.DataLen; i++) {
            d[5+i] = sendData.Data[i];
        }
        int send_num = sendto(sock_fd, d, 13, 0, (struct sockaddr *)&can2_addr_serv, can2_addr_serv_len);
        // ROS_INFO("send to ip:%s", can2_remote_ip.c_str()); //, can2_addr_serv.sin_addr.s_addr, can2_addr_serv.sin_port);
        if ( send_num < 0 ) {
            ROS_WARN("[CAN_DRIVER] Send data failed, res: %d", send_num);
        }
        break;
    }
    default:
        break;
    }

    if (param_show_sending_msg)
    {
        sendMsg->print();
    }
}

void CAN_app::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu _msg = *msg;

    SendPostureMsg* sendMsg = new SendPostureMsg(_msg, vehicle_weight);
    // sendMsg->print();
    VCI_CAN_OBJ sendData = sendMsg->getMessage();

    // // debug
    // double roll, pitch, yaw;
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(msg->orientation, quat);
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // roll = roll * 180/M_PI;
    // pitch = pitch * 180/M_PI;
    // yaw = yaw * 180/M_PI;
    // ROS_INFO("send angle: %d", int(-pitch)*10);  // IMU右手系, x前y左z上, 上坡时计算得到pitch为负数(TODO::换IMU验证). 对于vcu来讲, 上坡发送坡度为正值.

    switch (can_type)
    {
    case USB2CAN:
        if (VCI_Transmit(VCI_USBCAN2, CAN_id, main_can_port, &sendData, 1) != 1)
        {
            // ROS_INFO("send can id: %02x", CAN_id);
            ROS_WARN_STREAM("[can_module] Transmit failed once.");
            ROS_INFO_STREAM("VCI_USBCAN2 : " << VCI_USBCAN2);
            ROS_INFO_STREAM("CAN_id     : " << CAN_id);
            ROS_INFO_STREAM("CAN_port   : " << main_can_port + 1);
            ROS_INFO_STREAM("sendData : ");
            // sendMsg.print(); // 以十六进制的形式打印ecu->can消息
        }
        break;
    case EthCAN_UDP:
    {
        // BYTE *d;
        // ROS_INFO("1");
        // memset(d, 0, sizeof(BYTE)*13);
        // ROS_INFO("2");
        BYTE d[13];
        d[0] = (BYTE)sendData.DataLen;
        d[1] = (BYTE)((0xff000000 & sendData.ID) >> 24);
        d[2] = (BYTE)((0xff0000 & sendData.ID) >> 16);
        d[3] = (BYTE)((0xff00 & sendData.ID) >> 8);
        d[4] = (BYTE)((0xff & sendData.ID));
        for (int i = 0; i < sendData.DataLen; i++) {
            d[5+i] = sendData.Data[i];
        }
        int send_num = sendto(sock_fd, d, 13, 0, (struct sockaddr *)&can2_addr_serv, can2_addr_serv_len);
        // ROS_INFO("send to ip:%s", can2_remote_ip.c_str()); //, can2_addr_serv.sin_addr.s_addr, can2_addr_serv.sin_port);
        if ( send_num < 0 ) {
            ROS_WARN("[CAN_DRIVER] Send data failed, res: %d", send_num);
        }
        break;
    }
    default:
        break;
    }

    if (param_show_sending_msg)
    {
        sendMsg->print();
    }
}

class ThreadGuard
{
private:
    pthread_t &t1_;
    Param *param_;

public:
    explicit ThreadGuard(pthread_t &t1, Param *param) : t1_(t1), param_(param){};
    ~ThreadGuard()
    {
        if (!t1_)
        {
            param->mrun_num = 0;
            pthread_join(t1_, NULL);
        }
    }
    ThreadGuard(const ThreadGuard &) = delete;
    ThreadGuard &operator=(const ThreadGuard &) = delete;
};

void CAN_app::init_usb2can() {
    p_nh.param<bool>("debug_mode", param_debug, false);
    p_nh.param<int>("debug_can_id", debug_can_port, 1); // 默认读取can1数据来debug
    p_nh.param<int>("main_can_id", main_can_port, 2);
    p_nh.param<bool>("show_sending_msg", param_show_sending_msg, false);
    std::string car_type_str;
    p_nh.param<std::string>("car_type", car_type_str, "");
    nh.param<double>("vehicle_weight", vehicle_weight, 20);
    convertStringToType(car_type_str);
    switch (car_type)
    {
    case NWD:
        break;
    case JD02:
        wheel_radius = 0.0505;  // 单位m
        break;
    case JD03:
        wheel_radius = 0.065;
        break;
    case JD01:
        break;
    default:
        ROS_WARN_STREAM("This driver only supports NWD/JD01/JD02/JD03. Confirm your vehicle type.");
        return;
    }

    main_can_port = main_can_port - 1;
    debug_can_port = debug_can_port - 1;

    sub_ecu = nh.subscribe("ecu", 10, &CAN_app::ecu_cb, this);
    // sub_imu = nh.subscribe("imu_raw", 10, &CAN_app::imu_cb, this);
    pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 100);
    pub_battery_status = nh.advertise<can_msgs::battery>("battery_status", 100);

    pre_steer = 0.;

    // 打开CAN设备
    usleep(100000);
    int _ecode = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
    if (_ecode == 1) // params: 设备类型,设备索引,预留位(通常为0)  // 设备只需要打开一次
    {
        ROS_INFO_STREAM("[can_module] open device success\n");
    }
    else
    {
        printf("[can module] open USB2CAN device failed once\n");
        printf("[can module] Error code: %d", _ecode); // 返回值=1，表示操作成功； =0表示操作失败； =-1表示USB-CAN设备不存在或USB掉线
        printf("[can module] try to reset CAN-port 1 and 2 to solve this error");
        VCI_ResetCAN(VCI_USBCAN2, CAN_id, 0); //复位CAN通道 //上一次can未正确关闭会导致打不开,需要重新"拔插"一次
        VCI_ResetCAN(VCI_USBCAN2, CAN_id, 1);
        _ecode = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
        if (_ecode == 1)
        {
            ROS_INFO_STREAM("[can_module] open device success\n");
        }
        else
        {
            ROS_WARN_STREAM("[can_module] open device still failed, exit.\n");
            exit(1);
        }
    }

    config.AccCode = 0;          // 帧过滤验收码,详见说明文档及VCI_InitCAN  // 与AccMask共同决定哪些帧可以被接收
    config.AccMask = 0xFFFFFFFF; // 帧过滤屏蔽码,当前表示全部接收 TODO
    config.Filter = 0;           // 0/1 接收所有类型;2 只接收标准帧;3 只接收扩展帧
    config.Timing0 = 0x00;       // 这两个共同设置波特率,当前表示500k,其他请参照说明文档
    config.Timing1 = 0x1C;
    config.Mode = 0; // =0 正常模式; =1 只监听; =2 自发自收(回环模式)

    // 初始化CAN设备,
    usleep(100000);
    if (VCI_InitCAN(VCI_USBCAN2, CAN_id, main_can_port, &config) != 1) // params: 设备类型, 设备索引, can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Init CAN-" << main_can_port + 1);
        VCI_CloseDevice(VCI_USBCAN2, CAN_id);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-" << main_can_port + 1);
    }
    // 启动CAN设备, CAN_id = 0为默认,main_can_port = 0 对应CAN1
    usleep(100000);
    if (VCI_StartCAN(VCI_USBCAN2, CAN_id, main_can_port) != 1) // params: 设备类型,设备索引,can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Start CAN-" << main_can_port + 1);
        if (VCI_ResetCAN(VCI_USBCAN2, CAN_id, main_can_port) != 1)
        {
            ROS_ERROR_STREAM("[can_module] Try to reset can but failed, CAN-: " << main_can_port + 1);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            exit(1);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] reset can success");
        }
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-" << main_can_port + 1);
    }
}

void CAN_app::run_usb2can() {
    init_usb2can();

    param = new Param();
    param->mrun_num = 1;
    param->name = "Listener Thread";
    param->battery_status_publisher = pub_battery_status;
    param->vehicle_status_publisher = pub_vehicle_status;
    param->vehicle_status_low_rate_publisher = pub_vehicle_status_low_rate;
    param->debug_can_port = debug_can_port;
    param->main_can_port = main_can_port;
    int ret = pthread_create(&threadid, NULL, receive_func, param);
    ThreadGuard t1{threadid, param};

    pthread_t _thread_debug;
    if (param_debug)
    {
        // 目前can通道1闲置,因此用来做监听,进行debug
        int ret1 = pthread_create(&_thread_debug, NULL, thread_debug, param);
        ThreadGuard t2{_thread_debug, param};
    }
}

void CAN_app::init_eth_can() {
    p_nh.param<std::string>("can_eth_card", can_eth_card, "");
    p_nh.param<std::string>("can1_remote_ip", can1_remote_ip, "");
    p_nh.param<int>("can1_remote_port", can1_remote_port, 0); // 默认读取can1数据来debug
    p_nh.param<std::string>("can2_remote_ip", can2_remote_ip, "");
    p_nh.param<int>("can2_remote_port", can2_remote_port, 0); // 默认读取can1数据来debug
    p_nh.param<std::string>("local_ip", local_ip, "");
    p_nh.param<int>("local_port", local_port, 0); // 默认读取can1数据来debug   
    if (can_eth_card == "") {
        ROS_ERROR("[CAN] can_eth_card is not set!");
        exit(1);
    }

    p_nh.param<bool>("debug_mode", param_debug, false); 
    p_nh.param<bool>("show_sending_msg", param_show_sending_msg, false);
    std::string car_type_str;
    p_nh.param<std::string>("car_type", car_type_str, "");
    convertStringToType(car_type_str);
    switch (car_type)
    {
    case NWD:
        break;
    case JD02:
        wheel_radius = 0.0505;  // 单位m
        break;
    case JD03:
        wheel_radius = 0.065;
        break;
    case JD01:
        break;
    default:
        ROS_WARN_STREAM("This driver only supports NWD/JD01/JD02/JD03. Confirm your vehicle type.");
        return;
    }

    pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 20);
    pub_battery_status = nh.advertise<can_msgs::battery>("battery_status", 2);
    pub_vehicle_status_low_rate = nh.advertise<can_msgs::vehicle_status>("vehicle_status_low_rate", 2);

    pub_cluster = nh.advertise<visualization_msgs::MarkerArray>("radar_cluster_markers",0);
    pub_cluster_list = nh.advertise<mmware_msgs::ClusterList>("radar_cluster_list",0);

    pre_steer = 0.;
}

void CAN_app::can_recv_func(int id) {
    static int cnt = 0;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0) {
        ROS_ERROR("[CAN] Cannot create socket_fd to listen");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(local_port);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    // addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    addr_serv.sin_addr.s_addr = inet_addr(local_ip.c_str());
    len = sizeof(addr_serv);

    /* 绑定socket */
    int err_code = bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv));
    if(err_code < 0) {
        ROS_ERROR("[CAN] Cannot bind socket for listening, err_code: %d", err_code);
        return;
    }

    int recv_num;
    int send_num;
    unsigned char recv_buf[20];
    struct sockaddr_in addr_client;
    ROS_INFO("[CAN] Listen server is running:\n");
    // static std::set<int> ids;
    while( ros::ok() ) {

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

        if(recv_num < 0) {
            ROS_ERROR("[CAN] Can listen error: recv_num < 0");
            continue;
        }

        if(recv_num != 13) {
            ROS_WARN("[CAN] Can listen warn: recv_num = %d ( != 13 )", recv_num);
            continue;
        }

        // 16进制id
        int id = ((recv_buf[1] & 0xf0) >> 4 ) * 10000000
                + ((recv_buf[1] & 0x0f)) * 1000000
                + ((recv_buf[2] & 0xf0) >> 4 ) * 100000
                + ((recv_buf[2] & 0x0f)) * 10000
                + ((recv_buf[3] & 0xf0) >> 4 ) * 1000
                + ((recv_buf[3] & 0x0f)) * 100
                + ((recv_buf[4] & 0xf0) >> 4 ) * 10
                + ((recv_buf[4] & 0x0f));

        // ROS_INFO("received can id: %d", id);
        // ROS_INFO("origin data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", recv_buf[0],recv_buf[1],recv_buf[2],recv_buf[3],recv_buf[4],recv_buf[5],recv_buf[6],recv_buf[7],recv_buf[8],recv_buf[9],recv_buf[10],recv_buf[11],recv_buf[12]);
        // ids.insert(id);
        // std::string info;
        // for(auto i: ids) {
        //     info = info + std::to_string(i) + ", ";
        // }
        // ROS_INFO("received can id list: %s", info.c_str());
        switch (id)
        {
        case 51:
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }
            VehicleStatusMsg obj_vehicle(obj_can);
            can_msgs::vehicle_status msg_vehicle_status = obj_vehicle.getMessage();
            msg_vehicle_status.Header.stamp = ros::Time::now();
            msg_vehicle_status.left_wheel_speed = left_wheel_speed;
            msg_vehicle_status.right_wheel_speed = right_wheel_speed;
            pub_vehicle_status.publish(msg_vehicle_status);

            // low rate publisher
            if (cnt % 20 == 0) {
                pub_vehicle_status_low_rate.publish(msg_vehicle_status);
            }
            cnt++;
            break;
        }

        case 17904001:
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }
            BatteryMsg obj_battery(obj_can, true);
            can_msgs::battery msg_battery = obj_battery.getMessage();
            pub_battery_status.publish(msg_battery);
            break;
        }

        case 178:
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }
            WheelStatus msg_factory(obj_can, WHEEL_LEFT);
            msg_factory.init();
            left_wheel_speed = msg_factory.get_rpm()*wheel_radius*2*3.141592654/60;
            // ROS_INFO(" left rpm: %d", msg_factory.get_rpm());
            break;
        }

        case 188:
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }
            WheelStatus msg_factory(obj_can, WHEEL_RIGHT);
            right_wheel_speed = msg_factory.get_rpm()*wheel_radius*2*3.141592654/60;
            // ROS_INFO("right rpm: %d", msg_factory.get_rpm());
            break;
        }

        case 600:  // mmware cluster 0, 0x600
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }

            publish_mmware_cluster_map();

            cluster_list.header.stamp = ros::Time::now();
            cluster_list.cluster_count.data = GET_Cluster_0_Status_Cluster_NofClustersNear(obj_can.Data);

            cluster_map_.clear();
        }

        case 701:  // mmware cluster 1. 0x701
        {
            VCI_CAN_OBJ obj_can;
            for (int i = 0; i < 8; i++) {
                obj_can.Data[i] = recv_buf[i+5];
            }

            mmware_msgs::Cluster c;

            int id = GET_Cluster_1_General_Cluster_ID(obj_can.Data);
            c.cluster_id.data = id;

            c.cluster_general.cluster_distlong.data = 
                    CALC_Cluster_1_General_Cluster_DistLong(GET_Cluster_1_General_Cluster_DistLong(obj_can.Data),1.0);

            c.cluster_general.cluster_distlat.data = 
                    CALC_Cluster_1_General_Cluster_DistLat(GET_Cluster_1_General_Cluster_DistLat(obj_can.Data),1.0);

            c.cluster_general.cluster_vrellong.data = 
                    CALC_Cluster_1_General_Cluster_VrelLong(GET_Cluster_1_General_Cluster_VrelLong(obj_can.Data),1.0);

            c.cluster_general.cluster_vrellat.data = 
                    CALC_Cluster_1_General_Cluster_VrelLat(GET_Cluster_1_General_Cluster_VrelLat(obj_can.Data),1.0);

            c.cluster_general.cluster_dynprop.data = 
                    CALC_Cluster_1_General_Cluster_DynProp(GET_Cluster_1_General_Cluster_DynProp(obj_can.Data),1.0);

            c.cluster_general.cluster_rcs.data = 
                    CALC_Cluster_1_General_Cluster_RCS(GET_Cluster_1_General_Cluster_RCS(obj_can.Data),1.0);

            cluster_map_.insert(std::pair<int, mmware_msgs::Cluster>(id, c));
        }

        default:
            break;
        }

    }

    printf("[CAN] closing can listen thread\n");
    close(sock_fd);
    printf("[CAN] closing can listen thread, done\n");
}

void CAN_app::run_eth_can() {
    init_eth_can();

    struct ifreq ifr;
    memset(&ifr, 0x00, sizeof(ifr));
    // enp1s0
    strncpy(ifr.ifr_name, can_eth_card.c_str(), sizeof(ifr.ifr_name));

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(sock_fd, SOL_SOCKET, SO_BINDTODEVICE, (char *)&ifr, sizeof(ifr));
    if ( sock_fd < 0 ) {
        ROS_ERROR("[CAN_DRIVER] Cannot open socket for EthCan!");
        exit(1);
    }

    // CAN1
    memset(&can1_addr_serv, 0, sizeof(can1_addr_serv));
    can1_addr_serv.sin_family = AF_INET;
    can1_addr_serv.sin_addr.s_addr = inet_addr(can1_remote_ip.c_str());
    can1_addr_serv.sin_port = htons(can1_remote_port);
    can1_addr_serv_len = sizeof(can1_addr_serv);

    // CAN2
    memset(&can2_addr_serv, 0, sizeof(can2_addr_serv));
    can2_addr_serv.sin_family = AF_INET;
    can2_addr_serv.sin_addr.s_addr = inet_addr(can2_remote_ip.c_str());
    can2_addr_serv.sin_port = htons(can2_remote_port);
    can2_addr_serv_len = sizeof(can2_addr_serv);

    this->tasks.push_back(std::thread(&CAN_app::can_recv_func, this, 1));
    sub_ecu = nh.subscribe("ecu", 10, &CAN_app::ecu_cb, this);
    sub_imu = nh.subscribe("imu_raw", 10, &CAN_app::imu_cb, this);
}

void CAN_app::publish_mmware_cluster_map() {
    std::map<int, mmware_msgs::Cluster>::iterator itr;

    
    visualization_msgs::MarkerArray marker_array;

    //marker for ego car
    visualization_msgs::Marker mEgoCar;

    mEgoCar.header.stamp = ros::Time::now();
    mEgoCar.header.frame_id = "mmware";
    mEgoCar.ns = "ego";
    mEgoCar.id = 999;

    //if you want to use a cube comment out the next 2 line
    mEgoCar.type = 1; // cube
    mEgoCar.action = 0; // add/modify
    mEgoCar.pose.position.x = 0.0;
    mEgoCar.pose.position.y = 0.0;
    mEgoCar.pose.position.z = 0.0;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, M_PI/2);

    mEgoCar.pose.orientation.w = myQuaternion.getW();
    mEgoCar.pose.orientation.x = myQuaternion.getX();
    mEgoCar.pose.orientation.y = myQuaternion.getY();
    mEgoCar.pose.orientation.z = myQuaternion.getZ();
    mEgoCar.scale.x = 0.2;
    mEgoCar.scale.y = 0.2;
    mEgoCar.scale.z = 0.2;
    mEgoCar.color.r = 0.0;
    mEgoCar.color.g = 0.0;
    mEgoCar.color.b = 1.0;
    mEgoCar.color.a = 1.0;
    mEgoCar.lifetime = ros::Duration(0.2);
    mEgoCar.frame_locked = false;

    marker_array.markers.push_back(mEgoCar);

    for (itr = cluster_map_.begin(); itr != cluster_map_.end(); ++itr) {

            visualization_msgs::Marker mobject;
            visualization_msgs::Marker mtext;

            mtext.header.stamp = ros::Time::now();
            mtext.header.frame_id = "mmware";
            mtext.ns = "text";
            mtext.id = (itr->first+100);
            mtext.type = 1; //Cube
            mtext.action = 0; // add/modify
            mtext.pose.position.x = itr->second.cluster_general.cluster_distlong.data;
            mtext.pose.position.y = itr->second.cluster_general.cluster_distlat.data;
            mtext.pose.position.z = 4.0;

            //myQuaternion.setRPY(M_PI / 2, 0, 0);
            myQuaternion.setRPY(0, 0, 0);

            mtext.pose.orientation.w = myQuaternion.getW();
            mtext.pose.orientation.x = myQuaternion.getX();
            mtext.pose.orientation.y = myQuaternion.getY();
            mtext.pose.orientation.z = myQuaternion.getZ();
            // mtext.scale.x = 1.0;
            // mtext.scale.y = 1.0;
            mtext.scale.z = 2.0;
            mtext.color.r = 1.0;
            mtext.color.g = 1.0;
            mtext.color.b = 1.0;
            mtext.color.a = 1.0;
            mtext.lifetime = ros::Duration(0.2);
            mtext.frame_locked = false;
            mtext.type=9;
            mtext.text= "Cluster" + std::to_string(itr->first) + ": \n"
            + " RCS: " + std::to_string(itr->second.cluster_general.cluster_rcs.data) + "dBm^2"; // + " \n"
            //+ " V_long: " +   std::to_string(itr->second.cluster_general.cluster_vrellong.data) + "m/s" + " \n"
            //+ " V_lat: " + std::to_string(itr->second.cluster_general.cluster_vrellat.data) + "m/s" + " \n";
            //+ " Orientation: " + std::to_string(itr->second.cluster_general.cluster_vrellon);

            marker_array.markers.push_back(mtext);

            mobject.header.stamp = ros::Time::now();
            mobject.header.frame_id = "mmware";
            mobject.ns = "objects";
            mobject.id = itr->first;
            mobject.type = 1; //Cube
            mobject.action = 0; // add/modify
            mobject.pose.position.x = itr->second.cluster_general.cluster_distlong.data;
            mobject.pose.position.y = itr->second.cluster_general.cluster_distlat.data;
            mobject.pose.position.z = 1.0;

            myQuaternion.setRPY(0, 0, 0);

            mobject.pose.orientation.w = myQuaternion.getW();
            mobject.pose.orientation.x = myQuaternion.getX();
            mobject.pose.orientation.y = myQuaternion.getY();
            mobject.pose.orientation.z = myQuaternion.getZ();
            mobject.scale.x = 2.0;
            mobject.scale.y = 2.0;
            mobject.scale.z = 1.0;
            mobject.color.r = 0.0;
            mobject.color.g = 1.0;
            mobject.color.b = 0.0;
            mobject.color.a = 1.0;
            mobject.lifetime = ros::Duration(0.2);
            mobject.frame_locked = false;

            cluster_list.clusters.push_back(itr->second);

            marker_array.markers.push_back(mobject);


    }
    pub_cluster_list.publish(cluster_list);
    pub_cluster.publish(marker_array);
}

void CAN_app::run()
{
    int t;
    p_nh.param<int>("can_type", t, 0);
    can_type = CanType(t);

    switch (can_type)
    {
    case USB2CAN:
        run_usb2can();
        break;
    case EthCAN_UDP:
        run_eth_can();
        break;
    default:
        ROS_ERROR("[CAN_DRIVER] Unrecognized can type: %d", t);
        break;
    }

    ros::spin();
    ros::shutdown();
}
};  // namespace USB2CAN
    // 写在最后: 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
