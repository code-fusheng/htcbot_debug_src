/*
 * @Author: code-fusheng
 * @Date: 2024-04-27 13:24:44
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-22 16:33:42
 * @Description: 
 */
#ifndef HTCBOT_COMMON_H
#define HTCBOT_COMMON_H

#include <ros/ros.h>
#include <iostream>

namespace HtcbotCommonNS {
    
// 系统通用枚举

//  传感器类型
enum class SENSOR_TYPE {
    UNKNOWN,               // 
    GNSS,               // GNSS 惯导
    LASER,              // 激光雷达
    ULTRASONIC,         // 超声波雷达
    CAMERA,             // 摄像头
    DEPTH_CAMERA,       // 深度相机
    IMU,                // IMU
    VEHICLE,            // 底盘
    OTHER               // 其他
};

// 模块类型
enum class MODULE_TYPE {
    UNKNOWN,     
    MAP,
    LOCALIZER,
    SENSOR,
    WAYPOINT,
    PLANNER,
    CONTROL,
    GATEWAY
};

// 状态类型
enum class STATUS_TYPE {
    NONE,               // 默认
    READY,              // 就绪
    WARN,               // 警告
    DANGER,             // 危险
    EXC,                // 异常
    EXPIRED,             // 过期
};

// 音频场景
enum class VOICE_SCENES {
    
    UNKNOWN = 0,  
    AUTO_RUNNING = 1001,        // 自动运行
    FRONT_TRUN_LEFT = 1002,     // 前方左转
    FRONT_TRUN_RIGHT = 1003,    // 前方右转
    FRONT_STRAIGHT = 1004,      // 前方直行
    TRUN_LEFT_WARN = 1005,      // 左转,请注意!
    TRUN_RIGHT_WARN = 1006,     // 右转,请注意!
    TRUN_BACK_WARN = 1007,      // 倒车,请注意!
    
    WARN_BE_AVOID = 2001,       // 危险,请避让!
    WARN_BE_FAR = 2002,         // 危险,请远离!
    WARN_BE_NOT_FOLLOW = 2003,  // 危险,请勿跟车!

    STATUS_READY = 3001,      // 状态就绪
    STATUS_EXC = 3002,        // 状态异常 
    
    SENSOR_READY = 3100,         // 传感器状态就绪

    SENSOR_LASER_READY = 3120,   // 雷达就绪
    SENSOR_LASER_WARN = 3121,    // 雷达警告
    SENSOR_LASER_EXC = 3122,     // 雷达异常
    
    SENSOR_VEHICLE_READY = 3170,             // 底盘就绪
    SENSOR_VEHICLE_BATTERY_WARN = 3171,      // 电池警告
    SENSOR_VEHICLE_BATTERY_DANGER = 3172,    // 电池危险

    MODULE_READY = 3200,         // 模块状态就绪
    
    MODULE_LOCALIZER_READY = 3220,    // 定位就绪
    MODULE_LOCALIZER_WARN = 3221,     // 定位质量差
    MODULE_LOCALIZER_EXC_TRY_RECOVER = 3222,      // 定位异常 尝试恢复
    MODULE_LOCALIZER_GPS_OFFSET_TOO_FAR_WARN = 3223      // GPS 定位差

};

}

#endif //HTCBOT_COMMON_H