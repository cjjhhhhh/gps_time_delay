//
// Created by xiang on 2021/7/20.
// Modified: 去掉ROS依赖，保留TxtIO功能
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

#include <fstream>
#include <functional>
#include <utility>

#include "common/dataset_type.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/odom.h"

namespace sad {

/**
 * 读取本书提供的数据文本文件，并调用回调函数
 * 数据文本文件主要提供IMU/Odom/GNSS读数
 */
class TxtIO {
   public:
    TxtIO(const std::string &file_path) : fin(file_path) {}

    /// 定义回调函数
    using IMUProcessFuncType = std::function<void(const IMU &)>;
    using OdomProcessFuncType = std::function<void(const Odom &)>;
    using GNSSProcessFuncType = std::function<void(const GNSS &)>;

    TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }

    TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }

    // 遍历文件内容，调用回调函数
    void Go();

   private:
    /// 存储待组合的加速度和陀螺仪数据
    struct PendingAccData {
        double timestamp;
        Vec3d acce;
        bool valid = false;
    };
    
    struct PendingGyrData {
        double timestamp;
        Vec3d gyro;
        bool valid = false;
    };

    /// 处理各种数据格式
    void ProcessGPS(std::stringstream& ss);
    void ProcessACC(std::stringstream& ss);
    void ProcessGYR(std::stringstream& ss);
    
    /// 尝试组合IMU数据
    void TryCreateIMU();

    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;

    /// IMU数据组合相关
    PendingAccData pending_acc_;
    PendingGyrData pending_gyr_;
    static constexpr double TIME_SYNC_THRESHOLD = 0.05; // 50ms同步阈值
};

// 注释掉RosbagIO类，因为它依赖ROS
/*
class RosbagIO {
    // ... ROS相关功能暂时移除
};
*/

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IO_UTILS_H