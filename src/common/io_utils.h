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
#include <set>  

namespace sad {

/// NZZ数据结构
struct NZZ {
    std::string time_key_;  // 时间字符串，用于匹配 "2025-6-12 11:22:27"
    double heading_;        // 航向角（度）
    
    NZZ() = default;
    NZZ(const std::string& time_key, double heading) : time_key_(time_key), heading_(heading) {}
};

/// 带时间字符串的GPS数据结构，用于GPS-NZZ匹配
struct GPSWithTimeKey {
    GNSS gnss_data_;       // 原始GPS数据
    std::string time_key_; // 时间字符串，用于匹配 "2025-6-12 11:22:27"
    
    GPSWithTimeKey() = default;
    GPSWithTimeKey(const GNSS& gnss, const std::string& time_key) 
        : gnss_data_(gnss), time_key_(time_key) {}
};


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
    using NZZProcessFuncType = std::function<void(const NZZ &)>;
    using GPSWithTimeKeyProcessFuncType = std::function<void(const GPSWithTimeKey &)>;

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

    TxtIO &SetNZZProcessFunc(NZZProcessFuncType nzz_proc) {
        nzz_proc_ = std::move(nzz_proc);
        return *this;
    }

    TxtIO &SetGPSWithTimeKeyProcessFunc(GPSWithTimeKeyProcessFuncType gps_timekey_proc) {
        gps_timekey_proc_ = std::move(gps_timekey_proc);
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
    void ProcessNZZ(std::stringstream& ss);

    /// 尝试组合IMU数据
    void TryCreateIMU();

    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;
    NZZProcessFuncType nzz_proc_;
    GPSWithTimeKeyProcessFuncType gps_timekey_proc_;

    /// IMU数据组合相关
    PendingAccData pending_acc_;
    PendingGyrData pending_gyr_;
    static constexpr double TIME_SYNC_THRESHOLD = 0.05; // 50ms同步阈值

    /// NZZ数据去重相关
    std::set<std::string> processed_nzz_times_; // 已处理的NZZ时间，用于去重
};

// 注释掉RosbagIO类，因为它依赖ROS
/*
class RosbagIO {
    // ... ROS相关功能暂时移除
};
*/

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IO_UTILS_H