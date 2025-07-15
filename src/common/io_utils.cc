//
// Created by xiang on 2021/7/20.
// Modified: 去掉ROS依赖，保留TxtIO功能
//
#include "common/io_utils.h"

#include <glog/logging.h>
#include <sstream>
#include <vector>

namespace sad {

void TxtIO::Go() {
    if (!fin) {
        LOG(ERROR) << "未能找到文件";
        return;
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            continue;
        }

        if (line[0] == '#') {
            // 以#开头的是注释
            continue;
        }

        // load data from line
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;

        if (data_type == "$GPS" && gnss_proc_) {
            ProcessGPS(ss);
        } else if (data_type == "$ACC" && imu_proc_) {
            ProcessACC(ss);
        } else if (data_type == "$GYR" && imu_proc_) {
            ProcessGYR(ss);
        } else if (data_type == "IMU" && imu_proc_) {
            // 保持对原格式的兼容
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
        } else if (data_type == "ODOM" && odom_proc_) {
            // 保持对原格式的兼容
            double time, wl, wr;
            ss >> time >> wl >> wr;
            odom_proc_(Odom(time, wl, wr));
        } else if (data_type == "GNSS" && gnss_proc_) {
            // 保持对原格式的兼容
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            gnss_proc_(GNSS(time, 4, Vec3d(lat, lon, alt), heading, heading_valid));
        }
    }

    LOG(INFO) << "done.";
}

void TxtIO::ProcessGPS(std::stringstream& ss) {
    // GPS格式：时间戳、WGS84经纬度、航向、速度、高度、定位状态
    // 字段索引：1=时间戳, 7=经度_wgs84, 8=纬度_wgs84, 9=航向, 10=速度, 11=高度, 12=GPS状态
    std::vector<std::string> fields;
    std::string field;
    
    // 读取所有字段
    while (ss >> field) {
        fields.push_back(field);
    }
    
    if (fields.size() < 13) {
        LOG(WARNING) << "GPS数据字段不足，需要至少13个字段，实际：" << fields.size();
        return;
    }
    
    try {
        // 解析时间戳（毫秒转秒）
        double timestamp = std::stod(fields[0]) / 1000.0;
        
        // 使用WGS84经纬度（字段6、7）
        double longitude_wgs84 = std::stod(fields[6]) / 10000000.0;  // WGS84经度
        double latitude_wgs84 = std::stod(fields[7]) / 10000000.0;   // WGS84纬度
        
        // 解析航向（度）
        double heading = std::stod(fields[8]);
        
        // 解析速度（km/h）
        double speed = std::stod(fields[9]);
        
        // 解析高度（米）
        double altitude = std::stod(fields[10]);
        
        // 解析GPS状态
        bool gps_valid = (fields[11] == "A");
        
        // 创建GNSS数据
        Vec3d lat_lon_alt(latitude_wgs84, longitude_wgs84, altitude);
        gnss_proc_(GNSS(timestamp, gps_valid ? 4 : 0, lat_lon_alt, heading, gps_valid));
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析GPS数据失败: " << e.what();
    }
}

void TxtIO::ProcessACC(std::stringstream& ss) {
    // ACC格式：时间戳 有效轴 时间间隔 朝上轴读数 朝前轴读数 朝右轴读数
    // 坐标系转换：[朝上,朝前,朝右] -> [Z,Y,X] -> 重排为XYZ=[朝右,朝前,朝上]
    std::vector<std::string> fields;
    std::string field;
    
    // 读取所有字段
    while (ss >> field) {
        fields.push_back(field);
    }
    
    if (fields.size() < 6) {
        LOG(WARNING) << "ACC数据字段不足，需要至少6个字段，实际：" << fields.size();
        return;
    }
    
    try {
        // 解析时间戳（毫秒转秒）
        double timestamp = std::stod(fields[0]) / 1000.0;
        
        // 解析加速度数据（g转m/s²）
        // 数据顺序：朝上轴、朝前轴、朝右轴
        // 坐标系映射：右前上-XYZ = [朝右, 朝前, 朝上]
        double acc_up = std::stod(fields[3]) * 9.8;    // 朝上轴 -> Z
        double acc_front = std::stod(fields[4]) * 9.8; // 朝前轴 -> Y  
        double acc_right = std::stod(fields[5]) * 9.8; // 朝右轴 -> X
        
        // 存储加速度数据（按XYZ顺序）
        pending_acc_.timestamp = timestamp;
        pending_acc_.acce = Vec3d(acc_right, acc_front, acc_up); // [X, Y, Z]
        pending_acc_.valid = true;
        
        // 尝试组合IMU数据
        TryCreateIMU();
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析ACC数据失败: " << e.what();
    }
}

void TxtIO::ProcessGYR(std::stringstream& ss) {
    // GYR格式：时间戳 有效轴 时间间隔 温度值 朝上轴读数 朝前轴读数 朝右轴读数
    // 坐标系转换：[朝上,朝前,朝右] -> [Z,Y,X] -> 重排为XYZ=[朝右,朝前,朝上]
    std::vector<std::string> fields;
    std::string field;
    
    // 读取所有字段
    while (ss >> field) {
        fields.push_back(field);
    }
    
    if (fields.size() < 7) {
        LOG(WARNING) << "GYR数据字段不足，需要至少7个字段，实际：" << fields.size();
        return;
    }
    
    try {
        // 解析时间戳（毫秒转秒）
        double timestamp = std::stod(fields[0]) / 1000.0;
        
        // 解析陀螺仪数据（度/秒转弧度/秒）
        // 数据顺序：朝上轴、朝前轴、朝右轴
        // 坐标系映射：右前上-XYZ = [朝右, 朝前, 朝上]
        double gyro_up = std::stod(fields[4]) * math::kDEG2RAD;    // 朝上轴 -> Z
        double gyro_front = std::stod(fields[5]) * math::kDEG2RAD; // 朝前轴 -> Y
        double gyro_right = std::stod(fields[6]) * math::kDEG2RAD; // 朝右轴 -> X
        
        // 存储陀螺仪数据（按XYZ顺序）
        pending_gyr_.timestamp = timestamp;
        pending_gyr_.gyro = Vec3d(gyro_right, gyro_front, gyro_up); // [X, Y, Z]
        pending_gyr_.valid = true;
        
        // 尝试组合IMU数据
        TryCreateIMU();
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析GYR数据失败: " << e.what();
    }
}

void TxtIO::TryCreateIMU() {
    // 检查是否有有效的加速度和陀螺仪数据
    if (!pending_acc_.valid || !pending_gyr_.valid) {
        return;
    }
    
    // 检查时间戳是否接近（在阈值范围内）
    double time_diff = std::abs(pending_acc_.timestamp - pending_gyr_.timestamp);
    if (time_diff > TIME_SYNC_THRESHOLD) {
        // 时间差太大，保留较新的数据，丢弃较旧的数据
        if (pending_acc_.timestamp < pending_gyr_.timestamp) {
            pending_acc_.valid = false;
        } else {
            pending_gyr_.valid = false;
        }
        return;
    }
    
    // 使用较新的时间戳
    double timestamp = std::max(pending_acc_.timestamp, pending_gyr_.timestamp);
    
    // 创建IMU数据并调用回调
    IMU imu_data(timestamp, pending_gyr_.gyro, pending_acc_.acce);
    imu_proc_(imu_data);
    
    // 标记数据已使用
    pending_acc_.valid = false;
    pending_gyr_.valid = false;
}

}  // namespace sad