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
        } else if (data_type == "$NZZ" && nzz_proc_) {
            ProcessNZZ(ss);
        } else if (data_type == "$FBK" && fbk_proc_) {
            ProcessFBK(ss);
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
    // 时间字段：19=年, 20=月, 21=日, 22=时, 23=分, 24=秒
    std::vector<std::string> fields;
    std::string field;
    
    // 读取所有字段
    while (ss >> field) {
        fields.push_back(field);
    }
    
    if (fields.size() < 25) {  // 需要包含时间字段
        LOG(WARNING) << "GPS数据字段不足，需要至少25个字段，实际：" << fields.size();
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
        bool heading_valid = true;
        
        // 创建GNSS数据
        Vec3d lat_lon_alt(latitude_wgs84, longitude_wgs84, altitude);
        GNSS gnss_data(timestamp, gps_valid ? 4 : 0, lat_lon_alt, heading, heading_valid);
        
        // 调用原有的GNSS回调
        if (gnss_proc_) {
            gnss_proc_(gnss_data);
        }
        
        // 如果需要GPS+时间字符串匹配，提取时间字符串并调用对应回调
        if (gps_timekey_proc_) {
            // 提取GPS时间：年月日时分秒
            int year = std::stoi(fields[18]);   // 字段19-1=18
            int month = std::stoi(fields[19]);  // 字段20-1=19  
            int day = std::stoi(fields[20]);    // 字段21-1=20
            int hour = std::stoi(fields[21]);   // 字段22-1=21
            int minute = std::stoi(fields[22]); // 字段23-1=22
            int second = std::stoi(fields[23]); // 字段24-1=23
            
            // 构造时间字符串键，格式与NZZ一致："2025-6-12 11:22:27"
            std::string time_key = std::to_string(year) + "-" + std::to_string(month) + "-" + std::to_string(day) + 
                                  " " + std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second);
            
            GPSWithTimeKey gps_with_timekey(gnss_data, time_key);
            gps_timekey_proc_(gps_with_timekey);
        }
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析GPS数据失败: " << e.what();
    }
}

void TxtIO::ProcessNZZ(std::stringstream& ss) {
    // NZZ格式：$NZZ 2025-6-12 11:22:27 ... 271.862000 ...
    // 注意：$NZZ已经在Go()中被读取，所以这里：
    // fields[0] = 2025-6-12 (日期)
    // fields[1] = 11:22:27  (时间)
    // fields[11] = 271.862000 (航向角，对应Python中的fields[12])
    
    std::vector<std::string> fields;
    std::string field;
    
    // 读取所有字段
    while (ss >> field) {
        fields.push_back(field);
    }
    
    if (fields.size() < 12) {  // 需要至少12个字段才能访问fields[11]
        LOG(WARNING) << "NZZ数据字段不足，需要至少12个字段，实际：" << fields.size();
        return;
    }
    
    try {
        // 解析时间：fields[0] = 日期(2025-6-12), fields[1] = 时间(11:22:27)
        std::string date_str = fields[0];  // 2025-6-12
        std::string time_str = fields[1];  // 11:22:27
        
        // 构建时间字符串键，用于与GPS匹配
        std::string time_key = date_str + " " + time_str;  // "2025-6-12 11:22:27"
        
        // 去重：每秒只保留第一个NZZ数据（模仿Python逻辑）
        if (processed_nzz_times_.find(time_key) != processed_nzz_times_.end()) {
            // 该时间已处理过，跳过
            return;
        }
        
        // 标记该时间已处理
        processed_nzz_times_.insert(time_key);
        
        // 解析航向角（对应Python中的fields[12]，但这里是fields[11]因为$NZZ已被读取）
        double heading = std::stod(fields[11]);
        
        // 创建NZZ数据并调用回调
        NZZ nzz_data(time_key, heading);
        nzz_proc_(nzz_data);
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析NZZ数据失败: " << e.what();
    }
}

void TxtIO::ProcessFBK(std::stringstream& ss) {
    // FBK数据有两种格式：
    // flag行：$FBK flag,1,164385368,-0.153193,0.030816,...（逗号分隔）
    // misalignment行：$FBK misalignment pitch:-18.122493 heading:1.800880（空格分隔）
    
    std::string full_line;
    std::getline(ss, full_line);
    
    // 去除前后空格
    full_line.erase(0, full_line.find_first_not_of(" \t"));
    full_line.erase(full_line.find_last_not_of(" \t") + 1);
    
    if (full_line.empty()) {
        LOG(WARNING) << "FBK数据为空";
        return;
    }
    
    try {
        // 判断是flag行还是misalignment行
        if (full_line.find("flag") == 0) {
            // flag行：使用逗号分隔
            std::vector<std::string> fields;
            std::stringstream line_ss(full_line);
            std::string field;
            
            while (std::getline(line_ss, field, ',')) {
                // 去除前后空格
                field.erase(0, field.find_first_not_of(" \t"));
                field.erase(field.find_last_not_of(" \t") + 1);
                fields.push_back(field);
            }
            
            if (fields.size() < 3) {
                LOG(WARNING) << "FBK flag数据字段不足，需要至少3个字段";
                return;
            }
            
            // 提取时间戳（字段2，毫秒转秒）
            double timestamp = std::stod(fields[2]) / 1000.0;
            
            // 存储flag数据，等待下一行的misalignment
            pending_flag_ = FBKFlag(timestamp);
            pending_flag_valid_ = true;
                        
        } else if (full_line.find("misalignment") == 0) {
            // misalignment行：使用空格分隔
            if (!pending_flag_valid_) {
                LOG(WARNING) << "收到misalignment但没有对应的flag数据";
                return;
            }
            
            std::vector<std::string> fields;
            std::stringstream line_ss(full_line);
            std::string field;
            
            // 按空格分隔
            while (line_ss >> field) {
                fields.push_back(field);
            }
            
            if (fields.size() < 2) {
                LOG(WARNING) << "FBK misalignment数据字段不足";
                return;
            }
            
            double pitch = 0.0, heading = 0.0;
            bool pitch_found = false, heading_found = false;
            
            // fields[1] 包含 "pitch:-19.279136,heading:-1.083479"
            // 需要按逗号进一步分割
            std::string pitch_heading_str = fields[1];
            std::stringstream ph_ss(pitch_heading_str);
            std::string ph_field;
            
            while (std::getline(ph_ss, ph_field, ',')) {
                // 去除前后空格
                ph_field.erase(0, ph_field.find_first_not_of(" \t"));
                ph_field.erase(ph_field.find_last_not_of(" \t") + 1);
                
                if (ph_field.find("pitch:") == 0) {
                    // 从"pitch:-18.122493"中提取数值
                    std::string value_str = ph_field.substr(6); // 跳过"pitch:"
                    pitch = std::stod(value_str);
                    pitch_found = true;
                }
                
                if (ph_field.find("heading:") == 0) {
                    // 从"heading:1.800880"中提取数值
                    std::string value_str = ph_field.substr(8); // 跳过"heading:"
                    heading = std::stod(value_str);
                    heading_found = true;
                }
            }
            
            if (pitch_found && heading_found) {
                // 创建完整的FBK对并调用回调
                FBKMisalignment misalignment(pitch, heading);
                FBKPair fbk_pair(pending_flag_, misalignment);
                
                fbk_proc_(fbk_pair);
                
                // 重置pending状态
                pending_flag_valid_ = false;
            } else {
                LOG(WARNING) << "FBK misalignment数据解析失败，pitch_found: " << pitch_found 
                           << ", heading_found: " << heading_found;
            }
        } else {
            // 忽略其他格式的FBK行（如数字开头的行、info行等）
            // LOG(INFO) << "忽略FBK行: " << full_line.substr(0, 50) << "...";
            return;
        }
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "解析FBK数据失败: " << e.what();
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