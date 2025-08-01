//
// Created by xiang on 2021/11/11.
//

#include "ch3/eskf.hpp"
#include "common/io_utils.h"
#include "utm_convert.h"
#include "turn_detector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <queue>

DEFINE_string(txt_path, "/Users/cjj/Data/vdr_plog/Honor_V40/vdr_20250523_162014_895.log", "数据文件路径");
DEFINE_bool(offline_mode, false, "是否使用离线重组织模式，同时使用转弯检测");
DEFINE_double(gps_time_offset, 0.0, "GPS时间偏移");
DEFINE_bool(enable_turn_detection, true, "是否启用转弯检测（仅在离线模式下有效）");  // 新增，默认开启

//时间戳数据结构
struct TimeStampedData {
    double timestamp;
    enum DataType { IMU_TYPE, GPS_TYPE } type;

    sad::IMU imu_data;
    sad::GNSS gps_data;

    TimeStampedData(const sad::IMU& imu)
        : timestamp(imu.timestamp_), type(IMU_TYPE), imu_data(imu) {}

    TimeStampedData(const sad::GNSS& gnss)
        : timestamp(gnss.unix_time_), type(GPS_TYPE), gps_data(gnss) {}

    bool operator<(const TimeStampedData& other) const {
        return timestamp < other.timestamp;
    }
};


/**
 * 本程序演示使用RTK+IMU进行组合导航
 */
bool InitializeESKF(sad::ESKFD& eskf){
    // 陀螺零偏 (度/秒) 
    const double GYRO_BIAS_X = 0.001711;
    const double GYRO_BIAS_Y = -0.021235;
    const double GYRO_BIAS_Z = 0.049159;
    
    // 加速度零偏 (m/s²) 
    const double ACCEL_BIAS_X = -0.013369;
    const double ACCEL_BIAS_Y = -0.020087;
    const double ACCEL_BIAS_Z = 0.101552;
    
    sad::ESKFD::Options options;
    options.gyro_var_ = 2e-3;     // 陀螺噪声
    options.acce_var_ = 5e-2;     // 加速度噪声
    options.bias_gyro_var_ = 1e-6; // 陀螺零偏随机游走
    options.bias_acce_var_ = 1e-4; // 加速度零偏随机游走

    Vec3d init_bg(GYRO_BIAS_X * sad::math::kDEG2RAD, GYRO_BIAS_Y * sad::math::kDEG2RAD, GYRO_BIAS_Z * sad::math::kDEG2RAD);
    Vec3d init_ba(ACCEL_BIAS_X, ACCEL_BIAS_Y, ACCEL_BIAS_Z);
    Vec3d gravity(0, 0, -9.8);

    eskf.SetInitialConditions(options, init_bg, init_ba, gravity);
    return true;


}

//离线数据管理
class OfflineDataManager {
private:
    std::vector<TimeStampedData> all_data_;
    double gps_time_offset_ = 0.0;

    // 新增：GPS-NZZ匹配结果存储
    std::vector<std::pair<double, double>> matched_heading_data_; // (gps_timestamp, nzz_heading)

    // 新增：FBK数据存储
    std::vector<sad::FBKPair> fbk_data_;

public:

    //读取所有数据
    bool ReadAllData(const std::string& file_path,
                    std::vector<sad::IMU>& imu_data,
                    std::vector<sad::GNSS>& gps_data) {
                        
        // 新增：收集GPS-NZZ匹配数据
        std::vector<sad::GPSWithTimeKey> gps_with_timekey;
        std::vector<sad::NZZ> nzz_data;

        // 新增：收集FBK数据
        std::vector<sad::FBKPair> fbk_data;

        sad::TxtIO io(file_path);
        io.SetIMUProcessFunc([&](const sad::IMU& imu){
            imu_data.push_back(imu);
        }).SetGNSSProcessFunc([&](const sad::GNSS& gps){
            gps_data.push_back(gps);
        }).SetGPSWithTimeKeyProcessFunc([&](const sad::GPSWithTimeKey& gps_timekey){
            gps_with_timekey.push_back(gps_timekey);
        }).SetNZZProcessFunc([&](const sad::NZZ& nzz){
            nzz_data.push_back(nzz);
        }).SetFBKPairProcessFunc([&](const sad::FBKPair& fbk_pair){  // 新增
            fbk_data.push_back(fbk_pair);
        });

        io.Go();

        LOG(INFO) << "数据读取完成: GPS=" << gps_with_timekey.size() 
                  << ", NZZ=" << nzz_data.size() << ", FBK=" << fbk_data.size();
        
        // 新增：进行GPS-NZZ匹配
        MatchGPSNZZData(gps_with_timekey, nzz_data);

        fbk_data_ = fbk_data;
        return !imu_data.empty() && !gps_data.empty();
     }

    // 新增：获取匹配的航向数据
    const std::vector<std::pair<double, double>>& GetMatchedHeadingData() const {
        return matched_heading_data_;
    }

    // 新增：获取FBK数据
    const std::vector<sad::FBKPair>& GetFBKData() const {
        return fbk_data_;
    }

    void SetGPSTimeOffset(double offset) {
        gps_time_offset_ = offset;
        LOG(INFO) << "设置GPS时间偏移" << offset << "s";
    }

    bool LoadAndReorganizeData (const std::string& file_path) {
        std::vector<sad::IMU> imu_data;
        std::vector<sad::GNSS> gps_data;

        // 读取数据
        if(!ReadAllData(file_path, imu_data, gps_data)) {
            LOG(ERROR) << "数据读取失败" ;
            return false;
        }

        // 应用时间偏移
        ConvertToTimeStampedData (imu_data, gps_data);

        // 按时间戳排序
        std::sort (all_data_.begin(), all_data_.end());

        return true;
    }

    //获取重组织后的数据
    const std::vector<TimeStampedData>& GetReorganizedData() const {
        return all_data_;
    }

private:


    // 新增：GPS-NZZ匹配方法 - 对应Python的match_gps_nzz_data
    void MatchGPSNZZData(const std::vector<sad::GPSWithTimeKey>& gps_data,
                         const std::vector<sad::NZZ>& nzz_data) {
        matched_heading_data_.clear();
        
        LOG(INFO) << "开始GPS-NZZ数据匹配...";
        
        int direct_matches = 0;
        int fuzzy_matches = 0;
        
        for (const auto& gps : gps_data) {
            bool found_match = false;
            
            // 1. 直接匹配
            for (const auto& nzz : nzz_data) {
                if (gps.time_key_ == nzz.time_key_) {
                    // 应用GPS时间偏移
                    double adjusted_gps_time = gps.gnss_data_.unix_time_ + gps_time_offset_;
                    matched_heading_data_.emplace_back(adjusted_gps_time, nzz.heading_);
                    found_match = true;
                    direct_matches++;
                    break;
                }
            }
            
            // 2. 如果直接匹配失败，尝试模糊匹配
            if (!found_match) {
                std::string gps_normalized = NormalizeTimeKey(gps.time_key_);
                
                for (const auto& nzz : nzz_data) {
                    std::string nzz_normalized = NormalizeTimeKey(nzz.time_key_);
                    
                    if (gps_normalized == nzz_normalized) {
                        double adjusted_gps_time = gps.gnss_data_.unix_time_ + gps_time_offset_;
                        matched_heading_data_.emplace_back(adjusted_gps_time, nzz.heading_);
                        found_match = true;
                        fuzzy_matches++;
                        break;
                    }
                }
            }
        }
        
        // 按时间戳排序
        std::sort(matched_heading_data_.begin(), matched_heading_data_.end(),
                  [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                      return a.first < b.first;
                  });
        
        LOG(INFO) << "GPS-NZZ匹配完成:";
        LOG(INFO) << "  直接匹配: " << direct_matches << " 个";
        LOG(INFO) << "  模糊匹配: " << fuzzy_matches << " 个";
        LOG(INFO) << "  总匹配数: " << matched_heading_data_.size() << " 个";
    }
    
    // 新增：标准化时间字符串格式 - 对应Python的normalize_time_key
    std::string NormalizeTimeKey(const std::string& time_key) const {
        try {
            // 检查格式：应该包含 '-' 和 ':'
            if (time_key.find('-') == std::string::npos || 
                time_key.find(':') == std::string::npos) {
                return time_key;
            }
            
            // 分离日期和时间部分
            size_t space_pos = time_key.find(' ');
            if (space_pos == std::string::npos) {
                return time_key;
            }
            
            std::string date_part = time_key.substr(0, space_pos);
            std::string time_part = time_key.substr(space_pos + 1);
            
            // 处理日期部分：YYYY-M-D -> YYYY-MM-DD
            std::string normalized_date = NormalizeDatePart(date_part);
            
            // 处理时间部分：H:M:S -> HH:MM:SS
            std::string normalized_time = NormalizeTimePart(time_part);
            
            return normalized_date + " " + normalized_time;
            
        } catch (const std::exception& e) {
            LOG(WARNING) << "时间字符串标准化失败: " << time_key << ", 错误: " << e.what();
            return time_key;
        }
    }
    
    // 新增：标准化日期部分
    std::string NormalizeDatePart(const std::string& date_str) const {
        std::vector<std::string> parts;
        std::stringstream ss(date_str);
        std::string item;
        
        while (std::getline(ss, item, '-')) {
            parts.push_back(item);
        }
        
        if (parts.size() != 3) {
            return date_str;
        }
        
        // 补零：YYYY-MM-DD
        std::string year = parts[0];
        std::string month = (parts[1].length() == 1) ? "0" + parts[1] : parts[1];
        std::string day = (parts[2].length() == 1) ? "0" + parts[2] : parts[2];
        
        return year + "-" + month + "-" + day;
    }
    
    // 新增：标准化时间部分
    std::string NormalizeTimePart(const std::string& time_str) const {
        std::vector<std::string> parts;
        std::stringstream ss(time_str);
        std::string item;
        
        while (std::getline(ss, item, ':')) {
            parts.push_back(item);
        }
        
        if (parts.size() != 3) {
            return time_str;
        }
        
        // 补零：HH:MM:SS
        std::string hour = (parts[0].length() == 1) ? "0" + parts[0] : parts[0];
        std::string minute = (parts[1].length() == 1) ? "0" + parts[1] : parts[1];
        std::string second = (parts[2].length() == 1) ? "0" + parts[2] : parts[2];
        
        return hour + ":" + minute + ":" + second;
    }

     void ConvertToTimeStampedData(const std::vector<sad::IMU>& imu_data,
                                   const std::vector<sad::GNSS>& gps_data) {
        all_data_.clear();
        all_data_.reserve(imu_data.size() + gps_data.size());

        for (const auto& imu : imu_data) {
            all_data_.emplace_back(imu);
        }
        for (auto gps : gps_data) {
            gps.unix_time_ += gps_time_offset_;
            all_data_.emplace_back(gps);
        }
    }
};

//离线ESKF
class OfflineESKFProcessor {
private:
    sad::ESKFD eskf_;
    bool first_gps_processed_ = false;
    Vec3d origin_ = Vec3d::Zero();
    std::ofstream correction_file_; // 位置修正量
    std::ofstream lateral_residual_file_; // 横向残差

    // 新增：转弯段信息
    std::vector<std::pair<double, double>> turn_segments_;  // (start_time, end_time)

    // 新增：GPS-NZZ匹配数据存储
    struct MatchedGPSNZZ {
        double gps_timestamp;
        double nzz_heading;
        std::string time_key;
        
        MatchedGPSNZZ(double gps_ts, double heading, const std::string& key)
            : gps_timestamp(gps_ts), nzz_heading(heading), time_key(key) {}
    };

public:
    //初始化ESKF
    bool Initialize(const std::string& correction_output_path) {
        if (!InitializeESKF(eskf_)){
            return false;
        }
        correction_file_.open(correction_output_path);
        if(!correction_file_.is_open()){
            return false;
        }

        std::string lateral_path = correction_output_path.substr(0, correction_output_path.find_last_of('.')) + "_lateral.txt";
        lateral_residual_file_.open(lateral_path);
        if(!lateral_residual_file_.is_open()){
            return false;
        }

        return true;
    }

    //处理重组织后的数据
    bool ProcessReorganizedData(const std::vector<TimeStampedData>& data,
                                const std::string& output_path) {
        std::ofstream fout(output_path);
        std::string cov_path = output_path.substr(0, output_path.find_last_of('.')) + "_cov.txt";
        std::ofstream cov_file(cov_path);
        
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        auto save_result = [&](const sad::NavStated& state, const Vec3d& gps_pos, bool has_gps) {
            fout << std::setprecision(18) << state.timestamp_ << " " << std::setprecision(9);
            save_vec3(fout, state.p_);
            save_quat(fout, state.R_.unit_quaternion());
            save_vec3(fout, state.v_);
            save_vec3(fout, state.bg_);
            save_vec3(fout, state.ba_);
            if (has_gps) {
                save_vec3(fout, gps_pos);
                fout << "1";
            } else {
                fout<< "0 0 0 0";
            }
            fout << std::endl;
        };

        Vec3d latest_gps_pos = Vec3d::Zero();
        bool has_latest_gps = false;

        for (const auto& timestamped_data : data) {
            if (timestamped_data.type == TimeStampedData::IMU_TYPE) {
                if (ProcessIMU(timestamped_data.imu_data, cov_file)){
                    auto state = eskf_.GetNominalState();
                    save_result(state, latest_gps_pos, has_latest_gps);
                }
            } else {
                Vec3d gps_pos;
                if (ProcessGPS(timestamped_data.gps_data, gps_pos)) {
                    latest_gps_pos = gps_pos;
                    has_latest_gps = true;
                    eskf_.SaveCovariance(cov_file);
                }
            }
        }
        return true;
    }

    // 新增：设置转弯段信息
    void SetTurnSegments(const std::vector<TurnDetector::TurnSegment>& segments) {
        turn_segments_.clear();
        for (const auto& segment : segments) {
            turn_segments_.emplace_back(segment.start_time, segment.end_time);
        }
        LOG(INFO) << "设置转弯段信息: " << turn_segments_.size() << " 个转弯段";
    }

    // 新增：设置FBK数据
    void SetFBKData(const std::vector<sad::FBKPair>& fbk_data) {
        for (const auto& fbk_pair : fbk_data) {
            if (fbk_pair.valid_) {
                eskf_.AddFBKData(fbk_pair.flag_.timestamp_, 
                                fbk_pair.misalignment_.pitch_, 
                                fbk_pair.misalignment_.heading_);
            }
        }
        LOG(INFO) << "设置FBK数据: " << fbk_data.size() << " 个FBK数据对";
    }

private:
    bool ProcessIMU(const sad::IMU& imu, std::ofstream& cov_file) {
        //等待第一个GPS
        if(!first_gps_processed_) {
            return false;
        }

        bool success = eskf_.Predict(imu);
        if (success) {
            eskf_.SaveCovariance(cov_file);
        }
        return success;
    }

    // 新增：检查是否在转弯段内
    bool IsInTurnSegment(double timestamp) const {
        for (const auto& segment : turn_segments_) {
            if (timestamp >= segment.first && timestamp <= segment.second) {
                return true;
            }
        }
        return false;
    }

    bool ProcessGPS(const sad::GNSS& gps, Vec3d& gps_pos) {
        sad::GNSS gps_convert = gps;
        if (!sad::ConvertGps2UTM(gps_convert, Vec2d::Zero(), 0.0)) {
            LOG(WARNING) << "GPS坐标转换失败";
            return false;
        }
        if (!first_gps_processed_) {
            origin_ = gps_convert.utm_pose_.translation();
            first_gps_processed_ = true;
        }
        //应用原点偏移
        gps_pos = gps_convert.utm_pose_.translation() - origin_;
        gps_convert.utm_pose_.translation() -= origin_;
        
        Vec3d pos_before = eskf_.GetNominalState().p_;
        Vec3d pos_residual = gps_convert.utm_pose_.translation() - pos_before;

        double lateral_residual = eskf_.ComputeLateralResidual(pos_residual);
        double heading = eskf_.GetCurrentHeading();
        double speed = eskf_.GetNominalState().v_.norm();
        double residual_norm = pos_residual.norm();

        lateral_residual_file_ << std::fixed << std::setprecision(9)
                               << gps.unix_time_ << " "
                               << lateral_residual << " "
                               << heading << " "
                               << speed << " "
                               << pos_residual.x() << " " << pos_residual.y() << " "
                               << residual_norm
                               << std::endl;

        // 新增：根据转弯状态选择观测方式
        bool success = false;
        if (IsInTurnSegment(gps.unix_time_)) {
            // 转弯期间：只做位置观测
            success = eskf_.ObservePositionOnly(gps_convert);
        } else {
            // 直线期间：完整观测
            success = eskf_.ObserveGps(gps_convert);
        }

        if(success) {
            Vec3d pos_after = eskf_.GetNominalState().p_;
            Vec3d pos_correction = pos_after - pos_before;
            double correction_norm = pos_correction.norm();
            double residual_norm = pos_residual.norm();
            correction_file_ << std::fixed << std::setprecision(9)
                             << gps.unix_time_ << " "
                             << pos_correction.x() << " " << pos_correction.y() << " " << pos_correction.z() << " "
                             << correction_norm << " "
                             << pos_residual.x() << " " << pos_residual.y() << " " << pos_residual.z() << " "
                             << residual_norm
                             << std::endl;
        }
        return success;
    }
};

//离线模式
int RunOfflineMode() {
    LOG(INFO) << "离线模式";
    if (FLAGS_enable_turn_detection) {
        LOG(INFO) << "转弯检测: 启用";
    } else {
        LOG(INFO) << "转弯检测: 关闭";
    }
    LOG(INFO) << "GPS时间偏移" << FLAGS_gps_time_offset << "s";
    
    //数据管理器
    OfflineDataManager data_manager;
    data_manager.SetGPSTimeOffset(FLAGS_gps_time_offset);

    if(!data_manager.LoadAndReorganizeData(FLAGS_txt_path)) {
        LOG(ERROR) << "数据加载失败";
        return -1;
    }

    std::string correction_path_ = "corrections";
    if (FLAGS_gps_time_offset != 0.0){
        correction_path_ += "_" + std::to_string(static_cast<int>(FLAGS_gps_time_offset * 1000)) + "ms";
    }
    correction_path_ += ".txt";

    //ESKF处理器
    OfflineESKFProcessor processor;
    if (!processor.Initialize(correction_path_)) {
        LOG(ERROR) << "ESKF初始化失败";
        return -1;
    }

    // 设置FBK数据到处理器
    const auto& fbk_data = data_manager.GetFBKData();
    if (!fbk_data.empty()) {
        processor.SetFBKData(fbk_data);
    }

    // 转弯检测
    std::vector<TurnDetector::TurnSegment> detected_turns;
    if (FLAGS_enable_turn_detection) {
        LOG(INFO) << "开始转弯检测分析...";
        
        // 获取GPS-NZZ匹配数据
        const auto& matched_data = data_manager.GetMatchedHeadingData();
        
        if (matched_data.empty()) {
            LOG(WARNING) << "没有匹配的GPS-NZZ数据，跳过转弯检测";
        } else {
            // 转弯检测器配置
            TurnDetector turn_detector;
            TurnDetector::Config config;
            config.start_turn_rate_threshold = 3.0;
            config.end_turn_rate_threshold = 1.5;
            config.end_duration_threshold = 3.0;
            config.accumulated_angle_threshold = 30.0;
            
            // 转弯检测输出文件名
            std::string turn_output_filename = "turns_offline";
            if (FLAGS_gps_time_offset != 0.0) {
                int offset_ms = static_cast<int>(FLAGS_gps_time_offset * 1000);
                turn_output_filename += "_" + std::to_string(offset_ms) + "ms";
            }
            turn_output_filename += ".txt";
            
            if (!turn_detector.Initialize(turn_output_filename, config)) {
                LOG(ERROR) << "转弯检测器初始化失败";
                return -1;
            }
            
            // 添加匹配的航向数据进行转弯检测
            for (const auto& data : matched_data) {
                turn_detector.AddHeadingData(data.first, data.second);
            }
            
            // 完成转弯检测
            turn_detector.Finalize();

            // 获取检测到的转弯段
            detected_turns = turn_detector.GetDetectedTurns();

            LOG(INFO) << "转弯检测分析完成";
        }
    }

    // 设置转弯段信息到处理器
    if (!detected_turns.empty()) {
        processor.SetTurnSegments(detected_turns);
    }

    std::string output_path = "gins_offline";
    if (FLAGS_gps_time_offset != 0.0) {
        int offset_ms = static_cast<int>(FLAGS_gps_time_offset * 1000);
        output_path += "_" + std::to_string(offset_ms) + "ms";
    }
    output_path += ".txt";

    if (!processor.ProcessReorganizedData(data_manager.GetReorganizedData(), output_path)) {
        LOG(ERROR) << "数据处理失败";
        return -1;
    }

    return 0;
}

int RunRealtimeMode() {
    sad::ESKFD eskf;
    sad::TxtIO io(FLAGS_txt_path);
    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };
    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state,
                                                const Vec3d& gps_pos = Vec3d::Zero(), bool has_gps = false) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        if (has_gps) {
            save_vec3 (fout, gps_pos);
            fout << "1";
        } else {
            fout << "0 0 0 0";
        }
        fout << std::endl;
    };
    std::ofstream fout("/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/gins_realtime.txt");

    // 新增：P矩阵协方差数据文件
    std::ofstream cov_file("/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/covariance_realtime.txt");

    bool imu_inited = false, gnss_inited = false;

    LOG(INFO) << "初始化ESKF";
    if (InitializeESKF(eskf)) {
        imu_inited = true;
    }

    //GNSS缓存队列
    std::queue<sad::GNSS> pending_gps_queue;

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    //存储最新的GPS观测位置
    Vec3d latest_gps_pos = Vec3d::Zero();
    bool has_latest_gps = false;
    double latest_gps_time = 0.0;

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          // 记录IMU预测后的协方差
          eskf.SaveCovariance(cov_file);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto current_state = eskf.GetNominalState();
          double current_eskf_time = current_state.timestamp_;

          //检查是否有GPS数据需要处理
          while (!pending_gps_queue.empty()) {
            sad::GNSS& catch_gps = pending_gps_queue.front();
            //IMU递推到缓存的GNSS时刻
            if (current_eskf_time >= catch_gps.unix_time_) {
                LOG(INFO) << "=== 处理缓存的GPS数据 ===";
                LOG(INFO) << "IMU时间: " << std::fixed << std::setprecision(9) << current_eskf_time
                          << ", GPS时间: " << std::fixed << std::setprecision(9) << catch_gps.unix_time_;
                try{

                    eskf.ObserveGps(catch_gps);

                    // 记录GPS更新后的协方差
                    eskf.SaveCovariance(cov_file);

                    LOG(INFO) << "GPS观测成功, 时间同步正确";
                } catch (...) {
                    LOG (ERROR) << "GNSS观测失败";
                }
                pending_gps_queue.pop();
            }else {
                // IMU还没追上GPS时刻，退出循环
                LOG(INFO) << "等待IMU递推: current=" << std::fixed << std::setprecision(9) << current_eskf_time 
                          << ", waiting_gps=" << catch_gps.unix_time_;
                break;
            }
          }

          //检查是否有时间接近的GPS观测数据
          bool use_gps_obs = false;
          Vec3d gps_obs_pos = Vec3d::Zero();
          if (has_latest_gps) {
              use_gps_obs = true;
              gps_obs_pos = latest_gps_pos;
          }
          /// 记录数据以供绘图
          save_result(fout, current_state, gps_obs_pos, use_gps_obs);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            /// GNSS 处理函数 - 详细调试版本
            if (!imu_inited) {
                LOG(INFO) << "GPS: IMU未初始化，跳过";
                return;
            }
            //添加GNSS时间延迟
            sad::GNSS gnss_convert = gnss;
            gnss_convert.unix_time_ += 0.0;

            auto current_state = eskf.GetNominalState();
            double current_eskf_time = current_state.timestamp_;
            
            LOG(INFO) << "=== GPS数据到达 ===";
            LOG(INFO) << "原始GPS时间: " << gnss.unix_time_ << "s";
            LOG(INFO) << "延迟GPS时间: " << gnss_convert.unix_time_ << "s"; 
            LOG(INFO) << "当前ESKF时间: " << current_eskf_time << "s";
            LOG(INFO) << "时间差: " << (gnss_convert.unix_time_ - current_eskf_time) << "s";

            // 跳过太旧的GPS
            if (gnss_convert.unix_time_ < current_eskf_time - 5.0) {
                LOG(WARNING) << "GPS数据太旧，跳过";
                return;
            }
            if (!sad::ConvertGps2UTM(gnss_convert, Vec2d::Zero(), 0.0)) {
                LOG(WARNING) << "GPS坐标转换失败";
                return;
            }
            /// 设置地图原点（去掉原点）
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
                LOG(INFO) << "设置地图原点: " << origin.transpose();
            } else {
                LOG(INFO) << "步骤6 - 使用已有地图原点";
            }
            
            //保存GPS观测位置（去掉原点）
            Vec3d gps_obs_position = gnss_convert.utm_pose_.translation() - origin;
            latest_gps_pos = gps_obs_position;
            has_latest_gps = true;
            latest_gps_time = gnss_convert.unix_time_;

            LOG(INFO) << "步骤6.5 - 保存GPS观测位置" << gps_obs_position.transpose();

            gnss_convert.utm_pose_.translation() -= origin;
            
            LOG(INFO) << "步骤7 - 应用地图原点后，GPS时间戳: " << gnss_convert.unix_time_ << "s";

            try {
                if (current_eskf_time >= gnss_convert.unix_time_) {
                    LOG(INFO) << "GPS时间不超前, 立即处理";
                    eskf.ObserveGps(gnss_convert);
                    eskf.SaveCovariance(cov_file);
                    LOG(INFO) << "GPS观测成功";
                    gnss_inited = true;
                } else {
                    LOG(INFO) << "GPS时间超前, 缓存等待IMU递推";
                    pending_gps_queue.push(gnss_convert);
                    gnss_inited = true;
                }
            } catch (...) {
                LOG(ERROR) << "GPS观测异常";
            }

            
            LOG(INFO) << "=== GPS处理结束 ===";
        })

        .SetFBKPairProcessFunc([&](const sad::FBKPair& fbk_pair) {
            if (fbk_pair.valid_) {
                eskf.AddFBKData(fbk_pair.flag_.timestamp_, 
                            fbk_pair.misalignment_.pitch_, 
                            fbk_pair.misalignment_.heading_);
                LOG(INFO) << "添加FBK数据: t=" << fbk_pair.flag_.timestamp_ << "s, "
                        << "pitch=" << fbk_pair.misalignment_.pitch_ << "°, "
                        << "heading=" << fbk_pair.misalignment_.heading_ << "°";
            }
        })
        .Go();

    return 0;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_txt_path.empty()) {
        return -1;
    }

    if (FLAGS_offline_mode) {
        return RunOfflineMode();
    } else {
        return RunRealtimeMode();
    }
}