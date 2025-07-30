//
// 转弯检测器 - 基于ESKF航向数据
// Created for GNSS/INS integrated navigation
//

#ifndef TURN_DETECTOR_H
#define TURN_DETECTOR_H

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <glog/logging.h>

/**
 * 转弯检测器 - 与Python版本完全一致的实现
 */
class TurnDetector {
public:
    /**
     * 转弯段数据结构
     */
    struct TurnSegment {
        double start_time;
        double end_time;
        double total_angle;     // 累积角度(度)
        double avg_turn_rate;   // 平均转弯率(度/秒)
        std::string direction;  // "左转" 或 "右转"
        
        TurnSegment(double st, double et, double angle, double rate, const std::string& dir)
            : start_time(st), end_time(et), total_angle(angle), avg_turn_rate(rate), direction(dir) {}
        
        double Duration() const { return end_time - start_time; }
    };

    /**
     * 航向数据点
     */
    struct HeadingDataPoint {
        double timestamp;
        double heading;  // 航向角(度)
        
        HeadingDataPoint(double t, double h) : timestamp(t), heading(h) {}
    };

    /**
     * 转弯率数据点
     */
    struct TurnRatePoint {
        double timestamp;
        double turn_rate;  // 转弯率(度/秒)
        
        TurnRatePoint(double t, double rate) : timestamp(t), turn_rate(rate) {}
    };

    /**
     * 检测器配置参数
     */
    struct Config {
        double start_turn_rate_threshold;    // 开始转弯阈值(度/秒)
        double end_turn_rate_threshold;      // 结束转弯阈值(度/秒)
        double end_duration_threshold;       // 结束判断持续时间(秒)
        double accumulated_angle_threshold;  // 累积角度阈值(度)
        int smoothing_window_size;           // 平滑窗口大小
        
        // 构造函数设置默认值
        Config() : start_turn_rate_threshold(3.0),
                   end_turn_rate_threshold(1.5),
                   end_duration_threshold(3.0),
                   accumulated_angle_threshold(30.0),
                   smoothing_window_size(5) {}
    };

public:
    TurnDetector() = default;
    ~TurnDetector() = default;

    /**
     * 初始化转弯检测器
     */
    bool Initialize(const std::string& output_file, const Config& config = Config());

    /**
     * 添加航向数据点
     */
    void AddHeadingData(double timestamp, double heading);

    /**
     * 检查当前是否在转弯状态
     */
    bool IsInTurn() const { return in_turn_; }

    /**
     * 获取当前累积转角
     */
    double GetAccumulatedAngle() const { return accumulated_angle_; }

    /**
     * 完成检测，输出最终结果
     */
    void Finalize();

    /**
     * 获取检测到的转弯段
     */
    const std::vector<TurnSegment>& GetDetectedTurns() const {
        return detected_turns_;
    }

private:
    /**
     * 标准化航向角差值，处理360度跳变 - 对应Python的normalize_heading_diff
     */
    double NormalizeHeadingDiff(double h1, double h2) const;

    /**
     * 计算转弯率序列 - 对应Python的calculate_turn_rates
     */
    void CalculateTurnRates();

    /**
     * 对转弯率进行移动平均平滑 - 对应Python的smooth_turn_rates
     */
    std::vector<TurnRatePoint> SmoothTurnRates(const std::vector<TurnRatePoint>& turn_rates);

    /**
     * 检测转弯段 - 对应Python的detect_turn_segments
     */
    void DetectTurnSegments();

    /**
     * 记录转弯段
     */
    void RecordTurnSegment(const std::vector<TurnRatePoint>& smoothed_rates, 
                          int start_idx, int end_idx,
                          double accumulated_angle, 
                          const std::vector<double>& turn_rates_list, 
                          const std::string& turn_direction);

    /**
     * 保存检测结果
     */
    bool SaveResults();

private:
    // 配置参数
    Config config_;
    std::string output_file_;
    
    // 数据存储
    std::vector<HeadingDataPoint> heading_data_;
    std::vector<TurnRatePoint> turn_rates_;
    std::vector<TurnSegment> detected_turns_;
    
    // 检测状态
    bool initialized_ = false;
    bool in_turn_ = false;              // 是否在转弯状态
    bool in_end_timing_ = false;        // 是否在结束计时状态
    
    double turn_start_time_ = 0.0;      // 转弯开始时间
    double accumulated_angle_ = 0.0;    // 累积转角
    double end_timing_start_ = 0.0;     // 结束计时开始时间
    std::string turn_direction_;        // 当前转弯方向
    
    std::vector<double> turn_rates_list_;  // 转弯率历史，用于计算平均值
};

#endif // TURN_DETECTOR_H