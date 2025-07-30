//
// 转弯检测器实现 - 与Python版本完全一致
//

#include "turn_detector.h"
#include <cmath>
#include <numeric>
#include <iomanip>
#include <algorithm>

bool TurnDetector::Initialize(const std::string& output_file, const Config& config) {
    if (initialized_) {
        LOG(WARNING) << "TurnDetector已经初始化过";
        return true;
    }
    
    output_file_ = output_file;
    config_ = config;
    
    // 清空数据
    heading_data_.clear();
    turn_rates_.clear();
    detected_turns_.clear();
    
    // 重置状态
    in_turn_ = false;
    in_end_timing_ = false;
    accumulated_angle_ = 0.0;
    turn_rates_list_.clear();
    
    initialized_ = true;
    return true;
}

void TurnDetector::AddHeadingData(double timestamp, double heading) {
    if (!initialized_) {
        LOG(WARNING) << "TurnDetector未初始化，跳过数据";
        return;
    }
    
    // 标准化航向角到[0, 360)
    while (heading < 0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    
    heading_data_.emplace_back(timestamp, heading);
}

double TurnDetector::NormalizeHeadingDiff(double h1, double h2) const {
    double diff = h2 - h1;
    if (diff > 180.0) {
        diff -= 360.0;
    } else if (diff <= -180.0) {
        diff += 360.0;
    }
    return diff;
}

void TurnDetector::CalculateTurnRates() {
    turn_rates_.clear();
    
    if (heading_data_.size() < 2) {
        LOG(WARNING) << "航向数据点不足，无法计算转弯率";
        return;
    }
    
    // 对应Python的calculate_turn_rates函数
    for (size_t i = 1; i < heading_data_.size(); ++i) {
        const auto& curr = heading_data_[i];
        const auto& prev = heading_data_[i-1];
        
        double dt = curr.timestamp - prev.timestamp;
        if (dt <= 0.0) {
            continue;
        }
        
        // 计算航向角变化
        double dh = NormalizeHeadingDiff(prev.heading, curr.heading);
        
        // 计算转弯率 (度/秒)
        double turn_rate = dh / dt;
        
        turn_rates_.emplace_back(curr.timestamp, turn_rate);
    }
    
}

std::vector<TurnDetector::TurnRatePoint> TurnDetector::SmoothTurnRates(const std::vector<TurnRatePoint>& turn_rates) {
    if (turn_rates.size() < config_.smoothing_window_size) {
        return turn_rates;  // 数据点不足，直接返回
    }
    
    std::vector<TurnRatePoint> smoothed;
    smoothed.reserve(turn_rates.size());
    
    // 对应Python的smooth_turn_rates函数
    for (size_t i = 0; i < turn_rates.size(); ++i) {
        int start_idx = std::max(0, static_cast<int>(i) - config_.smoothing_window_size / 2);
        int end_idx = std::min(static_cast<int>(turn_rates.size()), 
                              static_cast<int>(i) + config_.smoothing_window_size / 2 + 1);
        
        double sum = 0.0;
        int count = 0;
        for (int j = start_idx; j < end_idx; ++j) {
            sum += turn_rates[j].turn_rate;
            count++;
        }
        
        double avg_rate = (count > 0) ? sum / count : 0.0;
        
        // 保持原始时间戳
        smoothed.emplace_back(turn_rates[i].timestamp, avg_rate);
    }
    
    return smoothed;
}

void TurnDetector::DetectTurnSegments() {
    if (turn_rates_.empty()) {
        LOG(WARNING) << "没有转弯率数据，无法进行转弯检测";
        return;
    }
    
    // 平滑处理减少噪声
    auto smoothed_rates = SmoothTurnRates(turn_rates_);
    
    detected_turns_.clear();
    
    // 状态变量 - 对应Python版本
    bool in_turn = false;
    bool in_end_timing = false;
    int turn_start_idx = 0;
    double accumulated_angle = 0.0;
    std::vector<double> turn_rates_list;
    std::string turn_direction;
    double end_timing_start = 0.0;
    
    for (size_t i = 0; i < smoothed_rates.size(); ++i) {
        double timestamp = smoothed_rates[i].timestamp;
        double turn_rate = smoothed_rates[i].turn_rate;
        double abs_turn_rate = std::abs(turn_rate);
        
        if (!in_turn) {
            // 状态1: 监听状态 - 检查是否开始转弯
            if (abs_turn_rate > config_.start_turn_rate_threshold) {
                // 开始新的转弯
                in_turn = true;
                in_end_timing = false;
                turn_start_idx = i;
                accumulated_angle = 0.0;
                turn_rates_list.clear();
                turn_rates_list.push_back(turn_rate);
                turn_direction = (turn_rate > 0) ? "左转" : "右转";
            }
        } else {
            // 在转弯状态中
            if (!in_end_timing) {
                // 状态2: 累积状态
                if (abs_turn_rate > config_.end_turn_rate_threshold) {
                    // 继续转弯 - 累积角度
                    if (i > 0) {
                        double dt = timestamp - smoothed_rates[i-1].timestamp;
                        double angle_change = turn_rate * dt;
                        
                        // 检查是否与主转弯方向一致
                        if ((turn_direction == "左转" && turn_rate > 0) || 
                            (turn_direction == "右转" && turn_rate < 0)) {
                            accumulated_angle += std::abs(angle_change);
                        } else {
                            // 反向转弯，考虑是否需要重置
                            if (abs_turn_rate > config_.start_turn_rate_threshold) {
                                // 方向明显改变，检查当前累积角度
                                if (accumulated_angle >= config_.accumulated_angle_threshold) {
                                    // 当前累积已足够，记录转弯
                                    RecordTurnSegment(smoothed_rates, turn_start_idx, i-1, 
                                                    accumulated_angle, turn_rates_list, turn_direction);
                                }
                                
                                // 重新开始新方向的转弯
                                turn_start_idx = i;
                                accumulated_angle = std::abs(angle_change);
                                turn_rates_list.clear();
                                turn_rates_list.push_back(turn_rate);
                                turn_direction = (turn_rate > 0) ? "左转" : "右转";
                            }
                        }
                    }
                    
                    turn_rates_list.push_back(turn_rate);
                } else {
                    // 转弯率降到结束阈值以下，开始结束计时
                    in_end_timing = true;
                    end_timing_start = timestamp;
                }
            } else {
                // 状态3: 结束判断状态
                if (abs_turn_rate <= config_.end_turn_rate_threshold) {
                    // 继续结束计时
                    double end_duration = timestamp - end_timing_start;
                    
                    if (end_duration >= config_.end_duration_threshold) {
                        // 结束计时达到要求，检查累积角度
                        if (accumulated_angle >= config_.accumulated_angle_threshold) {
                            // 累积角度足够，记录有效转弯
                            RecordTurnSegment(smoothed_rates, turn_start_idx, i, 
                                            accumulated_angle, turn_rates_list, turn_direction);
                        } else {
                            LOG(INFO) << "  时间 " << std::fixed << std::setprecision(1) 
                                      << timestamp << "s: 累积角度不足 " 
                                      << std::setprecision(1) << accumulated_angle << "°，丢弃转弯";
                        }
                        
                        // 重置状态
                        in_turn = false;
                        in_end_timing = false;
                    }
                } else {
                    // 转弯率又超过了结束阈值，回到累积状态
                    in_end_timing = false;
                    // 继续累积
                    if (i > 0) {
                        double dt = timestamp - smoothed_rates[i-1].timestamp;
                        double angle_change = turn_rate * dt;
                        if ((turn_direction == "左转" && turn_rate > 0) || 
                            (turn_direction == "右转" && turn_rate < 0)) {
                            accumulated_angle += std::abs(angle_change);
                        }
                    }
                    
                    turn_rates_list.push_back(turn_rate);
                }
            }
        }
    }
    
    // 处理文件结尾的转弯
    if (in_turn && !smoothed_rates.empty()) {
        if (accumulated_angle >= config_.accumulated_angle_threshold) {
            RecordTurnSegment(smoothed_rates, turn_start_idx, smoothed_rates.size()-1, 
                            accumulated_angle, turn_rates_list, turn_direction);
            LOG(INFO) << "  文件结尾: 记录最后转弯，累积角度 " 
                      << std::fixed << std::setprecision(1) << accumulated_angle << "°";
        }
    }
    
    LOG(INFO) << "转弯检测完成，共检测到 " << detected_turns_.size() << " 个转弯段";
}

void TurnDetector::RecordTurnSegment(const std::vector<TurnRatePoint>& smoothed_rates, 
                                   int start_idx, int end_idx,
                                   double accumulated_angle, 
                                   const std::vector<double>& turn_rates_list, 
                                   const std::string& turn_direction) {
    double start_time = smoothed_rates[start_idx].timestamp;
    double end_time = smoothed_rates[end_idx].timestamp;
    
    // 计算平均转弯率
    double avg_turn_rate = 0.0;
    if (!turn_rates_list.empty()) {
        double sum = 0.0;
        for (double rate : turn_rates_list) {
            sum += std::abs(rate);
        }
        avg_turn_rate = sum / turn_rates_list.size();
    }
    
    detected_turns_.emplace_back(start_time, end_time, accumulated_angle, avg_turn_rate, turn_direction);
    
    LOG(INFO) << "记录转弯段: " << std::fixed << std::setprecision(1)
              << start_time << "s - " << end_time << "s "
              << "(" << (end_time - start_time) << "s, " << turn_direction 
              << ", " << std::setprecision(1) << accumulated_angle << "°, " 
              << std::setprecision(2) << avg_turn_rate << "°/s)";
}

void TurnDetector::Finalize() {
    if (!initialized_) {
        LOG(WARNING) << "TurnDetector未初始化";
        return;
    }
    
    if (heading_data_.size() < 2) {
        LOG(WARNING) << "航向数据点不足: " << heading_data_.size();
        return;
    }
    
    LOG(INFO) << "开始处理航向数据: " << heading_data_.size() << " 个数据点";
    
    // 按时间戳排序
    std::sort(heading_data_.begin(), heading_data_.end(), 
              [](const HeadingDataPoint& a, const HeadingDataPoint& b) {
                  return a.timestamp < b.timestamp;
              });
    
    // 计算转弯率
    CalculateTurnRates();
    
    // 检测转弯段
    DetectTurnSegments();
    
    // 保存结果
    if (!SaveResults()) {
        LOG(ERROR) << "保存转弯检测结果失败";
    }
    
}

bool TurnDetector::SaveResults() {
    try {
        std::ofstream file(output_file_);
        if (!file.is_open()) {
            LOG(ERROR) << "无法打开输出文件: " << output_file_;
            return false;
        }
        
        // 写入文件头
        file << "# 转弯段检测结果 - 基于ESKF航向数据\n";
        file << "# 检测参数:\n";
        file << "#   开始转弯阈值: " << config_.start_turn_rate_threshold << "°/s\n";
        file << "#   结束转弯阈值: " << config_.end_turn_rate_threshold << "°/s，持续" 
             << config_.end_duration_threshold << "s\n";
        file << "#   累积角度阈值: " << config_.accumulated_angle_threshold << "°\n";
        file << "#   数据源: ESKF航向数据\n";
        file << "# 检测到 " << detected_turns_.size() << " 个转弯段\n";
        file << "#\n";
        file << "# 转弯ID,起始时间戳,结束时间戳,持续时间(s),累积角度(度),平均转弯率(度/s),转弯方向\n";
        
        // 写入检测结果
        for (size_t i = 0; i < detected_turns_.size(); ++i) {
            const auto& turn = detected_turns_[i];
            file << (i + 1) << ","
                 << std::fixed << std::setprecision(3) << turn.start_time << ","
                 << std::setprecision(3) << turn.end_time << ","
                 << std::setprecision(1) << turn.Duration() << ","
                 << std::setprecision(1) << turn.total_angle << ","
                 << std::setprecision(2) << turn.avg_turn_rate << ","
                 << turn.direction << "\n";
        }
        
        file.close();
        LOG(INFO) << "转弯检测结果已保存到: " << output_file_;
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "保存结果时发生异常: " << e.what();
        return false;
    }
}