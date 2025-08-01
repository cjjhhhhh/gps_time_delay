#!/usr/bin/env python3
"""
GPS转弯检测脚本 - 基于NZZ航向数据
使用NZZ的高精度航向数据进行转弯检测，移除速度限制
"""

import os
import re
import argparse
import numpy as np
from datetime import datetime
from typing import List, Dict, Tuple


class DataPoint:
    """数据点：GPS时间戳 + NZZ航向"""
    def __init__(self, timestamp: float, heading: float):
        self.timestamp = timestamp
        self.heading = heading


class TurnSegment:
    """转弯段"""
    def __init__(self, start_time: float, end_time: float, 
                 total_angle: float, avg_turn_rate: float, direction: str):
        self.start_time = start_time
        self.end_time = end_time
        self.total_angle = total_angle
        self.avg_turn_rate = avg_turn_rate
        self.direction = direction
    
    @property
    def duration(self) -> float:
        return self.end_time - self.start_time


class TurnDetector:
    """转弯检测器 - 基于NZZ航向数据"""
    
    def __init__(self, start_turn_rate_threshold: float = 3.0, 
                 end_turn_rate_threshold: float = 1.5,
                 end_duration_threshold: float = 3.0,
                 accumulated_angle_threshold: float = 30.0):
        """
        初始化转弯检测器
        
        Args:
            start_turn_rate_threshold: 开始转弯的转弯率阈值 (度/秒)
            end_turn_rate_threshold: 结束转弯的转弯率阈值 (度/秒)
            end_duration_threshold: 结束判断的持续时间 (秒)
            accumulated_angle_threshold: 累积角度阈值 (度)
        """
        self.start_turn_rate_threshold = start_turn_rate_threshold
        self.end_turn_rate_threshold = end_turn_rate_threshold
        self.end_duration_threshold = end_duration_threshold
        self.accumulated_angle_threshold = accumulated_angle_threshold
    
    def parse_log_data(self, log_file: str) -> List[DataPoint]:
        """
        解析日志文件，提取GPS时间戳和NZZ航向数据
        
        Args:
            log_file: 日志文件路径
            
        Returns:
            数据点列表 (GPS时间戳, NZZ航向)
        """
        gps_data = {}  # 时间秒 -> (时间戳, 时间字符串)
        nzz_data = {}  # 时间秒 -> 航向角
        
        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    
                    try:
                        if line.startswith('$GPS'):
                            self._parse_gps_line(line, gps_data)
                        elif line.startswith('$NZZ'):
                            self._parse_nzz_line(line, nzz_data)
                    
                    except (ValueError, IndexError) as e:
                        print(f"警告: 第{line_num}行数据解析失败: {e}")
                        continue
        
        except FileNotFoundError:
            print(f"错误: 文件不存在 {log_file}")
            return []
        except Exception as e:
            print(f"错误: 读取文件失败 {e}")
            return []
        
        # 匹配GPS和NZZ数据
        matched_data = self._match_gps_nzz_data(gps_data, nzz_data)
        
        print(f"解析完成: GPS数据 {len(gps_data)} 条, NZZ数据 {len(nzz_data)} 条, 匹配数据 {len(matched_data)} 条")
        return matched_data
    
    def _parse_gps_line(self, line: str, gps_data: dict):
        """解析GPS行数据"""
        fields = line.split()
        if len(fields) < 25:
            return
        
        # 提取GPS时间戳
        timestamp = float(fields[1]) / 1000.0  # 毫秒转秒
        
        # 提取GPS时间：年月日时分秒
        try:
            year = int(fields[19])
            month = int(fields[20])
            day = int(fields[21])
            hour = int(fields[22])
            minute = int(fields[23])
            second = int(fields[24])
            
            # 构造时间字符串（精确到秒）
            time_key = f"{year}-{month}-{day} {hour}:{minute}:{second}"
            
            # 只保存每秒的第一个GPS数据
            if time_key not in gps_data:
                gps_data[time_key] = (timestamp, time_key)
                
        except (ValueError, IndexError):
            return
    
    def _parse_nzz_line(self, line: str, nzz_data: dict):
        """解析NZZ行数据"""
        fields = line.split()
        if len(fields) < 20:  # 确保有足够的字段
            return
        
        try:
            # 提取NZZ时间：YYYY-M-D H:M:S
            nzz_time_str = f"{fields[1]} {fields[2]}"
            
            # 标准化时间格式，提取到秒级
            # fields[1] 格式: "2025-6-12"
            # fields[2] 格式: "11:22:27"
            time_key = nzz_time_str
            
            # 提取航向角（第12个字段，从0开始计数）
            # $NZZ 2025-6-12 11:22:27 ... 271.862000 ...
            #  0      1        2      ...    12     ...
            heading = float(fields[12])
            
            # 只保存每秒的第一个NZZ数据
            if time_key not in nzz_data:
                nzz_data[time_key] = heading
                
        except (ValueError, IndexError):
            return
    
    def _match_gps_nzz_data(self, gps_data: dict, nzz_data: dict) -> List[DataPoint]:
        """匹配GPS和NZZ数据"""
        matched_data = []
        
        for gps_time_key, (gps_timestamp, _) in gps_data.items():
            # 查找对应的NZZ数据
            # GPS: "2025-5-12 17:40:4"
            # NZZ: "2025-5-12 17:40:27"
            # 需要匹配到相同的年月日时分秒
            
            if gps_time_key in nzz_data:
                # 直接匹配
                nzz_heading = nzz_data[gps_time_key]
                matched_data.append(DataPoint(gps_timestamp, nzz_heading))
            else:
                # 尝试模糊匹配（处理时间格式差异）
                gps_normalized = self._normalize_time_key(gps_time_key)
                for nzz_time_key, nzz_heading in nzz_data.items():
                    nzz_normalized = self._normalize_time_key(nzz_time_key)
                    if gps_normalized == nzz_normalized:
                        matched_data.append(DataPoint(gps_timestamp, nzz_heading))
                        break
        
        # 按时间戳排序
        matched_data.sort(key=lambda x: x.timestamp)
        return matched_data
    
    def _normalize_time_key(self, time_key: str) -> str:
        """标准化时间字符串格式"""
        try:
            # 解析时间字符串
            if '-' in time_key and ':' in time_key:
                # 分离日期和时间部分
                date_part, time_part = time_key.split(' ')
                
                # 处理日期部分：YYYY-M-D -> YYYY-MM-DD
                year, month, day = date_part.split('-')
                normalized_date = f"{year}-{month.zfill(2)}-{day.zfill(2)}"
                
                # 处理时间部分：H:M:S -> HH:MM:SS
                hour, minute, second = time_part.split(':')
                normalized_time = f"{hour.zfill(2)}:{minute.zfill(2)}:{second.zfill(2)}"
                
                return f"{normalized_date} {normalized_time}"
        except:
            pass
        
        return time_key
    
    def normalize_heading_diff(self, h1: float, h2: float) -> float:
        """
        处理航向角360度跳变问题
        
        Args:
            h1: 前一个航向角
            h2: 当前航向角
            
        Returns:
            标准化的航向角差值 (-180, 180]
        """
        diff = h2 - h1
        if diff > 180:
            diff -= 360
        elif diff <= -180:
            diff += 360
        return diff
    
    def calculate_turn_rates(self, data_points: List[DataPoint]) -> List[Tuple[float, float]]:
        """
        计算转弯率序列
        
        Args:
            data_points: 数据点列表
            
        Returns:
            (时间戳, 转弯率)的列表
        """
        turn_rates = []
        
        for i in range(1, len(data_points)):
            curr = data_points[i]
            prev = data_points[i-1]
            
            dt = curr.timestamp - prev.timestamp
            if dt <= 0:
                continue
            
            # 计算航向角变化
            dh = self.normalize_heading_diff(prev.heading, curr.heading)
            
            # 计算转弯率 (度/秒)
            turn_rate = dh / dt
            
            turn_rates.append((curr.timestamp, turn_rate))
        
        return turn_rates
    
    def smooth_turn_rates(self, turn_rates: List[Tuple[float, float]], 
                         window_size: int = 5) -> List[Tuple[float, float]]:
        """
        对转弯率进行移动平均平滑
        
        Args:
            turn_rates: 原始转弯率数据 (时间戳, 转弯率)
            window_size: 滑动窗口大小
            
        Returns:
            平滑后的数据 (时间戳, 平滑转弯率)
        """
        if len(turn_rates) < window_size:
            return turn_rates
        
        smoothed = []
        for i in range(len(turn_rates)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(turn_rates), i + window_size // 2 + 1)
            
            window_rates = [rate for _, rate in turn_rates[start_idx:end_idx]]
            avg_rate = np.mean(window_rates)
            
            # 保持原始时间戳
            smoothed.append((turn_rates[i][0], avg_rate))
        
        return smoothed
    
    def detect_turn_segments(self, data_points: List[DataPoint]) -> List[TurnSegment]:
        """
        基于累积角度检测转弯段（无速度限制）
        
        Args:
            data_points: 数据点列表
            
        Returns:
            检测到的转弯段列表
        """
        if len(data_points) < 2:
            return []
        
        # 计算转弯率
        turn_rates = self.calculate_turn_rates(data_points)
        
        # 平滑处理减少噪声
        smoothed_rates = self.smooth_turn_rates(turn_rates, window_size=5)
        
        detected_turns = []
        
        # 状态变量
        in_turn = False              # 是否在转弯状态
        in_end_timing = False        # 是否在结束计时状态
        turn_start_idx = 0           # 转弯开始索引
        accumulated_angle = 0.0      # 累积转角
        turn_rates_list = []         # 转弯率列表（用于计算平均值）
        turn_direction = None        # 转弯方向（"left" 或 "right"）
        end_timing_start = 0.0       # 结束计时开始时间
        
        print(f"开始转弯检测，参数设置：")
        print(f"  开始阈值: {self.start_turn_rate_threshold}°/s")
        print(f"  结束阈值: {self.end_turn_rate_threshold}°/s，持续{self.end_duration_threshold}s")
        print(f"  累积角度阈值: {self.accumulated_angle_threshold}°")
        print(f"  无速度限制")
        
        for i, (timestamp, turn_rate) in enumerate(smoothed_rates):
            abs_turn_rate = abs(turn_rate)
            
            if not in_turn:
                # 状态1: 监听状态 - 检查是否开始转弯
                if abs_turn_rate > self.start_turn_rate_threshold:
                    # 开始新的转弯
                    in_turn = True
                    in_end_timing = False
                    turn_start_idx = i
                    accumulated_angle = 0.0
                    turn_rates_list = [turn_rate]
                    turn_direction = "left" if turn_rate > 0 else "right"
                    
                    print(f"  时间 {timestamp:.1f}s: 开始{turn_direction}转弯，转弯率 {abs_turn_rate:.2f}°/s")
                    
            else:
                # 在转弯状态中
                if not in_end_timing:
                    # 状态2: 累积状态
                    if abs_turn_rate > self.end_turn_rate_threshold:
                        # 继续转弯 - 累积角度
                        if i > 0:
                            dt = timestamp - smoothed_rates[i-1][0]
                            angle_change = turn_rate * dt
                            
                            # 检查是否与主转弯方向一致
                            if (turn_direction == "left" and turn_rate > 0) or \
                               (turn_direction == "right" and turn_rate < 0):
                                accumulated_angle += abs(angle_change)
                            else:
                                # 反向转弯，考虑是否需要重置
                                if abs(turn_rate) > self.start_turn_rate_threshold:
                                    # 方向明显改变，检查当前累积角度
                                    if accumulated_angle >= self.accumulated_angle_threshold:
                                        # 当前累积已足够，记录转弯
                                        self._record_turn(detected_turns, smoothed_rates, 
                                                        turn_start_idx, i-1, 
                                                        accumulated_angle, turn_rates_list, turn_direction)
                                    
                                    # 重新开始新方向的转弯
                                    turn_start_idx = i
                                    accumulated_angle = abs(angle_change)
                                    turn_rates_list = [turn_rate]
                                    turn_direction = "left" if turn_rate > 0 else "right"
                                    print(f"  时间 {timestamp:.1f}s: 转弯方向改变，重新开始{turn_direction}转弯")
                        
                        turn_rates_list.append(turn_rate)
                        
                    else:
                        # 转弯率降到结束阈值以下，开始结束计时
                        in_end_timing = True
                        end_timing_start = timestamp
                        print(f"  时间 {timestamp:.1f}s: 转弯率降到 {abs_turn_rate:.2f}°/s，开始结束计时")
                
                else:
                    # 状态3: 结束判断状态
                    if abs_turn_rate <= self.end_turn_rate_threshold:
                        # 继续结束计时
                        end_duration = timestamp - end_timing_start
                        
                        if end_duration >= self.end_duration_threshold:
                            # 结束计时达到要求，检查累积角度
                            if accumulated_angle >= self.accumulated_angle_threshold:
                                # 累积角度足够，记录有效转弯
                                self._record_turn(detected_turns, smoothed_rates, 
                                                turn_start_idx, i, 
                                                accumulated_angle, turn_rates_list, turn_direction)
                                print(f"  时间 {timestamp:.1f}s: 记录有效转弯，累积角度 {accumulated_angle:.1f}°")
                            else:
                                print(f"  时间 {timestamp:.1f}s: 累积角度不足 {accumulated_angle:.1f}°，丢弃转弯")
                            
                            # 重置状态
                            in_turn = False
                            in_end_timing = False
                            
                    else:
                        # 转弯率又超过了结束阈值，回到累积状态
                        in_end_timing = False
                        # 继续累积
                        if i > 0:
                            dt = timestamp - smoothed_rates[i-1][0]
                            angle_change = turn_rate * dt
                            if (turn_direction == "left" and turn_rate > 0) or \
                               (turn_direction == "right" and turn_rate < 0):
                                accumulated_angle += abs(angle_change)
                        
                        turn_rates_list.append(turn_rate)
                        print(f"  时间 {timestamp:.1f}s: 转弯率回升到 {abs_turn_rate:.2f}°/s，继续累积")
        
        # 处理文件结尾的转弯
        if in_turn and len(smoothed_rates) > turn_start_idx:
            if accumulated_angle >= self.accumulated_angle_threshold:
                self._record_turn(detected_turns, smoothed_rates, 
                                turn_start_idx, len(smoothed_rates)-1, 
                                accumulated_angle, turn_rates_list, turn_direction)
                print(f"  文件结尾: 记录最后转弯，累积角度 {accumulated_angle:.1f}°")
        
        return detected_turns
    
    def _record_turn(self, detected_turns: List[TurnSegment], 
                    smoothed_rates: List[Tuple[float, float]], 
                    start_idx: int, end_idx: int, 
                    accumulated_angle: float, turn_rates_list: List[float], 
                    turn_direction: str):
        """
        记录一个转弯段
        """
        start_time = smoothed_rates[start_idx][0]
        end_time = smoothed_rates[end_idx][0]
        avg_turn_rate = np.mean([abs(r) for r in turn_rates_list])
        direction = "左转" if turn_direction == "left" else "右转"
        
        turn_segment = TurnSegment(
            start_time=start_time,
            end_time=end_time,
            total_angle=accumulated_angle,
            avg_turn_rate=avg_turn_rate,
            direction=direction
        )
        detected_turns.append(turn_segment)
    
    def save_results(self, turn_segments: List[TurnSegment], 
                    output_file: str, log_filename: str):
        """
        保存转弯检测结果
        
        Args:
            turn_segments: 检测到的转弯段
            output_file: 输出文件路径
            log_filename: 原始日志文件名
        """
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(f"# 转弯段检测结果 - 基于NZZ航向数据\n")
                f.write(f"# 源文件: {log_filename}\n")
                f.write(f"# 检测参数:\n")
                f.write(f"#   开始转弯阈值: {self.start_turn_rate_threshold}°/s\n")
                f.write(f"#   结束转弯阈值: {self.end_turn_rate_threshold}°/s，持续{self.end_duration_threshold}s\n")
                f.write(f"#   累积角度阈值: {self.accumulated_angle_threshold}°\n")
                f.write(f"#   数据源: NZZ航向数据（无速度限制）\n")
                f.write(f"# 检测到 {len(turn_segments)} 个转弯段\n")
                f.write("#\n")
                f.write("# 转弯ID,起始时间戳,结束时间戳,持续时间(s),累积角度(度),平均转弯率(度/s),转弯方向\n")
                
                for i, turn in enumerate(turn_segments, 1):
                    f.write(f"{i},{turn.start_time:.3f},{turn.end_time:.3f},")
                    f.write(f"{turn.duration:.1f},{turn.total_angle:.1f},")
                    f.write(f"{turn.avg_turn_rate:.2f},{turn.direction}\n")
            
            print(f"结果已保存到: {output_file}")
            
        except Exception as e:
            print(f"错误: 保存结果失败 {e}")


def process_single_file(log_file: str, output_dir: str, detector: TurnDetector):
    """
    处理单个日志文件
    
    Args:
        log_file: 日志文件路径
        output_dir: 输出目录
        detector: 转弯检测器
    """
    log_filename = os.path.basename(log_file)
    base_name = os.path.splitext(log_filename)[0]
    
    print(f"\n处理文件: {log_filename}")
    
    # 解析GPS+NZZ数据
    data_points = detector.parse_log_data(log_file)
    if not data_points:
        print(f"警告: 文件 {log_filename} 没有有效的匹配数据")
        return
    
    # 检测转弯段
    turn_segments = detector.detect_turn_segments(data_points)
    
    print(f"\n检测结果:")
    print(f"检测到 {len(turn_segments)} 个转弯段:")
    for i, turn in enumerate(turn_segments, 1):
        print(f"  转弯{i}: {turn.start_time:.1f}s - {turn.end_time:.1f}s "
              f"({turn.duration:.1f}s, {turn.direction}, {turn.total_angle:.1f}°, {turn.avg_turn_rate:.2f}°/s)")
    
    # 保存结果
    output_file = os.path.join(output_dir, f"{base_name}_turns_nzz.txt")
    detector.save_results(turn_segments, output_file, log_filename)


def main():
    parser = argparse.ArgumentParser(description='GPS转弯检测工具 - 基于NZZ航向数据')
    parser.add_argument('--input', '-i', required=True,
                       help='输入文件或目录路径')
    parser.add_argument('--output', '-o', required=True,
                       help='输出目录路径')
    parser.add_argument('--start_threshold', type=float, default=3.0,
                       help='开始转弯阈值 (度/秒), 默认3.0')
    parser.add_argument('--end_threshold', type=float, default=1.5,
                       help='结束转弯阈值 (度/秒), 默认1.5')
    parser.add_argument('--end_duration', type=float, default=3.0,
                       help='结束判断持续时间 (秒), 默认3.0')
    parser.add_argument('--angle_threshold', type=float, default=30.0,
                       help='累积角度阈值 (度), 默认30.0')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 创建转弯检测器
    detector = TurnDetector(
        start_turn_rate_threshold=args.start_threshold,
        end_turn_rate_threshold=args.end_threshold,
        end_duration_threshold=args.end_duration,
        accumulated_angle_threshold=args.angle_threshold
    )
    
    print(f"转弯检测参数:")
    print(f"  开始转弯阈值: {args.start_threshold}°/s")
    print(f"  结束转弯阈值: {args.end_threshold}°/s，持续{args.end_duration}s")
    print(f"  累积角度阈值: {args.angle_threshold}°")
    print(f"  数据源: NZZ航向数据（无速度限制）")
    
    # 处理文件
    if os.path.isfile(args.input):
        # 单个文件
        process_single_file(args.input, args.output, detector)
    elif os.path.isdir(args.input):
        # 目录中的所有.log文件
        log_files = [f for f in os.listdir(args.input) if f.endswith('.log')]
        if not log_files:
            print(f"错误: 目录 {args.input} 中没有找到.log文件")
            return
        
        print(f"找到 {len(log_files)} 个日志文件")
        
        for log_file in sorted(log_files):
            full_path = os.path.join(args.input, log_file)
            try:
                process_single_file(full_path, args.output, detector)
            except Exception as e:
                print(f"错误: 处理文件 {log_file} 失败: {e}")
    else:
        print(f"错误: 输入路径 {args.input} 不存在")


if __name__ == "__main__":
    main()