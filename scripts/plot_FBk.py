#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
日志文件安装角解析和绘制脚本
解析.log文件中的$SINS和$FBK数据，提取时间戳和安装角信息
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse

class LogParser:
    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        self.timestamps = []
        self.pitch_angles = []
        self.heading_angles = []
        
    def parse_log(self):
        """解析日志文件，提取时间戳和安装角信息"""
        print(f"正在解析日志文件: {self.log_file_path}")
        
        current_timestamp = None
        
        try:
            with open(self.log_file_path, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    
                    # 解析$FBK flag行获取时间戳
                    if line.startswith('$FBK flag'):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                current_timestamp = int(parts[2])
                            except ValueError:
                                print(f"警告：第{line_num}行时间戳格式错误: {parts[2]}")
                                continue
                    
                    # 解析$FBK misalignment行获取安装角
                    elif line.startswith('$FBK misalignment'):
                        if current_timestamp is not None:
                            # 解析pitch和heading角度
                            pitch_match = re.search(r'pitch:([-+]?\d*\.?\d+)', line)
                            heading_match = re.search(r'heading:([-+]?\d*\.?\d+)', line)
                            
                            if pitch_match and heading_match:
                                pitch = float(pitch_match.group(1))
                                heading = float(heading_match.group(1))
                                
                                self.timestamps.append(current_timestamp)
                                self.pitch_angles.append(pitch)
                                self.heading_angles.append(heading)
                                
                                if len(self.timestamps) % 1000 == 0:
                                    print(f"已解析 {len(self.timestamps)} 条记录...")
                                
                                # 重置时间戳，避免重复使用
                                current_timestamp = None
                            else:
                                print(f"警告：第{line_num}行安装角格式错误")
                        else:
                            print(f"警告：第{line_num}行缺少对应的时间戳")
        
        except FileNotFoundError:
            print(f"错误：找不到文件 {self.log_file_path}")
            return False
        except Exception as e:
            print(f"解析文件时出错: {e}")
            return False
        
        print(f"解析完成，共获得 {len(self.timestamps)} 条有效记录")
        return len(self.timestamps) > 0
    
    def convert_timestamps(self, time_format='relative'):
        """
        转换时间戳为不同格式
        time_format: 'relative' - 相对时间（秒），'absolute' - 绝对时间戳，'datetime' - 日期时间
        """
        if not self.timestamps:
            return []
        
        if time_format == 'relative':
            # 转换为相对时间（以第一个时间戳为基准）
            start_time = self.timestamps[0]
            return [(ts - start_time) / 1000.0 for ts in self.timestamps]
        
        elif time_format == 'absolute':
            # 返回原始时间戳（转换为秒）
            return [ts / 1000.0 for ts in self.timestamps]
        
        elif time_format == 'datetime':
            # 尝试转换为日期时间（假设是Unix时间戳）
            try:
                return [datetime.fromtimestamp(ts / 1000.0) for ts in self.timestamps]
            except:
                print("无法转换为日期时间格式，使用相对时间")
                return self.convert_timestamps('relative')
        
        else:
            return self.convert_timestamps('relative')
    
    def plot_misalignment(self, save_path=None, time_format='relative'):
        """
        绘制安装角随时间的变化
        time_format: 'relative' - 相对时间（秒），'absolute' - 绝对时间戳，'datetime' - 日期时间
        """
        if not self.timestamps:
            print("没有数据可绘制")
            return
        
        # 转换时间戳
        time_data = self.convert_timestamps(time_format)
        
        # 设置中文字体
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建图表
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # 绘制pitch角度变化
        ax1.plot(time_data, self.pitch_angles, 'b-', linewidth=1, alpha=0.7)
        ax1.set_title('手机安装角随时间变化', fontsize=14, fontweight='bold')
        ax1.set_ylabel('Pitch角度 (度)', fontsize=12)
        ax1.grid(True, alpha=0.3)
        
        # 添加pitch角度统计信息
        pitch_mean = np.mean(self.pitch_angles)
        pitch_std = np.std(self.pitch_angles)
        ax1.axhline(y=pitch_mean, color='r', linestyle='--', alpha=0.8, 
                   label=f'均值: {pitch_mean:.3f}°')
        ax1.axhline(y=pitch_mean + pitch_std, color='r', linestyle=':', alpha=0.6, 
                   label=f'±1σ: {pitch_std:.3f}°')
        ax1.axhline(y=pitch_mean - pitch_std, color='r', linestyle=':', alpha=0.6)
        ax1.legend()
        
        # 绘制heading角度变化
        ax2.plot(time_data, self.heading_angles, 'g-', linewidth=1, alpha=0.7)
        ax2.set_ylabel('Heading角度 (度)', fontsize=12)
        ax2.grid(True, alpha=0.3)
        
        # 添加heading角度统计信息
        heading_mean = np.mean(self.heading_angles)
        heading_std = np.std(self.heading_angles)
        ax2.axhline(y=heading_mean, color='r', linestyle='--', alpha=0.8, 
                   label=f'均值: {heading_mean:.3f}°')
        ax2.axhline(y=heading_mean + heading_std, color='r', linestyle=':', alpha=0.6, 
                   label=f'±1σ: {heading_std:.3f}°')
        ax2.axhline(y=heading_mean - heading_std, color='r', linestyle=':', alpha=0.6)
        ax2.legend()
        
        # 根据时间格式设置x轴标签
        if time_format == 'relative':
            ax2.set_xlabel('相对时间 (秒)', fontsize=12)
            ax1.set_xlim(0, max(time_data))
            ax2.set_xlim(0, max(time_data))
        elif time_format == 'absolute':
            ax2.set_xlabel('绝对时间戳 (秒)', fontsize=12)
            ax1.set_xlim(min(time_data), max(time_data))
            ax2.set_xlim(min(time_data), max(time_data))
        elif time_format == 'datetime':
            ax2.set_xlabel('日期时间', fontsize=12)
            # 旋转x轴标签以避免重叠
            ax1.tick_params(axis='x', rotation=45)
            ax2.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        
        # 保存图表
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"图表已保存到: {save_path}")
        
        plt.show()
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.timestamps:
            print("没有数据可统计")
            return
        
        print("\n=== 时间戳信息 ===")
        print(f"数据点数量: {len(self.timestamps)}")
        print(f"原始时间戳范围: {self.timestamps[0]} - {self.timestamps[-1]}")
        print(f"时间跨度: {(self.timestamps[-1] - self.timestamps[0])/1000:.1f} 秒")
        
        # 尝试判断时间戳格式
        first_ts = self.timestamps[0]
        if first_ts > 1000000000000:  # 毫秒级Unix时间戳
            print("时间戳格式：毫秒级Unix时间戳")
            try:
                start_time = datetime.fromtimestamp(first_ts / 1000.0)
                end_time = datetime.fromtimestamp(self.timestamps[-1] / 1000.0)
                print(f"开始时间: {start_time}")
                print(f"结束时间: {end_time}")
            except:
                print("无法转换为日期时间")
        elif first_ts > 1000000000:  # 秒级Unix时间戳
            print("时间戳格式：秒级Unix时间戳")
            try:
                start_time = datetime.fromtimestamp(first_ts)
                end_time = datetime.fromtimestamp(self.timestamps[-1])
                print(f"开始时间: {start_time}")
                print(f"结束时间: {end_time}")
            except:
                print("无法转换为日期时间")
        else:
            print("时间戳格式：系统tick时间或其他格式")
        
        print("\n=== 安装角统计信息 ===")
        print(f"\nPitch角度统计:")
        print(f"  均值: {np.mean(self.pitch_angles):.6f}°")
        print(f"  标准差: {np.std(self.pitch_angles):.6f}°")
        print(f"  最小值: {np.min(self.pitch_angles):.6f}°")
        print(f"  最大值: {np.max(self.pitch_angles):.6f}°")
        print(f"  变化范围: {np.max(self.pitch_angles) - np.min(self.pitch_angles):.6f}°")
        
        print(f"\nHeading角度统计:")
        print(f"  均值: {np.mean(self.heading_angles):.6f}°")
        print(f"  标准差: {np.std(self.heading_angles):.6f}°")
        print(f"  最小值: {np.min(self.heading_angles):.6f}°")
        print(f"  最大值: {np.max(self.heading_angles):.6f}°")
        print(f"  变化范围: {np.max(self.heading_angles) - np.min(self.heading_angles):.6f}°")
        
        # 收敛性分析
        if len(self.pitch_angles) > 100:
            # 分析最后10%的数据是否收敛
            tail_length = len(self.pitch_angles) // 10
            pitch_tail_std = np.std(self.pitch_angles[-tail_length:])
            heading_tail_std = np.std(self.heading_angles[-tail_length:])
            
            print(f"\n收敛性分析（最后10%数据）:")
            print(f"  Pitch角度标准差: {pitch_tail_std:.6f}°")
            print(f"  Heading角度标准差: {heading_tail_std:.6f}°")
            
            if pitch_tail_std < 0.1 and heading_tail_std < 0.1:
                print("  >>> 安装角已收敛到稳定值")
            else:
                print("  >>> 安装角可能仍在变化")

def main():
    parser = argparse.ArgumentParser(description='解析日志文件中的安装角信息')
    parser.add_argument('log_file', help='日志文件路径')
    parser.add_argument('--save', '-s', help='保存图表的路径')
    parser.add_argument('--time-format', '-t', choices=['relative', 'absolute', 'datetime'], 
                       default='relative', help='时间显示格式 (default: relative)')
    
    args = parser.parse_args()
    
    # 创建解析器
    log_parser = LogParser(args.log_file)
    
    # 解析日志
    if log_parser.parse_log():
        # 打印统计信息
        log_parser.print_statistics()
        
        # 绘制图表
        log_parser.plot_misalignment(args.save, args.time_format)
    else:
        print("日志解析失败")

if __name__ == "__main__":
    # 如果没有命令行参数，使用默认路径
    import sys
    if len(sys.argv) == 1:
        # 修改为你的日志文件路径
        log_file_path = "/Users/cjj/Data/vdr_plog/vdr_20250613_181225_863.log"
        save_path = "misalignment_plot.png"
        time_format = 'absolute'  # 可以改为 'absolute' 或 'datetime'
        
        print(f"使用默认路径: {log_file_path}")
        print(f"时间格式: {time_format}")
        
        log_parser = LogParser(log_file_path)
        
        if log_parser.parse_log():
            log_parser.print_statistics()
            log_parser.plot_misalignment(save_path, time_format)
        else:
            print("日志解析失败")
    else:
        main()