#!/usr/bin/env python3
"""
GPS时间戳差值可视化脚本
简单绘制GPS数据前后时间戳的差值图表
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from typing import List


def parse_gps_timestamps(log_file: str) -> List[float]:
    """
    解析日志文件中的GPS时间戳
    
    Args:
        log_file: 日志文件路径
        
    Returns:
        GPS时间戳列表 (秒)
    """
    timestamps = []
    
    try:
        with open(log_file, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                if line.startswith('$GPS'):
                    try:
                        fields = line.split()
                        if len(fields) >= 25:
                            # 提取毫秒级时间戳并转换为秒
                            timestamp = float(fields[1]) / 1000.0
                            timestamps.append(timestamp)
                    except Exception as e:
                        print(f"警告: 第{line_num}行GPS数据解析失败: {e}")
                        continue
    
    except FileNotFoundError:
        print(f"错误: 文件不存在 {log_file}")
        return []
    except Exception as e:
        print(f"错误: 读取文件失败 {e}")
        return []
    
    return sorted(timestamps)  # 确保时间序列有序


def create_timestamp_diff_plot(timestamps: List[float], output_file: str, log_filename: str):
    """
    创建时间戳差值图表
    
    Args:
        timestamps: GPS时间戳列表
        output_file: 输出图片路径
        log_filename: 日志文件名
    """
    if len(timestamps) < 2:
        print("数据不足，跳过绘图")
        return
    
    # 计算时间戳差值
    timestamps = np.array(timestamps)
    diffs = np.diff(timestamps)
    
    # 基本统计
    mean_diff = np.mean(diffs)
    min_diff = np.min(diffs)
    max_diff = np.max(diffs)
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 创建图表
    plt.figure(figsize=(12, 6))
    
    # 绘制时间差值
    time_points = timestamps[1:]  # 对应差值的时间点
    plt.plot(time_points, diffs, 'b-', alpha=0.7, linewidth=0.8, label='时间间隔')
    
    # 添加平均值线
    plt.axhline(y=mean_diff, color='r', linestyle='--', alpha=0.8, 
                label=f'平均值: {mean_diff:.3f}s')
    
    # 设置标题和标签
    plt.title(f'GPS时间戳差值分析 - {log_filename}', fontsize=14, fontweight='bold')
    plt.xlabel('GPS时间戳 (s)', fontsize=12)
    plt.ylabel('时间间隔 (s)', fontsize=12)
    
    # 添加统计信息文本框
    stats_text = f'''统计信息:
数据点数: {len(timestamps)}
平均间隔: {mean_diff:.3f}s
最小间隔: {min_diff:.3f}s
最大间隔: {max_diff:.3f}s'''
    
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # 设置纵轴刻度为每格0.05s
    plt.gca().yaxis.set_major_locator(plt.MultipleLocator(0.05))
    
    # 设置网格和图例
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')
    plt.tight_layout()
    
    # 保存图片
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"时间戳差值图表已保存: {output_file}")
    print(f"  平均间隔: {mean_diff:.3f}s")
    print(f"  最小间隔: {min_diff:.3f}s")
    print(f"  最大间隔: {max_diff:.3f}s")


def process_single_file(log_file: str, output_dir: str):
    """
    处理单个日志文件
    
    Args:
        log_file: 日志文件路径
        output_dir: 输出目录
    """
    log_filename = os.path.basename(log_file)
    base_name = os.path.splitext(log_filename)[0]
    
    print(f"\n处理文件: {log_filename}")
    
    # 解析GPS时间戳
    timestamps = parse_gps_timestamps(log_file)
    print(f"提取到 {len(timestamps)} 个GPS时间戳")
    
    if len(timestamps) < 2:
        print(f"警告: 文件 {log_filename} 时间戳数据不足，跳过")
        return
    
    # 创建差值图表
    output_file = os.path.join(output_dir, f"{base_name}_timestamp_diffs.png")
    create_timestamp_diff_plot(timestamps, output_file, log_filename)


def main():
    parser = argparse.ArgumentParser(description='GPS时间戳差值可视化工具')
    parser.add_argument('--input', '-i', required=True,
                       help='输入文件或目录路径')
    parser.add_argument('--output', '-o', required=True,
                       help='输出目录路径')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    print(f"GPS时间戳差值可视化")
    print(f"输入: {args.input}")
    print(f"输出: {args.output}")
    
    # 处理文件
    if os.path.isfile(args.input):
        # 单个文件
        process_single_file(args.input, args.output)
        
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
                process_single_file(full_path, args.output)
            except Exception as e:
                print(f"错误: 处理文件 {log_file} 失败: {e}")
    else:
        print(f"错误: 输入路径 {args.input} 不存在")
        return
    
    print(f"\n处理完成！所有图表已保存到: {args.output}")


if __name__ == "__main__":
    main()