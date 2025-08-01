#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版横向残差分析脚本 - 支持时间戳范围限制

使用方法:
1. 修改main()函数中的START_TIME和END_TIME来设置分析时间范围
2. 运行: python3 simple_lateral_analysis.py [数据目录路径]

时间戳示例:
- 如果你的数据时间戳是 1686651145.123, 1686651200.456 等
- 设置 START_TIME = 1686651150.0  # 从这个时间开始分析
- 设置 END_TIME = 1686651180.0    # 到这个时间结束分析
"""

import numpy as np
import pandas as pd
import glob
import os
import sys
import re

def extract_time_delay_from_filename(filename):
    """从文件名提取时间延迟值"""
    match = re.search(r'_(-?\d+)ms', filename)
    if match:
        return int(match.group(1)) / 1000.0  # 转换为秒
    else:
        return 0.0

def analyze_single_file(filepath, start_time=None, end_time=None):
    """分析单个横向残差文件"""
    try:
        # 读取数据，跳过注释行
        data = pd.read_csv(filepath, sep=' ', comment='#', 
                          names=['timestamp', 'lateral_residual', 'heading', 'speed', 
                                'utm_residual_x', 'utm_residual_y', 'utm_residual_norm'])
        
        original_count = len(data)
        
        # 应用时间戳过滤
        if start_time is not None:
            data = data[data['timestamp'] >= start_time]
        if end_time is not None:
            data = data[data['timestamp'] <= end_time]
            
        filtered_count = len(data)
        
        if filtered_count == 0:
            print(f"⚠️ 警告：时间范围过滤后无数据 {os.path.basename(filepath)}")
            return None, None, 0, 0
        
        lateral = data['lateral_residual'].values
        
        # 计算统计指标
        stats = {
            'rms': np.sqrt(np.mean(lateral**2)),
            'std': np.std(lateral),
            'mean': np.mean(lateral),
            'max_abs': np.max(np.abs(lateral)),
            'count': filtered_count
        }
        
        return stats, data, original_count, filtered_count
        
    except Exception as e:
        print(f"读取文件失败 {filepath}: {e}")
        return None, None, 0, 0

def main():
    """主函数"""
    # ===== 时间戳范围设置 =====
    # 设置为 None 表示不限制，或设置具体的起始/结束时间戳
    START_TIME = 868905.770    # 例如: 1686651145.0
    END_TIME = 869075.894      # 例如: 1686651200.0
    
    # 如果你想分析特定时间段，修改上面两行，例如：
    # START_TIME = 1686651145.0  # 开始时间戳
    # END_TIME = 1686651200.0    # 结束时间戳
    # ===============================
    
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3"
    
    # 查找所有横向残差文件
    pattern = os.path.join(data_dir, "*_lateral.txt")
    files = glob.glob(pattern)
    
    if not files:
        print(f"未找到横向残差文件，路径: {pattern}")
        print("请确保已运行ESKF并生成了横向残差文件")
        return
    
    print(f"找到 {len(files)} 个横向残差文件")
    
    # 显示时间范围设置
    if START_TIME is not None or END_TIME is not None:
        print(f"📅 时间范围限制:")
        if START_TIME is not None:
            print(f"   开始时间: {START_TIME}")
        if END_TIME is not None:
            print(f"   结束时间: {END_TIME}")
        print()
    else:
        print("📅 分析全部时间范围")
        
        # 显示第一个文件的时间戳范围作为参考
        if files:
            try:
                first_file_data = pd.read_csv(files[0], sep=' ', comment='#', 
                                            names=['timestamp', 'lateral_residual', 'heading', 'speed', 
                                                  'utm_residual_x', 'utm_residual_y', 'utm_residual_norm'])
                min_time = first_file_data['timestamp'].min()
                max_time = first_file_data['timestamp'].max()
                print(f"💡 数据时间戳范围参考: {min_time:.3f} - {max_time:.3f}")
                print(f"   (可用此范围设置START_TIME和END_TIME)")
            except:
                pass
        print()
    
    print("=" * 80)
    print(f"{'时间延迟(s)':<12} {'RMS(m)':<10} {'最大(m)':<10} {'标准差(m)':<12} {'数据点':<10} {'过滤率':<8}")
    print("=" * 80)
    
    results = []
    
    for filepath in sorted(files):
        filename = os.path.basename(filepath)
        time_delay = extract_time_delay_from_filename(filename)
        
        stats, data, original_count, filtered_count = analyze_single_file(
            filepath, START_TIME, END_TIME)
        
        if stats is None:
            continue
        
        results.append((time_delay, stats, filename))
        
        # 计算过滤率
        filter_rate = filtered_count / original_count if original_count > 0 else 0
        
        # 打印结果
        print(f"{time_delay:<12.3f} {stats['rms']:<10.4f} {stats['max_abs']:<10.4f} "
              f"{stats['std']:<12.4f} {stats['count']:<10} {filter_rate*100:<7.1f}%")
    
    if not results:
        print("没有有效数据")
        return
    
    print("=" * 80)
    
    # 找出最优结果
    best_result = min(results, key=lambda x: x[1]['rms'])
    best_delay, best_stats, best_file = best_result
    
    print(f"\n🎯 最优时间延迟: {best_delay:.3f}s")
    print(f"   对应RMS: {best_stats['rms']:.4f}m")
    print(f"   文件名: {best_file}")
    
    # 简单的敏感性分析
    rms_values = [r[1]['rms'] for r in results]
    rms_range = max(rms_values) - min(rms_values)
    
    print(f"\n📊 敏感性分析:")
    print(f"   RMS范围: {min(rms_values):.4f} - {max(rms_values):.4f}m")
    print(f"   变化幅度: {rms_range:.4f}m")
    
    if rms_range > 0.01:  # 1cm以上差异才算敏感
        print("   ⚠️  系统对时间延迟敏感，建议精确标定")
    else:
        print("   ✅ 系统对时间延迟相对不敏感")

if __name__ == "__main__":
    main()