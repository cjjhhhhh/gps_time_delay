#!/usr/bin/env python3
"""
转弯时间段位置修正量RMS分析脚本 - 单文件版本
用于分析单个日志文件在所有GPS偏移下的转弯时段定位精度
支持整段分析和转弯段分析
"""

import numpy as np
import argparse
import os
import sys
from collections import defaultdict

def load_corrections(corrections_file):
    """读取corrections文件"""
    try:
        data = np.loadtxt(corrections_file)
        return {
            'timestamps': data[:, 0],
            'dx': data[:, 1], 
            'dy': data[:, 2],
            'dz': data[:, 3]
        }
    except Exception as e:
        print(f"跳过：无法读取corrections文件 {corrections_file}: {e}")
        return None

def load_turn_segments(turns_file):
    """读取转弯段信息"""
    if not turns_file or not os.path.exists(turns_file):
        return []
        
    turn_segments = []
    try:
        with open(turns_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                
                parts = line.split(',')
                if len(parts) >= 3:
                    turn_id = int(parts[0])
                    start_time = float(parts[1])
                    end_time = float(parts[2])
                    duration = float(parts[3]) if len(parts) > 3 else (end_time - start_time)
                    direction = parts[6] if len(parts) > 6 else "未知"
                    
                    turn_segments.append({
                        'turn_id': turn_id,
                        'start_time': start_time,
                        'end_time': end_time,
                        'duration': duration,
                        'direction': direction
                    })
        
        return turn_segments
    except Exception as e:
        print(f"跳过：无法读取转弯文件 {turns_file}: {e}")
        return []

def calculate_planar_rms(dx, dy):
    """计算平面RMS"""
    if len(dx) == 0 or len(dy) == 0:
        return 0.0
    
    planar_errors = np.sqrt(dx**2 + dy**2)
    return np.sqrt(np.mean(planar_errors**2))

def analyze_single_offset_full_trajectory(corrections_file, gps_offset):
    """分析单个偏移值的整段轨迹RMS"""
    corrections = load_corrections(corrections_file)
    if corrections is None:
        return {}
    
    # 计算整段轨迹的RMS
    dx = corrections['dx']
    dy = corrections['dy']
    
    if len(dx) == 0:
        return {}
    
    rms = calculate_planar_rms(dx, dy)
    
    # 使用ID=0表示整段轨迹
    results = {
        0: {
            'turn_id': 0,
            'start_time': corrections['timestamps'][0],
            'end_time': corrections['timestamps'][-1],
            'duration': corrections['timestamps'][-1] - corrections['timestamps'][0],
            'direction': '整段轨迹',
            'data_points': len(dx),
            'rms': rms,
            'gps_offset': gps_offset
        }
    }
    
    return results

def analyze_single_offset_turns(corrections_file, turns_file, gps_offset):
    """分析单个偏移值的转弯段RMS"""
    corrections = load_corrections(corrections_file)
    if corrections is None:
        return {}
    
    turn_segments = load_turn_segments(turns_file)
    if not turn_segments:
        return {}
    
    results = {}
    
    # 计算每个转弯段的RMS
    for turn in turn_segments:
        # 筛选该转弯段的数据
        mask = ((corrections['timestamps'] >= turn['start_time']) & 
                (corrections['timestamps'] <= turn['end_time']))
        
        if not np.any(mask):
            continue
        
        dx_turn = corrections['dx'][mask]
        dy_turn = corrections['dy'][mask]
        
        if len(dx_turn) == 0:
            continue
        
        # 计算RMS
        rms = calculate_planar_rms(dx_turn, dy_turn)
        
        results[turn['turn_id']] = {
            'turn_id': turn['turn_id'],
            'start_time': turn['start_time'],
            'end_time': turn['end_time'],
            'duration': turn['duration'],
            'direction': turn['direction'],
            'data_points': len(dx_turn),
            'rms': rms,
            'gps_offset': gps_offset
        }
    
    return results

def main():
    parser = argparse.ArgumentParser(description='单文件转弯时间段位置修正量RMS分析')
    parser.add_argument('--log_dir', required=True, help='日志结果目录路径')
    parser.add_argument('--turns', help='转弯分析文件路径（可选，不提供则分析整段轨迹）') 
    parser.add_argument('--log_name', required=True, help='日志文件名')
    parser.add_argument('--offsets', required=True, help='GPS偏移值列表(逗号分隔)')
    parser.add_argument('--output_suffix', default='', help='输出文件名后缀')
    
    args = parser.parse_args()
    
    # 解析偏移值列表
    gps_offsets = [float(x.strip()) for x in args.offsets.split(',')]
    
    # 判断分析模式
    is_full_trajectory = not args.turns or not os.path.exists(args.turns)
    
    if is_full_trajectory:
        print(f"模式：整段轨迹分析 - {args.log_name}")
        analysis_func = analyze_single_offset_full_trajectory
    else:
        print(f"模式：转弯段分析 - {args.log_name}")
        analysis_func = lambda cf, gps_off: analyze_single_offset_turns(cf, args.turns, gps_off)
    
    # 收集所有偏移值的分析结果
    all_results = defaultdict(list)  # turn_id -> list of results
    successful_offsets = []
    
    for gps_offset in gps_offsets:
        # 构造corrections文件路径
        if gps_offset == 0.00:
            corrections_file = os.path.join(args.log_dir, "corrections.txt")
        else:
            offset_ms = int(gps_offset * 1000)
            corrections_file = os.path.join(args.log_dir, f"corrections_{offset_ms}ms.txt")
        
        # 分析该偏移值
        if is_full_trajectory:
            results = analysis_func(corrections_file, gps_offset)
        else:
            results = analysis_func(corrections_file, gps_offset)
        
        if results:
            successful_offsets.append(gps_offset)
            for turn_id, result in results.items():
                all_results[turn_id].append(result)
    
    if not all_results:
        print(f"跳过：{args.log_name} 未找到任何有效的RMS数据")
        return
    
    # 构造输出文件名
    base_filename = "turn_rms_analysis"
    if args.output_suffix:
        output_filename = f"{base_filename}{args.output_suffix}.txt"
    else:
        output_filename = f"{base_filename}.txt"
    
    output_file = os.path.join(args.log_dir, output_filename)
    
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            mode_desc = "整段轨迹" if is_full_trajectory else "转弯时间段"
            f.write(f"# {mode_desc}位置修正量RMS分析结果\n")
            f.write(f"# 日志文件: {args.log_name}\n")
            f.write(f"# 分析时间: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 成功偏移: {', '.join([f'{x:.2f}s' for x in successful_offsets])}\n")
            f.write(f"# 列：GPS偏移(s),平面RMS(m),数据点数,开始时间,结束时间,持续时间(s),转弯方向\n")
            f.write(f"\n")
            
            # 按转弯ID排序输出
            for turn_id in sorted(all_results.keys()):
                results_for_turn = all_results[turn_id]
                
                # 写入转弯段标题
                first_result = results_for_turn[0]
                if is_full_trajectory:
                    f.write(f"# 整段轨迹 ({first_result['direction']}, "
                           f"{first_result['start_time']:.1f}s-{first_result['end_time']:.1f}s, "
                           f"持续{first_result['duration']:.1f}s)\n")
                else:
                    f.write(f"# 转弯段 {turn_id} ({first_result['direction']}, "
                           f"{first_result['start_time']:.1f}s-{first_result['end_time']:.1f}s, "
                           f"持续{first_result['duration']:.1f}s)\n")
                
                # 按GPS偏移排序输出该转弯段的所有结果
                results_for_turn.sort(key=lambda x: x['gps_offset'])
                for result in results_for_turn:
                    f.write(f"{result['gps_offset']:.2f},{result['rms']:.4f},"
                          f"{result['data_points']},{result['start_time']:.3f},"
                          f"{result['end_time']:.3f},{result['duration']:.1f},"
                          f"{result['direction']}\n")
                
                # 转弯段之间空行
                f.write(f"\n")
        
        # 统计信息
        total_segments = len(all_results)
        total_analyses = sum(len(results) for results in all_results.values())
        avg_rms = np.mean([r['rms'] for results in all_results.values() for r in results])
        
        segment_desc = "整段轨迹" if is_full_trajectory else f"{total_segments}个转弯段"
        print(f"✓ {args.log_name}: {segment_desc}, {total_analyses}次分析, 平均RMS={avg_rms:.4f}m")
        print(f"  结果保存: {output_file}")
        
    except Exception as e:
        print(f"✗ {args.log_name}: 保存结果失败 {e}")

if __name__ == "__main__":
    main()