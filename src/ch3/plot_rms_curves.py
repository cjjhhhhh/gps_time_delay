#!/usr/bin/env python3
"""
RMS曲线绘图脚本
用于绘制每个转弯段的位置修正量RMS和横向残差RMS随GPS延迟的变化曲线
支持整段轨迹和转弯段分析结果
"""

import matplotlib
matplotlib.use('Agg')  # 设置无显示后端
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import sys
from collections import defaultdict

# 设置matplotlib后端和中文字体
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Liberation Sans', 'Bitstream Vera Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

def parse_rms_file(rms_file):
    """解析RMS分析文件"""
    turn_data = defaultdict(list)
    
    if not os.path.exists(rms_file):
        return {}
    
    try:
        with open(rms_file, 'r', encoding='utf-8') as f:
            current_turn_id = None
            current_turn_info = None
            
            for line in f:
                line = line.strip()
                
                # 解析转弯段标题或整段轨迹标题
                if line.startswith('# 转弯段') or line.startswith('# 整段轨迹'):
                    # 提取信息
                    # 格式: # 转弯段 1 (左转, 120.5s-145.8s, 持续25.3s)
                    # 或: # 整段轨迹 (整段轨迹, 120.5s-145.8s, 持续25.3s)
                    parts = line.split('(')
                    if len(parts) >= 2:
                        turn_part = parts[0].strip()
                        info_part = parts[1].rstrip(')')
                        
                        # 提取ID
                        if '整段轨迹' in turn_part:
                            current_turn_id = 0  # 整段轨迹使用ID=0
                            current_turn_info = info_part
                        else:
                            # 提取转弯段ID
                            turn_id_str = turn_part.replace('# 转弯段', '').strip()
                            try:
                                current_turn_id = int(turn_id_str)
                                current_turn_info = info_part
                            except ValueError:
                                continue
                
                # 解析数据行 (跳过注释和空行)
                elif not line.startswith('#') and line and current_turn_id is not None:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        try:
                            gps_offset = float(parts[0])
                            rms_value = float(parts[1])
                            
                            turn_data[current_turn_id].append({
                                'gps_offset': gps_offset,
                                'rms': rms_value,
                                'info': current_turn_info
                            })
                        except ValueError:
                            continue
    
    except Exception as e:
        print(f"解析RMS文件失败 {rms_file}: {e}")
        return {}
    
    return dict(turn_data)

def plot_turn_rms_curves(pos_data, lateral_data, turn_id, output_file, title_suffix=""):
    """绘制单个转弯段或整段轨迹的RMS曲线图"""
    
    # 创建图表
    plt.figure(figsize=(10, 6))
    
    # 设置网格
    plt.grid(True, alpha=0.3)
    
    has_data = False
    turn_info = ""
    direction_en = ""
    
    # 用于存储改进量计算的数据
    pos_best_rms = None
    pos_zero_rms = None
    lateral_best_rms = None
    lateral_zero_rms = None
    
    # 绘制位置RMS曲线
    if turn_id in pos_data and pos_data[turn_id]:
        pos_points = sorted(pos_data[turn_id], key=lambda x: x['gps_offset'])
        offsets = [p['gps_offset'] for p in pos_points]
        rms_values = [p['rms'] for p in pos_points]
        turn_info = pos_points[0]['info']
        
        # 转换方向为英文
        if "左转" in turn_info:
            direction_en = turn_info.replace("左转", "Left Turn")
        elif "右转" in turn_info:
            direction_en = turn_info.replace("右转", "Right Turn")
        elif "整段轨迹" in turn_info:
            direction_en = turn_info.replace("整段轨迹", "Full Trajectory")
        else:
            direction_en = turn_info.replace("未知", "Unknown")
        
        plt.plot(offsets, rms_values, 'o-', color='#1f77b4', linewidth=2, 
                markersize=6, label='Position RMS', alpha=0.8)
        
        # 标记最优点
        min_idx = np.argmin(rms_values)
        min_offset = offsets[min_idx]
        min_rms = rms_values[min_idx]
        plt.plot(min_offset, min_rms, 'o', color='#1f77b4', markersize=10, 
                markerfacecolor='white', markeredgewidth=2, 
                label=f'Position Best: {min_offset:.2f}s ({min_rms:.4f}m)')
        
        # 存储最佳RMS值
        pos_best_rms = min_rms
        
        # 查找0.0延迟对应的RMS值
        for i, offset in enumerate(offsets):
            if abs(offset - 0.0) < 1e-6:  # 浮点数比较
                pos_zero_rms = rms_values[i]
                break
        
        has_data = True
    
    # 绘制横向残差RMS曲线
    if turn_id in lateral_data and lateral_data[turn_id]:
        lateral_points = sorted(lateral_data[turn_id], key=lambda x: x['gps_offset'])
        offsets = [p['gps_offset'] for p in lateral_points]
        rms_values = [p['rms'] for p in lateral_points]
        if not turn_info:
            turn_info = lateral_points[0]['info']
            # 转换方向为英文
            if "左转" in turn_info:
                direction_en = turn_info.replace("左转", "Left Turn")
            elif "右转" in turn_info:
                direction_en = turn_info.replace("右转", "Right Turn")
            elif "整段轨迹" in turn_info:
                direction_en = turn_info.replace("整段轨迹", "Full Trajectory")
            else:
                direction_en = turn_info.replace("未知", "Unknown")
        elif not direction_en:
            # 转换方向为英文
            if "左转" in turn_info:
                direction_en = turn_info.replace("左转", "Left Turn")
            elif "右转" in turn_info:
                direction_en = turn_info.replace("右转", "Right Turn")
            elif "整段轨迹" in turn_info:
                direction_en = turn_info.replace("整段轨迹", "Full Trajectory")
            else:
                direction_en = turn_info.replace("未知", "Unknown")
        
        plt.plot(offsets, rms_values, 's-', color='#d62728', linewidth=2, 
                markersize=6, label='Lateral Residual RMS', alpha=0.8)
        
        # 标记最优点
        min_idx = np.argmin(rms_values)
        min_offset = offsets[min_idx]
        min_rms = rms_values[min_idx]
        plt.plot(min_offset, min_rms, 's', color='#d62728', markersize=10, 
                markerfacecolor='white', markeredgewidth=2,
                label=f'Lateral Best: {min_offset:.2f}s ({min_rms:.4f}m)')
        
        # 存储最佳RMS值
        lateral_best_rms = min_rms
        
        # 查找0.0延迟对应的RMS值
        for i, offset in enumerate(offsets):
            if abs(offset - 0.0) < 1e-6:  # 浮点数比较
                lateral_zero_rms = rms_values[i]
                break
        
        has_data = True
    
    if not has_data:
        plt.close()
        return False
    
    # 计算并显示改进量
    improvement_text = []
    if pos_best_rms is not None and pos_zero_rms is not None:
        pos_improvement = pos_zero_rms - pos_best_rms
        improvement_text.append(f'Position Improvement: {pos_improvement:.4f}m')
    
    if lateral_best_rms is not None and lateral_zero_rms is not None:
        lateral_improvement = lateral_zero_rms - lateral_best_rms
        improvement_text.append(f'Lateral Improvement: {lateral_improvement:.4f}m')
    
    # 在左下角添加改进量文本框
    if improvement_text:
        improvement_str = '\n'.join(improvement_text)
        plt.text(0.02, 0.02, improvement_str, transform=plt.gca().transAxes,
                fontsize=10, verticalalignment='bottom', horizontalalignment='left',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
    
    # 设置图表属性
    plt.xlabel('GPS Time Offset (s)', fontsize=12)
    plt.ylabel('RMS (m)', fontsize=12)
    
    # 设置标题
    if turn_id == 0:
        title = f'Full Trajectory ({direction_en}){title_suffix}'
    else:
        title = f'Turn Segment {turn_id} ({direction_en}){title_suffix}'
    
    plt.title(title, fontsize=14, fontweight='bold')
    
    # 设置X轴范围
    all_offsets = []
    if turn_id in pos_data and pos_data[turn_id]:
        all_offsets.extend([p['gps_offset'] for p in pos_data[turn_id]])
    if turn_id in lateral_data and lateral_data[turn_id]:
        all_offsets.extend([p['gps_offset'] for p in lateral_data[turn_id]])

    if all_offsets:
        x_min = min(all_offsets)
        x_max = max(all_offsets)
        x_margin = (x_max - x_min) * 0.05  # 5%边距
        plt.xlim(x_min - x_margin, x_max + x_margin)    
        
    # 设置Y轴从0开始
    y_min, y_max = plt.ylim()
    plt.ylim(0, y_max * 1.1)
    
    # 设置图例
    plt.legend(loc='best', fontsize=10, framealpha=0.9)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图表
    try:
        plt.savefig(output_file, dpi=300, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        plt.close()
        return True
    except Exception as e:
        print(f"保存图表失败 {output_file}: {e}")
        plt.close()
        return False

def process_vdr_folder(vdr_dir, log_name):
    """处理单个vdr文件夹，生成所有类型的RMS曲线图"""
    
    # 定义不同类型的分析文件
    analysis_types = {
        "turns": {
            "pos_file": os.path.join(vdr_dir, "turn_rms_analysis_turns.txt"),
            "lateral_file": os.path.join(vdr_dir, "turn_lateral_analysis_turns.txt"),
            "title_suffix": " (Turning Segments)"
        },
        "full": {
            "pos_file": os.path.join(vdr_dir, "turn_rms_analysis_full.txt"), 
            "lateral_file": os.path.join(vdr_dir, "turn_lateral_analysis_full.txt"),
            "title_suffix": " (Full Trajectory)"
        },
        # 兼容原有的文件名（无后缀）
        "default": {
            "pos_file": os.path.join(vdr_dir, "turn_rms_analysis.txt"),
            "lateral_file": os.path.join(vdr_dir, "turn_lateral_analysis.txt"),
            "title_suffix": ""
        }
    }
    
    total_success = 0
    
    for analysis_type, files in analysis_types.items():
        pos_file = files["pos_file"]
        lateral_file = files["lateral_file"]
        title_suffix = files["title_suffix"]
        
        # 检查是否有对应的分析文件
        if not os.path.exists(pos_file) and not os.path.exists(lateral_file):
            continue
            
        print(f"处理 {analysis_type} 分析结果...")
        
        # 解析分析文件
        pos_data = parse_rms_file(pos_file)
        lateral_data = parse_rms_file(lateral_file)
        
        # 获取所有转弯段ID
        all_turn_ids = set()
        if pos_data:
            all_turn_ids.update(pos_data.keys())
        if lateral_data:
            all_turn_ids.update(lateral_data.keys())
        
        if not all_turn_ids:
            continue
        
        # 创建对应的plots子目录
        if analysis_type == "default":
            plots_dir = os.path.join(vdr_dir, "plots")
        else:
            plots_dir = os.path.join(vdr_dir, "plots", analysis_type)
        os.makedirs(plots_dir, exist_ok=True)
        
        # 为每个段生成图表
        for turn_id in sorted(all_turn_ids):
            if turn_id == 0:  # 整段轨迹
                output_file = os.path.join(plots_dir, f"full_trajectory_rms_plot.png")
            else:  # 转弯段
                output_file = os.path.join(plots_dir, f"turn_{turn_id}_rms_plot.png")
            
            if plot_turn_rms_curves(pos_data, lateral_data, turn_id, 
                                  output_file, title_suffix):
                total_success += 1
    
    if total_success > 0:
        print(f"✓ {log_name}: 生成了 {total_success} 个RMS曲线图")
        print(f"  图表保存: {os.path.join(vdr_dir, 'plots')}/")
    else:
        print(f"✗ {log_name}: 未能生成任何图表")
    
    return total_success

def main():
    parser = argparse.ArgumentParser(description='生成转弯段RMS曲线图')
    parser.add_argument('--vdr_dir', required=True, help='vdr文件夹路径')
    parser.add_argument('--log_name', required=True, help='日志文件名')
    
    args = parser.parse_args()
    
    # 检查matplotlib依赖
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as e:
        print(f"跳过绘图: 缺少依赖库 - {e}")
        print("请安装: pip install matplotlib numpy")
        return
    
    # 检查vdr目录是否存在
    if not os.path.exists(args.vdr_dir):
        print(f"跳过绘图: 目录不存在 - {args.vdr_dir}")
        return
    
    # 处理vdr文件夹
    process_vdr_folder(args.vdr_dir, args.log_name)

if __name__ == "__main__":
    main()