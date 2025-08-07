#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹对比绘制脚本
基于GPS时间延迟优化结果，绘制轨迹对比图
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import json
import glob
from pathlib import Path

# 添加plotly支持
try:
    import plotly.graph_objects as go
    import plotly.offline as pyo
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

def read_trajectory_file(filepath):
    """读取ESKF轨迹文件 (gins_offline.txt格式)"""
    if not os.path.exists(filepath):
        return None
    
    try:
        data = np.loadtxt(filepath)
        if data.size == 0:
            return None
        
        # 确保是2D数组
        if data.ndim == 1:
            data = data.reshape(1, -1)
        
        # 验证数据格式：应该至少有21列
        # timestamp px py pz qw qx qy qz vx vy vz bgx bgy bgz bax bay baz gps_px gps_py gps_pz gps_valid
        if data.shape[1] < 21:
            print(f"警告: 轨迹文件格式不正确，列数: {data.shape[1]}, 期望至少21列")
        
        return data
    except Exception as e:
        print(f"读取轨迹文件失败: {filepath}, 错误: {e}")
        return None

def read_turn_segments(turn_file):
    """读取转弯段信息 - 解析CSV格式的转弯段文件"""
    if not os.path.exists(turn_file):
        return []
    
    turns = []
    try:
        with open(turn_file, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                # 跳过空行和注释行
                if not line or line.startswith('#'):
                    continue
                
                # 解析CSV格式的数据行
                # 格式: 转弯ID,起始时间戳,结束时间戳,持续时间(s),累积角度(度),平均转弯率(度/s),转弯方向
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 3:
                        try:
                            turn_id = int(parts[0])
                            start_time = float(parts[1])
                            end_time = float(parts[2])
                            
                            # 提取额外信息（如果可用）
                            duration = float(parts[3]) if len(parts) > 3 else end_time - start_time
                            total_angle = float(parts[4]) if len(parts) > 4 else 0.0
                            avg_turn_rate = float(parts[5]) if len(parts) > 5 else 0.0
                            direction = parts[6].strip() if len(parts) > 6 else "未知"
                            
                            turns.append({
                                'id': turn_id,
                                'start': start_time,
                                'end': end_time,
                                'duration': duration,
                                'total_angle': total_angle,
                                'avg_turn_rate': avg_turn_rate,
                                'direction': direction
                            })
                        except (ValueError, IndexError) as e:
                            print(f"警告: 第{line_num}行数据解析失败: {line} - {e}")
                            continue
        
        if turns:
            print(f"读取到 {len(turns)} 个转弯段")
        
        return turns
    except Exception as e:
        print(f"读取转弯段文件失败: {turn_file}, 错误: {e}")
        return []

def parse_rms_file_by_turn(rms_file):
    """解析RMS分析文件，按转弯段ID分组返回数据"""
    from collections import defaultdict
    
    turn_data = defaultdict(list)
    
    if not os.path.exists(rms_file):
        return {}
    
    try:
        with open(rms_file, 'r', encoding='utf-8') as f:
            current_turn_id = None
            
            for line in f:
                line = line.strip()
                
                # 解析转弯段标题
                if line.startswith('# 转弯段'):
                    # 格式: # 转弯段 1 (左转, 120.5s-145.8s, 持续25.3s)
                    parts = line.split('(')
                    if len(parts) >= 1:
                        turn_part = parts[0].strip()
                        turn_id_str = turn_part.replace('# 转弯段', '').strip()
                        try:
                            current_turn_id = int(turn_id_str)
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
                                'rms': rms_value
                            })
                        except ValueError:
                            continue
    
    except Exception as e:
        print(f"解析RMS文件失败 {rms_file}: {e}")
        return {}
    
    return dict(turn_data)

def find_optimal_delay_for_turn(log_dir, turn_id):
    """为指定转弯段找到最优延迟值，基于改进量选择位置RMS或横向残差RMS"""
    
    # 读取位置RMS分析文件
    pos_rms_file = os.path.join(log_dir, "turn_rms_analysis_turns.txt")
    pos_data = parse_rms_file_by_turn(pos_rms_file)
    
    # 读取横向残差RMS分析文件
    lateral_rms_file = os.path.join(log_dir, "turn_lateral_analysis_turns.txt")
    lateral_data = parse_rms_file_by_turn(lateral_rms_file)
    
    pos_optimal_delay = None
    pos_improvement = 0
    lateral_optimal_delay = None
    lateral_improvement = 0
    
    # 分析位置RMS数据
    if turn_id in pos_data and pos_data[turn_id]:
        pos_points = pos_data[turn_id]
        
        # 找到最优延迟和对应的RMS值
        min_rms = float('inf')
        zero_rms = None
        
        for point in pos_points:
            offset = point['gps_offset']
            rms = point['rms']
            
            if rms < min_rms:
                min_rms = rms
                pos_optimal_delay = offset
            
            # 记录0.0延迟的RMS值
            if abs(offset - 0.0) < 1e-6:
                zero_rms = rms
        
        # 计算改进量
        if zero_rms is not None:
            pos_improvement = zero_rms - min_rms
    
    # 分析横向残差RMS数据
    if turn_id in lateral_data and lateral_data[turn_id]:
        lateral_points = lateral_data[turn_id]
        
        # 找到最优延迟和对应的RMS值
        min_rms = float('inf')
        zero_rms = None
        
        for point in lateral_points:
            offset = point['gps_offset']
            rms = point['rms']
            
            if rms < min_rms:
                min_rms = rms
                lateral_optimal_delay = offset
            
            # 记录0.0延迟的RMS值
            if abs(offset - 0.0) < 1e-6:
                zero_rms = rms
        
        # 计算改进量
        if zero_rms is not None:
            lateral_improvement = zero_rms - min_rms
    
    # 根据改进量选择最优延迟
    if pos_improvement > lateral_improvement:
        if pos_optimal_delay is not None:
            print(f"转弯段 {turn_id}: 选择位置RMS最优延迟 {pos_optimal_delay}s (改进量: {pos_improvement:.4f}m)")
            return pos_optimal_delay
    else:
        if lateral_optimal_delay is not None:
            print(f"转弯段 {turn_id}: 选择横向残差RMS最优延迟 {lateral_optimal_delay}s (改进量: {lateral_improvement:.4f}m)")
            return lateral_optimal_delay
    
    # 如果都没有找到，尝试从整段轨迹分析中获取
    print(f"转弯段 {turn_id}: 未找到转弯段级别的最优延迟，尝试使用整段轨迹分析")
    return find_optimal_delay_from_full_analysis(log_dir)

def find_optimal_delay_from_full_analysis(log_dir):
    """从整段轨迹分析中找到最优延迟值（回退方法）"""
    rms_files = [
        "turn_rms_analysis_full.txt",
        "turn_rms_analysis.txt"
    ]
    
    for rms_file in rms_files:
        rms_path = os.path.join(log_dir, rms_file)
        if os.path.exists(rms_path):
            optimal_delay = parse_optimal_from_full_rms_file(rms_path)
            if optimal_delay is not None:
                print(f"从 {rms_file} 中读取到整段轨迹最优延迟: {optimal_delay}s")
                return optimal_delay
    
    return None

def parse_optimal_from_full_rms_file(rms_file):
    """从整段轨迹RMS分析文件中解析最优延迟值"""
    try:
        with open(rms_file, 'r', encoding='utf-8') as f:
            min_rms = float('inf')
            optimal_delay = None
            
            for line in f:
                line = line.strip()
                
                # 跳过注释行和空行
                if not line or line.startswith('#'):
                    continue
                
                # 解析数据行：GPS偏移(s),平面RMS(m),数据点数,开始时间,结束时间,持续时间(s),转弯方向
                parts = line.split(',') if ',' in line else line.split()
                if len(parts) >= 2:
                    try:
                        delay = float(parts[0])
                        rms = float(parts[1])
                        
                        if rms < min_rms:
                            min_rms = rms
                            optimal_delay = delay
                    except (ValueError, IndexError):
                        continue
            
            return optimal_delay
            
    except Exception as e:
        print(f"解析RMS文件失败: {e}")
        return None


def extract_turn_data(data, turn_info):
    """提取转弯段数据"""
    if data is None or len(data) == 0:
        return None
    
    # 假设数据格式: [timestamp, x, y, ...]
    timestamps = data[:, 0]
    
    # 找到转弯段时间范围内的数据
    mask = (timestamps >= turn_info['start']) & (timestamps <= turn_info['end'])
    turn_data = data[mask]
    
    if len(turn_data) == 0:
        return None
    
    return turn_data

def plot_full_trajectory_interactive(data_0, log_name, output_dir):
    """绘制交互式完整轨迹图"""
    if not PLOTLY_AVAILABLE:
        print("Plotly不可用，跳过交互式完整轨迹图")
        return None
        
    if data_0 is None or len(data_0) == 0:
        print(f"无法绘制 {log_name} 的完整轨迹：数据为空")
        return None
    
    x_coords = data_0[:, 1]
    y_coords = data_0[:, 2]
    timestamps = data_0[:, 0]
    
    # 计算速度（如果有速度数据）
    if data_0.shape[1] >= 11:
        vx, vy, vz = data_0[:, 8], data_0[:, 9], data_0[:, 10]
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
    else:
        speed = np.zeros(len(data_0))
    
    fig = go.Figure()
    
    # 添加轨迹，颜色按速度变化
    fig.add_trace(go.Scatter(
        x=x_coords, y=y_coords,
        mode='lines+markers',
        name='ESKF轨迹',
        line=dict(width=2),
        marker=dict(
            size=3,
            color=speed,
            colorscale='Viridis',
            showscale=True,
            colorbar=dict(title="速度 (m/s)")
        ),
        hovertemplate='<b>ESKF轨迹</b><br>' +
                      'X: %{x:.2f}m<br>' +
                      'Y: %{y:.2f}m<br>' +
                      '时间: %{customdata:.1f}s<br>' +
                      '<extra></extra>',
        customdata=timestamps
    ))
    
    # 标记起点和终点
    fig.add_trace(go.Scatter(
        x=[x_coords[0]], y=[y_coords[0]],
        mode='markers',
        name='起点',
        marker=dict(size=12, color='green', symbol='circle'),
        hovertemplate='<b>起点</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    fig.add_trace(go.Scatter(
        x=[x_coords[-1]], y=[y_coords[-1]],
        mode='markers',
        name='终点',
        marker=dict(size=12, color='red', symbol='square'),
        hovertemplate='<b>终点</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # 如果有GPS观测数据，也添加进去
    if data_0.shape[1] >= 20:
        gps_valid = data_0[:, 20] if data_0.shape[1] > 20 else np.ones(len(data_0))
        valid_gps_mask = gps_valid == 1
        if np.sum(valid_gps_mask) > 0:
            fig.add_trace(go.Scatter(
                x=data_0[valid_gps_mask, 17], 
                y=data_0[valid_gps_mask, 18],
                mode='markers',
                name='GPS观测点',
                marker=dict(size=6, color='orange', symbol='diamond'),
                hovertemplate='<b>GPS观测</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
            ))
    
    fig.update_layout(
        title=dict(
            text=f'{log_name} - 完整轨迹 (0.0s延迟)',
            font=dict(size=18),
            x=0.5
        ),
        xaxis=dict(title='X坐标 (m)', showgrid=True),
        yaxis=dict(title='Y坐标 (m)', scaleanchor="x", scaleratio=1, showgrid=True),
        hovermode='closest',
        showlegend=True,
        width=1200,
        height=900,
        template='plotly_white'
    )
    
    # 添加注释说明
    fig.add_annotation(
        text="💡 提示：使用鼠标滚轮缩放，拖拽平移，双击重置视图",
        xref="paper", yref="paper",
        x=0.5, y=-0.1,
        showarrow=False,
        font=dict(size=12, color="gray"),
        xanchor='center'
    )
    
    # 保存HTML
    html_file = os.path.join(output_dir, f"{log_name}_full_trajectory_interactive.html")
    
    # 配置HTML输出选项
    config = {
        'displayModeBar': True,
        'displaylogo': False,
        'modeBarButtonsToRemove': ['lasso2d', 'select2d'],
        'toImageButtonOptions': {
            'format': 'png',
            'filename': f'{log_name}_full_trajectory',
            'height': 900,
            'width': 1200,
            'scale': 2
        }
    }
    
    pyo.plot(fig, filename=html_file, auto_open=False, config=config)
    return html_file

def plot_turn_comparison_interactive(data_0, data_optimal, turn_info, optimal_delay, log_name, output_dir):
    """生成交互式HTML轨迹对比图"""
    if not PLOTLY_AVAILABLE:
        print("Plotly不可用，跳过交互式转弯段对比图")
        return None
        
    # 提取转弯段数据
    turn_data_0 = extract_turn_data(data_0, turn_info)
    turn_data_optimal = extract_turn_data(data_optimal, turn_info)
    
    if turn_data_0 is None or turn_data_optimal is None:
        print(f"转弯段 {turn_info['id']} 数据不足，跳过")
        return None
    
    x0, y0 = turn_data_0[:, 1], turn_data_0[:, 2]
    x_opt, y_opt = turn_data_optimal[:, 1], turn_data_optimal[:, 2]
    timestamps_0 = turn_data_0[:, 0]
    timestamps_opt = turn_data_optimal[:, 0]
    
    # 创建交互式图形
    fig = go.Figure()
    
    # 添加0.0s延迟轨迹
    fig.add_trace(go.Scatter(
        x=x0, y=y0,
        mode='lines+markers',
        name='0.0s延迟轨迹',
        line=dict(color='blue', width=3),
        marker=dict(size=4, color='blue', opacity=0.7),
        hovertemplate='<b>0.0s延迟轨迹</b><br>' +
                      'X: %{x:.2f}m<br>' +
                      'Y: %{y:.2f}m<br>' +
                      '时间: %{customdata:.1f}s<br>' +
                      '点序号: %{pointNumber}<br>' +
                      '<extra></extra>',
        customdata=timestamps_0
    ))
    
    # 添加最优延迟轨迹
    fig.add_trace(go.Scatter(
        x=x_opt, y=y_opt,
        mode='lines+markers',
        name=f'{optimal_delay:.3f}s延迟轨迹',
        line=dict(color='red', width=3),
        marker=dict(size=4, color='red', opacity=0.7),
        hovertemplate=f'<b>{optimal_delay:.3f}s延迟轨迹</b><br>' +
                      'X: %{x:.2f}m<br>' +
                      'Y: %{y:.2f}m<br>' +
                      '时间: %{customdata:.1f}s<br>' +
                      '点序号: %{pointNumber}<br>' +
                      '<extra></extra>',
        customdata=timestamps_opt
    ))
    
    # 标记起点
    fig.add_trace(go.Scatter(
        x=[x0[0]], y=[y0[0]],
        mode='markers',
        name='起点',
        marker=dict(size=15, color='green', symbol='circle', 
                   line=dict(width=2, color='darkgreen')),
        hovertemplate='<b>起点</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # 标记终点
    fig.add_trace(go.Scatter(
        x=[x0[-1]], y=[y0[-1]],
        mode='markers',
        name='终点',
        marker=dict(size=15, color='purple', symbol='square',
                   line=dict(width=2, color='darkmagenta')),
        hovertemplate='<b>终点</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # 添加GPS观测点（如果有的话）
    if turn_data_0.shape[1] >= 20:
        gps_valid_0 = turn_data_0[:, 20] if turn_data_0.shape[1] > 20 else np.ones(len(turn_data_0))
        valid_gps_mask_0 = gps_valid_0 == 1
        if np.sum(valid_gps_mask_0) > 0:
            fig.add_trace(go.Scatter(
                x=turn_data_0[valid_gps_mask_0, 17], 
                y=turn_data_0[valid_gps_mask_0, 18],
                mode='markers',
                name='GPS观测点',
                marker=dict(size=8, color='orange', symbol='diamond',
                           line=dict(width=1, color='darkorange')),
                hovertemplate='<b>GPS观测点</b><br>' +
                              'X: %{x:.2f}m<br>' +
                              'Y: %{y:.2f}m<br>' +
                              '<extra></extra>'
            ))
    
    # 计算显示范围
    all_x = np.concatenate([x0, x_opt])
    all_y = np.concatenate([y0, y_opt])
    margin = max(5, (all_x.max() - all_x.min()) * 0.1)
    
    # 设置布局
    title_text = f'{log_name} - 转弯段 {turn_info["id"]} 轨迹对比<br>'
    title_text += f'<sub>时间: {turn_info["start"]:.1f}s - {turn_info["end"]:.1f}s '
    title_text += f'({turn_info["duration"]:.1f}s, {turn_info["total_angle"]:.1f}°, {turn_info["direction"]})</sub>'
    
    fig.update_layout(
        title=dict(
            text=title_text,
            font=dict(size=18),
            x=0.5,
            xanchor='center'
        ),
        xaxis=dict(
            title=dict(text='X坐标 (m)', font=dict(size=14)),
            tickfont=dict(size=12),
            range=[all_x.min() - margin, all_x.max() + margin],
            showgrid=True,
            gridwidth=1,
            gridcolor='lightgray'
        ),
        yaxis=dict(
            title=dict(text='Y坐标 (m)', font=dict(size=14)),
            tickfont=dict(size=12),
            range=[all_y.min() - margin, all_y.max() + margin],
            scaleanchor="x",  # 关键：保持等比例
            scaleratio=1,
            showgrid=True,
            gridwidth=1,
            gridcolor='lightgray'
        ),
        hovermode='closest',
        showlegend=True,
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor='rgba(255,255,255,0.8)',
            bordercolor='gray',
            borderwidth=1
        ),
        width=1200,
        height=900,
        template='plotly_white',
        # 添加工具栏配置
        modebar=dict(
            bgcolor='rgba(255,255,255,0.8)',
            color='gray',
            activecolor='blue'
        )
    )
    
    # 添加注释说明
    fig.add_annotation(
        text="💡 提示：使用鼠标滚轮缩放，拖拽平移，双击重置视图，点击工具栏可导出PNG",
        xref="paper", yref="paper",
        x=0.5, y=-0.1,
        showarrow=False,
        font=dict(size=12, color="gray"),
        xanchor='center'
    )
    
    # 保存为HTML
    html_file = os.path.join(output_dir, f"{log_name}_turn_{turn_info['id']}_interactive.html")
    
    # 配置HTML输出选项
    config = {
        'displayModeBar': True,  # 显示工具栏
        'displaylogo': False,    # 隐藏plotly logo
        'modeBarButtonsToRemove': ['lasso2d', 'select2d'],
        'toImageButtonOptions': {
            'format': 'png',
            'filename': f'{log_name}_turn_{turn_info["id"]}_comparison',
            'height': 900,
            'width': 1200,
            'scale': 2  # 高分辨率导出
        }
    }
    
    pyo.plot(fig, filename=html_file, auto_open=False, config=config)
    return html_file

def plot_full_trajectory(data_0, log_name, output_dir):
    """绘制完整轨迹图（静态版本）"""
    if data_0 is None or len(data_0) == 0:
        print(f"无法绘制 {log_name} 的完整轨迹：数据为空")
        return
    
    plt.figure(figsize=(12, 10))
    
    # 绘制轨迹
    x_coords = data_0[:, 1]  # 假设第2列是x坐标
    y_coords = data_0[:, 2]  # 假设第3列是y坐标
    
    plt.plot(x_coords, y_coords, 'b-', linewidth=1.5, alpha=0.8, label='ESKF轨迹')
    
    # 标记起点和终点
    plt.plot(x_coords[0], y_coords[0], 'go', markersize=8, label='起点')
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=8, label='终点')
    
    plt.xlabel('X坐标 (m)')
    plt.ylabel('Y坐标 (m)')
    plt.title(f'{log_name} - 完整轨迹 (0.0s延迟)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 保存图片
    output_file = os.path.join(output_dir, f"{log_name}_full_trajectory.png")
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_turn_comparison(data_0, data_optimal, turn_info, optimal_delay, log_name, output_dir):
    """绘制转弯段对比图（静态版本）"""
    # 提取转弯段数据
    turn_data_0 = extract_turn_data(data_0, turn_info)
    turn_data_optimal = extract_turn_data(data_optimal, turn_info)
    
    if turn_data_0 is None or turn_data_optimal is None:
        print(f"转弯段 {turn_info['id']} 数据不足，跳过")
        return
    
    plt.figure(figsize=(12, 8))
    
    # 绘制两条轨迹
    x0, y0 = turn_data_0[:, 1], turn_data_0[:, 2]
    x_opt, y_opt = turn_data_optimal[:, 1], turn_data_optimal[:, 2]
    
    plt.plot(x0, y0, 'b-', linewidth=2, alpha=0.8, label=f'0.0s延迟轨迹')
    plt.plot(x_opt, y_opt, 'r-', linewidth=2, alpha=0.8, label=f'{optimal_delay:.3f}s延迟轨迹')
    
    # 标记起点
    plt.plot(x0[0], y0[0], 'go', markersize=8, label='起点')
    plt.plot(x_opt[0], y_opt[0], 'go', markersize=8)
    
    # 自动设置显示范围
    all_x = np.concatenate([x0, x_opt])
    all_y = np.concatenate([y0, y_opt])
    
    margin = max(5, (all_x.max() - all_x.min()) * 0.1)  # 至少5米边距
    plt.xlim(all_x.min() - margin, all_x.max() + margin)
    plt.ylim(all_y.min() - margin, all_y.max() + margin)
    
    plt.xlabel('X坐标 (m)')
    plt.ylabel('Y坐标 (m)')
    
    # 构建更详细的标题信息
    title = f'{log_name} - 转弯段 {turn_info["id"]} 轨迹对比\n'
    title += f'时间: {turn_info["start"]:.1f}s - {turn_info["end"]:.1f}s '
    title += f'({turn_info["duration"]:.1f}s, {turn_info["total_angle"]:.1f}°)'
    
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 保存图片
    output_file = os.path.join(output_dir, f"{log_name}_turn_{turn_info['id']}_comparison.png")
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def process_log(log_dir, turn_analysis_dir):
    """处理单个日志文件"""
    log_name = os.path.basename(log_dir)
    print(f"处理日志: {log_name}")
    
    # 创建输出目录
    plots_dir = os.path.join(log_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    
    # 读取0.0延迟的ESKF轨迹文件
    gins_0_file = os.path.join(log_dir, "gins_offline.txt")
    data_0 = read_trajectory_file(gins_0_file)
    
    if data_0 is None:
        print(f"无法读取基准轨迹文件: {gins_0_file}")
        return
    
    # 绘制完整轨迹（静态和交互式）
    plot_full_trajectory(data_0, log_name, plots_dir)
    if PLOTLY_AVAILABLE:
        plot_full_trajectory_interactive(data_0, log_name, plots_dir)
    
    # 读取转弯段信息
    turn_file = os.path.join(turn_analysis_dir, f"{log_name}_turns_nzz.txt")
    turns = read_turn_segments(turn_file)
    
    if not turns:
        print(f"未找到转弯段信息，跳过转弯段对比图")
        return
    
    # 为每个转弯段单独查找最优延迟并绘制对比图
    for turn_info in turns:
        turn_id = turn_info['id']
        
        # 为该转弯段查找最优延迟
        optimal_delay = find_optimal_delay_for_turn(log_dir, turn_id)
        
        if optimal_delay is None:
            print(f"转弯段 {turn_id}: 未找到最优延迟，跳过")
            continue
        
        # 读取该转弯段的最优延迟轨迹文件
        if abs(optimal_delay) < 1e-6:  # 0.0延迟使用基准文件
            gins_opt_file = os.path.join(log_dir, "gins_offline.txt")
            data_optimal = data_0  # 直接使用已读取的基准数据
        else:
            delay_ms = int(optimal_delay * 1000)
            gins_opt_file = os.path.join(log_dir, f"gins_offline_{delay_ms:+d}ms.txt")
            data_optimal = read_trajectory_file(gins_opt_file)
        
        if data_optimal is None:
            print(f"转弯段 {turn_id}: 无法读取最优延迟轨迹文件: {gins_opt_file}")
            continue
        
        # 绘制该转弯段的对比图（静态和交互式）
        plot_turn_comparison(data_0, data_optimal, turn_info, optimal_delay, log_name, plots_dir)
        if PLOTLY_AVAILABLE:
            plot_turn_comparison_interactive(data_0, data_optimal, turn_info, optimal_delay, log_name, plots_dir)

def main():
    parser = argparse.ArgumentParser(description='绘制轨迹对比图')
    parser.add_argument('--input', required=True, help='结果目录路径')
    parser.add_argument('--log', help='指定处理的日志名称（可选）')
    
    args = parser.parse_args()
    
    input_dir = args.input
    if not os.path.exists(input_dir):
        print(f"输入目录不存在: {input_dir}")
        return 1
    
    # 转弯分析目录
    turn_analysis_dir = os.path.join(input_dir, "turn_analysis")
    if not os.path.exists(turn_analysis_dir):
        print(f"转弯分析目录不存在: {turn_analysis_dir}")
        return 1
    
    if args.log:
        # 处理指定日志
        log_dir = os.path.join(input_dir, args.log)
        if os.path.exists(log_dir):
            process_log(log_dir, turn_analysis_dir)
        else:
            print(f"指定的日志目录不存在: {log_dir}")
            return 1
    else:
        # 处理所有日志
        log_dirs = [d for d in os.listdir(input_dir) 
                   if os.path.isdir(os.path.join(input_dir, d)) and d.startswith('vdr_')]
        
        if not log_dirs:
            print(f"在 {input_dir} 中未找到日志目录")
            return 1
        
        for log_name in sorted(log_dirs):
            log_dir = os.path.join(input_dir, log_name)
            process_log(log_dir, turn_analysis_dir)
    
    print("=== 轨迹对比图生成完成 ===")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
