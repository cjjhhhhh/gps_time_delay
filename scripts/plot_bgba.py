#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制ESKF输出的陀螺零偏(bg)和加速度零偏(ba)数据
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

class BiasPlotter:
    def __init__(self, data_file):
        self.data_file = data_file
        self.timestamps = []
        self.bg_data = []  # 陀螺零偏 [bg_x, bg_y, bg_z]
        self.ba_data = []  # 加速度零偏 [ba_x, ba_y, ba_z]
        
    def parse_data(self):
        """解析ESKF输出数据文件"""
        print(f"正在解析数据文件: {self.data_file}")
        
        try:
            with open(self.data_file, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    
                    parts = line.split()
                    if len(parts) < 18:  # 至少需要18列数据
                        continue
                    
                    # 数据格式: timestamp p_x p_y p_z q_w q_x q_y q_z v_x v_y v_z bg_x bg_y bg_z ba_x ba_y ba_z ...
                    timestamp = float(parts[0])
                    
                    # 提取bg数据 (第12-14列，索引11-13)
                    bg_x = float(parts[11])
                    bg_y = float(parts[12])
                    bg_z = float(parts[13])
                    
                    # 提取ba数据 (第15-17列，索引14-16)
                    ba_x = float(parts[14])
                    ba_y = float(parts[15])
                    ba_z = float(parts[16])
                    
                    self.timestamps.append(timestamp)
                    self.bg_data.append([bg_x, bg_y, bg_z])
                    self.ba_data.append([ba_x, ba_y, ba_z])
                    
                    if line_num % 10000 == 0:
                        print(f"已解析 {line_num} 行数据...")
                        
        except FileNotFoundError:
            print(f"错误：找不到文件 {self.data_file}")
            return False
        except Exception as e:
            print(f"解析文件时出错: {e}")
            return False
        
        # 转换为numpy数组
        self.timestamps = np.array(self.timestamps)
        self.bg_data = np.array(self.bg_data)
        self.ba_data = np.array(self.ba_data)
        
        print(f"解析完成，共获得 {len(self.timestamps)} 条记录")
        return len(self.timestamps) > 0
    
    def plot_bias_data(self, save_path=None):
        """绘制零偏数据"""
        if len(self.timestamps) == 0:
            print("没有数据可绘制")
            return
        
        # 使用绝对时间戳
        time_data = self.timestamps
        
        # 设置中文字体
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建6×1子图
        fig, axes = plt.subplots(6, 1, figsize=(12, 16))
        fig.suptitle('陀螺零偏(bg)和加速度零偏(ba)随时间变化', fontsize=16, fontweight='bold')
        
        # 陀螺零偏标签和单位
        bg_labels = ['陀螺零偏 bg_x', '陀螺零偏 bg_y', '陀螺零偏 bg_z']
        bg_unit = '(rad/s)'
        
        # 加速度零偏标签和单位
        ba_labels = ['加速度零偏 ba_x', '加速度零偏 ba_y', '加速度零偏 ba_z']
        ba_unit = '(m/s²)'
        
        # 绘制陀螺零偏 (前三行)
        for i in range(3):
            ax = axes[i]
            ax.plot(time_data, self.bg_data[:, i], 'b-', linewidth=0.8, alpha=0.8)
            ax.set_title(f'{bg_labels[i]} {bg_unit}', fontsize=12)
            ax.set_ylabel('零偏值 (rad/s)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(min(time_data), max(time_data))
            
            # 添加统计信息
            mean_val = np.mean(self.bg_data[:, i])
            std_val = np.std(self.bg_data[:, i])
            ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.8, 
                      label=f'均值: {mean_val:.6f}')
            ax.axhline(y=mean_val + std_val, color='r', linestyle=':', alpha=0.6,
                      label=f'±1σ: {std_val:.6f}')
            ax.axhline(y=mean_val - std_val, color='r', linestyle=':', alpha=0.6)
            ax.legend(fontsize=8)
        
        # 绘制加速度零偏 (后三行)
        for i in range(3):
            ax = axes[i + 3]
            ax.plot(time_data, self.ba_data[:, i], 'g-', linewidth=0.8, alpha=0.8)
            ax.set_title(f'{ba_labels[i]} {ba_unit}', fontsize=12)
            ax.set_ylabel('零偏值 (m/s²)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(min(time_data), max(time_data))
            
            # 添加统计信息
            mean_val = np.mean(self.ba_data[:, i])
            std_val = np.std(self.ba_data[:, i])
            ax.axhline(y=mean_val, color='r', linestyle='--', alpha=0.8, 
                      label=f'均值: {mean_val:.6f}')
            ax.axhline(y=mean_val + std_val, color='r', linestyle=':', alpha=0.6,
                      label=f'±1σ: {std_val:.6f}')
            ax.axhline(y=mean_val - std_val, color='r', linestyle=':', alpha=0.6)
            ax.legend(fontsize=8)
        
        # 只有最后一个子图显示x轴标签
        axes[-1].set_xlabel('time', fontsize=10)
        
        plt.tight_layout()
        
        # 保存图表
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"图表已保存到: {save_path}")
        
        plt.show()
    
    def print_statistics(self):
        """打印统计信息"""
        if len(self.timestamps) == 0:
            print("没有数据可统计")
            return
        
        print("\n=== 零偏数据统计信息 ===")
        print(f"数据点数量: {len(self.timestamps)}")
        print(f"时间戳范围: {self.timestamps[0]} - {self.timestamps[-1]}")
        print(f"时间跨度: {(self.timestamps[-1] - self.timestamps[0]):.1f} 秒")
        
        # 陀螺零偏统计
        print(f"\n陀螺零偏(bg)统计:")
        bg_labels = ['bg_x', 'bg_y', 'bg_z']
        for i in range(3):
            mean_val = np.mean(self.bg_data[:, i])
            std_val = np.std(self.bg_data[:, i])
            min_val = np.min(self.bg_data[:, i])
            max_val = np.max(self.bg_data[:, i])
            print(f"  {bg_labels[i]}: 均值={mean_val:.8f}, 标准差={std_val:.8f}, "
                  f"最小值={min_val:.8f}, 最大值={max_val:.8f}")
        
        # 加速度零偏统计
        print(f"\n加速度零偏(ba)统计:")
        ba_labels = ['ba_x', 'ba_y', 'ba_z']
        for i in range(3):
            mean_val = np.mean(self.ba_data[:, i])
            std_val = np.std(self.ba_data[:, i])
            min_val = np.min(self.ba_data[:, i])
            max_val = np.max(self.ba_data[:, i])
            print(f"  {ba_labels[i]}: 均值={mean_val:.8f}, 标准差={std_val:.8f}, "
                  f"最小值={min_val:.8f}, 最大值={max_val:.8f}")
        
        # 收敛性分析
        if len(self.timestamps) > 1000:
            print(f"\n收敛性分析（最后10%数据）:")
            tail_length = len(self.timestamps) // 10
            
            print("  陀螺零偏收敛性:")
            for i in range(3):
                tail_std = np.std(self.bg_data[-tail_length:, i])
                print(f"    {bg_labels[i]}标准差: {tail_std:.8f}")
            
            print("  加速度零偏收敛性:")
            for i in range(3):
                tail_std = np.std(self.ba_data[-tail_length:, i])
                print(f"    {ba_labels[i]}标准差: {tail_std:.8f}")

def main():
    parser = argparse.ArgumentParser(description='绘制ESKF零偏数据')
    parser.add_argument('data_file', help='ESKF输出数据文件路径')
    parser.add_argument('--save', '-s', help='保存图表的路径')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not Path(args.data_file).exists():
        print(f"错误：文件 {args.data_file} 不存在")
        return
    
    # 创建绘图器
    plotter = BiasPlotter(args.data_file)
    
    # 解析数据
    if plotter.parse_data():
        # 打印统计信息
        plotter.print_statistics()
        
        # 绘制图表
        plotter.plot_bias_data(args.save)
    else:
        print("数据解析失败")

if __name__ == "__main__":
    # 如果没有命令行参数，使用默认路径
    import sys
    if len(sys.argv) == 1:
        # 修改为你的数据文件路径
        data_file_path = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/bin/gins_offline.txt"
        save_path = "bias_plot.png"
        
        print(f"使用默认路径: {data_file_path}")
        
        if not Path(data_file_path).exists():
            print(f"错误：文件 {data_file_path} 不存在")
            sys.exit(1)
        
        plotter = BiasPlotter(data_file_path)
        
        if plotter.parse_data():
            plotter.print_statistics()
            plotter.plot_bias_data(save_path)
        else:
            print("数据解析失败")
    else:
        main()