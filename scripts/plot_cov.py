#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

def plot_diagonal_covariance(data_file):
    """直接绘制P矩阵18个对角元素的变化"""
    
    # 读取数据
    try:
        data = np.loadtxt(data_file)
        print(f"加载数据: {data.shape}")
    except Exception as e:
        print(f"加载失败: {e}")
        return
    
    time = data[:, 0]  # 时间戳
    covariance = data[:, 1:19]  # 18个对角元素（方差）
    
    # 转换为相对时间（从0开始，更直观）
    relative_time = time - time[0]
    
    # 状态变量标签
    state_labels = [
        'pos_x', 'pos_y', 'pos_z',           # 0-2: 位置方差
        'vel_x', 'vel_y', 'vel_z',           # 3-5: 速度方差
        'att_x', 'att_y', 'att_z',           # 6-8: 姿态方差
        'bg_x', 'bg_y', 'bg_z',              # 9-11: 陀螺偏差方差
        'ba_x', 'ba_y', 'ba_z',              # 12-14: 加速度偏差方差
        'grav_x', 'grav_y', 'grav_z'         # 15-17: 重力方差
    ]
    
    # 创建大图：3行6列，显示所有18个状态
    fig, axes = plt.subplots(3, 6, figsize=(20, 12))
    fig.suptitle('P矩阵对角元素变化 (方差) - 观察锯齿状模式', fontsize=16)
    
    # 将axes展平为一维数组方便索引
    axes_flat = axes.flatten()
    
    for i in range(18):
        ax = axes_flat[i]
        
        # 绘制方差变化曲线
        ax.plot(relative_time, covariance[:, i], 'b-', linewidth=0.8, alpha=0.8)
        
        # 设置标题和标签
        ax.set_title(f'{state_labels[i]}', fontsize=10)
        ax.set_xlabel('时间(秒)', fontsize=8)
        ax.set_ylabel('方差', fontsize=8)
        
        # 使用对数坐标（方差变化范围很大）
        ax.set_yscale('log')
        ax.grid(True, alpha=0.3)
        
        # 调整刻度字体大小
        ax.tick_params(labelsize=8)
    
    plt.tight_layout()
    plt.show()

def plot_key_states(data_file):
    """重点观察几个关键状态的锯齿变化"""
    
    # 读取数据
    data = np.loadtxt(data_file)
    time = data[:, 0]
    covariance = data[:, 1:19]
    relative_time = time - time[0]
    
    # 创建2x2子图，重点观察关键状态
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('关键状态方差变化 - 锯齿状模式观察', fontsize=16)
    
    # 位置X方差
    axes[0,0].plot(relative_time, covariance[:, 0], 'r-', linewidth=1.2)
    axes[0,0].set_title('位置X方差 (pos_x)')
    axes[0,0].set_xlabel('时间(秒)')
    axes[0,0].set_ylabel('方差')
    axes[0,0].set_yscale('log')
    axes[0,0].grid(True, alpha=0.3)
    
    # 速度X方差
    axes[0,1].plot(relative_time, covariance[:, 3], 'g-', linewidth=1.2)
    axes[0,1].set_title('速度X方差 (vel_x)')
    axes[0,1].set_xlabel('时间(秒)')
    axes[0,1].set_ylabel('方差')
    axes[0,1].set_yscale('log')
    axes[0,1].grid(True, alpha=0.3)
    
    # 姿态Z方差（偏航角）
    axes[1,0].plot(relative_time, covariance[:, 8], 'b-', linewidth=1.2)
    axes[1,0].set_title('姿态Z方差 (att_z, 偏航角)')
    axes[1,0].set_xlabel('时间(秒)')
    axes[1,0].set_ylabel('方差')
    axes[1,0].set_yscale('log')
    axes[1,0].grid(True, alpha=0.3)
    
    # 陀螺X偏差方差
    axes[1,1].plot(relative_time, covariance[:, 9], 'm-', linewidth=1.2)
    axes[1,1].set_title('陀螺X偏差方差 (bg_x)')
    axes[1,1].set_xlabel('时间(秒)')
    axes[1,1].set_ylabel('方差')
    axes[1,1].set_yscale('log')
    axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = data_file.replace('.txt', '_key_states.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"保存图片: {output_file}")
    
    plt.show()

def analyze_sawtooth_pattern(data_file):
    """分析锯齿状模式的统计特征"""
    
    data = np.loadtxt(data_file)
    time = data[:, 0]
    covariance = data[:, 1:19]
    
    print("=== 锯齿状模式分析 ===")
    print(f"数据时长: {(time[-1] - time[0]):.2f} 秒")
    print(f"数据点数: {len(time)}")
    
    # 分析位置X方差的变化模式
    pos_x_var = covariance[:, 0]
    
    # 计算一阶差分（变化率）
    diff_var = np.diff(pos_x_var)
    
    # 统计增长和下降的次数
    increases = np.sum(diff_var > 0)
    decreases = np.sum(diff_var < 0)
    
    # 找到大幅下降（可能的GPS更新）
    threshold = -2 * np.std(diff_var)
    big_drops = np.sum(diff_var < threshold)
    
    print(f"\n位置X方差变化分析:")
    print(f"  增长次数: {increases}")
    print(f"  下降次数: {decreases}")
    print(f"  大幅下降次数 (可能GPS更新): {big_drops}")
    print(f"  平均增长率: {np.mean(diff_var[diff_var > 0]):.2e}")
    print(f"  平均下降率: {np.mean(diff_var[diff_var < 0]):.2e}")
    
    # 计算方差的变化范围
    var_max = np.max(pos_x_var)
    var_min = np.min(pos_x_var)
    var_ratio = var_max / var_min
    
    print(f"\n位置X方差范围:")
    print(f"  最大值: {var_max:.2e}")
    print(f"  最小值: {var_min:.2e}")
    print(f"  最大/最小比值: {var_ratio:.2f}")

def main():
    # 数据文件路径（修改为你的实际路径）
    data_file = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/covariance_new1.txt"
    
    # 检查文件是否存在
    if not os.path.exists(data_file):
        print(f"数据文件不存在: {data_file}")
        return
    
    print("1. 绘制所有18个状态的P矩阵对角元素...")
    plot_diagonal_covariance(data_file)
    
    print("2. 重点观察关键状态的锯齿变化...")
    plot_key_states(data_file)
    
    print("3. 分析锯齿状模式...")
    analyze_sawtooth_pattern(data_file)
    
    print("\n分析完成！")
    print("观察要点:")
    print("- 收敛后应该看到锯齿状模式")
    print("- IMU预测时方差缓慢增长")
    print("- GPS更新时方差突然下降")
    print("- 锯齿的频率对应GPS更新频率")

if __name__ == "__main__":
    main()