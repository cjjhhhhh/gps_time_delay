#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

def analyze_observability(data_file):
    """根据理论章节分析各状态的可观测度"""
    
    # 读取数据
    data = np.loadtxt(data_file)
    time = data[:, 0]
    covariance = data[:, 1:19]  # 18个对角元素（方差）
    
    # 转换为相对时间
    relative_time = time - time[0]
    
    # 状态变量标签
    state_labels = [
        'pos_x', 'pos_y', 'pos_z',           # 0-2: 位置
        'vel_x', 'vel_y', 'vel_z',           # 3-5: 速度
        'att_x', 'att_y', 'att_z',           # 6-8: 姿态
        'bg_x', 'bg_y', 'bg_z',              # 9-11: 陀螺偏差
        'ba_x', 'ba_y', 'ba_z',              # 12-14: 加速度偏差
        'grav_x', 'grav_y', 'grav_z'         # 15-17: 重力
    ]
    
    # 计算可观测度 σ_(k|j) = √P_0 / √P_k (初始标准差/当前标准差)
    initial_std = np.sqrt(covariance[0, :])  # 初始标准差 (18,)
    current_std = np.sqrt(covariance)        # 当前标准差 (时间点数, 18)
    
    # 避免除零错误
    current_std[current_std == 0] = 1e-10
    
    # 计算可观测度指标：初始标准差 / 当前标准差
    observability_ratio = initial_std / current_std
    
    print("=== 可观测度分析 ===")
    print("根据理论：")
    print("- 不可观测: σ ≤ 1")
    print("- 弱可观测: 1 < σ ≤ 2") 
    print("- 中等可观测: 2 < σ ≤ 10")
    print("- 强可观测: σ > 10")
    print()
    
    # 分析最终可观测度
    final_ratio = observability_ratio[-1, :]
    
    print("最终可观测度分析:")
    for i, label in enumerate(state_labels):
        ratio = final_ratio[i]
        if ratio <= 1:
            level = "不可观测"
        elif ratio <= 2:
            level = "弱可观测"
        elif ratio <= 10:
            level = "中等可观测"
        else:
            level = "强可观测"
        
        print(f"  {label:8s}: {ratio:.3f} ({level})")
    
    return relative_time, observability_ratio, state_labels

def plot_observability_curves(relative_time, observability_ratio, state_labels, data_file):
    """绘制可观测度变化曲线"""
    
    # 创建大图：3行6列
    fig, axes = plt.subplots(3, 6, figsize=(20, 12))
    fig.suptitle('状态可观测度变化分析 σ(k) = √P(0)/√P(k)', fontsize=16)
    
    axes_flat = axes.flatten()
    
    for i in range(18):
        ax = axes_flat[i]
        
        # 绘制可观测度比值曲线
        ax.plot(relative_time, observability_ratio[:, i], 'b-', linewidth=1.0)
        
        # 添加可观测度分级线
        ax.axhline(y=1, color='r', linestyle='--', alpha=0.7, label='不可观测阈值')
        ax.axhline(y=2, color='orange', linestyle='--', alpha=0.7, label='弱可观测阈值')
        ax.axhline(y=10, color='green', linestyle='--', alpha=0.7, label='强可观测阈值')
        
        ax.set_title(f'{state_labels[i]}', fontsize=10)
        ax.set_xlabel('时间(秒)', fontsize=8)
        ax.set_ylabel('σ(k)', fontsize=8)
        ax.set_yscale('log')  # 对数坐标
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # 只在第一个子图显示图例
        if i == 0:
            ax.legend(fontsize=6)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = data_file.replace('.txt', '_observability.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"保存可观测度图片: {output_file}")
    
    plt.show()

def plot_observability_log_curves(relative_time, observability_ratio, state_labels, data_file):
    """绘制对数可观测度曲线 k - lg(σ)"""
    
    # 选择几个关键状态进行详细分析
    key_states = [0, 3, 8, 9, 12, 15]  # pos_x, vel_x, att_z, bg_x, ba_x, grav_x
    key_labels = [state_labels[i] for i in key_states]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('关键状态可观测度对数曲线 lg(σ(k)) vs 时间', fontsize=16)
    
    axes_flat = axes.flatten()
    
    for idx, state_idx in enumerate(key_states):
        ax = axes_flat[idx]
        
        # 计算对数可观测度
        log_observability = np.log10(observability_ratio[:, state_idx])
        
        ax.plot(relative_time, log_observability, 'b-', linewidth=1.5)
        
        # 添加参考线
        ax.axhline(y=0, color='r', linestyle='--', alpha=0.7, label='lg(σ)=0 (σ=1)')
        ax.axhline(y=np.log10(2), color='orange', linestyle='--', alpha=0.7, label='lg(σ)=0.3 (σ=2)')
        ax.axhline(y=1, color='green', linestyle='--', alpha=0.7, label='lg(σ)=1 (σ=10)')
        
        ax.set_title(f'{key_labels[idx]} 可观测度对数曲线')
        ax.set_xlabel('时间(秒)')
        ax.set_ylabel('lg(σ(k))')
        ax.grid(True, alpha=0.3)
        
        if idx == 0:
            ax.legend(fontsize=8)
    
    plt.tight_layout()
    
    # 保存图片
    output_file = data_file.replace('.txt', '_observability_log.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"保存对数可观测度图片: {output_file}")
    
    plt.show()

def analyze_convergence_rate(relative_time, observability_ratio, state_labels):
    """分析收敛速度"""
    
    print("\n=== 收敛速度分析 ===")
    
    # 定义收敛判断标准：相对变化率小于1%
    convergence_threshold = 0.01
    
    for i, label in enumerate(state_labels):
        ratio_series = observability_ratio[:, i]
        
        # 计算变化率
        if len(ratio_series) > 100:
            # 取后100个点的变化率
            recent_changes = np.abs(np.diff(ratio_series[-100:]))
            mean_change_rate = np.mean(recent_changes) / ratio_series[-1]
            
            # 寻找收敛时间（变化率持续小于阈值）
            changes = np.abs(np.diff(ratio_series))
            relative_changes = changes / ratio_series[1:]
            
            # 找到第一次变化率小于阈值且持续50个点的时刻
            convergence_idx = None
            for j in range(50, len(relative_changes)):
                if np.all(relative_changes[j-50:j] < convergence_threshold):
                    convergence_idx = j
                    break
            
            if convergence_idx is not None:
                convergence_time = relative_time[convergence_idx]
                final_ratio = ratio_series[-1]
                
                print(f"  {label:8s}: 收敛时间={convergence_time:6.1f}s, 最终比值={final_ratio:.3f}")
            else:
                print(f"  {label:8s}: 未收敛")

def main():
    # 数据文件路径
    data_file = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/covariance.txt"
    
    if not os.path.exists(data_file):
        print(f"数据文件不存在: {data_file}")
        return
    
    # 进行可观测度分析
    relative_time, observability_ratio, state_labels = analyze_observability(data_file)
    
    # 绘制可观测度曲线
    plot_observability_curves(relative_time, observability_ratio, state_labels, data_file)
    
    # 绘制对数可观测度曲线
    plot_observability_log_curves(relative_time, observability_ratio, state_labels, data_file)
    
    # 分析收敛速度
    analyze_convergence_rate(relative_time, observability_ratio, state_labels)
    
    print("\n=== 可观测度评估建议 ===")
    print("1. σ > 10 的状态：可观测性强，滤波效果好")
    print("2. 2 < σ ≤ 10 的状态：中等可观测，需要足够观测时间")
    print("3. 1 < σ ≤ 2 的状态：弱可观测，精度提升有限")
    print("4. σ ≤ 1 的状态：不可观测，考虑移除或固定参数")

if __name__ == "__main__":
    main()