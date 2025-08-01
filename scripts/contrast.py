# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

# 新格式：timestamp px py pz qw qx qy qz vx vy vz bgx bgy bgz bax bay baz gps_px gps_py gps_pz gps_valid
if __name__ == '__main__':
    # 固定的两个文件路径
    file1_path = "/Users/cjj/Data/log_results/Honor_V40/vdr_20250617_102928_117/gins_offline.txt"
    file2_path = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/bin/gins_offline.txt"
    
    try:
        # 读取两个文件
        path_data1 = np.loadtxt(file1_path)
        path_data2 = np.loadtxt(file2_path)
        print(f"File 1 loaded: {path_data1.shape[0]} points")
        print(f"File 2 loaded: {path_data2.shape[0]} points")
        
        # 打印数据范围进行调试
        print(f"File 1 X range: [{path_data1[:, 1].min():.2f}, {path_data1[:, 1].max():.2f}]")
        print(f"File 1 Y range: [{path_data1[:, 2].min():.2f}, {path_data1[:, 2].max():.2f}]")
        print(f"File 2 X range: [{path_data2[:, 1].min():.2f}, {path_data2[:, 1].max():.2f}]")
        print(f"File 2 Y range: [{path_data2[:, 2].min():.2f}, {path_data2[:, 2].max():.2f}]")
        
        # 计算两个轨迹的距离差异
        if path_data1.shape[0] == path_data2.shape[0]:
            diff_x = np.abs(path_data1[:, 1] - path_data2[:, 1])
            diff_y = np.abs(path_data1[:, 2] - path_data2[:, 2])
            print(f"Max X difference: {diff_x.max():.6f}")
            print(f"Max Y difference: {diff_y.max():.6f}")
            print(f"Mean X difference: {diff_x.mean():.6f}")
            print(f"Mean Y difference: {diff_y.mean():.6f}")
        
    except Exception as e:
        print(f"Error loading files: {e}")
        exit(1)
    
    def plot_gps_points(ax, path_data1, path_data2, label_prefix="", print_stats=False):
        """绘制GPS点 - 简化版本"""
        # 绘制文件1的GPS点 - 红色
        if path_data1.shape[1] >= 20:
            gps_valid1 = path_data1[:, 20] if path_data1.shape[1] > 20 else np.ones(len(path_data1))
            valid_gps_mask1 = gps_valid1 == 1
            if np.sum(valid_gps_mask1) > 0:
                ax.scatter(path_data1[valid_gps_mask1, 17], path_data1[valid_gps_mask1, 18], 
                          s=8, c='red', alpha=0.8, marker='o', label=f'{label_prefix}gins_delay GPS')
        
        # 绘制文件2的GPS点 - 橙色
        if path_data2.shape[1] >= 20:
            gps_valid2 = path_data2[:, 20] if path_data2.shape[1] > 20 else np.ones(len(path_data2))
            valid_gps_mask2 = gps_valid2 == 1
            if np.sum(valid_gps_mask2) > 0:
                ax.scatter(path_data2[valid_gps_mask2, 17], path_data2[valid_gps_mask2, 18], 
                          s=8, c='orange', alpha=0.8, marker='s', label=f'{label_prefix}gins_origin GPS')
        
        # 只在第一次调用时打印统计信息
        if print_stats:
            gps_count1 = np.sum(gps_valid1 == 1) if path_data1.shape[1] >= 20 else 0
            gps_count2 = np.sum(gps_valid2 == 1) if path_data2.shape[1] >= 20 else 0
            print(f"GPS点统计:")
            print(f"  gins_delay GPS点数: {gps_count1}")
            print(f"  gins_origin GPS点数: {gps_count2}")
            print(f"  注意: 如果点重合，只能看到一个颜色（后画的橙色会覆盖红色）")
    
    plt.rcParams['figure.figsize'] = (16.0, 12.0)
    
    # 创建交互式图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16.0, 12.0))
    
    # 添加交互功能
    def on_hover(event):
        if event.inaxes in [ax1, ax2]:
            # 检查是否在scatter点上
            for scatter_obj, data, label in event.inaxes.scatter_objects:
                contains, info = scatter_obj.contains(event)
                if contains:
                    ind = info['ind'][0]
                    x, y = data[ind, 1], data[ind, 2]
                    vx, vy, vz = data[ind, 8], data[ind, 9], data[ind, 10]
                    v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
                    time = data[ind, 0]
                    
                    # 显示信息
                    tooltip_text = f'{label}\nTime: {time:.2f}s\nPos: ({x:.2f}, {y:.2f})\nVel: ({vx:.2f}, {vy:.2f}, {vz:.2f})\nSpeed: {v_mag:.2f} m/s'
                    
                    # 清除之前的annotation
                    for child in event.inaxes.get_children():
                        if hasattr(child, 'get_text') and ('Time:' in str(child.get_text()) or 'gins_' in str(child.get_text())):
                            child.remove()
                    
                    # 添加新的annotation
                    event.inaxes.annotate(tooltip_text, xy=(x, y), xytext=(20, 20), 
                                        textcoords='offset points', 
                                        bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.8),
                                        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
                    plt.draw()
                    break

    # 轨迹 - 完整视图
    plt.sca(ax1)
    
    # 文件1 - 蓝色虚线
    ax1.plot(path_data1[:, 1], path_data1[:, 2], 'b--', linewidth=1.5, alpha=0.8, label='gins_delay trajectory')
    scatter1_1 = ax1.scatter(path_data1[:, 1], path_data1[:, 2], s=4, c='blue', alpha=0.9, label='gins_delay points', zorder=5)
    
    # 文件2 - 绿色虚线
    ax1.plot(path_data2[:, 1], path_data2[:, 2], 'g--', linewidth=1.5, alpha=0.8, label='gins_origin trajectory')
    scatter1_2 = ax1.scatter(path_data2[:, 1], path_data2[:, 2], s=4, c='green', alpha=0.8, label='gins_origin points', zorder=4)
    
    # 保存scatter对象用于交互
    ax1.scatter_objects = [
        (scatter1_1, path_data1, 'gins_delay'),
        (scatter1_2, path_data2, 'gins_origin')
    ]
    
    # 绘制GPS点（只在第一次打印统计信息）
    plot_gps_points(ax1, path_data1, path_data2, "", print_stats=True)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.grid()
    ax1.set_title('2D trajectory comparison (Blue/Green dashed lines, Red=gins_delay GPS, Orange=gins_origin GPS)')
    ax1.legend()
    
    # 添加固定窗口显示
    plt.sca(ax2)
    
    # 文件1 - 蓝色虚线
    ax2.plot(path_data1[:, 1], path_data1[:, 2], 'b--', linewidth=1.5, alpha=0.8, label='gins_delay trajectory')
    scatter2_1 = ax2.scatter(path_data1[:, 1], path_data1[:, 2], s=4, c='blue', alpha=0.9, label='gins_delay points', zorder=5)
    
    # 文件2 - 绿色虚线
    ax2.plot(path_data2[:, 1], path_data2[:, 2], 'g--', linewidth=1.5, alpha=0.8, label='gins_origin trajectory')
    scatter2_2 = ax2.scatter(path_data2[:, 1], path_data2[:, 2], s=4, c='green', alpha=0.8, label='gins_delay points', zorder=4)
    
    # 保存scatter对象用于交互
    ax2.scatter_objects = [
        (scatter2_1, path_data1, 'gins_delay'),
        (scatter2_2, path_data2, 'gins_origin')
    ]
    
    # 绘制GPS点（不打印统计信息）
    plot_gps_points(ax2, path_data1, path_data2, "", print_stats=False)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.grid()
    ax2.set_xlim(-5900, -5800)  # 设置x轴范围
    ax2.set_ylim(12300, 12900)  # 设置y轴范围
    ax2.set_title('Fixed Window: X[-5900,-5800], Y[12300,12900] (Red=gins_delay GPS, Orange=gins_origin GPS)')
    ax2.legend()
    
    # 连接鼠标悬停事件
    fig.canvas.mpl_connect('motion_notify_event', on_hover)

    # 姿态对比
    plt.figure(figsize=(16, 12))  # 创建新图，增加高度
    
    # 姿态对比 - 文件1
    plt.subplot(221)
    plt.plot(path_data1[:, 0], path_data1[:, 4], 'r', label='qw')
    plt.plot(path_data1[:, 0], path_data1[:, 5], 'g', label='qx')
    plt.plot(path_data1[:, 0], path_data1[:, 6], 'b', label='qy')
    plt.plot(path_data1[:, 0], path_data1[:, 7], 'k', label='qz')
    plt.title('gins_delay quaternion')
    plt.legend()
    plt.grid()

    # 姿态对比 - 文件2
    plt.subplot(222)
    plt.plot(path_data2[:, 0], path_data2[:, 4], 'r', label='qw')
    plt.plot(path_data2[:, 0], path_data2[:, 5], 'g', label='qx')
    plt.plot(path_data2[:, 0], path_data2[:, 6], 'b', label='qy')
    plt.plot(path_data2[:, 0], path_data2[:, 7], 'k', label='qz')
    plt.title('gins_origin quaternion')
    plt.legend()
    plt.grid()

    # 速度对比 - 文件1
    plt.subplot(223)
    plt.plot(path_data1[:, 0], path_data1[:, 8], 'r', label='vx')
    plt.plot(path_data1[:, 0], path_data1[:, 9], 'g', label='vy')
    plt.plot(path_data1[:, 0], path_data1[:, 10], 'b', label='vz')
    plt.title('gins_delay velocity')
    plt.legend()
    plt.grid()
    
    # 速度对比 - 文件2
    plt.subplot(224)
    plt.plot(path_data2[:, 0], path_data2[:, 8], 'r', label='vx')
    plt.plot(path_data2[:, 0], path_data2[:, 9], 'g', label='vy')
    plt.plot(path_data2[:, 0], path_data2[:, 10], 'b', label='vz')
    plt.title('gins_origin velocity')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
    exit(0)