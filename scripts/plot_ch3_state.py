# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

# 新格式：timestamp px py pz qw qx qy qz vx vy vz bgx bgy bgz bax bay baz gps_px gps_py gps_pz gps_valid
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        plt.rcParams['figure.figsize'] = (12.0, 10.0)
        
        # 创建交互式图形 - 只保留一个子图
        fig, ax = plt.subplots(1, 1, figsize=(12.0, 10.0))
        
        # 添加交互功能
        def on_hover(event):
            if event.inaxes == ax:
                # 找到最近的点
                if hasattr(event.inaxes, 'scatter_points'):
                    contains, info = event.inaxes.scatter_points.contains(event)
                    if contains:
                        ind = info['ind'][0]
                        x, y = path_data[ind, 1], path_data[ind, 2]
                        vx, vy, vz = path_data[ind, 8], path_data[ind, 9], path_data[ind, 10]
                        v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
                        time = path_data[ind, 0]
                        
                        # 显示信息
                        tooltip_text = f'Time: {time:.2f}s\nPos: ({x:.2f}, {y:.2f})\nVel: ({vx:.2f}, {vy:.2f}, {vz:.2f})\nSpeed: {v_mag:.2f} m/s'
                        
                        # 清除之前的annotation
                        for child in event.inaxes.get_children():
                            if hasattr(child, 'get_text') and 'Time:' in str(child.get_text()):
                                child.remove()
                        
                        # 添加新的annotation
                        event.inaxes.annotate(tooltip_text, xy=(x, y), xytext=(20, 20), 
                                            textcoords='offset points', 
                                            bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.8),
                                            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
                        plt.draw()

        # ESKF轨迹
        ax.plot(path_data[:, 1], path_data[:, 2], 'b-', linewidth=1, alpha=0.7, label='ESKF trajectory')
        scatter = ax.scatter(path_data[:, 1], path_data[:, 2], s=2, c='blue', alpha=0.8, label='ESKF points')
        ax.scatter_points = scatter  # 保存scatter对象用于交互
        
        # GPS观测点（如果数据格式包含GPS列）
        if path_data.shape[1] >= 20:
            # 筛选有效的GPS观测点
            gps_valid = path_data[:, 20] if path_data.shape[1] > 20 else np.ones(len(path_data))
            valid_gps_mask = gps_valid == 1
            if np.sum(valid_gps_mask) > 0:
                ax.scatter(path_data[valid_gps_mask, 17], path_data[valid_gps_mask, 18], 
                           s=8, c='red', alpha=0.8, marker='o', label='GPS observations')
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.grid(True, alpha=0.3)
        ax.set_title('2D Trajectory - Equal Scale (hover for velocity info)')
        ax.legend()
        
        # 关键修改：设置等比例显示
        ax.set_aspect('equal', adjustable='box')
        
        # 连接鼠标悬停事件
        fig.canvas.mpl_connect('motion_notify_event', on_hover)

        # 姿态
        plt.figure(figsize=(16, 6))  # 创建新图
        plt.subplot(121)
        plt.plot(path_data[:, 0], path_data[:, 4], 'r')
        plt.plot(path_data[:, 0], path_data[:, 5], 'g')
        plt.plot(path_data[:, 0], path_data[:, 6], 'b')
        plt.plot(path_data[:, 0], path_data[:, 7], 'k')
        plt.title('q')
        plt.legend(['qw', 'qx', 'qy', 'qz'])

        # 速度
        plt.subplot(122)
        plt.plot(path_data[:, 0], path_data[:, 8], 'r')
        plt.plot(path_data[:, 0], path_data[:, 9], 'g')
        plt.plot(path_data[:, 0], path_data[:, 10], 'b')
        plt.title('v')
        plt.legend(['vx', 'vy', 'vz'])

        plt.show()
        exit(1)