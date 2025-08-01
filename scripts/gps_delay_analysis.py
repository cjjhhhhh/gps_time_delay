import numpy as np

# ======= 用户手动指定文件路径 ========
file_no_delay = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/corrections.txt"           # 无延迟corrections文件
file_with_delay = "/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/corrections_-350ms.txt"    # 有延迟corrections文件
# =====================================

# ======= 用户手动设置时间戳范围 ========
start_time = 868905.770  # 设置起始时间戳（单位：秒）
end_time = 869075.894    # 设置结束时间戳（单位：秒）
# =====================================

def load_corrections(file_path, start_time, end_time):
    """读取corrections文件，并根据时间戳过滤数据"""
    data = np.loadtxt(file_path)
    timestamps = data[:, 0]
    corrections = data[:, 1:4]  # 位置修正 dx dy dz
    
    # 只保留在指定时间段内的数据
    mask = (timestamps >= start_time) & (timestamps <= end_time)
    filtered_data = data[mask]
    
    return filtered_data[:, 1:3]  # 只返回 dx, dy

def calculate_planar_rms(corrections):
    """计算平面位置修正量的RMS（仅dx和dy）"""
    dx = corrections[:, 0]  # x方向修正
    dy = corrections[:, 1]  # y方向修正
    
    # 计算平面RMS：sqrt(dx²+dy²)的均方根
    planar_errors = np.sqrt(dx**2 + dy**2)
    planar_rms = np.sqrt(np.mean(planar_errors**2))
    
    return planar_rms

if __name__ == "__main__":
    try:
        # 加载数据
        corrections_no_delay = load_corrections(file_no_delay, start_time, end_time)
        corrections_with_delay = load_corrections(file_with_delay, start_time, end_time)
        
        # 计算RMS
        rms_no_delay = calculate_planar_rms(corrections_no_delay)
        rms_with_delay = calculate_planar_rms(corrections_with_delay)
        
        # 输出结果
        print(f"无延迟平面RMS: {rms_no_delay:.4f} m")
        print(f"有延迟平面RMS: {rms_with_delay:.4f} m")
        print(f"RMS差异: {rms_with_delay - rms_no_delay:.4f} m")
        
    except FileNotFoundError as e:
        print(f"错误: 文件未找到")
    except Exception as e:
        print(f"错误: {e}")