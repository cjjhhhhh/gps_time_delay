import matplotlib.pyplot as plt

# ======= 手动输入每个时间延迟的RMS ========
time_offsets = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]

# 直接在代码中输入每个时间偏移量的RMS值
rms_values = [
    3.26,  # 对应0.0秒偏移的RMS值
    3.21,  # 对应0.05秒偏移的RMS值
    3.23,  # 对应0.10秒偏移的RMS值
    3.24,  # 对应0.15秒偏移的RMS值
    3.00,  # 对应0.20秒偏移的RMS值
    3.18,  # 对应0.25秒偏移的RMS值
    3.12,  # 对应0.30秒偏移的RMS值
    3.38,  # 对应0.35秒偏移的RMS值
    3.27   # 对应0.40秒偏移的RMS值
]

# =====================================

def plot_rms_comparison(time_offsets, rms_values):
    # 绘制时间偏移量与RMS的关系图
    plt.plot(time_offsets, rms_values, marker='o')
    plt.xlabel("Time Offsets Adjusted from the Original Timestamps [s]")
    plt.ylabel("RMS of Corrections to Position Estimates [m]")
    plt.title("RMS of Corrections for Different Time Offsets")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# 绘制图表
plot_rms_comparison(time_offsets, rms_values)
