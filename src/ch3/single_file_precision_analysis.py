#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from scipy.interpolate import interp1d
import argparse
import utm
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def try_read_file(file_path, encodings=['utf-8', 'gbk', 'gb2312', 'latin-1', 'cp1252']):
    """尝试用多种编码读取文件"""
    for encoding in encodings:
        try:
            with open(file_path, 'r', encoding=encoding) as f:
                return f.read(), encoding
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError(f"无法用任何编码读取文件: {file_path}")

def get_vdr_time_mapping(vdr_file):
    """从VDR文件建立tick时间戳与UTC时间的映射关系"""
    print("建立VDR时间映射...")
    
    content, encoding = try_read_file(vdr_file)
    
    # 正则表达式匹配
    RE_STR = re.compile(r"\$STR ([\d]{4}-[\d]{2}-[\d]{2} [\d]{2}:[\d]{2}:[\d]+[\.][\d]+) .*")
    RE_TICK = re.compile(r"\$(ACC|GYR) ([\d]*) .*")
    
    utc_time = None
    tick_time = None
    li_time = []
    
    for line in content.split('\n'):
        line = line.strip()
        if not line:
            continue
            
        # 匹配UTC时间
        utc_match = RE_STR.match(line)
        if utc_match:
            try:
                utc_time_str = utc_match.groups()[0]
                utc_time = datetime.strptime(utc_time_str, "%Y-%m-%d %H:%M:%S.%f")
            except ValueError:
                continue
        
        # 匹配tick时间
        tick_match = RE_TICK.match(line)
        if tick_match:
            try:
                tick_time = int(tick_match.groups()[1])  # 毫秒
            except ValueError:
                continue
        
        # 如果两个时间都有，添加到列表
        if utc_time is not None and tick_time is not None:
            li_time.append([utc_time, tick_time])
            utc_time = None
            tick_time = None
    
    if not li_time:
        print("警告: 未找到时间映射数据")
        return None
    
    # 创建DataFrame
    df_time = pd.DataFrame(li_time, columns=["utc", "tick"])
    
    # 去重并排序
    df_time.drop_duplicates(subset=["utc"], keep="first", inplace=True)
    df_time.drop_duplicates(subset=["tick"], keep="first", inplace=True)
    df_time.sort_values(by="utc", inplace=True)
    df_time.reset_index(drop=True, inplace=True)
    
    # 转换UTC时间为时间戳（毫秒）
    df_time["utc_ts"] = (pd.to_datetime(df_time["utc"], format="mixed").astype('int64') // 1e6).astype('int64')
    
    print(f"建立时间映射: {len(df_time)} 个时间点")
    if len(df_time) > 0:
        print(f"时间范围: {df_time['utc'].iloc[0]} 到 {df_time['utc'].iloc[-1]}")
        print(f"Tick范围: {df_time['tick'].iloc[0]} 到 {df_time['tick'].iloc[-1]}")
    
    return df_time

def create_time_interpolator(df_time):
    """创建时间插值函数"""
    if df_time is None or len(df_time) < 2:
        print("警告: 时间映射数据不足，无法创建插值函数")
        return None
    
    try:
        interp_func = interp1d(
            df_time["tick"],
            df_time["utc_ts"], 
            kind="linear",
            fill_value="extrapolate"
        )
        print("时间插值函数创建成功")
        return interp_func
    except Exception as e:
        print(f"创建插值函数失败: {e}")
        return None

def getVdrData(vdr_file):
    """提取VDR文件中的GPS数据 - 采用参考代码的简单直接方式"""
    li_gps = []
    li_pos_feat = []
    li_azi_feat = []
    
    content, encoding = try_read_file(vdr_file)
    
    for line in content.split('\n'):
        if line.startswith("$GPS"):
            try:
                gps = re.split(r" ", line.strip())
                if len(gps) >= 25:  # 确保有足够的字段
                    # 构建UTC时间字符串
                    UTC = "{}-{:0>2d}-{:0>2d} {:0>2d}:{:0>2d}:{:0>2d}".format(
                        int(gps[19]),  # 年份
                        int(gps[20]),  # 月份
                        int(gps[21]),  # 日期
                        int(gps[22]),  # 小时
                        int(gps[23]),  # 分钟
                        int(gps[24]),  # 秒钟
                    )
                    
                    # 提取GPS数据
                    li_gps.append([
                        UTC,
                        float(gps[8]) / 1e7,  # wgs84 lat (纬度)
                        float(gps[7]) / 1e7,  # wgs84 lon (经度)
                        float(gps[9]),        # heading (航向)
                    ])
            except (ValueError, IndexError):
                continue
    
    print(f"提取GPS数据: {len(li_gps)} 条")
    return li_gps

def getCptData(cpt_file):
    """提取CPT文件中的真值数据，采用与参考代码一致的解析逻辑"""
    import math
    li_cpt = []
    
    # 从文件名提取日期信息 - 修复日期解析逻辑
    base_name = os.path.basename(cpt_file)
    date_match = re.match(r"([0-9]+)", base_name)
    
    if date_match:
        date_infp = date_match.group(1)
        
        # 根据数字长度判断格式
        if len(date_infp) == 4:  # 如 "0708" -> 2025-07-08
            year = "2025"  # 默认年份
            month = date_infp[:2]
            day = date_infp[2:]
        elif len(date_infp) >= 6:  # 如 "20250708" 或更长
            if len(date_infp) == 6:  # "250708" -> 2025-07-08
                year = "20" + date_infp[:2]
                month = date_infp[2:4]
                day = date_infp[4:6]
            else:  # "20250708" -> 2025-07-08
                year = date_infp[:4]
                month = date_infp[4:6]
                day = date_infp[6:8]
        else:
            # 默认值
            year = "2025"
            month = "07"
            day = "08"
    else:
        # 默认值
        year = "2025"
        month = "07"
        day = "08"
    
    print(f"从文件名 {base_name} 解析日期: {year}-{month}-{day}")
    
    content, encoding = try_read_file(cpt_file)
    
    UTC = ""
    for line in content.split('\n'):
        line = line.strip()
        if not line:
            continue
            
        # 先从$GPGGA行提取时间信息
        if line.startswith("$GPGGA"):
            cpt_info = re.split(r" |,", line.strip())
            if len(cpt_info) > 1 and cpt_info[1].endswith("00"):  # 只保留整秒时刻
                try:
                    # 直接在这里转换为北京时间（+8小时）
                    UTC = "{}-{:0>2d}-{:0>2d} {:0>2d}:{:0>2d}:{:0>2d}".format(
                        int(year),
                        int(month),
                        int(day),
                        int(cpt_info[1][:2]) + 8,  # UTC时间+8小时转北京时间
                        int(cpt_info[1][2:4]),
                        int(cpt_info[1][4:6]),
                    )
                except (ValueError, IndexError):
                    UTC = ""
                    continue
        
        # 然后从#INSPVAXA行提取位置信息
        if line.startswith("#INSPVAXA") and UTC != "":
            cpt_info = re.split(r" |,|;", line.strip())
            try:
                # 检查数据质量：只保留RTK固定解且坐标有效的数据
                if (len(cpt_info) > 21 and 
                    cpt_info[11] == "INS_RTKFIXED" and 
                    float(cpt_info[12]) > 0 and 
                    float(cpt_info[13]) > 0):
                    
                    lat = float(cpt_info[12])
                    lon = float(cpt_info[13])
                    azi = float(cpt_info[21])
                    ve = float(cpt_info[16])
                    vn = float(cpt_info[17])
                    vd = float(cpt_info[18])
                    vel = math.sqrt(ve * ve + vn * vn + vd * vd)
                    
                    li_cpt.append([UTC, lat, lon, azi, vel])
                    UTC = ""  # 清空UTC，确保每个时间点只匹配一次
            except (ValueError, IndexError):
                UTC = ""  # 出错时清空UTC
                continue
    
    print(f"提取CPT数据: {len(li_cpt)} 条（RTK固定解，整秒时刻）")
    return li_cpt

def parse_time(time_str):
    """解析时间字符串为datetime对象"""
    try:
        return datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S")
    except ValueError:
        return None

def distVincenty(lat1, lon1, lat2, lon2):
    """计算两点之间的距离与北零航向，单位:米/rad - 与参考代码完全一致"""
    import math
    
    a = 6378137.0  # vincentyConstantA(WGS84) ##单位:米
    b = 6356752.3142451  # vincentyConstantB(WGS84) ##单位:米
    f = 1 / 298.257223563  # vincentyConstantF(WGS84)
    L = math.radians(lon2 - lon1)
    U1 = math.atan((1 - f) * math.tan(math.radians(lat1)))
    U2 = math.atan((1 - f) * math.tan(math.radians(lat2)))
    sinU1 = math.sin(U1)
    cosU1 = math.cos(U1)
    sinU2 = math.sin(U2)
    cosU2 = math.cos(U2)
    lambda1 = L
    lambdaP = 2 * math.pi
    iterLimit = 20

    sinLambda = 0.0
    cosLambda = 0.0
    sinSigma = 0.0
    cosSigma = 0.0
    sigma = 0.0
    alpha = 0.0
    cosSqAlpha = 0.0
    cos2SigmaM = 0.0
    C = 0.0
    
    while abs(lambda1 - lambdaP) > 1e-12 and --iterLimit > 0:
        sinLambda = math.sin(lambda1)
        cosLambda = math.cos(lambda1)
        sinSigma = math.sqrt(
            (cosU2 * sinLambda) * (cosU2 * sinLambda)
            + (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
            * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
        )
        if sinSigma == 0:
            return 0.0, 0.0
        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda
        sigma = math.atan2(sinSigma, cosSigma)
        alpha = math.asin(cosU1 * cosU2 * sinLambda / sinSigma)
        cosSqAlpha = math.cos(alpha) * math.cos(alpha)
        cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha
        C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha))
        lambdaP = lambda1
        lambda1 = L + (1 - C) * f * math.sin(alpha) * (
            sigma
            + C
            * sinSigma
            * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM))
        )

    if iterLimit == 0:
        return 0.0, 0.0

    uSq = cosSqAlpha * (a * a - b * b) / (b * b)
    A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)))
    B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)))
    deltaSigma = (
        B
        * sinSigma
        * (
            cos2SigmaM
            + B
            / 4
            * (
                cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)
                - B
                / 6
                * cos2SigmaM
                * (-3 + 4 * sinSigma * sinSigma)
                * (-3 + 4 * cos2SigmaM * cos2SigmaM)
            )
        )
    )
    s = b * A * (sigma - deltaSigma)
    d = s  # 距离（米）
    azi = math.atan2(cosU2 * sinLambda, cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
    return d, azi  # 返回距离和方位角

def get_first_gps_origin_from_data(gps_data):
    """从已提取的GPS数据中获取第一个点作为原点"""
    if not gps_data:
        print("警告: 没有GPS数据用于获取原点")
        return None, None, None, None
    
    try:
        # 使用第一个GPS点作为原点
        first_gps = gps_data[0]
        lat = first_gps[1]  # 纬度
        lon = first_gps[2]  # 经度
        
        # 转换为UTM坐标作为原点
        utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(lat, lon)
        print(f"获取GPS原点: 纬度={lat:.8f}, 经度={lon:.8f}")
        print(f"UTM原点: X={utm_x:.3f}, Y={utm_y:.3f}, 区域={zone_number}{zone_letter}")
        return utm_x, utm_y, zone_number, zone_letter
        
    except Exception as e:
        print(f"获取GPS原点失败: {e}")
        return None, None, None, None

def get_first_gps_origin(vdr_file):
    """获取第一个有效GPS点作为原点 - 兼容性函数"""
    # 直接调用getVdrData获取GPS数据，然后取第一个点
    gps_data = getVdrData(vdr_file)
    return get_first_gps_origin_from_data(gps_data)

def load_eskf_results(eskf_file, time_interpolator=None, gps_data=None):
    """加载ESKF结果文件(gins_offline.txt)，使用时间插值转换和原点校正"""
    eskf_data = []
    
    if not os.path.exists(eskf_file):
        print(f"警告: ESKF文件不存在: {eskf_file}")
        return eskf_data
    
    # 获取GPS原点
    origin_x, origin_y, zone_number, zone_letter = None, None, None, None
    if gps_data:
        origin_x, origin_y, zone_number, zone_letter = get_first_gps_origin_from_data(gps_data)
    
    if origin_x is None:
        print("警告: 无法获取GPS原点，ESKF分析可能失败")
        return eskf_data
    
    try:
        content, encoding = try_read_file(eskf_file)
        print(f"使用编码 {encoding} 读取ESKF文件")
        
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 4:
                try:
                    # ESKF输出格式: timestamp_seconds x_relative y_relative z ...
                    eskf_seconds = float(parts[0])
                    x_relative = float(parts[1])  # 相对于原点的X偏移（米）
                    y_relative = float(parts[2])  # 相对于原点的Y偏移（米）
                    z = float(parts[3])           # 高度
                    
                    # 宽松的时间过滤：选择最接近整秒的数据点
                    # 对于25Hz数据，每秒有25个点，我们选择最接近整秒的那个
                    time_fraction = eskf_seconds - int(eskf_seconds)
                    if time_fraction <= 0.02 or time_fraction >= 0.98:  # 允许±0.02秒的误差
                        pass  # 保留这些数据点
                    else:
                        continue  # 跳过其他时间点
                    
                    # 将相对坐标转换为绝对UTM坐标
                    absolute_utm_x = origin_x + x_relative
                    absolute_utm_y = origin_y + y_relative
                    
                    if time_interpolator is not None:
                        # 使用时间插值转换
                        eskf_tick = int(eskf_seconds * 1000)  # 转换为毫秒tick
                        try:
                            utc_ts_ms = time_interpolator(eskf_tick)
                            
                            # 调试信息：显示前几个转换过程
                            if len(eskf_data) < 3:
                                print(f"调试: eskf_seconds={eskf_seconds}, eskf_tick={eskf_tick}, utc_ts_ms={utc_ts_ms}")
                                print(f"相对坐标: ({x_relative:.3f}, {y_relative:.3f}) -> 绝对坐标: ({absolute_utm_x:.3f}, {absolute_utm_y:.3f})")
                            
                            # 从UTC毫秒时间戳转换为datetime对象
                            beijing_datetime = datetime.fromtimestamp((utc_ts_ms - 8*3600*1000) / 1000)
                            time_str = beijing_datetime.strftime("%Y-%m-%d %H:%M:%S")
                        except Exception as e:
                            print(f"时间插值失败 eskf_tick={eskf_tick}, utc_ts_ms={utc_ts_ms}: {e}")
                            continue
                    else:
                        # 如果没有插值函数，使用简单的时间戳转换
                        time_str = f"tick_{int(eskf_seconds)}"
                    
                    # 存储绝对UTM坐标和区域信息
                    eskf_data.append([time_str, absolute_utm_x, absolute_utm_y, z, zone_number, zone_letter])
                except (ValueError, IndexError):
                    continue
        
        print(f"加载ESKF数据: {len(eskf_data)} 条（仅整秒时刻）")
        if eskf_data and time_interpolator is not None:
            print(f"ESKF时间范围: {eskf_data[0][0]} 到 {eskf_data[-1][0]}")
        return eskf_data
        
    except Exception as e:
        print(f"读取ESKF文件出错: {e}")
        return eskf_data

def align_and_calculate_rms(gps_data, cpt_data, time_tolerance=0.02):
    """对齐GPS和CPT数据并直接计算RMS误差 - 优化版本"""
    print(f"GPS数据: {len(gps_data)} 条")
    print(f"CPT数据: {len(cpt_data)} 条")
    
    # 解析时间并排序
    gps_parsed = []
    for gps in gps_data:
        time_obj = parse_time(gps[0])
        if time_obj:
            gps_parsed.append((time_obj, gps[1], gps[2]))
    
    cpt_parsed = []
    for cpt in cpt_data:
        time_obj = parse_time(cpt[0])
        if time_obj:
            cpt_parsed.append((time_obj, cpt[1], cpt[2]))
    
    gps_parsed.sort(key=lambda x: x[0])
    cpt_parsed.sort(key=lambda x: x[0])
    
    print(f"有效数据 - GPS: {len(gps_parsed)} 条，CPT: {len(cpt_parsed)} 条")
    
    # 显示时间范围
    if gps_parsed:
        print(f"GPS时间范围: {gps_parsed[0][0]} 到 {gps_parsed[-1][0]}")
    if cpt_parsed:
        print(f"CPT时间范围: {cpt_parsed[0][0]} 到 {cpt_parsed[-1][0]}")
    
    # 创建CPT时间索引字典，提高查找效率
    cpt_dict = {}
    for cpt_time, cpt_lat, cpt_lon in cpt_parsed:
        time_key = cpt_time.strftime("%Y-%m-%d %H:%M:%S")
        cpt_dict[time_key] = (cpt_lat, cpt_lon)
    
    # 计算位置误差 - 针对每个GPS时间点在CPT中查找
    errors_squared = []
    matched_data = []  # 保存匹配的详细数据
    matched_count = 0
    
    for gps_time, gps_lat, gps_lon in gps_parsed:
        best_match = None
        min_time_diff = float('inf')
        matched_cpt_time = None
        
        # 在CPT数据的时间范围内搜索匹配点
        for cpt_time, cpt_lat, cpt_lon in cpt_parsed:
            time_diff = abs((gps_time - cpt_time).total_seconds())
            if time_diff <= time_tolerance and time_diff < min_time_diff:
                min_time_diff = time_diff
                best_match = (cpt_lat, cpt_lon, time_diff)
                matched_cpt_time = cpt_time
        
        if best_match:
            cpt_lat, cpt_lon, time_diff = best_match
            distance_error, _ = distVincenty(gps_lat, gps_lon, cpt_lat, cpt_lon)
            errors_squared.append(distance_error ** 2)
            matched_count += 1
            
            # 保存匹配数据：时间戳, GPS经度, GPS纬度, CPT经度, CPT纬度
            matched_data.append([
                gps_time.strftime("%Y-%m-%d %H:%M:%S"),
                gps_lat, gps_lon, cpt_lat, cpt_lon
            ])
            
            # 显示前几个匹配的详细信息
            if matched_count <= 3:
                print(f"匹配 {matched_count}: GPS({gps_time.strftime('%Y-%m-%d %H:%M:%S')}) vs CPT({matched_cpt_time.strftime('%Y-%m-%d %H:%M:%S')}), 时差:{time_diff:.1f}s, 误差:{distance_error:.2f}m")
    
    if not errors_squared:
        print("错误: 没有找到匹配的数据点")
        print("可能原因: GPS和CPT的时间范围不重叠")
        return None, 0, []
    
    rms_error = np.sqrt(np.mean(errors_squared))
    print(f"成功匹配 {matched_count} 对数据点")
    
    return rms_error, matched_count, matched_data

def align_and_calculate_eskf_rms(eskf_data, cpt_data, time_tolerance=0.02):
    """对齐ESKF和CPT数据并计算RMS误差，使用精确UTM转换"""
    print(f"ESKF数据: {len(eskf_data)} 条")
    print(f"CPT数据: {len(cpt_data)} 条")
    
    if not eskf_data:
        print("错误: 没有ESKF数据")
        return None, 0, []
    
    # 解析时间并排序
    eskf_parsed = []
    for eskf in eskf_data:
        time_obj = parse_time(eskf[0])
        if time_obj and len(eskf) >= 6:
            # eskf格式: [time_str, absolute_utm_x, absolute_utm_y, z, zone_number, zone_letter]
            eskf_parsed.append((time_obj, eskf[1], eskf[2], eskf[4], eskf[5]))  # time, x_utm, y_utm, zone_number, zone_letter
    
    cpt_parsed = []
    for cpt in cpt_data:
        time_obj = parse_time(cpt[0])
        if time_obj:
            # CPT数据格式: [UTC, lat, lon, azi, vel]
            cpt_parsed.append((time_obj, cpt[1], cpt[2]))
    
    eskf_parsed.sort(key=lambda x: x[0])
    cpt_parsed.sort(key=lambda x: x[0])
    
    print(f"有效数据 - ESKF: {len(eskf_parsed)} 条，CPT: {len(cpt_parsed)} 条")
    
    # 显示时间范围
    if eskf_parsed:
        print(f"ESKF时间范围: {eskf_parsed[0][0]} 到 {eskf_parsed[-1][0]}")
    if cpt_parsed:
        print(f"CPT时间范围: {cpt_parsed[0][0]} 到 {cpt_parsed[-1][0]}")
    
    # 计算位置误差
    errors_squared = []
    matched_data = []
    matched_count = 0
    
    # 使用ESKF数据中的UTM区域信息
    if eskf_parsed:
        zone_number, zone_letter = eskf_parsed[0][3], eskf_parsed[0][4]
        print(f"使用UTM区域: {zone_number}{zone_letter}")
    else:
        print("错误: 没有ESKF数据用于确定UTM区域")
        return None, 0, []
    
    for eskf_time, x_utm, y_utm, _, _ in eskf_parsed:
        best_match = None
        min_time_diff = float('inf')
        
        # 在CPT数据中查找最佳匹配
        for cpt_time, cpt_lat, cpt_lon in cpt_parsed:
            time_diff = abs((eskf_time - cpt_time).total_seconds())
            if time_diff <= time_tolerance and time_diff < min_time_diff:
                min_time_diff = time_diff
                best_match = (cpt_lat, cpt_lon, time_diff, cpt_time)
        
        if best_match:
            cpt_lat, cpt_lon, time_diff, matched_cpt_time = best_match
            
            try:
                # 使用精确UTM转换将绝对UTM坐标转换为经纬度
                eskf_lat, eskf_lon = utm.to_latlon(x_utm, y_utm, zone_number, zone_letter)
                
                # 计算距离误差
                distance_error, _ = distVincenty(eskf_lat, eskf_lon, cpt_lat, cpt_lon)
                errors_squared.append(distance_error ** 2)
                matched_count += 1
                
                # 保存匹配数据
                matched_data.append([
                    eskf_time.strftime("%Y-%m-%d %H:%M:%S"),
                    eskf_lat, eskf_lon, cpt_lat, cpt_lon
                ])
                
                # 显示前几个匹配的详细信息
                if matched_count <= 3:
                    print(f"匹配 {matched_count}: ESKF({eskf_time.strftime('%Y-%m-%d %H:%M:%S')}) vs CPT({matched_cpt_time.strftime('%Y-%m-%d %H:%M:%S')}), 时差:{time_diff:.1f}s, 误差:{distance_error:.2f}m")
                    
            except Exception as e:
                print(f"UTM转换错误 - UTM坐标: ({x_utm:.3f}, {y_utm:.3f}), 区域: {zone_number}{zone_letter}, 错误: {e}")
                continue
    
    if not errors_squared:
        print("错误: 没有找到匹配的ESKF数据点")
        return None, 0, []
    
    rms_error = np.sqrt(np.mean(errors_squared))
    print(f"成功匹配 {matched_count} 对ESKF数据点")
    
    return rms_error, matched_count, matched_data

def main():
    parser = argparse.ArgumentParser(description='GPS和ESKF位置RMS误差分析')
    
    # 设置默认路径
    default_vdr = '/Users/cjj/Data/vdr_plog/Android_with_truth/Android_0717/vdr_20250717_164201_346.log'
    default_cpt = '/Users/cjj/Data/CPT/0717-1.txt'
    default_output = '/Users/cjj/Data/log_results/Android_with_truth/Android_0717/vdr_20250717_164201_346/'
    default_eskf = '/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/bin/gins_offline.txt'
    
    parser.add_argument('--vdr', default=default_vdr, help=f'VDR日志文件路径 (默认: {default_vdr})')
    parser.add_argument('--cpt', default=default_cpt, help=f'CPT真值文件路径 (默认: {default_cpt})')
    parser.add_argument('--eskf', default=default_eskf, help=f'ESKF结果文件路径 (默认: {default_eskf})')
    parser.add_argument('--output', default=default_output, help=f'输出目录 (默认: {default_output})')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.vdr):
        print(f"错误: VDR文件不存在: {args.vdr}")
        return
    
    if not os.path.exists(args.cpt):
        print(f"错误: CPT文件不存在: {args.cpt}")
        return
    
    print("开始GPS和ESKF位置RMS误差分析...")
    print(f"VDR文件: {args.vdr}")
    print(f"CPT文件: {args.cpt}")
    print(f"ESKF文件: {args.eskf}")
    
    # 提取数据
    print("\n提取GPS数据...")
    gps_data = getVdrData(args.vdr)
    
    print("提取CPT真值数据...")
    cpt_data = getCptData(args.cpt)
    
    print("提取ESKF结果数据...")
    # 建立时间映射
    df_time = get_vdr_time_mapping(args.vdr)
    time_interpolator = create_time_interpolator(df_time)
    
    # 加载ESKF数据（使用时间插值）
    eskf_data = load_eskf_results(args.eskf, time_interpolator, gps_data)
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    # 计算GPS RMS误差
    print("\n=== GPS位置精度分析 ===")
    gps_rms_error, gps_matched_count, gps_matched_data = align_and_calculate_rms(gps_data, cpt_data)
    
    # 计算ESKF RMS误差
    print("\n=== ESKF位置精度分析 ===")
    eskf_rms_error, eskf_matched_count, eskf_matched_data = align_and_calculate_eskf_rms(eskf_data, cpt_data)
    
    # 输出结果
    print(f"\n=== 综合分析结果 ===")
    
    if gps_rms_error is not None:
        print(f"GPS位置RMS误差: {gps_rms_error:.3f} 米")
        print(f"GPS匹配数据点数: {gps_matched_count}")
        
        # 保存GPS详细匹配数据
        gps_detail_file = os.path.join(args.output, "gps_cpt_matched_data.txt")
        with open(gps_detail_file, 'w', encoding='utf-8') as f:
            f.write("# 时间戳 GPS纬度 GPS经度 CPT纬度 CPT经度\n")
            for data in gps_matched_data:
                timestamp, gps_lat, gps_lon, cpt_lat, cpt_lon = data
                f.write(f"{timestamp} {gps_lat:.8f} {gps_lon:.8f} {cpt_lat:.8f} {cpt_lon:.8f}\n")
        print(f"GPS详细匹配数据已保存到: {gps_detail_file}")
    else:
        print("GPS分析失败: 无法计算RMS误差")
    
    if eskf_rms_error is not None:
        print(f"ESKF位置RMS误差: {eskf_rms_error:.3f} 米")
        print(f"ESKF匹配数据点数: {eskf_matched_count}")
        
        # 保存ESKF详细匹配数据
        eskf_detail_file = os.path.join(args.output, "eskf_cpt_matched_data.txt")
        with open(eskf_detail_file, 'w', encoding='utf-8') as f:
            f.write("# 时间戳 ESKF纬度 ESKF经度 CPT纬度 CPT经度\n")
            for data in eskf_matched_data:
                timestamp, eskf_lat, eskf_lon, cpt_lat, cpt_lon = data
                f.write(f"{timestamp} {eskf_lat:.8f} {eskf_lon:.8f} {cpt_lat:.8f} {cpt_lon:.8f}\n")
        print(f"ESKF详细匹配数据已保存到: {eskf_detail_file}")
    else:
        print("ESKF分析失败: 无法计算RMS误差")
    
    # 保存综合结果摘要
    summary_file = os.path.join(args.output, "precision_analysis_summary.txt")
    with open(summary_file, 'w', encoding='utf-8') as f:
        f.write("=== GPS和ESKF位置精度分析结果 ===\n\n")
        
        if gps_rms_error is not None:
            f.write(f"GPS位置RMS误差: {gps_rms_error:.3f} 米\n")
            f.write(f"GPS匹配数据点数: {gps_matched_count}\n\n")
        else:
            f.write("GPS分析失败\n\n")
        
        if eskf_rms_error is not None:
            f.write(f"ESKF位置RMS误差: {eskf_rms_error:.3f} 米\n")
            f.write(f"ESKF匹配数据点数: {eskf_matched_count}\n\n")
        else:
            f.write("ESKF分析失败\n\n")
        
        # 如果两个都成功，显示改进情况
        if gps_rms_error is not None and eskf_rms_error is not None:
            improvement = gps_rms_error - eskf_rms_error
            improvement_percent = (improvement / gps_rms_error) * 100
            f.write(f"ESKF相对GPS的改进: {improvement:.3f} 米 ({improvement_percent:.1f}%)\n")
    
    print(f"综合分析结果已保存到: {summary_file}")
    
    # 绘制GPS和CPT轨迹对比图 - 只绘制时间重合部分
    print("\n=== 绘制轨迹对比图 ===")
    if gps_matched_data and len(gps_matched_data) > 0:
        try:
            # 从匹配数据中提取坐标，只绘制重合部分
            gps_lats = [match[1] for match in gps_matched_data]  # GPS纬度
            gps_lons = [match[2] for match in gps_matched_data]  # GPS经度
            cpt_lats = [match[3] for match in gps_matched_data]  # CPT纬度
            cpt_lons = [match[4] for match in gps_matched_data]  # CPT经度
            
            print(f"绘制重合时间段轨迹: {len(gps_matched_data)} 个匹配点")
            
            # 创建图形
            plt.figure(figsize=(12, 8))
            
            # 绘制GPS轨迹（纯散点，无连线）
            plt.scatter(gps_lons, gps_lats, c='blue', s=20, alpha=0.7, label='GPS轨迹', marker='o')
            
            # 绘制CPT真值轨迹（纯散点，无连线）
            plt.scatter(cpt_lons, cpt_lats, c='red', s=20, alpha=0.8, label='CPT真值轨迹', marker='s')
            
            # 不绘制连接线，保持轨迹清晰
            print(f"绘制轨迹对比图，共 {len(gps_matched_data)} 个匹配点")
            
            # 标记起点和终点
            if len(gps_matched_data) > 0:
                # 起点
                plt.plot(gps_lons[0], gps_lats[0], 'go', markersize=8, label='起点')
                plt.plot(cpt_lons[0], cpt_lats[0], 'go', markersize=8)
                
                # 终点
                plt.plot(gps_lons[-1], gps_lats[-1], 'mo', markersize=8, label='终点')
                plt.plot(cpt_lons[-1], cpt_lats[-1], 'mo', markersize=8)
            
            # 设置图形属性
            plt.xlabel('经度 (°)', fontsize=12)
            plt.ylabel('纬度 (°)', fontsize=12)
            plt.title('GPS与CPT真值轨迹对比（重合时间段）', fontsize=14, fontweight='bold')
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            
            # 设置坐标轴格式
            plt.ticklabel_format(useOffset=False, style='plain')
            
            # 调整布局
            plt.tight_layout()
            
            # 保存图形
            plot_file = os.path.join(args.output, "gps_cpt_trajectory_comparison.png")
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"轨迹对比图已保存到: {plot_file}")
            
            # 显示图形
            plt.show()
            
        except Exception as e:
            print(f"绘图过程中出现错误: {e}")
    else:
        print("没有匹配数据进行绘图")
        
    # 如果有ESKF数据，也绘制ESKF轨迹对比图
    if eskf_matched_data and len(eskf_matched_data) > 0:
        try:
            print(f"\n绘制ESKF轨迹对比图: {len(eskf_matched_data)} 个匹配点")
            
            # 从ESKF匹配数据中提取坐标
            eskf_lats = [match[1] for match in eskf_matched_data]  # ESKF纬度
            eskf_lons = [match[2] for match in eskf_matched_data]  # ESKF经度
            cpt_lats_eskf = [match[3] for match in eskf_matched_data]  # CPT纬度
            cpt_lons_eskf = [match[4] for match in eskf_matched_data]  # CPT经度
            
            # 创建ESKF对比图
            plt.figure(figsize=(12, 8))
            
            # 绘制ESKF轨迹（纯散点，无连线）
            plt.scatter(eskf_lons, eskf_lats, c='green', s=20, alpha=0.7, label='ESKF轨迹', marker='o')
            
            # 绘制CPT真值轨迹（纯散点，无连线）
            plt.scatter(cpt_lons_eskf, cpt_lats_eskf, c='red', s=20, alpha=0.8, label='CPT真值轨迹', marker='s')
            
            # 不绘制连接线，保持轨迹清晰
            
            # 标记起点和终点
            if len(eskf_matched_data) > 0:
                plt.plot(eskf_lons[0], eskf_lats[0], 'go', markersize=8, label='起点')
                plt.plot(cpt_lons_eskf[0], cpt_lats_eskf[0], 'go', markersize=8)
                plt.plot(eskf_lons[-1], eskf_lats[-1], 'mo', markersize=8, label='终点')
                plt.plot(cpt_lons_eskf[-1], cpt_lats_eskf[-1], 'mo', markersize=8)
            
            plt.xlabel('经度 (°)', fontsize=12)
            plt.ylabel('纬度 (°)', fontsize=12)
            plt.title('ESKF与CPT真值轨迹对比（重合时间段）', fontsize=14, fontweight='bold')
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            plt.ticklabel_format(useOffset=False, style='plain')
            plt.tight_layout()
            
            # 保存ESKF对比图
            eskf_plot_file = os.path.join(args.output, "eskf_cpt_trajectory_comparison.png")
            plt.savefig(eskf_plot_file, dpi=300, bbox_inches='tight')
            print(f"ESKF轨迹对比图已保存到: {eskf_plot_file}")
            
            plt.show()
            
        except Exception as e:
            print(f"绘制ESKF轨迹图时出现错误: {e}")

if __name__ == "__main__":
    main()
