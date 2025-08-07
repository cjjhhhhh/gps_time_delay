#!/usr/bin/env python3
import os
import sys
import re

def parse_turn_data(file_path, data_type):
    """解析转弯段数据文件"""
    results = {}
    
    if not os.path.exists(file_path):
        print(f"  文件不存在: {file_path}")
        return results
    
    print(f"  解析文件: {file_path}")
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 按转弯段分割
    turn_sections = re.split(r'# 转弯段 (\d+)', content)[1:]  # 跳过第一个空部分
    
    for i in range(0, len(turn_sections), 2):
        if i + 1 >= len(turn_sections):
            break
            
        turn_id = int(turn_sections[i])
        turn_data = turn_sections[i + 1]
        
        print(f"    开始解析转弯段{turn_id}")
        
        # 找到所有数据行（以负号或数字开头的行）
        data_lines = []
        for line in turn_data.strip().split('\n'):
            line = line.strip()
            if line and not line.startswith('#') and (',' in line):
                # 解析CSV格式的数据行
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        delay = float(parts[0])
                        rms = float(parts[1])
                        data_lines.append((delay, rms))
                    except ValueError:
                        continue
        
        if data_lines:
            # 找到最小RMS值及其对应的延迟
            min_rms = min(data_lines, key=lambda x: x[1])
            delay, rms = min_rms
            
            # 计算改善量（相对于0延迟的RMS）
            zero_delay_rms = None
            for d, r in data_lines:
                if abs(d - 0.0) < 0.001:  # 找到0延迟的RMS
                    zero_delay_rms = r
                    break
            
            if zero_delay_rms is not None:
                improvement_m = zero_delay_rms - rms
                improvement_cm = improvement_m * 100  # 转换为厘米
                
                results[turn_id] = {
                    'delay': delay,
                    'rms': rms,
                    'improvement_cm': improvement_cm
                }
                print(f"      转弯段{turn_id}: 最优延迟={delay}s, RMS={rms:.4f}m, 改善={improvement_cm:.1f}cm")
            else:
                print(f"      转弯段{turn_id}: 未找到0延迟基准数据")
        else:
            print(f"      转弯段{turn_id}: 未找到有效数据行")
    
    return results

def generate_summary_table(base_dir):
    """生成转弯段优化汇总表"""
    output_file = os.path.join(base_dir, 'turn_optimization_summary.txt')
    
    # 收集所有日志目录
    log_dirs = []
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        if os.path.isdir(item_path) and item.startswith('vdr_'):
            log_dirs.append(item)
    
    log_dirs.sort()
    
    # 写入汇总表
    with open(output_file, 'w', encoding='utf-8') as f:
        # 写入正确的表头
        f.write("日志名称\t转弯段ID\t位置RMS优化结果\t横向RMS优化结果\n")
        
        for log_name in log_dirs:
            print(f"处理 {log_name}...")
            log_path = os.path.join(base_dir, log_name)
            
            # 解析位置RMS数据
            pos_file = os.path.join(log_path, 'turn_rms_analysis_turns.txt')
            pos_results = parse_turn_data(pos_file, 'position')
            
            # 解析横向RMS数据
            lat_file = os.path.join(log_path, 'turn_lateral_analysis_turns.txt')
            lat_results = parse_turn_data(lat_file, 'lateral')
            
            # 获取所有转弯段ID
            all_turn_ids = set(pos_results.keys()) | set(lat_results.keys())
            
            # 只有当有转弯段数据时才处理
            if all_turn_ids:
                for turn_id in sorted(all_turn_ids):
                    pos_result = pos_results.get(turn_id, {})
                    lat_result = lat_results.get(turn_id, {})
                    
                    # 格式化位置RMS结果
                    if pos_result:
                        pos_str = f"{pos_result['improvement_cm']:.0f}cm({pos_result['delay']:.2f}s)"
                    else:
                        pos_str = "N/A"
                    
                    # 格式化横向RMS结果
                    if lat_result:
                        lat_str = f"{lat_result['improvement_cm']:.0f}cm({lat_result['delay']:.2f}s)"
                    else:
                        lat_str = "N/A"
                    
                    # 写入数据行
                    f.write(f"{log_name}\t{turn_id}\t{pos_str}\t{lat_str}\n")
                
                # 在每个日志的数据后添加空行分隔
                f.write("\n")
    
    print(f"\n转弯段优化汇总表已生成: {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python3 generate_turn_summary_final.py <结果目录>")
        sys.exit(1)
    
    base_dir = sys.argv[1]
    if not os.path.exists(base_dir):
        print(f"错误: 目录不存在 {base_dir}")
        sys.exit(1)
    
    generate_summary_table(base_dir)