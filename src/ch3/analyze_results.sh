#!/bin/bash

# ESKF处理结果分析脚本
# 用于对ESKF批量处理结果进行转弯时段RMS分析
# 用法: ./analyze_results.sh [结果目录路径]

# 默认参数设置
RESULTS_DIR="${1:-/Users/cjj/Data/log_results/XiaoMi11}"

# 全局变量
start_time=0
pos_rms_success=0
pos_rms_skipped=0
lateral_rms_success=0
lateral_rms_skipped=0
plot_success=0
plot_skipped=0

# macOS兼容的颜色输出定义
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    PURPLE='\033[0;35m'
    CYAN='\033[0;36m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    PURPLE=''
    CYAN=''
    NC=''
fi

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_progress() {
    echo -e "${PURPLE}[PROGRESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# 检查环境和依赖
check_prerequisites() {
    log_info "检查分析环境..."
    
    # 检查结果目录
    if [[ ! -d "$RESULTS_DIR" ]]; then
        log_error "结果目录不存在: $RESULTS_DIR"
        exit 1
    fi
    
    # 检查processing_summary.txt
    if [[ ! -f "$RESULTS_DIR/processing_summary.txt" ]]; then
        log_error "处理汇总文件不存在: $RESULTS_DIR/processing_summary.txt"
        exit 1
    fi
    
    # 检查turn_analysis目录
    if [[ ! -d "$RESULTS_DIR/turn_analysis" ]]; then
        log_error "转弯分析目录不存在: $RESULTS_DIR/turn_analysis"
        exit 1
    fi
    
    # 检查Python环境
    if ! command -v python3 &> /dev/null; then
        log_error "Python3未安装，无法进行RMS分析"
        exit 1
    fi
    
    # 检查Python分析脚本
    local script_dir=$(dirname "$0")
    if [[ ! -f "$script_dir/auto_pos_rms.py" ]]; then
        log_error "位置RMS分析脚本不存在: $script_dir/auto_pos_rms.py"
        exit 1
    fi
    
    if [[ ! -f "$script_dir/auto_lateral_residuals_rms.py" ]]; then
        log_error "横向残差分析脚本不存在: $script_dir/auto_lateral_residuals_rms.py"
        exit 1
    fi
    
    if [[ ! -f "$script_dir/plot_rms_curves.py" ]]; then
        log_error "RMS曲线绘图脚本不存在: $script_dir/plot_rms_curves.py"
        exit 1
    fi
    
    # 检查matplotlib依赖（可选）
    if python3 -c "import matplotlib.pyplot, numpy" 2>/dev/null; then
        log_success "matplotlib依赖检查通过"
    else
        log_warning "matplotlib依赖缺失，将跳过绘图功能"
        log_warning "安装方法: pip install matplotlib numpy"
    fi
    
    log_success "环境检查完成"
}

# 构造corrections文件路径
get_corrections_file_path() {
    local log_name="$1"
    local gps_offset="$2"
    local log_dir="$RESULTS_DIR/$log_name"
    
    if [[ "$gps_offset" == "0.00" ]]; then
        echo "$log_dir/corrections.txt"
    else
        # 将偏移值转换为毫秒
        local offset_ms=$(printf "%.0f" $(echo "$gps_offset * 1000" | bc -l 2>/dev/null || echo "0"))
        if [[ $offset_ms -eq 0 ]]; then
            echo "$log_dir/corrections.txt"
        else
            echo "$log_dir/corrections_${offset_ms}ms.txt"
        fi
    fi
}

# 构造lateral文件路径
get_lateral_file_path() {
    local log_name="$1"
    local gps_offset="$2"
    local log_dir="$RESULTS_DIR/$log_name"
    
    if [[ "$gps_offset" == "0.00" ]]; then
        echo "$log_dir/corrections_lateral.txt"
    else
        # 将偏移值转换为毫秒
        local offset_ms=$(printf "%.0f" $(echo "$gps_offset * 1000" | bc -l 2>/dev/null || echo "0"))
        if [[ $offset_ms -eq 0 ]]; then
            echo "$log_dir/corrections_lateral.txt"
        else
            echo "$log_dir/corrections_${offset_ms}ms_lateral.txt"
        fi
    fi
}

# 执行单个文件的绘图
perform_plot_generation() {
    local log_name="$1"
    local log_dir="$2"
    
    local script_dir=$(dirname "$0")
    
    # 检查是否至少有一个分析文件（更新文件名检查）
    local has_analysis=false
    
    if [[ -f "$log_dir/turn_rms_analysis.txt" ]] || [[ -f "$log_dir/turn_lateral_analysis.txt" ]] || \
       [[ -f "$log_dir/turn_rms_analysis_full.txt" ]] || [[ -f "$log_dir/turn_lateral_analysis_full.txt" ]] || \
       [[ -f "$log_dir/turn_rms_analysis_turns.txt" ]] || [[ -f "$log_dir/turn_lateral_analysis_turns.txt" ]]; then
        has_analysis=true
    fi
    
    if [[ "$has_analysis" == false ]]; then
        echo "跳过绘图: $log_name - 分析文件不存在"
        plot_skipped=$((plot_skipped + 1))
        return 1
    fi
    
    # 显示执行信息
    echo "正在为 $log_name 生成RMS曲线图..."
    echo "执行命令: python3 $script_dir/plot_rms_curves.py --vdr_dir '$log_dir' --log_name '$log_name'"
    
    # 执行绘图
    if python3 "$script_dir/plot_rms_curves.py" \
        --vdr_dir "$log_dir" \
        --log_name "$log_name"; then
        plot_success=$((plot_success + 1))
        return 0
    else
        echo "绘图失败，退出码: $?"
        plot_skipped=$((plot_skipped + 1))
        return 1
    fi
}

# 执行单个文件的RMS分析 - 修正版
perform_file_analysis() {
    local log_name="$1"
    local offsets="$2"
    
    local script_dir=$(dirname "$0")
    local log_dir="$RESULTS_DIR/$log_name"
    local turns_file="$RESULTS_DIR/turn_analysis/${log_name}_turns_nzz.txt"
    
    # 检查日志目录是否存在
    if [[ ! -d "$log_dir" ]]; then
        log_warning "跳过分析: 日志目录不存在 - $log_dir"
        pos_rms_skipped=$((pos_rms_skipped + 2))  # 转弯段+整段
        lateral_rms_skipped=$((lateral_rms_skipped + 2))
        plot_skipped=$((plot_skipped + 1))
        return 1
    fi
    
    local pos_success=false
    local lateral_success=false
    
    # ========================================
    # 新增：整段数据RMS分析
    # ========================================
    log_info "分析整段数据: $log_name"
    
    # 执行整段位置RMS分析（不使用--turns参数）
    if python3 "$script_dir/auto_pos_rms.py" \
        --log_dir "$log_dir" \
        --log_name "$log_name" \
        --offsets="$offsets" \
        --output_suffix="_full" ; then
        pos_rms_success=$((pos_rms_success + 1))
        pos_success=true
        log_success "整段位置RMS分析完成"
    else
        pos_rms_skipped=$((pos_rms_skipped + 1))
        log_warning "整段位置RMS分析失败"
    fi
    
    # 执行整段横向残差RMS分析
    if python3 "$script_dir/auto_lateral_residuals_rms.py" \
        --log_dir "$log_dir" \
        --log_name "$log_name" \
        --offsets="$offsets" \
        --output_suffix="_full" ; then
        lateral_rms_success=$((lateral_rms_success + 1))
        lateral_success=true
        log_success "整段横向残差RMS分析完成"
    else
        lateral_rms_skipped=$((lateral_rms_skipped + 1))
        log_warning "整段横向残差RMS分析失败"
    fi
    
    # ========================================
    # 原有：转弯段数据RMS分析
    # ========================================
    
    # 检查转弯文件是否存在
    if [[ -f "$turns_file" ]]; then
        log_info "分析转弯段数据: $log_name"
        
        # 执行转弯段位置RMS分析
        if python3 "$script_dir/auto_pos_rms.py" \
            --log_dir "$log_dir" \
            --turns "$turns_file" \
            --log_name "$log_name" \
            --offsets="$offsets" \
            --output_suffix="_turns" ; then
            pos_rms_success=$((pos_rms_success + 1))
            pos_success=true
            log_success "转弯段位置RMS分析完成"
        else
            pos_rms_skipped=$((pos_rms_skipped + 1))
            log_warning "转弯段位置RMS分析失败"
        fi
        
        # 执行转弯段横向残差RMS分析
        if python3 "$script_dir/auto_lateral_residuals_rms.py" \
            --log_dir "$log_dir" \
            --turns "$turns_file" \
            --log_name "$log_name" \
            --offsets="$offsets" \
            --output_suffix="_turns" ; then
            lateral_rms_success=$((lateral_rms_success + 1))
            lateral_success=true
            log_success "转弯段横向残差RMS分析完成"
        else
            lateral_rms_skipped=$((lateral_rms_skipped + 1))
            log_warning "转弯段横向残差RMS分析失败"
        fi
    else
        log_warning "跳过转弯段分析: 转弯文件不存在 - $(basename "$turns_file")"
        pos_rms_skipped=$((pos_rms_skipped + 1))
        lateral_rms_skipped=$((lateral_rms_skipped + 1))
    fi
    
    # 如果有分析成功，执行绘图
    if [[ "$pos_success" == true ]] || [[ "$lateral_success" == true ]]; then
        perform_plot_generation "$log_name" "$log_dir"
        return 0
    else
        plot_skipped=$((plot_skipped + 1))
        return 1
    fi
}

parse_processing_summary() {
    local summary_file="$RESULTS_DIR/processing_summary.txt"
    
    log_info "解析处理汇总文件..." >&2
    
    # 临时文件用于收集数据
    local temp_file="/tmp/analyze_results_$$"
    
    # 读取SUCCESS状态的记录，按文件名分组
    grep "SUCCESS" "$summary_file" 2>/dev/null | while IFS=',' read -r timestamp log_name gps_offset status output_dir duration file_size; do
        echo "$log_name,$gps_offset"
    done | sort | awk -F',' '
    {
        if (file != $1) {
            if (file != "") {
                print file ":" offsets
            }
            file = $1
            offsets = $2
        } else {
            offsets = offsets "," $2
        }
    }
    END {
        if (file != "") {
            print file ":" offsets
        }
    }' > "$temp_file"
    
    # 统计信息
    local total_files=$(wc -l < "$temp_file" 2>/dev/null || echo "0")
    local total_success=$(grep "SUCCESS" "$summary_file" 2>/dev/null | wc -l | xargs)
    
    log_info "找到 $total_success 个成功处理的记录，涉及 $total_files 个日志文件" >&2
    
    # 输出结果并清理临时文件
    cat "$temp_file"
    rm -f "$temp_file"
}

# 批量执行RMS分析
run_batch_analysis() {
    log_info "开始按文件批量RMS分析..."
    
    # 获取文件和对应的偏移值
    local file_data=($(parse_processing_summary))
    local total_files=${#file_data[@]}
    
    if [[ $total_files -eq 0 ]]; then
        log_warning "未找到成功处理的记录，跳过分析"
        return
    fi
    
    log_info "开始分析 $total_files 个日志文件"
    
    local current_file=0
    
    # 遍历每个文件
    for file_entry in "${file_data[@]}"; do
        current_file=$((current_file + 1))
        
        # 解析文件名和偏移值列表
        local log_name=$(echo "$file_entry" | cut -d':' -f1)
        local offsets=$(echo "$file_entry" | cut -d':' -f2)
        
        # 计算进度
        local progress=$((current_file * 100 / total_files))
        
        printf "\n${PURPLE}[进度: %d/%d (%d%%)]${NC} 分析文件: %s\n" \
            $current_file $total_files $progress "$log_name"
        printf "  偏移值: %s\n" "$offsets"
        
        # 执行该文件的RMS分析
        perform_file_analysis "$log_name" "$offsets"
        
        # 显示当前统计
        printf "当前进度: 位置RMS[成功:%d 跳过:%d] 横向残差[成功:%d 跳过:%d] 绘图[成功:%d 跳过:%d]\n" \
            $pos_rms_success $pos_rms_skipped $lateral_rms_success $lateral_rms_skipped $plot_success $plot_skipped
    done
    
    log_success "按文件批量RMS分析完成"
}

# 生成最优延迟汇总表 - 修复版本
generate_optimal_delays_summary() {
    local optimal_file="$RESULTS_DIR/optimal_delays_summary.txt"
    
    log_info "生成最优延迟汇总表..."
    
    # 创建CSV表头
    echo "日志文件,类型,转弯段,位置最优延迟(s),横向最优延迟(s)" > "$optimal_file"
    
    local total_entries=0
    
    # 遍历所有vdr_*目录
    for log_dir in "$RESULTS_DIR"/vdr_*/; do
        if [[ -d "$log_dir" ]]; then
            local log_name=$(basename "$log_dir")
            
            echo "DEBUG: 处理目录 $log_name"
            
            # 定义不同类型的分析文件
            local analysis_types=(
                "full:turn_rms_analysis_full.txt:turn_lateral_analysis_full.txt:整段轨迹"
                "turns:turn_rms_analysis_turns.txt:turn_lateral_analysis_turns.txt:转弯段"
                "default:turn_rms_analysis.txt:turn_lateral_analysis.txt:默认"
            )
            
            for type_info in "${analysis_types[@]}"; do
                IFS=':' read -ra info_parts <<< "$type_info"
                local type_name="${info_parts[0]}"
                local pos_filename="${info_parts[1]}"
                local lateral_filename="${info_parts[2]}"
                local type_desc="${info_parts[3]}"
                
                local pos_file="$log_dir/$pos_filename"
                local lateral_file="$log_dir/$lateral_filename"
                
                echo "DEBUG: 检查 $type_name 类型文件"
                echo "DEBUG: 位置文件 $pos_file 存在: $(test -f "$pos_file" && echo "是" || echo "否")"
                echo "DEBUG: 横向文件 $lateral_file 存在: $(test -f "$lateral_file" && echo "是" || echo "否")"
                
                # 检查是否有对应的分析文件
                if [[ ! -f "$pos_file" ]] && [[ ! -f "$lateral_file" ]]; then
                    continue
                fi
                
                # 使用临时文件存储解析结果
                local temp_pos_file="/tmp/pos_delays_${type_name}_$$_$(date +%s)"
                local temp_lateral_file="/tmp/lateral_delays_${type_name}_$$_$(date +%s)"
                
                # 解析位置RMS分析文件
                if [[ -f "$pos_file" ]]; then
                    echo "DEBUG: 开始解析位置RMS文件 ($type_name)"
                    parse_optimal_delays_to_file "$pos_file" "$temp_pos_file"
                else
                    touch "$temp_pos_file"
                fi
                
                # 解析横向残差分析文件
                if [[ -f "$lateral_file" ]]; then
                    echo "DEBUG: 开始解析横向残差文件 ($type_name)"
                    parse_optimal_delays_to_file "$lateral_file" "$temp_lateral_file"
                else
                    touch "$temp_lateral_file"
                fi
                
                # 合并结果并写入汇总文件
                local all_turn_ids=($(cat "$temp_pos_file" "$temp_lateral_file" 2>/dev/null | cut -d',' -f1 | sort -nu))
                echo "DEBUG: $type_name 类型所有转弯段ID: ${all_turn_ids[*]}"
                
                # 为每个转弯段生成汇总行
                for turn_id in "${all_turn_ids[@]}"; do
                    if [[ -n "$turn_id" ]]; then
                        local pos_delay=$(grep "^$turn_id," "$temp_pos_file" 2>/dev/null | cut -d',' -f2)
                        local lateral_delay=$(grep "^$turn_id," "$temp_lateral_file" 2>/dev/null | cut -d',' -f2)
                        
                        pos_delay="${pos_delay:-N/A}"
                        lateral_delay="${lateral_delay:-N/A}"
                        
                        local segment_desc
                        if [[ "$turn_id" == "0" ]]; then
                            segment_desc="整段轨迹"
                        else
                            segment_desc="转弯段$turn_id"
                        fi
                        
                        echo "DEBUG: 写入 $type_desc $segment_desc: pos=$pos_delay, lateral=$lateral_delay"
                        echo "$log_name,$type_desc,$segment_desc,$pos_delay,$lateral_delay" >> "$optimal_file"
                        total_entries=$((total_entries + 1))
                    fi
                done
                
                # 清理临时文件
                rm -f "$temp_pos_file" "$temp_lateral_file"
                
                # 如果有数据，添加空行分隔
                if [[ ${#all_turn_ids[@]} -gt 0 ]]; then
                    echo "" >> "$optimal_file"
                fi
            done
        fi
    done
    
    log_success "最优延迟汇总表已生成: $optimal_file ($total_entries 条记录)"
}

# 新的解析函数 - 将结果写入文件而不是关联数组
parse_optimal_delays_to_file() {
    local analysis_file="$1"
    local output_file="$2"
    
    echo "DEBUG: parse_optimal_delays_to_file - 解析文件 $analysis_file"
    echo "DEBUG: parse_optimal_delays_to_file - 输出到 $output_file"
    
    if [[ ! -f "$analysis_file" ]]; then
        echo "DEBUG: parse_optimal_delays_to_file - 文件不存在"
        return
    fi
    
    # 清空输出文件
    > "$output_file"
    
    local current_turn_id=""
    local min_rms=999999
    local optimal_delay=""
    local line_count=0
    
    while IFS= read -r line; do
        line_count=$((line_count + 1))
        line=$(echo "$line" | xargs)  # 去除前后空格
        
        # 解析转弯段标题
        if [[ "$line" =~ ^#\ 转弯段\ ([0-9]+)\ \((.+)\)$ ]]; then
            echo "DEBUG: parse_optimal_delays_to_file - 第${line_count}行匹配转弯段标题: $line"
            
            # 保存上一个转弯段的结果
            if [[ -n "$current_turn_id" && -n "$optimal_delay" ]]; then
                echo "DEBUG: parse_optimal_delays_to_file - 保存转弯段 $current_turn_id，最优延迟: $optimal_delay"
                echo "$current_turn_id,$optimal_delay" >> "$output_file"
            fi
            
            # 开始新的转弯段
            current_turn_id="${BASH_REMATCH[1]}"
            min_rms=999999
            optimal_delay=""
            
            echo "DEBUG: parse_optimal_delays_to_file - 开始新转弯段 ID=$current_turn_id"
            
        # 解析数据行
        elif [[ -n "$current_turn_id" && ! "$line" =~ ^# && -n "$line" ]]; then
            local fields
            IFS=',' read -ra fields <<< "$line"
            
            if [[ ${#fields[@]} -ge 2 ]]; then
                local gps_offset="${fields[0]}"
                local rms_value="${fields[1]}"
                
                # 去除空格
                gps_offset=$(echo "$gps_offset" | xargs)
                rms_value=$(echo "$rms_value" | xargs)
                
                echo "DEBUG: parse_optimal_delays_to_file - 第${line_count}行数据: offset=$gps_offset, rms=$rms_value"
                
                # 验证数值格式
                if [[ "$gps_offset" =~ ^-?[0-9]+\.?[0-9]*$ && "$rms_value" =~ ^[0-9]+\.?[0-9]*$ ]]; then
                    # 使用awk进行浮点数比较
                    local is_better=$(awk -v rms="$rms_value" -v min="$min_rms" 'BEGIN {print (rms < min) ? 1 : 0}')
                    
                    if [[ "$is_better" == "1" ]]; then
                        echo "DEBUG: parse_optimal_delays_to_file - 发现更优值: $rms_value < $min_rms"
                        min_rms="$rms_value"
                        optimal_delay="$gps_offset"
                    fi
                else
                    echo "DEBUG: parse_optimal_delays_to_file - 数据格式无效: offset='$gps_offset', rms='$rms_value'"
                fi
            else
                echo "DEBUG: parse_optimal_delays_to_file - 字段数不够: ${#fields[@]} < 2"
            fi
        fi
    done < "$analysis_file"
    
    # 保存最后一个转弯段的结果
    if [[ -n "$current_turn_id" && -n "$optimal_delay" ]]; then
        echo "DEBUG: parse_optimal_delays_to_file - 保存最后转弯段 $current_turn_id，最优延迟: $optimal_delay"
        echo "$current_turn_id,$optimal_delay" >> "$output_file"
    fi
    
    echo "DEBUG: parse_optimal_delays_to_file - 解析完成，总行数: $line_count"
}

# 生成分析汇总报告
generate_analysis_summary() {
    local summary_file="$RESULTS_DIR/analysis_summary.txt"
    
    log_info "生成分析汇总报告..."
    
    # 创建汇总报告
    cat > "$summary_file" << EOF
# ESKF结果RMS分析汇总报告
# 生成时间: $(date)
# 分析目录: $RESULTS_DIR

=== 分析统计 ===
按文件分析统计:
  位置RMS分析: 成功 $pos_rms_success 个文件, 跳过 $pos_rms_skipped 个文件
  横向残差分析: 成功 $lateral_rms_success 个文件, 跳过 $lateral_rms_skipped 个文件
  RMS曲线绘图: 成功 $plot_success 个文件, 跳过 $plot_skipped 个文件

=== 输出文件分布 ===
EOF

    # 扫描各个文件夹的分析结果
    local pos_files=0
    local lateral_files=0
    local plot_files=0
    local total_pos_segments=0
    local total_lateral_segments=0
    local total_plots=0
    
    echo "各文件夹分析结果:" >> "$summary_file"
    
    # 在generate_analysis_summary函数中，修改文件检查逻辑
    for log_dir in "$RESULTS_DIR"/vdr_*/; do
        if [[ -d "$log_dir" ]]; then
            local log_name=$(basename "$log_dir")
            local has_any=false
            
            # 检查所有可能的位置RMS分析文件
            for pos_file in "$log_dir/turn_rms_analysis.txt" "$log_dir/turn_rms_analysis_full.txt" "$log_dir/turn_rms_analysis_turns.txt"; do
                if [[ -f "$pos_file" ]]; then
                    local filename=$(basename "$pos_file")
                    local pos_segments=$(grep -v "^#" "$pos_file" 2>/dev/null | grep -v "^$" | wc -l | xargs)
                    echo "  $log_name/$filename ($pos_segments 行数据)" >> "$summary_file"
                    pos_files=$((pos_files + 1))
                    total_pos_segments=$((total_pos_segments + pos_segments))
                    has_any=true
                fi
            done
            
            # 检查所有可能的横向残差分析文件
            for lateral_file in "$log_dir/turn_lateral_analysis.txt" "$log_dir/turn_lateral_analysis_full.txt" "$log_dir/turn_lateral_analysis_turns.txt"; do
                if [[ -f "$lateral_file" ]]; then
                    local filename=$(basename "$lateral_file")
                    local lateral_segments=$(grep -v "^#" "$lateral_file" 2>/dev/null | grep -v "^$" | wc -l | xargs)
                    echo "  $log_name/$filename ($lateral_segments 行数据)" >> "$summary_file"
                    lateral_files=$((lateral_files + 1))
                    total_lateral_segments=$((total_lateral_segments + lateral_segments))
                    has_any=true
                fi
            done
            
            # 检查绘图文件（保持不变）
            if [[ -d "$log_dir/plots" ]]; then
                local plot_count=$(find "$log_dir/plots" -name "*.png" 2>/dev/null | wc -l | xargs)
                if [[ $plot_count -gt 0 ]]; then
                    echo "  $log_name/plots/ ($plot_count 个RMS曲线图)" >> "$summary_file"
                    plot_files=$((plot_files + 1))
                    total_plots=$((total_plots + plot_count))
                    has_any=true
                fi
            fi
            
            # 如果都没有，标记为无分析结果
            if [[ "$has_any" == false ]]; then
                echo "  $log_name/: 无分析结果" >> "$summary_file"
            fi
        fi
    done
    
    cat >> "$summary_file" << EOF

=== 汇总统计 ===
位置RMS分析文件: $pos_files 个 (总数据行: $total_pos_segments)
横向残差分析文件: $lateral_files 个 (总数据行: $total_lateral_segments)
RMS曲线图文件: $plot_files 个文件夹 (总图表数: $total_plots)

=== 使用说明 ===
1. 每个vdr_xxx/目录内包含该文件的独立分析结果
2. turn_rms_analysis.txt - 位置RMS分析，按转弯段分组
3. turn_lateral_analysis.txt - 横向残差分析，按转弯段分组
4. plots/ - RMS曲线图，每个转弯段一个PNG文件
5. 不同转弯段之间用空行分隔，便于阅读
6. 每个转弯段显示所有GPS偏移值的分析结果

=== 查看建议 ===
查看特定文件的分析结果:
  cat $RESULTS_DIR/vdr_xxx/turn_rms_analysis.txt
  cat $RESULTS_DIR/vdr_xxx/turn_lateral_analysis.txt

查看RMS曲线图:
  open $RESULTS_DIR/vdr_xxx/plots/

批量查看所有位置RMS结果:
  find $RESULTS_DIR -name "turn_rms_analysis.txt" -exec echo "=== {} ===" \; -exec cat {} \;

批量打开所有图表目录:
  open $RESULTS_DIR/vdr_*/plots/
EOF
    
    log_success "分析汇总报告已生成: $summary_file"
}

# 显示最终统计
show_final_summary() {
    local end_time=$(date +%s)
    local total_time=$((end_time - start_time))
    local hours=$((total_time / 3600))
    local minutes=$(((total_time % 3600) / 60))
    local seconds=$((total_time % 60))
    
    echo
    echo "================================"
    log_success "ESKF结果分析完成！"
    echo
    
    echo "分析统计:"
    echo "  位置RMS分析: 成功 $pos_rms_success 个文件, 跳过 $pos_rms_skipped 个文件"
    echo "  横向残差分析: 成功 $lateral_rms_success 个文件, 跳过 $lateral_rms_skipped 个文件"
    echo "  RMS曲线绘图: 成功 $plot_success 个文件, 跳过 $plot_skipped 个文件"
    echo "  总耗时: ${hours}h ${minutes}m ${seconds}s"
    echo
    
    echo "输出文件分布:"
    
    # 统计各文件夹的分析结果
    local pos_files=0
    local lateral_files=0
    local plot_dirs=0
    
    for log_dir in "$RESULTS_DIR"/vdr_*/; do
        if [[ -d "$log_dir" ]]; then
            local log_name=$(basename "$log_dir")
            local status_line="  $log_name/: "
            
            if [[ -f "$log_dir/turn_rms_analysis.txt" ]]; then
                status_line="${status_line}位置RMS✓ "
                pos_files=$((pos_files + 1))
            fi
            
            if [[ -f "$log_dir/turn_lateral_analysis.txt" ]]; then
                status_line="${status_line}横向残差✓ "
                lateral_files=$((lateral_files + 1))
            fi
            
            if [[ -d "$log_dir/plots" ]] && [[ $(ls "$log_dir/plots"/*.png 2>/dev/null | wc -l) -gt 0 ]]; then
                local plot_count=$(ls "$log_dir/plots"/*.png 2>/dev/null | wc -l | xargs)
                status_line="${status_line}图表✓($plot_count)"
                plot_dirs=$((plot_dirs + 1))
            fi
            
            if [[ "$status_line" == "  $log_name/: " ]]; then
                status_line="${status_line}无分析结果"
            fi
            
            echo "$status_line"
        fi
    done
    
    echo
    echo "文件统计:"
    echo "  位置RMS分析文件: $pos_files 个"
    echo "  横向残差分析文件: $lateral_files 个"
    echo "  RMS曲线图目录: $plot_dirs 个"
    echo "  分析汇总报告: analysis_summary.txt"
    echo
    
    echo "查看建议:"
    echo "  查看特定文件: cat $RESULTS_DIR/vdr_xxx/turn_rms_analysis.txt"
    echo "  查看RMS曲线图: open $RESULTS_DIR/vdr_xxx/plots/"
    echo "  批量查看位置RMS: find $RESULTS_DIR -name 'turn_rms_analysis.txt' -exec cat {} \;"
    echo "  批量查看横向残差: find $RESULTS_DIR -name 'turn_lateral_analysis.txt' -exec cat {} \;"
    echo
    echo "结果目录: $RESULTS_DIR"
    
    # 打开结果目录
    if command -v open &> /dev/null; then
        log_info "在Finder中打开结果目录..."
        open "$RESULTS_DIR"
    fi
    
    echo "================================"
}

# 显示帮助信息
show_help() {
    echo -e "${CYAN}ESKF结果分析脚本${NC}"
    echo
    echo "用法:"
    echo "  $0 [结果目录路径]"
    echo
    echo "参数:"
    echo "  结果目录路径    ESKF批量处理的结果目录"
    echo
    echo "示例:"
    echo "  $0"
    echo "  $0 ~/Data/log_results"
    echo
    echo "功能:"
    echo "  - 自动解析processing_summary.txt"
    echo "  - 按文件分组执行转弯时段RMS分析"
    echo "  - 在每个vdr_xxx/目录内生成独立分析结果"
    echo "  - 转弯段之间用空行分隔，提高可读性"
    echo "  - 自动生成RMS曲线图，可视化分析结果"
    echo "  - 支持批量查看和分析汇总"
}

# 信号处理函数
cleanup() {
    log_warning "收到中断信号，正在清理..."
    exit 1
}

# 主程序入口
main() {
    # 设置信号处理
    trap cleanup SIGINT SIGTERM
    
    # 初始化全局变量
    start_time=$(date +%s)
    pos_rms_success=0
    pos_rms_skipped=0
    lateral_rms_success=0
    lateral_rms_skipped=0
    plot_success=0
    plot_skipped=0
    
    # 检查帮助
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
        exit 0
    fi
    
    # 显示开始信息
    echo
    echo -e "${CYAN}=== ESKF结果分析脚本 ===${NC}"
    echo "分析目录: $RESULTS_DIR"
    echo "开始时间: $(date)"
    echo
    
    # 执行主要流程
    check_prerequisites
    run_batch_analysis
    generate_optimal_delays_summary
    generate_analysis_summary
    show_final_summary
}

# 调用主函数
main "$@"