#!/bin/bash

# Mac版本的GNSS/INS日志批量处理脚本 - 兼容性修复版
# 针对macOS环境优化，兼容旧版bash
# 用法: ./mac_batch_process.sh [日志文件夹路径] [可执行文件路径] [输出目录路径]

# 默认参数设置
LOG_DIR="${1:-/Users/cjj/Data/vdr_plog/XiaoMi11}"
EXEC_PATH="${2:-/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/bin/run_eskf_gins}"
OUTPUT_BASE_DIR="${3:-/Users/cjj/Data/log_results/XiaoMi11}"
# GPS时间偏移范围设置
GPS_OFFSET_START=0.00
GPS_OFFSET_END=-0.40
GPS_OFFSET_STEP=-0.05

# 全局变量声明（兼容旧版bash）
start_time=0
success_count=0
failed_count=0

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

# 测试可执行文件
test_executable() {
    log_info "测试可执行文件..."
    
    # 检查文件是否存在
    if [[ ! -f "$EXEC_PATH" ]]; then
        log_error "可执行文件不存在: $EXEC_PATH"
        return 1
    fi
    
    # 检查执行权限
    if [[ ! -x "$EXEC_PATH" ]]; then
        log_error "文件没有执行权限: $EXEC_PATH"
        return 1
    fi
    
    # 测试运行（显示帮助信息）
    log_info "尝试运行可执行文件获取帮助信息..."
    local test_output
    test_output=$("$EXEC_PATH" --help 2>&1)
    local exit_code=$?
    
    # 检查是否输出了帮助信息（表明程序能正常运行）
    if echo "$test_output" | grep -q "Flags from"; then
        log_success "可执行文件测试通过 (检测到gflags帮助输出)"
        return 0
    elif echo "$test_output" | grep -q "usage\|help\|Usage\|Help"; then
        log_success "可执行文件测试通过 (检测到帮助信息)"
        return 0
    elif [[ $exit_code -eq 0 ]]; then
        log_success "可执行文件测试通过 (退出码正常)"
        return 0
    else
        log_error "可执行文件测试失败，退出码: $exit_code"
        log_error "错误输出: $test_output"
        
        # 检查依赖库
        if command -v otool &> /dev/null; then
            log_info "检查依赖库..."
            otool -L "$EXEC_PATH" | head -10
        fi
        
        return 1
    fi
}

# Mac系统检查
check_mac_environment() {
    log_info "检查Mac环境..."
    
    # 检查macOS版本
    local mac_version=$(sw_vers -productVersion 2>/dev/null || echo "unknown")
    log_info "macOS版本: $mac_version"
    
    # 检查bash版本
    local bash_version=$($SHELL --version | head -n1 2>/dev/null || echo "unknown")
    log_info "Shell版本: $bash_version"
    
    # 检查是否安装了Homebrew
    if ! command -v brew &> /dev/null; then
        log_warning "未检测到Homebrew，某些依赖可能需要手动安装"
    else
        log_success "Homebrew已安装: $(brew --version 2>/dev/null | head -n1 || echo "unknown version")"
    fi
    
    # 检查bc命令
    if ! command -v bc &> /dev/null; then
        log_warning "bc命令未安装，将使用整数计算模式"
    else
        log_success "bc命令可用"
    fi
}

# 检查必要条件
check_prerequisites() {
    log_info "检查处理环境..."
    
    # 检查日志文件夹
    if [[ ! -d "$LOG_DIR" ]]; then
        log_error "日志文件夹不存在: $LOG_DIR"
        exit 1
    fi
    
    # 测试可执行文件
    if ! test_executable; then
        log_error "可执行文件测试失败，请检查程序是否正常"
        exit 1
    fi
    
    # 创建输出目录
    if [[ ! -d "$OUTPUT_BASE_DIR" ]]; then
        mkdir -p "$OUTPUT_BASE_DIR"
        if [[ $? -eq 0 ]]; then
            log_info "创建输出目录: $OUTPUT_BASE_DIR"
        else
            log_error "无法创建输出目录: $OUTPUT_BASE_DIR"
            exit 1
        fi
    fi
    
    log_success "环境检查完成"
}

# 生成GPS偏移数组（兼容版本）
generate_gps_offsets() {
    local offsets=""
    local current=$GPS_OFFSET_START
    
    # 判断步长方向
    local is_positive_step=true
    if command -v bc &> /dev/null; then
        if (( $(echo "$GPS_OFFSET_STEP < 0" | bc -l) )); then
            is_positive_step=false
        fi
    else
        # 简单判断：如果包含负号就是负步长
        if [[ "$GPS_OFFSET_STEP" == *"-"* ]]; then
            is_positive_step=false
        fi
    fi
    
    while true; do
        offsets="$offsets $(printf "%.2f" $current)"
        
        # 计算下一个值
        if command -v bc &> /dev/null; then
            local next=$(echo "$current + $GPS_OFFSET_STEP" | bc -l 2>/dev/null)
            
            # 根据步长方向判断是否结束
            if [[ "$is_positive_step" == true ]]; then
                # 正偏移：递增直到超过结束值
                if (( $(echo "$next > $GPS_OFFSET_END" | bc -l 2>/dev/null) )); then
                    break
                fi
            else
                # 负偏移：递减直到小于结束值
                if (( $(echo "$next < $GPS_OFFSET_END" | bc -l 2>/dev/null) )); then
                    break
                fi
            fi
            current=$next
        else
            # 不使用bc的备用方案
            break  # 如果没有bc，只返回起始值
        fi
    done
    
    echo $offsets
}

# 获取文件大小（Mac版本）
get_file_size() {
    local file="$1"
    if [[ -f "$file" ]]; then
        stat -f%z "$file" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

# 处理单个日志文件
process_single_log() {
    local log_file="$1"
    local gps_offset="$2" 
    local log_name=$(basename "$log_file" .log)
    
    # 创建输出目录
    local log_output_dir="$OUTPUT_BASE_DIR/${log_name}"
    mkdir -p "$log_output_dir"
    
    # 保存当前目录
    local original_dir=$(pwd)
    
    cd "$log_output_dir"
    
    # 构造执行命令
    local cmd="\"$EXEC_PATH\" --txt_path=\"$log_file\" --offline_mode=true --gps_time_offset=$gps_offset"
    
    log_info "在目录 $log_output_dir 中执行程序"
    
    # 执行程序
    local process_start_time=$(date +%s)
    if timeout 300 bash -c "$cmd" > "${log_name}_offset_${gps_offset}.log" 2>&1; then

        local process_end_time=$(date +%s)
        local process_duration=$((process_end_time - process_start_time))
        
        # 检查输出文件大小
        local output_file_size=0
        if [[ -f "corrections.txt" ]]; then
            output_file_size=$(get_file_size "corrections.txt")
        fi
        
        echo "$(date '+%Y-%m-%d %H:%M:%S'),${log_name},${gps_offset},SUCCESS,${log_output_dir},${process_duration},${output_file_size}" >> "$OUTPUT_BASE_DIR/processing_summary.txt"

        log_success "处理完成，结果文件已保存在: $log_output_dir"
        success_count=$((success_count + 1))
        
        # 🔧 重要：返回原目录
        cd "$original_dir"
        return 0
    else

        local process_end_time=$(date +%s)
        local process_duration=$((process_end_time - process_start_time))
        
        echo "$(date '+%Y-%m-%d %H:%M:%S'),${log_name},${gps_offset},FAILED,${log_output_dir},${process_duration},0" >> "$OUTPUT_BASE_DIR/processing_summary.txt"

        log_error "处理失败"
        failed_count=$((failed_count + 1))
        
        # 🔧 重要：返回原目录
        cd "$original_dir"
        return 1
    fi
}

# 显示系统信息
show_system_info() {
    echo
    echo -e "${CYAN}=== 系统信息 ===${NC}"
    echo "操作系统: $(uname -s) $(uname -r)"
    echo "架构: $(uname -m)"
    echo "CPU核心数: $(sysctl -n hw.ncpu 2>/dev/null || echo "unknown")"
    local mem_gb=$(( $(sysctl -n hw.memsize 2>/dev/null || echo "0") / 1024 / 1024 / 1024 ))
    echo "内存大小: ${mem_gb}GB"
    echo "当前用户: $(whoami)"
    echo "工作目录: $(pwd)"
    echo -e "${CYAN}=====================${NC}"
    echo
}

# 主处理函数
main_process() {
    show_system_info
    
    log_info "开始智能批量处理..."
    log_info "日志文件夹: $LOG_DIR"
    log_info "可执行文件: $EXEC_PATH"
    log_info "输出目录: $OUTPUT_BASE_DIR"
    
    # 查找所有.log文件
    local log_files=()
    while IFS= read -r -d '' file; do
        log_files+=("$file")
    done < <(find "$LOG_DIR" -name "*.log" -type f -print0)
    
    local total_logs=${#log_files[@]}
    
    if [[ $total_logs -eq 0 ]]; then
        log_warning "未找到任何.log文件在目录: $LOG_DIR"
        exit 0
    fi
    
    log_info "找到 $total_logs 个日志文件"

    # ========================================
# 第0阶段：GPS时间戳差值可视化
# ========================================

echo
echo "========================================"
log_info "第0阶段：GPS时间戳差值可视化"
echo "========================================"

local timestamp_plot_dir="$OUTPUT_BASE_DIR/timestamp_plots"
mkdir -p "$timestamp_plot_dir"
    local script_dir=$(dirname "$0")

local timestamp_script="$script_dir/plot_gps_timestamps.py"

if [[ -f "$timestamp_script" ]]; then
    log_info "生成GPS时间戳差值图表..."
    
    if python3 "$timestamp_script" \
        --input "$LOG_DIR" \
        --output "$timestamp_plot_dir" > "$timestamp_plot_dir/plot_generation.log" 2>&1; then
        
        log_success "时间戳差值图表生成完成"
        log_info "图表保存位置: $timestamp_plot_dir"
    else
        log_error "时间戳图表生成失败，详情查看: $timestamp_plot_dir/plot_generation.log"
    fi
else
    log_warning "时间戳绘图脚本不存在: $timestamp_script，跳过此步骤"
fi


    
    # ========================================
    # 第一阶段：转弯检测筛选
    # ========================================
    
    echo
    echo "========================================"
    log_info "第一阶段：转弯检测筛选"
    echo "========================================"
    
    # 创建转弯分析目录
    local turn_analysis_dir="$OUTPUT_BASE_DIR/turn_analysis"
    mkdir -p "$turn_analysis_dir"
    
    # 转弯检测脚本路径
    local turn_script="$script_dir/detect_turns.py"
    
    # 检查转弯检测脚本
    if [[ ! -f "$turn_script" ]]; then
        log_error "转弯检测脚本不存在: $turn_script"
        log_error "无法进行智能筛选，退出处理"
        exit 1
    fi
    
    # 检查Python环境
    if ! command -v python3 &> /dev/null; then
        log_error "Python3未安装，无法进行转弯检测"
        exit 1
    fi
    
    # 执行转弯检测
    log_info "转弯检测参数: 开始阈值=3°/s, 结束阈值=1.5°/s, 持续时间=3s, 累积角度=30°"
    local turn_start_time=$(date +%s)
        
    if python3 "$turn_script" \
        --input "$LOG_DIR" \
        --output "$turn_analysis_dir" \
        --start_threshold 3.0 \
        --end_threshold 1.5 \
        --end_duration 3.0 \
        --angle_threshold 30.0 > "$turn_analysis_dir/turn_detection.log" 2>&1; then
        
        local turn_end_time=$(date +%s)
        local turn_duration=$((turn_end_time - turn_start_time))
        log_success "转弯检测完成 (耗时: ${turn_duration}s)"
    else
        log_error "转弯检测失败"
        log_error "详细错误信息请查看: $turn_analysis_dir/turn_detection.log"
        exit 1
    fi
    
    # 分析转弯检测结果，筛选有转弯的文件
    local files_with_turns=()
    local files_without_turns=()
    
    log_info "分析转弯检测结果..."
    
    for log_file in "${log_files[@]}"; do
        local log_name=$(basename "$log_file" .log)
        local turn_file="$turn_analysis_dir/${log_name}_turns_nzz.txt"
        
        if [[ -f "$turn_file" ]]; then
            # 计算转弯段数量（排除注释行）
            local turn_count=$(grep -v "^#" "$turn_file" 2>/dev/null | wc -l)
            
            if [[ $turn_count -gt 0 ]]; then
                files_with_turns+=("$log_file")
                log_success "  $log_name: $turn_count 个转弯段 ✓"
            else
                files_without_turns+=("$log_file")
                log_info "  $log_name: 无转弯段 ○"
            fi
        else
            files_without_turns+=("$log_file")
            log_warning "  $log_name: 转弯检测失败 ✗"
        fi
    done
    
    local files_with_turns_count=${#files_with_turns[@]}
    local files_without_turns_count=${#files_without_turns[@]}
    
    echo
    log_info "筛选结果统计:"
    echo "  总文件数: $total_logs"
    echo "  有转弯文件: $files_with_turns_count"
    echo "  无转弯文件: $files_without_turns_count"
    
    # 创建筛选结果汇总文件
    local filter_summary="$turn_analysis_dir/filter_summary.txt"
    echo "# 转弯筛选结果汇总" > "$filter_summary"
    echo "# 生成时间: $(date)" >> "$filter_summary"
    echo "# 总文件数: $total_logs" >> "$filter_summary"
    echo "# 有转弯文件: $files_with_turns_count" >> "$filter_summary"
    echo "# 无转弯文件: $files_without_turns_count" >> "$filter_summary"
    echo "#" >> "$filter_summary"
    echo "# 状态,文件名,转弯段数量" >> "$filter_summary"
    
    for log_file in "${files_with_turns[@]}"; do
        local log_name=$(basename "$log_file" .log)
        local turn_file="$turn_analysis_dir/${log_name}_turns_nzz.txt"
        local turn_count=$(grep -v "^#" "$turn_file" 2>/dev/null | wc -l)
        echo "有转弯,$log_name,$turn_count" >> "$filter_summary"
    done
    
    for log_file in "${files_without_turns[@]}"; do
        local log_name=$(basename "$log_file" .log)
        echo "无转弯,$log_name,0" >> "$filter_summary"
    done
    
    log_info "筛选汇总报告: $filter_summary"

    # 自动移动无转弯文件到备份目录
    if [[ $files_without_turns_count -gt 0 ]]; then
        echo
        echo "========================================"
        log_info "自动清理无转弯文件"
        echo "========================================"
        
        echo "无转弯文件列表："
        local cleanup_size=0
        for log_file in "${files_without_turns[@]}"; do
            local log_name=$(basename "$log_file")
            local file_size=$(get_file_size "$log_file")
            local size_mb=$((file_size / 1024 / 1024))
            echo "  - $log_name (${size_mb}MB)"
            cleanup_size=$((cleanup_size + file_size))
        done
        
        local cleanup_size_mb=$((cleanup_size / 1024 / 1024))
        
        echo
        log_info "将移动 $files_without_turns_count 个文件到备份目录"
        log_info "预计释放空间: ${cleanup_size_mb}MB"
        
        # 执行移动到备份目录
        move_to_backup "${files_without_turns[@]}"
    else
        echo
        log_success "所有文件都有转弯段，无需清理"
    fi
    
    # 如果没有转弯文件，直接结束
    if [[ $files_with_turns_count -eq 0 ]]; then
        echo
        log_warning "没有检测到转弯文件，跳过ESKF处理"
        log_info "所有结果已保存在: $OUTPUT_BASE_DIR"
        return 0
    fi
    
    # ========================================
    # 第二阶段：ESKF处理（仅处理有转弯的文件）
    # ========================================
    
    echo
    echo "========================================"
    log_info "第二阶段：ESKF处理有转弯的文件"
    echo "========================================"
    
    log_info "将处理 $files_with_turns_count 个有转弯的文件（跳过 $files_without_turns_count 个无转弯文件）"
    
    # 生成GPS偏移数组
    local gps_offsets_str=$(generate_gps_offsets)
    local gps_offsets=($gps_offsets_str)
    local total_offsets=${#gps_offsets[@]}
    
    log_info "GPS偏移值: $gps_offsets_str"
    log_info "GPS偏移数量: $total_offsets 个值"
    
    # 初始化汇总文件
    echo "时间戳,日志文件,GPS偏移,状态,输出文件,处理时间,文件大小" > "$OUTPUT_BASE_DIR/processing_summary.txt"
    
    # 统计变量
    local total_tasks=$((files_with_turns_count * total_offsets))
    local current_task=0
    
    log_info "总任务数: $total_tasks (仅处理有转弯的文件)"
    echo "================================"
    
    # 双重循环处理（仅处理有转弯的文件）
    for log_file in "${files_with_turns[@]}"; do
        local log_name=$(basename "$log_file" .log)
        local turn_count=$(grep -v "^#" "$turn_analysis_dir/${log_name}_turns_nzz.txt" 2>/dev/null | wc -l)
        
        log_progress "开始处理有转弯的日志文件: $log_name ($turn_count 个转弯段)"
        
        for gps_offset in "${gps_offsets[@]}"; do
            current_task=$((current_task + 1))
            
            # 计算进度
            local progress=$((current_task * 100 / total_tasks))
            
            printf "\n${PURPLE}[进度: %d/%d (%d%%)]${NC}\n" $current_task $total_tasks $progress
            
            process_single_log "$log_file" "$gps_offset"
            
            printf "总体进度: 成功: %d, 失败: %d\n" $success_count $failed_count
        done
        
        log_success "日志文件 $log_name 处理完成"
        echo "--------------------------------"
    done
    
    # 记录跳过的文件信息
    if [[ $files_without_turns_count -gt 0 ]]; then
        echo
        log_info "跳过的无转弯文件："
        for log_file in "${files_without_turns[@]}"; do
            local log_name=$(basename "$log_file" .log)
            echo "  - $log_name"
            
            # 在汇总文件中记录跳过的文件
            for gps_offset in "${gps_offsets[@]}"; do
                echo "$(date '+%Y-%m-%d %H:%M:%S'),${log_name},${gps_offset},SKIPPED_NO_TURNS,,0,0" >> "$OUTPUT_BASE_DIR/processing_summary.txt"
            done
        done
    fi
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
    log_success "智能批量处理完成！"
    echo
    
    # 读取筛选统计
    local filter_summary="$OUTPUT_BASE_DIR/turn_analysis/filter_summary.txt"
    if [[ -f "$filter_summary" ]]; then
        local total_files=$(grep "# 总文件数:" "$filter_summary" | cut -d: -f2 | xargs)
        local files_with_turns=$(grep "# 有转弯文件:" "$filter_summary" | cut -d: -f2 | xargs)
        local files_without_turns=$(grep "# 无转弯文件:" "$filter_summary" | cut -d: -f2 | xargs)
        
        echo "文件筛选统计:"
        echo "  总文件数: $total_files"
        echo "  有转弯文件: $files_with_turns (已处理)"
        echo "  无转弯文件: $files_without_turns (已跳过)"
    fi
    
    echo
    echo "ESKF处理统计:"
    echo "  总任务数: $((success_count + failed_count))"
    echo "  成功: $success_count"
    echo "  失败: $failed_count"
    
    if [[ $((success_count + failed_count)) -gt 0 ]]; then
        local success_rate=$((success_count * 100 / (success_count + failed_count)))
        echo "  成功率: ${success_rate}%"
    fi
    
    echo "  总耗时: ${hours}h ${minutes}m ${seconds}s"
    echo
    echo "结果文件位置: $OUTPUT_BASE_DIR"
    echo "详细汇总: $OUTPUT_BASE_DIR/processing_summary.txt"
    echo "转弯分析: $OUTPUT_BASE_DIR/turn_analysis/"
    
    # 计算节省的时间
    if [[ -f "$filter_summary" ]]; then
        local files_without_turns=$(grep "# 无转弯文件:" "$filter_summary" | cut -d: -f2 | xargs)
        if [[ $files_without_turns -gt 0 ]]; then
            local gps_offsets_str=$(generate_gps_offsets)
            local gps_offsets=($gps_offsets_str)
            local skipped_tasks=$((files_without_turns * ${#gps_offsets[@]}))
            echo
            echo "智能处理优势:"
            echo "  跳过任务数: $skipped_tasks"
            echo "  节省时间: 估计节省 $(( skipped_tasks * 2 / 60 )) 分钟"
        fi
    fi
    
    # 如果所有任务都失败，给出建议
    if [[ $success_count -eq 0 && $failed_count -gt 0 ]]; then
        echo
        log_error "所有ESKF任务都失败了！建议检查："
        echo "  1. 可执行文件是否正常工作"
        echo "  2. 参数格式是否正确"
        echo "  3. 依赖库是否完整"
        echo "  4. 查看错误日志文件获取详细信息"
    fi

    # 打开结果目录
    if command -v open &> /dev/null; then
        log_info "在Finder中打开结果目录..."
        open "$OUTPUT_BASE_DIR"
    fi
    
    echo "================================"
}

# 信号处理函数
cleanup() {
    log_warning "收到中断信号，正在清理..."
    exit 1
}

# 显示帮助信息
show_help() {
    echo -e "${CYAN}GNSS/INS日志批量处理脚本 (Mac兼容版)${NC}"
    echo
    echo "用法:"
    echo "  $0 [日志文件夹路径] [可执行文件路径] [输出目录路径]"
    echo
    echo "参数:"
    echo "  日志文件夹路径    包含.log文件的文件夹"
    echo "  可执行文件路径    run_eskf_gins可执行文件"
    echo "  输出目录路径      结果保存目录"
    echo
    echo "示例:"
    echo "  $0"
    echo "  $0 ~/Documents/logs"
    echo "  $0 ~/Documents/logs ./build/run_eskf_gins ~/Results"
    echo
    echo "GPS偏移范围: ${GPS_OFFSET_START}s 到 ${GPS_OFFSET_END}s (步长: ${GPS_OFFSET_STEP}s)"
}


# 移动文件到备份目录
move_to_backup() {
    local files_to_move=("$@")
    local backup_dir="$OUTPUT_BASE_DIR/no_turns_backup"
    
    log_info "创建备份目录: $backup_dir"
    mkdir -p "$backup_dir"
    
    log_info "移动无转弯文件到备份目录..."
    
    local moved_count=0
    local total_size=0
    
    for file in "${files_to_move[@]}"; do
        local filename=$(basename "$file")
        local file_size=$(get_file_size "$file")
        total_size=$((total_size + file_size))
        
        if mv "$file" "$backup_dir/"; then
            log_success "  ✓ 移动: $filename"
            moved_count=$((moved_count + 1))
        else
            log_error "  ✗ 移动失败: $filename"
        fi
    done
    
    local total_size_mb=$((total_size / 1024 / 1024))
    
    echo
    log_success "文件移动完成！"
    echo "  移动文件数: $moved_count"
    echo "  释放空间: ${total_size_mb}MB"
    echo "  备份位置: $backup_dir"
    echo
    log_info "恢复方法: mv $backup_dir/* $LOG_DIR/"
    
    # 更新原目录文件统计
    local remaining_files=$(find "$LOG_DIR" -name "*.log" -type f | wc -l)
    log_info "原目录剩余文件: $remaining_files 个（仅含转弯路段）"
}

# 主程序入口
main() {
    # 设置信号处理
    trap cleanup SIGINT SIGTERM
    
    # 初始化全局变量
    start_time=$(date +%s)
    success_count=0
    failed_count=0
    
    # 检查帮助
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
        exit 0
    fi
    
    # 检查环境
    check_mac_environment
    
    # 执行主要流程
    check_prerequisites
    main_process
    show_final_summary
}

# 调用主函数
main "$@"