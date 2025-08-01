#!/bin/bash

# PLOG批量转换脚本
# 将文件夹下的所有.plog文件转换为.log文件
# 用法: ./convert_plog_batch.sh [plog目录] [DecodeLogger路径]

# 默认参数设置
PLOG_DIR="${1:-/Users/cjj/Data/vdr_plog/XiaoMi11}"
DECODER_PATH="${2:-/Users/cjj/Downloads/DecodeLogger}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

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

# 检查环境
check_environment() {
    log_info "检查转换环境..."
    
    # 检查plog目录
    if [[ ! -d "$PLOG_DIR" ]]; then
        log_error "PLOG目录不存在: $PLOG_DIR"
        exit 1
    fi
    
    # 检查DecodeLogger
    if [[ ! -f "$DECODER_PATH" ]]; then
        log_error "DecodeLogger不存在: $DECODER_PATH"
        exit 1
    fi
    
    # 检查DecodeLogger执行权限
    if [[ ! -x "$DECODER_PATH" ]]; then
        log_error "DecodeLogger没有执行权限: $DECODER_PATH"
        exit 1
    fi
    
    log_success "环境检查完成"
}

# 获取文件大小
get_file_size() {
    local file="$1"
    if [[ -f "$file" ]]; then
        stat -f%z "$file" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

# 转换单个plog文件
convert_single_plog() {
    local plog_file="$1"
    local filename=$(basename "$plog_file" .plog)
    local log_file="${plog_file%.plog}.log"
    
    # 检查是否已经存在log文件
    if [[ -f "$log_file" ]]; then
        local plog_size=$(get_file_size "$plog_file")
        local log_size=$(get_file_size "$log_file")
        local plog_time=$(stat -f%m "$plog_file" 2>/dev/null || echo "0")
        local log_time=$(stat -f%m "$log_file" 2>/dev/null || echo "0")
        
        # 如果log文件比plog文件新且不为空，删除plog文件并跳过转换
        if [[ $log_time -ge $plog_time && $log_size -gt 0 ]]; then
            if rm "$plog_file"; then
                log_warning "  跳过 $filename (已存在log文件，已删除plog)"
            else
                log_warning "  跳过 $filename (已存在log文件，plog删除失败)"
            fi
            return 2  # 跳过状态
        fi
    fi
    
    local plog_size=$(get_file_size "$plog_file")
    local plog_size_mb=$((plog_size / 1024 / 1024))
    
    log_info "  转换: $filename (${plog_size_mb}MB)"
    
    # 执行转换
    local start_time=$(date +%s)
    if "$DECODER_PATH" "$plog_file" > /dev/null 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        # 检查输出文件是否存在且不为空
        if [[ -f "$log_file" && -s "$log_file" ]]; then
            local log_size=$(get_file_size "$log_file")
            local size_mb=$((log_size / 1024 / 1024))
            
            # 🔧 转换成功，删除原plog文件
            if rm "$plog_file"; then
                log_success "  ✓ $filename (${duration}s, ${size_mb}MB, 原文件已删除)"
                return 0  # 成功状态
            else
                log_success "  ✓ $filename (${duration}s, ${size_mb}MB)"
                log_warning "  ⚠ $filename (原plog文件删除失败)"
                return 0  # 仍然算成功，因为转换完成了
            fi
        else
            log_error "  ✗ $filename (转换失败：输出文件为空或不存在)"
            return 1  # 失败状态
        fi
    else
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        log_error "  ✗ $filename (转换失败，耗时: ${duration}s)"
        return 1  # 失败状态
    fi
}

# 显示系统信息
show_system_info() {
    echo
    echo -e "${CYAN}=== PLOG批量转换工具 ===${NC}"
    echo "PLOG目录: $PLOG_DIR"
    echo "解码器路径: $DECODER_PATH"
    echo "操作系统: $(uname -s) $(uname -r)"
    echo "当前时间: $(date)"
    echo -e "${CYAN}=============================${NC}"
    echo
}

# 主处理函数
main_process() {
    show_system_info
    
    log_info "开始PLOG批量转换..."
    
    # 查找所有.plog文件
    local plog_files=()
    while IFS= read -r -d '' file; do
        plog_files+=("$file")
    done < <(find "$PLOG_DIR" -name "*.plog" -type f -print0)
    
    local total_plogs=${#plog_files[@]}
    
    if [[ $total_plogs -eq 0 ]]; then
        log_warning "未找到任何.plog文件在目录: $PLOG_DIR"
        exit 0
    fi
    
    log_info "找到 $total_plogs 个PLOG文件"
    
    # 统计变量
    local success_count=0
    local failed_count=0
    local skipped_count=0
    local current_count=0
    
    echo
    echo "========================================"
    log_info "开始转换处理"
    echo "========================================"
    
    local start_time=$(date +%s)
    
    # 处理每个plog文件
    for plog_file in "${plog_files[@]}"; do
        current_count=$((current_count + 1))
        
        # 显示进度
        local progress=$((current_count * 100 / total_plogs))
        printf "\n${PURPLE}[进度: %d/%d (%d%%)]${NC}\n" $current_count $total_plogs $progress
        
        # 转换文件
        convert_single_plog "$plog_file"
        local result=$?
        
        case $result in
            0)
                success_count=$((success_count + 1))
                ;;
            1)
                failed_count=$((failed_count + 1))
                ;;
            2)
                skipped_count=$((skipped_count + 1))
                ;;
        esac
        
        printf "当前统计: 成功: %d, 失败: %d, 跳过: %d\n" $success_count $failed_count $skipped_count
    done
    
    local end_time=$(date +%s)
    local total_time=$((end_time - start_time))
    local hours=$((total_time / 3600))
    local minutes=$(((total_time % 3600) / 60))
    local seconds=$((total_time % 60))
    
    echo
    echo "========================================"
    log_success "PLOG批量转换完成！"
    echo
    echo "转换统计:"
    echo "  总文件数: $total_plogs"
    echo "  转换成功: $success_count"
    echo "  转换失败: $failed_count"
    echo "  跳过文件: $skipped_count"
    
    if [[ $total_plogs -gt 0 ]]; then
        local success_rate=$(( (success_count + skipped_count) * 100 / total_plogs))
        echo "  有效率: ${success_rate}%"
    fi
    
    echo "  总耗时: ${hours}h ${minutes}m ${seconds}s"
    echo
    
    # 统计转换后的文件和空间释放
    local log_files=$(find "$PLOG_DIR" -name "*.log" -type f | wc -l)
    local remaining_plogs=$(find "$PLOG_DIR" -name "*.plog" -type f | wc -l)
    local total_log_size=0
    
    while IFS= read -r -d '' file; do
        local size=$(get_file_size "$file")
        total_log_size=$((total_log_size + size))
    done < <(find "$PLOG_DIR" -name "*.log" -type f -print0)
    
    local total_log_size_mb=$((total_log_size / 1024 / 1024))
    
    echo "转换结果:"
    echo "  LOG文件数: $log_files"
    echo "  LOG总大小: ${total_log_size_mb}MB"
    echo "  剩余PLOG文件: $remaining_plogs"
    echo "  保存位置: $PLOG_DIR"
    
    # 空间释放统计
    local deleted_plogs=$((success_count + skipped_count))
    if [[ $deleted_plogs -gt 0 ]]; then
        echo
        log_success "空间释放统计:"
        echo "  已删除PLOG文件: $deleted_plogs 个"
        echo "  空间释放: 原PLOG文件占用的空间已释放"
    fi
    
    # 如果有失败的文件，给出建议
    if [[ $failed_count -gt 0 ]]; then
        echo
        log_warning "转换失败建议："
        echo "  1. 检查PLOG文件是否损坏"
        echo "  2. 检查磁盘空间是否充足"
        echo "  3. 检查DecodeLogger程序权限"
        echo "  4. 可以重新运行脚本继续转换失败的文件"
    fi
    
    echo "========================================"
}

# 显示帮助信息
show_help() {
    echo -e "${CYAN}PLOG批量转换工具${NC}"
    echo
    echo "用法:"
    echo "  $0 [PLOG目录] [DecodeLogger路径]"
    echo
    echo "参数:"
    echo "  PLOG目录          包含.plog文件的目录"
    echo "  DecodeLogger路径  DecodeLogger可执行文件路径"
    echo
    echo "示例:"
    echo "  $0"
    echo "  $0 ~/Data/plog_files"
    echo "  $0 ~/Data/plog_files ~/Tools/DecodeLogger"
    echo
    echo "功能:"
    echo "  - 自动发现目录下的所有.plog文件"
    echo "  - 批量转换为.log文件"
    echo "  - 转换成功后自动删除原.plog文件释放空间"
    echo "  - 智能跳过已转换的文件"
    echo "  - 显示详细的转换进度和统计"
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
    
    # 检查帮助
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
        exit 0
    fi
    
    # 检查环境
    check_environment
    
    # 执行主要流程
    main_process
}

# 调用主函数
main "$@"