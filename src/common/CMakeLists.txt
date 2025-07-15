# common库源文件
set(COMMON_SRCS
    io_utils.cc
    timer/timer.cc
)

# 创建common库
add_library(minimal_slam_common ${COMMON_SRCS})
target_link_libraries(minimal_slam_common glog gflags)
target_include_directories(minimal_slam_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
