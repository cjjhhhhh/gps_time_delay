cmake_minimum_required(VERSION 3.10)
project(minimal_slam_ch3)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 基础依赖
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

# 可选依赖（如果安装了就用，没有就跳过）
find_package(yaml-cpp QUIET)

# 添加库搜索路径
link_directories(/opt/homebrew/lib)

# 设置库路径变量
set(CMAKE_LIBRARY_PATH "/opt/homebrew/lib;${CMAKE_LIBRARY_PATH}")

# 包含目录
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 添加子目录
add_subdirectory(src)
