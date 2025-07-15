# ch3可执行文件

# IMU积分测试
add_executable(run_imu_integration
    run_imu_integration.cc
    static_imu_init.cc
    utm_convert.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

target_link_libraries(run_imu_integration
    minimal_slam_common
    /opt/homebrew/lib/libglog.dylib
    /opt/homebrew/lib/libgflags.dylib
)

# ESKF GNSS/INS融合（去掉Pangolin依赖）
add_executable(run_eskf_gins
    run_eskf_gins.cc
    static_imu_init.cc
    utm_convert.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

target_link_libraries(run_eskf_gins
    minimal_slam_common
    /opt/homebrew/lib/libglog.dylib
    /opt/homebrew/lib/libgflags.dylib
)
# GNSS处理
add_executable(process_gnss
    process_gnss.cc
    static_imu_init.cc
    utm_convert.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

target_link_libraries(process_gnss
    minimal_slam_common
    /opt/homebrew/lib/libglog.dylib
    /opt/homebrew/lib/libgflags.dylib
)
