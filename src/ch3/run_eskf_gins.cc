//
// Created by xiang on 2021/11/11.
//

#include "ch3/eskf.hpp"
#include "common/io_utils.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <queue>

DEFINE_string(txt_path, "/Users/cjj/Data/vdr_plog/vdr_20250613_181225_863.log", "数据文件路径");

/**
 * 本程序演示使用RTK+IMU进行组合导航
 */
bool InitializeESKF(sad::ESKFD& eskf){
    // 陀螺零偏 (度/秒) 
    const double GYRO_BIAS_X = 0.001711;
    const double GYRO_BIAS_Y = -0.021235;
    const double GYRO_BIAS_Z = 0.049159;
    
    // 加速度零偏 (m/s²) 
    const double ACCEL_BIAS_X = -0.013369;
    const double ACCEL_BIAS_Y = -0.020087;
    const double ACCEL_BIAS_Z = 0.101552;
    
    sad::ESKFD::Options options;
    options.gyro_var_ = 2e-3;     // 陀螺噪声
    options.acce_var_ = 5e-2;     // 加速度噪声
    options.bias_gyro_var_ = 1e-6; // 陀螺零偏随机游走
    options.bias_acce_var_ = 1e-4; // 加速度零偏随机游走

    Vec3d init_bg(GYRO_BIAS_X * sad::math::kDEG2RAD, GYRO_BIAS_Y * sad::math::kDEG2RAD, GYRO_BIAS_Z * sad::math::kDEG2RAD);
    Vec3d init_ba(ACCEL_BIAS_X, ACCEL_BIAS_Y, ACCEL_BIAS_Z);
    Vec3d gravity(0, 0, -9.8);

    eskf.SetInitialConditions(options, init_bg, init_ba, gravity);
    return true;


}
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_txt_path.empty()) {
        return -1;
    }

    // 初始化器
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state,
                                                const Vec3d& gps_pos = Vec3d::Zero(), bool has_gps = false) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        //添加GPS观测位置
        if (has_gps) {
            save_vec3 (fout, gps_pos);
            fout << "1";
        } else {
            fout << "0 0 0 0";
        }
        fout << std::endl;
    };

    std::ofstream fout("/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/gins_new1.txt");

    // 新增：P矩阵协方差数据文件
    std::ofstream cov_file("/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/covariance_new1.txt");

    bool imu_inited = false, gnss_inited = false;

  
    LOG(INFO) << "初始化ESKF";
    if (InitializeESKF(eskf)) {
        imu_inited = true;
    }

    //GNSS缓存队列
    std::queue<sad::GNSS> pending_gps_queue;

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    //存储最新的GPS观测位置
    Vec3d latest_gps_pos = Vec3d::Zero();
    bool has_latest_gps = false;
    double latest_gps_time = 0.0;

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          // 记录IMU预测后的协方差
          eskf.SaveCovariance(cov_file);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto current_state = eskf.GetNominalState();
          double current_eskf_time = current_state.timestamp_;

          //检查是否有GPS数据需要处理
          while (!pending_gps_queue.empty()) {
            sad::GNSS& catch_gps = pending_gps_queue.front();
            //IMU递推到缓存的GNSS时刻
            if (current_eskf_time >= catch_gps.unix_time_) {
                LOG(INFO) << "=== 处理缓存的GPS数据 ===";
                LOG(INFO) << "IMU时间: " << std::fixed << std::setprecision(9) << current_eskf_time
                          << ", GPS时间: " << std::fixed << std::setprecision(9) << catch_gps.unix_time_;
                try{

                    eskf.ObserveGps(catch_gps);

                    // 记录GPS更新后的协方差
                    eskf.SaveCovariance(cov_file);

                    LOG(INFO) << "GPS观测成功, 时间同步正确";
                } catch (...) {
                    LOG (ERROR) << "GNSS观测失败";
                }
                pending_gps_queue.pop();
            }else {
                // IMU还没追上GPS时刻，退出循环
                LOG(INFO) << "等待IMU递推: current=" << std::fixed << std::setprecision(9) << current_eskf_time 
                          << ", waiting_gps=" << catch_gps.unix_time_;
                break;
            }
          }

          //检查是否有时间接近的GPS观测数据
          bool use_gps_obs = false;
          Vec3d gps_obs_pos = Vec3d::Zero();
          if (has_latest_gps) {
              use_gps_obs = true;
              gps_obs_pos = latest_gps_pos;
          }
          /// 记录数据以供绘图
          save_result(fout, current_state, gps_obs_pos, use_gps_obs);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            /// GNSS 处理函数 - 详细调试版本
            if (!imu_inited) {
                LOG(INFO) << "GPS: IMU未初始化，跳过";
                return;
            }
            //添加GNSS时间延迟
            sad::GNSS gnss_convert = gnss;
            gnss_convert.unix_time_ += 0.0;

            auto current_state = eskf.GetNominalState();
            double current_eskf_time = current_state.timestamp_;
            
            LOG(INFO) << "=== GPS数据到达 ===";
            LOG(INFO) << "原始GPS时间: " << gnss.unix_time_ << "s";
            LOG(INFO) << "延迟GPS时间: " << gnss_convert.unix_time_ << "s"; 
            LOG(INFO) << "当前ESKF时间: " << current_eskf_time << "s";
            LOG(INFO) << "时间差: " << (gnss_convert.unix_time_ - current_eskf_time) << "s";

            // 跳过太旧的GPS
            if (gnss_convert.unix_time_ < current_eskf_time - 5.0) {
                LOG(WARNING) << "GPS数据太旧，跳过";
                return;
            }
            if (!sad::ConvertGps2UTM(gnss_convert, Vec2d::Zero(), 0.0)) {
                LOG(WARNING) << "GPS坐标转换失败";
                return;
            }
            /// 设置地图原点（去掉原点）
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
                LOG(INFO) << "设置地图原点: " << origin.transpose();
            } else {
                LOG(INFO) << "步骤6 - 使用已有地图原点";
            }
            
            //保存GPS观测位置（去掉原点）
            Vec3d gps_obs_position = gnss_convert.utm_pose_.translation() - origin;
            latest_gps_pos = gps_obs_position;
            has_latest_gps = true;
            latest_gps_time = gnss_convert.unix_time_;

            LOG(INFO) << "步骤6.5 - 保存GPS观测位置" << gps_obs_position.transpose();

            gnss_convert.utm_pose_.translation() -= origin;
            
            LOG(INFO) << "步骤7 - 应用地图原点后，GPS时间戳: " << gnss_convert.unix_time_ << "s";

            try {
                if (current_eskf_time >= gnss_convert.unix_time_) {
                    LOG(INFO) << "GPS时间不超前, 立即处理";
                    eskf.ObserveGps(gnss_convert);
                    eskf.SaveCovariance(cov_file);
                    LOG(INFO) << "GPS观测成功";
                    gnss_inited = true;
                } else {
                    LOG(INFO) << "GPS时间超前, 缓存等待IMU递推";
                    pending_gps_queue.push(gnss_convert);
                    gnss_inited = true;
                }
            } catch (...) {
                LOG(ERROR) << "GPS观测异常";
            }

            
            LOG(INFO) << "=== GPS处理结束 ===";
        })
        .Go();

    return 0;
}