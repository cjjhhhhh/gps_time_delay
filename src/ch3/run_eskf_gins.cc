//
// Created by xiang on 2021/11/11.
//

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "/Users/cjj/Data/vdr_plog/vdr_20250613_181225_863.log", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, false, "是否显示图形界面");
DEFINE_bool(with_odom, false, "是否加入轮速计信息");

/**
 * 本程序演示使用RTK+IMU进行组合导航
 */
bool InitializeESKF(sad::ESKFD& eskf){
    // 陀螺零偏 (度/秒) - 从SINS数据获取
    const double GYRO_BIAS_X = 0.001711;
    const double GYRO_BIAS_Y = -0.021235;
    const double GYRO_BIAS_Z = 0.049159;
    
    // 加速度零偏 (m/s²) - 从SINS数据获取
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

    std::string file_ext = FLAGS_txt_path.substr(FLAGS_txt_path.find_last_of('.') + 1);
    bool is_log_file = (file_ext == "log" || file_ext == "LOG");

    // 初始化器
    sad::StaticIMUInit imu_init;  // 使用默认配置
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    std::ofstream fout("/Users/cjj/work/GNSS_INS/slam/gnss_imu_time/data/ch3/gins_origin.txt");
    bool imu_inited = false, gnss_inited = false;

    // std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    // if (FLAGS_with_ui) {
    //     ui = std::make_shared<sad::ui::PangolinWindow>();
    //     ui->Init();
    // }

    // 根据文件类型选择初始化方式
    if (is_log_file) {
        LOG(INFO) << "检测到日志文件，使用SINS零偏参数";
        if (InitializeESKF(eskf)) {
            imu_inited = true;
        }
    } else {
        LOG(INFO) << "普通数据文件，使用静态初始化";
    }

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数
          if (!is_log_file) {
              if (!imu_init.InitSuccess()) {
                  imu_init.AddIMU(imu);
                  return;
              }

              if (!imu_inited) {
                  sad::ESKFD::Options options;
                  options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
                  options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
                  eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
                  imu_inited = true;
                  return;
              }
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          eskf.Predict(imu);

          /// predict就会更新ESKF，所以此时就可以发送数据
          auto state = eskf.GetNominalState();
        //   if (ui) {
        //       ui->UpdateNavState(state);
        //   }

          /// 记录数据以供绘图
          save_result(fout, state);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            /// GNSS 处理函数 - 详细调试版本
            if (!imu_inited) {
                LOG(INFO) << "GPS: IMU未初始化，跳过";
                return;
            }
            
            auto current_state = eskf.GetNominalState();
            double current_eskf_time = current_state.timestamp_;
            
            LOG(INFO) << "=== GPS处理开始 ===";
            LOG(INFO) << "步骤1 - 收到GPS数据";
            LOG(INFO) << std::fixed << std::setprecision(6) 
                      << "  GPS时间戳: " << gnss.unix_time_ << "s";
            LOG(INFO) << std::fixed << std::setprecision(6) 
                      << "  ESKF时间: " << current_eskf_time << "s";
            LOG(INFO) << std::fixed << std::setprecision(6) 
                      << "  时间差: " << (gnss.unix_time_ - current_eskf_time) << "s";
            
            // 如果GPS时间戳太旧（超过5秒），直接跳过
            if (gnss.unix_time_ < current_eskf_time - 5.0) {
                LOG(WARNING) << "步骤2 - GPS时间戳太旧，跳过处理。时间差: " << (current_eskf_time - gnss.unix_time_) << "s";
                return;
            }
            LOG(INFO) << "步骤2 - GPS时间戳检查通过";

            sad::GNSS gnss_convert = gnss;
            LOG(INFO) << "步骤3 - 创建GPS副本，时间戳: " << gnss_convert.unix_time_ << "s";

            if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                if (is_log_file && sad::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle)) {
                    gnss_convert.heading_valid_ = false;
                    LOG(INFO) << "步骤4 - GPS坐标转换成功，但航向无效";
                } else {
                    LOG(WARNING) << "步骤4 - GPS坐标转换失败";
                    return;
                }
            } else {
                LOG(INFO) << "步骤4 - GPS坐标转换成功，航向有效";
            }
            
            LOG(INFO) << "步骤5 - GPS转换后时间戳: " << gnss_convert.unix_time_ << "s";

            /// 去掉原点
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
                LOG(INFO) << "步骤6 - 设置地图原点: " << origin.transpose();
            } else {
                LOG(INFO) << "步骤6 - 使用已有地图原点";
            }
            gnss_convert.utm_pose_.translation() -= origin;
            
            LOG(INFO) << "步骤7 - 应用地图原点后，GPS时间戳: " << gnss_convert.unix_time_ << "s";

            // 要求RTK heading有效，才能合入ESKF
            if (is_log_file || gnss_convert.heading_valid_) {
                LOG(INFO) << "步骤8 - 准备进行GPS观测";
                
                // 最后检查一次ESKF时间
                auto final_state = eskf.GetNominalState();
                double final_eskf_time = final_state.timestamp_;
                LOG(INFO) << "步骤9 - 最终检查 (高精度):";
                LOG(INFO) << std::fixed << std::setprecision(6) 
                        << "  GPS时间戳: " << gnss_convert.unix_time_ << "s";
                LOG(INFO) << std::fixed << std::setprecision(6) 
                        << "  ESKF时间: " << final_eskf_time << "s";
                LOG(INFO) << std::fixed << std::setprecision(6) 
                        << "  精确时间差: " << (gnss_convert.unix_time_ - final_eskf_time) << "s";
                    
                if (gnss_convert.unix_time_ < final_eskf_time) {
                    LOG(WARNING) << "检测到时间戳冲突！";
                    LOG(WARNING) << std::fixed << std::setprecision(9) 
                                << "  GPS=" << gnss_convert.unix_time_ 
                                << "s < ESKF=" << final_eskf_time << "s";
                    LOG(WARNING) << std::fixed << std::setprecision(9) 
                                << "  精确差值: " << (final_eskf_time - gnss_convert.unix_time_) << "s";
                } else {
                    LOG(INFO) << "步骤10 - 时间戳检查通过";
                }
                
                LOG(INFO) << "步骤11 - 调用eskf.ObserveGps()...";
                try {
                    eskf.ObserveGps(gnss_convert);
                    LOG(INFO) << "步骤12 - GPS观测成功！";
                    
                    auto state = eskf.GetNominalState();
                    save_result(fout, state);
                    gnss_inited = true;
                    
                } catch (const std::exception& e) {
                    LOG(ERROR) << "步骤12 - GPS观测异常: " << e.what();
                } catch (...) {
                    LOG(ERROR) << "步骤12 - GPS观测未知异常";
                }
            } else {
                LOG(INFO) << "步骤8 - GPS航向无效，跳过观测";
            }
            
            LOG(INFO) << "=== GPS处理结束 ===";
        })
        .SetOdomProcessFunc([&](const sad::Odom& odom) {
            /// Odom 处理函数，本章Odom只给初始化使用
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
            }
        })
        .Go();

    // while (ui && !ui->ShouldQuit()) {
    //     usleep(1e5);
    // }
    // if (ui) {
    //     ui->Quit();
    // }
    return 0;
}