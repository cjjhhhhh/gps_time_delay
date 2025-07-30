//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_ESKF_HPP
#define SLAM_IN_AUTO_DRIVING_ESKF_HPP

#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include <fstream> 

#include <glog/logging.h>
#include <iomanip>

namespace sad {

/**
 * 书本第3章介绍的误差卡尔曼滤波器
 * 可以指定观测GNSS的读数，GNSS应该事先转换到车体坐标系
 *
 * 本书使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav，与书本对应
 * @tparam S    状态变量的精度，取float或double
 */
template <typename S = double>
class ESKF {
   public:
    /// 类型定义
    using SO3 = Sophus::SO3<S>;                     // 旋转变量类型
    using VecT = Eigen::Matrix<S, 3, 1>;            // 向量类型
    using Vec18T = Eigen::Matrix<S, 18, 1>;         // 18维向量类型
    using Mat3T = Eigen::Matrix<S, 3, 3>;           // 3x3矩阵类型
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声类型
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS噪声类型
    using Mat18T = Eigen::Matrix<S, 18, 18>;        // 18维方差类型
    using NavStateT = NavState<S>;                  // 整体名义状态变量类型

    struct Options {
        Options() = default;

        /// IMU 测量与零偏参数 Q阵参数
        double imu_dt_ = 0.04;  // IMU测量间隔
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        double gyro_var_ = 1e-5;       // 陀螺测量标准差
        double acce_var_ = 1e-2;       // 加计测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差


        /// RTK 观测参数 R阵参数
        double gnss_pos_noise_ = 5.0;                   // GNSS位置噪声
        double gnss_height_noise_ = 1.0;                // GNSS高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声

        //手机安装角参数
        double phone_roll_install_ = 0.0 * math::kDEG2RAD;
        double phone_pitch_install_ = (90 + (-19.549240)) * math::kDEG2RAD;
        double phone_heading_install_ = -1.584286 * math::kDEG2RAD;

        /// 时间延迟补偿参数
        bool enable_time_compensation_ = false;  // 是否启用时间补偿
        double fixed_time_delay_ = 0.2;         // 固定时间延迟（秒，正值表示IMU滞后于GNSS）

        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新陀螺bias
        bool update_bias_acce_ = true;  // 是否更新加计bias
    };

    /**
     * 初始零偏取零
     */
    ESKF(Options option = Options()) : options_(option) { BuildNoise(option); BuildPhoneInstallMatrix(); }

    /**
     * 设置初始条件
     * @param options 噪声项配置
     * @param init_bg 初始零偏 陀螺
     * @param init_ba 初始零偏 加计
     * @param gravity 重力
     */
    void SetInitialConditions(Options options, const VecT& init_bg, const VecT& init_ba,
                              const VecT& gravity = VecT(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
        BuildPhoneInstallMatrix();
    }

    /// 使用IMU递推
    bool Predict(const IMU& imu);

    /// 使用GPS观测
    bool ObserveGps(const GNSS& gnss);

    /// 新增：仅观测位置，不观测航向
    bool ObservePositionOnly(const GNSS& gnss);

    /**
     * 使用SE3进行观测
     * @param pose  观测位姿
     * @param trans_noise 平移噪声
     * @param ang_noise   角度噪声
     * @return
     */
    bool ObserveSE3(const SE3& pose, double trans_noise = 3.0, double ang_noise = 3.0 * math::kDEG2RAD);

    /// 新增：仅观测位置部分
    bool ObservePositionOnly(const SE3& pose, double trans_noise = 3.0);

    /// accessors
    /// 获取全量状态
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// 获取SE3 状态
    SE3 GetNominalSE3() const { return SE3(R_, p_); }

    /// 设置状态X
    void SetX(const NavStated& x, const Vec3d& grav) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }

    /// 设置协方差
    void SetCov(const Mat18T& cov) { cov_ = cov; }

    /// 获取重力
    Vec3d GetGravity() const { return g_; }

    /// 获取当前时间补偿设置
    double GetTimeCompensation() const {
        return options_.enable_time_compensation_ ? options_.fixed_time_delay_ : 0.0;
    }

    /// 动态设置时间补偿参数
    void SetTimeCompensation(bool enable, double delay = 0.2) {
        options_.enable_time_compensation_ = enable;
        options_.fixed_time_delay_ = delay;
        
        LOG(INFO) << "Time compensation " << (enable ? "ENABLED" : "DISABLED") 
                  << ", delay = " << delay << "s";
    }

void SaveCovariance(std::ofstream& cov_file) const {
    cov_file << std::setprecision(18) << current_time_ << " ";
    
    // 保存18个对角元素
    for (int i = 0; i < 18; ++i) {
        cov_file << std::setprecision(9) << cov_(i, i) << " ";
    }
    cov_file << std::endl;
}

double GetCurrentHeading() const {
    return atan2(R_.matrix()(1, 0), R_.matrix()(0, 0));
}

double ComputeLateralResidual(const Vec3d& utm_residual) const {
    double heading = GetCurrentHeading();
    double dis_e = utm_residual.x(); // 东向残差
    double dis_n = utm_residual.y(); // 北向残差
    return dis_e * cos(heading) - dis_n * sin(heading);
}
   private:

    void Euler2Cbn(double roll, double pitch, double heading, Mat3T &Cbn) {
        Mat3T C1, C2, C3, Cnb;
        
        // 绕X轴旋转roll角
        C1(0, 0) = cos(roll);  C1(0, 1) = 0;          C1(0, 2) = -sin(roll);
        C1(1, 0) = 0;          C1(1, 1) = 1;          C1(1, 2) = 0;
        C1(2, 0) = sin(roll);  C1(2, 1) = 0;          C1(2, 2) = cos(roll);
        
        // 绕Y轴旋转pitch角
        C2(0, 0) = 1;          C2(0, 1) = 0;           C2(0, 2) = 0;
        C2(1, 0) = 0;          C2(1, 1) = cos(pitch);  C2(1, 2) = sin(pitch);
        C2(2, 0) = 0;          C2(2, 1) = -sin(pitch); C2(2, 2) = cos(pitch);
        
        // 绕Z轴旋转heading角
        C3(0, 0) = cos(heading); C3(0, 1) = -sin(heading); C3(0, 2) = 0;
        C3(1, 0) = sin(heading); C3(1, 1) = cos(heading);  C3(1, 2) = 0;
        C3(2, 0) = 0;           C3(2, 1) = 0;              C3(2, 2) = 1;
        
        // 计算转换矩阵
        Cnb = C1 * C2 * C3;
        Cbn = Cnb.transpose();
    }

    void BuildPhoneInstallMatrix() {
        // 计算手机到车体的转换矩阵
        Euler2Cbn(options_.phone_roll_install_, 
                    options_.phone_pitch_install_, 
                    options_.phone_heading_install_, 
                    C_phone_to_body_);
    }

    IMU ApplyPhoneInstallCorrection (const IMU& imu) const {
        IMU corrected_imu = imu;
        VecT body_acce = C_phone_to_body_ * imu.acce_;
        VecT body_gyro = C_phone_to_body_ * imu.gyro_;

        double body_z_accel = body_acce[2];
        // LOG(INFO) << "Z轴加速度: " << body_z_accel << " m/s² (理论值应接近±9.8)";

        corrected_imu.acce_ = body_acce;
        corrected_imu.gyro_ = body_gyro;

        return corrected_imu;
    }

    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;

        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;


        // 设置GNSS状态
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

    /// 更新名义状态变量，重置error state
    void UpdateAndReset() {
        //更新名义状态
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }

        g_ += dx_.template block<3, 1>(15, 0);

        //协方差投影
        ProjectCov();
        //重置误差状态
        dx_.setZero();
    }

    /// 对P阵进行投影，参考式(3.63)
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

    IMU ApplyTimeCompensation(const IMU& imu) const {
        if (!options_.enable_time_compensation_) {
            return imu; 
        }
        
        IMU compensated_imu = imu;
        // 正的time_delay表示IMU滞后于GNSS，所以要给IMU时间戳加上延迟
        compensated_imu.timestamp_ += options_.fixed_time_delay_;
        
        return compensated_imu;
    }

    /// 成员变量
    double current_time_ = 0.0;  // 当前时间

    /// 名义状态
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3 R_;
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();
    VecT g_{0, 0, -9.8};

    /// 误差状态
    Vec18T dx_ = Vec18T::Zero();

    /// 协方差阵
    Mat18T cov_ = Mat18T::Identity();

    /// 噪声阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    /// 标志位
    bool first_gnss_ = true;  // 是否为第一个gnss数据

    Mat3T C_phone_to_body_ = Mat3T::Identity();

    /// 配置项
    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    // assert(imu.timestamp_ >= current_time_);

    //应用手机安装角补偿
    IMU corrected_imu = ApplyPhoneInstallCorrection(imu);

    // 应用时间补偿
    IMU compensated_imu = ApplyTimeCompensation(corrected_imu);

    double dt = compensated_imu.timestamp_ - current_time_;

   if (dt < 0) {
        // IMU时间早于系统时间，跳过（GPS延迟导致）
        LOG(INFO) << "skip early imu: dt = " << dt;
        return false;
    }
    
    if (dt > (5 * options_.imu_dt_)) {
        // 时间间隔不对，可能是第一个IMU数据，没有历史信息
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = compensated_imu.timestamp_;
        return false;
    }

    // nominal state 递推
    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (compensated_imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    VecT new_v = v_ + R_ * (compensated_imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((compensated_imu.gyro_ - bg_) * dt);

    //状态更新
    R_ = new_R;
    v_ = new_v;
    p_ = new_p;
    // 其余状态维度不变

    // error state 递推
    // 计算运动过程雅可比矩阵 F，见(3.47)
    // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
    Mat18T F = Mat18T::Identity();                                                 // 主对角线
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // p 对 v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(compensated_imu.acce_ - ba_) * dt;  // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v 对 ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v 对 g
    F.template block<3, 3>(6, 6) = SO3::exp(-(compensated_imu.gyro_ - bg_) * dt).matrix();     // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta 对 bg

    // mean and cov prediction
    dx_ = F * dx_;  // 这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
    cov_ = F * cov_.eval() * F.transpose() + Q_; //协方差传播
    current_time_ = compensated_imu.timestamp_;
    return true;
}


template <typename S>
bool ESKF<S>::ObserveGps(const GNSS& gnss) {
    /// GNSS 观测的修正 观测更新
    
    // const double TIME_TOLERANCE = 0.2;
    // if (processed_gnss.unix_time_ < current_time_ - TIME_TOLERANCE) {
    //     return false;
    // }

    //首次GNSS的话直接设置初始位姿
    if (first_gnss_) {
        double initial_yaw_rad = atan2(gnss.utm_pose_.so3().matrix()(1, 0), 
                                      gnss.utm_pose_.so3().matrix()(0, 0));
        double initial_yaw_deg = initial_yaw_rad * 180.0 / M_PI;
        if (initial_yaw_deg < 0) initial_yaw_deg += 360.0;
        
        LOG(INFO) << "ESKF初始航向: " << initial_yaw_deg << "°";

        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }

    if (!gnss.heading_valid_) {
        LOG(WARNING) << "GPS航向数据无效, 跳过观测更新";
        return false;
    }
    ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_, options_.gnss_ang_noise_);
    // current_time_ = std::max(current_time_, processed_gnss.unix_time_);

    return true;
}

template <typename S>
bool ESKF<S>::ObservePositionOnly(const GNSS& gnss) {
    /// 仅位置观测，不观测航向
    
    //首次GNSS的话直接设置初始位姿
    if (first_gnss_) {
        double initial_yaw_rad = atan2(gnss.utm_pose_.so3().matrix()(1, 0), 
                                      gnss.utm_pose_.so3().matrix()(0, 0));
        double initial_yaw_deg = initial_yaw_rad * 180.0 / M_PI;
        if (initial_yaw_deg < 0) initial_yaw_deg += 360.0;
        
        LOG(INFO) << "ESKF初始航向: " << initial_yaw_deg << "°";

        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }

    // 只观测位置部分
    ObservePositionOnly(gnss.utm_pose_, options_.gnss_pos_noise_);
    return true;
}


template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    /// 既有旋转，也有平移
    /// 观测状态变量中的p, R，H为6x18，其余为零

    //1. 观测模型雅可比矩阵H
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // P部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R部分（3.66)

    // 卡尔曼增益和更新过程
    //2. 观测噪声协方差矩阵R
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    
    //3. 卡尔曼增益计算K
    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // 更新x和cov
    
    //4. 观测残差计算
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);          // 平移部分
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // 旋转部分(3.67)
    
    //清除对横滚roll、俯仰pitch的观测残差
    innov[3] = 0.0;
    innov[4] = 0.0;

    //5. 状态更新
    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}

template <typename S>
bool ESKF<S>::ObservePositionOnly(const SE3& pose, double trans_noise) {
    /// 仅观测位置，H为3x18矩阵
    
    //1. 观测模型雅可比矩阵H - 只有位置部分
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // 只有P部分

    // 卡尔曼增益和更新过程
    //2. 观测噪声协方差矩阵R - 只有位置噪声
    Vec3d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise;
    Mat3T V = noise_vec.asDiagonal();
    
    //3. 卡尔曼增益计算K
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    //4. 观测残差计算 - 只有位置部分
    Vec3d innov = pose.translation() - p_;

    //5. 状态更新
    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_ESKF_HPP