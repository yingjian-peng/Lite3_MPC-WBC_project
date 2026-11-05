/*===============================================================================================
 * @file 01_state_estimator.hpp
 * @brief 基于支撑足零速假设估计 Lite3 基座速度、位置和高度
 * @details 输入 IMU 姿态/角速度、关节角/角速度和计划支撑标志；输出世界系基座状态及有效性。
 * 零速假设：v_foot_world≈0，通过足端雅可比反解每条支撑腿给出的基座速度，再筛选一致候选。
 * 平地假设：地面 z=0，由球心运动学和球足半径恢复基座高度；水平位置用速度短时积分。
 * 姿态直接采用 IMU；acc_body 保留在输入中，当前不参与融合。没有可靠支撑或时间异常时 valid=false。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include "00_lite3_model.hpp"                       // 统一坐标/关节约定及 Lite3 运动学、动力学模型

#include <algorithm>                                // 最值、夹取和排序等标准算法
#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <cmath>                                    // 三角函数、绝对值和其他数学函数
#include <limits>                                   // 数值搜索使用的极值常量
#include <vector>                                   // 随支撑数量或预测时域变化的动态数组



namespace mpc_wbc {

//===============================================================================================
// @brief 状态估计器的几何、时间和异常候选筛选参数
//===============================================================================================
struct EstimatorParams {
    double foot_radius = 0.022;                                 // 球足半径 [m]，应与模型和仿真一致
    double support_vel_tol = 0.30;                              // 各腿速度候选相对代表候选的最大一致残差 [m/s]
    double max_dt = 0.05;                                       // 初始化后允许的最大采样间隔 [s]，超限则本帧无效
    double height_filter_tau = 0.0;                             // 高度一阶低通时间常数 [s]；非正时直接采用运动学高度
    double max_accel = 30.0;                                    // 速度候选门限与全局速度变化率上限 [m/s²]；超限时限幅跟随，避免冻结旧估计
};



//===============================================================================================
// @brief 状态估计器单帧输入，坐标与关节排列沿用 00_lite3_model.hpp
//===============================================================================================
struct EstimatorInput {
    double t = 0.0;                                             // 测量时间戳 [s]，使用仿真接口时间
    Vec3d rpy = Vec3d::Zero();                                  // IMU 的 [roll,pitch,yaw] [rad]，采用 ZYX 约定
    Vec3d omega_body = Vec3d::Zero();                           // IMU 机体系角速度 [rad/s]
    Vec3d acc_body = Vec3d::Zero();                             // IMU 机体系加速度 [m/s²]，当前不参与估计
    Vec12d q = Vec12d::Zero();                                  // 12 个关节角 [rad]，按四腿各三个关节排列
    Vec12d dq = Vec12d::Zero();                                 // 12 个关节角速度 [rad/s]
    std::array<bool, kNumLegs> contact = {{true, true, true, true}};  // 计划支撑标志，顺序 FL/FR/HL/HR；不是力传感器触地测量
};



//===============================================================================================
// @brief 状态估计结果及有效性，供 MPC 和 WBC 计算使用
//===============================================================================================
struct StateEstimate {
    Vec3d base_pos_w = Vec3d::Zero();                           // 基座世界系位置 [m]；xy 由速度积分，z 由支撑足恢复
    Vec3d base_vel_w = Vec3d::Zero();                           // 基座世界系线速度 [m/s]
    Mat3d R_WB = Mat3d::Identity();                             // 机体系向量到世界系的旋转矩阵
    Vec3d omega_body = Vec3d::Zero();                           // 直接透传的 IMU 机体系角速度 [rad/s]
    double height = 0.0;                                        // 基座高度 [m]，等于 base_pos_w.z()
    double confidence = 0.0;                                    // 有效帧的置信度 [0,1]；无效时为 0，触发速度限幅时降低
    int num_support = 0;                                        // 通过一致性筛选后参与融合的支撑腿数量
    bool valid = false;                                         // 时间、支撑高度和速度一致性检查是否全部通过
};



//===============================================================================================
// @brief 平地运动学状态估计器：支撑腿筛选、速度融合和高度恢复
//===============================================================================================
class StateEstimator 
{
public:

    //------------------------------------------------------------------------------------
    // @brief 保存估计配置，并将球足半径同步到内部模型
    //------------------------------------------------------------------------------------
    explicit StateEstimator(const EstimatorParams& params = EstimatorParams()) : params_(params) {
        model_.geo.foot_radius = params_.foot_radius;
    }

    //------------------------------------------------------------------------------------
    // @brief 读取当前估计参数，不修改估计器状态
    //------------------------------------------------------------------------------------
    const EstimatorParams& params() const { return params_; }

    //------------------------------------------------------------------------------------
    // @brief 清空估计状态并设置初始时间和位置
    // @param base_pos_w0 初始世界系位置 [m]；z 会在有可靠支撑时由运动学重新恢复。
    //------------------------------------------------------------------------------------
    void Reset(double t, const Vec3d& base_pos_w0 = Vec3d::Zero()) {
        state_ = StateEstimate();
        state_.base_pos_w = base_pos_w0;
        last_t_ = t;
        initialized_ = false;
    }

    //------------------------------------------------------------------------------------
    // @brief 读取最近一帧缓存的状态估计结果
    //------------------------------------------------------------------------------------
    const StateEstimate& state() const { 
        return state_; 
    }

    //------------------------------------------------------------------------------------
    // @brief 处理一帧测量，返回更新后的状态和有效性
    // @details 按时间检查→逐腿速度反解→候选筛选→速度限幅→高度恢复→位置更新执行。
    // @return 无效帧仍包含缓存或候选状态，调用者应先检查 valid 再使用其控制含义。
    //------------------------------------------------------------------------------------
    StateEstimate Update(const EstimatorInput& in)
    {
        // 步骤 1：由 IMU 更新姿态，直接保存机体系角速度
        const Mat3d R = RpyToRotation(in.rpy);
        state_.R_WB = R;
        state_.omega_body = in.omega_body;

        // 步骤 2：检查时间增量；初始化后的回退、重帧或长间隔使本帧无效
        double dt = in.t - last_t_;
        const bool time_ok = initialized_ ? (dt > 0.0 && dt <= params_.max_dt) : true;
        if (dt <= 0.0) dt = 0.0;

        // 步骤 3：仅对计划支撑腿，用 v_foot=0 反解基座世界系速度候选
        const Mat12x18d J = model_.FootJacobianWorld(in.q, R);
        std::vector<Vec3d> cand;
        std::vector<int> cand_leg;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!in.contact[leg]) continue;
            const int row = 3 * leg;
            const Vec3d v_leg = -(J.block<3, 3>(row, 3) * in.omega_body + J.block<3, 12>(row, 6) * in.dq);
            cand.push_back(v_leg);
            cand_leg.push_back(leg);
        }

        // 步骤 4：优先保留接近上一估计的逐腿候选；若全被拒绝，保留原候选交给后续限幅
        if (initialized_ && dt > 0.0 && !cand.empty()) {
            const double gate = params_.max_accel * dt;
            std::vector<Vec3d> kept;
            std::vector<int> kept_leg;
            for (size_t i = 0; i < cand.size(); ++i) {
                if ((cand[i] - state_.base_vel_w).norm() <= gate) {
                    kept.push_back(cand[i]);
                    kept_leg.push_back(cand_leg[i]);
                }
            }
            if (!kept.empty()) {
                cand.swap(kept);
                cand_leg.swap(kept_leg);
            }
        }

        // 步骤 5：选取到其他候选距离总和最小的代表候选，平均其一致子集
        const int n_total = static_cast<int>(cand.size());
        std::vector<int> consistent;
        Vec3d base_vel = state_.base_vel_w;
        bool vel_ok = false;
        double spread = 0.0;
        if (!cand.empty()) {
            int best = 0;
            double best_cost = std::numeric_limits<double>::max();
            for (size_t i = 0; i < cand.size(); ++i) {
                double cost = 0.0;
                for (size_t j = 0; j < cand.size(); ++j) cost += (cand[i] - cand[j]).norm();
                if (cost < best_cost) {
                    best_cost = cost;
                    best = static_cast<int>(i);
                }
            }
            Vec3d sum = Vec3d::Zero();
            for (size_t i = 0; i < cand.size(); ++i) {
                const double e = (cand[i] - cand[best]).norm();
                if (e <= params_.support_vel_tol) {
                    sum += cand[i];
                    consistent.push_back(cand_leg[i]);
                    spread = std::max(spread, e);
                }
            }
            if (!consistent.empty()) {
                base_vel = Vec3d(sum / static_cast<double>(consistent.size()));
                vel_ok = true;
            }
        }
        state_.num_support = static_cast<int>(consistent.size());

        // 步骤 6：把融合速度的变化量限制为 max_accel*dt；不丢弃整帧，避免估计冻结
        bool gate_rejected = false;
        if (initialized_ && vel_ok && dt > 0.0) {
            const Vec3d jump = base_vel - state_.base_vel_w;
            const double max_jump = params_.max_accel * dt;
            if (jump.norm() > max_jump) {
                base_vel = state_.base_vel_w + jump.normalized() * max_jump;
                gate_rejected = true;
            }
        }

        // 步骤 7：用一致支撑腿恢复平地高度：height = foot_radius - (R_WB*p_foot_body).z
        double height_sum = 0.0;
        for (int leg : consistent) {
            const Vec3d p_b = model_.FootPositionBody(leg, in.q.segment<3>(3 * leg));
            height_sum += params_.foot_radius - (R * p_b).z();
        }
        const int h_n = static_cast<int>(consistent.size());
        const bool height_ok = (h_n > 0);

        // 步骤 8：可信速度用于积分 xy，高度可选一阶低通，最终保存本帧速度候选
        const bool vel_trust = time_ok && vel_ok;
        if (!initialized_) {
            if (height_ok) {
                state_.base_pos_w.z() = height_sum / h_n;
                initialized_ = true;
            }
            state_.base_vel_w = base_vel;
        } 
        else {
            if (vel_trust) state_.base_pos_w.head<2>() += base_vel.head<2>() * dt;
            if (height_ok) {
                const double h = height_sum / h_n;
                if (params_.height_filter_tau > 0.0 && dt > 0.0) {
                    const double a = std::min(1.0, dt / params_.height_filter_tau);
                    state_.base_pos_w.z() = (1.0 - a) * state_.base_pos_w.z() + a * h;
                } else {
                    state_.base_pos_w.z() = h;
                }
            }
            state_.base_vel_w = base_vel;
        }
        (void)vel_ok;
        state_.height = state_.base_pos_w.z();

        // 步骤 9：综合时间、高度和支撑一致性设置 valid，并给触发限幅的结果降低置信度
        const bool corroborated = (static_cast<int>(consistent.size()) >= 2) || (n_total == 1);
        state_.valid = time_ok && height_ok && vel_ok && initialized_ && corroborated;
        double conf = 0.0;
        if (state_.valid) {
            conf = std::min(1.0, static_cast<double>(h_n) / 2.0);
            if (spread > params_.support_vel_tol) {
                conf *= params_.support_vel_tol / spread;
            }
            if (gate_rejected) conf *= 0.3;  // 本帧速度是外推的，降低置信度
        }
        state_.confidence = std::max(0.0, std::min(1.0, conf));

        last_t_ = in.t;
        return state_;
    }


private:
    // 内部缓存：参数、运动学模型、最近估计及上一采样时间
    EstimatorParams params_;                                    // 本估计器的阈值、半径与滤波配置
    Lite3Model model_;                                          // 用于球心位置和雅可比计算的内部模型
    StateEstimate state_;                                       // 最近一次更新的估计结果；无效帧仍会保存部分候选量
    double last_t_ = 0.0;                                       // 上一输入时间戳 [s]
    bool initialized_ = false;                                  // 是否已经通过可靠支撑恢复过初始高度
};

}  // namespace mpc_wbc



