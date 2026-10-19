/**
 * @file 01_state_estimator.hpp
 * @brief Lite3 平地状态估计（S5）。
 *
 * 估计方法与假设：
 *   - 本工程 `RobotInterface` 没有可用的基座线速度/位置与接触力，需要自行估计；
 *   - 平地上、可信支撑腿满足 v_foot^W ≈ 0，由此反解基座速度：
 *         v_foot_i^W = J_all,i · qvel = 0
 *       => v_B^W = −( J_all,i[:,3:6]·ω_B + J_all,i[:,6:18]·dq )
 *     对各支撑腿做稳健平均（剔除残差过大的腿）；
 *   - 基座高度由地面 z=0 与支撑足 FK 恢复：p_B.z = r − (R_WB·p_foot^B).z；
 *   - 水平位置用估计速度短时积分（世界系足端锚点更新留待后续版本）；
 *   - 姿态与角速度直接用 IMU；输出 valid/confidence，全摆动/丢包/不收敛时置为无效。
 *
 * 注意：`acc_body` 当前不参与融合（其是否含重力的约定需实测核对）。
 */
#pragma once

#include "00_lite3_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace mpc_wbc {

struct EstimatorParams {
    double foot_radius = 0.022;            // 球足半径
    double support_vel_tol = 0.30;         // 支撑腿零速候选的残差阈值 [m/s]
    double max_dt = 0.05;                  // 超过则视为丢包，标记无效 [s]
    double height_filter_tau = 0.0;        // >0 时对高度做一阶低通 [s]
};

struct EstimatorInput {
    double t = 0.0;
    Vec3d rpy = Vec3d::Zero();             // IMU 姿态（ZYX）
    Vec3d omega_body = Vec3d::Zero();      // IMU 角速度（机体系）
    Vec3d acc_body = Vec3d::Zero();        // 暂不融合
    Vec12d q = Vec12d::Zero();
    Vec12d dq = Vec12d::Zero();
    std::array<bool, kNumLegs> contact = {{true, true, true, true}};
};

struct StateEstimate {
    Vec3d base_pos_w = Vec3d::Zero();      // 世界系基座位置
    Vec3d base_vel_w = Vec3d::Zero();      // 世界系基座线速度
    Mat3d R_WB = Mat3d::Identity();
    Vec3d omega_body = Vec3d::Zero();
    double height = 0.0;                   // = base_pos_w.z
    double confidence = 0.0;               // 0..1
    int num_support = 0;
    bool valid = false;
};

class StateEstimator {
public:
    explicit StateEstimator(const EstimatorParams& params = EstimatorParams())
        : params_(params) {
        model_.geo.foot_radius = params_.foot_radius;
    }

    const EstimatorParams& params() const { return params_; }

    /// 复位，可给出初始世界位置（xy 会随后积分，z 由支撑足恢复）。
    void Reset(double t, const Vec3d& base_pos_w0 = Vec3d::Zero()) {
        state_ = StateEstimate();
        state_.base_pos_w = base_pos_w0;
        last_t_ = t;
        initialized_ = false;
    }

    const StateEstimate& state() const { return state_; }

    StateEstimate Update(const EstimatorInput& in) {
        const Mat3d R = RpyToRotation(in.rpy);
        state_.R_WB = R;
        state_.omega_body = in.omega_body;

        // 时间检查：丢包/回退
        double dt = in.t - last_t_;
        const bool time_ok = initialized_ ? (dt > 0.0 && dt <= params_.max_dt) : true;
        if (dt <= 0.0) dt = 0.0;

        // ---- 支撑腿零速反解基座速度（稳健选择：先取 medoid，再取一致子集）----
        const Mat12x18d J = model_.FootJacobianWorld(in.q, R);
        std::vector<Vec3d> cand;
        std::vector<int> cand_leg;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!in.contact[leg]) continue;
            const int row = 3 * leg;
            const Vec3d v_leg = -(J.block<3, 3>(row, 3) * in.omega_body +
                                  J.block<3, 12>(row, 6) * in.dq);
            cand.push_back(v_leg);
            cand_leg.push_back(leg);
        }

        // medoid：到其他候选取值距离之和最小者，避免单个误标接触的腿污染结果
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

        // ---- 支撑足恢复高度（只用一致支撑腿）----
        double height_sum = 0.0;
        for (int leg : consistent) {
            const Vec3d p_b = model_.FootPositionBody(leg, in.q.segment<3>(3 * leg));
            height_sum += params_.foot_radius - (R * p_b).z();
        }
        const int h_n = static_cast<int>(consistent.size());
        const bool height_ok = (h_n > 0);

        // ---- 位置更新（仅在速度可信时积分 xy）----
        const bool vel_trust = time_ok && vel_ok;
        if (!initialized_) {
            if (height_ok) {
                state_.base_pos_w.z() = height_sum / h_n;
                initialized_ = true;
            }
            state_.base_vel_w = base_vel;
        } else {
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

        // ---- 有效性与置信度 ----
        // 支撑候选需相互印证：一致子集 >=2，或本来就只有 1 条支撑腿时才可信。
        const bool corroborated = (static_cast<int>(consistent.size()) >= 2) || (n_total == 1);
        state_.valid = time_ok && height_ok && vel_ok && initialized_ && corroborated;
        double conf = 0.0;
        if (state_.valid) {
            conf = std::min(1.0, static_cast<double>(h_n) / 2.0);
            if (spread > params_.support_vel_tol) {
                conf *= params_.support_vel_tol / spread;
            }
        }
        state_.confidence = std::max(0.0, std::min(1.0, conf));

        last_t_ = in.t;
        return state_;
    }

private:
    EstimatorParams params_;
    Lite3Model model_;
    StateEstimate state_;
    double last_t_ = 0.0;
    bool initialized_ = false;
};

}  // namespace mpc_wbc
