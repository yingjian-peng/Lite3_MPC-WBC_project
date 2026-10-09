/**
 * @file 02_gait_trajectory.hpp
 * @brief Lite3 平地 trot 步态与足端轨迹（S3）。
 *
 * 理论依据 docs/03 第 2 章：
 *   - 全局相位 θ(t) = frac((t-t0)/T)，腿相位 φ_i = frac(θ + δ_i)，支撑判据 c_i = φ_i < d；
 *   - 相内进度 ξ_i：支撑 φ/d，摆动 (φ-d)/(1-d)；
 *   - 支撑足在机体系内随身体前进而后扫；摆动足用五次曲线水平插值 + 余弦包络抬脚；
 *   - 支撑/摆动切换的足端位置连续。
 *
 * 机体系约定（与 docs/02 §3 一致）：x 前、y 左、z 上；足端目标是 FOOT 球足球心。
 *
 * 与文档的差异（本实现更贴合物理一致性，已在 docs/04 §8 记录）：
 *   - 步长用 stance 时长折算 Δp = T_stance·(v + ω×p_foot)，T_stance = T·d
 *     （docs/03 §2.2 用整个周期 T，且旋转项用髋位置；此处用名义足位更准确）。
 *   - 支撑相用机体系线性后扫（开环）。世界系锚点锁定需要基座位置/速度估计，留到 S5 之后。
 */
#pragma once

#include "00_lite3_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace mpc_wbc {

/// 步态参数（初值来自 docs/03 附录 C，全部待标定）。
struct GaitParams {
    double period = 0.60;        // 步态周期 T [s]
    double duty = 0.68;          // 支撑占空比 d
    double swing_height = 0.06;  // 摆动抬脚高度 H [m]

    // 速度命令上限（把归一化 [-1,1] 映射到物理单位）
    double max_vx = 0.20;  // m/s
    double max_vy = 0.15;  // m/s
    double max_wz = 0.30;  // rad/s

    // 行走/停步滞回阈值（归一化速度：各轴相对其上限的比值取最大）
    double start_threshold = 0.03;
    double stop_threshold = 0.015;

    // 相位偏移，顺序 FL, FR, HL, HR（对角小跑）
    std::array<double, kNumLegs> phase_offset = {{0.0, 0.5, 0.5, 0.0}};
};

/// 机体系速度命令。
struct GaitCommand {
    double vx = 0.0;  // m/s
    double vy = 0.0;  // m/s
    double wz = 0.0;  // rad/s（绕机身 z）
};

/// 单腿步态状态。
struct LegGaitState {
    bool contact = true;             // 支撑相
    double phase = 0.0;              // φ_i ∈ [0,1)
    double progress = 0.0;           // ξ_i ∈ [0,1]
    Vec3d foot_pos = Vec3d::Zero();  // 机体系目标（球心）
    Vec3d foot_vel = Vec3d::Zero();  // 机体系目标速度
};

class GaitTrajectory {
public:
    explicit GaitTrajectory(const GaitParams& params = GaitParams()) : params_(params) {
        InitNominal();
    }

    const GaitParams& params() const { return params_; }
    void set_params(const GaitParams& p) { params_ = p; }

    /// 名义足位（机体系，球心），由默认站立角 FK 得到；12 维，按 FL,FR,HL,HR 的 xyz 排列。
    const Vec12d& nominal_feet() const { return nominal_feet_; }

    /// 用给定关节角重设名义足位（用于非默认站姿）。
    void SetNominalFromJointAngles(const Vec12d& q) {
        for (int leg = 0; leg < kNumLegs; ++leg)
            nominal_feet_.segment<3>(3 * leg) = model_.FootPositionBody(leg, q.segment<3>(3 * leg));
    }

    /// 归一化 [-1,1] 命令 -> 物理单位（含死区）。
    static GaitCommand ScaleCommand(const Vec3d& normalized, const GaitParams& p,
                                    double deadband = 0.0) {
        auto scale = [deadband](double c, double limit) {
            if (std::fabs(c) < deadband) return 0.0;
            return std::max(-1.0, std::min(1.0, c)) * limit;
        };
        GaitCommand cmd;
        cmd.vx = scale(normalized(0), p.max_vx);
        cmd.vy = scale(normalized(1), p.max_vy);
        cmd.wz = scale(normalized(2), p.max_wz);
        return cmd;
    }

    /// 归一化速度：各轴相对上限的比值取最大。
    double NormalizedSpeed(const GaitCommand& c) const {
        const double v = std::hypot(c.vx, c.vy) / std::max(1e-9, params_.max_vx);
        const double w = std::fabs(c.wz) / std::max(1e-9, params_.max_wz);
        return std::max(v, w);
    }

    bool stepping() const { return stepping_; }
    double phase_start_time() const { return t_start_; }

    /// 复位到静立（所有足在名义位置）。
    void Reset(double t) {
        stepping_ = false;
        t_start_ = t;
    }

    /// 每控制周期调用一次：更新行走/停步滞回状态并返回四腿目标。
    std::array<LegGaitState, kNumLegs> Update(double t, const GaitCommand& cmd) {
        const double ns = NormalizedSpeed(cmd);
        if (!stepping_) {
            if (ns > params_.start_threshold) {
                stepping_ = true;
                t_start_ = t;  // θ=0 时四足均处于支撑相，可从静立平稳起走
            }
        } else if (ns < params_.stop_threshold) {
            stepping_ = false;
        }
        return Evaluate(t, cmd);
    }

    /// 只读求值（不改状态），用于日志/预测。
    std::array<LegGaitState, kNumLegs> Evaluate(double t, const GaitCommand& cmd) const {
        std::array<LegGaitState, kNumLegs> out;
        if (!stepping_) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                LegGaitState st;
                st.foot_pos = nominal_feet_.segment<3>(3 * leg);
                out[leg] = st;  // contact=true, vel=0
            }
            return out;
        }

        const double T = params_.period;
        const double d = params_.duty;
        const double t_stance = T * d;
        const double theta = GlobalPhase(t);

        for (int leg = 0; leg < kNumLegs; ++leg) {
            LegGaitState st;
            st.phase = Wrap01(theta + params_.phase_offset[leg]);
            st.contact = st.phase < d;

            const Vec3d p0 = nominal_feet_.segment<3>(3 * leg);
            // 步长向量：Δp = T_stance·(v_cmd + ω×p0_xy)，只含水平分量。
            const Vec3d dp(t_stance * (cmd.vx - cmd.wz * p0.y()),
                           t_stance * (cmd.vy + cmd.wz * p0.x()),
                           0.0);

            if (st.contact) {
                st.progress = st.phase / d;
                st.foot_pos = p0 + (0.5 - st.progress) * dp;
                st.foot_vel = -dp / t_stance;  // d/dt[(1/2-ξ)Δp], dξ/dt = 1/T_stance
            } else {
                st.progress = (st.phase - d) / (1.0 - d);
                const double xi = st.progress;
                st.foot_pos = p0 + (SwingCurve(xi) - 0.5) * dp;
                st.foot_pos.z() = p0.z() + SwingHeightCurve(xi, params_.swing_height);

                const double dxi_dt = 1.0 / (T * (1.0 - d));
                st.foot_vel.x() = dp.x() * SwingCurveDerivative(xi) * dxi_dt;
                st.foot_vel.y() = dp.y() * SwingCurveDerivative(xi) * dxi_dt;
                st.foot_vel.z() = SwingHeightDerivative(xi, params_.swing_height) * dxi_dt;
            }
            out[leg] = st;
        }
        return out;
    }

    /// 未来 N 步的支撑表（true=支撑），供 MPC/WBC 使用同一相位源。
    template <int N>
    std::array<std::array<bool, kNumLegs>, N> ContactSchedule(double t, double dt) const {
        std::array<std::array<bool, kNumLegs>, N> out{};
        for (int k = 0; k < N; ++k) {
            const double theta = stepping_ ? PhaseAt(t + k * dt) : 0.0;
            for (int leg = 0; leg < kNumLegs; ++leg)
                out[k][leg] = !stepping_ || (Wrap01(theta + params_.phase_offset[leg]) < params_.duty);
        }
        return out;
    }

    double GlobalPhase(double t) const { return stepping_ ? PhaseAt(t) : 0.0; }

    // --- 相内插值函数（docs/03 §2.2） ---
    /// 水平五次曲线 s(ξ)=10ξ³−15ξ⁴+6ξ⁵，s(0)=0, s(1)=1, s'(0)=s'(1)=0。
    static double SwingCurve(double xi) {
        return xi * xi * xi * (10.0 + xi * (-15.0 + 6.0 * xi));
    }
    static double SwingCurveDerivative(double xi) {
        return 30.0 * xi * xi * (1.0 - xi) * (1.0 - xi);  // 30ξ²−60ξ³+30ξ⁴
    }
    /// 竖直余弦包络 h(ξ)=H/2·(1−cos 2πξ)，h(0)=h(1)=0。
    static double SwingHeightCurve(double xi, double H) {
        return 0.5 * H * (1.0 - std::cos(2.0 * kPi * xi));
    }
    static double SwingHeightDerivative(double xi, double H) {
        return kPi * H * std::sin(2.0 * kPi * xi);
    }

private:
    void InitNominal() {
        SetNominalFromJointAngles(Lite3Model::DefaultStandJointAngles());
    }
    double PhaseAt(double t) const { return Wrap01((t - t_start_) / params_.period); }

    static double Wrap01(double x) {
        x -= std::floor(x);
        return x;
    }

    GaitParams params_;
    Lite3Model model_;
    Vec12d nominal_feet_ = Vec12d::Zero();
    bool stepping_ = false;
    double t_start_ = 0.0;
};

}  // namespace mpc_wbc
