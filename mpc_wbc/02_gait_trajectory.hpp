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
    // 摆动抬脚高度 H [m]。0.06 → 0.10：实测开环（位置环）下 0.06 只能得到 ~3–5 cm 的实际
    // 净空（软接触下沉 + 关节跟踪滞后各吃掉一部分），摆动脚会刮地、把身体推着走；提到 0.10 后
    // 稳态速度从命令的 0.65 倍升到 0.95 倍、足端滑移从 0.082 降到 0.052 m/s（见 docs/04 §7 第 6 项）。
    double swing_height = 0.10;

    // 速度命令上限（把归一化 [-1,1] 映射到物理单位）。
    // 三段式设计目标见 docs/00：vx 0.60 / vy 0.35 / wz 0.50（方案 B 包络）。
    // 当前实现只有一组 T/d/H，直接按高速上限运行会用错周期，因此这里保持
    // 低速段安全上限；实现分段参数选择并通过验收后再放开。
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

/// 三段式步态参数：周期、占空比、抬腿高度按速度段独立整定。
/// 主运动方向按 |vx|/max_vx、|vy|/max_vy、|wz|/max_wz 的最大分量判定；
/// 段边界附近做平滑插值，避免步态参数跳变。
struct SegmentedGaitParams {
    GaitParams low;
    GaitParams mid;
    GaitParams high;

    // 全局归一化上限（只用于判定主运动方向，不改变命令单位）。
    double max_vx = 0.60;  // m/s
    double max_vy = 0.35;  // m/s
    double max_wz = 0.50;  // rad/s

    // 主运动方向的分段边界（物理单位）。
    double vx_low_mid = 0.20;   // m/s
    double vx_mid_high = 0.35;  // m/s
    double vy_low_mid = 0.15;   // m/s
    double vy_mid_high = 0.22;  // m/s
    double wz_low_mid = 0.20;   // rad/s
    double wz_mid_high = 0.30;  // rad/s

    // 边界插值带宽（物理单位）；0 表示直接切换。
    // 带宽从 0.02/0.015/0.02 放宽到 0.10/0.075/0.10：减速时命令会从高速段一路穿过
    // 中速段落到低速段，周期 0.44 → 1.05 → 1.20 逐级拉长；窄带宽下这是一步内的
    // 跳变（v=0.35 m/s 时足端行程 dp=v·T·d 从 0.096 直接翻到 0.179 m），实测三轴
    // 急停倾角冲击 13°。放宽后同一个过渡分散到 0.10 m/s 的命令区间，倾角降到 5.2°，
    // 稳态选段不受影响（验收工况的主轴分量都在纯段区间内）。
    double blend_vx = 0.10;   // m/s
    double blend_vy = 0.075;  // m/s
    double blend_wz = 0.10;   // rad/s

    /// 开环基线 openloop-baseline-v1 的三段参数（docs/00 §3）。
    /// 位置环与 MPC/WBC 共用这一份，避免两条路径跑不同步态导致结论不可比。
    static SegmentedGaitParams Baseline() {
        SegmentedGaitParams s;
        s.low = s.mid = s.high = GaitParams();
        s.low.period = 1.20;
        s.low.duty = 0.72;
        s.low.swing_height = 0.07;
        s.mid.period = 0.75;
        s.mid.duty = 0.70;
        s.mid.swing_height = 0.07;
        s.high.period = 0.44;
        s.high.duty = 0.625;
        s.high.swing_height = 0.07;
        for (GaitParams* p : {&s.low, &s.mid, &s.high}) {
            p->max_vx = s.max_vx;
            p->max_vy = s.max_vy;
            p->max_wz = s.max_wz;
        }
        return s;
    }

    static double SmoothStep01(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    static GaitParams Blend(const GaitParams& a, const GaitParams& b, double alpha) {
        alpha = std::max(0.0, std::min(1.0, alpha));
        GaitParams r = a;
        r.period = a.period + alpha * (b.period - a.period);
        r.duty = a.duty + alpha * (b.duty - a.duty);
        r.swing_height = a.swing_height + alpha * (b.swing_height - a.swing_height);
        r.max_vx = std::max(a.max_vx, b.max_vx);
        r.max_vy = std::max(a.max_vy, b.max_vy);
        r.max_wz = std::max(a.max_wz, b.max_wz);
        r.start_threshold = std::max(a.start_threshold, b.start_threshold);
        r.stop_threshold = std::max(a.stop_threshold, b.stop_threshold);
        // phase_offset 必须一致；默认沿用 a。
        return r;
    }

    static GaitParams SelectAxis(double s, double b1, double b2, double blend,
                                 const GaitParams& low, const GaitParams& mid,
                                 const GaitParams& high) {
        const double h = 0.5 * std::max(1e-9, blend);
        if (s <= b1 - h) return low;
        if (s < b1 + h)
            return Blend(low, mid, SmoothStep01((s - (b1 - h)) / (2.0 * h)));
        if (s <= b2 - h) return mid;
        if (s < b2 + h)
            return Blend(mid, high, SmoothStep01((s - (b2 - h)) / (2.0 * h)));
        return high;
    }

    /// 选择当前命令对应的步态参数（含边界插值）。
    GaitParams Select(const GaitCommand& cmd) const {
        const double ax = std::fabs(cmd.vx) / std::max(1e-9, max_vx);
        const double ay = std::fabs(cmd.vy) / std::max(1e-9, max_vy);
        const double aw = std::fabs(cmd.wz) / std::max(1e-9, max_wz);
        if (ax >= ay && ax >= aw)
            return SelectAxis(std::fabs(cmd.vx), vx_low_mid, vx_mid_high, blend_vx,
                              low, mid, high);
        if (ay >= aw)
            return SelectAxis(std::fabs(cmd.vy), vy_low_mid, vy_mid_high, blend_vy,
                              low, mid, high);
        return SelectAxis(std::fabs(cmd.wz), wz_low_mid, wz_mid_high, blend_wz,
                          low, mid, high);
    }

    /// 返回 0=低速、1=中速、2=高速（不含边界插值，用于日志/选段）。
    int SelectIndex(const GaitCommand& cmd) const {
        const double ax = std::fabs(cmd.vx) / std::max(1e-9, max_vx);
        const double ay = std::fabs(cmd.vy) / std::max(1e-9, max_vy);
        const double aw = std::fabs(cmd.wz) / std::max(1e-9, max_wz);
        double s = 0.0, b1 = 0.0, b2 = 0.0;
        if (ax >= ay && ax >= aw) {
            s = std::fabs(cmd.vx);
            b1 = vx_low_mid;
            b2 = vx_mid_high;
        } else if (ay >= aw) {
            s = std::fabs(cmd.vy);
            b1 = vy_low_mid;
            b2 = vy_mid_high;
        } else {
            s = std::fabs(cmd.wz);
            b1 = wz_low_mid;
            b2 = wz_mid_high;
        }
        if (s <= b1) return 0;
        if (s <= b2) return 1;
        return 2;
    }
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
    explicit GaitTrajectory(const GaitParams& params = GaitParams(),
                            const SegmentedGaitParams* schedule = nullptr)
        : params_(params), schedule_(schedule) {
        InitNominal();
    }

    const GaitParams& params() const { return params_; }
    void set_params(const GaitParams& p) { params_ = p; }

    /// 绑定三段式参数表；nullptr 表示使用固定 params_。
    void SetSchedule(const SegmentedGaitParams* schedule) { schedule_ = schedule; }

    /// 切换参数时保持当前步态相位连续，避免周期 T 改变导致相位跳变。
    void SetParamsAt(double t, const GaitParams& p) {
        if (stepping_ && params_.period > 1e-9 && p.period > 1e-9) {
            const double phase = Wrap01((t - t_start_) / params_.period);
            t_start_ = t - phase * p.period;
        }
        params_ = p;
    }

    /// 名义足位（机体系，球心），由默认站立角 FK 得到；12 维，按 FL,FR,HL,HR 的 xyz 排列。
    const Vec12d& nominal_feet() const { return nominal_feet_; }

    /// 用给定关节角重设名义足位（用于非默认站姿）。
    void SetNominalFromJointAngles(const Vec12d& q) {
        for (int leg = 0; leg < kNumLegs; ++leg)
            nominal_feet_.segment<3>(3 * leg) = model_.FootPositionBody(leg, q.segment<3>(3 * leg));
    }

    /// 标称足位整体平移（机体系）。用于按整机质心配平：官方模型质心在髋中心后方 7.2 mm，
    /// 而默认站立角的标称落足中心在髋中心前方 6.1 mm，两者相差 13.3 mm，后腿长期偏载。
    void ShiftNominal(const Vec3d& delta) {
        for (int leg = 0; leg < kNumLegs; ++leg)
            nominal_feet_.segment<3>(3 * leg) += delta;
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

    /// 归一化速度：各轴相对其上限的比值取最大（用于起步/停步滞回）。
    double NormalizedSpeed(const GaitCommand& c) const {
        const double vx = std::fabs(c.vx) / std::max(1e-9, params_.max_vx);
        const double vy = std::fabs(c.vy) / std::max(1e-9, params_.max_vy);
        const double w = std::fabs(c.wz) / std::max(1e-9, params_.max_wz);
        return std::max({vx, vy, w});
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
        if (schedule_ != nullptr) SetParamsAt(t, schedule_->Select(cmd));
        const double ns = NormalizedSpeed(cmd);
        if (!stepping_) {
            if (ns > params_.start_threshold) {
                stepping_ = true;
                t_start_ = t;  // θ=0 时四足均处于支撑相，可从静立平稳起走
            }
        } else if (ns < params_.stop_threshold) {
            // 软停步：命令已归零，但不立即退出步态。若中途退出，足端目标会从
            // 当前相位位置瞬间拉回标称位（低速段步幅可达 0.17 m，实测倾角冲击
            // 13°）。等相位刚过 θ=0 时四足同时处于支撑相且足端正好在标称位，
            // 此时退出没有任何位置跳变。代价是多走完当前一步（≤1 个周期）。
            const double theta = Wrap01((t - t_start_) / params_.period);
            if (theta < 0.01) stepping_ = false;
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
    const SegmentedGaitParams* schedule_ = nullptr;
    Lite3Model model_;
    Vec12d nominal_feet_ = Vec12d::Zero();
    bool stepping_ = false;
    double t_start_ = 0.0;
};

}  // namespace mpc_wbc
