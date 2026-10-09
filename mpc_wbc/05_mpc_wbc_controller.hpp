/**
 * @file 05_mpc_wbc_controller.hpp
 * @brief S8 控制器编排：估计 → 步态/足端轨迹 → SRBD-MPC → Weighted-QP WBC → 12×5 关节命令。
 *
 * 设计约束（docs/04 §3 S8）：
 *   - **无线程**：`Step()` 内部不做任何线程/锁/IO，只吃「一帧状态快照」，吐「一帧命令」；
 *     线程与快照发布由 `state_machine/mpc_control_state.hpp` 负责。
 *   - **不持有 RobotInterface**：上层把测量值填进 `ControllerInput`，把 `ControllerOutput`
 *     写进关节命令；这样离线 harness（MuJoCo app）与真机状态机共用同一套算法。
 *
 * 线程分工（S8 的实测结论）：
 *   - **1 kHz 路径**（`Step`）：估计 → 步态 → WBC → 命令。必须严格按周期跑，不能被打断。
 *   - **MPC 路径**（`BuildMpcRequest` + `SolveWrench`）：2-5 Hz 门控即可，但**耗时抖动大**
 *     （盒约束 QP 在退化点会跑到 10~30 ms）。实测把它放在 1 kHz 线程里会让 WBC 漏拍、
 *     机器人 4 s 后失稳（trot 段 tilt 极限环），因此状态机把它放到独立线程：
 *     1 kHz 线程只做 `BuildMpcRequest`（纯赋值，无求解），MPC 线程解完写回
 *     `SeqLockBox<MpcWrench>`，1 kHz 线程读最近一次的 wrench。
 *
 * 每个控制周期的数据流：
 *   IMU(q/dq/rpy/ω) ─┬─> StateEstimator ─> p_base / v_base（世界系）
 *                    ├─> GaitTrajectory ─> 支撑集 + 机体系足端目标
 *                    ├─> CompositeCom* ─> 质心位置/惯量 ─> SrbdPreviewMpc（20 Hz 门控）
 *                    │                                     └─> 净力/净力矩 → a_des, α_des
 *                    └─> WeightedWbc ─> q̈/f/τ
 *   τ_wbc ─> 反解 τ_ff = τ_wbc − [kp(q_des−q) + kd(dq_des−dq)] ─> 12×5 命令
 *
 * 低层执行律（仿真与真机一致）：τ_applied = kp(q_des−q) + kd(dq_des−dq) + τ_ff。
 * WBC 工作时 PD 只作兜底（默认缩到 0.5 倍），因此发给执行器的总力矩 ≈ τ_wbc；
 * WBC 未工作（起立/降级/故障）时 τ_ff=0，退化成普通位置环。
 */
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "00_lite3_model.hpp"
#include "01_state_estimator.hpp"
#include "02_gait_trajectory.hpp"
#include "03_srbd_mpc.hpp"
#include "04_weighted_wbc.hpp"

namespace mpc_wbc {

constexpr int kMaxContactPreview = 64;  // 支撑表一次最多向前看多少节点（须 ≥ SrbdMpcParams::N）

struct MpcWbcControllerParams {
    // ---- 起立与站立 ----
    Vec12d stand_q = Lite3Model::DefaultStandJointAngles();
    // 进入本状态前机器人已由 StandUpState 站好，这里的斜坡只是**平滑接管**（防切换瞬间跳变）；
    // 不要指望它把机器人从趴地姿势"起立"（实测会把机身顶翻）——起立属于 StandUpState。
    double stand_ramp_time = 0.30;   // 从进入时的关节角平滑插值到 stand_q [s]
    double stand_settle_time = 0.20;  // 斜坡结束后再保持位置环多久才使能 WBC [s]
    double com_height = 0.0;         // 期望质心高度 [m]；0 表示用站立构型自动算出
    // ---- 时序 ----
    double mpc_period = 0.05;        // MPC 重解周期 [s]（20 Hz；帧间复用上次期望 wrench）
    double max_dt = 0.05;            // 单帧时间间隔上限，超过视为丢包（只用命令、不推进估计）
    double wrench_max_age = 0.15;    // MPC wrench 快照最长可用时间 [s]，超过则退化为零期望加速度
    // 速度命令平滑：一阶低通 + 变化率限幅。
    // 实测：harness 用「0 → 0.15 m/s」阶跃命令会在 1~2 个步态周期内把机身 pitch 顶到 40°+ 而翻车；
    // 状态机路径的键盘命令天然是渐变的，连走 6 s「前进」pitch 始终 < 2°。所以命令必须先过滤。
    double cmd_tau = 0.25;           // 一阶低通时间常数 [s]（0 = 关闭）
    double cmd_slew_v = 0.8;         // 速度命令变化率上限 [m/s²]（0 = 关闭）
    double cmd_slew_w = 2.0;         // 偏航命令变化率上限 [rad/s²]
    // 质心高度命令的软启动变化率 [m/s]（0 = 关闭软启动，直接用 com_height）。
    // 实测：engage 时若高度命令与实测差 0.3 m，MPC 会立刻要求 ~30 m/s² 的竖直加速度，
    // 把 τ 顶到限幅附近、偶发顶翻机身。0.06 m/s 约 5 s 走完 0.3 m，起立过程平滑。
    double com_height_slew = 0.06;
    // ---- 低层 PD ----
    double kp_pos = 100.0;           // 位置环档位（docs/03 附录 C）
    double kd_pos = 2.5;
    double kp_scale_wbc = 0.5;       // WBC 工作时的 PD 缩放（参考工程推荐 1 kHz WBC + 0.4~0.5）
    double kd_scale_wbc = 0.5;
    // ---- 安全 ----
    double tilt_limit = 0.8;         // roll/pitch 超限 → 故障（rad）
    double wbc_enable_tilt = 0.35;   // 姿态超过该值不允许使能 WBC（rad）
    int max_consecutive_wbc_fail = 20;  // 连续 WBC 求解失败帧数上限 → 故障
    // WBC 使能滞回：估计短暂失效（规划支撑腿与实际不符）时不要一帧一帧地开关 WBC——
    // 切换会同时改变 PD 增益（×2）与 τ_ff（0 ↔ τ_wbc−pd），实测这种抖动比"撑着跑"更糟。
    int wbc_enable_hold_frames = 40;    // 已使能后，门关闭（估计失效/支撑数不足/姿态偏大）允许连续多少帧仍保持 WBC（1 kHz 下 40 ms）；姿态超 tilt_limit 判故障则立即失效
    bool swing_target_yaw_only = true;  // 摆动目标机体系→世界系只用实际偏航（防倾斜污染轨迹）
    // ---- 子模块参数 ----
    EstimatorParams estimator;
    GaitParams gait;
    SrbdMpcParams mpc;
    WbcParams wbc;
};

/// 一帧的完整输入快照（由上层的状态机或离线 harness 填充）。
struct ControllerInput {
    double t = 0.0;                        // 接口时间戳 [s]
    Vec3d rpy = Vec3d::Zero();             // IMU 姿态（ZYX，世界系）
    Vec3d omega_body = Vec3d::Zero();      // IMU 角速度（机体系）
    Vec3d acc_body = Vec3d::Zero();        // IMU 加速度（估计器暂不融合，仅诊断）
    Vec12d q = Vec12d::Zero();             // 关节角
    Vec12d dq = Vec12d::Zero();            // 关节角速度
    GaitCommand cmd;                       // 速度命令（物理单位，机体系）
};

/// MPC 线程的输入快照：1 kHz 线程填好、MPC 线程只读（不含任何 Eigen 动态分配）。
struct MpcRequest {
    double t = 0.0;
    Vec3d rpy = Vec3d::Zero();
    Vec3d omega_body = Vec3d::Zero();
    Vec12d q = Vec12d::Zero();
    Vec3d base_pos = Vec3d::Zero();
    Vec3d base_vel = Vec3d::Zero();
    Vec3d com_body = Vec3d::Zero();
    bool est_valid = false;
    int num_support = 0;
    GaitCommand cmd;
    double height_cmd = 0.0;           // 质心高度命令（由 1 kHz 线程软启动后给出）
    std::array<std::array<bool, kNumLegs>, kMaxContactPreview> contact{};
    int contact_steps = 0;
};

/// MPC 线程的输出：期望 wrench + 诊断（POD，可被 seqlock 安全拷贝）。
struct MpcWrench {
    bool valid = false;
    double t = 0.0;                    // 采样时刻
    Vec3d a_des_world = Vec3d::Zero();
    Vec3d alpha_des_body = Vec3d::Zero();
    std::array<Vec3d, kNumLegs> foot_force_ref{};
    Vec3d net_force = Vec3d::Zero();
    Vec3d net_moment = Vec3d::Zero();
    int status = -1;                   // -1=未跑 0=ok 1=stale 2=failed
    int reason = 0;                    // status==2 的原因：1=估计无效 2=支撑腿<1 3=MPC 无有效解
    int iters = 0;
    double qp_residual = 0.0;
    double solve_ms = 0.0;
};

/// 单生产者/单消费者快照箱（seqlock）：写者不阻塞，读者最多重试几次。
template <typename T>
class SeqLockBox {
public:
    void Write(const T& value) {
        const uint64_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);   // 奇 = 正在写
        std::atomic_thread_fence(std::memory_order_release);
        value_ = value;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_relaxed);   // 偶 = 写完
    }

    bool Read(T* out) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const uint64_t s0 = seq_.load(std::memory_order_relaxed);
            if (s0 & 1U) continue;                       // 写者正在写
            std::atomic_thread_fence(std::memory_order_acquire);
            *out = value_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s0) return true;
        }
        return false;
    }

    /// 读并返回序号（用于判断「有没有新 wrench」）。
    bool ReadWithSeq(T* out, uint64_t* seq_out) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const uint64_t s0 = seq_.load(std::memory_order_relaxed);
            if (s0 & 1U) continue;
            std::atomic_thread_fence(std::memory_order_acquire);
            *out = value_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s0) {
                if (seq_out) *seq_out = s0;
                return true;
            }
        }
        return false;
    }

private:
    std::atomic<uint64_t> seq_{0};
    T value_{};
};

/// 一帧的输出：12×5 关节命令 + 诊断。
struct ControllerOutput {
    enum class Mode { kRamp, kStand, kWalk, kFault };

    /// 12×5 关节命令，列序 = [kp, q_des, kd, dq_des, τ_ff]（与 RobotInterface::SetJointCommand 一致）
    Eigen::Matrix<float, 12, 5> joint_cmd = Eigen::Matrix<float, 12, 5>::Zero();
    Mode mode = Mode::kRamp;
    bool wbc_active = false;   // 本帧的 τ_ff 是否来自 WBC
    bool wbc_stale = false;    // WBC 沿用旧解（kStale）
    bool fault = false;        // 触发故障（姿态/连续求解失败）→ 上层应切到阻尼状态

    // 诊断量
    double dt = 0.0;
    Vec12d q_des = Vec12d::Zero();
    Vec12d tau_wbc = Vec12d::Zero();
    Vec12d tau_pd = Vec12d::Zero();
    std::array<Vec3d, kNumLegs> foot_force{};   // WBC 逐脚力（世界系）
    std::array<bool, kNumLegs> stance{{true, true, true, true}};
    Vec3d base_pos = Vec3d::Zero();
    Vec3d base_vel = Vec3d::Zero();
    Vec3d com_pos = Vec3d::Zero();
    Vec3d a_des = Vec3d::Zero();
    Vec3d alpha_des = Vec3d::Zero();
    double tilt = 0.0;               // max(|roll|, |pitch|)
    double height = 0.0;             // 估计基座高度
    double est_confidence = 0.0;
    double wbc_eom_residual = 0.0;
    double wbc_contact_residual = 0.0;
    double wbc_cone_violation = 0.0;
    double wbc_base_accel_error = 0.0;
    double wbc_swing_accel_error = 0.0;
    double tau_abs_max = 0.0;
    int wbc_iters = 0;
    // 摆动腿诊断：WBC 看到的期望/实际足端世界 z（支撑腿记 0）+ 计划支撑腿数
    std::array<double, kNumLegs> swing_des_z{};
    std::array<double, kNumLegs> swing_now_z{};
    int stance_count = 0;
    bool mpc_fresh = false;
    int mpc_status = -1;             // -1=本帧未跑 0=ok 1=stale 2=failed（-1 ⇔ 本帧 a_des 被清零）
    int mpc_reason = 0;              // 清零原因：0=正常 1=估计无效 2=支撑腿<1 3=MPC 无解 4=wrench 过期
    bool mpc_held = false;           // 本帧沿用了最近一次有效 wrench（不是本帧新解）
    double mpc_wrench_age = 0.0;     // 本帧实际使用的 wrench 年龄 [s]
    double mpc_net_fz = 0.0;
    int mpc_iters = 0;          // MPC 盒约束 QP 迭代数（诊断用）
    double mpc_qp_residual = 0.0;
    double wbc_ms = 0.0;
    double mpc_ms = 0.0;
    double step_ms = 0.0;
};

class MpcWbcController {
public:
    explicit MpcWbcController(const MpcWbcControllerParams& params = MpcWbcControllerParams())
        : params_(params), model_(), estimator_(params.estimator), gait_(params.gait),
          mpc_(params.mpc), wbc_(params.wbc) {
        ComputeStandGeometry();
    }

    const MpcWbcControllerParams& params() const { return params_; }
    const StateEstimate& estimate() const { return estimate_; }
    const MpcWrench& wrench() const { return wrench_in_; }

    void set_params(const MpcWbcControllerParams& p) {
        params_ = p;
        estimator_ = StateEstimator(p.estimator);
        gait_.set_params(p.gait);
        mpc_.set_params(p.mpc);
        wbc_.set_params(p.wbc);
        ComputeStandGeometry();
    }

    /// 速度命令平滑（一阶低通 + 变化率限幅，按 dt 推进一步）。dt ≤ 0 时只做限幅不推进滤波器。
    GaitCommand SmoothCommand(const GaitCommand& target, double dt) {
        const MpcWbcControllerParams& p = params_;
        // dt ≤ 0（同一时间戳重复调用 / 首帧与 Reset 对齐）不做任何推进：
        // 早期版本在这里"直接取目标"，结果首帧就把命令拉满，平滑形同虚设。
        if (dt <= 0.0) return cmd_filtered_;
        auto slew = [&](double cur, double tgt, double rate) {
            if (rate <= 0.0) return tgt;
            const double max_step = rate * dt;
            return cur + std::max(-max_step, std::min(max_step, tgt - cur));
        };
        // 顺序很重要：**先低通、后限幅**。反过来（先限幅再低通）等于两个滞后串联，
        // 低通的滞后正好抵消限幅的爬升，命令会几乎不动（实测 0.2 s 只爬到 0.0006 m/s，
        // 3 s 才到 0.01 m/s）。低通的起始斜率 = 阶跃/tau，取 tau=0.25 s 时对 0.2 m/s
        // 的阶跃正好是 0.8 m/s²，与限幅同量级；限幅只在超大阶跃（如 0→1.0）时才兜底。
        GaitCommand lp = target;
        if (p.cmd_tau > 0.0) {
            const double a = std::min(1.0, dt / p.cmd_tau);
            lp.vx = cmd_filtered_.vx + a * (target.vx - cmd_filtered_.vx);
            lp.vy = cmd_filtered_.vy + a * (target.vy - cmd_filtered_.vy);
            lp.wz = cmd_filtered_.wz + a * (target.wz - cmd_filtered_.wz);
        }
        cmd_filtered_.vx = slew(cmd_filtered_.vx, lp.vx, p.cmd_slew_v);
        cmd_filtered_.vy = slew(cmd_filtered_.vy, lp.vy, p.cmd_slew_v);
        cmd_filtered_.wz = slew(cmd_filtered_.wz, lp.wz, p.cmd_slew_w);
        return cmd_filtered_;
    }

    /// 进入控制状态时调用（记录起立起点、清空子模块缓存）。
    void Reset(double t, const Vec12d& q_now) {
        t_enter_ = t;
        last_t_ = t;
        t_mpc_ = t - 1e9;
        q_start_ = q_now;
        q_des_ = q_now;
        cmd_filtered_ = GaitCommand();
        height_cmd_ = 0.0;
        height_cmd_initialized_ = false;
        last_dt_ = 0.0;
        dq_des_.setZero();
        gait_.Reset(t);
        wbc_.Reset();
        estimator_.Reset(t);   // 世界原点 = 进入该状态时的机身位置
        estimate_ = StateEstimate();
        wrench_in_ = MpcWrench();
        last_good_wrench_ = MpcWrench();
        seq_wrench_.Write(MpcWrench());
        a_des_.setZero();
        alpha_des_.setZero();
        wbc_active_ = false;
        wbc_fail_count_ = 0;
        invalid_frames_ = 0;
        fault_ = false;
    }

    /// 1 kHz 路径：估计 → 步态 → WBC（用外部传入的 MPC wrench）→ 命令。无线程、无 IO。
    ControllerOutput Step(const ControllerInput& in, const MpcWrench& wrench) {
        wrench_in_ = wrench;
        return StepImpl(in);
    }

    /// 单线程便捷版：内部按 mpc_period 门控调用 SolveWrench（离线自检用；会有 MPC 抖动）。
    ControllerOutput Step(const ControllerInput& in) {
        MpcWrench cached;
        if (seq_wrench_.Read(&cached) && cached.valid) {
            wrench_in_ = cached;
        } else {
            wrench_in_.valid = false;
        }
        ControllerOutput out = StepImpl(in);
        // 门控：到点（或首次/上帧失败）就在本线程解一次，写完快照
        if (out.wbc_active || out.mpc_status == -1) {
            if (in.t - t_mpc_ >= params_.mpc_period || !wrench_in_.valid) {
                const MpcRequest req = BuildMpcRequest(in);
                const MpcWrench w = SolveWrench(req);
                seq_wrench_.Write(w);
                wrench_in_ = w;
                t_mpc_ = in.t;
            }
        }
        return out;
    }

    /// 1 kHz 线程用：把「MPC 需要的一切」打包成快照（纯赋值，不做任何求解）。
    /// 只有把它交给 MPC 线程，1 kHz 路径才不会被 QP 抖动打断。
    MpcRequest BuildMpcRequest(const ControllerInput& in) {
        MpcRequest req;
        req.t = in.t;
        req.rpy = in.rpy;
        req.omega_body = in.omega_body;
        req.q = in.q;
        req.base_pos = estimate_.base_pos_w;
        req.base_vel = estimate_.base_vel_w;
        req.com_body = model_.CompositeComBody(in.q);
        req.est_valid = estimate_.valid;
        req.num_support = estimate_.num_support;
        // 命令用平滑后的值（比入参晚一帧，1 ms 陈旧对 20 Hz 的 MPC 无影响）——
        // 这样 1 kHz 线程与 MPC 线程看到的是同一条命令轨迹。
        req.cmd = cmd_filtered_;
        // 质心高度命令软启动：engage 瞬间 MPC 若看到「高度差 0.3 m」会立刻要 30 m/s² 的竖直
        // 加速度（实测 τ 顶到 28.7 N·m、偶发把机身顶翻）。这里从**实测高度**起步，按
        // com_height_slew 慢慢爬到期望高度。
        const double com_now = estimate_.base_pos_w.z() + model_.CompositeComBody(in.q).z();
        if (!height_cmd_initialized_ || com_now <= 0.05) {
            height_cmd_ = params_.com_height;   // 还没站起来：直接用目标高度
            height_cmd_initialized_ = true;
        } else if (params_.com_height_slew <= 0.0) {
            height_cmd_ = params_.com_height;
        } else {
            const double max_step = params_.com_height_slew *
                                    (last_dt_ > 0.0 ? last_dt_ : params_.mpc_period);
            const double err = params_.com_height - height_cmd_;
            height_cmd_ += std::max(-max_step, std::min(max_step, err));
        }
        req.height_cmd = height_cmd_;
        const int N = std::max(1, std::min(kMaxContactPreview, params_.mpc.N));
        const auto sched = gait_.ContactSchedule<kMaxContactPreview>(in.t, params_.mpc.dt);
        for (int k = 0; k < N; ++k) req.contact[k] = sched[k];
        req.contact_steps = N;
        return req;
    }

    /// MPC 线程用：解一次 SRBD-MPC 并换算成基座期望加速度（唯一可能耗时抖动的部分）。
    MpcWrench SolveWrench(const MpcRequest& req) {
        MpcWrench w;
        w.t = req.t;
        if (!req.est_valid) {
            w.status = 2;
            w.reason = 1;
            return w;  // 保持上一次 wrench（调用方读旧快照）
        }
        if (req.num_support < 1) {
            w.status = 2;
            w.reason = 2;
            return w;
        }
        const Mat3d R = RpyToRotation(req.rpy);
        SrbdMpcInput mpc_in;
        const Vec3d com_world = req.base_pos + R * req.com_body;
        mpc_in.p_com = com_world;
        mpc_in.v_com = req.base_vel + req.omega_body.cross(com_world - req.base_pos);
        mpc_in.rpy = req.rpy;
        mpc_in.omega = req.omega_body;
        mpc_in.inertia_com_world = model_.CompositeInertiaWorld(req.q, R, req.com_body);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d foot_body = model_.FootPositionBody(leg, req.q.segment<3>(3 * leg));
            mpc_in.foot_pos_world[leg] =
                req.base_pos + R * foot_body + Vec3d(0.0, 0.0, -model_.geo.foot_radius);
        }
        const Mat3d R_yaw = YawOnly(R);
        mpc_in.v_cmd = R_yaw * Vec3d(req.cmd.vx, req.cmd.vy, 0.0);
        mpc_in.yaw_rate_cmd = req.cmd.wz;
        mpc_in.height_cmd = req.height_cmd;
        mpc_in.contact.assign(req.contact.begin(), req.contact.begin() + req.contact_steps);

        const SrbdMpcOutput out = mpc_.Solve(req.t, mpc_in);
        w.status = static_cast<int>(out.status);
        w.iters = out.iters;
        w.qp_residual = out.qp_residual;
        w.solve_ms = out.solve_ms;
        if (!out.ok()) {
            w.reason = 3;
            return w;  // 失败：保持上一次 wrench
        }
        w.valid = true;
        w.foot_force_ref = out.foot_force;
        w.net_force = out.net_force;
        w.net_moment = out.net_moment;
        ComputeBaseAccelDesired(out, mpc_in.inertia_com_world, req.omega_body, R,
                                model_.TotalMass(), params_.mpc.gravity, &w.a_des_world,
                                &w.alpha_des_body);
        return w;
    }

    /// 站立/行走时的期望质心高度（由站立构型自动算出，供上层设置 MPC 的高度命令）。
    double stand_com_height() const { return stand_com_height_; }

    /// 站立时基座原点高度（球足触地、关节角 = stand_q）。
    double stand_base_height() const { return stand_base_height_; }

    /// 平滑后的速度命令（诊断/测试用）。
    const GaitCommand& smoothed_command() const { return cmd_filtered_; }

    /// 是否已触发故障（姿态超限 / WBC 连续失败）——上层据此切到阻尼状态。
    bool fault() const { return fault_; }

    /// 最近一次写出的 wrench 快照（供上层线程间共享）。
    const SeqLockBox<MpcWrench>& wrench_box() const { return seq_wrench_; }

private:
    ControllerOutput StepImpl(const ControllerInput& in) {
        const auto t0 = std::chrono::steady_clock::now();
        ControllerOutput out;
        const MpcWbcControllerParams& p = params_;

        // ---- 时间步（仅诊断；子模块各自按绝对时间戳工作，丢包不会静默推进状态） ----
        double dt = in.t - last_t_;
        if (!(dt > 0.0) || dt > p.max_dt) dt = 0.0;
        out.dt = dt;

        const Mat3d R = RpyToRotation(in.rpy);
        out.base_pos = estimate_.base_pos_w;
        out.base_vel = estimate_.base_vel_w;
        out.height = estimate_.height;
        out.est_confidence = estimate_.confidence;
        out.tilt = std::max(std::fabs(in.rpy(0)), std::fabs(in.rpy(1)));

        // ---- 阶段判定 ----
        const double rel = in.t - t_enter_;
        const bool ramping = rel < p.stand_ramp_time;
        const bool settling = !ramping && rel < p.stand_ramp_time + p.stand_settle_time;

        // ---- 命令平滑（阶跃命令会顶翻机身，见 cmd_tau 注释） ----
        last_dt_ = dt;
        const GaitCommand cmd = SmoothCommand(in.cmd, dt);

        // ---- 步态（同一相位源驱动支撑集、足端目标与 MPC 的接触计划） ----
        const auto legs = gait_.Update(in.t, cmd);
        out.stance = {{legs[kFL].contact, legs[kFR].contact, legs[kHL].contact, legs[kHR].contact}};
        const bool stepping = gait_.stepping();

        // ---- 姿态保护 ----
        // 故障（→ 上层切阻尼）只看 tilt_limit；wbc_enable_tilt 只决定"敢不敢使能 WBC"，
        // 起立/被推动阶段姿态略大时不至于直接判故障。
        if (out.tilt > p.tilt_limit) fault_ = true;
        const bool tilt_ok_for_wbc = out.tilt <= p.wbc_enable_tilt;
        if (fault_) {
            // 故障：保持上一帧目标角（或当前角），纯位置环兜底，等上层切到阻尼状态
            out.mode = ControllerOutput::Mode::kFault;
            out.q_des = q_des_;
            out.joint_cmd.col(0).setConstant(p.kp_pos);
            out.joint_cmd.col(1) = out.q_des.cast<float>();
            out.joint_cmd.col(2).setConstant(p.kd_pos);
            out.joint_cmd.col(3).setZero();
            out.joint_cmd.col(4).setZero();
            out.fault = true;
            last_t_ = in.t;
            out.step_ms = ElapsedMs(t0);
            return out;
        }

        // ---- 目标关节角：起立斜坡用关节空间插值，之后由步态足端 IK 得到 ----
        if (ramping) {
            const double s = SmoothStep01(rel / std::max(1e-6, p.stand_ramp_time));
            q_des_ = (1.0 - s) * q_start_ + s * p.stand_q;
        } else {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                Vec3d seed = q_des_.segment<3>(3 * leg);
                Vec3d q_leg;
                if (model_.InverseKinematics(leg, legs[leg].foot_pos, &q_leg, &seed))
                    q_des_.segment<3>(3 * leg) = q_leg;
            }
        }
        ClipToJointLimits(&q_des_);
        out.q_des = q_des_;

        // ---- 状态估计（用**计划支撑集**做零速反解） ----
        EstimatorInput est_in;
        est_in.t = in.t;
        est_in.rpy = in.rpy;
        est_in.omega_body = in.omega_body;
        est_in.acc_body = in.acc_body;
        est_in.q = in.q;
        est_in.dq = in.dq;
        est_in.contact = out.stance;
        estimate_ = estimator_.Update(est_in);
        out.base_pos = estimate_.base_pos_w;
        out.base_vel = estimate_.base_vel_w;
        out.height = estimate_.height;
        out.est_confidence = estimate_.confidence;

        // 世界速度装配（WBC/MPC 共用）：qvel = [v_world; ω_body; dq]
        Vec18d qvel = Vec18d::Zero();
        qvel.head<3>() = estimate_.base_vel_w;
        qvel.segment<3>(3) = in.omega_body;
        qvel.tail<12>() = in.dq;

        const Vec3d com_body = model_.CompositeComBody(in.q);
        const Vec3d com_world = estimate_.base_pos_w + R * com_body;
        out.com_pos = com_world;

        // ---- WBC 使能条件 ----
        // WBC 使能（带滞回）：只在「斜坡结束 + 姿态正常 + 估计有效」时开启；
        // 已开启后允许估计短暂失效（hold 窗内继续用 WBC，超时才退回位置环）。
        const bool wbc_gate_open = !ramping && !settling && estimate_.valid &&
                                   estimate_.num_support >= 1 && tilt_ok_for_wbc;
        if (wbc_gate_open) {
            invalid_frames_ = 0;
            wbc_active_ = true;
        } else if (wbc_active_ && !ramping && !settling && !fault_ &&
                   ++invalid_frames_ <= p.wbc_enable_hold_frames &&
                   p.wbc_enable_hold_frames > 0) {
            // 滞回窗内保持。注意这里不再重复检查 tilt_ok_for_wbc：姿态在
            // (wbc_enable_tilt, tilt_limit] 之间属于"还想试一把"的区域，一帧退回位置环
            // 会把 kp 立刻翻倍 + τ_ff 清零（比撑着跑更糟）。真正超限由上面的 fault_ 立即兜底。
            wbc_active_ = true;
        } else {
            invalid_frames_ = 0;
            wbc_active_ = false;
        }
        const bool wbc_ready = wbc_active_;

        if (wbc_ready) {
            // ---- MPC（门控 20 Hz，失败则沿用上次期望 wrench） ----
            WbcInput win;
            win.q = in.q;
            win.qvel = qvel;
            win.R_wb = R;
            win.stance = out.stance;
            win.q_nominal = p.stand_q;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const Vec3d foot_body = model_.FootPositionBody(leg, in.q.segment<3>(3 * leg));
                win.foot_pos_world[leg] = estimate_.base_pos_w + R * foot_body;
            }
            const Mat12x18d Jc = model_.ContactJacobianWorld(in.q, R);
            for (int leg = 0; leg < kNumLegs; ++leg)
                win.foot_vel_world[leg] = Jc.block<3, kWbcQdd>(3 * leg, 0) * qvel;

            // 用最近一次**有效**的 MPC wrench（可能来自别的线程）。
            //
            // 关键：单帧求解失败（估计瞬时无效 / 支撑腿<1 / MPC 无有效解）**不得**把上一帧还新鲜的
            // wrench 一并作废——否则 WBC 会在「有力参考」与「零加速度」之间断续切换。实测 trot 行走中
            // 19% 的 MPC 周期被清零（翻车前窗口 30%），是竖向振荡的激励源；而估计器的数值本身是准的
            // （height 与真值差 <1 mm），只是零速反解的 `valid` 标志在摆动腿落地时翻假。
            // 因此这里保留最近一次有效 wrench，直到它超过 wrench_max_age 才退化为零加速度
            // （「原地保持比重放旧 wrench 安全」仍由 age 上限保证）。
            const MpcWrench& w = wrench_in_;
            if (w.valid) last_good_wrench_ = w;
            const auto fresh_enough = [&](const MpcWrench& c) {
                const double age = in.t - c.t;
                return c.valid && age >= -1e-9 && age <= p.wrench_max_age;
            };
            const MpcWrench* src = nullptr;
            if (fresh_enough(w)) {
                src = &w;                       // 本帧快照
            } else if (fresh_enough(last_good_wrench_)) {
                src = &last_good_wrench_;       // 沿用最近一次有效 wrench
            }

            out.mpc_status = src ? w.status : -1;   // -1 ⇔ 本帧 a_des 被清零
            out.mpc_reason = src ? 0 : (w.valid ? 4 : (w.reason != 0 ? w.reason : 4));
            out.mpc_held = (src == &last_good_wrench_);
            out.mpc_wrench_age = src ? (in.t - src->t) : 0.0;
            out.mpc_fresh = false;
            out.mpc_ms = w.solve_ms;
            out.mpc_iters = w.iters;
            out.mpc_qp_residual = w.qp_residual;
            if (src != nullptr) {
                win.foot_force_ref = src->foot_force_ref;
                a_des_ = src->a_des_world;
                alpha_des_ = src->alpha_des_body;
                out.mpc_net_fz = src->net_force.z();
                out.mpc_fresh = (w.valid && out.mpc_wrench_age < 2.0 * p.mpc_period);
            } else {
                a_des_.setZero();
                alpha_des_.setZero();
            }
            win.a_des_world = a_des_;
            win.alpha_des_body = alpha_des_;
            out.a_des = a_des_;            // 诊断：本帧真正送进 WBC 的期望加速度
            out.alpha_des = alpha_des_;

            // 摆动足期望：机体系目标 → 世界系（默认只用偏航，避免把机身倾斜写进轨迹）
            const Mat3d R_ref = p.swing_target_yaw_only ? YawOnly(R) : R;
            out.stance_count = 0;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (win.stance[leg]) {
                    ++out.stance_count;
                    continue;
                }
                win.foot_des_world[leg] = estimate_.base_pos_w + R_ref * legs[leg].foot_pos;
                win.foot_vel_des_world[leg] = R_ref * legs[leg].foot_vel;
                out.swing_des_z[leg] = win.foot_des_world[leg].z();
                out.swing_now_z[leg] = win.foot_pos_world[leg].z();
            }

            // ---- WBC ----
            const WbcOutput wout = wbc_.Solve(in.t, win);
            out.wbc_stale = (wout.status == WbcOutput::Status::kStale);
            if (wout.ok()) {
                out.tau_wbc = wout.tau;
                out.foot_force = wout.foot_force;
                out.wbc_eom_residual = wout.eom_residual;
                out.wbc_contact_residual = wout.contact_residual;
                out.wbc_cone_violation = wout.cone_violation;
                out.wbc_base_accel_error = wout.base_accel_error;
                out.wbc_swing_accel_error = wout.swing_accel_error;
                out.tau_abs_max = wout.tau_abs_max;
                out.wbc_iters = wout.iters;
                out.wbc_ms = wout.solve_ms;
                wbc_fail_count_ = 0;
                out.wbc_active = true;
            } else {
                ++wbc_fail_count_;
                if (wbc_fail_count_ >= p.max_consecutive_wbc_fail) {
                    fault_ = true;
                    out.fault = true;
                }
                wbc_active_ = false;  // 本帧退回位置环
            }
        } else {
            out.mpc_status = -1;  // 本帧没跑 MPC（未使能 WBC）
        }

        // ---- 关节命令：τ_ff 反解，使执行器总力矩 = WBC 力矩 ----
        const double kp = wbc_active_ ? p.kp_pos * p.kp_scale_wbc : p.kp_pos;
        const double kd = wbc_active_ ? p.kd_pos * p.kd_scale_wbc : p.kd_pos;
        out.joint_cmd.col(0).setConstant(kp);
        out.joint_cmd.col(1) = q_des_.cast<float>();
        out.joint_cmd.col(2).setConstant(kd);
        out.joint_cmd.col(3) = dq_des_.cast<float>();  // 期望关节速度（站立/步态均为 0）
        for (int j = 0; j < 12; ++j) {
            const double pd = kp * (q_des_(j) - in.q(j)) + kd * (dq_des_(j) - in.dq(j));
            out.tau_pd(j) = pd;
            // WBC 工作时：反解 τ_ff 使**总力矩** = τ_wbc（执行器总力矩 = PD + τ_ff）。
            // WBC 未工作（起立斜坡/降级/故障）时必须 τ_ff = 0，否则等于把 PD 又反向叠加一次
            // （实测：起立段 τ_ff=−pd 被 ±30 夹住后总力矩多出 −30 N·m，机器人被推翻）。
            double tau_ff = 0.0;
            if (wbc_active_) {
                tau_ff = out.tau_wbc(j) - pd;
                // 总力矩由 WBC 约束保证在 ±tau_max 内；这里再夹一次前馈，防 PD 项瞬态把前馈顶爆
                tau_ff = std::max(-p.wbc.tau_max, std::min(p.wbc.tau_max, tau_ff));
            }
            out.joint_cmd(j, 4) = static_cast<float>(tau_ff);
        }

        out.mode = ramping ? ControllerOutput::Mode::kRamp
                           : (stepping ? ControllerOutput::Mode::kWalk : ControllerOutput::Mode::kStand);
        last_t_ = in.t;
        out.step_ms = ElapsedMs(t0);
        return out;
    }

private:
    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }

    static double SmoothStep01(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    /// 只保留偏航的旋转矩阵（摆脚目标参考系用）。
    static Mat3d YawOnly(const Mat3d& R) {
        const double yaw = std::atan2(R(1, 0), R(0, 0));
        return Rz(yaw);
    }

    static Mat3d Rz(double yaw) {
        Mat3d R = Mat3d::Identity();
        const double c = std::cos(yaw), s = std::sin(yaw);
        R(0, 0) = c;
        R(0, 1) = -s;
        R(1, 0) = s;
        R(1, 1) = c;
        return R;
    }

    void ClipToJointLimits(Vec12d* q) const {
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int j = 0; j < 3; ++j) {
                const int idx = 3 * leg + j;
                const double lo = model_.geo.joint_lower(j), hi = model_.geo.joint_upper(j);
                (*q)(idx) = std::max(lo, std::min(hi, (*q)(idx)));
            }
    }

    void ComputeStandGeometry() {
        double min_foot_z = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg)
            min_foot_z = std::min(min_foot_z,
                                  model_.FootPositionBody(leg, params_.stand_q.segment<3>(3 * leg)).z());
        stand_base_height_ = -min_foot_z + model_.geo.foot_radius;
        stand_com_height_ = stand_base_height_ + model_.CompositeComBody(params_.stand_q).z();
        if (!(params_.com_height > 0.0)) params_.com_height = stand_com_height_;
    }

    MpcWbcControllerParams params_;
    Lite3Model model_;
    StateEstimator estimator_;
    GaitTrajectory gait_;
    SrbdPreviewMpc mpc_;
    WeightedWbc wbc_;

    StateEstimate estimate_;
    GaitCommand cmd_filtered_;               // 平滑后的速度命令（步态/MPC 共用）
    double height_cmd_ = 0.0;                // 质心高度命令（软启动状态）
    bool height_cmd_initialized_ = false;
    double last_dt_ = 0.0;
    MpcWrench wrench_in_;                    // 本帧使用的 wrench（来自本线程或序列锁快照）
    MpcWrench last_good_wrench_;             // 最近一次有效 wrench（求解失败时按 age 上限沿用）
    SeqLockBox<MpcWrench> seq_wrench_;       // 跨线程发布用（单线程模式下只当缓存）
    Vec3d a_des_ = Vec3d::Zero();
    Vec3d alpha_des_ = Vec3d::Zero();

    double t_enter_ = 0.0;
    double last_t_ = 0.0;
    double t_mpc_ = -1e9;
    Vec12d q_start_ = Vec12d::Zero();
    Vec12d q_des_ = Vec12d::Zero();
    Vec12d dq_des_ = Vec12d::Zero();
    int wbc_fail_count_ = 0;
    int invalid_frames_ = 0;
    bool wbc_active_ = false;
    bool fault_ = false;

    double stand_base_height_ = 0.0;
    double stand_com_height_ = 0.0;
};

/// MPC 工作线程：把「解 MPC」从 1 kHz 控制线程里搬走。
///   1 kHz 线程：`requests.Write(BuildMpcRequest(in))` → `wrenches.Read(&w)` → `Step(in, w)`
///   MPC 线程  ：按 `period` 醒来，取最新请求解一次，把 wrench 写回快照。
/// 实测（trot，闭环 MuJoCo）：MPC 单次 0.8 ms 平均但会偶发 10~30 ms（盒约束 QP 退化点），
/// 放在 1 kHz 线程里会让 WBC 漏拍并导致行走失稳。
class MpcWorkerThread {
public:
    MpcWorkerThread(MpcWbcController* controller, double period)
        : ctrl_(controller), period_(period) {}
    ~MpcWorkerThread() { Stop(); }

    void Start() {
        if (running_) return;
        running_ = true;
        thread_ = std::thread(&MpcWorkerThread::Loop, this);
    }

    void Stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    SeqLockBox<MpcRequest> requests;   // 1 kHz → MPC
    SeqLockBox<MpcWrench> wrenches;    // MPC → 1 kHz

    int solve_count() const { return solve_count_; }

private:
    void Loop() {
        uint64_t last_seq = 0;
        while (running_) {
            MpcRequest req;
            uint64_t seq = 0;
            if (requests.ReadWithSeq(&req, &seq) && seq != last_seq) {
                last_seq = seq;
                wrenches.Write(ctrl_->SolveWrench(req));
                ++solve_count_;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(period_));
        }
    }

    MpcWbcController* ctrl_;
    double period_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<int> solve_count_{0};
};

}  // namespace mpc_wbc
