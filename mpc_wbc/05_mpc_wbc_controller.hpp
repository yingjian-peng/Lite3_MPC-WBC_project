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
    // ---- 三轴组合约束（docs/00 §2.2）----
    //   sqrt(vx²+vy²) + combo_wz_ratio·|wz| ≤ combo_max，超限时三轴同比例缩比。
    // 这条约束在设计文档里一直有（0.18 / 0.67），但**代码里从未实现**。实测三轴同时给
    // 满量程（vx 0.60 + vy 0.50 + wz 1.00 → s = 0.961）会让 MPC 打满、侧倾冲到 46.6°
    // 直接翻车（`build/single_axis/max3`）；而单轴/纯转向都远小于 0.67（0.60 / 0.50 /
    // 0.18），所以补上后**不影响已验证的单轴包线**。combo_max ≤ 0 表示关闭（旧行为）。
    double combo_wz_ratio = 0.18;
    double combo_max = 0.67;
    // ---- 转向-平移「竞争项」（默认为 0 = 关闭，见下）----
    //
    // 历史：曾根据 `analyze_axis_sweep.py` 的统计拟合出 Δv ≈ 1.7·vxy·ωz²，并据此加了
    // `vxy ≤ tol/(gain·ωz²)` 的守卫。**后来发现那是分析脚本的测量伪影**：脚本把整段
    // 世界速度用「该段平均偏航」只旋转了一次，而原地转向时机身一直在转（0.5 rad/s×8 s
    // = 4 rad），均值因此被系统性压小，转得越快压得越厉害 —— 正好伪造出一条 ∝ωz² 的
    // “损失”。改成**逐帧**用该帧偏航旋转后，同一份 sim.csv：
    //   (0.60, ωz0.50) 旧口径 0.222（“损失 0.379”）→ 逐帧口径 0.640（无损失）
    //   (0.50vy, ωz0.50) 旧口径 0.155（“损失 0.345”）→ 逐帧口径 0.498（无损失）
    // 也就是说**转向并不抢平移的摩擦预算**，三轴同时给也能跟上（只是姿态变差）。
    // 因此这里默认关闭（0），只保留线性安全项；参数保留以便将来在真机上重新标定。
    double combo_turn_gain = 1.7;
    double combo_turn_loss = 0.0;   // ≤0 表示关闭
    // 超限时的分配策略：
    //   0 = 同比缩比（三轴按 k³ 等比缩，最“公平”，保方向保比例）← 默认
    //   1 = 转向优先（只削平移，保 |wz|；遥控习惯“方向不能丢”）
    //   2 = 平移优先（只削 wz，保平移；“先走起来再微调方向”）
    int combo_priority = 0;
    // ---- 单轴硬上限（命令级钳位）----
    // 与 `gait_schedule.max_vx/vy/wz` 同一套值，但那个只用于主轴判定，不钳位。
    // 实测（2026-11-01）：vx 0.60 / vy 0.50 / wz 1.00 是单轴可用上限，超过即退化，
    // 所以在这里硬钳一下，保证任何上游（手柄、上层状态机、测试脚本）都超不过去。
    // <=0 表示不钳。
    double cmd_max_vx = 0.60;
    double cmd_max_vy = 0.50;
    double cmd_max_wz = 1.00;
    // ---- 步态级速度/偏航率反馈（只作用于**足端轨迹**的步幅，不改变 MPC 参考）----
    // 动机：足端轨迹用命令速度算步幅 `Δp = T_stance·(v + ω×p0)`，而实际机身速度由接触
    // 滑动、摆动腿动量等决定。实测单轴 vx 闭环稳态**恒快 0.04–0.05 m/s**（0.15 → 0.204、
    // 0.60 → 0.652），纯开环调参改不掉；用机体速度误差修正步幅可直接把稳态误差压到
    // 一个增益的倒数。增益取 0 则完全回到旧行为。
    double gait_vel_fb = 0.0;        // 机体速度误差 → 步幅修正（无量纲，建议 0.3–0.6）
    double gait_yaw_fb = 0.0;        // 偏航率误差 → 转向步幅修正（无量纲，建议 0.2–0.5）
    double gait_vel_trim = 0.0;      // 沿运动方向的步幅前馈修正 [m/s]（抵消恒定偏快）
    double gait_cmd_limit_scale = 1.3;  // 反馈后步幅命令的放大上限（相对包络）
    // **命令级**线速度偏置补偿 [m/s]（沿运动方向把 |v_cmd| 减掉这个值，再同时送进
    // 步态与 MPC 参考）。
    //
    // 实测（闭环、单轴 8 s 段）：机身稳态速度近似等于 `指令 + 0.045`（vx 0.15→0.204、
    // 0.30→0.355、0.45→0.494、0.60→0.653；vy 0.10→0.156、0.35→0.365）。
    // 只改步幅（gait_vel_trim）或只改 MPC 参考（mpc_vel_trim）都无效，**同时**改才有效：
    // S8_CMD_SCALE=0.667 的对照实验里 0.15→0.138、0.30→0.263、0.45→0.362，即整体
    // 按比例缩、偏置随之下降 → 证明是「指令 + 常量」型偏差。
    // 0 = 不补偿（默认）。仿真里 0.05 可把 0.15–0.60 的跟踪误差压到 0.01 量级；
    // 换到真机必须重新标定（这是接触/滑移标定，不是通用控制增益）。
    double cmd_vel_trim = 0.0;
    // MPC 速度参考的前馈修正 [m/s]（沿运动方向缩小 |v_ref|）。
    // 实测：机身稳态速度恒比指令快 0.04–0.05 m/s（vx 0.15→0.204、0.30→0.374、
    // 0.45→0.495、0.60→0.652；vy 0.10→0.160、0.35→0.368），而**改足端步幅
    // （反馈/trim）完全无效**——说明机身速度由 MPC 的速度参考闭环决定，不是步幅。
    // 根因是 MPC 没有积分项，恒定扰动下必然有稳态误差；把参考预先减去该偏置即可。
    double mpc_vel_trim = 0.0;
    // 质心高度命令的软启动变化率 [m/s]（0 = 关闭软启动，直接用 com_height）。
    // 实测：engage 时若高度命令与实测差 0.3 m，MPC 会立刻要求 ~30 m/s² 的竖直加速度，
    // 把 τ 顶到限幅附近、偶发顶翻机身。0.06 m/s 约 5 s 走完 0.3 m，起立过程平滑。
    double com_height_slew = 0.06;
    // ---- 低层 PD ----
    double kp_pos = 100.0;           // 位置环档位（docs/03 附录 C）
    double kd_pos = 2.5;
    // WBC 工作时的 PD 缩放。**必须保持 1.0**：低层 PD 是内环，WBC 通过 τ_ff 只负责把总力矩
    // 拉到 τ_wbc，两者职责正交，把内环打折只会让关节跟踪变差。
    //
    // 历史：原值 0.5（理由写的是「参考工程推荐 1 kHz WBC + 0.4~0.5」），但 legged_control
    // 实际上直接用 task.info 里的整档 kp/kd，从不打折。实测打折的代价：
    //   0.45 m/s 关节跟踪误差漂到 0.1–0.3 rad → τ_ff = τ_wbc − pd 常年顶在 ±30 N·m 上限
    //   （饱和率 44%），踩不到 30 时又反过来推翻长驱力矩。
    // 2026-10-21 扫描（0.45 m/s，S8_CMD_SCALE=3.0）：
    //   scale=0.25 → 12.9 s 失稳；0.35 → 9.0 s 失稳；0.5 → 12.9 s 失稳；**1.0 → 全程通过**
    // 换成 1.0 后 0.15 / 0.30 / 0.45 / 0.60 m/s 全部 rc=0（0.75 超 docs/00 包线，仍失效）。
    double kp_scale_wbc = 1.0;
    double kd_scale_wbc = 1.0;
    // 把 PD 叠加到 WBC 力矩之上的比例（0 = 完全抵消 PD，旧行为；1 = τ_wbc + pd）。
    // 默认只作用于支撑腿（摆动腿已有足端任务，叠加 PD 会与其抢）。
    double pd_extra = 0.0;
    bool pd_extra_stance_only = true;
    // 是否把步态的期望足端速度用腿雅可比反解成期望关节速度（喂给 PD 的 kd 项与执行器前馈）。
    // true ：PD 的 kd 项量的是「真实关节速度相对**计划**关节速度」的偏差。
    // false：dq_des 恒为 0，kd 项退化为标准的速度阻尼 −kd·dq（与 S4 开环基线/站立环一致）。
    //
    // 2026-10-21 A/B（0.45 m/s，kp_scale=1.0，两者都 rc=0）—— 实测**没有可测收益**，故默认 false：
    //   DQ_FF=0：forward 2.87° / turn 9.21° / stop4 17.0°，τ_ff 饱和 15–66%
    //   DQ_FF=1：forward 5.20° / turn 6.26° / stop4 12.4°，τ_ff 饱和 32–59%
    // 混合结果、互有胜负；且 true 时 τ_ff 饱和率整体更高（因为步态给的 foot_vel 是**平均速度**，
    // 与真实关节速度并不逐帧匹配，反而放大了 kd 项）。保留开关供后续换更准的参考轨迹时复验。
    bool swing_joint_vel_ff = false;
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
    // 三段式步态调度：打开后 gait 只在 use_gait_schedule=false 时生效。
    // 默认用开环基线 openloop-baseline-v1 的同一套参数，保证两条路径可比。
    bool use_gait_schedule = true;
    SegmentedGaitParams gait_schedule = SegmentedGaitParams::BaselineClosedLoop();
    // 标称足位前后偏移（按整机质心配平，与位置环基线一致）[m]
    double nominal_x = -0.0133;
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
    double yaw_ref = 0.0;              // 偏航角参考（世界系，**连续不归一化**，由 1 kHz 线程积分）
    double yaw_meas = 0.0;             // 连续（不归一化）的测量偏航角，与 yaw_ref 同一条数轴
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
        ApplyGaitConfig();
        ComputeStandGeometry();
    }

    const MpcWbcControllerParams& params() const { return params_; }
    const StateEstimate& estimate() const { return estimate_; }
    const MpcWrench& wrench() const { return wrench_in_; }

    void set_params(const MpcWbcControllerParams& p) {
        params_ = p;
        estimator_ = StateEstimator(p.estimator);
        gait_.set_params(p.gait);
        ApplyGaitConfig();
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
        yaw_ref_ = 0.0;              // 进入时视为航向零点，之后由 yaw_rate_cmd 积分
        yaw_ref_initialized_ = false;
        yaw_meas_cont_ = 0.0;
        yaw_meas_prev_ = 0.0;
        yaw_meas_initialized_ = false;
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
        // 软停步期间把 MPC 的速度/偏航参考按同一条步幅曲线缩下来：参考与足端规划
        // 同源，停步时才不会「参考已归零、腿还在扫」而互相顶。
        {
            const double ps = gait_.plan_scale();
            if (ps < 1.0) {
                req.cmd.vx *= ps;
                req.cmd.vy *= ps;
                req.cmd.wz *= ps;
            }
        }
        // MPC 速度参考前馈修正：机身稳态速度恒快一个常量（见 mpc_vel_trim 注释）。
        if (params_.mpc_vel_trim > 0.0) {
            const double vm = std::hypot(req.cmd.vx, req.cmd.vy);
            if (vm > 1e-6) {
                // 最多修正一半，避免很小的指令被削成 0（那会变成「原地踏步但报运动指令」）。
                const double trim = std::min(params_.mpc_vel_trim, 0.5 * vm);
                const double k = (vm - trim) / vm;
                req.cmd.vx *= k;
                req.cmd.vy *= k;
            }
        }
        // 偏航参考由 1 kHz 线程积分（不能用测量偏航，否则扰动造成的偏航永不回正）。
        //
        // **整条偏航链路都用「连续不归一化」的角度**：测量值 `rpy.z()` 被仿真按 ±π 包络，
        // 一旦机身航向越过 ±180° 就会出现分支切换。之前的做法是把参考也包到 ±π、再在
        // MPC 入口对齐分支，数学上等价、实测却仍有与绝对航向相关的失稳：从 0° 起以
        // 0.30 rad/s 转向，航向走到 ≈100° 后机开始 1 Hz 涡动（倾角 10–20°），
        // 而同一段转动从 +90° 起步则完全干净；0.15 rad/s 连跑 45 s 也是「航向进入
        // |yaw|≳110° 失稳、出了 ±180° 窗口自动收敛」。现改为在 1 kHz 线程里对测量偏航
        // 做增量累加得到连续航向，参考同轴累加、不再 WrapPi，两路都无分支。
        if (!yaw_meas_initialized_) {
            yaw_meas_cont_ = in.rpy.z();
            yaw_meas_prev_ = in.rpy.z();
            yaw_meas_initialized_ = true;
        } else {
            yaw_meas_cont_ += WrapPi(in.rpy.z() - yaw_meas_prev_);
            yaw_meas_prev_ = in.rpy.z();
        }
        if (!yaw_ref_initialized_) {
            yaw_ref_ = yaw_meas_cont_;
            yaw_ref_initialized_ = true;
        } else {
            const double d = in.t - last_yaw_ref_t_;
            // 停步收步期间按同一条步幅曲线缩小偏航参考的推进速度，避免参考跑到机身前面。
            const double ps = gait_.plan_scale();
            if (d > 0.0 && d < 0.1) yaw_ref_ += ps * cmd_filtered_.wz * d;
        }
        last_yaw_ref_t_ = in.t;
        req.yaw_ref = yaw_ref_;
        req.yaw_meas = yaw_meas_cont_;
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
        mpc_in.R_wb = R;
        // 偏航参考与测量都已是**连续不归一化**的角度（见 BuildMpcRequest 注释），
        // 这里直接相减即可，不需要任何 WrapPi 对齐。
        mpc_in.rpy.z() = req.yaw_meas;
        mpc_in.yaw_ref = req.yaw_ref;
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

        // ---- 命令级线速度偏置补偿（步态与 MPC 参考共用同一个目标） ----
        GaitCommand target = in.cmd;
        // ---- 单轴硬上限（单轴指令就是这三条）----
        if (p.cmd_max_vx > 0.0) target.vx = std::max(-p.cmd_max_vx, std::min(p.cmd_max_vx, target.vx));
        if (p.cmd_max_vy > 0.0) target.vy = std::max(-p.cmd_max_vy, std::min(p.cmd_max_vy, target.vy));
        if (p.cmd_max_wz > 0.0) target.wz = std::max(-p.cmd_max_wz, std::min(p.cmd_max_wz, target.wz));
        if (p.cmd_vel_trim > 0.0) {
            const double vm = std::hypot(target.vx, target.vy);
            if (vm > 1e-6) {
                const double trim = std::min(p.cmd_vel_trim, 0.5 * vm);
                target.vx *= (vm - trim) / vm;
                target.vy *= (vm - trim) / vm;
            }
        }
        // ---- 三轴组合约束（docs/00 §2.2）：超限时三轴同比例缩比，方向不变 ----
        if (p.combo_max > 0.0) {
            const double s = std::hypot(target.vx, target.vy) +
                             p.combo_wz_ratio * std::fabs(target.wz);
            if (s > p.combo_max) {
                const double k = p.combo_max / s;
                target.vx *= k;
                target.vy *= k;
                target.wz *= k;
            }
        }
        // ---- 转向-平移摩擦竞争守卫（策略可选）----
        // 线性约束只能防翻车；要保证「平移还能跟上」，必须把实测的平方竞争项也盖住。
        // 超限后的分配策略见 `combo_priority`。
        if (p.combo_turn_loss > 0.0) {
            const double vxy = std::hypot(target.vx, target.vy);
            const double wz_abs = std::fabs(target.wz);
            const double w2 = target.wz * target.wz;
            const double budget = p.combo_turn_loss / std::max(1e-6, p.combo_turn_gain);
            if (vxy > 1e-9 && w2 > 1e-12 && vxy * w2 > budget) {
                if (p.combo_priority == 1) {
                    // 转向优先：保 wz，削平移
                    const double k = budget / (vxy * w2);   // <1
                    target.vx *= k;
                    target.vy *= k;
                } else if (p.combo_priority == 2) {
                    // 平移优先：保平移，削 wz（求 vxy·w² = budget）
                    const double k = std::sqrt(budget / vxy) / wz_abs;   // <1
                    target.wz *= k;
                } else {
                    // 同比缩比：三轴同乘 k，使 k·vxy·(k·wz)² = budget → k³
                    const double k = std::cbrt(budget / (vxy * w2));     // <1
                    target.vx *= k;
                    target.vy *= k;
                    target.wz *= k;
                }
            }
        }
        // ---- 命令平滑（阶跃命令会顶翻机身，见 cmd_tau 注释） ----
        last_dt_ = dt;
        const GaitCommand cmd = SmoothCommand(target, dt);

        // ---- 步态级速度/偏航率反馈（只改**足端步幅**，不动 MPC 的速度参考） ----
        // 估计量比本帧晚 1 ms（估计器在下方才更新），对 0.5–2 Hz 的步幅修正足够新鲜。
        GaitCommand gait_cmd = cmd;
        if (p.gait_vel_fb != 0.0 || p.gait_yaw_fb != 0.0 || p.gait_vel_trim != 0.0) {
            const Vec3d v_body = R.transpose() * estimate_.base_vel_w;
            const double wz_meas = in.omega_body.z();
            if (p.gait_vel_trim != 0.0) {
                // 沿命令方向收缩步幅（抵消「实际恒快一个常量」的标定偏差）。
                const double vmag = std::hypot(cmd.vx, cmd.vy);
                if (vmag > 1e-6) {
                    const double k = std::max(0.0, vmag - p.gait_vel_trim) / vmag;
                    gait_cmd.vx *= k;
                    gait_cmd.vy *= k;
                }
            }
            if (p.gait_vel_fb != 0.0) {
                gait_cmd.vx += p.gait_vel_fb * (cmd.vx - v_body.x());
                gait_cmd.vy += p.gait_vel_fb * (cmd.vy - v_body.y());
            }
            if (p.gait_yaw_fb != 0.0) {
                gait_cmd.wz += p.gait_yaw_fb * (cmd.wz - wz_meas);
            }
            // 限幅：反馈最多把步幅放大到包络的 gait_cmd_limit_scale 倍。
            const double lim = std::max(1.0, p.gait_cmd_limit_scale);
            const double lvx = lim * p.gait_schedule.max_vx;
            const double lvy = lim * p.gait_schedule.max_vy;
            const double lwz = lim * p.gait_schedule.max_wz;
            gait_cmd.vx = std::max(-lvx, std::min(lvx, gait_cmd.vx));
            gait_cmd.vy = std::max(-lvy, std::min(lvy, gait_cmd.vy));
            gait_cmd.wz = std::max(-lwz, std::min(lwz, gait_cmd.wz));
        }

        // ---- 步态（同一相位源驱动支撑集、足端目标与 MPC 的接触计划） ----
        // 停步判定用**原始**命令：平滑后的命令衰减太慢，会给收步叠加 ~0.75 s 延迟。
        const bool raw_cmd_zero =
            gait_.NormalizedSpeed(in.cmd) < gait_.params().stop_threshold;
        const auto legs = gait_.Update(in.t, gait_cmd, raw_cmd_zero);
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
            dq_des_.setZero();  // 斜坡段的关节速度不参与前馈，显式清零防旧值残留
        } else {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                Vec3d seed = q_des_.segment<3>(3 * leg);
                Vec3d q_leg = seed;
                if (model_.InverseKinematics(leg, legs[leg].foot_pos, &q_leg, &seed))
                    q_des_.segment<3>(3 * leg) = q_leg;
                // 期望关节速度：v_foot^B = J_leg·dq  ⇒  dq_des = J_leg⁻¹·v_foot_des^B
                // 是否启用由 swing_joint_vel_ff 控制（env S8_DQ_FF）。默认关闭，A/B 结论见参数注释。
                if (p.swing_joint_vel_ff) {
                    const Mat3d J_leg = model_.LegJacobianBody(leg, q_leg);
                    Eigen::FullPivLU<Mat3d> lu(J_leg);
                    if (lu.isInvertible())
                        dq_des_.segment<3>(3 * leg) = lu.solve(legs[leg].foot_vel);
                    else
                        dq_des_.segment<3>(3 * leg).setZero();
                } else {
                    dq_des_.segment<3>(3 * leg).setZero();
                }
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
            // 名义关节角 = 步态足端 IK 的 q_des_（不是站姿 stand_q）。
            //
            // 支撑腿只有「足端零加速度」3 行约束，而髋俯仰与膝关节可以在「足端不动」的
            // 前提下互相补偿（自运动），所以支撑腿关节构型在约束的零空间里是自由的。
            // 又因为 tau_ff = τ_wbc − pd 让总力矩恰等于 τ_wbc，PD 被完全抵消、对关节
            // 位置没有任何权威 —— 于是自运动无人约束，qerr 长期停在 0.22 rad，pd 常驻
            // 20+ N·m，稍有扰动就顶穿 ±30，tau_ff 饱和后力矩不再等于 τ_wbc，随即失稳。
            // WBC 的姿势任务（w_posture_stance）就是补这个位置权威的入口，它的参考必须
            // 是**步态给出的关节目标** q_des_；用 stand_q 会在行走时反着拉。
            win.q_nominal = q_des_;
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
            // 四足的**规划世界足位/足速**都给出来：摆动腿用于足端 PD，支撑腿用于接触
            // 约束的位置/速度反馈（`kp_stance_pos` / `kd_stance_pos`）。两者共用同一份
            // 计划，保证一致。
            // 支撑腿的速度参考必须是规划值而不是零：原地转向时足端要沿 ω×r 滑动，
            // 阻尼到零等于持续输入反向水平力（力臂 = 质心高度），是转向「涡动」的诱因。
            for (int leg = 0; leg < kNumLegs; ++leg) {
                win.foot_pos_des_world[leg] = estimate_.base_pos_w + R_ref * legs[leg].foot_pos;
                win.foot_vel_des_world[leg] = R_ref * legs[leg].foot_vel;
            }
            out.stance_count = 0;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (win.stance[leg]) {
                    ++out.stance_count;
                    continue;
                }
                win.foot_des_world[leg] = win.foot_pos_des_world[leg];
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
        out.joint_cmd.col(3) = dq_des_.cast<float>();  // 期望关节速度（步态足端速度反解，站立为 0）
        for (int j = 0; j < 12; ++j) {
            const double pd = kp * (q_des_(j) - in.q(j)) + kd * (dq_des_(j) - in.dq(j));
            out.tau_pd(j) = pd;
            // WBC 工作时：反解 τ_ff 使**总力矩** = τ_wbc（执行器总力矩 = PD + τ_ff）。
            // WBC 未工作（起立斜坡/降级/故障）时必须 τ_ff = 0，否则等于把 PD 又反向叠加一次
            // （实测：起立段 τ_ff=−pd 被 ±30 夹住后总力矩多出 −30 N·m，机器人被推翻）。
            double tau_ff = 0.0;
            if (wbc_active_) {
                // 注：曾经试过去掉下面这次夹取（理由：执行器端只对总力矩限幅，而 τ_wbc 已被
                // WBC 约束在 ±tau_max 内，理论上不夹时长驱力矩恰等于 τ_wbc）。实测反而更差：
                // 0.45 m/s 下 τ_ff 峰值从 30 涨到 60 N·m，walk 段 6.5 s 就倾到 13.6°、8.0 s 失稳
                // （带夹取时能撑到 12.9 s）。原因是 pd 项本身就有 5–16 N·m（kp=50 下对应
                // 0.1–0.3 rad 的关节跟踪误差），τ_ff = τ_wbc − pd 的绝对值因此经常超 30；
                // 放开夹取等于把这个 PD 失配整包灌进前馈，反而放大扰动。
                // 结论：τ_ff 顶限是「关节跟踪不足」的**症状**而非独立 bug，应从 kp/kd 与参考
                // 轨迹的激进度上治（见 docs/05 的 PD 增益扫描），此处保留夹取。
                tau_ff = out.tau_wbc(j) - pd;
                // 可选：把一部分/全部 PD 叠加到 WBC 之上：
                //   τ_applied = τ_wbc + pd_extra·pd
                // （τ_ff = τ_wbc − pd + pd_extra·pd）。pd_extra=0 就是原来的「完全抵消 PD」。
                // 对齐 legged_control 的执行律 τ = τ_wbc + kp(q_des−q) + kd(dq_des−dq)：
                // 支撑腿只有加速度级接触约束、没有位置权威（腿会整体滑移），叠加 PD 是最
                // 直接的位置反馈；代价是 WBC 的等效力矩被 PD 顶偏，需要小步验证。
                if (p.pd_extra > 0.0) {
                    const int leg = j / 3;
                    if (!p.pd_extra_stance_only || out.stance[static_cast<int>(leg)])
                        tau_ff += p.pd_extra * pd;
                }
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

    /// 把角度归一化到 (−π, π]，避免偏航参考长期积分后越界。
    static double WrapPi(double a) {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0.0) a += 2.0 * M_PI;
        return a - M_PI;
    }

    void ClipToJointLimits(Vec12d* q) const {
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int j = 0; j < 3; ++j) {
                const int idx = 3 * leg + j;
                const double lo = model_.geo.joint_lower(j), hi = model_.geo.joint_upper(j);
                (*q)(idx) = std::max(lo, std::min(hi, (*q)(idx)));
            }
    }

    /// 应用步态配置：绑定三段式调度表，并按 nominal_x 平移标称足位。
    /// 幂等：重复调用只补足与已生效值的差值，不会累积叠加。
    void ApplyGaitConfig() {
        gait_.SetSchedule(params_.use_gait_schedule ? &params_.gait_schedule : nullptr);
        const double dx = params_.nominal_x - applied_nominal_x_;
        if (std::fabs(dx) > 1e-12) {
            gait_.ShiftNominal(Vec3d(dx, 0.0, 0.0));
            applied_nominal_x_ = params_.nominal_x;
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
    double applied_nominal_x_ = 0.0;  // ApplyGaitConfig 已施加的标称前后偏移
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
    double yaw_ref_ = 0.0;              // 偏航角参考（世界系，连续不归一化，1 kHz 积分）
    double last_yaw_ref_t_ = 0.0;
    bool yaw_ref_initialized_ = false;
    double yaw_meas_cont_ = 0.0;        // 连续（不归一化）的测量偏航角
    double yaw_meas_prev_ = 0.0;
    bool yaw_meas_initialized_ = false;
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
