/**
 * @file 04_weighted_wbc.hpp
 * @brief S7 统一 Weighted-QP 全身控制（WBC）。
 *
 * 理论对应：docs/03 第 5 章（WBC）、附录 D；实现参考 docs/03 第 11 章（Weighted-QP WBC）。
 * 求解器：本工程自研 `qp_ipm.hpp`（原-对偶内点，零第三方依赖；见 qp_ipm.hpp 头注释说明为何不用主动集）。
 *
 * 决策变量 x ∈ R^42（与 MuJoCo qvel 参数化一致）：
 *   [0,18) 全身广义加速度 q̈ = [a_world(3); α_body(3); ddq(12)]
 *   [18,30) 四脚接触力 f（世界系，FL/FR/HL/HR 各 3）
 *   [30,42) 关节力矩 τ（12）
 *
 * 硬约束（**行布局固定**，便于诊断与后续换成热启动求解器）：
 *   等式 30 行 = 18(EoM) + 4×3(每腿 3 行)：
 *     EoM      : M·q̈ − J_allᵀ·f − Sᵀ·τ = −h                     （18 行）
 *     支撑腿   : Jc_i·q̈ = −Jdotc_i·q̇   （接触点加速度为零；**必须含 Jdot·v 项**）
 *     摆动腿   : f_i = 0
 *   不等式 20 行（≥ 形式）= 4×5 摩擦锥（金字塔）：fz ≥ 0、μfz ± fx ≥ 0、μfz ± fy ≥ 0；
 *     摆动腿的 5 行置为「0 ≥ −1」的空行（恒满足，保持布局不变、数值上无害）。
 *   不等式 4 行（可选）= 每脚法向力上限 fz ≤ fz_max（`foot_force_max > 0` 时启用；摆动腿同为空行）。
 *   不等式 24 行 = 关节力矩上下界（±τ_max，以 ≥ 行表达）。
 *
 * 软任务（代价 Σ w·‖A·x − b‖²）：
 *   ① MPC 第 0 节点逐脚力跟踪（`w_f_ref`，**默认很弱**：参考工程实测强跟踪会拉偏平衡，必要时置 0）
 *   ② 基座 6D 加速度跟踪（`w_base`，来自 SRBD-MPC 的净力/净力矩，见 ComputeBaseAccelDesired）
 *   ③ 摆动足任务空间 PD（`w_swing`, kp/kd = `kp_swing`/`kd_swing`）
 *   ④ 摆动腿关节接近名义站姿（`w_posture`）
 *   ⑤ 力矩努力正则（`w_tau`，零空间性质，默认很小）
 *
 * 与低层执行律的关系（docs/04 S7 的风险项）：仿真/真机的关节命令是
 *   τ_applied = kp·(q_des − q) + kd·(dq_des − dq) + tau_ff
 * 若要把 WBC 的 τ 真正作用到机器人上，必须用**同一帧实测 q,dq** 反解
 *   tau_ff = τ_wbc − kp·(q_des − q) − kd·(dq_des − dq)
 * 否则会与 PD 项重复。本模块只输出 τ_wbc，反解由 S8 的控制器完成。
 */
#pragma once

#include "00_lite3_model.hpp"
#include "03_srbd_mpc.hpp"
#include "qp_ipm.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <vector>

namespace mpc_wbc {

// 决策变量与行数布局（固定）
constexpr int kWbcQdd = kNumGenVel;                    // 18
constexpr int kWbcNf = kNumContactForces;              // 12
constexpr int kWbcNtau = kNumJoints;                   // 12
constexpr int kWbcNvar = kWbcQdd + kWbcNf + kWbcNtau;  // 42
constexpr int kWbcNEq = kNumGenVel + kNumContactForces;  // 30
constexpr int kWbcNConeRows = 5 * kNumLegs;             // 20
constexpr int kWbcNCapRows = kNumLegs;                  // 4（可选）
constexpr int kWbcNTauRows = 2 * kNumJoints;            // 24
constexpr int kWbcNGe = kWbcNConeRows + kWbcNCapRows + kWbcNTauRows;

struct WbcParams {
    // 任务权重。
    // **量级参考 legged_control（go1）：swingLeg=100、baseAccel=1、contactForce=0.01**，
    // 即摆动足任务的优先级远高于基座加速度任务。旧默认（base=1e3、swing=1e2）恰好
    // 反过来（基座压摆动 10 倍），实测摆动足净空只有命令值的 25%、FL/HR 有 85–90%
    // 的摆动时间拖地。
    double w_base = 1.0;     // 基座 6D 加速度任务
    double w_swing = 100.0;  // 摆动足任务空间 PD
    // MPC 逐脚力跟踪权重（**默认 0 = 关闭**）。docs/04 把它列为软任务 ①，但它与基座加速度
    // 任务直接竞争：参考工程实测「强跟踪会把平衡（基座任务）拉偏」，最终也把它去掉了。
    // 需要时（例如想让 WBC 尽量复现 MPC 的分配）显式打开，并注意基座跟踪误差会变大。
    double w_f_ref = 0.0;
    // 摆动腿关节站姿任务（**默认 0 = 关闭**）。它与摆动足笛卡尔任务是**同一组关节**
    // 上的两个目标，会直接抢权重：对角支撑 + qvel≠0 实测（w_swing=1e2）
    //   w_posture=0   → 摆动任务残差 0.033 m/s²、基座误差 0.193
    //   w_posture=1   → 摆动任务残差 3.92  m/s²、基座误差 0.745（两项同时变差）
    // 参考工程（09_weighted_wbc）没有该任务，只有「基座 6D + 摆动足」两项。需要时显式打开。
    double w_posture = 0.0;
    // 力矩努力正则（零空间二次项，**默认 0 = 关闭**）。它不是零空间的：w_tau=1e-3 已足以把
    // 基座任务从 q̈_x=0.5 拉到 0.4999869（误差 1.3e-5，w_tau=0 时 4.2e-9），因为「跟准基座」
    // 与「省力矩」在同一组接触力/力矩上折中；参考工程实测它还会造成承力甩锅与静立缓漂。
    double w_tau = 0.0;
    // 任务增益（参考 legged_control go1：kp=350、kd=37，kd/kp≈0.106 近似临界阻尼）
    double kp_swing = 350.0;
    double kd_swing = 37.0;
    double kp_posture = 25.0;
    double kd_posture = 2.0;
    // 物理与限幅
    double mu = 0.6;                 // 摩擦系数
    double tau_max = 30.0;           // 关节力矩限幅 N·m（MuJoCo ctrlrange ±30）
    double foot_force_max = 200.0;   // 单脚法向力上限 N（≤0 关闭该约束）
    double gravity = 9.81;           // 重力加速度（沿世界系 −z）；接入 S8 时与仿真/真机统一
    double contact_accel_max = 200.0;  // 接触加速度目标限幅（防任务 PD 爆冲）[m/s²]
    double reg = 1e-8;               // Hessian 正则（保证 KKT 可解）
    QpIpmOptions qp;                 // 内点法选项
    double max_solution_age = 0.01;  // 求解失败后沿用旧解的最长时间 [s]
};

struct WbcInput {
    // 状态
    Vec12d q = Vec12d::Zero();
    Vec18d qvel = Vec18d::Zero();
    Mat3d R_wb = Mat3d::Identity();
    std::array<bool, kNumLegs> stance = {{true, true, true, true}};
    std::array<Vec3d, kNumLegs> foot_pos_world{};  // 当前足端（球心）世界位置
    std::array<Vec3d, kNumLegs> foot_vel_world{};  // 当前足端世界速度（可由 Jc·q̇ 得到）

    // 摆动足目标（支撑腿忽略）
    std::array<Vec3d, kNumLegs> foot_des_world{};
    std::array<Vec3d, kNumLegs> foot_vel_des_world{};

    // 基座期望加速度（来自 SRBD-MPC，见 ComputeBaseAccelDesired）
    Vec3d a_des_world = Vec3d::Zero();
    Vec3d alpha_des_body = Vec3d::Zero();

    // MPC 第 0 节点逐脚接触力（世界系，软参考）
    std::array<Vec3d, kNumLegs> foot_force_ref{};

    // 站姿参考
    Vec12d q_nominal = Lite3Model::DefaultStandJointAngles();
};

struct WbcOutput {
    enum class Status { kOk, kStale, kFailed };

    Status status = Status::kFailed;
    Vec18d qdd = Vec18d::Zero();                    // 全身广义加速度
    std::array<Vec3d, kNumLegs> foot_force{};       // 四脚接触力（世界系）
    Vec12d tau = Vec12d::Zero();                    // 关节力矩

    // 诊断（每帧重算，用于验收与故障排查）
    double eom_residual = 0.0;      // ‖M q̈ + h − Jᵀf − Sᵀτ‖∞
    double contact_residual = 0.0;  // 支撑腿‖Jc q̈ + Jdotc q̇‖∞ / 摆动腿‖f‖∞ 的最大值
    double cone_violation = 0.0;    // 摩擦锥最大违反（>0 即违反）
    double tau_abs_max = 0.0;       // 最大关节力矩绝对值
    double base_accel_error = 0.0;  // ‖q̈[0:6] − [a_des; α_des]‖∞
    double swing_accel_error = 0.0; // 摆动软任务残差 ‖Jc q̈ − b_task‖∞（b_task 含 −J̇q̇，与 QP 一致）

    int iters = 0;
    double qp_gap = 0.0;
    double qp_dual_res = 0.0;
    double solve_ms = 0.0;

    bool ok() const { return status != Status::kFailed; }
    bool fresh() const { return status == Status::kOk; }
};

/// 由 SRBD-MPC 输出换算 WBC 的基座期望加速度（共享函数，避免多处重复推导）。
///   a_world     = F_net/m + g          （质心线加速度）
///   α_body      = Rᵀ·I_com⁻¹·(M_net − ω × (I_com·ω))
/// 近似说明：SRBD 认为整机作为一个刚体随基座一起转动，故基座角加速度 = 质心角加速度；
/// 基座原点线加速度与质心线加速度之差（α×r + ω×ω×r）被忽略（质心偏置 ≤ 1 cm）。
inline void ComputeBaseAccelDesired(const SrbdMpcOutput& mpc_out, const Mat3d& inertia_com_world,
                                    const Vec3d& omega_body, const Mat3d& R_wb, double mass,
                                    double gravity, Vec3d* a_world_out, Vec3d* alpha_body_out) {
    const Vec3d I_omega = inertia_com_world * omega_body;
    const Vec3d alpha_world =
        inertia_com_world.ldlt().solve(mpc_out.net_moment - omega_body.cross(I_omega));
    if (a_world_out) *a_world_out = mpc_out.net_force / mass + Vec3d(0.0, 0.0, -gravity);
    if (alpha_body_out) *alpha_body_out = R_wb.transpose() * alpha_world;
}

/// S7 统一 Weighted-QP WBC。
class WeightedWbc {
public:
    explicit WeightedWbc(const WbcParams& params = WbcParams())
        : params_(params), model_() {}

    const WbcParams& params() const { return params_; }
    void set_params(const WbcParams& p) { params_ = p; }

    void Reset() {
        have_solution_ = false;
        last_solution_time_ = 0.0;
        last_output_ = WbcOutput();
    }

    /// 组装 QP 时用到的模型量（暴露出来便于测试独立交叉验证）。
    struct QpTerms {
        Mat18d M;
        Vec18d h;
        Mat12x18d Jc;
        VecContact12d Jdotv;
    };

    /// 组装后的 QP：min ½xᵀHx + gᵀx s.t. A_eq x = b_eq, A_ge x ≥ b_ge。
    struct Qp {
        Eigen::MatrixXd H;
        Eigen::VectorXd g;
        Eigen::MatrixXd A_eq;
        Eigen::VectorXd b_eq;
        Eigen::MatrixXd A_ge;
        Eigen::VectorXd b_ge;
        bool valid = false;
    };

    /// 只组装、不求解。返回 false 表示「无支撑腿」保护已触发（此时 qp->valid == false）。
    bool BuildQp(const WbcInput& in, Qp* qp, QpTerms* terms = nullptr) const {
        QpTerms t;
        t.M = model_.MassMatrix(in.q, in.R_wb);
        const double gravity_vec[3] = {0.0, 0.0, -params_.gravity};
        t.h = model_.BiasForce(in.q, in.qvel, in.R_wb,
                               Vec3d(gravity_vec[0], gravity_vec[1], gravity_vec[2]));
        t.Jc = model_.ContactJacobianWorld(in.q, in.R_wb);
        t.Jdotv = model_.ContactBiasAccel(in.q, in.R_wb, in.qvel);
        if (!AssembleQp(in, t, qp)) return false;
        if (terms) *terms = t;
        return true;
    }

    /// 求解一帧。t 为当前时间 [s]（用于判断沿用旧解是否过期）。
    WbcOutput Solve(double t, const WbcInput& in) {
        const auto t0 = std::chrono::steady_clock::now();
        WbcOutput out = SolveOnce(in);
        if (!out.fresh()) return Finish(out, t0, t);

        have_solution_ = true;
        last_solution_time_ = t;
        last_output_ = out;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    /// 只求一次、不涉及旧解兜底（给单元测试用）。
    WbcOutput SolveOnce(const WbcInput& in) const {
        WbcOutput out;
        Qp qp;
        QpTerms terms;
        if (!BuildQp(in, &qp, &terms)) {
            // 保护：一条支撑腿都没有时，「接触力=0 的自由落体解」在数学上仍然可行，但对控制无意义
            // （参考工程同样显式拒绝）。置 kFailed，由上层的失败兜底处理。
            out.status = WbcOutput::Status::kFailed;
            return out;
        }
        const Mat18d& M = terms.M;
        const Vec18d& h = terms.h;
        const Mat12x18d& Jc = terms.Jc;
        const VecContact12d& Jdotv = terms.Jdotv;
        const WbcParams& p = params_;
        QpProblem qp_problem;
        qp_problem.H = qp.H;
        qp_problem.g = qp.g;
        qp_problem.A_eq = qp.A_eq;
        qp_problem.b_eq = qp.b_eq;
        qp_problem.A_ge = qp.A_ge;
        qp_problem.b_ge = qp.b_ge;
        const QpIpmResult res = SolveQpIpm(qp_problem, p.qp, warm_start_ ? &last_x_ref_ : nullptr);

        out.iters = res.info.iters;
        out.qp_gap = res.info.gap;

        out.iters = res.info.iters;
        out.qp_gap = res.info.gap;
        out.qp_dual_res = res.info.dual_res;
        if (!res.ok()) {
            out.status = WbcOutput::Status::kFailed;
            return out;
        }

        out.qdd = res.x.head<kWbcQdd>();
        for (int leg = 0; leg < kNumLegs; ++leg)
            out.foot_force[leg] = res.x.segment<3>(kWbcQdd + 3 * leg);
        out.tau = res.x.segment<kWbcNtau>(kWbcQdd + kWbcNf);

        last_x_ref_ = res.x;  // 下一帧的热启动初值

        // ---- 诊断（用未正则的物理量重算）----
        Vec18d tau_gen = Vec18d::Zero();
        tau_gen.tail<kWbcNtau>() = out.tau;
        Eigen::VectorXd f_all(kWbcNf);
        for (int leg = 0; leg < kNumLegs; ++leg) f_all.segment<3>(3 * leg) = out.foot_force[leg];
        out.eom_residual =
            (M * out.qdd + h - Jc.transpose() * f_all - tau_gen).cwiseAbs().maxCoeff();

        double contact_res = 0.0, cone_viol = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d f = out.foot_force[leg];
            if (in.stance[leg]) {
                const Vec3d acc = Jc.block<3, kWbcQdd>(3 * leg, 0) * out.qdd +
                                  Jdotv.segment<3>(3 * leg);
                contact_res = std::max(contact_res, acc.cwiseAbs().maxCoeff());
                const double viol[5] = {-f.z(),        f.x() - p.mu * f.z(), -f.x() - p.mu * f.z(),
                                        f.y() - p.mu * f.z(), -f.y() - p.mu * f.z()};
                for (double v : viol) cone_viol = std::max(cone_viol, v);
            } else {
                contact_res = std::max(contact_res, f.cwiseAbs().maxCoeff());
            }
        }
        out.contact_residual = contact_res;
        out.cone_violation = cone_viol;
        out.tau_abs_max = out.tau.cwiseAbs().maxCoeff();
        Eigen::Matrix<double, 6, 1> base_des;
        base_des << in.a_des_world, in.alpha_des_body;
        out.base_accel_error = (out.qdd.head<6>() - base_des).cwiseAbs().maxCoeff();
        double swing_err = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            const Vec3d err = in.foot_des_world[leg] - in.foot_pos_world[leg];
            const Vec3d edot = in.foot_vel_des_world[leg] - in.foot_vel_world[leg];
            // 必须与 BuildQp ② 的 b 完全相同（含 `− J̇q̇` 与同一限幅），否则这里量的不是
            // QP 真正在最小化的那个残差：漏掉 J̇q̇ 时即使 w_swing 扫 5 个数量级残差也只
            // 降 1.4×（因为剩下的全是 J̇q̇ 本身），看起来像「权重不起作用」。
            Vec3d target = p.kp_swing * err + p.kd_swing * edot - Jdotv.segment<3>(3 * leg);
            for (int c = 0; c < 3; ++c)
                target(c) = std::min(std::max(target(c), -p.contact_accel_max), p.contact_accel_max);
            const Vec3d acc = Jc.block<3, kWbcQdd>(3 * leg, 0) * out.qdd;
            swing_err = std::max(swing_err, (acc - target).cwiseAbs().maxCoeff());
        }
        out.swing_accel_error = swing_err;

        out.status = WbcOutput::Status::kOk;
        return out;
    }

    /// 组装硬约束与软任务，写入 qp。返回 false = 无支撑腿。
    bool AssembleQp(const WbcInput& in, const QpTerms& t, Qp* qp) const {
        const WbcParams& p = params_;
        bool any_stance = false;
        for (int leg = 0; leg < kNumLegs; ++leg) any_stance = any_stance || in.stance[leg];
        if (!any_stance) return false;
        qp->valid = true;
        const Mat18d& M = t.M;
        const Mat12x18d& Jc = t.Jc;
        const VecContact12d& Jdotv = t.Jdotv;
        // ---- 硬约束 ----
        Eigen::MatrixXd& A_eq = qp->A_eq = Eigen::MatrixXd::Zero(kWbcNEq, kWbcNvar);
        Eigen::VectorXd& b_eq = qp->b_eq = Eigen::VectorXd::Zero(kWbcNEq);
        // EoM: M q̈ − J_allᵀ f − Sᵀ τ = −h
        A_eq.block<kWbcQdd, kWbcQdd>(0, 0) = M;
        A_eq.block<kWbcQdd, kWbcNf>(0, kWbcQdd) = -Jc.transpose();
        // S = [0₆ₓ₁₂; I₁₂] → Sᵀ 的非零块在行 6..17（对应 12 个关节），故 τ 的系数为 −1。
        A_eq.block<kWbcNtau, kWbcNtau>(kWbcQdd - kWbcNtau, kWbcQdd + kWbcNf) =
            -Eigen::MatrixXd::Identity(kWbcNtau, kWbcNtau);
        b_eq.head<kWbcQdd>() = -t.h;
        // 每腿 3 行：支撑 → Jc q̈ = −Jdotc q̇；摆动 → f_i = 0
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int r = kWbcQdd + 3 * leg;
            if (in.stance[leg]) {
                A_eq.block<3, kWbcQdd>(r, 0) = Jc.block<3, kWbcQdd>(3 * leg, 0);
                b_eq.segment<3>(r) = -Jdotv.segment<3>(3 * leg);
            } else {
                A_eq.block<3, 3>(r, kWbcQdd + 3 * leg).setIdentity();
            }
        }

        // 不等式（≥ 形式）
        Eigen::MatrixXd& A_ge = qp->A_ge = Eigen::MatrixXd::Zero(kWbcNGe, kWbcNvar);
        Eigen::VectorXd& b_ge = qp->b_ge = Eigen::VectorXd::Zero(kWbcNGe);
        const double mu = p.mu;
        // 摩擦金字塔：fz ≥ 0；μfz − fx ≥ 0；μfz + fx ≥ 0；μfz − fy ≥ 0；μfz + fy ≥ 0
        const double cone[5][3] = {{0.0, 0.0, 1.0},
                                   {-1.0, 0.0, mu},
                                   {1.0, 0.0, mu},
                                   {0.0, -1.0, mu},
                                   {0.0, 1.0, mu}};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int r0 = 5 * leg;
            if (in.stance[leg]) {
                for (int k = 0; k < 5; ++k)
                    for (int c = 0; c < 3; ++c)
                        A_ge(r0 + k, kWbcQdd + 3 * leg + c) = cone[k][c];
            } else {
                // 空行：0 ≥ −1，恒满足；保持行数固定且不产生 0/0（s = 1 远离边界）
                for (int k = 0; k < 5; ++k) b_ge(r0 + k) = -1.0;
            }
        }
        // 单脚法向力上限
        const int r_cap = kWbcNConeRows;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (p.foot_force_max > 0.0 && in.stance[leg]) {
                A_ge(r_cap + leg, kWbcQdd + 3 * leg + 2) = -1.0;
                b_ge(r_cap + leg) = -p.foot_force_max;
            } else {
                b_ge(r_cap + leg) = -1.0;  // 空行
            }
        }
        // 关节力矩上下界：τ_j ≥ −τ_max 与 −τ_j ≥ −τ_max
        const int r_tau = kWbcNConeRows + kWbcNCapRows;
        for (int j = 0; j < kWbcNtau; ++j) {
            const int col = kWbcQdd + kWbcNf + j;
            A_ge(r_tau + j, col) = 1.0;
            A_ge(r_tau + kWbcNtau + j, col) = -1.0;
            b_ge(r_tau + j) = -p.tau_max;
            b_ge(r_tau + kWbcNtau + j) = -p.tau_max;
        }

        // ---- 软任务：累加 H = Σ w·AᵀA，g = −Σ w·Aᵀb ----
        Eigen::MatrixXd& H = qp->H = Eigen::MatrixXd::Zero(kWbcNvar, kWbcNvar);
        Eigen::VectorXd& g = qp->g = Eigen::VectorXd::Zero(kWbcNvar);
        auto add_task = [&](const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double w) {
            if (w <= 0.0) return;
            H.noalias() += w * (A.transpose() * A);
            g.noalias() -= w * (A.transpose() * b);
        };

        // ① 基座 6D 期望加速度
        {
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, kWbcNvar);
            A.block<6, 6>(0, 0).setIdentity();
            Eigen::VectorXd b(6);
            b << in.a_des_world, in.alpha_des_body;
            add_task(A, b, p.w_base);
        }
        // ② 摆动足任务空间 PD：J q̈ = kp(p_des − p) + kd(v_des − v) − J̇q̇
        // 最后一项必须有：左脚加速度是 a = J q̈ + J̇q̇，漏掉 J̇q̇ 会让任务目标偏
        // 一个与向心/科氏同量级的量（摆动腿关节速度 5 rad/s、力臂 0.2 m 时约
        // 5 m/s²，与 kp·err 同量级），表现为摆动足跟不上、净空只有命令值的 25%。
        // 参考工程 legged_control 的 formulateSwingLegTask 同样减了 dj*v。
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, kWbcNvar);
            A.block<3, kWbcQdd>(0, 0) = Jc.block<3, kWbcQdd>(3 * leg, 0);
            const Vec3d err = in.foot_des_world[leg] - in.foot_pos_world[leg];
            const Vec3d edot = in.foot_vel_des_world[leg] - in.foot_vel_world[leg];
            Vec3d b = p.kp_swing * err + p.kd_swing * edot - Jdotv.segment<3>(3 * leg);
            for (int c = 0; c < 3; ++c)  // 限幅，防大误差时任务目标爆冲
                b(c) = std::min(std::max(b(c), -p.contact_accel_max), p.contact_accel_max);
            add_task(A, b, p.w_swing);
        }
        // ③ MPC 逐脚力参考（弱）
        if (p.w_f_ref > 0.0) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.stance[leg]) continue;
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, kWbcNvar);
                A.block<3, 3>(0, kWbcQdd + 3 * leg).setIdentity();
                add_task(A, in.foot_force_ref[leg], p.w_f_ref);
            }
        }
        // ④ 摆动腿关节接近名义站姿
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            for (int k = 0; k < 3; ++k) {
                const int j = 3 * leg + k;
                const double b = p.kp_posture * (in.q_nominal(j) - in.q(j));
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, kWbcNvar);
                A(0, 6 + j) = 1.0;
                Eigen::VectorXd bv = Eigen::VectorXd::Constant(1, b);
                add_task(A, bv, p.w_posture);
            }
        }
        // ⑤ 力矩努力正则（零空间）
        if (p.w_tau > 0.0) {
            for (int j = 0; j < kWbcNtau; ++j) {
                const int col = kWbcQdd + kWbcNf + j;
                H(col, col) += p.w_tau;
            }
        }
        for (int i = 0; i < kWbcNvar; ++i) H(i, i) += p.reg;
        return true;
    }

    /// 记录上一帧解作为下一帧的热启动初值（内点法用它降迭代次数，不属于收敛保证）。
    void set_warm_start_enabled(bool en) { warm_start_ = en; }

private:
    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    WbcOutput Finish(WbcOutput& out, const std::chrono::steady_clock::time_point& t0, double t) {
        if (have_solution_ && (t - last_solution_time_) <= params_.max_solution_age) {
            WbcOutput stale = last_output_;
            stale.status = WbcOutput::Status::kStale;
            stale.solve_ms = ElapsedMs(t0);
            return stale;
        }
        out.status = WbcOutput::Status::kFailed;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    WbcParams params_;
    Lite3Model model_;
    bool warm_start_ = true;
    mutable Eigen::VectorXd last_x_ref_;  // 上一帧解（内点法用它当迭代初值）
    bool have_solution_ = false;
    double last_solution_time_ = 0.0;
    WbcOutput last_output_;
};

}  // namespace mpc_wbc
