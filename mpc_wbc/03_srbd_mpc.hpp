/**
 * @file 03_srbd_mpc.hpp
 * @brief S6 预测时域 SRBD-MPC（单刚体近似 + N 步滚动预测）。
 *
 * 理论对应：docs/03 第 4 章（MPC 与 SRBD）、第 10 章（ComPreviewMpc）。
 *
 * 模型（SRBD：把整机看成一个刚体，四腿作用浓缩为四个接触力）：
 *   m·a_com  = Σ f_i + m·g
 *   I_com·ω̇ = Σ (p_fi − p_com) × f_i − ω × (I_com·ω)
 * 状态 x = [p_com(3); v_com(3); rpy(3); ω_body(3)]，用等时间步 Δt 离散：
 *   p⁺ = p + Δt·v + ½Δt²·a
 *   v⁺ = v + Δt·a
 *   rpy⁺ = rpy + Δt·T(rpy₀)·ω   （T 为 ZYX 欧拉角速率矩阵，取当前姿态后在本时域内冻结）
 *   ω⁺ = ω + Δt·I_com⁻¹·M_body
 *
 * 决策变量不是力本身，而是**每只支撑脚 × 每预测节点 × 每锥生成方向**的 λ ≥ 0：
 *   f_i = Σ_j λ_j·d_j,  d_j = (μ·cosφ_j, μ·sinφ_j, 1)
 * 4 个生成方向（φ = 0/90/180/270°）构成的四棱锥与 docs/03 §5.5 的五判据
 * （f_z ≥ 0、|f_x| ≤ μf_z、|f_y| ≤ μf_z）等价。摆动脚在该节点**不生成变量**
 * → 摆动脚受力结构性为 0；λ 上界取 f_z,max/4 → 单脚法向力 ≤ f_z,max。
 *
 * 求解：把系统对每个 λ 的**冲激响应**拼成灵敏度矩阵 S（12N × V），
 * 代价化为 ½λᵀ(SᵀWS + R)λ + (SᵀW·e_free)ᵀλ，交给盒约束 QP（box_qp.hpp）。
 * 灵敏度递推必须先「用齐次动力学 A 前推一步」再叠加该步的 B·λ——顺序反了静载荷会算错
 * （docs/03 §10.2 的坑）。重力只出现在**自由轨迹**里，不在 A/B 中。
 *
 * 输出：**第 0 节点的逐脚力**（世界系）+ 绕质心的净力/净力矩（S7 WBC 的期望 wrench）
 * + 预测轨迹 + 求解状态（耗时/迭代/是否沿用旧解）。
 */
#pragma once

#include "00_lite3_model.hpp"
#include "box_qp.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace mpc_wbc {

/// 每个摩擦锥的生成方向数（4 → 四棱锥）。
constexpr int kConeGenerators = 4;

struct SrbdMpcParams {
    int N = 10;                    // 预测步数
    double dt = 0.05;              // 步长 [s]，总时域 N·dt
    double gravity = 9.81;         // 重力加速度 [m/s²]（沿世界系 −z）
    double mu = 0.6;               // 摩擦系数
    double foot_force_max = 200.0; // 单脚法向力上限 [N]
    double q_p_xy = 10.0;          // 质心水平位置代价
    double q_p_z = 200.0;          // 质心高度代价（远大于水平：不许上下跳）
    double q_v_xy = 10.0;          // 质心水平速度代价
    double q_v_z = 10.0;           // 质心竖直速度代价
    double q_rp = 50.0;            // roll/pitch 姿态代价
    double q_yaw = 20.0;           // 偏航角代价
    double q_w_rp = 5.0;           // roll/pitch 角速度（阻尼）
    double q_w_yaw = 2.0;          // 偏航角速度
    double r_lambda = 1e-6;        // λ 正则化（须远小于跟踪权重，否则会拉偏承重）
    BoxQpOptions qp;               // 盒约束 QP 选项
    bool warm_start = true;        // 用上一帧解按时域平移热启动
    double max_solution_age = 0.2; // 求解失败后沿用旧解的最长时间 [s]
};

struct SrbdMpcInput {
    // 当前状态
    Vec3d p_com = Vec3d::Zero();              // 质心位置（世界系）
    Vec3d v_com = Vec3d::Zero();              // 质心速度（世界系）
    Vec3d rpy = Vec3d::Zero();                // 基座姿态（ZYX 欧拉角，**测量值**）
    Vec3d omega = Vec3d::Zero();              // 基座角速度（机体系）
    Mat3d inertia_com_world = Mat3d::Zero();  // 绕质心的整机惯量（世界系）
    std::array<Vec3d, kNumLegs> foot_pos_world{};  // 四足足端作用点（世界系）

    // 命令
    Vec3d v_cmd = Vec3d::Zero();  // 期望质心速度（世界系，水平两维有效）
    double yaw_rate_cmd = 0.0;    // 期望偏航角速度 [rad/s]
    // 偏航角**参考**（世界系），由上层用 yaw_rate_cmd 积分得到。
    //
    // 绝不能用 `rpy.z()`（测量值）当参考：那样参考会随实际偏航一起漂，MPC 对偏航扰动
    // 没有任何回正激励，任何一次扰动造成的偏航都变成永久误差。实测闭环 5 分钟
    // ±0.30 m/s 往返 10 次（空转测试）累积 **−12.1°** 航向漂移，而开环位置环是 −0.03°；
    // 逐次排查发现每次**方向反转**都会踢出 −3～−5° 的偏航台阶，之后不再回正。
    double yaw_ref = 0.0;
    double height_cmd = 0.0;      // 期望质心高度 [m]

    // 未来 N 步接触计划（来自步态相位），长度须 ≥ N
    std::vector<std::array<bool, kNumLegs>> contact;
};

struct SrbdMpcOutput {
    enum class Status { kOk, kStale, kFailed };

    Status status = Status::kFailed;
    std::array<Vec3d, kNumLegs> foot_force{};  // 第 0 节点逐脚力（世界系 [N]）
    Vec3d net_force = Vec3d::Zero();           // 第 0 节点净力（世界系 [N]）
    Vec3d net_moment = Vec3d::Zero();          // 第 0 节点绕质心净力矩（世界系 [N·m]）
    std::vector<Vec3d> pred_p;                 // 预测质心位置（N 步）
    std::vector<Vec3d> pred_v;                 // 预测质心速度
    std::vector<Vec3d> pred_rpy;               // 预测姿态
    std::vector<Vec3d> pred_omega;             // 预测机体角速度

    int nvar = 0;              // 决策变量维数
    int iters = 0;             // QP 迭代次数
    double qp_residual = 0.0;  // QP 投影梯度残差
    double cost = 0.0;         // 最优代价
    double solve_ms = 0.0;     // 求解耗时 [ms]

    bool ok() const { return status != Status::kFailed; }  // 有可用结果（含沿用旧解）
    bool fresh() const { return status == Status::kOk; }   // 本帧真正解出
};

/// S6 预测时域 SRBD-MPC。
class SrbdPreviewMpc {
public:
    explicit SrbdPreviewMpc(const SrbdMpcParams& params = SrbdMpcParams())
        : params_(params), mass_(Lite3Model().TotalMass()) {}

    const SrbdMpcParams& params() const { return params_; }
    void set_params(const SrbdMpcParams& p) { params_ = p; }

    /// 整机质量（默认取 Lite3 模型刚体质量之和）。
    double mass() const { return mass_; }
    void set_mass(double m) { mass_ = m; }

    /// 清空热启动与旧解缓存。
    void Reset() {
        last_lambda_.resize(0);
        last_var_index_.clear();
        have_solution_ = false;
        last_solution_time_ = 0.0;
        last_output_ = SrbdMpcOutput();
    }

    /// ZYX 欧拉角速率矩阵：rpẏ = T(rpy)·ω_body。
    /// pitch 接近 ±90° 时该参数化奇异；本工程 tilt 很小，超出 ±1.5 rad 时退化为单位阵。
    static Mat3d EulerRateMatrix(const Vec3d& rpy) {
        const double r = rpy(0), p = rpy(1);
        if (std::fabs(p) > 1.5) return Mat3d::Identity();
        const double cp = std::cos(p), sp = std::sin(p);
        const double cr = std::cos(r), sr = std::sin(r);
        const double tp = sp / cp;
        Mat3d T;
        T << 1.0, sr * tp, cr * tp,
             0.0, cr, -sr,
             0.0, sr / cp, cr / cp;
        return T;
    }

    /// 求解一帧。t 为当前时间 [s]（用于判断沿用旧解是否过期）。
    SrbdMpcOutput Solve(double t, const SrbdMpcInput& in) {
        const auto t0 = std::chrono::steady_clock::now();
        SrbdMpcOutput out;
        const int N = std::min(params_.N, static_cast<int>(in.contact.size()));
        const double dt = params_.dt;
        const double m = mass_;
        if (N < 1 || m <= 0.0 || dt <= 0.0) return Finish(out, t0, t);

        // ---- 决策变量：每 (节点 k, 支撑脚 leg, 锥方向 gen) 一个 λ ----
        struct Var {
            int k, leg, gen;
        };
        std::vector<Var> vars;
        vars.reserve(static_cast<size_t>(N) * kNumLegs * kConeGenerators);
        for (int k = 0; k < N; ++k) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.contact[k][leg]) continue;  // 摆动脚：不生成变量 → f_i ≡ 0
                for (int g = 0; g < kConeGenerators; ++g) vars.push_back({k, leg, g});
            }
        }
        const int V = static_cast<int>(vars.size());
        out.nvar = V;
        if (V == 0) return Finish(out, t0, t);  // 全摆动：无可行解

        // 四棱锥生成方向：f = Σ λ_j d_j，d_j 的 z 分量恒为 1 → f_z = Σ λ_j ≥ 0。
        static const double kConeXY[kConeGenerators][2] = {
            {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
        std::array<Vec3d, kConeGenerators> gens;
        for (int g = 0; g < kConeGenerators; ++g) {
            gens[g] = Vec3d(params_.mu * kConeXY[g][0], params_.mu * kConeXY[g][1], 1.0);
        }

        // ---- 离散化：齐次 A（不含重力）与重力偏置 ----
        const Mat3d T = EulerRateMatrix(in.rpy);
        Eigen::Matrix<double, 12, 12> A = Eigen::Matrix<double, 12, 12>::Identity();
        A.block<3, 3>(0, 3) = dt * Mat3d::Identity();
        A.block<3, 3>(6, 9) = dt * T;

        Eigen::Matrix<double, 12, 1> x0;
        x0 << in.p_com, in.v_com, in.rpy, in.omega;

        Eigen::Matrix<double, 12, 1> gravity_term = Eigen::Matrix<double, 12, 1>::Zero();
        gravity_term.segment<3>(0) = Vec3d(0.0, 0.0, -0.5 * dt * dt * params_.gravity);
        gravity_term.segment<3>(3) = Vec3d(0.0, 0.0, -dt * params_.gravity);

        const Mat3d inertia_inv = in.inertia_com_world.inverse();
        if (!inertia_inv.allFinite() || !x0.allFinite()) return Finish(out, t0, t);

        // ---- 自由（无足力）轨迹 ----
        std::vector<Eigen::Matrix<double, 12, 1>> x_free(N);
        {
            Eigen::Matrix<double, 12, 1> x = x0;
            for (int k = 0; k < N; ++k) {
                x = A * x + gravity_term;
                if (!x.allFinite()) return Finish(out, t0, t);
                x_free[k] = x;
            }
        }

        // ---- 灵敏度矩阵 S（12N × V）：每个 λ 的冲激响应 ----
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(12 * N, V);
        for (int v = 0; v < V; ++v) {
            const Var& var = vars[v];
            const Vec3d d = gens[var.gen];
            const Vec3d r = in.foot_pos_world[var.leg] - in.p_com;
            const Vec3d alpha = inertia_inv * r.cross(d);

            Eigen::Matrix<double, 12, 1> B = Eigen::Matrix<double, 12, 1>::Zero();
            B.segment<3>(0) = (0.5 * dt * dt / m) * d;
            B.segment<3>(3) = (dt / m) * d;
            B.segment<3>(6) = 0.5 * dt * dt * (T * alpha);
            B.segment<3>(9) = dt * alpha;

            Eigen::Matrix<double, 12, 1> s = Eigen::Matrix<double, 12, 1>::Zero();
            for (int k = 0; k < N; ++k) {
                s = A * s;               // 先齐次推进
                if (k == var.k) s += B;  // 再叠加该步输入
                if (k >= var.k) S.block<12, 1>(12 * k, v) = s;
            }
        }

        // ---- 权重 + 参考轨迹 + 组装 QP ----
        Eigen::Matrix<double, 12, 12> W = Eigen::Matrix<double, 12, 12>::Zero();
        W.diagonal() << params_.q_p_xy, params_.q_p_xy, params_.q_p_z,
            params_.q_v_xy, params_.q_v_xy, params_.q_v_z,
            params_.q_rp, params_.q_rp, params_.q_yaw,
            params_.q_w_rp, params_.q_w_rp, params_.q_w_yaw;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(V, V);
        Eigen::VectorXd gvec = Eigen::VectorXd::Zero(V);
        for (int k = 0; k < N; ++k) {
            const double tk = dt * static_cast<double>(k + 1);
            // 参考位置按命令速度从当前质心推进；高度恒为期望值；姿态水平（yaw 按命令转）。
            Eigen::Matrix<double, 12, 1> ref;
            ref.segment<3>(0) = Vec3d(in.p_com.x() + in.v_cmd.x() * tk,
                                      in.p_com.y() + in.v_cmd.y() * tk, in.height_cmd);
            ref.segment<3>(3) = in.v_cmd;
            ref.segment<3>(6) = Vec3d(0.0, 0.0, in.yaw_ref + in.yaw_rate_cmd * tk);
            ref.segment<3>(9) = Vec3d(0.0, 0.0, in.yaw_rate_cmd);

            const Eigen::Matrix<double, 12, 1> e = x_free[k] - ref;
            const Eigen::MatrixXd Sk = S.block(12 * k, 0, 12, V);
            H.noalias() += Sk.transpose() * W * Sk;
            gvec.noalias() += Sk.transpose() * W * e;
        }
        for (int v = 0; v < V; ++v) H(v, v) += params_.r_lambda;

        // ---- 盒约束：λᵥ ≥ 0；Σᵥ λᵥ = f_z，故每方向上界 = f_z,max/4 ----
        const Eigen::VectorXd lb = Eigen::VectorXd::Zero(V);
        const Eigen::VectorXd ub =
            Eigen::VectorXd::Constant(V, params_.foot_force_max / kConeGenerators);

        // 热启动：上一帧的 λ 按「时域向前平移一格」映射到本帧变量。
        Eigen::VectorXd warm;
        const Eigen::VectorXd* warm_ptr = nullptr;
        if (params_.warm_start && last_lambda_.size() > 0) {
            warm = Eigen::VectorXd::Zero(V);
            for (int v = 0; v < V; ++v) {
                const Var& var = vars[v];
                const int want_k = (var.k == 0) ? 0 : var.k - 1;
                const auto it = last_var_index_.find(Key(want_k, var.leg, var.gen));
                if (it != last_var_index_.end() && it->second < last_lambda_.size()) {
                    warm(v) = last_lambda_(it->second);
                }
            }
            warm_ptr = &warm;
        }

        BoxQpInfo qp_info;
        const Eigen::VectorXd lambda =
            SolveBoxQp(H, gvec, lb, ub, warm_ptr, params_.qp, &qp_info);
        if (lambda.size() != V || !lambda.allFinite()) return Finish(out, t0, t);

        out.iters = qp_info.iters + qp_info.polish;
        out.qp_residual = qp_info.residual;

        // ---- 第 0 节点逐脚力 + 净力/净力矩 ----
        for (int leg = 0; leg < kNumLegs; ++leg) out.foot_force[leg].setZero();
        for (int v = 0; v < V; ++v) {
            if (vars[v].k != 0) continue;
            out.foot_force[vars[v].leg] += lambda(v) * gens[vars[v].gen];
        }
        for (int leg = 0; leg < kNumLegs; ++leg) {
            out.net_force += out.foot_force[leg];
            out.net_moment += (in.foot_pos_world[leg] - in.p_com).cross(out.foot_force[leg]);
        }

        // ---- 预测轨迹：用最优 λ 推进（含 ω×Iω 耦合与姿态相关的力矩换轴）----
        {
            const Vec3d I_omega = in.inertia_com_world * in.omega;
            Eigen::Matrix<double, 12, 1> x = x0;
            for (int k = 0; k < N; ++k) {
                Vec3d f = Vec3d::Zero(), moment = Vec3d::Zero();
                for (int v = 0; v < V; ++v) {
                    if (vars[v].k != k) continue;
                    const Vec3d fj = lambda(v) * gens[vars[v].gen];
                    f += fj;
                    moment += (in.foot_pos_world[vars[v].leg] - in.p_com).cross(fj);
                }
                const Mat3d R = RpyToRotation(Eigen::Vector3d(x.segment<3>(6)));
                const Vec3d alpha =
                    inertia_inv * (moment - R.transpose() * in.omega.cross(I_omega));
                const Vec3d a = Vec3d(0.0, 0.0, -params_.gravity) + f / m;
                Eigen::Matrix<double, 12, 1> xn = x;
                xn.segment<3>(0) = x.segment<3>(0) + dt * x.segment<3>(3) + 0.5 * dt * dt * a;
                xn.segment<3>(3) = x.segment<3>(3) + dt * a;
                xn.segment<3>(6) =
                    x.segment<3>(6) + dt * (EulerRateMatrix(x.segment<3>(6)) * x.segment<3>(9));
                xn.segment<3>(9) = x.segment<3>(9) + dt * alpha;
                x = xn;
                out.pred_p.push_back(x.segment<3>(0));
                out.pred_v.push_back(x.segment<3>(3));
                out.pred_rpy.push_back(x.segment<3>(6));
                out.pred_omega.push_back(x.segment<3>(9));
            }
        }
        out.cost = 0.5 * lambda.dot(H * lambda) + gvec.dot(lambda);

        // ---- 缓存热启动与旧解 ----
        last_lambda_ = lambda;
        last_var_index_.clear();
        for (int v = 0; v < V; ++v) last_var_index_[Key(vars[v].k, vars[v].leg, vars[v].gen)] = v;
        have_solution_ = true;
        last_solution_time_ = t;
        out.status = SrbdMpcOutput::Status::kOk;
        last_output_ = out;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

private:
    static std::array<int, 3> Key(int k, int leg, int gen) { return {k, leg, gen}; }

    struct ArrayHash {
        size_t operator()(const std::array<int, 3>& a) const {
            return static_cast<size_t>(a[0]) * 1000003u + static_cast<size_t>(a[1]) * 10007u +
                   static_cast<size_t>(a[2]);
        }
    };

    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    /// 收尾：本帧无有效解时沿用未过期的旧解，否则报失败。
    SrbdMpcOutput Finish(SrbdMpcOutput& out,
                         const std::chrono::steady_clock::time_point& t0, double t) {
        if (have_solution_ && (t - last_solution_time_) <= params_.max_solution_age) {
            SrbdMpcOutput stale = last_output_;  // pred_* 为旧解对应的预测，属已知行为
            stale.status = SrbdMpcOutput::Status::kStale;
            stale.solve_ms = ElapsedMs(t0);
            stale.nvar = out.nvar;
            return stale;
        }
        out.status = SrbdMpcOutput::Status::kFailed;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    SrbdMpcParams params_;
    double mass_ = 11.9376;
    Eigen::VectorXd last_lambda_;
    std::unordered_map<std::array<int, 3>, int, ArrayHash> last_var_index_;
    bool have_solution_ = false;
    double last_solution_time_ = 0.0;
    SrbdMpcOutput last_output_;
};

}  // namespace mpc_wbc
