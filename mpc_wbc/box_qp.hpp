/**
 * @file box_qp.hpp
 * @brief 盒约束 QP 求解器：min 0.5·xᵀHx + gᵀx,  lb ≤ x ≤ ub（H 对称半正定）。
 *
 * 用途：S6 预测时域 SRBD-MPC 的接触力 QP——决策是「每脚×每节点×每锥方向 λ ≥ 0」，
 * 摩擦锥被结构性满足（λ ≥ 0 且 Σλ = f_z），盒约束之外没有别的约束。零第三方依赖。
 *
 * 算法（三级，逐级兜底，最终取目标函数最小者）：
 *   ① 无约束解 x = −H⁻¹g（LDLT）。若落在盒内即全局最优，一次求解结束——本 MPC 多数
 *      场景属于这一类（解在盒内部，例如静立悬停、四脚支撑巡航）。
 *   ② **主动集法**：维护显式 active 集（贴在上下界的变量），free 集上解
 *      H_FF·d = −g_F，步长按盒边界截断；到 free 集驻点后按 KKT 乘子逐个移出违反者。
 *      起点优先取「无约束解截断到盒内」——它通常已经很接近最优，比从盒中心出发少走很多步
 *      （实测 100 帧 trot：mean 7.5 次迭代到残差 1e-13）。
 *   ③ 兜底 **带盒截断的坐标下降**（Gauss-Seidel：逐变量一维精确最小化再夹到盒内），
 *      单调下降、必收敛，仅前两级没收住时使用。
 *
 * KKT 判据统一用投影梯度残差 ‖x − proj(x − grad)‖∞：为 0 ⇔ KKT 成立，凸问题下即全局最优。
 * 残差与迭代次数通过 BoxQpInfo 回报，便于上层监控求解质量。
 *
 * 注意：本文件只解盒约束，不能承担带等式/一般不等式的 WBC（见 docs/04 S7）。
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mpc_wbc {

struct BoxQpOptions {
    int max_iters = 100;     // 投影牛顿外层迭代上限
    int polish_iters = 2000; // 兜底坐标下降迭代（扫描）上限
    double tol = 1e-10;      // 投影梯度残差容差
};

struct BoxQpInfo {
    int iters = 0;           // 投影牛顿迭代次数
    int polish = 0;          // 兜底投影梯度迭代次数
    double residual = 0.0;   // 最终投影梯度残差
    bool converged = false;
};

/// 求解盒约束 QP。x_init（可空）为热启动，长度须等于 lb.size()。
inline Eigen::VectorXd SolveBoxQp(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                                  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                  const Eigen::VectorXd* x_init = nullptr,
                                  const BoxQpOptions& opt = BoxQpOptions(),
                                  BoxQpInfo* info_out = nullptr) {
    BoxQpInfo info;
    const int n = static_cast<int>(H.rows());
    if (n <= 0) {
        if (info_out) *info_out = info;
        return Eigen::VectorXd();
    }

    auto project = [&](const Eigen::VectorXd& v) {
        Eigen::VectorXd o(n);
        for (int i = 0; i < n; ++i) o(i) = std::min(std::max(v(i), lb(i)), ub(i));
        return o;
    };
    auto grad_at = [&](const Eigen::VectorXd& v) { return H * v + g; };
    auto objective = [&](const Eigen::VectorXd& v) { return 0.5 * v.dot(H * v) + g.dot(v); };
    auto pg_residual = [&](const Eigen::VectorXd& v) {
        return (v - project(v - grad_at(v))).cwiseAbs().maxCoeff();
    };

    Eigen::VectorXd best;
    double best_obj = std::numeric_limits<double>::max();
    auto keep_best = [&](const Eigen::VectorXd& v) {
        const double f = objective(v);
        if (f < best_obj) {
            best_obj = f;
            best = v;
        }
    };

    // ---- ① 无约束解：落在盒内即全局最优 ----
    const Eigen::VectorXd x_unc = H.ldlt().solve(-g);
    if (x_unc.allFinite() && ((x_unc - project(x_unc)).cwiseAbs().maxCoeff() < 1e-12)) {
        info.residual = pg_residual(x_unc);
        info.converged = info.residual <= std::max(opt.tol, 1e-8);
        if (info_out) *info_out = info;
        return x_unc;
    }

    // ---- 起点：热启动（截断）优先，否则取盒中心 ----
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) x(i) = 0.5 * (lb(i) + ub(i));
    if (x_unc.allFinite()) x = project(x_unc);   // 无约束解截断到盒内，通常已很接近最优
    if (x_init && x_init->size() == n) {
        const Eigen::VectorXd cand = project(*x_init);
        if (objective(cand) < objective(x)) x = cand;
    }
    keep_best(x);

    // ---- ② 主动集法（投影牛顿步 + 盒截断步长 + 乘子检查） ----
    // 维护显式 active 集（贴在下界或上界的变量），只在乘子不满足时逐个移出，
    // 避免「按当前点重算 free 集」造成的抖动/爬行（见文件头注释）。
    const double bound_eps = 1e-12;
    std::vector<char> active(n, 0);
    for (int i = 0; i < n; ++i) {
        if (x(i) <= lb(i) + bound_eps || x(i) >= ub(i) - bound_eps) active[i] = 1;
    }

    int budget = 4 * n + 200;
    for (int it = 0; it < std::min(opt.max_iters, budget); ++it) {
        if (pg_residual(x) <= opt.tol) break;
        info.iters = it + 1;

        const Eigen::VectorXd grad = grad_at(x);
        Eigen::VectorXi free_idx(n);
        int nf = 0;
        for (int i = 0; i < n; ++i)
            if (!active[i]) free_idx(nf++) = i;

        Eigen::VectorXd d = Eigen::VectorXd::Zero(n);
        bool stationary_on_free = true;
        if (nf > 0) {
            Eigen::MatrixXd Hff(nf, nf);
            Eigen::VectorXd gf(nf);
            for (int a = 0; a < nf; ++a) {
                for (int b = 0; b < nf; ++b) Hff(a, b) = H(free_idx(a), free_idx(b));
                gf(a) = grad(free_idx(a));
            }
            const Eigen::VectorXd d_free = Hff.ldlt().solve(-gf);
            if (!d_free.allFinite()) break;
            for (int a = 0; a < nf; ++a) d(free_idx(a)) = d_free(a);
            double x_scale = 1.0;
            for (int a = 0; a < nf; ++a) x_scale = std::max(x_scale, std::fabs(x(free_idx(a))));
            stationary_on_free = d_free.cwiseAbs().maxCoeff() < 1e-9 * x_scale;
        }

        if (stationary_on_free) {
            // free 集上已到驻点：检查 active 集乘子，移出违反最严重的一个。
            int worst = -1;
            double worst_violation = 0.0;
            for (int i = 0; i < n; ++i) {
                if (!active[i]) continue;
                const bool at_lb = x(i) <= lb(i) + bound_eps;
                const double violation = at_lb ? -grad(i) : grad(i);
                if (violation > worst_violation) {
                    worst_violation = violation;
                    worst = i;
                }
            }
            if (worst < 0) break;  // KKT 全部满足（或无 active 变量）
            active[worst] = 0;
            continue;
        }

        // 步长：不超过 1，也不越过盒边界（撞到的变量记入 active 集）。
        double alpha = 1.0;
        for (int a = 0; a < nf; ++a) {
            const int i = free_idx(a);
            if (d(i) > 0.0) {
                alpha = std::min(alpha, (ub(i) - x(i)) / d(i));
            } else if (d(i) < 0.0) {
                alpha = std::min(alpha, (lb(i) - x(i)) / d(i));
            }
        }
        if (alpha <= 0.0) {
            // 方向在首个边界处即被挡住：把那些变量固定住，下一轮重解。
            for (int a = 0; a < nf; ++a) {
                const int i = free_idx(a);
                if ((d(i) > 0.0 && x(i) >= ub(i) - bound_eps) ||
                    (d(i) < 0.0 && x(i) <= lb(i) + bound_eps)) {
                    active[i] = 1;
                }
            }
            continue;
        }
        x += alpha * d;
        for (int i = 0; i < n; ++i) {
            if (active[i]) x(i) = std::min(std::max(x(i), lb(i)), ub(i));
            else if (x(i) <= lb(i) + bound_eps) {
                x(i) = std::max(x(i), lb(i));
                active[i] = 1;
            } else if (x(i) >= ub(i) - bound_eps) {
                x(i) = std::min(x(i), ub(i));
                active[i] = 1;
            }
        }
        keep_best(x);
    }

    // ---- ③ 兜底：带盒截断的坐标下降（Gauss-Seidel） ----
    if (pg_residual(x) > opt.tol) {
        Eigen::VectorXd z = x;
        for (int it = 0; it < opt.polish_iters; ++it) {
            double max_delta = 0.0;
            for (int i = 0; i < n; ++i) {
                const double hii = std::max(H(i, i), 1e-12);
                const double off = H.row(i).dot(z) - H(i, i) * z(i);  // Σ_{j≠i} H_ij z_j
                const double zi = std::min(std::max(-(g(i) + off) / hii, lb(i)), ub(i));
                max_delta = std::max(max_delta, std::fabs(zi - z(i)));
                z(i) = zi;
            }
            info.polish = it + 1;
            if (max_delta < opt.tol) break;
        }
        keep_best(z);
    }

    info.residual = pg_residual(best);
    info.converged = info.residual <= std::max(opt.tol, 1e-8);
    if (info_out) *info_out = info;
    return best;
}

}  // namespace mpc_wbc
