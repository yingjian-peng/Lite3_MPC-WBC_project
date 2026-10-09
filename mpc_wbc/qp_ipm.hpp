/**
 * @file qp_ipm.hpp
 * @brief 自研稠密 QP 求解器（原-对偶内点法，Mehrotra 预测-校正）。零第三方依赖。
 *
 * 求解标准形式：
 *     min  ½·xᵀHx + gᵀx
 *     s.t. A_eq·x = b_eq        （等式，m_eq 行）
 *          A_ge·x ≥ b_ge        （不等式，m_ge 行；乘子 z ≥ 0）
 * H 需对称半正定（WBC 里 H = AᵀA + 正则，故取 reg 后正定）。变量上下界请自行展开成 ≥ 行
 * （a_iᵀx ≥ b_i），或用 qp_box_rows.hpp 的辅助函数生成——见 WBC 的行布局说明。
 *
 * 为什么用内点法（而不是主动集）：
 *   - 不需可行起点、不会因退化约束在边界反复切换（docs/04 S7 的 WBC 约束行列很多是
 *     “贴 0”的摩擦锥行，主动集在这种退化点容易爬行）；
 *   - 每步只解一次对称 KKT 线性系统，失败可判定（残差可回报），符合 S7「QP 失败可报告」。
 * 代价：不支持热启动，迭代次数固定（~10-30），S10 若要 1 kHz 需另做主动集+工作集缓存。
 *
 * 数值细节：
 *   - KKT 系统 [[H+GᵀDG, −Aᵀ],[A, −δI]] 用**部分主元 LU** 求解（对称但不定，Eigen 的 LDLT
 *     在这种矩阵上会给出错误符号，实测会发散；规模小，LU 的额外代价可接受），
 *     失败再回退 FullPivLU；极小正则 (ε,δ) 只影响末位有效数字，收敛判据仍按**未正则**的残差算；
 *   - 起点：x 用调用者给的初值（可为 0），s/z 取正（必要时整体平移到严格内点），
 *     等式残差由牛顿步自然收敛；
 *   - 收敛判据：相对原始残差、对偶残差、以及互补间隙 max(sᵀz/m)。
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace mpc_wbc {

struct QpProblem {
    Eigen::MatrixXd H;     // n×n，对称半正定
    Eigen::VectorXd g;     // n
    Eigen::MatrixXd A_eq;  // m_eq×n（可为 0×n）
    Eigen::VectorXd b_eq;  // m_eq
    Eigen::MatrixXd A_ge;  // m_ge×n（a_iᵀx ≥ b_i，可为 0×n）
    Eigen::VectorXd b_ge;  // m_ge
};

struct QpIpmOptions {
    int max_iters = 60;          // 外层牛顿迭代上限
    double tol = 1e-9;           // 相对容差（原始/对偶残差与互补间隙共用）
    double step_scale = 0.995;   // 步长安全系数（保持 s,z > 0）
    double reg_primal = 1e-10;   // H 块正则
    double reg_dual = 1e-10;     // 等式乘子块正则
};

struct QpIpmInfo {
    int iters = 0;
    double pri_res = 0.0;   // max(‖A_eq x − b_eq‖∞, 不等式违反)
    double dual_res = 0.0;  // ‖H x + g − A_eqᵀ y − A_geᵀ z‖∞
    double gap = 0.0;       // sᵀz/m
    double min_z = 0.0;     // min(z)：应为 ≥ 0（对偶可行）
    bool converged = false;
    bool primal_infeasible = false;  // 迭代到上限仍未满足原始残差，且残差不再下降
};

struct QpIpmResult {
    Eigen::VectorXd x;
    Eigen::VectorXd y;   // 等式乘子
    Eigen::VectorXd z;   // 不等式乘子（≥ 0）
    QpIpmInfo info;
    bool ok() const { return info.converged; }
};

/**
 * 求解 QP。x_init 可空（空则从 x=0 起步，等式残差由牛顿步收敛）。
 */
inline QpIpmResult SolveQpIpm(const QpProblem& qp, const QpIpmOptions& opt = QpIpmOptions(),
                              const Eigen::VectorXd* x_init = nullptr) {
    QpIpmResult res;
    const int n = static_cast<int>(qp.H.rows());
    const int me = static_cast<int>(qp.A_eq.rows());
    const int mg = static_cast<int>(qp.A_ge.rows());
    if (n <= 0 || qp.g.size() != n) return res;
    // 允许把「没有等式/没有不等式」写成空矩阵（0×0），这里统一成 0×n。
    if (qp.A_eq.cols() != n || qp.b_eq.size() != me || qp.A_ge.cols() != n ||
        qp.b_ge.size() != mg) {
        if (!((me == 0 && qp.A_eq.size() == 0 && qp.b_eq.size() == 0) ||
              (mg == 0 && qp.A_ge.size() == 0 && qp.b_ge.size() == 0))) {
            return res;
        }
    }

    const Eigen::MatrixXd Heq = qp.H + opt.reg_primal * Eigen::MatrixXd::Identity(n, n);
    const Eigen::MatrixXd A = qp.A_eq;
    const Eigen::MatrixXd G = qp.A_ge;
    const Eigen::VectorXd& g = qp.g;
    const Eigen::VectorXd& b = qp.b_eq;
    const Eigen::VectorXd& h = qp.b_ge;

    Eigen::VectorXd x = (x_init && x_init->size() == n) ? *x_init : Eigen::VectorXd::Zero(n);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(me);
    Eigen::VectorXd s = Eigen::VectorXd::Zero(mg);
    Eigen::VectorXd z = Eigen::VectorXd::Ones(mg);
    // 内点起点：只需 s>0、z>0（s=Gx−h 的一致性由牛顿步恢复）。逐行取
    // s_i = max(G_i x − h_i, 1)：既保证严格内点，又避免出现 s_i≈0 使 KKT 里的
    // z_i/s_i 爆炸（实测「整体平移到 1e-3」的初始化会让对偶变量发散到 1e20）。
    for (int i = 0; i < mg; ++i) {
        s(i) = std::max(G.row(i).dot(x) - h(i), 1.0);
    }

    const double scale = 1.0 + std::max(std::max(g.norm(), b.norm()), h.norm());
    const double gap_tol = opt.tol * scale;

    for (int it = 0; it < opt.max_iters; ++it) {
        res.info.iters = it + 1;

        // ---- 残差 ----
        Eigen::VectorXd r_eq = (me > 0) ? Eigen::VectorXd(A * x - b) : Eigen::VectorXd();
        Eigen::VectorXd r_ge = (mg > 0) ? Eigen::VectorXd(G * x - s - h) : Eigen::VectorXd();
        Eigen::VectorXd r_dual = Heq * x + g;
        if (me > 0) r_dual -= A.transpose() * y;
        if (mg > 0) r_dual -= G.transpose() * z;

        double pri_violation = 0.0;
        for (int i = 0; i < mg; ++i) {
            const double slack = G.row(i).dot(x) - h(i);
            if (slack < 0.0) pri_violation = std::max(pri_violation, -slack);
        }
        const double pri_res = std::max(me > 0 ? r_eq.cwiseAbs().maxCoeff() : 0.0, pri_violation);
        const double dual_res = r_dual.cwiseAbs().maxCoeff();
        const double mu = (mg > 0) ? s.dot(z) / mg : 0.0;

        res.info.pri_res = pri_res;
        res.info.dual_res = dual_res;
        res.info.gap = mu;
        res.info.min_z = (mg > 0) ? z.minCoeff() : 0.0;
        if (std::max(pri_res, dual_res) <= opt.tol * scale && mu <= gap_tol) {
            res.info.converged = true;
            break;
        }

        // ---- KKT 线性系统求解器（同一结构解两次：预测 + 校正） ----
        const int kkt_n = n + me;
        auto solve_kkt = [&](const Eigen::VectorXd& rhs1, const Eigen::VectorXd& rhs2,
                             Eigen::VectorXd* dx, Eigen::VectorXd* dy) {
            Eigen::MatrixXd K = Eigen::MatrixXd::Zero(kkt_n, kkt_n);
            for (int i = 0; i < mg; ++i) {
                const Eigen::VectorXd& gi = G.row(i).transpose();
                K.block(0, 0, n, n) += (z(i) / std::max(s(i), 1e-12)) * (gi * gi.transpose());
            }
            K.block(0, 0, n, n) += Heq;
            if (me > 0) {
                K.block(0, n, n, me) = -A.transpose();
                K.block(n, 0, me, n) = A;
                K.block(n, n, me, me) = -opt.reg_dual * Eigen::MatrixXd::Identity(me, me);
            }
            Eigen::VectorXd rhs(kkt_n);
            rhs.head(n) = rhs1;
            if (me > 0) rhs.tail(me) = rhs2;

            // 注意：KKT 是对称**不定**矩阵，Eigen 的 LDLT 在这里会给出错误的符号
            // （实测 dy 反号 → 迭代发散），必须用部分主元 LU。矩阵规模 ≤ 150，代价可接受。
            Eigen::PartialPivLU<Eigen::MatrixXd> lu(K);
            Eigen::VectorXd sol = lu.solve(rhs);
            if (!sol.allFinite() || (K * sol - rhs).cwiseAbs().maxCoeff() > 1e-6 * (1.0 + rhs.norm())) {
                Eigen::FullPivLU<Eigen::MatrixXd> flu(K);
                sol = flu.solve(rhs);
            }
            if (!sol.allFinite()) return false;
            *dx = sol.head(n);
            *dy = (me > 0) ? Eigen::VectorXd(sol.tail(me)) : Eigen::VectorXd();
            return true;
        };

        auto direction_rhs = [&](const Eigen::VectorXd& r_eq_, const Eigen::VectorXd& r_dual_,
                                 const Eigen::VectorXd& r_ge_, const Eigen::VectorXd& comp_rhs,
                                 Eigen::VectorXd* rhs1, Eigen::VectorXd* rhs2) {
            // 由 Gᵀ·Δz 代入平稳性方程得到：rhs1 = −r_dual − Gᵀ·t，
            //   t_i = (z_i/s_i)·r_ge,i − comp_i/s_i
            // （comp 项的符号是推导里最容易写错的地方，写反会让迭代停滞。）
            Eigen::VectorXd t = Eigen::VectorXd::Zero(mg);
            for (int i = 0; i < mg; ++i) {
                const double si = std::max(s(i), 1e-12);
                t(i) = (z(i) / si) * r_ge_(i) - comp_rhs(i) / si;
            }
            *rhs1 = -r_dual_ - (mg > 0 ? Eigen::VectorXd(G.transpose() * t) : Eigen::VectorXd::Zero(n));
            *rhs2 = -r_eq_;
        };

        // 由互补性方程解出 Δs、Δz：
        //   Δs = G·Δx + r_ge
        //   Δz = −(z/s)∘Δs + comp/s      ← comp/s 这一项是“向心项”，漏掉会让互补性永远不收敛
        auto take_direction = [&](const Eigen::VectorXd& rhs1, const Eigen::VectorXd& rhs2,
                                  const Eigen::VectorXd& comp, Eigen::VectorXd* dx,
                                  Eigen::VectorXd* dy, Eigen::VectorXd* ds, Eigen::VectorXd* dz) {
            if (!solve_kkt(rhs1, rhs2, dx, dy)) return false;
            if (mg > 0) {
                *ds = G * (*dx) + r_ge;
                dz->resize(mg);
                for (int i = 0; i < mg; ++i) {
                    const double si = std::max(s(i), 1e-12);
                    (*dz)(i) = -(z(i) / si) * (*ds)(i) + comp(i) / si;
                }
            }
            return true;
        };

        // ---- 预测步（仿射方向，互补项 rhs = −s∘z） ----
        Eigen::VectorXd comp_aff = (mg > 0) ? Eigen::VectorXd(-s.cwiseProduct(z))
                                            : Eigen::VectorXd();
        Eigen::VectorXd rhs1, rhs2;
        direction_rhs(r_eq, r_dual, r_ge, comp_aff, &rhs1, &rhs2);
        Eigen::VectorXd dx, dy, ds, dz;
        if (!take_direction(rhs1, rhs2, comp_aff, &dx, &dy, &ds, &dz)) break;

        double alpha_aff_p = 1.0, alpha_aff_d = 1.0;
        for (int i = 0; i < mg; ++i) {
            if (ds(i) < 0.0) alpha_aff_p = std::min(alpha_aff_p, -s(i) / ds(i));
            if (dz(i) < 0.0) alpha_aff_d = std::min(alpha_aff_d, -z(i) / dz(i));
        }
        double mu_aff = 0.0;
        if (mg > 0) {
            for (int i = 0; i < mg; ++i)
                mu_aff += (s(i) + alpha_aff_p * ds(i)) * (z(i) + alpha_aff_d * dz(i));
            mu_aff /= mg;
        }
        double sigma = 0.0;
        if (mu > 1e-15) {
            sigma = std::pow(std::max(mu_aff / mu, 0.0), 3.0);
            sigma = std::min(std::max(sigma, 0.0), 0.9);
        }

        // ---- 校正步（互补项 rhs = σμe − s∘z） ----
        Eigen::VectorXd comp_cor = Eigen::VectorXd::Zero(mg);
        for (int i = 0; i < mg; ++i) comp_cor(i) = sigma * mu - s(i) * z(i);
        direction_rhs(r_eq, r_dual, r_ge, comp_cor, &rhs1, &rhs2);
        if (!take_direction(rhs1, rhs2, comp_cor, &dx, &dy, &ds, &dz)) break;

        // 最大可行步长先算出来，再乘安全系数；**只有被约束挡住时（α_max < 1）才打折**，
        // 否则取满步（牛顿步）。两个坑：
        //   - 直接写 min(step_scale, −s/ds) 会取到边界值把 s 打到恰好 0 → 下轮 z/s = 0/0；
        //   - 无约束限制时也乘 0.995，会让迭代变成几何收敛（每步只走 99.5%），几乎不收敛。
        double max_p = 1.0, max_d = 1.0;
        for (int i = 0; i < mg; ++i) {
            if (ds(i) < 0.0) max_p = std::min(max_p, -s(i) / ds(i));
            if (dz(i) < 0.0) max_d = std::min(max_d, -z(i) / dz(i));
        }
        const double alpha_p = (max_p >= 1.0) ? 1.0 : opt.step_scale * max_p;
        const double alpha_d = (max_d >= 1.0) ? 1.0 : opt.step_scale * max_d;

        x += alpha_p * dx;
        if (me > 0) y += alpha_d * dy;
        if (mg > 0) {
            s += alpha_p * ds;
            z += alpha_d * dz;
        }
        if (!x.allFinite() || (mg > 0 && (!s.allFinite() || !z.allFinite()))) break;
    }

    res.x = x;
    res.y = y;
    res.z = z;
    if (!res.info.converged) {
        // 到上限仍未达容差：若原始残差明显超标且不再下降，标记原始不可行。
        res.info.primal_infeasible = res.info.pri_res > 1e-6 * scale;
    }
    return res;
}

}  // namespace mpc_wbc
