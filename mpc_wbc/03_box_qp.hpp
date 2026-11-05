/*===============================================================================================
 * @file 03_box_qp.hpp
 * @brief 带上下界的凸二次规划求解器，供 SRBD-MPC 接触力优化调用
 * @details 标准形式：min 0.5*xᵀ*H*x + gᵀ*x，约束 lb <= x <= ub。
 * H 为对称凸二次项，SRBD-MPC 通过正则改善可解性；调用者应保证矩阵尺寸和上下界一致。
 * 计算顺序：尝试无约束解→显式主动集迭代→必要时坐标下降，保留已记录的较低代价可行点。
 * 本模块仅依赖 Eigen 和标准库，不处理 WBC 的等式或一般线性不等式约束。
 * 收敛依据为投影梯度残差；有限迭代结束可能仍未收敛，调用者可读取 BoxQpInfo。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include <Eigen/Dense>                              // Eigen 矩阵、向量及线性方程求解
#include <algorithm>                                // 最值、夹取和排序等标准算法
#include <cmath>                                    // 三角函数、绝对值和其他数学函数
#include <limits>                                   // 数值搜索使用的极值常量
#include <vector>                                   // 随支撑数量或预测时域变化的动态数组



namespace mpc_wbc {

//===============================================================================================
// @brief 盒约束 QP 的求解精度与迭代预算
//===============================================================================================
struct BoxQpOptions {
    int max_iters = 100;                                        // 主动集/自由变量牛顿迭代上限；实际还受内部预算限制
    int polish_iters = 2000;                                    // 兜底坐标下降的最大整轮扫描次数
    double tol = 1e-10;                                         // 投影梯度残差及坐标变化的停止容差
};



//===============================================================================================
// @brief 盒约束 QP 的迭代记录和收敛诊断
//===============================================================================================
struct BoxQpInfo {
    int iters = 0;                                              // 主动集阶段记录的迭代次数
    int polish = 0;                                             // 坐标下降阶段记录的扫描次数
    double residual = 0.0;                                      // 返回点的投影梯度无穷范数残差
    bool converged = false;                                     // 残差是否不超过 max(tol,1e-8)
};



//------------------------------------------------------------------------------------
// @brief 求解一组带变量上下界的凸 QP，并返回当前保留的解
// @param H n×n 二次项；g、lb、ub 均为 n 维，且每维 lb<=ub。
// @param x_init 可选初值；维数为 n 时先投影到可行域，再按代价决定是否采用。
// @param info_out 可选诊断输出；n<=0 时返回空向量。
// @details 投影残差 r = ||x-proj(x-(H*x+g))||∞；凸问题在 r=0 时满足盒约束 KKT 条件。
//------------------------------------------------------------------------------------
inline Eigen::VectorXd SolveBoxQp(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                                  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                  const Eigen::VectorXd* x_init = nullptr,
                                  const BoxQpOptions& opt = BoxQpOptions(),
                                  BoxQpInfo* info_out = nullptr) 
{
    BoxQpInfo info;
    const int n = static_cast<int>(H.rows());
    if (n <= 0) {
        if (info_out) *info_out = info;
        return Eigen::VectorXd();
    }

    // 步骤 1：建立可行域投影、梯度、目标函数和 KKT 残差计算工具。
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

    // 步骤 2：记录搜索过程中代价较低的候选，供后续阶段统一返回。
    Eigen::VectorXd best;
    double best_obj = std::numeric_limits<double>::max();
    auto keep_best = [&](const Eigen::VectorXd& v) {
        const double f = objective(v);
        if (f < best_obj) {
            best_obj = f;
            best = v;
        }
    };

    // 步骤 3：先求无约束最优点；若有限且位于盒内，直接返回该点。
    const Eigen::VectorXd x_unc = H.ldlt().solve(-g);
    if (x_unc.allFinite() && ((x_unc - project(x_unc)).cwiseAbs().maxCoeff() < 1e-12)) {
        info.residual = pg_residual(x_unc);
        info.converged = info.residual <= std::max(opt.tol, 1e-8);
        if (info_out) *info_out = info;
        return x_unc;
    }

    // 步骤 4：选择可行起点：盒中心→无约束解投影→代价更优的用户初值。
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) x(i) = 0.5 * (lb(i) + ub(i));
    if (x_unc.allFinite()) x = project(x_unc);   // 无约束解截断到盒内，通常已很接近最优
    if (x_init && x_init->size() == n) {
        const Eigen::VectorXd cand = project(*x_init);
        if (objective(cand) < objective(x)) x = cand;
    }
    keep_best(x);

    // 步骤 5：建立主动边界集合；只在自由变量子空间计算牛顿方向。
    // active[i]=1 表示当前变量固定在某个上下界；集合会随边界命中和乘子检查更新。
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

        // 步骤 6：自由变量已到驻点时检查边界乘子，释放违反最严重的边界变量。
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

        // 步骤 7：按最先命中的盒边界截断步长；无法前进时把阻挡变量加入主动集。
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

    // 步骤 8：残差仍超限时，逐坐标精确最小化并截断到盒内，作为兜底改进。
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

    // 步骤 9：根据保留点重算残差并返回诊断；返回向量不等于保证 converged=true。
    info.residual = pg_residual(best);
    info.converged = info.residual <= std::max(opt.tol, 1e-8);
    if (info_out) *info_out = info;
    return best;
}

}  // namespace mpc_wbc



