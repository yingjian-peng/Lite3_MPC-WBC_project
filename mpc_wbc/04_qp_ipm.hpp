/*===============================================================================================
 * @file 04_qp_ipm.hpp
 * @brief 含等式和一般线性不等式的稠密 QP 原-对偶内点求解器
 * @details 目标 min 0.5*xᵀ*H*x + gᵀ*x；约束 A_eq*x=b_eq、A_ge*x>=b_ge。
 * 将不等式写为 G*x-s=h，引入正松弛量 s 和非负乘子 z；通过仿射预测与中心化校正推进。
 * 调用者负责维度一致与凸性；没有约束的一组矩阵宜显式设置为 0×n。
 * 当前 KKT 块采用 [-Aᵀ; A] 的乘子符号约定，使用 LU 求解，不假设系统为对称正定。
 * 仅复用 x_init 作为原始变量初值；等式乘子、松弛量和不等式乘子每次重新初始化。
 * 本模块依赖 Eigen；失败通过 info 报告，不执行控制模式切换或日志输出。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include <Eigen/Dense>                              // Eigen 矩阵、向量及线性方程求解
#include <algorithm>                                // 最值、夹取和排序等标准算法
#include <cmath>                                    // 三角函数、绝对值和其他数学函数



namespace mpc_wbc {
//===============================================================================================
// @brief QP 标准形式的代价矩阵与约束数据
//===============================================================================================
struct QpProblem {
    Eigen::MatrixXd H;                                          // n×n 对称半正定 Hessian；求解内部会加入原始变量正则
    Eigen::VectorXd g;                                          // n 维一次项
    Eigen::MatrixXd A_eq;                                       // m_eq×n 等式系数矩阵
    Eigen::VectorXd b_eq;                                       // m_eq 维等式右端项
    Eigen::MatrixXd A_ge;                                       // m_ge×n 不等式系数矩阵，约束方向固定为 >=
    Eigen::VectorXd b_ge;                                       // m_ge 维不等式右端项
};



//===============================================================================================
// @brief 内点迭代精度、步长和数值正则配置
//===============================================================================================
struct QpIpmOptions {
    int max_iters = 60;                                         // 预测/校正外层迭代次数上限
    double tol = 1e-9;                                          // 原始残差、对偶残差和互补间隙的缩放容差
    double step_scale = 0.995;                                  // 受约束阻挡时的步长安全系数，保持 s、z 严格为正
    double reg_primal = 1e-10;                                  // 原始变量 Hessian 块的对角正则
    double reg_dual = 1e-10;                                    // 等式乘子 KKT 对角块的正则大小
};



//===============================================================================================
// @brief 求解状态和残差诊断，供 WBC 判断本帧是否可用
//===============================================================================================
struct QpIpmInfo {
    int iters = 0;                                              // 本次实际执行的外层迭代数
    double pri_res = 0.0;                                       // 等式误差与实际不等式违反的最大无穷范数
    double dual_res = 0.0;                                      // 使用正则化 Hessian 计算的平稳性残差无穷范数
    double gap = 0.0;                                           // 平均互补量 sᵀ*z/m_ge；无不等式时为 0
    double min_z = 0.0;                                         // 不等式乘子最小值；无不等式时为 0
    bool converged = false;                                     // 原始/对偶残差和互补间隙是否同时满足停止容差
    bool primal_infeasible = false;                             // 未收敛且原始残差超阈值的诊断标志，不构成严格不可行证书
};



//===============================================================================================
// @brief QP 原始解、乘子及收敛诊断
//===============================================================================================
struct QpIpmResult {
    Eigen::VectorXd x;                                          // 原始决策变量 n 维
    Eigen::VectorXd y;                                          // 等式约束乘子 m_eq 维
    Eigen::VectorXd z;                                          // 不等式乘子 m_ge 维，算法迭代中保持非负
    QpIpmInfo info;                                             // 收敛状态、残差和迭代数

    //------------------------------------------------------------------------------------
    // @brief 读取本次迭代是否达到收敛条件
    //------------------------------------------------------------------------------------
    bool ok() const { return info.converged; }
};



//------------------------------------------------------------------------------------
// @brief 执行原-对偶内点迭代，返回最后一次变量及求解诊断
// @param qp 凸二次代价与等式/不等式数据，约束方向必须与 QpProblem 定义一致。
// @param x_init 可选 n 维原始变量初值；为空或维数不匹配时使用零向量。
// @return 通过 result.ok() 判断收敛；维度检查、线性求解或有限值检查失败均可能提前退出。
// @details 不要求输入 x 满足约束；正松弛量起点与实际约束之间的残差由牛顿步恢复。
//------------------------------------------------------------------------------------
inline QpIpmResult SolveQpIpm(const QpProblem& qp, const QpIpmOptions& opt = QpIpmOptions(), const Eigen::VectorXd* x_init = nullptr) 
{
    // 步骤 1：取得变量/约束维度，并进行基本输入检查。
    QpIpmResult res;
    const int n = static_cast<int>(qp.H.rows());
    const int me = static_cast<int>(qp.A_eq.rows());
    const int mg = static_cast<int>(qp.A_ge.rows());
    if (n <= 0 || qp.g.size() != n) return res;
    // 基本维度检查；主线 WBC 提供固定尺寸矩阵，本函数不会在此重新调整矩阵尺寸。
    if (qp.A_eq.cols() != n || qp.b_eq.size() != me || qp.A_ge.cols() != n ||
        qp.b_ge.size() != mg) {
        if (!((me == 0 && qp.A_eq.size() == 0 && qp.b_eq.size() == 0) ||
              (mg == 0 && qp.A_ge.size() == 0 && qp.b_ge.size() == 0))) {
            return res;
        }
    }

    // 步骤 2：加入 Hessian 对角正则，并用 A/G、b/h 简化等式和不等式记号。
    const Eigen::MatrixXd Heq = qp.H + opt.reg_primal * Eigen::MatrixXd::Identity(n, n);
    const Eigen::MatrixXd A = qp.A_eq;
    const Eigen::MatrixXd G = qp.A_ge;
    const Eigen::VectorXd& g = qp.g;
    const Eigen::VectorXd& b = qp.b_eq;
    const Eigen::VectorXd& h = qp.b_ge;

    // 步骤 3：初始化 x/y/s/z；每个 s_i 至少取 1，避免 z_i/s_i 初始比值过大。
    Eigen::VectorXd x = (x_init && x_init->size() == n) ? *x_init : Eigen::VectorXd::Zero(n);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(me);
    Eigen::VectorXd s = Eigen::VectorXd::Zero(mg);
    Eigen::VectorXd z = Eigen::VectorXd::Ones(mg);
    // 起点要求 s>0、z>0；约束一致性由后续迭代恢复。逐行取 s_i=max(G_i*x-h_i,1)，
    // 避免过小松弛量导致 KKT 中 z_i/s_i 数值过大。
    for (int i = 0; i < mg; ++i) {
        // 正松弛起点允许暂时不满足 G*x-s=h，但必须避免落到 s=0 的奇异边界。
        s(i) = std::max(G.row(i).dot(x) - h(i), 1.0);
    }

    // 步骤 4：根据一次项和约束右端项建立残差缩放尺度。
    const double scale = 1.0 + std::max(std::max(g.norm(), b.norm()), h.norm());
    const double gap_tol = opt.tol * scale;
    for (int it = 0; it < opt.max_iters; ++it) {
        res.info.iters = it + 1;

        // 步骤 5：计算等式、松弛一致性、平稳性残差和平均互补量，并判断收敛。
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

        // 步骤 6：消去松弛和不等式乘子方向，构造 n+m_eq 维 KKT 线性系统求解器。
        const int kkt_n = n + me;
        auto solve_kkt = [&](const Eigen::VectorXd& rhs1, const Eigen::VectorXd& rhs2, Eigen::VectorXd* dx, Eigen::VectorXd* dy) 
        {
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

            // 数值求解：先用部分主元 LU；结果非有限或线性残差过大时再用全主元 LU。
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

        // 步骤 7：把原始/对偶及互补残差转换为 KKT 右端项。
        auto direction_rhs = [&](const Eigen::VectorXd& r_eq_, const Eigen::VectorXd& r_dual_,
                                 const Eigen::VectorXd& r_ge_, const Eigen::VectorXd& comp_rhs,
                                 Eigen::VectorXd* rhs1, Eigen::VectorXd* rhs2) 
        {

            // 消元公式：t_i=(z_i/s_i)*r_ge_i-comp_i/s_i；右端符号须与乘子定义一致。
            Eigen::VectorXd t = Eigen::VectorXd::Zero(mg);
            for (int i = 0; i < mg; ++i) {
                const double si = std::max(s(i), 1e-12);
                t(i) = (z(i) / si) * r_ge_(i) - comp_rhs(i) / si;
            }
            *rhs1 = -r_dual_ - (mg > 0 ? Eigen::VectorXd(G.transpose() * t) : Eigen::VectorXd::Zero(n));
            *rhs2 = -r_eq_;
        };

        // 步骤 8：先求 dx/dy，再根据 ds=G*dx+r_ge 恢复 ds/dz。
        auto take_direction = [&](const Eigen::VectorXd& rhs1, const Eigen::VectorXd& rhs2,
                                  const Eigen::VectorXd& comp, Eigen::VectorXd* dx,
                                  Eigen::VectorXd* dy, Eigen::VectorXd* ds, Eigen::VectorXd* dz) 
        {
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

        // 步骤 9：仿射预测步使用 comp=-s*z，估计可行步长及预测互补量。
        Eigen::VectorXd comp_aff = (mg > 0) ? Eigen::VectorXd(-s.cwiseProduct(z)) : Eigen::VectorXd();
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

        // 步骤 10：由预测互补量与当前互补量的比值计算中心化参数，并限制在 [0,0.9]。
        double sigma = 0.0;
        if (mu > 1e-15) {
            sigma = std::pow(std::max(mu_aff / mu, 0.0), 3.0);
            sigma = std::min(std::max(sigma, 0.0), 0.9);
        }

        // 步骤 11：中心化校正步使用 comp=sigma*mu-s*z，重新计算搜索方向。
        Eigen::VectorXd comp_cor = Eigen::VectorXd::Zero(mg);
        for (int i = 0; i < mg; ++i) comp_cor(i) = sigma * mu - s(i) * z(i);
        direction_rhs(r_eq, r_dual, r_ge, comp_cor, &rhs1, &rhs2);
        if (!take_direction(rhs1, rhs2, comp_cor, &dx, &dy, &ds, &dz)) break;

        // 步骤 12：分别计算原始与对偶最大可行步长，仅在被边界阻挡时乘安全系数。
        double max_p = 1.0, max_d = 1.0;
        for (int i = 0; i < mg; ++i) {
            if (ds(i) < 0.0) max_p = std::min(max_p, -s(i) / ds(i));
            if (dz(i) < 0.0) max_d = std::min(max_d, -z(i) / dz(i));
        }
        const double alpha_p = (max_p >= 1.0) ? 1.0 : opt.step_scale * max_p;
        const double alpha_d = (max_d >= 1.0) ? 1.0 : opt.step_scale * max_d;

        // 步骤 13：更新 x/s 与 y/z；遇到非有限值时退出迭代。
        x += alpha_p * dx;
        if (me > 0) y += alpha_d * dy;
        if (mg > 0) {
            s += alpha_p * ds;
            z += alpha_d * dz;
        }
        if (!x.allFinite() || (mg > 0 && (!s.allFinite() || !z.allFinite()))) break;
    }

    // 步骤 14：保存最终变量；未收敛时依据原始残差补充不可行诊断标志。
    res.x = x;
    res.y = y;
    res.z = z;
    if (!res.info.converged) {
        // 未收敛且原始残差超阈值时置诊断标志；这里不检查残差下降趋势，也不生成不可行证书。
        res.info.primal_infeasible = res.info.pri_res > 1e-6 * scale;
    }
    return res;
}

}  // namespace mpc_wbc



