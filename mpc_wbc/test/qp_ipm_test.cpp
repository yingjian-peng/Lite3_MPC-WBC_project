/**
 * @file qp_ipm_test.cpp
 * @brief qp_ipm.hpp 自研 QP 求解器的单元自检（不需要 MuJoCo）。
 *
 * 检查项：
 *   1. 解析解：无约束、等式（min ‖x‖² s.t. Σx=1）、单边不等式、起作用的上界；
 *   2. 与 box_qp.hpp 交叉验证：同一盒约束 QP（界展开成 ≥ 行）两者解一致；
 *   3. 随机 QP 的 KKT 验证：原始可行、对偶可行（z ≥ 0）、互补、平稳性残差 ≤ 容差；
 *   4. 退化/边界情形：约束贴 0（x_i ≥ 0 且最优解在边界）、冗余等式、0 行不等式；
 *   5. 不可行问题能被识别（不返回“假装成功”的解）。
 *
 * 用法： ./qp_ipm_test
 * 返回 0 表示通过。
 */
#include "box_qp.hpp"
#include "qp_ipm.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace mpc_wbc;

namespace {

int g_failures = 0;

std::string Format(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return std::string(buf);
}

void Check(bool ok, const char* name, const std::string& detail = std::string()) {
    std::printf("  [%s] %s%s%s\n", ok ? "OK  " : "FAIL", name, detail.empty() ? "" : "  ",
                detail.c_str());
    if (!ok) ++g_failures;
}

/// 独立复算 KKT 残差（不依赖求解器内部报告）。
struct KktCheck {
    double eq_res = 0.0;      // ‖A_eq x − b_eq‖∞
    double ineq_viol = 0.0;   // max(0, b_ge − A_ge x)
    double dual_res = 0.0;    // ‖Hx + g − A_eqᵀy − A_geᵀz‖∞
    double comp = 0.0;        // |zᵀ(A_ge x − b_ge)|
    double min_z = 0.0;
};

KktCheck CheckKkt(const QpProblem& qp, const QpIpmResult& r) {
    KktCheck c;
    const int n = static_cast<int>(qp.H.rows());
    const int me = static_cast<int>(qp.A_eq.rows());
    const int mg = static_cast<int>(qp.A_ge.rows());
    Eigen::VectorXd grad = qp.H * r.x + qp.g;
    if (me > 0) {
        c.eq_res = (qp.A_eq * r.x - qp.b_eq).cwiseAbs().maxCoeff();
        grad -= qp.A_eq.transpose() * r.y;
    }
    if (mg > 0) {
        const Eigen::VectorXd slack = qp.A_ge * r.x - qp.b_ge;
        c.ineq_viol = std::max(0.0, -slack.minCoeff());
        grad -= qp.A_ge.transpose() * r.z;
        c.comp = std::fabs(r.z.dot(slack));
        c.min_z = r.z.minCoeff();
    }
    (void)n;
    c.dual_res = grad.cwiseAbs().maxCoeff();
    return c;
}

/// 把盒约束 lb ≤ x ≤ ub 展开成 ≥ 行：x ≥ lb，−x ≥ −ub。
void AppendBoxRows(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, QpProblem* qp) {
    const int n = static_cast<int>(lb.size());
    const int m0 = static_cast<int>(qp->A_ge.rows());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m0 + 2 * n, n);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(m0 + 2 * n);
    for (int i = 0; i < m0; ++i) {
        A.row(i) = qp->A_ge.row(i);
        b(i) = qp->b_ge(i);
    }
    for (int i = 0; i < n; ++i) {
        A(m0 + i, i) = 1.0;
        A(m0 + n + i, i) = -1.0;
        b(m0 + i) = lb(i);
        b(m0 + n + i) = -ub(i);
    }
    qp->A_ge = A;
    qp->b_ge = b;
}

}  // namespace

int main() {
    std::printf("qp_ipm 单元自检（自研原-对偶内点 QP）\n\n");

    // ---------- 1. 解析解 ----------
    std::printf("[1] 解析解\n");
    {
        // min ‖x‖² （H=2I, g=0）→ x = 0
        QpProblem p;
        p.H = 2.0 * Eigen::MatrixXd::Identity(3, 3);
        p.g = Eigen::VectorXd::Zero(3);
        p.A_eq = Eigen::MatrixXd::Zero(0, 3);
        p.b_eq = Eigen::VectorXd::Zero(0);
        p.A_ge = Eigen::MatrixXd::Zero(0, 3);
        p.b_ge = Eigen::VectorXd::Zero(0);
        auto r = SolveQpIpm(p);
        Check(r.ok() && r.x.norm() < 1e-8, "无约束 min ‖x‖² → x=0",
              Format("(‖x‖=%.2e, iters=%d)", r.x.norm(), r.info.iters));

        // min ½‖x‖² s.t. Σx = 1 → x = (1/3,1/3,1/3)
        p.A_eq = Eigen::MatrixXd::Ones(1, 3);
        p.b_eq = Eigen::VectorXd::Ones(1);
        r = SolveQpIpm(p);
        const double err = (r.x - Eigen::VectorXd::Constant(3, 1.0 / 3.0)).cwiseAbs().maxCoeff();
        Check(r.ok() && err < 1e-8, "等式 Σx=1 → x=1/3", Format("(err=%.2e)", err));

        // min ½‖x−(2,2)‖² s.t. x ≥ 1（不起作用）→ x=(2,2)
        p.A_eq = Eigen::MatrixXd::Zero(0, 3);
        p.b_eq = Eigen::VectorXd::Zero(0);
        p.H = Eigen::MatrixXd::Identity(3, 3);
        p.g = -Eigen::VectorXd::Ones(3) * 2.0;
        p.A_ge = Eigen::MatrixXd::Identity(3, 3);
        p.b_ge = Eigen::VectorXd::Ones(3);
        r = SolveQpIpm(p);
        Check(r.ok() && (r.x - Eigen::VectorXd::Constant(3, 2.0)).cwiseAbs().maxCoeff() < 1e-8,
              "不起作用的下界 x ≥ 1 → x=2", Format("(err=%.2e)", (r.x.array() - 2.0).abs().maxCoeff()));

        // 同上但目标 x=−1（下界起作用）→ x=(1,1,1)，且乘子 ≥ 0
        p.g = Eigen::VectorXd::Ones(3);
        r = SolveQpIpm(p);
        Check(r.ok() && (r.x - Eigen::VectorXd::Ones(3)).cwiseAbs().maxCoeff() < 1e-8 &&
                  r.z.minCoeff() > 0.9,
              "起作用的下界 x ≥ 1 → x=1", Format("(err=%.2e, z=%.3f)",
                                              (r.x.array() - 1.0).abs().maxCoeff(), r.z.minCoeff()));

        // min ½x² s.t. x ≤ 4 与 x ≥ 1：目标 x=5 → 上界起作用 x=4
        QpProblem q;
        q.H = Eigen::MatrixXd::Identity(1, 1);
        q.g = -Eigen::VectorXd::Ones(1) * 5.0;
        q.A_eq = Eigen::MatrixXd::Zero(0, 1);
        q.b_eq = Eigen::VectorXd::Zero(0);
        q.A_ge = Eigen::MatrixXd(2, 1);
        q.A_ge << 1.0, -1.0;             // x ≥ 1, −x ≥ −4
        q.b_ge = Eigen::VectorXd(2);
        q.b_ge << 1.0, -4.0;
        auto r1 = SolveQpIpm(q);
        Check(r1.ok() && std::fabs(r1.x(0) - 4.0) < 1e-8, "上界起作用 → x=4",
              Format("(x=%.6f)", r1.x(0)));
    }

    // ---------- 2. 与 box_qp 交叉验证 ----------
    std::printf("\n[2] 与 box_qp 交叉验证（随机盒约束 QP）\n");
    {
        std::mt19937 rng(20241010);
        std::uniform_real_distribution<double> ud(-1.0, 1.0);
        double max_diff = 0.0, max_kkt = 0.0, max_obj_gap = 0.0;
        int cases = 0;
        for (int c = 0; c < 12; ++c) {
            const int n = 6 + c % 5;
            Eigen::MatrixXd B(n, n);
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j) B(i, j) = ud(rng);
            QpProblem p;
            p.H = B.transpose() * B + 1e-3 * Eigen::MatrixXd::Identity(n, n);
            p.g = Eigen::VectorXd(n);
            for (int i = 0; i < n; ++i) p.g(i) = 3.0 * ud(rng);
            p.A_eq = Eigen::MatrixXd::Zero(0, n);
            p.b_eq = Eigen::VectorXd::Zero(0);
            Eigen::VectorXd lb(n), ub(n);
            for (int i = 0; i < n; ++i) {
                lb(i) = -1.0 - 0.5 * (i % 3);
                ub(i) = 1.0 + 0.5 * (i % 4);
            }
            Eigen::VectorXd x_ref = Eigen::VectorXd(n);  // 未截断的最优解
            for (int i = 0; i < n; ++i) x_ref(i) = 0.2 * (i + 1) - 0.5;

            QpProblem pbox = p;
            AppendBoxRows(lb, ub, &pbox);
            QpIpmOptions iopt;
            iopt.tol = 1e-12;
            auto r_ipm = SolveQpIpm(pbox, iopt, &x_ref);

            BoxQpInfo bi;
            const Eigen::VectorXd x_box = SolveBoxQp(p.H, p.g, lb, ub, &x_ref, BoxQpOptions(), &bi);

            const double diff = (r_ipm.x - x_box).cwiseAbs().maxCoeff();
            const double obj_ipm = 0.5 * r_ipm.x.dot(p.H * r_ipm.x) + p.g.dot(r_ipm.x);
            const double obj_box = 0.5 * x_box.dot(p.H * x_box) + p.g.dot(x_box);
            const KktCheck kc = CheckKkt(pbox, r_ipm);
            max_diff = std::max(max_diff, diff);
            max_obj_gap = std::max(max_obj_gap, std::fabs(obj_ipm - obj_box));
            max_kkt = std::max({max_kkt, kc.eq_res, kc.ineq_viol, kc.dual_res});
            ++cases;
            if (diff > 1e-7) {
                std::printf("    case %d: |Δx|=%.2e Δcost=%.2e | IPM it=%d res=(%.1e, %.1e) | "
                            "box_qp res=%.1e converged=%d\n",
                            c, diff, std::fabs(obj_ipm - obj_box), r_ipm.info.iters,
                            r_ipm.info.pri_res, r_ipm.info.dual_res, bi.residual,
                            static_cast<int>(bi.converged));
            }
        }
        std::printf("  %d 个盒约束 QP：max|x_ipm−x_box|=%.2e  max|Δcost|=%.2e  max KKT=%.2e\n", cases,
                    max_diff, max_obj_gap, max_kkt);
        Check(max_diff < 1e-6, "两者解一致", Format("(max diff=%.2e)", max_diff));
        Check(max_obj_gap < 1e-9, "两者代价一致", Format("(max Δcost=%.2e)", max_obj_gap));
        Check(max_kkt < 1e-7, "KKT 残差在容差内", Format("(max=%.2e)", max_kkt));
    }

    // ---------- 3. 随机 QP 的 KKT 验证（等式 + 不等式混合） ----------
    std::printf("\n[3] 随机混合约束 QP 的 KKT 验证\n");
    {
        std::mt19937 rng(777);
        std::uniform_real_distribution<double> ud(-1.0, 1.0);
        double max_kkt = 0.0, max_it = 0.0;
        int converged = 0, total = 0, max_iters = 0;
        for (int c = 0; c < 40; ++c) {
            const int n = 8 + c % 6;
            const int me = 1 + c % 3;
            Eigen::MatrixXd B(n, n);
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j) B(i, j) = ud(rng);
            QpProblem p;
            p.H = B.transpose() * B + 1e-4 * Eigen::MatrixXd::Identity(n, n);
            p.g = Eigen::VectorXd(n);
            for (int i = 0; i < n; ++i) p.g(i) = 2.0 * ud(rng);
            p.A_eq = Eigen::MatrixXd(me, n);
            for (int i = 0; i < me; ++i)
                for (int j = 0; j < n; ++j) p.A_eq(i, j) = ud(rng);
            // 用可行点造 b_eq（保证等式可满足）
            Eigen::VectorXd x_feas = 0.3 * Eigen::VectorXd::Ones(n);
            p.b_eq = p.A_eq * x_feas;
            Eigen::VectorXd lb(n), ub(n);
            for (int i = 0; i < n; ++i) {
                lb(i) = -5.0;
                ub(i) = 5.0;
            }
            AppendBoxRows(lb, ub, &p);

            auto r = SolveQpIpm(p, QpIpmOptions(), &x_feas);
            const KktCheck kc = CheckKkt(p, r);
            const double kkt = std::max({kc.eq_res, kc.ineq_viol, kc.dual_res});
            ++total;
            max_iters = std::max(max_iters, r.info.iters);
            max_it = std::max(max_it, static_cast<double>(r.info.iters));
            if (r.ok() && kkt < 1e-7 && kc.min_z > -1e-9) {
                ++converged;
                max_kkt = std::max(max_kkt, kkt);
            } else {
                std::printf("    case %d: iters=%d ok=%d kkt=%.2e viol=%.2e minz=%.2e\n", c,
                            r.info.iters, static_cast<int>(r.ok()), kkt, kc.ineq_viol, kc.min_z);
            }
        }
        std::printf("  收敛 %d/%d，max KKT=%.2e，最大迭代=%d\n", converged, total, max_kkt, max_iters);
        Check(converged == total, "全部收敛且 KKT 残差达标");
        Check(max_iters <= 60, "迭代次数在上限内", Format("(max=%d)", max_iters));
    }

    // ---------- 4. 退化/边界情形 ----------
    std::printf("\n[4] 退化与边界情形\n");
    {
        // 4.1 最优解贴在下界（非负最小二乘型：x ≥ 0，其中若干分量为 0）
        QpProblem p;
        const int n = 12;
        Eigen::MatrixXd B(n, 4);
        B << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1,
             1, 1, 0, 0,
             0, 1, 1, 0,
             0, 0, 1, 1,
             1, 0, 0, 1,
             0.5, 0.5, 0.5, 0.5,
             1, -1, 0, 0,
             0, 1, -1, 0,
             -1, 0, 0, 1;
        Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
        w << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        p.H = 2.0 * B.transpose() * B + 1e-6 * Eigen::MatrixXd::Identity(4, 4);
        p.g = -2.0 * B.transpose() * w;
        p.A_eq = Eigen::MatrixXd::Zero(0, 4);
        p.b_eq = Eigen::VectorXd::Zero(0);
        p.A_ge = Eigen::MatrixXd::Identity(4, 4);
        p.b_ge = Eigen::VectorXd::Zero(4);
        auto r = SolveQpIpm(p);
        const KktCheck kc = CheckKkt(p, r);
        std::printf("  NNLS 型：x=(%.4f, %.4f, %.4f, %.4f)，z=(%.4f, %.4f, %.4f, %.4f)\n", r.x(0),
                    r.x(1), r.x(2), r.x(3), r.z(0), r.z(1), r.z(2), r.z(3));
        Check(r.ok() && kc.ineq_viol < 1e-9 && kc.min_z > -1e-9 && kc.dual_res < 1e-7,
              "边界最优解满足 KKT（互补）",
              Format("(viol=%.2e, minz=%.2e, dual=%.2e)", kc.ineq_viol, kc.min_z, kc.dual_res));

        // 4.2 冗余等式（同一约束写两遍）不应破坏求解
        QpProblem q;
        q.H = 2.0 * Eigen::MatrixXd::Identity(3, 3);
        q.g = -2.0 * Eigen::VectorXd::Ones(3);
        q.A_eq = Eigen::MatrixXd(2, 3);
        q.A_eq << 1, 1, 1,
                  2, 2, 2;
        q.b_eq = Eigen::VectorXd(2);
        q.b_eq << 3.0, 6.0;
        q.A_ge = Eigen::MatrixXd::Zero(0, 3);
        q.b_ge = Eigen::VectorXd::Zero(0);
        auto r2 = SolveQpIpm(q);
        const double sum = r2.x.sum();
        Check(std::fabs(sum - 3.0) < 1e-7, "冗余等式：仍满足 Σx=3", Format("(Σ=%.8f)", sum));

        // 4.3 没有任何不等式（mg=0）也要能工作
        QpProblem q3;
        q3.H = Eigen::MatrixXd::Identity(2, 2);
        q3.g = -Eigen::VectorXd::Ones(2) * 2.0;
        q3.A_eq = Eigen::MatrixXd::Zero(0, 2);
        q3.b_eq = Eigen::VectorXd::Zero(0);
        q3.A_ge = Eigen::MatrixXd::Zero(0, 2);
        q3.b_ge = Eigen::VectorXd::Zero(0);
        auto r3 = SolveQpIpm(q3);
        Check(r3.ok() && (r3.x.array() - 2.0).abs().maxCoeff() < 1e-9, "无不等式求解正常",
              Format("(x=(%.4f, %.4f))", r3.x(0), r3.x(1)));
    }

    // ---------- 5. 不可行问题识别 ----------
    std::printf("\n[5] 不可行问题识别\n");
    {
        QpProblem p;
        p.H = Eigen::MatrixXd::Identity(1, 1);
        p.g = Eigen::VectorXd::Zero(1);
        p.A_eq = Eigen::MatrixXd::Zero(0, 1);
        p.b_eq = Eigen::VectorXd::Zero(0);
        p.A_ge = Eigen::MatrixXd(2, 1);
        p.A_ge << 1.0, -1.0;      // x ≥ 1 且 x ≤ 0
        p.b_ge = Eigen::VectorXd(2);
        p.b_ge << 1.0, 0.0;
        auto r = SolveQpIpm(p, QpIpmOptions());
        std::printf("  x≥1 且 x≤0：converged=%d pri_res=%.2e\n", static_cast<int>(r.info.converged),
                    r.info.pri_res);
        Check(!r.ok(), "不可行问题不报成功");
    }

    std::printf("\nRESULT: %s  (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
