/**
 * @file lite3_dynamics_test.cpp
 * @brief S2 全身动力学对表：读取 gen_mujoco_dynamics_cases.py 生成的真值，比较
 *        mpc_wbc/00_lite3_model.hpp 的 M(q) / h(q,qvel)，并做接触雅可比自检。
 *
 * 用法： ./lite3_dynamics_test <dyn_cases.txt>
 * 返回 0 表示通过。
 */
#include "00_lite3_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

using namespace mpc_wbc;

namespace {
struct ErrorStats {
    double max_abs = 0.0;
    double sum_sq = 0.0;
    long count = 0;
    void Add(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
        const Eigen::MatrixXd e = (a - b).cwiseAbs();
        max_abs = std::max(max_abs, e.maxCoeff());
        sum_sq += e.squaredNorm();
        count += static_cast<long>(e.size());
    }
    double Rms() const { return count ? std::sqrt(sum_sq / count) : 0.0; }
};

bool Check(const char* name, const ErrorStats& s, double tol) {
    const bool ok = s.max_abs < tol;
    std::printf("  %-32s max_abs=%.3e  rms=%.3e  n=%ld  %s\n",
                name, s.max_abs, s.Rms(), s.count, ok ? "OK" : "FAIL");
    return ok;
}
}  // namespace

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "mpc_wbc/test/dyn_cases.txt";
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open cases file: " << path << "\n";
        return 2;
    }

    int n = 0;
    in >> n;
    if (n <= 0) {
        std::cerr << "empty cases file\n";
        return 2;
    }

    Lite3Model model;

    ErrorStats m_err, h_err, jc_err, com_err, inertia_err;
    double max_mass_block_err = 0.0;
    double max_asym = 0.0;
    double min_eig = 1e300;

    for (int c = 0; c < n; ++c) {
        Vec3d base_pos, gravity, com_ref;
        Vec4d quat;
        Vec12d q;
        Vec18d qvel;
        for (int i = 0; i < 3; ++i) in >> base_pos(i);
        for (int i = 0; i < 4; ++i) in >> quat(i);
        for (int i = 0; i < 12; ++i) in >> q(i);
        for (int i = 0; i < 18; ++i) in >> qvel(i);
        for (int i = 0; i < 3; ++i) in >> gravity(i);
        for (int i = 0; i < 3; ++i) in >> com_ref(i);

        Mat18d M_ref;
        for (int i = 0; i < 18; ++i)
            for (int j = 0; j < 18; ++j) in >> M_ref(i, j);
        Vec18d bias_ref;
        for (int i = 0; i < 18; ++i) in >> bias_ref(i);
        Mat12x18d Jc_ref;
        for (int i = 0; i < 12; ++i)
            for (int j = 0; j < 18; ++j) in >> Jc_ref(i, j);

        const Mat3d R_B = QuatToRotation(quat);

        const Mat18d M = model.MassMatrix(q, R_B);
        m_err.Add(M, M_ref);
        const Vec18d h = model.BiasForce(q, qvel, R_B, gravity);
        h_err.Add(h, bias_ref);
        jc_err.Add(model.ContactJacobianWorld(q, R_B), Jc_ref);

        // S6 用到的整机质心与合成惯量：分别对 MuJoCo 的 subtree_com 与 M(q) 的
        // 浮动基旋转块（= 绕浮动基原点的合成惯量，机体系表达）。
        com_err.Add(model.CompositeComWorld(q, R_B, base_pos), com_ref);
        inertia_err.Add(model.CompositeInertiaBody(q, Vec3d::Zero()), M_ref.block<3, 3>(3, 3));

        // 结构检查：左上 3x3 应等于总质量 × I；M 对称；正定。
        const double mt = model.TotalMass();
        max_mass_block_err = std::max(
            max_mass_block_err, (M.block<3, 3>(0, 0) - mt * Mat3d::Identity()).cwiseAbs().maxCoeff());
        max_asym = std::max(max_asym, (M - M.transpose()).cwiseAbs().maxCoeff());
        Eigen::SelfAdjointEigenSolver<Mat18d> es(M);
        min_eig = std::min(min_eig, es.eigenvalues().minCoeff());
    }

    std::printf("Lite3 dynamics table-check vs MuJoCo  (%d cases)\n", n);
    bool ok = true;
    ok &= Check("Mass matrix M(q)", m_err, 1e-9);
    ok &= Check("Bias force h(q,qvel)", h_err, 1e-9);
    ok &= Check("Contact Jacobian Jc", jc_err, 1e-9);
    ok &= Check("整机质心 (subtree_com)", com_err, 1e-9);
    ok &= Check("整机合成惯量绕基座原点", inertia_err, 1e-9);
    std::printf("  %-32s max_abs=%.3e  (expect ~0)\n", "M base 3x3 vs m_total*I", max_mass_block_err);
    std::printf("  %-32s max_abs=%.3e  (expect ~0)\n", "M asymmetry", max_asym);
    std::printf("  %-32s min_eig=%.3e  (expect >0)\n", "M smallest eigenvalue", min_eig);
    if (max_mass_block_err > 1e-9 || max_asym > 1e-9 || min_eig <= 0.0) ok = false;

    std::printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
