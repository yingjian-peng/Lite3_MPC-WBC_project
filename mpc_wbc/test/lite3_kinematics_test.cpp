/**
 * @file lite3_kinematics_test.cpp
 * @brief S1 运动学对表程序：读取 gen_mujoco_cases.py 生成的 MuJoCo 真值，逐项比较
 *        本工程 mpc_wbc/00_lite3_model.hpp 的 FK / 单腿雅可比 / 全身足端雅可比 / IK 回代。
 *
 * 用法：
 *    ./lite3_kinematics_test <cases.txt>
 * 返回 0 表示全部通过阈值；非 0 表示失败。
 */
#include "00_lite3_model.hpp"

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
    std::printf("  %-34s max_abs=%.3e  rms=%.3e  n=%ld  %s\n",
                name, s.max_abs, s.Rms(), s.count, ok ? "OK" : "FAIL");
    return ok;
}
}  // namespace

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "mpc_wbc/test/cases.txt";
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

    ErrorStats fk_body_err;       // FK(机体系) vs MuJoCo
    ErrorStats leg_jac_err;       // 单腿雅可比(机体系) vs MuJoCo
    ErrorStats full_jac_err;      // 全身足端雅可比(世界系) vs MuJoCo
    ErrorStats ik_roundtrip_err;  // 无种子 IK：FK(IK(p)) vs p
    ErrorStats ik_seeded_q_err;   // 带种子 IK：恢复的关节角 vs 采样 q
    int ik_fail = 0;

    for (int c = 0; c < n; ++c) {
        Vec3d base_pos;
        Vec4d quat;
        Vec12d q;
        for (int i = 0; i < 3; ++i) in >> base_pos(i);
        for (int i = 0; i < 4; ++i) in >> quat(i);
        for (int i = 0; i < 12; ++i) in >> q(i);

        Eigen::Matrix<double, 3, kNumLegs> ref_foot_world;
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int i = 0; i < 3; ++i) in >> ref_foot_world(i, leg);

        Mat12x18d ref_jac;
        for (int r = 0; r < 12; ++r)
            for (int col = 0; col < kNumGenVel; ++col) in >> ref_jac(r, col);

        const Mat3d R_WB = QuatToRotation(quat);

        // ---- FK：机体系足位 vs MuJoCo（把世界足位转回机体系） ----
        const Eigen::Matrix<double, 3, kNumLegs> foot_body = model.FootPositionsBody(q);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d ref_body = R_WB.transpose() * (ref_foot_world.col(leg) - base_pos);
            fk_body_err.Add(foot_body.col(leg), ref_body);
        }

        // ---- 全身足端雅可比：世界系 12x18 ----
        full_jac_err.Add(model.FootJacobianWorld(q, R_WB), ref_jac);

        // ---- 单腿雅可比 + IK 回代 ----
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d q_leg = q.segment<3>(3 * leg);
            const Vec3d p_body = foot_body.col(leg);

            const Mat3d J_leg_cpp = model.LegJacobianBody(leg, q_leg);
            // MuJoCo 中该腿 3 个关节对应 qvel 的第 6+3*leg .. 8+3*leg 列
            const Mat3d J_leg_ref =
                R_WB.transpose() * ref_jac.block<3, 3>(3 * leg, 6 + 3 * leg);
            leg_jac_err.Add(J_leg_cpp, J_leg_ref);

            Vec3d q_ik;
            if (!model.InverseKinematics(leg, p_body, &q_ik)) {
                ++ik_fail;
                continue;
            }
            ik_roundtrip_err.Add(model.FootPositionBody(leg, q_ik), p_body);

            // 带种子（上一帧关节角）时，应恢复出原始采样角。
            Vec3d q_ik_seeded;
            if (model.InverseKinematics(leg, p_body, &q_ik_seeded, &q_leg))
                ik_seeded_q_err.Add(q_ik_seeded, q_leg);
        }
    }

    std::printf("Lite3 kinematics table-check vs MuJoCo  (%d cases)\n", n);
    bool ok = true;
    ok &= Check("FK body-frame [m]", fk_body_err, 1e-9);
    ok &= Check("Leg Jacobian body [m/rad]", leg_jac_err, 1e-7);
    ok &= Check("Full foot Jacobian world", full_jac_err, 1e-7);
    ok &= Check("IK round-trip FK(IK(p)) [m]", ik_roundtrip_err, 1e-9);
    ok &= Check("IK seeded joint recovery [rad]", ik_seeded_q_err, 1e-9);
    std::printf("  %-34s %d / %d\n", "IK unreachable", ik_fail, n * kNumLegs);
    if (ik_fail != 0) ok = false;

    std::printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
