/**
 * @file lite3_wbc_test.cpp
 * @brief S7 自检：Weighted-QP WBC 的物理正确性 + 接触偏置加速度对表。
 *
 * 用法： ./lite3_wbc_test [contact_accel_cases.txt] [wbc_mujoco_cases.csv]
 *
 * 检查项：
 *   [1] 接触点偏置加速度 Jdot·v 对表 MuJoCo（用 mj_integratePos + 中心差分测出的接触加速度）
 *   [2] 静立悬停：四脚支撑时 Σfz ≈ m·g、净力矩 ≈ 0、EoM/接触残差 ~0、摩擦锥满足、τ 在限幅内
 *   [3] 基座 6D 加速度跟踪（±x/±y/偏航）+ 与 MPC 期望力的方向一致
 *   [4] 摆动腿任务空间 PD + 支撑腿承重 + 摆动脚受力恒为 0（含 qvel≠0 的科氏项）
 *   [5] 极限情形：大期望加速度时摩擦锥起作用、力矩不超限、锥违反 ≈ 0
 *   [6] 与 SRBD-MPC 串联（MPC → ComputeBaseAccelDesired → WBC），悬停净力与 MPC 一致
 *   [7] 求解耗时（200 帧 trot 计划）
 *   [8] 失败兜底（kStale / kFailed）
 *   [9] 低层 PD 反解：tau_ff = τ_wbc − kp(q_des−q) − kd(dq_des−dq) 代入执行律可精确复现 τ_wbc
 *   [10] 导出 (q, qvel, qdd, f, τ) 供 check_wbc_hover_mujoco.py 在 MuJoCo 里独立复核
 *
 * 返回 0 表示通过。
 */
#include "04_weighted_wbc.hpp"

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iomanip>
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

/// 名义站立构型：球足最低点刚好落地。
struct Rig {
    Lite3Model model;
    Vec12d q = Lite3Model::DefaultStandJointAngles();
    Mat3d R = Mat3d::Identity();
    Vec3d p_base = Vec3d::Zero();
    Vec3d p_com = Vec3d::Zero();
    Mat3d I_com_world = Mat3d::Zero();
    std::array<Vec3d, kNumLegs> feet{};

    Rig() {
        double min_z = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg)
            min_z = std::min(min_z, model.FootPositionBody(leg, q.segment<3>(3 * leg)).z());
        p_base = Vec3d(0.0, 0.0, -min_z + model.geo.foot_radius);
        p_com = model.CompositeComWorld(q, R, p_base);
        I_com_world = model.CompositeInertiaWorld(q, R, model.CompositeComBody(q));
        for (int leg = 0; leg < kNumLegs; ++leg)
            feet[leg] = p_base + R * model.FootPositionBody(leg, q.segment<3>(3 * leg));
    }

    /// 填入状态相关字段（qvel、足端位置/速度、R、q）。
    void FillBase(WbcInput* in, const Vec18d& qvel) const {
        in->q = q;
        in->R_wb = R;
        in->qvel = qvel;
        const Mat12x18d Jc = model.ContactJacobianWorld(q, R);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            in->foot_pos_world[leg] = feet[leg];
            in->foot_vel_world[leg] = Jc.block<3, kWbcQdd>(3 * leg, 0) * qvel;
            in->foot_des_world[leg] = feet[leg];
            in->foot_vel_des_world[leg] = Vec3d::Zero();
        }
    }
};

const char* LegName(int leg) { return leg == kFL ? "FL" : leg == kFR ? "FR" : leg == kHL ? "HL" : "HR"; }

/// 支撑腿摩擦锥/法向力检查（世界系，z 向上）。
bool ConeOk(const std::array<Vec3d, kNumLegs>& f, const std::array<bool, kNumLegs>& stance,
            double mu, double fz_max, double tol = 1e-6) {
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!stance[leg]) {
            if (f[leg].norm() > tol) return false;
            continue;
        }
        if (f[leg].z() < -tol) return false;
        if (std::fabs(f[leg].x()) > mu * f[leg].z() + tol) return false;
        if (std::fabs(f[leg].y()) > mu * f[leg].z() + tol) return false;
        if (fz_max > 0.0 && f[leg].z() > fz_max + tol) return false;
    }
    return true;
}

double WeightTotalZ(const std::array<Vec3d, kNumLegs>& f) {
    double s = 0.0;
    for (const Vec3d& v : f) s += v.z();
    return s;
}

/// 导出给 MuJoCo 复核用的行：(q12, qvel18, qdd18, f12, tau12, R9, stance4)
struct DumpRows {
    struct Row {
        Vec12d q;
        Vec18d qvel, qdd;
        std::array<Vec3d, kNumLegs> f;
        Vec12d tau;
        Mat3d R;
        std::array<bool, kNumLegs> stance{};
    };
    std::vector<Row> rows;
};

void WriteDump(const std::string& path, const DumpRows& d) {
    std::ofstream out(path);
    if (!out) {
        std::printf("  （警告：无法写入 %s，跳过 MuJoCo 复核用的导出）\n", path.c_str());
        return;
    }
    // 必须给足有效位：MuJoCo 复核要做正动力学（M·qacc + h = Jᵀf + Sᵀτ），f/τ 少一位有效数字
    // 就会被腿的小惯量放大成 ~1e-3 rad/s² 的假误差（默认 6 位有效数字踩过这个坑）。
    out << std::setprecision(17);
    for (const auto& row : d.rows) {
        for (int i = 0; i < 12; ++i) out << row.q(i) << " ";
        for (int i = 0; i < 18; ++i) out << row.qvel(i) << " ";
        for (int i = 0; i < 18; ++i) out << row.qdd(i) << " ";
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int c = 0; c < 3; ++c) out << row.f[leg](c) << " ";
        for (int i = 0; i < 12; ++i) out << row.tau(i) << " ";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) out << row.R(i, j) << " ";
        for (int leg = 0; leg < kNumLegs; ++leg) out << (row.stance[leg] ? 1 : 0) << " ";
        out << "\n";
    }
    std::printf("  已导出 %zu 个复核用例到 %s\n", d.rows.size(), path.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    const std::string accel_cases_path = (argc > 1) ? argv[1] : std::string();
    const std::string dump_path = (argc > 2) ? argv[2] : std::string("wbc_mujoco_cases.csv");

    Rig rig;
    const double m = rig.model.TotalMass();
    const double g = 9.81;
    const double weight = m * g;
    WeightedWbc wbc;
    DumpRows dump;

    std::printf("S7 Weighted-QP WBC 自检\n");
    std::printf("  mass=%.4f kg  weight=%.2f N  tau_max=%.1f N·m  mu=%.2f  w_base=%.0f  "
                "w_swing=%.0f  w_f_ref=%.1f\n\n",
                m, weight, wbc.params().tau_max, wbc.params().mu, wbc.params().w_base,
                wbc.params().w_swing, wbc.params().w_f_ref);

    // ---------- 1. 接触偏置加速度对表（MuJoCo） ----------
    std::printf("[1] 接触点偏置加速度 Jdot·v 对表 MuJoCo\n");
    if (!accel_cases_path.empty()) {
        std::ifstream in(accel_cases_path);
        if (!in) {
            std::printf("  无法打开 %s（跳过）\n\n", accel_cases_path.c_str());
        } else {
            int n = 0;
            in >> n;
            double max_err = 0.0, max_acc = 0.0;
            for (int c = 0; c < n && in; ++c) {
                Vec3d base_pos;
                Vec4d quat;
                Vec12d q;
                Vec18d qvel, qdd;
                for (int i = 0; i < 3; ++i) in >> base_pos(i);
                for (int i = 0; i < 4; ++i) in >> quat(i);
                for (int i = 0; i < 12; ++i) in >> q(i);
                for (int i = 0; i < 18; ++i) in >> qvel(i);
                for (int i = 0; i < 18; ++i) in >> qdd(i);
                VecContact12d a_ref;
                for (int i = 0; i < 12; ++i) in >> a_ref(i);
                if (!in) break;
                const Mat3d R = QuatToRotation(quat);
                const VecContact12d a_mine =
                    rig.model.ContactJacobianWorld(q, R) * qdd +
                    rig.model.ContactBiasAccel(q, R, qvel);
                max_err = std::max(max_err, (a_mine - a_ref).cwiseAbs().maxCoeff());
                max_acc = std::max(max_acc, a_ref.cwiseAbs().maxCoeff());
            }
            std::printf("  %d 个随机工况：max|Jc·q̈ + Jdot·q̇ − a_c^MuJoCo| = %.3e m/s²（真值量级 %.1f）\n",
                        n, max_err, max_acc);
            Check(max_err < 1e-6, "接触加速度恒等式与 MuJoCo 一致",
                  Format("(max err=%.2e)", max_err));
        }
    } else {
        std::printf("  （未提供真值文件，跳过；用 mpc_wbc/test/run_wbc_test.sh 会自动生成）\n");
    }

    // ---------- 2. 静立悬停 ----------
    std::printf("\n[2] 静立悬停（四脚支撑，期望加速度 0）\n");
    {
        WbcInput in;
        rig.FillBase(&in, Vec18d::Zero());
        for (int leg = 0; leg < kNumLegs; ++leg) in.foot_force_ref[leg] = Vec3d(0, 0, weight / 4.0);
        const WbcOutput out = wbc.SolveOnce(in);
        const double sum_fz = WeightTotalZ(out.foot_force);
        Vec3d net_m(0, 0, 0), net_f(0, 0, 0);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            net_f += out.foot_force[leg];
            net_m += (rig.feet[leg] - rig.p_com).cross(out.foot_force[leg]);
        }
        std::printf("  Σfz=%.4f N (mg=%.4f)  净力=(%.2e, %.2e, %.2e)  净力矩 |M|=%.2e Nm\n", sum_fz,
                    weight, net_f.x(), net_f.y(), net_f.z(), net_m.norm());
        for (int leg = 0; leg < kNumLegs; ++leg)
            std::printf("   %s f=(%+7.3f, %+7.3f, %8.3f) N  τ=(%+6.2f, %+6.2f, %+6.2f) Nm\n",
                        LegName(leg), out.foot_force[leg].x(), out.foot_force[leg].y(),
                        out.foot_force[leg].z(), out.tau(3 * leg), out.tau(3 * leg + 1),
                        out.tau(3 * leg + 2));
        std::printf("  EoM 残差=%.2e  接触残差=%.2e  锥违反=%.2e  |τ|max=%.2f  base 加速度误差=%.2e  "
                    "iters=%d  gap=%.2e\n",
                    out.eom_residual, out.contact_residual, out.cone_violation, out.tau_abs_max,
                    out.base_accel_error, out.iters, out.qp_gap);
        Check(out.fresh(), "求解成功");
        Check(std::fabs(sum_fz - weight) < 0.01 * weight, "Σfz ≈ m·g",
              Format("(%.4f N, 误差 %.4f%%)", sum_fz, 100.0 * std::fabs(sum_fz - weight) / weight));
        Check(net_m.norm() < 0.5, "净力矩 ≈ 0", Format("(|M|=%.2e Nm)", net_m.norm()));
        Check(out.eom_residual < 1e-9, "EoM 残差 < 1e-9", Format("(%.2e)", out.eom_residual));
        Check(out.contact_residual < 1e-9, "支撑无滑残差 < 1e-9",
              Format("(%.2e)", out.contact_residual));
        Check(out.cone_violation < 1e-6, "摩擦锥满足",
              Format("(违反 %.2e)", out.cone_violation));
        Check(out.tau_abs_max <= wbc.params().tau_max + 1e-9, "力矩不超限",
              Format("(|τ|max=%.3f ≤ %.1f)", out.tau_abs_max, wbc.params().tau_max));
        Check(out.base_accel_error < 1e-4, "基座加速度跟踪 0（误差 < 1e-4 m/s²，受 QP 容差限制）",
              Format("(err=%.2e)", out.base_accel_error));
        Check(ConeOk(out.foot_force, in.stance, wbc.params().mu, wbc.params().foot_force_max),
              "逐脚摩擦锥/上限检查（独立复算）");
        dump.rows.push_back({in.q, in.qvel, out.qdd, out.foot_force, out.tau, in.R_wb, in.stance});
    }

    // ---------- 3. 基座加速度跟踪 ----------
    std::printf("\n[3] 基座期望加速度跟踪\n");
    {
        struct Case {
            const char* name;
            Vec3d a;
            Vec3d alpha;
            int sign_axis;  // 0=x,1=y,2=z
        };
        const Case cases[] = {
            {"+x", Vec3d(2.0, 0, 0), Vec3d::Zero(), 0},
            {"-x", Vec3d(-2.0, 0, 0), Vec3d::Zero(), 0},
            {"+y", Vec3d(0, 2.0, 0), Vec3d::Zero(), 1},
            {"+yaw", Vec3d::Zero(), Vec3d(0, 0, 2.0), 2},
        };
        for (const Case& c : cases) {
            WbcInput in;
            rig.FillBase(&in, Vec18d::Zero());
            in.a_des_world = c.a;
            in.alpha_des_body = c.alpha;
            const WbcOutput out = wbc.SolveOnce(in);
            Vec3d net_f = Vec3d::Zero();
            Vec3d net_m = Vec3d::Zero();
            for (int leg = 0; leg < kNumLegs; ++leg) {
                net_f += out.foot_force[leg];
                net_m += (rig.feet[leg] - rig.p_com).cross(out.foot_force[leg]);
            }
            std::printf("  %-5s a_des=(%.1f, %.1f, %.1f) α_des=(%.1f, %.1f, %.1f)\n", c.name,
                        c.a.x(), c.a.y(), c.a.z(), c.alpha.x(), c.alpha.y(), c.alpha.z());
            std::printf("        → q̈[0:6]=(%+.4f, %+.4f, %+.4f | %+.4f, %+.4f, %+.4f)  净力=(%+.2f, "
                        "%+.2f, %.2f) N  M_z=%+.3f Nm  锥违反=%.1e  EoM=%.1e\n",
                        out.qdd(0), out.qdd(1), out.qdd(2), out.qdd(3), out.qdd(4), out.qdd(5),
                        net_f.x(), net_f.y(), net_f.z(), net_m.z(), out.cone_violation,
                        out.eom_residual);
            // 四脚支撑且没有摆动腿任务 → 基座任务不被任何任务竞争，应被**精确**满足
            // （与参考工程 smoke 的「a_des=0.5 → q̈_x=0.5000 精确」同口径）。
            // 若这里退化到 1e-5 量级，多半是又混进了与基座/接触力竞争的软任务（见 S7 记录）。
            Check(out.fresh() && out.base_accel_error < 1e-3 * c.a.norm() + 1e-6,
                  "无竞争任务时基座加速度精确跟踪（相对误差 < 1e-3）",
                  Format("(err=%.2e, |a_des|=%.2f)", out.base_accel_error, c.a.norm()));
            Check(out.eom_residual < 1e-9 && out.contact_residual < 1e-9, "硬约束残差达标",
                  Format("(eom=%.1e, contact=%.1e)", out.eom_residual, out.contact_residual));
            if (c.sign_axis != 2) {
                Check(net_f(c.sign_axis) * c.a(c.sign_axis) > 0.0, "净力方向与期望加速度一致",
                      Format("(f=%+.2f N)", net_f(c.sign_axis)));
            } else {
                Check(net_m.z() * c.alpha.z() > 0.0, "偏航力矩方向正确",
                      Format("(M_z=%+.3f Nm)", net_m.z()));
            }
            Check(ConeOk(out.foot_force, in.stance, wbc.params().mu, wbc.params().foot_force_max),
                  "摩擦锥满足（独立复算）");
        }
    }

    // ---------- 4. 摆动腿任务与支撑承重（含科氏项） ----------
    std::printf("\n[4] 摆动腿任务 + 支撑承重（qvel≠0，含科氏项）\n");
    {
        Vec18d qvel = Vec18d::Zero();
        qvel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.5, 0.3, -1.0, 0.5, -0.3, 0.8, -0.4, 0.2, -0.8,
            0.4, -0.2;  // 关节速度
        WbcInput in;
        rig.FillBase(&in, qvel);
        // 对角支撑（FL/HR）——质心投影落在支撑对角线上，静态可平衡；
        // 若用「同侧/同前后」两脚支撑，质心在支撑多边形外，静态不可行（会被 QP 判成互相打架）。
        in.stance = {{true, false, false, true}};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!in.stance[leg]) {
                in.foot_des_world[leg] = rig.feet[leg] + Vec3d(0.02, 0.0, 0.05);  // 抬脚 + 前移
                in.foot_vel_des_world[leg] = Vec3d(0.1, 0.0, 0.0);
            }
        }
        const WbcOutput out = wbc.SolveOnce(in);
        const double sum_fz = WeightTotalZ(out.foot_force);
        std::printf("  支撑 FL/HR：");
        for (int leg = 0; leg < kNumLegs; ++leg)
            if (in.stance[leg])
                std::printf("%s f=(%+.3f, %+.3f, %7.3f) N  ", LegName(leg), out.foot_force[leg].x(),
                            out.foot_force[leg].y(), out.foot_force[leg].z());
        std::printf("\n  摆动 FR/HL：");
        for (int leg = 0; leg < kNumLegs; ++leg)
            if (!in.stance[leg])
                std::printf("%s |f|=%.2e N  ", LegName(leg), out.foot_force[leg].norm());
        std::printf("\n  Σfz(支撑)=%.3f N  EoM=%.2e  接触残差=%.2e  摆动足任务残差=%.3e m/s²\n",
                    sum_fz, out.eom_residual, out.contact_residual, out.swing_accel_error);
        std::printf("  基座加速度误差=%.3e  |τ|max=%.3f  锥违反=%.1e  q̈[2]=%.4f\n",
                    out.base_accel_error, out.tau_abs_max, out.cone_violation, out.qdd(2));
        Check(out.fresh(), "求解成功");
        bool swing_zero = true;
        for (int leg = 0; leg < kNumLegs; ++leg)
            if (!in.stance[leg] && out.foot_force[leg].norm() > 1e-9) swing_zero = false;
        Check(swing_zero, "摆动脚受力恒为 0（硬等式）");
        Check(out.contact_residual < 1e-8, "支撑无滑残差 < 1e-8（含 Jdot·v）",
              Format("(%.2e)", out.contact_residual));
        // 摆动任务与基座任务是加权折中（w_base/w_swing=10），因此断言的是
        // 「打开任务后残差比关闭任务时降两个数量级」，而不是绝对满足——与参考工程
        // 09_weighted_wbc 的验收口径一致（软任务只保证加权最优）。
        {
            WbcParams no_task = wbc.params();
            no_task.w_swing = 0.0;
            const double swing_err_off = WeightedWbc(no_task).SolveOnce(in).swing_accel_error;
            std::printf("  摆动任务残差：打开=%.3e  关闭=%.3e m/s²（降 %.1f 倍）\n",
                        out.swing_accel_error, swing_err_off,
                        swing_err_off / std::max(out.swing_accel_error, 1e-12));
            Check(out.swing_accel_error < 0.02 * swing_err_off,
                  "摆动足任务空间 PD 主导（残差比关闭任务时降 ≥50 倍）",
                  Format("(%.2e vs %.2e m/s²)", out.swing_accel_error, swing_err_off));
        }
        Check(out.eom_residual < 1e-9, "EoM 残差 < 1e-9（含科氏项）", Format("(%.2e)", out.eom_residual));
        Check(ConeOk(out.foot_force, in.stance, wbc.params().mu, wbc.params().foot_force_max),
              "摩擦锥满足（独立复算）");
        // 注意：这里**不能**再断言 Σfz = m·g —— qvel≠0 时 EoM 的 z 行含四肢的科氏/离心偏置项，
        // 平衡式是 Σfz = m·(g + a_z) + bias（实测偏差 ~2.7%，EoM 残差仍 1e-15）。
        // 静态平衡的检查放在下面的「静止子情形」里，那时 Σfz = m·g 才严格成立。
        dump.rows.push_back({in.q, in.qvel, out.qdd, out.foot_force, out.tau, in.R_wb, in.stance});

        // 另一对角支撑（FR/HL 支撑，FL/HR 摆动）
        WbcInput t;
        rig.FillBase(&t, qvel);
        t.stance = {{false, true, true, false}};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!t.stance[leg]) {
                t.foot_des_world[leg] = rig.feet[leg] + Vec3d(0.05, 0.0, 0.06);
                t.foot_vel_des_world[leg] = Vec3d(0.2, 0.0, 0.1);
            }
        }
        t.a_des_world = Vec3d(0.5, 0.0, 0.0);
        const WbcOutput tout = wbc.SolveOnce(t);
        std::printf("  对角 trot：Σfz=%.3f N  EoM=%.2e  接触=%.2e  摆动任务=%.2e  锥违反=%.1e  iters=%d\n",
                    WeightTotalZ(tout.foot_force), tout.eom_residual, tout.contact_residual,
                    tout.swing_accel_error, tout.cone_violation, tout.iters);
        Check(tout.fresh() && tout.eom_residual < 1e-9 && tout.contact_residual < 1e-8,
              "对角支撑同样满足硬约束");
        Check(ConeOk(tout.foot_force, t.stance, wbc.params().mu, wbc.params().foot_force_max),
              "对角支撑摩擦锥满足");
        dump.rows.push_back({t.q, t.qvel, tout.qdd, tout.foot_force, tout.tau, t.R_wb, t.stance});

        // 静止子情形：qvel=0 时 Σfz = m·g 严格成立（这是「支撑腿承重」的正确检查口径）
        WbcInput stat;
        rig.FillBase(&stat, Vec18d::Zero());
        stat.stance = {{true, false, false, true}};
        // 摆动目标 = 当前足端（不要求摆动腿加速），否则摆动腿的加速反力会额外压地：
        // 实测摆动目标抬 5 cm 时 Σfz = 120.73 N（比 m·g 高 3.1%），那是物理正确的结果。
        const WbcOutput sout = wbc.SolveOnce(stat);
        const double stat_fz = WeightTotalZ(sout.foot_force);
        // 承重的正确口径：Σfz = (M·q̈ + h)_z —— 由模型独立复算（不是 WBC 内部的残差）。
        // 不能用 Σfz = m·g：① 腿部加速度经 M 的非对角块耦合进基座 z 行；② 摆动腿任务
        // (Jc_swing·q̈ = 0) 即使目标=当前足端也要求摆动腿"不产生足端加速度"，它会通过 EoM
        // 与基座任务折中（实测把 q̈_z 拉到 −0.0036 m/s²）。
        // 「Σfz = m·g 严格成立」只适用于**没有摆动腿**的构型（见 [2] 与参考工程 smoke [1]）。
        const Mat18d Ms = rig.model.MassMatrix(stat.q, stat.R_wb);
        const Vec18d hs = rig.model.BiasForce(stat.q, stat.qvel, stat.R_wb, Vec3d(0.0, 0.0, -g));
        const double fz_demand = (Ms * sout.qdd + hs)(2);
        std::printf("  静止对角支撑：Σfz=%.4f N (mg=%.4f, 模型需求 (M·q̈+h)_z=%.4f, q̈_z=%.5f)  "
                    "base err=%.2e\n",
                    stat_fz, weight, fz_demand, sout.qdd(2), sout.base_accel_error);
        Check(std::fabs(stat_fz - fz_demand) < 1e-8, "两脚支撑：Σfz = (M·q̈+h)_z（模型独立复算）",
              Format("(%.6f vs %.6f N)", stat_fz, fz_demand));
        Check(std::fabs(stat_fz - weight) < 0.01 * weight, "两脚支撑承重偏差 < 1%",
              Format("(%.4f N, 误差 %.4f%%)", stat_fz,
                     100.0 * std::fabs(stat_fz - weight) / weight));

        // ---- 加权最优性证书（KKT）：独立组装 QP 并取回乘子 y/z，复查驻点 + 互补 + 对偶可行 ----
        // 作用：证明「两项任务都没被精确满足」不是求解器没收敛，而是给定权重下的最优折中。
        {
            WeightedWbc::Qp qp;
            Check(wbc.BuildQp(in, &qp), "组装 QP（供独立最优性复核）");
            const QpIpmResult r = SolveQpIpm({qp.H, qp.g, qp.A_eq, qp.b_eq, qp.A_ge, qp.b_ge});
            // 驻点按求解器自身的符号约定：H x + g − A_eqᵀ y − A_geᵀ z = 0（见 qp_ipm info.dual_res）
            const Eigen::VectorXd stationarity =
                qp.H * r.x + qp.g - qp.A_eq.transpose() * r.y - qp.A_ge.transpose() * r.z;
            const Eigen::VectorXd slack = qp.A_ge * r.x - qp.b_ge;
            double comp = 0.0;
            for (int i = 0; i < slack.size(); ++i)
                comp = std::max(comp, std::fabs(r.z(i) * slack(i)));
            std::printf("  KKT：驻点=%.2e  互补=%.2e  min(z)=%.2e  gap=%.2e\n",
                        stationarity.cwiseAbs().maxCoeff(), comp, r.z.minCoeff(), r.info.gap);
            Check(stationarity.cwiseAbs().maxCoeff() < 1e-6, "KKT 驻点条件",
                  Format("(%.2e)", stationarity.cwiseAbs().maxCoeff()));
            Check(comp < 1e-6, "KKT 互补松弛", Format("(%.2e)", comp));
            Check(r.z.minCoeff() > -1e-9, "对偶可行 z ≥ 0",
                  Format("(min z=%.2e)", r.z.minCoeff()));
            // 权重敏感性：w_base↑ 则基座误差↓ ⇔ 折中是「权重驱动」的，而不是被某个 bug 钉住
            double prev_base = 1e9;
            bool monotone = true;
            for (double wb : {1.0e3, 1.0e4, 1.0e5}) {
                WbcParams pp = wbc.params();
                pp.w_base = wb;
                const WbcOutput o = WeightedWbc(pp).SolveOnce(in);
                std::printf("    w_base=%.0e → 基座误差=%.3e  摆动残差=%.3e\n", wb, o.base_accel_error,
                            o.swing_accel_error);
                if (!(o.base_accel_error < prev_base)) monotone = false;
                prev_base = o.base_accel_error;
            }
            Check(monotone, "基座误差随 w_base 单调下降（折中由权重驱动）");
            double prev_swing = 1e9;
            bool mono_s = true;
            for (double ws : {1.0e2, 1.0e3, 1.0e4}) {
                WbcParams pp = wbc.params();
                pp.w_swing = ws;
                const WbcOutput o = WeightedWbc(pp).SolveOnce(in);
                if (!(o.swing_accel_error < prev_swing)) mono_s = false;
                prev_swing = o.swing_accel_error;
            }
            Check(mono_s, "摆动任务残差随 w_swing 单调下降");
        }
    }

    // ---------- 5. 极限情形 ----------
    std::printf("\n[5] 极限情形（大期望加速度 / 悬空腿）\n");
    {
        WbcInput in;
        rig.FillBase(&in, Vec18d::Zero());
        in.a_des_world = Vec3d(6.0, 4.0, 0.0);
        in.alpha_des_body = Vec3d(0.0, 0.0, 5.0);
        const WbcOutput out = wbc.SolveOnce(in);
        std::printf("  a_des=(6,4,0), α_des=(0,0,5)：锥违反=%.2e  |τ|max=%.3f  接触残差=%.2e  EoM=%.2e  iters=%d\n",
                    out.cone_violation, out.tau_abs_max, out.contact_residual, out.eom_residual,
                    out.iters);
        Check(out.fresh(), "极限工况仍能求解");
        Check(out.cone_violation < 1e-6, "摩擦锥未被违反",
              Format("(违反 %.2e)", out.cone_violation));
        Check(out.tau_abs_max <= wbc.params().tau_max + 1e-9, "力矩不超限",
              Format("(|τ|max=%.3f)", out.tau_abs_max));
        Check(out.eom_residual < 1e-9 && out.contact_residual < 1e-8, "硬约束残差达标");

        // 全摆动 → 无可支撑，应报失败（保护）
        WbcInput all_swing;
        rig.FillBase(&all_swing, Vec18d::Zero());
        all_swing.stance = {{false, false, false, false}};
        WeightedWbc fresh;
        const WbcOutput bad = fresh.Solve(1.0, all_swing);
        Check(!bad.ok(), "全摆动（无支撑）不返回可用解");
    }

    // ---------- 6. 与 SRBD-MPC 串联 ----------
    std::printf("\n[6] 与 SRBD-MPC 串联（MPC → ComputeBaseAccelDesired → WBC）\n");
    {
        SrbdPreviewMpc mpc;
        SrbdMpcInput mpc_in;
        mpc_in.p_com = rig.p_com;
        mpc_in.inertia_com_world = rig.I_com_world;
        mpc_in.foot_pos_world = rig.feet;
        mpc_in.height_cmd = rig.p_com.z();
        mpc_in.contact.assign(mpc.params().N,
                              std::array<bool, kNumLegs>{{true, true, true, true}});
        mpc_in.v_cmd = Vec3d(0.3, 0.0, 0.0);
        const SrbdMpcOutput mpc_out = mpc.Solve(0.0, mpc_in);

        WbcInput in;
        rig.FillBase(&in, Vec18d::Zero());
        in.foot_force_ref = mpc_out.foot_force;
        ComputeBaseAccelDesired(mpc_out, rig.I_com_world, in.qvel.segment<3>(3), in.R_wb, m,
                                g, &in.a_des_world, &in.alpha_des_body);
        WeightedWbc w_on;  // 打开软任务 ①（MPC 逐脚力跟踪）
        WbcParams p_on = w_on.params();
        p_on.w_f_ref = 1.0;
        w_on.set_params(p_on);
        const WbcOutput out = w_on.SolveOnce(in);
        const WbcOutput out_off = wbc.SolveOnce(in);  // w_f_ref = 0 对照
        auto force_ref_dist = [&](const WbcOutput& o) {
            double d = 0.0;
            for (int leg = 0; leg < kNumLegs; ++leg)
                d = std::max(d, (o.foot_force[leg] - mpc_out.foot_force[leg]).norm());
            return d;
        };
        std::printf("  逐脚力与 MPC 参考的最大偏差：w_f_ref=1 → %.4f N，w_f_ref=0 → %.4f N\n",
                    force_ref_dist(out), force_ref_dist(out_off));
        Check(force_ref_dist(out) < force_ref_dist(out_off),
              "打开软任务 ① 后逐脚力更贴近 MPC 参考（任务生效）");
        // 力参考任务与基座任务竞争：打开后基座误差明显变大（这是 docs/04 记的已知取舍），
        // 但仍应远小于期望加速度本身。
        Check(w_on.params().w_f_ref > 0.0 &&
                  out.base_accel_error < 0.05 * in.a_des_world.norm() + 0.02,
              "力参考任务未把基座跟踪拉垮（误差 < 5%）",
              Format("(base err=%.2e vs |a_des|=%.2f)", out.base_accel_error,
                     in.a_des_world.norm()));
        Vec3d net_f(0, 0, 0);
        for (int leg = 0; leg < kNumLegs; ++leg) net_f += out.foot_force[leg];
        std::printf("  MPC: v_cmd=0.3 → F_net=(%.3f, %.3f, %.3f) N, M_net=(%.3e, %.3e, %.3e) Nm\n",
                    mpc_out.net_force.x(), mpc_out.net_force.y(), mpc_out.net_force.z(),
                    mpc_out.net_moment.x(), mpc_out.net_moment.y(), mpc_out.net_moment.z());
        std::printf("  换算的 a_des=(%.3f, %.3f, %.3f) m/s², α_des=(%.3e, %.3e, %.3e) rad/s²\n",
                    in.a_des_world.x(), in.a_des_world.y(), in.a_des_world.z(),
                    in.alpha_des_body.x(), in.alpha_des_body.y(), in.alpha_des_body.z());
        std::printf("  WBC: 净力=(%.3f, %.3f, %.3f) N  Σfz=%.3f N  base 加速度误差=%.2e  锥违反=%.1e\n",
                    net_f.x(), net_f.y(), net_f.z(), net_f.z(), out.base_accel_error,
                    out.cone_violation);
        Check(out.fresh(), "WBC 求解成功");
        // 此时 a_des.z ≠ 0（MPC 为了让质心保持高度给了一点竖直加速度），
        // 所以正确的平衡判据是 Σf ≈ m·(g + a_des.z)（= MPC 的 F_net.z），而不是 m·g。
        const double expect_fz = m * (g + in.a_des_world.z());
        Check(std::fabs(net_f.z() - expect_fz) < 0.03 * expect_fz, "串联后竖直力 ≈ m(g+a_des.z)",
              Format("(Σfz=%.3f N，期望 %.3f N，误差 %.2f%%)", net_f.z(), expect_fz,
                     100.0 * std::fabs(net_f.z() - expect_fz) / expect_fz));
        Check(net_f.x() > 0.0, "前向净力方向与 MPC 期望一致",
              Format("(Fx=%.3f N)", net_f.x()));
        Check(out.eom_residual < 1e-9 && out.contact_residual < 1e-8, "硬约束残差达标");
        Check(ConeOk(out.foot_force, in.stance, wbc.params().mu, wbc.params().foot_force_max),
              "摩擦锥满足");
        dump.rows.push_back({in.q, in.qvel, out.qdd, out.foot_force, out.tau, in.R_wb, in.stance});
    }

    // ---------- 7. 求解耗时 ----------
    std::printf("\n[7] 求解耗时（trot 计划前后移动，200 帧）\n");
    {
        WeightedWbc bench;
        double sum = 0.0, max_ms = 0.0;
        int sum_iters = 0, fails = 0;
        const int frames = 200;
        for (int i = 0; i < frames; ++i) {
            const double t = 0.01 * i;
            Vec18d qvel = Vec18d::Zero();
            qvel.segment<12>(6).setConstant(0.5 * std::sin(0.3 * t));
            WbcInput in;
            rig.FillBase(&in, qvel);
            // 对角小跑：每 ~0.3 s 换一对支撑
            const bool phase = std::fmod(t, 0.6) < 0.3;
            in.stance = {{phase, !phase, !phase, phase}};
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.stance[leg]) {
                    in.foot_des_world[leg] =
                        rig.feet[leg] + Vec3d(0.05 * std::sin(2.0 * t), 0.0, 0.05);
                    in.foot_vel_des_world[leg] = Vec3d(0.15 * std::cos(2.0 * t), 0.0, 0.0);
                }
            }
            in.a_des_world = Vec3d(0.4, 0.1 * std::sin(t), 0.0);
            const WbcOutput out = bench.Solve(t, in);
            sum += out.solve_ms;
            max_ms = std::max(max_ms, out.solve_ms);
            sum_iters += out.iters;
            if (!out.fresh()) ++fails;
        }
        std::printf("  %d 帧：mean=%.3f ms  max=%.3f ms  mean iters=%.1f  失败帧=%d\n", frames,
                    sum / frames, max_ms, static_cast<double>(sum_iters) / frames, fails);
        Check(fails == 0, "连续 200 帧全部求解成功");
        Check(sum / frames < 20.0, "单帧均值 < 20 ms（S7 目标 20-50 Hz）",
              Format("(mean=%.3f ms)", sum / frames));
        Check(max_ms < 50.0, "最坏单帧 < 50 ms", Format("(max=%.3f ms)", max_ms));
    }

    // ---------- 8. 失败兜底 ----------
    std::printf("\n[8] 失败兜底\n");
    {
        WeightedWbc fw;
        WbcInput bad;
        bad.stance = {{false, false, false, false}};  // 无支撑：物理上不成立
        const WbcOutput none = fw.Solve(0.0, bad);
        Check(!none.ok(), "从未成功过时报 kFailed");

        WbcInput good;
        rig.FillBase(&good, Vec18d::Zero());
        const WbcOutput ok1 = fw.Solve(1.0, good);
        Check(ok1.fresh(), "有效输入求解成功");
        const WbcOutput stale = fw.Solve(1.002, bad);
        Check(stale.status == WbcOutput::Status::kStale && stale.ok(), "旧解未过期 → kStale 可用");
        Check(std::fabs(stale.tau.norm() - ok1.tau.norm()) < 1e-12, "沿用旧解内容一致");
        const WbcOutput expired = fw.Solve(1.002 + fw.params().max_solution_age + 0.001, bad);
        Check(!expired.ok(), "旧解过期后报 kFailed");
    }

    // ---------- 9. 低层 PD 反解一致性 ----------
    std::printf("\n[9] 低层 PD 反解（tau_ff = τ_wbc − kp(q_des−q) − kd(dq_des−dq)）\n");
    {
        WbcInput in;
        rig.FillBase(&in, Vec18d::Zero());
        in.a_des_world = Vec3d(1.0, 0.0, 0.0);
        const WbcOutput out = wbc.SolveOnce(in);
        // 假想低层 PD 参数（与 S4 位置环同量级）
        const double kp = 100.0, kd = 2.5;
        Vec12d q_des = rig.q;
        q_des.array() += 0.05;
        const Vec12d dq_des = Vec12d::Zero();
        const Vec12d tau_ff = out.tau - kp * (q_des - in.q) - kd * (dq_des - in.qvel.tail<12>());
        // 执行律 τ_applied = kp(q_des−q) + kd(dq_des−dq) + tau_ff 应精确等于 τ_wbc
        const Vec12d tau_applied =
            kp * (q_des - in.q) + kd * (dq_des - in.qvel.tail<12>()) + tau_ff;
        const double err = (tau_applied - out.tau).cwiseAbs().maxCoeff();
        std::printf("  kp=%.0f kd=%.1f, q_des−q=0.05 rad → |tau_ff|max=%.3f Nm，复现误差=%.2e Nm\n", kp,
                    kd, tau_ff.cwiseAbs().maxCoeff(), err);
        Check(err < 1e-12, "反解前馈可精确复现 WBC 力矩",
              Format("(err=%.2e)", err));
        Check(tau_ff.cwiseAbs().maxCoeff() <= wbc.params().tau_max + 1e-9,
              "前馈力矩在限幅内", Format("(|tau_ff|max=%.3f)", tau_ff.cwiseAbs().maxCoeff()));
    }

    // ---------- 10. 导出 MuJoCo 复核用例 ----------
    std::printf("\n[10] 导出 MuJoCo 复核用例\n");
    WriteDump(dump_path, dump);

    std::printf("\nRESULT: %s  (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
