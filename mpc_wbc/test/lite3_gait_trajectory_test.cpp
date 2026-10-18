/**
 * @file lite3_gait_trajectory_test.cpp
 * @brief S3 步态与足端轨迹自检（不需要 MuJoCo）。
 *
 * 覆盖：
 *   1) 零命令停步（四足静立，不持续踏步）；
 *   2) 起步/停步滞回；
 *   3) 六种单轴 + 混合命令下足目标可达（IK 成功）且不越关节限位；
 *   4) 足端位置全程连续（支撑↔摆动切换无跳变）；
 *   5) 支撑相足端机体系速度 = -(v + ω×p0)；
 *   6) 未来支撑表与逐点接触判据一致。
 */
#include "02_gait_trajectory.hpp"

#include <cstdio>
#include <vector>

using namespace mpc_wbc;

namespace {
int g_failures = 0;

void Check(bool ok, const char* name, const char* detail = "") {
    std::printf("  [%s] %s %s\n", ok ? "OK  " : "FAIL", name, detail);
    if (!ok) ++g_failures;
}
}  // namespace

int main() {
    Lite3Model model;
    GaitParams p;
    GaitTrajectory gait(p);
    const Vec12d nominal = gait.nominal_feet();

    // ---- 1) 零命令停步 ----
    {
        gait.Reset(0.0);
        bool all_contact = true, all_at_nominal = true;
        for (double t = 0.0; t < 2.0 * p.period; t += 0.01) {
            const auto st = gait.Update(t, GaitCommand{});
            for (int leg = 0; leg < kNumLegs; ++leg) {
                all_contact &= st[leg].contact;
                all_at_nominal &= (st[leg].foot_pos - nominal.segment<3>(3 * leg)).norm() < 1e-12;
            }
        }
        Check(!gait.stepping() && all_contact && all_at_nominal, "zero-command stand");
    }

    // ---- 2) 起步/停步滞回 ----
    {
        gait.Reset(10.0);
        gait.Update(10.0, GaitCommand{0.0, 0.0, 0.0});
        const bool stopped = !gait.stepping();
        gait.Update(10.0, GaitCommand{0.2, 0.0, 0.0});
        const bool started = gait.stepping();
        gait.Update(10.5, GaitCommand{0.0, 0.0, 0.0});
        const bool stopped_again = !gait.stepping();
        Check(stopped && started && stopped_again, "start/stop hysteresis");
    }

    // ---- 3) 六种单轴 + 混合命令：可达且不越限 ----
    struct Case { const char* name; GaitCommand cmd; };
    const std::vector<Case> cases = {
        {"fwd", {0.20, 0.0, 0.0}},  {"back", {-0.20, 0.0, 0.0}},
        {"left", {0.0, 0.15, 0.0}}, {"right", {0.0, -0.15, 0.0}},
        {"turn+", {0.0, 0.0, 0.30}}, {"turn-", {0.0, 0.0, -0.30}},
        {"mixed", {0.15, -0.10, 0.20}},
    };
    int ik_fail = 0, limit_viol = 0;
    for (const auto& c : cases) {
        gait.Reset(0.0);
        gait.Update(0.0, c.cmd);
        for (double t = 0.0; t < 2.0 * p.period; t += p.period / 400.0) {
            const auto st = gait.Evaluate(t, c.cmd);
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const Vec3d seed = Lite3Model::DefaultStandLegAngles();
                Vec3d q;
                if (!model.InverseKinematics(leg, st[leg].foot_pos, &q, &seed)) {
                    ++ik_fail;
                    continue;
                }
                for (int j = 0; j < 3; ++j)
                    if (q(j) < model.geo.joint_lower(j) - 1e-9 || q(j) > model.geo.joint_upper(j) + 1e-9)
                        ++limit_viol;
            }
        }
    }
    Check(ik_fail == 0, "all foot targets reachable (IK)", ik_fail ? "" : "");
    Check(limit_viol == 0, "IK solutions within joint limits");
    if (ik_fail || limit_viol)
        std::printf("        ik_fail=%d limit_viol=%d\n", ik_fail, limit_viol);

    // ---- 4) 位置连续（支撑↔摆动切换无跳变） ----
    {
        const GaitCommand cmd{0.15, -0.10, 0.20};
        gait.Reset(0.0);
        gait.Update(0.0, cmd);
        double max_jump = 0.0;
        const double dt = 1e-4;
        auto prev = gait.Evaluate(0.0, cmd);
        for (double t = dt; t < 2.0 * p.period; t += dt) {
            const auto cur = gait.Evaluate(t, cmd);
            for (int leg = 0; leg < kNumLegs; ++leg)
                max_jump = std::max(max_jump, (cur[leg].foot_pos - prev[leg].foot_pos).norm());
            prev = cur;
        }
        // 最大足速约 1.3 m/s，dt=1e-4 时单步应 < 2e-4；给 1e-3 余量。
        Check(max_jump < 1e-3, "foot position continuity");
        std::printf("        max per-step jump = %.3e (dt=1e-4)\n", max_jump);
    }

    // ---- 5) 支撑相机体系速度 = -(v + ω×p0) ----
    {
        const GaitCommand cmd{0.20, -0.05, 0.15};  // 含旋转，检验 ω×p0 项
        gait.Reset(0.0);
        gait.Update(0.0, cmd);
        const double eps = 1e-6;
        double max_err = 0.0;
        int checked = 0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            // 取该腿支撑相中段（ξ=0.34）对应的时刻
            double x = 0.34 * p.duty - p.phase_offset[leg];
            x -= std::floor(x);
            const double t_mid = p.period * x;
            const auto st = gait.Evaluate(t_mid, cmd);
            if (!st[leg].contact) continue;
            ++checked;
            const Vec3d v_fd = (gait.Evaluate(t_mid + eps, cmd)[leg].foot_pos -
                                gait.Evaluate(t_mid - eps, cmd)[leg].foot_pos) /
                               (2.0 * eps);
            const Vec3d p0 = nominal.segment<3>(3 * leg);
            const Vec3d expect(-(cmd.vx - cmd.wz * p0.y()), -(cmd.vy + cmd.wz * p0.x()), 0.0);
            max_err = std::max(max_err, (v_fd - expect).cwiseAbs().maxCoeff());
        }
        Check(max_err < 1e-6 && checked > 0, "stance body-frame velocity = -(v + w x p0)");
        std::printf("        max velocity error = %.3e (checked %d legs)\n", max_err, checked);
    }

    // ---- 6) 支撑表与逐点接触一致 ----
    {
        const GaitCommand cmd{0.15, 0.0, 0.15};
        gait.Reset(0.0);
        gait.Update(0.0, cmd);
        const double dt = 0.02;
        const auto sched = gait.ContactSchedule<10>(0.05, dt);
        int mismatch = 0;
        for (int k = 0; k < 10; ++k) {
            const auto st = gait.Evaluate(0.05 + k * dt, cmd);
            for (int leg = 0; leg < kNumLegs; ++leg)
                if (sched[k][leg] != st[leg].contact) ++mismatch;
        }
        Check(mismatch == 0, "contact schedule matches per-leg contact");
    }

    // ---- 7) 曲线端点性质（单元检查） ----
    {
        bool ok = std::fabs(GaitTrajectory::SwingCurve(0.0)) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingCurve(1.0) - 1.0) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingCurveDerivative(0.0)) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingCurveDerivative(1.0)) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingHeightCurve(0.0, 0.06)) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingHeightCurve(1.0, 0.06)) < 1e-12 &&
                  std::fabs(GaitTrajectory::SwingHeightCurve(0.5, 0.06) - 0.06) < 1e-12;
        Check(ok, "swing curve boundary properties");
    }

    // ---- 8) 三段式参数选择与切换时相位连续 ----
    {
        SegmentedGaitParams schedule;
        schedule.low = p;
        schedule.low.period = 0.65;
        schedule.low.duty = 0.70;
        schedule.low.swing_height = 0.09;
        schedule.mid = p;
        schedule.mid.period = 0.50;
        schedule.mid.duty = 0.66;
        schedule.mid.swing_height = 0.08;
        schedule.high = p;
        schedule.high.period = 0.375;
        schedule.high.duty = 0.625;
        schedule.high.swing_height = 0.07;
        schedule.max_vx = 0.50;
        schedule.max_vy = 0.30;
        schedule.max_wz = 0.40;
        for (GaitParams* sp : {&schedule.low, &schedule.mid, &schedule.high}) {
            sp->max_vx = schedule.max_vx;
            sp->max_vy = schedule.max_vy;
            sp->max_wz = schedule.max_wz;
        }

        const auto lo = schedule.Select(GaitCommand{0.15, 0.0, 0.0});
        const auto mi = schedule.Select(GaitCommand{0.275, 0.0, 0.0});
        const auto hi = schedule.Select(GaitCommand{0.425, 0.0, 0.0});
        Check(std::fabs(lo.period - 0.65) < 1e-12 &&
                  std::fabs(mi.period - 0.50) < 1e-12 &&
                  std::fabs(hi.period - 0.375) < 1e-12,
              "segmented params select T by speed");
        Check(schedule.SelectIndex(GaitCommand{0.15, 0.0, 0.0}) == 0 &&
                  schedule.SelectIndex(GaitCommand{0.275, 0.0, 0.0}) == 1 &&
                  schedule.SelectIndex(GaitCommand{0.425, 0.0, 0.0}) == 2,
              "segmented index by speed");

        GaitTrajectory seg_gait(p, &schedule);
        seg_gait.Reset(0.0);
        seg_gait.Update(0.0, GaitCommand{0.15, 0.0, 0.0});
        const double t_switch = 1.30;
        seg_gait.Update(t_switch, GaitCommand{0.15, 0.0, 0.0});
        const double phase_before = seg_gait.GlobalPhase(t_switch);
        seg_gait.Update(t_switch, GaitCommand{0.425, 0.0, 0.0});
        const double phase_after = seg_gait.GlobalPhase(t_switch);
        Check(std::fabs(phase_before - phase_after) < 1e-9,
              "phase continuous when switching T");
        Check(std::fabs(seg_gait.params().period - 0.375) < 1e-12,
              "high segment params active after switch");
    }

    std::printf("RESULT: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? 0 : 1;
}
