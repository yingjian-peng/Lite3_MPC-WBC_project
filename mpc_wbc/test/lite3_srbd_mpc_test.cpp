/**
 * @file lite3_srbd_mpc_test.cpp
 * @brief S6 离线自检：预测时域 SRBD-MPC 的物理正确性（不需要 MuJoCo）。
 *
 * 检查项：
 *   1. 静立悬停：净力 ≈ m·g、逐脚 ≈ m·g/4、净力矩 ≈ 0、预测高度不漂；
 *   2. 速度响应：前/后/横移方向正确、随命令单调、预测末端速度跟随、刹车反号；
 *   3. 偏航响应：净力矩 z 方向正确、预测偏航角/角速度跟随；
 *   4. 接触计划与约束：逐脚 f_z ≥ 0、|f_x|,|f_y| ≤ μ f_z、f_z ≤ f_z,max；摆动脚受力恒为 0；
 *      两对角腿支撑时仍能承重；
 *   5. 姿态通道：roll/pitch 扰动产生反向恢复力矩且预测姿态收敛；角速度阻尼力矩反号；
 *   6. 求解健康：QP 残差收敛、耗时记录、热启动有效、连续 100 帧全部成功；
 *   7. 失败兜底：无解时先沿用未过期旧解(kStale)，超时或从未解出则报 kFailed；
 *   8. 闭环：用同一 SRBD 模型当被控对象，重规划 200 步（0 → 0.3 m/s），速度收敛、高度不漂。
 *
 * 用法： ./lite3_srbd_mpc_test
 * 返回 0 表示通过。
 */
#include "03_srbd_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
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

/// 名义站立构型：四脚支撑，球足最低点刚好落地。
struct StandRig {
    Lite3Model model;
    Vec12d q = Lite3Model::DefaultStandJointAngles();
    Mat3d R_B = Mat3d::Identity();
    Vec3d p_base = Vec3d::Zero();
    Vec3d p_com = Vec3d::Zero();
    Mat3d I_com_world = Mat3d::Zero();
    std::array<Vec3d, kNumLegs> feet{};

    StandRig() {
        double min_z = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg)
            min_z = std::min(min_z, model.FootPositionBody(leg, q.segment<3>(3 * leg)).z());
        p_base = Vec3d(0.0, 0.0, -min_z + model.geo.foot_radius);
        p_com = model.CompositeComWorld(q, R_B, p_base);
        I_com_world = model.CompositeInertiaWorld(q, R_B, model.CompositeComBody(q));
        for (int leg = 0; leg < kNumLegs; ++leg)
            feet[leg] = p_base + R_B * model.FootPositionBody(leg, q.segment<3>(3 * leg));
    }
};

std::vector<std::array<bool, kNumLegs>> AllStance(int n) {
    return std::vector<std::array<bool, kNumLegs>>(
        n, std::array<bool, kNumLegs>{{true, true, true, true}});
}

/// 对角小跑接触计划：FL/HR 与 FR/HL 各占相位一半，duty 决定重叠时长。
/// t0 为计划起点时间（k 节点取 t0 + k·dt）。
std::vector<std::array<bool, kNumLegs>> TrotScheduleFrom(double t0, int n, double dt,
                                                         double duty = 0.6, double period = 0.6) {
    std::vector<std::array<bool, kNumLegs>> out(n);
    for (int k = 0; k < n; ++k) {
        const double t = t0 + dt * static_cast<double>(k);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double offset = (leg == kFL || leg == kHR) ? 0.0 : 0.5;
            double phase = std::fmod(t / period + offset, 1.0);
            if (phase < 0.0) phase += 1.0;
            out[k][leg] = phase < duty;
        }
    }
    return out;
}

SrbdMpcInput MakeInput(const StandRig& rig, const std::vector<std::array<bool, kNumLegs>>& plan) {
    SrbdMpcInput in;
    in.p_com = rig.p_com;
    in.v_com = Vec3d::Zero();
    in.rpy = Vec3d::Zero();
    in.omega = Vec3d::Zero();
    in.inertia_com_world = rig.I_com_world;
    in.foot_pos_world = rig.feet;
    in.height_cmd = rig.p_com.z();
    in.contact = plan;
    return in;
}

/// 逐脚摩擦锥/法向力约束检查。
bool ConeOk(const std::array<Vec3d, kNumLegs>& f, double mu, double fz_max, double tol = 1e-9) {
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (f[leg].z() < -tol) return false;
        if (std::fabs(f[leg].x()) > mu * f[leg].z() + tol) return false;
        if (std::fabs(f[leg].y()) > mu * f[leg].z() + tol) return false;
        if (f[leg].z() > fz_max + tol) return false;
    }
    return true;
}

double TotalZ(const std::array<Vec3d, kNumLegs>& f) {
    double s = 0.0;
    for (const Vec3d& v : f) s += v.z();
    return s;
}

}  // namespace

int main() {
    StandRig rig;
    const double m = rig.model.TotalMass();
    const double g = 9.81;
    const double weight = m * g;
    SrbdPreviewMpc mpc;
    std::printf("S6 SRBD preview-MPC offline self-check\n");
    std::printf("  mass=%.4f kg  weight=%.2f N  p_com=(%.4f, %.4f, %.4f)\n", m, weight, rig.p_com.x(),
                rig.p_com.y(), rig.p_com.z());
    std::printf("  I_com diag (世界系) = (%.5f, %.5f, %.5f) kg·m²\n", rig.I_com_world(0, 0),
                rig.I_com_world(1, 1), rig.I_com_world(2, 2));
    std::printf("  N=%d  dt=%.3f s  时域=%.2f s  mu=%.2f  fz_max=%.0f N\n\n", mpc.params().N,
                mpc.params().dt, mpc.params().N * mpc.params().dt, mpc.params().mu,
                mpc.params().foot_force_max);

    // ---------- 1. 静立悬停 ----------
    std::printf("[1] 静立悬停（四脚全程支撑）\n");
    {
        SrbdMpcInput in = MakeInput(rig, AllStance(mpc.params().N));
        const SrbdMpcOutput out = mpc.Solve(0.0, in);
        const double fz_err = std::fabs(out.net_force.z() - weight);
        double max_xy = 0.0;
        double fz_min = 1e9, fz_max = -1e9;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            max_xy = std::max(max_xy, std::hypot(out.foot_force[leg].x(), out.foot_force[leg].y()));
            fz_min = std::min(fz_min, out.foot_force[leg].z());
            fz_max = std::max(fz_max, out.foot_force[leg].z());
        }
        double max_z_drift = 0.0;
        for (const Vec3d& p : out.pred_p)
            max_z_drift = std::max(max_z_drift, std::fabs(p.z() - in.height_cmd));
        std::printf("  net_F=(%.4f, %.4f, %.4f) N  net_M=(%.2e, %.2e, %.2e) Nm\n", out.net_force.x(),
                    out.net_force.y(), out.net_force.z(), out.net_moment.x(), out.net_moment.y(),
                    out.net_moment.z());
        std::printf("  |Fz−mg|=%.5f N (%.4f%%)  逐脚 fz∈[%.3f, %.3f] N (mg/4=%.3f)  max|f_xy|=%.4f N\n",
                    fz_err, 100.0 * fz_err / weight, fz_min, fz_max, weight / 4.0, max_xy);
        std::printf("  预测高度漂移=%.2e m  nvar=%d  iters=%d  residual=%.2e  solve=%.3f ms\n\n",
                    max_z_drift, out.nvar, out.iters, out.qp_residual, out.solve_ms);
        Check(out.fresh(), "本帧求解成功");
        Check(fz_err < 0.01 * weight, "净竖直力 ≈ m·g", Format("(%.4f%%)", 100.0 * fz_err / weight));
        // 四点支撑是静不定问题，负载分配由代价决定；这里只要求「每脚都在 mg/4 量级」，
        // 真正的平衡条件由 Σfz = mg 与净力矩 ≈ 0 两条检查承担。
        Check(fz_min > 0.5 * weight / 4.0 && fz_max < 1.5 * weight / 4.0, "逐脚竖直力在 mg/4 量级",
              Format("([%.3f, %.3f] N)", fz_min, fz_max));
        Check(max_xy < 1.0, "静立时水平足力接近 0", Format("(max=%.4f N)", max_xy));
        Check(out.net_moment.norm() < 0.1, "静立净力矩 ≈ 0",
              Format("(|M|=%.2e Nm)", out.net_moment.norm()));
        Check(max_z_drift < 1e-4, "预测高度不漂", Format("(drift=%.2e m)", max_z_drift));
        Check(ConeOk(out.foot_force, mpc.params().mu, mpc.params().foot_force_max),
              "摩擦锥/法向力约束满足");
        Check(out.qp_residual < 1e-8, "QP 残差收敛", Format("(res=%.2e)", out.qp_residual));
    }

    // ---------- 2. 速度响应 ----------
    std::printf("[2] 速度响应（前/后/横移）\n");
    {
        double prev_force = -1e9;
        bool monotone = true;
        for (double v : {0.1, 0.2, 0.3}) {
            SrbdMpcInput in = MakeInput(rig, AllStance(mpc.params().N));
            in.v_cmd = Vec3d(v, 0.0, 0.0);
            const SrbdMpcOutput out = mpc.Solve(v, in);
            const double terminal_vx = out.pred_v.back().x();
            std::printf("  v_cmd=%.2f → F0.x=%+8.3f N  F0.z=%8.3f N  pred_vx(末端)=%.4f m/s  nvar=%d\n", v,
                        out.net_force.x(), out.net_force.z(), terminal_vx, out.nvar);
            Check(out.net_force.x() > 0.0, "前进净力为正");
            Check(std::fabs(terminal_vx - v) < 0.1 * v + 0.02, "预测末端 vx 跟随命令",
                  Format("(err=%+.4f m/s)", terminal_vx - v));
            Check(std::fabs(out.net_force.z() - weight) < 0.15 * weight, "加速时仍承重",
                  Format("(Fz=%.2f N)", out.net_force.z()));
            monotone = monotone && out.net_force.x() > prev_force;
            prev_force = out.net_force.x();
        }
        Check(monotone, "净力随速度命令单调增大");

        SrbdMpcInput brake = MakeInput(rig, AllStance(mpc.params().N));
        brake.v_com = Vec3d(0.3, 0.0, 0.0);
        brake.v_cmd = Vec3d::Zero();
        const SrbdMpcOutput bout = mpc.Solve(9.0, brake);
        std::printf("  刹车 v0=0.30 → F0.x=%+8.3f N  pred_vx(末端)=%.4f m/s\n", bout.net_force.x(),
                    bout.pred_v.back().x());
        Check(bout.net_force.x() < 0.0, "刹车净力为负");
        Check(std::fabs(bout.pred_v.back().x()) < 0.06, "预测末端速度接近 0",
              Format("(vx=%.4f m/s)", bout.pred_v.back().x()));

        for (double vy : {0.2, -0.2}) {
            SrbdMpcInput in = MakeInput(rig, AllStance(mpc.params().N));
            in.v_cmd = Vec3d(0.0, vy, 0.0);
            const SrbdMpcOutput out = mpc.Solve(20.0, in);
            std::printf("  v_cmd.y=%+.2f → F0.y=%+8.3f N  pred_vy(末端)=%.4f m/s\n", vy,
                        out.net_force.y(), out.pred_v.back().y());
            Check(out.net_force.y() * vy > 0.0, "横移净力方向与命令一致");
            Check(std::fabs(out.pred_v.back().y() - vy) < 0.1 * std::fabs(vy) + 0.02,
                  "预测末端 vy 跟随命令", Format("(err=%+.4f m/s)", out.pred_v.back().y() - vy));
        }
        std::printf("\n");
    }

    // ---------- 3. 偏航响应 ----------
    std::printf("[3] 偏航响应\n");
    {
        SrbdMpcInput in = MakeInput(rig, AllStance(mpc.params().N));
        in.yaw_rate_cmd = 0.6;
        const SrbdMpcOutput out = mpc.Solve(30.0, in);
        std::printf("  yaw_rate_cmd=+0.6 → M0.z=%+8.4f Nm  pred_yaw(末端)=%+.4f rad  pred_wz=%+.4f rad/s\n",
                    out.net_moment.z(), out.pred_rpy.back().z(), out.pred_omega.back().z());
        std::printf("  逐脚 fz=(%.2f, %.2f, %.2f, %.2f) N\n", out.foot_force[0].z(),
                    out.foot_force[1].z(), out.foot_force[2].z(), out.foot_force[3].z());
        Check(out.net_moment.z() > 0.0, "偏航力矩方向为正");
        Check(out.pred_rpy.back().z() > 0.2, "预测偏航角增加",
              Format("(yaw=%.4f rad)", out.pred_rpy.back().z()));
        Check(ConeOk(out.foot_force, mpc.params().mu, mpc.params().foot_force_max),
              "摩擦锥约束满足");

        SrbdMpcInput neg = in;
        neg.yaw_rate_cmd = -0.6;
        const SrbdMpcOutput nout = mpc.Solve(31.0, neg);
        std::printf("  yaw_rate_cmd=-0.6 → M0.z=%+8.4f Nm\n\n", nout.net_moment.z());
        Check(nout.net_moment.z() < 0.0, "反向偏航力矩反号");
    }

    // ---------- 4. 接触计划与约束 ----------
    std::printf("[4] 接触计划（trot）与约束\n");
    {
        const auto plan = TrotScheduleFrom(0.42, mpc.params().N, mpc.params().dt);  // k=0 时 FL/HR 摆动
        int expected_vars = 0;
        for (const auto& row : plan)
            for (bool c : row)
                if (c) expected_vars += kConeGenerators;

        SrbdMpcInput in = MakeInput(rig, plan);
        const SrbdMpcOutput out = mpc.Solve(40.0, in);
        std::printf("  k=0 支撑: FL=%d FR=%d HL=%d HR=%d\n", plan[0][0], plan[0][1], plan[0][2],
                    plan[0][3]);
        for (int leg = 0; leg < kNumLegs; ++leg)
            std::printf("   leg%d f=(%+7.3f, %+7.3f, %8.3f) N\n", leg, out.foot_force[leg].x(),
                        out.foot_force[leg].y(), out.foot_force[leg].z());
        bool swing_zero = true;
        for (int leg = 0; leg < kNumLegs; ++leg)
            if (!plan[0][leg] && out.foot_force[leg].norm() > 1e-12) swing_zero = false;
        Check(swing_zero, "摆动脚（k=0）受力恒为 0");
        Check(ConeOk(out.foot_force, mpc.params().mu, mpc.params().foot_force_max),
              "摩擦锥/法向力约束满足");
        Check(TotalZ(out.foot_force) > 0.9 * weight, "支撑脚承担机重",
              Format("(Σfz=%.2f N / mg=%.2f N)", TotalZ(out.foot_force), weight));
        Check(out.nvar == expected_vars, "决策变量数 = 支撑脚×4方向×N",
              Format("(nvar=%d, 期望=%d)", out.nvar, expected_vars));

        auto two_leg = AllStance(mpc.params().N);
        for (auto& row : two_leg) {
            row[kFR] = false;
            row[kHL] = false;
        }
        SrbdMpcInput t_in = MakeInput(rig, two_leg);
        const SrbdMpcOutput t_out = mpc.Solve(41.0, t_in);
        const double two_z = t_out.foot_force[kFL].z() + t_out.foot_force[kHR].z();
        std::printf("  仅 FL/HR 支撑 → fz=(%.2f, %.2f) N（合计 %.2f N）\n\n", t_out.foot_force[kFL].z(),
                    t_out.foot_force[kHR].z(), two_z);
        Check(t_out.foot_force[kFR].norm() < 1e-12 && t_out.foot_force[kHL].norm() < 1e-12,
              "非支撑腿力为 0");
        Check(std::fabs(two_z - weight) < 0.05 * weight, "两支撑腿合计承重",
              Format("(Σfz=%.2f N)", two_z));
        Check(ConeOk(t_out.foot_force, mpc.params().mu, mpc.params().foot_force_max),
              "两脚支撑时摩擦锥满足");
    }

    // ---------- 5. 姿态通道 ----------
    std::printf("[5] 姿态通道（恢复力矩与阻尼）\n");
    {
        SrbdMpcInput in = MakeInput(rig, AllStance(mpc.params().N));
        in.rpy = Vec3d(0.10, 0.0, 0.0);
        const SrbdMpcOutput out = mpc.Solve(50.0, in);
        std::printf("  roll=+0.10 → M0.x=%+8.4f Nm  pred_roll(末端)=%+.4f rad\n", out.net_moment.x(),
                    out.pred_rpy.back().x());
        Check(out.net_moment.x() < 0.0, "roll 扰动产生反向恢复力矩");
        Check(std::fabs(out.pred_rpy.back().x()) < 0.05, "预测 roll 被拉回",
              Format("(末端=%+.4f rad)", out.pred_rpy.back().x()));

        SrbdMpcInput pitch = MakeInput(rig, AllStance(mpc.params().N));
        pitch.rpy = Vec3d(0.0, 0.10, 0.0);
        const SrbdMpcOutput pout = mpc.Solve(51.0, pitch);
        std::printf("  pitch=+0.10 → M0.y=%+8.4f Nm  pred_pitch(末端)=%+.4f rad\n", pout.net_moment.y(),
                    pout.pred_rpy.back().y());
        Check(pout.net_moment.y() < 0.0, "pitch 扰动产生反向恢复力矩");
        Check(std::fabs(pout.pred_rpy.back().y()) < 0.05, "预测 pitch 被拉回",
              Format("(末端=%+.4f rad)", pout.pred_rpy.back().y()));

        SrbdMpcInput spin = MakeInput(rig, AllStance(mpc.params().N));
        spin.omega = Vec3d(0.0, 0.5, 0.0);
        const SrbdMpcOutput sout = mpc.Solve(52.0, spin);
        std::printf("  omega.y=+0.5 → M0.y=%+8.4f Nm  pred_wy(末端)=%+.4f rad/s\n", sout.net_moment.y(),
                    sout.pred_omega.back().y());
        Check(sout.net_moment.y() < 0.0, "角速度阻尼力矩反号");
        Check(sout.pred_omega.back().y() < 0.5, "预测角速度被阻尼",
              Format("(末端=%+.4f rad/s)", sout.pred_omega.back().y()));

        SrbdMpcInput level = MakeInput(rig, AllStance(mpc.params().N));
        const SrbdMpcOutput lout = mpc.Solve(53.0, level);
        Check(lout.net_moment.norm() < 0.5, "水平静止时无多余姿态力矩",
              Format("(|M|=%.2e Nm)", lout.net_moment.norm()));
    }

    // ---------- 6. 求解健康与热启动 ----------
    std::printf("\n[6] 求解健康 / 热启动\n");
    {
        SrbdPreviewMpc smpc;
        SrbdMpcInput in = MakeInput(rig, TrotScheduleFrom(0.42, smpc.params().N, smpc.params().dt));
        in.v_cmd = Vec3d(0.3, 0.0, 0.0);
        const SrbdMpcOutput first = smpc.Solve(0.0, in);
        // 20 Hz 重规划：接触计划随时域前移一格、命令微变 → 热启动应接近最优
        in.contact = TrotScheduleFrom(0.42 + smpc.params().dt, smpc.params().N, smpc.params().dt);
        in.v_cmd = Vec3d(0.31, 0.0, 0.0);
        const SrbdMpcOutput second = smpc.Solve(smpc.params().dt, in);
        std::printf("  冷启动 iters=%d residual=%.2e solve=%.3f ms\n", first.iters, first.qp_residual,
                    first.solve_ms);
        std::printf("  热启动 iters=%d residual=%.2e solve=%.3f ms\n", second.iters,
                    second.qp_residual, second.solve_ms);
        Check(first.qp_residual < 1e-8 && second.qp_residual < 1e-8, "QP 均收敛");
        Check(second.iters <= first.iters + 2, "热启动迭代次数不增加",
              Format("(%d → %d)", first.iters, second.iters));
        Check(first.solve_ms < 50.0 && second.solve_ms < 50.0, "单帧求解耗时 < 50 ms",
              Format("(%.3f / %.3f ms)", first.solve_ms, second.solve_ms));

        SrbdPreviewMpc bench;
        double max_ms = 0.0, sum_ms = 0.0;
        int sum_iters = 0;
        bool all_fresh = true;
        for (int i = 0; i < 100; ++i) {
            const double t = 0.05 * i;
            SrbdMpcInput bi = MakeInput(rig, TrotScheduleFrom(t, bench.params().N, bench.params().dt));
            bi.v_cmd = Vec3d(0.3, 0.05 * std::sin(0.1 * t), 0.0);
            const SrbdMpcOutput bo = bench.Solve(t, bi);
            max_ms = std::max(max_ms, bo.solve_ms);
            sum_ms += bo.solve_ms;
            sum_iters += bo.iters;
            all_fresh = all_fresh && bo.fresh();
        }
        std::printf("  连续 100 帧（trot 计划随时域前移）：mean=%.3f ms  max=%.3f ms  mean iters=%.2f\n",
                    sum_ms / 100.0, max_ms, sum_iters / 100.0);
        Check(all_fresh, "连续 100 帧全部求解成功");
        Check(sum_ms / 100.0 < 20.0, "满足 20 Hz 求解预算（50 ms）",
              Format("(mean=%.3f ms)", sum_ms / 100.0));
    }

    // ---------- 7. 失败兜底 ----------
    std::printf("\n[7] 失败兜底（沿用旧解 / 报失败）\n");
    {
        SrbdPreviewMpc fmpc;
        SrbdMpcInput bad = MakeInput(rig, AllStance(fmpc.params().N));
        bad.inertia_com_world = Mat3d::Zero();  // 不可逆 → 无效输入
        Check(!fmpc.Solve(0.0, bad).ok(), "从未成功过时无效输入报 kFailed");

        SrbdMpcInput good = MakeInput(rig, AllStance(fmpc.params().N));
        const SrbdMpcOutput ok1 = fmpc.Solve(1.00, good);
        Check(ok1.fresh(), "有效输入求解成功");

        const SrbdMpcOutput stale = fmpc.Solve(1.05, bad);
        std::printf("  失败帧 status=%s  Fz=%.3f N（原解 %.3f N）\n",
                    stale.status == SrbdMpcOutput::Status::kStale ? "kStale" : "其它",
                    stale.net_force.z(), ok1.net_force.z());
        Check(stale.status == SrbdMpcOutput::Status::kStale, "旧解未过期 → kStale 且 ok() 可用");
        Check(stale.ok() && std::fabs(stale.net_force.z() - ok1.net_force.z()) < 1e-9,
              "沿用旧解内容一致");

        const SrbdMpcOutput expired = fmpc.Solve(1.05 + fmpc.params().max_solution_age + 0.01, bad);
        Check(!expired.ok(), "旧解过期后报 kFailed");

        SrbdMpcInput empty;
        empty.inertia_com_world = Mat3d::Identity();
        Check(!fmpc.Solve(2.0, empty).ok(), "空接触计划报失败");
    }

    // ---------- 8. 闭环（SRBD 模型当被控对象） ----------
    std::printf("\n[8] 闭环跟踪（SRBD 模型 + 每帧重规划）\n");
    {
        SrbdPreviewMpc cmp;
        const double dt = cmp.params().dt;
        Vec3d p = rig.p_com, v = Vec3d::Zero();
        Vec3d rpy = Vec3d::Zero(), omega = Vec3d::Zero();
        std::array<Vec3d, kNumLegs> feet_rel{};
        for (int leg = 0; leg < kNumLegs; ++leg) feet_rel[leg] = rig.feet[leg] - rig.p_com;

        double max_z_err = 0.0, max_roll = 0.0;
        bool fresh_ok = true;
        const int steps = 200;
        for (int i = 0; i < steps; ++i) {
            SrbdMpcInput in = MakeInput(rig, AllStance(cmp.params().N));
            in.p_com = p;
            in.v_com = v;
            in.rpy = rpy;
            in.omega = omega;
            in.v_cmd = Vec3d(0.3, 0.0, 0.0);
            in.height_cmd = rig.p_com.z();
            for (int leg = 0; leg < kNumLegs; ++leg) in.foot_pos_world[leg] = p + feet_rel[leg];

            const SrbdMpcOutput out = cmp.Solve(dt * i, in);
            fresh_ok = fresh_ok && out.fresh();
            if (!fresh_ok) break;

            const Mat3d R = RpyToRotation(rpy);
            const Vec3d I_om = in.inertia_com_world * omega;
            const Vec3d alpha = in.inertia_com_world.inverse() *
                                (out.net_moment - R.transpose() * omega.cross(I_om));
            const Vec3d a = Vec3d(0.0, 0.0, -g) + out.net_force / m;
            p = p + dt * v + 0.5 * dt * dt * a;
            v = v + dt * a;
            rpy = rpy + dt * (SrbdPreviewMpc::EulerRateMatrix(rpy) * omega);
            omega = omega + dt * alpha;

            if (i > 20) {
                max_z_err = std::max(max_z_err, std::fabs(p.z() - rig.p_com.z()));
                max_roll = std::max(max_roll, std::fabs(rpy.x()));
            }
        }
        std::printf("  %d 步后 vx=%.4f m/s（命令 0.3）  z 误差 max=%.4f m  roll max=%.5f rad\n", steps,
                    v.x(), max_z_err, max_roll);
        Check(fresh_ok, "闭环每帧求解成功");
        Check(std::fabs(v.x() - 0.3) < 0.02, "闭环速度跟随命令", Format("(vx=%.4f m/s)", v.x()));
        Check(max_z_err < 0.03, "闭环高度不漂", Format("(max=%.4f m)", max_z_err));
        Check(max_roll < 0.05, "闭环姿态保持水平", Format("(max=%.4f rad)", max_roll));
    }

    std::printf("\nRESULT: %s  (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
