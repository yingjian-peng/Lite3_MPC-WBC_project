/**
 * @file position_loop_sim.cpp
 * @brief S4 位置环跑步基线（与 MuJoCo 仿真通过 UDP 通信）。
 *
 * 控制律： q_des = IK(步态足端目标)；下发 {kp, q_des, kd, 0, 0}，
 *          仿真侧执行 tau = kp(q_des-q) + kd(0-dq)。
 * 时间线： 静落 -> 站立斜坡 -> 保持 -> 前进/后退/侧移/转向（各段间停步）。
 * 安全：   姿态 roll/pitch 超限则停止并返回非零。
 *
 * 用法： ./mpc_wbc_position_loop_sim [phases_csv]
 *        （需先启动 interface/robot/simulation/mujoco_simulation.py）
 */
#include "simulation/simulation_interface.hpp"

#include "00_lite3_model.hpp"
#include "02_gait_trajectory.hpp"
#include "scenario_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <cstdio>
#include <fstream>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace types;
using namespace interface;
using namespace mpc_wbc;

namespace {

struct Phase {
    std::string name;
    double duration;
    GaitCommand cmd;
};

double SmoothStep(double x) {
    x = std::max(0.0, std::min(1.0, x));
    return x * x * (3.0 - 2.0 * x);
}

/// 以最大变化量向目标靠近（docs/03 §7.1 的 `Approach`，参考实现同名）。
double Approach(double current, double target, double max_delta) {
    const double d = target - current;
    if (std::fabs(d) <= max_delta) return target;
    return current + (d >= 0.0 ? max_delta : -max_delta);
}

/// 步幅限幅：Δp_leg = t_stance·(v_cmd + ω×p0_xy) 对各轴线性，故按「各腿各轴超出比例的最大值」
/// 统一缩放命令即可（与参考实现「步长限幅」等价）。
GaitCommand LimitStep(const GaitCommand& in, double t_stance, const Vec12d& nominal, double step_x,
                      double step_y) {
    double s = 1.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3d p0 = nominal.segment<3>(3 * leg);
        const double px = p0.x(), py = p0.y();
        const double dpx = t_stance * (in.vx - in.wz * py);
        const double dpy = t_stance * (in.vy + in.wz * px);
        if (step_x > 0.0 && std::fabs(dpx) > step_x) s = std::min(s, step_x / std::fabs(dpx));
        if (step_y > 0.0 && std::fabs(dpy) > step_y) s = std::min(s, step_y / std::fabs(dpy));
    }
    GaitCommand out;
    out.vx = in.vx * s;
    out.vy = in.vy * s;
    out.wz = in.wz * s;
    return out;
}

/// 起立段固定为 settle/stand/hold（与 S8 harness 的 settle/fold/raise/hold 等价分工），
/// 之后接运动段：默认内置六方向序列，或 S9_SCENARIO 指定的场景文件（与 MPC 侧同一份）。
std::vector<Phase> BuildPhases() {
    std::vector<Phase> p = {
        {"settle", 0.8, {0.0, 0.0, 0.0}},
        {"stand", 1.5, {0.0, 0.0, 0.0}},
        {"hold", 1.0, {0.0, 0.0, 0.0}},
    };
    const char* scenario = std::getenv("S9_SCENARIO");
    if (scenario != nullptr && scenario_io::LooksLikePath(scenario)) {
        std::vector<scenario_io::Item> items;
        std::string err;
        if (!scenario_io::Load(scenario, &items, &err)) {
            std::fprintf(stderr, "[s4] %s\n", err.c_str());
            std::exit(2);
        }
        p[2].duration = 0.5;
        for (const auto& it : items) p.push_back({it.name, it.duration, {it.vx, it.vy, it.wz}});
        std::printf("[s4] scenario %s: %zu motion phases\n", scenario, items.size());
        return p;
    }
    const std::vector<Phase> builtin = {
        {"walk_fwd", 3.0, {0.15, 0.0, 0.0}},
        {"stop1", 0.8, {0.0, 0.0, 0.0}},
        {"walk_back", 3.0, {-0.15, 0.0, 0.0}},
        {"stop2", 0.8, {0.0, 0.0, 0.0}},
        {"walk_left", 3.0, {0.0, 0.12, 0.0}},
        {"stop3", 0.8, {0.0, 0.0, 0.0}},
        {"turn", 3.0, {0.0, 0.0, 0.3}},
        {"stop4", 0.8, {0.0, 0.0, 0.0}},
    };
    p.insert(p.end(), builtin.begin(), builtin.end());
    return p;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string phases_csv = (argc > 1) ? argv[1] : "s4_phases.csv";
    // kp/kd：docs/03 附录 C 给的是 100/2.5；实测（仅足端接触模型、抬脚 0.10 m）把 kp 提到 200
    // 让稳态速度从命令的 0.65 倍升到 0.95 倍（峰值力矩 18.4 N·m，限幅 30 N·m 有余量）。
    // 再往上（kp300）会打到限幅、滑移回升、位移反而掉到 0.75 倍 —— 200 是拐点。
    double kp = 200.0, kd = 2.5;
    // 开环调参旋钮（docs/04 §7）：步态形状与 PD 增益、支撑相竖直预压。
    // 开环能走多快基本由「支撑脚是否真的钉住 + 摆动脚是否真的离地」决定，
    // 所以这几个量要能单独扫。
    GaitParams gait_params;
    double stance_press = 0.0;  // 支撑相把足端 z 额外下压 [m]（软接触下提高接地可靠性）
    double cmd_scale = 1.0;     // 命令缩放（用于摸「开环稳定速度区间」）

    // ---- docs/03 §7 那套 M0/M1 工程细节（默认全关，保持当前基线；给了值就按参考实现跑）----
    // 参考值：死区 0.06、命令斜坡 2.5 /s、起步足端混合 1.0 s、关节目标速率限幅 6 rad/s、
    // 步幅限幅 stepX/stepY = 0.10/0.035 m、M1 增益 0.5/0.5/0.5 + 一阶低通 0.05 s + 单脚限幅 0.04 m。
    double deadzone = 0.0;      // S4_DEADZONE
    double cmd_rate = 0.0;      // S4_RAMP
    double blend_win = 0.0;     // S4_BLEND
    double jv_rate = 0.0;       // S4_JV_RATE
    double step_x = 0.0;        // S4_STEP_X
    double step_y = 0.0;        // S4_STEP_Y
    // 扫掠增益 λ（对应参考实现里"仿真传动效率<1 用 λ 慢积分标定"那一层）：
    // λ 只放大**送进步态的**命令（=放大规划步幅），phases.csv 仍记原始意图，
    // 所以分析口径依然是"实际速度 / 原始命令"，不会自我欺骗。
    double sweep_gain = 1.0;    // S4_SWEEP_GAIN
    bool m1_on = false;         // S4_M1
    double m1_gh = 0.5, m1_gr = 0.5, m1_gp = 0.5;
    // 符号默认按**本工程约定**标定（y 向左、roll>0 = 左侧抬起、pitch>0 = 机头向下）：
    // 参考实现（docs/03 §7.3）用的是 SIGN_ROLL=-1，那是给他们那台机器的约定；照搬会把 roll
    // 波动放大（roll_sd 0.64°→0.95°、尖峰 3.4°→4.4°），见 docs/04 §8「S9 第五项」。
    double m1_tau = 0.05, m1_dz = 0.04, m1_sr = 1.0, m1_sp = 1.0;
    if (const char* e = std::getenv("S4_KP")) kp = std::atof(e);
    if (const char* e = std::getenv("S4_KD")) kd = std::atof(e);
    if (const char* e = std::getenv("S4_SWING_HEIGHT")) gait_params.swing_height = std::atof(e);
    if (const char* e = std::getenv("S4_DUTY")) gait_params.duty = std::atof(e);
    if (const char* e = std::getenv("S4_PERIOD")) gait_params.period = std::atof(e);
    if (const char* e = std::getenv("S4_STANCE_PRESS")) stance_press = std::atof(e);
    if (const char* e = std::getenv("S4_CMD_SCALE")) cmd_scale = std::atof(e);
    if (const char* e = std::getenv("S4_DEADZONE")) deadzone = std::atof(e);
    if (const char* e = std::getenv("S4_RAMP")) cmd_rate = std::atof(e);
    if (const char* e = std::getenv("S4_BLEND")) blend_win = std::atof(e);
    if (const char* e = std::getenv("S4_JV_RATE")) jv_rate = std::atof(e);
    if (const char* e = std::getenv("S4_STEP_X")) step_x = std::atof(e);
    if (const char* e = std::getenv("S4_STEP_Y")) step_y = std::atof(e);
    if (const char* e = std::getenv("S4_SWEEP_GAIN")) sweep_gain = std::atof(e);
    if (const char* e = std::getenv("S4_M1")) m1_on = std::atoi(e) != 0;
    if (const char* e = std::getenv("S4_M1_GAIN_H")) m1_gh = std::atof(e);
    if (const char* e = std::getenv("S4_M1_GAIN_ROLL")) m1_gr = std::atof(e);
    if (const char* e = std::getenv("S4_M1_GAIN_PITCH")) m1_gp = std::atof(e);
    if (const char* e = std::getenv("S4_M1_TAU")) m1_tau = std::atof(e);
    if (const char* e = std::getenv("S4_M1_DZ_LIM")) m1_dz = std::atof(e);
    if (const char* e = std::getenv("S4_M1_SIGN_ROLL")) m1_sr = std::atof(e);
    if (const char* e = std::getenv("S4_M1_SIGN_PITCH")) m1_sp = std::atof(e);
    // 起始方式：standing（默认）= 仿真从站立平衡位开始，本程序从第一帧就保持站立构型；
    // drop = 旧的「零力矩落地 + 关节空间起立斜坡」，需配合 SIM_INIT_POSE=drop 使用。
    const bool start_standing =
        std::string(std::getenv("S4_START") ? std::getenv("S4_START") : "standing") != "drop";

    auto ri = std::make_shared<SimulationInterface>("Lite3", 12);
    ri->Start();

    // 等待第一帧仿真数据
    const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (ri->GetInterfaceTimeStamp() <= 0.0) {
        if (std::chrono::steady_clock::now() > wait_deadline) {
            std::fprintf(stderr, "[s4] no simulation data received\n");
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    Lite3Model model;
    GaitTrajectory gait(gait_params);
    const Vec12d stand = Lite3Model::DefaultStandJointAngles();

    std::vector<Phase> phases = BuildPhases();
    if (start_standing) {
        // 名字只是为了日志/CSV 自解释，分析器按行号跳过前缀，不依赖名字
        phases[0].name = "hold_stand_a";
        phases[1].name = "hold_stand_b";
    }
    if (cmd_scale != 1.0) {
        for (auto& p : phases) { p.cmd.vx *= cmd_scale; p.cmd.vy *= cmd_scale; p.cmd.wz *= cmd_scale; }
        std::printf("[s4] command scaled by %.3f\n", cmd_scale);
    }
    double total_duration = 0.0;
    for (const auto& p : phases) total_duration += p.duration;

    const double t0 = ri->GetInterfaceTimeStamp();

    std::ofstream pcsv(phases_csv);
    pcsv << "phase,t_start,t_end,vx,vy,wz\n";
    pcsv.flush();

    // 逐控制周期轨迹（env S4_TRACE=<path>）：排查「同一场景逐次结果不同」用——
    // 看命令流从第几个周期开始分歧、以及周期数/时间戳是否 1:1。
    std::ofstream tcsv;
    const char* trace_path = std::getenv("S4_TRACE");
    if (trace_path != nullptr) {
        tcsv.open(trace_path);
        tcsv << "iter,t,rel,phase_idx";
        for (int i = 0; i < 12; ++i) tcsv << ",cmd" << i;
        tcsv << "\n";
    }
    long long iter = 0;

    Vec12d q_start = stand;      // settle 结束时的关节角
    Vec12d q_des = stand;
    Vec12d q_des_prev = stand;
    size_t cur_idx = -1;

    const auto period = std::chrono::microseconds(1000);
    auto next_tick = std::chrono::steady_clock::now();
    int status = 0;

    double last_t = -1.0;
    double last_rel = 0.0;                     // 上一周期的 rel（算实际 dt，给斜坡/速率限幅用）
    double last_rel_jv = 0.0;
    const Vec12d nominal_feet = gait.nominal_feet();
    GaitCommand cmd_filt;                      // S4_RAMP 的滤波状态
    bool gait_was_stepping = false;            // 步态起步沿检测（起步混合 / M1 期望高度）
    double t_gait_start = 0.0;
    Eigen::Matrix<double, 3, kNumLegs> feet_at_start;
    double h_des = 0.0;
    Vec3d rpy_lp = Vec3d::Zero();              // M1 一阶低通状态
    bool lp_init = false;
    while (true) {
        next_tick += period;
        std::this_thread::sleep_until(next_tick);

        const double t = ri->GetInterfaceTimeStamp();
        // 同一观测只算一次：仿真侧锁步（每观测一步），重复计算会得到种子略变的 IK 解，
        // 使「下发的是哪一份命令」取决于发送线程与主循环的墙钟竞争 → 逐次结果不可复现。
        if (t <= last_t) continue;
        last_t = t;
        const double rel = t - t0;
        if (rel >= total_duration) break;

        // 当前阶段
        double acc = 0.0;
        size_t idx = 0;
        for (; idx < phases.size(); ++idx) {
            if (rel < acc + phases[idx].duration) break;
            acc += phases[idx].duration;
        }
        if (idx >= phases.size()) break;
        const double local = rel - acc;

        if (idx != cur_idx) {
            cur_idx = idx;
            pcsv << phases[idx].name << "," << (t0 + acc) << "," << (t0 + acc + phases[idx].duration)
                 << "," << phases[idx].cmd.vx << "," << phases[idx].cmd.vy << "," << phases[idx].cmd.wz
                 << "\n";
            pcsv.flush();
            std::printf("[s4] t=%.2f enter phase '%s' (%.1fs) cmd=(%.2f,%.2f,%.2f)\n", rel,
                        phases[idx].name.c_str(), phases[idx].duration, phases[idx].cmd.vx,
                        phases[idx].cmd.vy, phases[idx].cmd.wz);
        }

        const Vec12d q_meas = ri->GetJointPosition().cast<double>();
        const Vec12d dq_meas = ri->GetJointVelocity().cast<double>();
        (void)dq_meas;

        double eff_kp = kp, eff_kd = kd;

        if (idx == 0) {
            if (start_standing) {
                // 仿真已从站立平衡位开始：本程序从第一帧就用 PD 保持站立构型
                q_des = stand;
            } else {
                // settle：自由下落/落地，不施加力矩
                q_start = q_meas;
                q_des = q_meas;
                eff_kp = 0.0;
                eff_kd = 0.0;
            }
        } else if (idx == 1) {
            if (start_standing) {
                q_des = stand;  // 已在站立构型，斜坡退化为保持
            } else {
                // 站立斜坡：关节空间平滑插值
                const double s = SmoothStep(local / phases[1].duration);
                q_des = (1.0 - s) * q_start + s * stand;
            }
        } else {
            // ---- 命令预处理（docs/03 §7.1/§7.2：死区 -> 步幅限幅 -> 斜坡滤波）----
            const double dt_ctrl = std::max(1e-6, rel - last_rel);
            last_rel = rel;
            GaitCommand cmd = phases[idx].cmd;
            if (deadzone > 0.0) {
                if (std::fabs(cmd.vx) <= deadzone) cmd.vx = 0.0;
                if (std::fabs(cmd.vy) <= deadzone) cmd.vy = 0.0;
                if (std::fabs(cmd.wz) <= deadzone) cmd.wz = 0.0;
            }
            if (sweep_gain != 1.0) {
                cmd.vx *= sweep_gain;
                cmd.vy *= sweep_gain;
                cmd.wz *= sweep_gain;
            }
            if (step_x > 0.0 || step_y > 0.0)
                cmd = LimitStep(cmd, gait_params.period * gait_params.duty, nominal_feet, step_x, step_y);
            if (cmd_rate > 0.0) {
                const double md = cmd_rate * dt_ctrl;
                cmd_filt.vx = Approach(cmd_filt.vx, cmd.vx, md);
                cmd_filt.vy = Approach(cmd_filt.vy, cmd.vy, md);
                cmd_filt.wz = Approach(cmd_filt.wz, cmd.wz, md);
                cmd = cmd_filt;
            }

            // 步态 + 起步足端混合（1 s smoothstep：从进入步态时的实测脚位混到规划轨迹）+ M1
            auto legs = gait.Update(rel, cmd);
            const bool stepping = gait.stepping();
            if (stepping && !gait_was_stepping) {
                t_gait_start = rel;
                feet_at_start = model.FootPositionsBody(q_meas);
                double zsum = 0.0;
                int nst = 0;
                for (int leg = 0; leg < kNumLegs; ++leg)
                    if (legs[leg].contact) { zsum += feet_at_start(2, leg); ++nst; }
                h_des = (nst > 0) ? model.geo.foot_radius - zsum / nst : 0.0;
            }
            gait_was_stepping = stepping;

            const double alpha = (blend_win > 0.0 && stepping)
                                     ? SmoothStep((rel - t_gait_start) / blend_win)
                                     : 1.0;
            for (int leg = 0; leg < kNumLegs; ++leg)
                if (alpha < 1.0)
                    legs[leg].foot_pos = (1.0 - alpha) * feet_at_start.col(leg) + alpha * legs[leg].foot_pos;

            // M1：机身高度/姿态补偿（只改支撑脚目标 z，docs/03 §7.3）
            if (m1_on && stepping) {
                const Vec3f rpy_raw = ri->GetImuRpy();
                if (!lp_init) { lp_init = true; rpy_lp = rpy_raw.cast<double>(); }
                const double a = dt_ctrl / (m1_tau + dt_ctrl);
                rpy_lp = (1.0 - a) * rpy_lp + a * rpy_raw.cast<double>();
                double h_meas = 0.0;
                int nst = 0;
                for (int leg = 0; leg < kNumLegs; ++leg)
                    if (legs[leg].contact) {
                        h_meas += model.FootPositionBody(leg, q_meas.segment<3>(3 * leg)).z();
                        ++nst;
                    }
                const double h_err = (nst > 0) ? (model.geo.foot_radius - h_meas / nst) - h_des : 0.0;
                for (int leg = 0; leg < kNumLegs; ++leg) {
                    if (!legs[leg].contact) continue;  // 只改支撑脚
                    const double lever_x = legs[leg].foot_pos.x();
                    const double lever_y = legs[leg].foot_pos.y();
                    double dz = m1_gh * h_err + m1_sr * m1_gr * (lever_y * rpy_lp(0)) -
                                m1_sp * m1_gp * (lever_x * rpy_lp(1));
                    dz = std::max(-m1_dz, std::min(m1_dz, dz));
                    legs[leg].foot_pos.z() += dz;
                }
            }

            for (int leg = 0; leg < kNumLegs; ++leg) {
                Vec3d foot = legs[leg].foot_pos;
                if (legs[leg].contact && stance_press > 0.0) foot.z() -= stance_press;
                Vec3d seed = q_des_prev.segment<3>(3 * leg);
                Vec3d q_leg;
                if (model.InverseKinematics(leg, foot, &q_leg, &seed))
                    q_des.segment<3>(3 * leg) = q_leg;
                else
                    q_des.segment<3>(3 * leg) = q_des_prev.segment<3>(3 * leg);  // 兜底：沿用上一帧
            }
        }

        // 关节目标速率限幅（docs/03 §7.1：6 rad/s，防 PD 速度项过大）
        if (jv_rate > 0.0) {
            const double md = jv_rate * std::max(1e-6, rel - last_rel_jv);
            for (int i = 0; i < 12; ++i) q_des(i) = Approach(q_des_prev(i), q_des(i), md);
        }
        last_rel_jv = rel;

        // 关节位置限幅
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int j = 0; j < 3; ++j) {
                const double lo = model.geo.joint_lower(j), hi = model.geo.joint_upper(j);
                q_des(3 * leg + j) = std::max(lo, std::min(hi, q_des(3 * leg + j)));
            }

        // 安全：姿态超限则退出
        const Vec3f rpy = ri->GetImuRpy();
        if (idx >= 1 && (std::fabs(rpy(0)) > 0.8f || std::fabs(rpy(1)) > 0.8f)) {
            std::fprintf(stderr, "[s4] attitude limit exceeded at t=%.2f: roll=%.3f pitch=%.3f\n", rel,
                         rpy(0), rpy(1));
            status = 3;
            break;
        }

        Eigen::Matrix<float, Eigen::Dynamic, 5> cmd(12, 5);
        cmd.col(0).setConstant(static_cast<float>(eff_kp));
        cmd.col(1) = q_des.cast<float>();
        cmd.col(2).setConstant(static_cast<float>(eff_kd));
        cmd.col(3).setZero();
        cmd.col(4).setZero();
        ri->SetJointCommand(cmd);

        q_des_prev = q_des;

        if (tcsv.is_open()) {
            tcsv << iter++ << "," << std::setprecision(10) << t << "," << rel << "," << idx;
            for (int i = 0; i < 12; ++i) tcsv << "," << q_des(i);
            tcsv << "\n";
        }
    }

    // 退出：停止接口线程并保持站姿（便于仿真继续稳定）
    ri->Stop();
    pcsv.close();
    std::printf("[s4] finished, status=%d\n", status);
    std::fflush(stdout);
    return status;
}
