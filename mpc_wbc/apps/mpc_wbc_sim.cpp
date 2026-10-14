/**
 * @file mpc_wbc_sim.cpp
 * @brief S8 离线 harness：整机 MPC+WBC 通过 UDP 与 MuJoCo 通信（无状态机、无 ONNX）。
 *
 * 与 `position_loop_sim.cpp`（S4 位置环基线）同一套时间线/协议，但控制律换成
 * `MpcWbcController`：估计 → 步态 → SRBD-MPC(20 Hz) → Weighted-QP WBC → 12×5 命令。
 * 作用是把「算法编排」先在最短路径上跑通；状态机版本（`state_machine/mpc_control_state.hpp`）
 * 复用同一个控制器对象。
 *
 * 时间线： 静落(无扭矩) -> 起立(位置环，等价 StandUpState) -> [MPC+WBC 接管] 站立 ->
 *          前进/后退/侧移/转向（段间停步）
 *
 * 为什么起立用位置环：真机上进入 MPC 状态前机器人已由 `StandUpState` 站好，从「趴地」直接
 * 插值到蹲姿并不稳（实测：趴地时腿几乎伸直、机身贴地，插值过程会把机身顶翻）。这里与状态机
 * 保持同一分工：位置环负责起立，MPC+WBC 只负责站立之后的平衡与行走。
 * 安全：   姿态超限或控制器故障 → 停止并返回非零。
 *
 * 用法： ./mpc_wbc_mpc_wbc_sim [phases_csv] [diag_csv]
 *        （需先启动 interface/robot/simulation/mujoco_simulation.py）
 */
#include "simulation/simulation_interface.hpp"

#include "05_mpc_wbc_controller.hpp"
#include "scenario_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
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

double SmoothStep01(double x) {
    x = std::max(0.0, std::min(1.0, x));
    return x * x * (3.0 - 2.0 * x);
}

/// 三次样条（与 state_machine/standup_state.hpp 的 GetCubicSplinePos 同式）。
double CubicSpline(double x0, double v0, double xf, double vf, double t, double T) {
    if (t >= T) return xf;
    const double c = v0;
    const double a = (vf * T - 2.0 * xf + v0 * T + 2.0 * x0) / std::pow(T, 3);
    const double b = (3.0 * xf - vf * T - 2.0 * v0 * T - 3.0 * x0) / std::pow(T, 2);
    return a * t * t * t + b * t * t + c * t + x0;
}

/// 腿长 h -> HipY / Knee（与 StandUpState::GetHipYPosByHeight/GetKneePosByHeight 同式）。
double HipYByHeight(double h, double l1, double l2) {
    if (std::fabs(h) >= l1 + l2) return 0.0;
    return -std::acos((l1 * l1 + h * h - l2 * l2) / (2.0 * h * l1));
}
double KneeByHeight(double h, double l1, double l2) {
    if (std::fabs(h) >= l1 + l2) return 0.0;
    return M_PI - std::acos((l1 * l1 + l2 * l2 - h * h) / (2.0 * l1 * l2));
}

const char* ModeName(ControllerOutput::Mode m) {
    switch (m) {
        case ControllerOutput::Mode::kRamp: return "ramp";
        case ControllerOutput::Mode::kStand: return "stand";
        case ControllerOutput::Mode::kWalk: return "walk";
        case ControllerOutput::Mode::kFault: return "fault";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv) {
    const std::string phases_csv = (argc > 1) ? argv[1] : "s8_phases.csv";
    const std::string diag_csv = (argc > 2) ? argv[2] : "s8_diag.csv";

    auto ri = std::make_shared<SimulationInterface>("Lite3", 12);
    ri->Start();

    const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (ri->GetInterfaceTimeStamp() <= 0.0) {
        if (std::chrono::steady_clock::now() > wait_deadline) {
            std::fprintf(stderr, "[s8] no simulation data received\n");
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    const std::string phase_set = std::getenv("S8_PHASES") ? std::getenv("S8_PHASES") : "walk";
    // 起立段（两阶段，与 StandUpState 等价）在所有场景里都一样 → 保证两条路径"起点"相同
    std::vector<Phase> phases = {
        {"settle", 0.8, {0.0, 0.0, 0.0}},   // 自由落地，无扭矩
        {"fold", 1.5, {0.0, 0.0, 0.0}},     // 起立阶段 1：收腿到 h=pre_height（等价 StandUpState）
        {"raise", 1.5, {0.0, 0.0, 0.0}},    // 起立阶段 2：腿长 h 从 pre_height 升到 stand_height
        {"hold", 1.0, {0.0, 0.0, 0.0}},
        {"walk_fwd", 3.0, {0.15, 0.0, 0.0}},
        {"stop1", 1.0, {0.0, 0.0, 0.0}},
        {"walk_back", 3.0, {-0.15, 0.0, 0.0}},
        {"stop2", 1.0, {0.0, 0.0, 0.0}},
        {"walk_left", 3.0, {0.0, 0.12, 0.0}},
        {"stop3", 1.0, {0.0, 0.0, 0.0}},
        {"turn", 3.0, {0.0, 0.0, 0.4}},
        {"stop4", 1.0, {0.0, 0.0, 0.0}},
    };
    if (phase_set == "stand") {  // 只验证站立：MPC+WBC 长时间保持平衡
        phases = {{"settle", 0.8, {0.0, 0.0, 0.0}},
                  {"fold", 1.5, {0.0, 0.0, 0.0}},
                  {"raise", 1.5, {0.0, 0.0, 0.0}},
                  {"hold", 20.0, {0.0, 0.0, 0.0}}};
    } else if (scenario_io::LooksLikePath(phase_set)) {
        // S9 A/B：从场景文件读运动段（与 S4 基线共用同一份命令序列）
        std::vector<scenario_io::Item> items;
        std::string err;
        if (!scenario_io::Load(phase_set, &items, &err)) {
            std::fprintf(stderr, "[s8] %s\n", err.c_str());
            return 2;
        }
        phases.resize(4);  // 保留 settle/fold/raise/hold，换掉 hold 时长
        phases[3].duration = 0.5;
        for (const auto& it : items)
            phases.push_back({it.name, it.duration, {it.vx, it.vy, it.wz}});
        std::printf("[s8] scenario %s: %zu motion phases, total %.1f s\n", phase_set.c_str(),
                    items.size(), [&] {
                        double s = 0.0;
                        for (const auto& p : phases) s += p.duration;
                        return s;
                    }());
    }
    double total_duration = 0.0;
    for (const auto& p : phases) total_duration += p.duration;
    // 起立两阶段参数（与 state_machine/parameters/lite3_control_parameters.cpp 一致）
    const double kPreHeight = 0.12;    // pre_height_
    const double kStandHeight = 0.33;  // stand_height_
    const double kSettle = phases[0].duration;
    const double kFoldEnd = phases[0].duration + phases[1].duration;
    const double kHandover = kFoldEnd + phases[2].duration;  // 控制器最早在 raise 结束时接管
    // 交棒判据（不只看时间）：位置环把腿摆到站姿才允许 MPC+WBC 接管。
    // 只看时间时，若这一轮起立慢半拍（静落弹跳 / CPU 抢占），控制器会在半蹲状态被启用 → 必翻；
    // 那属于"接管条件"问题而非控制律问题，会把 S9 的整定数据污染成随机 PASS/FAIL。
    const double kHandoverErr = 0.15;   // 关节角与 stand_q 的最大误差 [rad]
    const double kHandoverTilt = 0.20;  // 姿态 [rad]
    const double kHandoverWait = 1.5;   // 宽限：超过则判"起立未收敛"退出（而不是带病交棒）

    const double t0 = ri->GetInterfaceTimeStamp();

    std::ofstream pcsv(phases_csv);
    pcsv << "phase,t_start,t_end,vx,vy,wz\n";
    pcsv.flush();

    std::ofstream dcsv(diag_csv);
    dcsv << "t,phase,mode,wbc,stale,dt,tilt_deg,height,est_conf,est_valid,num_support,"
            "kp,tau_ff_max,tau_wbc_max,sum_fz,a_des_x,a_des_z,alpha_des_y,eom_res,contact_res,"
            "cone_viol,base_acc_err,swing_acc_err,wbc_iters,wbc_ms,mpc_ms,step_ms,mpc_status,"
            "mpc_iters,mpc_res,v_est_x,v_est_y,wrench_age,mpc_reason,mpc_held,"
                 "nstance,sdz0,saz0,sdz1,saz1,sdz2,saz2,sdz3,saz3\n";
    dcsv.flush();

    double cmd_scale = 1.0;   // 命令缩放（A/B 用）
    MpcWbcControllerParams cparams;
    if (const char* e = std::getenv("S8_PD_SCALE")) cparams.kp_scale_wbc = cparams.kd_scale_wbc = std::atof(e);
    if (const char* e = std::getenv("S8_W_SWING")) cparams.wbc.w_swing = std::atof(e);
    if (const char* e = std::getenv("S8_MPC_PERIOD")) cparams.mpc_period = std::atof(e);
    if (const char* e = std::getenv("S8_HYST")) cparams.wbc_enable_hold_frames = std::atoi(e);
    if (const char* e = std::getenv("S8_CMD_SCALE")) cmd_scale = std::atof(e);
    std::printf("[s8] pd_scale=%.2f w_swing=%.0f mpc_period=%.3f\n", cparams.kp_scale_wbc,
                cparams.wbc.w_swing, cparams.mpc_period);
    MpcWbcController controller(cparams);
    MpcWorkerThread mpc_worker(&controller, cparams.mpc_period);
    std::printf("[s8] 站立：基座高度=%.4f m，质心高度=%.4f m\n", controller.stand_base_height(),
                controller.stand_com_height());

    bool controller_started = false;
    int post_handover_frames = 0;
    Vec12d q_standup_start = Vec12d::Zero();
    size_t cur_idx = -1;
    int status = 0;

    const auto period = std::chrono::microseconds(1000);
    auto next_tick = std::chrono::steady_clock::now();

    // 用 -1 起，允许程序启动后立即处理读到的第一帧观测。
    double last_seen_t = -1.0;
    auto last_seen_wall = std::chrono::steady_clock::now();

    while (true) {
        next_tick += period;
        std::this_thread::sleep_until(next_tick);

        const double t = ri->GetInterfaceTimeStamp();
        const double rel = t - t0;
        if (rel >= total_duration) break;

        // 仿真掉线保护；只在时间戳前进时更新控制目标。
        if (t > last_seen_t + 1e-12) {
            last_seen_t = t;
            last_seen_wall = std::chrono::steady_clock::now();
        } else if (std::chrono::steady_clock::now() - last_seen_wall > std::chrono::seconds(1)) {
            std::fprintf(stderr, "[s8] simulation stalled at t=%.2f (no new data for 1 s)\n", rel);
            status = 5;
            break;
        } else {
            continue;
        }

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
            std::printf("[s8] t=%.2f enter phase '%s' (%.1fs) cmd=(%.2f,%.2f,%.2f)\n", rel,
                        phases[idx].name.c_str(), phases[idx].duration, phases[idx].cmd.vx,
                        phases[idx].cmd.vy, phases[idx].cmd.wz);
        }

        const Vec12d q_meas = ri->GetJointPosition().cast<double>();
        const Vec12d dq_meas = ri->GetJointVelocity().cast<double>();
        const Vec3f rpy_f = ri->GetImuRpy();
        const Vec3f omg_f = ri->GetImuOmega();
        const Vec3f acc_f = ri->GetImuAcc();
        const Vec3d rpy = rpy_f.cast<double>();
        const double tilt = std::max(std::fabs(rpy(0)), std::fabs(rpy(1)));

        Eigen::Matrix<float, Eigen::Dynamic, 5> cmd(12, 5);
        cmd.setZero();

        ControllerOutput out;
        const double standup_err = (q_meas - controller.params().stand_q).cwiseAbs().maxCoeff();
        const bool standup_ready = standup_err < kHandoverErr && tilt < kHandoverTilt;
        if (!controller_started && rel > kHandover + kHandoverWait && !standup_ready) {
            std::fprintf(stderr,
                         "[s8] 起立未收敛（关节误差 %.3f rad > %.2f，tilt %.1f deg），放弃交棒\n",
                         standup_err, kHandoverErr, tilt * 180.0 / M_PI);
            status = 5;
            break;
        }
        if (!controller_started && (rel < kHandover || !standup_ready)) {
            // 静落 + 位置环起立（与 S4 同一套档位：kp=100 / kd=2.5）
            const MpcWbcControllerParams& cp = controller.params();
            const double l1 = 0.20, l2 = 0.21012;  // thigh_len_ / shank_len_（与 MJCF 一致）
            if (rel < kSettle) {
                q_standup_start = q_meas;
                cmd.col(0).setZero();
                cmd.col(1) = q_meas.cast<float>();
                cmd.col(2).setZero();
            } else if (rel < kFoldEnd) {
                // 阶段 1：从落地姿势收腿到 h=pre_height（等价 StandUpState 的样条段）
                Vec12d q_target;
                const double t = rel - kSettle;
                for (int leg = 0; leg < kNumLegs; ++leg)
                    for (int j = 0; j < 3; ++j) {
                        const double target = (j == 0) ? 0.0
                                             : (j == 1) ? HipYByHeight(kPreHeight, l1, l2)
                                                        : KneeByHeight(kPreHeight, l1, l2);
                        q_target(3 * leg + j) =
                            CubicSpline(q_standup_start(3 * leg + j), 0.0, target, 0.0, t,
                                        phases[1].duration);
                    }
                cmd.col(0).setConstant(100.0f);
                cmd.col(1) = q_target.cast<float>();
                cmd.col(2).setConstant(2.5f);
            } else {
                // 阶段 2：腿长从 pre_height 升到 stand_height（等价 StandUpState 的高度段）
                const double t = rel - kFoldEnd;
                const double h = CubicSpline(kPreHeight, 0.0, kStandHeight, 0.0, t, phases[2].duration);
                Vec12d q_target;
                for (int leg = 0; leg < kNumLegs; ++leg) {
                    q_target(3 * leg + 0) = 0.0;
                    q_target(3 * leg + 1) = HipYByHeight(h, l1, l2);
                    q_target(3 * leg + 2) = KneeByHeight(h, l1, l2);
                }
                (void)cp;
                cmd.col(0).setConstant(100.0f);
                cmd.col(1) = q_target.cast<float>();
                cmd.col(2).setConstant(2.5f);
            }
            cmd.col(3).setZero();
            cmd.col(4).setZero();
        } else {
            if (!controller_started) {
                controller_started = true;
                controller.Reset(t, q_meas);
                mpc_worker.Start();
                std::printf("[s8] t=%.2f MPC+WBC engaged (起立误差 %.3f rad, tilt %.1f deg)\n", rel,
                            standup_err, tilt * 180.0 / M_PI);
            }
            ControllerInput in;
            in.t = t;
            in.rpy = rpy;
            in.omega_body = omg_f.cast<double>();
            in.acc_body = acc_f.cast<double>();
            in.q = q_meas;
            in.dq = dq_meas;
            in.cmd = phases[idx].cmd;
            in.cmd.vx *= cmd_scale;
            in.cmd.vy *= cmd_scale;
            in.cmd.wz *= cmd_scale;
            // 1 kHz 路径：只打包 MPC 请求（不求解）→ 读最近一次 wrench → 算命令
            mpc_worker.requests.Write(controller.BuildMpcRequest(in));
            MpcWrench wrench;
            if (!mpc_worker.wrenches.Read(&wrench)) wrench = controller.wrench();
            out = controller.Step(in, wrench);
            cmd = out.joint_cmd;
            // 接管 0.2 s 后回读估计器（engage 那一瞬还没跑过 Step，打印估计值没有意义）
            if (++post_handover_frames == 200)
                std::printf("[s8] 接管 0.2s：估计高度 %.4f m，置信度 %.2f，支撑腿 %d，WBC %s\n",
                            out.height, out.est_confidence, controller.estimate().num_support,
                            out.wbc_active ? "on" : "off");
        }
        ri->SetJointCommand(cmd);

        if (controller_started) {
            double tau_ff_max = 0.0, sum_fz = 0.0;
            for (int j = 0; j < 12; ++j)
                tau_ff_max = std::max(tau_ff_max, std::fabs(static_cast<double>(cmd(j, 4))));
            for (int leg = 0; leg < kNumLegs; ++leg) sum_fz += out.foot_force[leg].z();
            dcsv << t << "," << phases[idx].name << "," << ModeName(out.mode) << ","
                 << (out.wbc_active ? 1 : 0) << "," << (out.wbc_stale ? 1 : 0) << "," << out.dt << ","
                 << (tilt * 180.0 / M_PI) << "," << out.height << "," << out.est_confidence << ","
                 << (controller.estimate().valid ? 1 : 0) << "," << controller.estimate().num_support
                 << "," << cmd(0, 0) << "," << tau_ff_max << ","
                 << out.tau_wbc.cwiseAbs().maxCoeff() << "," << sum_fz << "," << out.a_des.x() << ","
                 << out.a_des.z() << "," << out.alpha_des.y() << "," << out.wbc_eom_residual << ","
                 << out.wbc_contact_residual << "," << out.wbc_cone_violation << ","
                 << out.wbc_base_accel_error << "," << out.wbc_swing_accel_error << ","
                 << out.wbc_iters << "," << out.wbc_ms << "," << out.mpc_ms << "," << out.step_ms
                 << "," << out.mpc_status << "," << out.mpc_iters << "," << out.mpc_qp_residual
                 << "," << controller.estimate().base_vel_w.x() << ","
                 << controller.estimate().base_vel_w.y() << "," << out.mpc_wrench_age << ","
                 << out.mpc_reason << "," << (out.mpc_held ? 1 : 0) << "," << out.stance_count
                 << "," << out.swing_des_z[0] << "," << out.swing_now_z[0]
                 << "," << out.swing_des_z[1] << "," << out.swing_now_z[1]
                 << "," << out.swing_des_z[2] << "," << out.swing_now_z[2]
                 << "," << out.swing_des_z[3] << "," << out.swing_now_z[3]
                 << "\n";
            // 注意：文件 flush 会阻塞几 ms，绝不能放在控制计时区间里（这里在写行之后）
            if (static_cast<long>(rel * 1000) % 500 == 0) dcsv.flush();
            if (out.fault) {
                std::fprintf(stderr, "[s8] controller fault at t=%.2f (tilt=%.1f deg, phase=%s)\n",
                             rel, tilt * 180.0 / M_PI, phases[idx].name.c_str());
                status = 4;
                break;
            }
        }

        // 安全：姿态超限则退出
        if (rel > kHandover && tilt > 0.8) {
            std::fprintf(stderr, "[s8] attitude limit exceeded at t=%.2f: roll=%.3f pitch=%.3f\n", rel,
                         rpy(0), rpy(1));
            status = 3;
            break;
        }
    }

    mpc_worker.Stop();
    ri->Stop();
    pcsv.close();
    dcsv.close();
    std::printf("[s8] finished, status=%d (MPC 线程解了 %d 次)\n", status, mpc_worker.solve_count());
    std::fflush(stdout);
    return status;
}
