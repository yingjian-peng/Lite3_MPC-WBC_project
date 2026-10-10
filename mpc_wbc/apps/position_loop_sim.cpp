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
    if (const char* e = std::getenv("S4_KP")) kp = std::atof(e);
    if (const char* e = std::getenv("S4_KD")) kd = std::atof(e);
    if (const char* e = std::getenv("S4_SWING_HEIGHT")) gait_params.swing_height = std::atof(e);
    if (const char* e = std::getenv("S4_DUTY")) gait_params.duty = std::atof(e);
    if (const char* e = std::getenv("S4_PERIOD")) gait_params.period = std::atof(e);
    if (const char* e = std::getenv("S4_STANCE_PRESS")) stance_press = std::atof(e);
    if (const char* e = std::getenv("S4_CMD_SCALE")) cmd_scale = std::atof(e);

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

    Vec12d q_start = stand;      // settle 结束时的关节角
    Vec12d q_des = stand;
    Vec12d q_des_prev = stand;
    size_t cur_idx = -1;

    const auto period = std::chrono::microseconds(1000);
    auto next_tick = std::chrono::steady_clock::now();
    int status = 0;

    while (true) {
        next_tick += period;
        std::this_thread::sleep_until(next_tick);

        const double t = ri->GetInterfaceTimeStamp();
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
            // settle：自由下落/落地，不施加力矩
            q_start = q_meas;
            q_des = q_meas;
            eff_kp = 0.0;
            eff_kd = 0.0;
        } else if (idx == 1) {
            // 站立斜坡：关节空间平滑插值
            const double s = SmoothStep(local / phases[1].duration);
            q_des = (1.0 - s) * q_start + s * stand;
        } else {
            // 步态 + IK
            const auto legs = gait.Update(rel, phases[idx].cmd);
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
    }

    // 退出：停止接口线程并保持站姿（便于仿真继续稳定）
    ri->Stop();
    pcsv.close();
    std::printf("[s4] finished, status=%d\n", status);
    std::fflush(stdout);
    return status;
}
