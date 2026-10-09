/**
 * @file mpc_control_state.hpp
 * @brief S8 MPC+WBC 控制状态（对应 docs/04 §3 阶段 D）。
 *
 * 线程分工（实测结论，见 docs/04 §8 S8）：
 *   - **主线程 / 1 kHz**（`Run()`，由 `StateMachine::Run()` 在接口时间戳前进时调用）：
 *     读测量 → `BuildMpcRequest`（纯赋值）→ 读最近一次 wrench 快照 → `Step` 算 12×5 命令
 *     → `SetJointCommand`。这条路径**唯一**写关节命令，且实测单帧 p99 < 0.6 ms。
 *   - **MPC 工作线程**：只解 SRBD-MPC（盒约束 QP，偶发 10~30 ms 抖动），结果写进序列锁快照。
 *     放在主线程里会让 1 kHz 的 WBC 漏拍、行走失稳（实测 trot 段 4 s 后 tilt 发散）。
 *
 * 进出约束（docs/04 §3 S8 通过条件）：
 *   - `OnEnter` 先复位控制器并记录起始关节角；`OnExit` **先 join 工作线程**再放开命令写权，
 *     保证「无双写、无陈旧命令」。
 *   - 进入前机器人应由 `StandUpState` 站好（本状态的斜坡只是平滑接管，不负责从趴地起立）。
 */
#pragma once

#include "state_base.h"
#include "05_mpc_wbc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

class MPCControlState final : public StateBase {
public:
    MPCControlState(const RobotType& robot_type, const std::string& state_name,
                    std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr), controller_(LoadParams()),
          mpc_worker_(&controller_, controller_.params().mpc_period) {
        std::printf("[mpc_wbc] 站立：基座高度=%.4f m，质心高度=%.4f m；pd_scale=%.2f "
                    "w_base=%.0f w_swing=%.0f mpc_period=%.3f\n",
                    controller_.stand_base_height(), controller_.stand_com_height(),
                    controller_.params().kp_scale_wbc, controller_.params().wbc.w_base,
                    controller_.params().wbc.w_swing, controller_.params().mpc_period);
    }

    ~MPCControlState() { StopWorker(); }

    void OnEnter() override {
        const double t = ri_ptr_->GetInterfaceTimeStamp();
        const mpc_wbc::Vec12d q = ri_ptr_->GetJointPosition().cast<double>();
        controller_.Reset(t, q);
        last_run_t_ = t;
        started_ = true;
        mpc_worker_.Start();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override {
        StopWorker();   // 先停并回收工作线程，再交出命令写权（避免线程里还持有 SetJointCommand 的所有权）
        started_ = false;
    }

    void Run() override {
        const double t = ri_ptr_->GetInterfaceTimeStamp();
        if (t <= last_run_t_) return;   // 同一时间戳不重复推进
        last_run_t_ = t;

        mpc_wbc::ControllerInput in;
        in.t = t;
        in.rpy = ri_ptr_->GetImuRpy().cast<double>();
        in.omega_body = ri_ptr_->GetImuOmega().cast<double>();
        in.acc_body = ri_ptr_->GetImuAcc().cast<double>();
        in.q = ri_ptr_->GetJointPosition().cast<double>();
        in.dq = ri_ptr_->GetJointVelocity().cast<double>();
        in.cmd = BuildGaitCommand();

        // 1 kHz 路径：打包 MPC 请求 → 读最近一次 wrench → 算命令（不在本线程解 MPC）
        mpc_worker_.requests.Write(controller_.BuildMpcRequest(in));
        mpc_wbc::MpcWrench wrench;
        if (!mpc_worker_.wrenches.Read(&wrench)) wrench = controller_.wrench();

        const mpc_wbc::ControllerOutput out = controller_.Step(in, wrench);
        last_output_ = out;
        ri_ptr_->SetJointCommand(out.joint_cmd);

        if (ds_ptr_) {
            ds_ptr_->InsertScopeData(0, out.step_ms);          // 1 kHz 单帧耗时
            ds_ptr_->InsertScopeData(1, static_cast<float>(out.tilt * 180.0 / M_PI));
            ds_ptr_->InsertScopeData(2, out.wbc_active ? 1.0f : 0.0f);
            ds_ptr_->InsertScopeData(3, static_cast<float>(out.wbc_ms));
            ds_ptr_->InsertScopeData(4, static_cast<float>(out.mpc_ms));
        }
        if (out.fault) {
            std::printf("[mpc_wbc] 控制器故障（tilt=%.1f°, dt=%.3f ms）\n", out.tilt * 180.0 / M_PI,
                        out.step_ms);
        }
    }

    bool LoseControlJudge() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        if (controller_.fault()) return true;
        const Vec3f rpy = ri_ptr_->GetImuRpy();
        if (std::fabs(rpy(0)) > 45.f / 180.f * M_PI || std::fabs(rpy(1)) > 45.f / 180.f * M_PI) {
            std::printf("[mpc_wbc] 姿态超限：roll=%.1f° pitch=%.1f°\n", rpy(0) * 180.f / M_PI,
                        rpy(1) * 180.f / M_PI);
            return true;
        }
        return false;
    }

    StateName GetNextStateName() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::LieDown))
            return StateName::kLieDown;
        return StateName::kRLControl;   // 首版沿用数值 6 与按键语义（docs/04 §7 第 4 项）
    }

    /// 诊断（供日志/测试）。
    const mpc_wbc::ControllerOutput& last_output() const { return last_output_; }
    int mpc_solve_count() const { return mpc_worker_.solve_count(); }

private:
    static mpc_wbc::MpcWbcControllerParams LoadParams() {
        mpc_wbc::MpcWbcControllerParams p;
        auto env_d = [](const char* name, double def) {
            const char* s = std::getenv(name);
            return s ? std::atof(s) : def;
        };
        p.kp_scale_wbc = p.kd_scale_wbc = env_d("MPC_WBC_PD_SCALE", p.kp_scale_wbc);
        p.mpc_period = env_d("MPC_WBC_MPC_PERIOD", p.mpc_period);
        p.wbc.w_base = env_d("MPC_WBC_W_BASE", p.wbc.w_base);
        p.wbc.w_swing = env_d("MPC_WBC_W_SWING", p.wbc.w_swing);
        p.gait.period = env_d("MPC_WBC_GAIT_PERIOD", p.gait.period);
        p.gait.duty = env_d("MPC_WBC_GAIT_DUTY", p.gait.duty);
        p.gait.swing_height = env_d("MPC_WBC_SWING_HEIGHT", p.gait.swing_height);
        return p;
    }

    /// 用户命令 → 物理速度命令（沿用 ONNX 路径的 forward/side/turnning scale 语义）。
    mpc_wbc::GaitCommand BuildGaitCommand() const {
        const auto cmd = uc_ptr_->GetUserCommand();
        return mpc_wbc::GaitTrajectory::ScaleCommand(
            mpc_wbc::Vec3d(cmd.forward_vel_scale, cmd.side_vel_scale, cmd.turnning_vel_scale),
            controller_.params().gait);
    }

    void StopWorker() { mpc_worker_.Stop(); }

    mpc_wbc::MpcWbcController controller_;
    mpc_wbc::MpcWorkerThread mpc_worker_;   // 只解 MPC；1 kHz 路径不依赖它的完成
    mpc_wbc::ControllerOutput last_output_;
    double last_run_t_ = 0.0;
    bool started_ = false;
};
