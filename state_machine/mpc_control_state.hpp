/*===============================================================================================
 * @file mpc_control_state.hpp
 * @brief MPC+WBC 状态适配：连接状态机、仿真接口与 07 核心控制器
 * @details 主线程在收到新测量时发布 MPC 请求、读取最近参考、计算并发送关节命令。
 * MPC 工作线程只求解 SRBD-MPC，通过快照发布结果，不读写机器人接口。
 * OnEnter 在站立完成后复位并启动求解线程；OnExit 停止并回收线程。
 * 本状态负责按键切换和失控转阻尼，具体运动控制由 MpcWbcController 实现。
 ===============================================================================================*/
#pragma once

#include "state_base.h"
#include "07_mpc_wbc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

class MPCControlState final : public StateBase {
public:
    MPCControlState(const RobotType& robot_type, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr), controller_(LoadParams()),
          mpc_worker_(&controller_, controller_.params().mpc_period) 
    {
        std::printf("[mpc_wbc] 站立：基座高度=%.4f m，质心高度=%.4f m；pd_scale=%.2f "
                    "w_base=%.0f w_swing=%.0f mpc_period=%.3f\n",
                    controller_.stand_base_height(), controller_.stand_com_height(),
                    controller_.params().kp_scale_wbc, controller_.params().wbc.w_base,
                    controller_.params().wbc.w_swing, controller_.params().mpc_period);
    }
    ~MPCControlState() { mpc_worker_.Stop(); }

    void OnEnter() override {
        const double t = ri_ptr_->GetInterfaceTimeStamp();
        const mpc_wbc::Vec12d q = ri_ptr_->GetJointPosition().cast<double>();
        controller_.Reset(t, q);
        last_run_t_ = t;
        mpc_worker_.Start();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::MPCControlMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override {
        mpc_worker_.Stop();   // 退出 MPC 状态前停止并回收求解线程；关节命令始终由主线程发送
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
        return StateName::kMPCControl;
    }

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

    /// 用户命令 → 物理速度命令（沿用 ONNX 路径的 forward/side/turnning scale 语义）
    mpc_wbc::GaitCommand BuildGaitCommand() const {
        const auto cmd = uc_ptr_->GetUserCommand();
        const auto& params = controller_.params();
        mpc_wbc::GaitParams input_limits = params.gait;
        if (params.use_gait_schedule) {
            input_limits.max_vx = params.gait_schedule.max_vx;
            input_limits.max_vy = params.gait_schedule.max_vy;
            input_limits.max_wz = params.gait_schedule.max_wz;
        }
        return mpc_wbc::GaitTrajectory::ScaleCommand(
            mpc_wbc::Vec3d(cmd.forward_vel_scale, cmd.side_vel_scale, cmd.turnning_vel_scale),
            input_limits);
    }

    mpc_wbc::MpcWbcController controller_;
    mpc_wbc::MpcWorkerThread mpc_worker_;
    double last_run_t_ = 0.0;
};
