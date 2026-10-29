/**
 * @file state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "liedown_state.hpp"

#include "rl_control_state_onnx.hpp"
#include "mpc_control_state.hpp"

#include <fcntl.h>
#include <unistd.h>
#ifdef BUILD_SIMULATION
    #include "keyboard_interface.hpp"
    #include "simulation/simulation_interface.hpp"
 #else
    #include "skydroid_gamepad_interface.hpp"
    #include "retroid_gamepad_interface.hpp"
    #include "xbox_gamepad_interface.hpp"
    #include "hardware/hardware_interface.hpp"
#endif
#include "data_streaming.hpp"

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> rl_controller_onnx_;   // 原 ONNX 策略状态
    std::shared_ptr<StateBase> mpc_controller_;       // S8 MPC+WBC 状态
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> liedown_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<DataStreaming> ds_ptr_;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));

        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);
       
        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kLieDown:{
                return liedown_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type){
        #ifdef BUILD_SIMULATION
            std::cout << "Simulation input: KeyboardInterface" << std::endl;
            uc_ptr_ = std::make_shared<KeyboardInterface>();
        #else
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        #endif
        if(robot_type == RobotType::Lite3){
            #ifdef BUILD_SIMULATION
                ri_ptr_ = std::make_shared<SimulationInterface>("Lite3");
            #else
                ri_ptr_ = std::make_shared<HardwareInterface>("Lite3");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        }else{
            std::cerr << "error" << std::endl;
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);

        // 启动配置：onnx（默认）或 mpc_wbc（LITE3_CONTROL_PATH=mpc_wbc）。
        // 两条路径**互斥构造**：选 MPC 时不构造 ONNX 状态（ONNX 构造会立刻加载 policy.onnx，
        // 缺文件会直接抛异常终止），反之亦然。
        const char* ctrl_path = std::getenv("LITE3_CONTROL_PATH");
        const bool use_mpc = (ctrl_path != nullptr && std::string(ctrl_path) == "mpc_wbc");
        if (use_mpc) {
            mpc_controller_ = std::make_shared<MPCControlState>(robot_type, "mpc_wbc_control", data_ptr);
            rl_controller_ = mpc_controller_;
        } else {
            rl_controller_onnx_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
            rl_controller_ = rl_controller_onnx_;
        }
        std::cout << "Control path: " << (use_mpc ? "mpc_wbc (MPC+WBC)" : "onnx (RL policy)")
                  << std::endl;
        


        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);
        liedown_controller_ = std::make_shared<LieDownState>(robot_type, "liedown_state", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
   
        // std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();
        
        current_controller_->OnEnter();  
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static double time_record = 0;
        while(true){
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();
                
                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++cnt;
                this->GetDataStreaming();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }

};
