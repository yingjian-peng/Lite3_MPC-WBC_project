/*===============================================================================================
 * @file state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用
 ===============================================================================================*/
#include "state_base.h"                                             // 引入状态基类头文件，定义状态机的基本接口和功能
#include "idle_state.hpp"                                           // 引入空闲状态头文件，定义机器人在空闲状态下的行为和控制逻辑
#include "standup_state.hpp"                                        // 引入站立状态头文件，定义机器人在站立状态下的行为和控制逻辑
#include "joint_damping_state.hpp"                                  // 引入关节阻尼状态头文件，定义机器人在关节阻尼状态下的行为和控制逻辑
#include "liedown_state.hpp"                                        // 引入躺下状态头文件，定义机器人在躺下状态下的行为和控制逻辑

#include "rl_control_state.hpp"                                     // 引入基于 ONNX 的强化学习控制状态头文件，定义机器人在强化学习控制状态下的行为和控制逻辑
#include "mpc_control_state.hpp"                                    // 引入基于模型预测控制（MPC）的控制状态头文件，定义机器人在 MPC 控制状态下的行为和控制逻辑

#include <fcntl.h>                                                  // 引入文件控制头文件，提供对文件描述符的操作和控制功能
#include <unistd.h>                                                 // 引入 Unix 标准头文件，提供对 POSIX 操作系统 API 的访问和功能
#include <cstdlib>                                                  // 读取用户输入方式的环境变量
#include <exception>                                                // 捕获 RL/ONNX 初始化异常

#include "keyboard_interface.hpp"                                 // 仿真使用键盘接收用户命令
#include "xbox_gamepad_interface.hpp"                              // USB 手柄使用 Linux joystick 接口
#include "simulation/simulation_interface.hpp"                    // 仿真接口负责收发机器人状态和关节命令

#include "data_streaming.hpp"                                       // 引入数据流传输头文件，定义用于机器人状态和控制数据的实时传输和记录功能



//===============================================================================================
// @brief 状态机类：管理机器人控制状态的切换和运行
//===============================================================================================
class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;                 // 当前控制器指针，指向当前活动的状态对象
    std::shared_ptr<StateBase> idle_controller_;                    // 空闲状态控制器指针，指向空闲状态对象
    std::shared_ptr<StateBase> standup_controller_;                 // 站立状态控制器指针，指向站立状态对象
    std::shared_ptr<StateBase> rl_controller_;                      // ONNX 强化学习控制状态
    std::shared_ptr<StateBase> mpc_controller_;                     // S8 MPC+WBC 状态
    std::shared_ptr<StateBase> joint_damping_controller_;           // 关节阻尼状态控制器指针，指向关节阻尼状态对象
    std::shared_ptr<StateBase> liedown_controller_;                 // 躺下状态控制器指针，指向躺下状态对象

    StateName current_state_name_, next_state_name_;                // 当前状态名称和下一个状态名称，用于管理状态切换逻辑

    std::shared_ptr<UserCommandInterface> uc_ptr_;                  // 用户命令接口指针，指向用于获取用户输入命令的接口对象
    std::shared_ptr<RobotInterface> ri_ptr_;                        // 机器人接口指针，指向用于与机器人硬件或仿真环境进行交互的接口对象
    std::shared_ptr<ControlParameters> cp_ptr_;                     // 控制参数指针，指向用于存储和管理机器人控制参数的对象
    std::shared_ptr<ControllerData> controller_data_;               // 各状态共用的接口和参数集合
    RobotType robot_type_;                                           // 当前机器人类型，用于延迟创建对应的 RL 状态

    std::shared_ptr<DataStreaming> ds_ptr_;                         // 数据流传输指针，指向用于实时传输和记录机器人状态和控制数据的对象

    //------------------------------------------------------------------------------------
    // @brief 获取数据流传输对象并插入当前机器人状态和控制数据
    //------------------------------------------------------------------------------------
    void GetDataStreaming()
    {
        if(!ri_ptr_) return;

        VecXf pos = ri_ptr_->GetJointPosition();                    // 获取机器人关节位置数据
        VecXf vel = ri_ptr_->GetJointVelocity();                    // 获取机器人关节速度数据
        VecXf tau = ri_ptr_->GetJointTorque();                      // 获取机器人关节力矩数据
        Vec3f rpy = ri_ptr_->GetImuRpy();                           // 获取机器人 IMU 的滚转、俯仰和偏航角数据
        Vec3f acc = ri_ptr_->GetImuAcc();                           // 获取机器人 IMU 的加速度数据
        Vec3f omg = ri_ptr_->GetImuOmega();                         // 获取机器人 IMU 的角速度数据
        MatXf jc = ri_ptr_->GetJointCommand();                      // 获取机器人关节命令数据，包括期望位置、速度和力矩等信息

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp()); // 插入机器人接口时间戳，用于数据同步和记录

        ds_ptr_->InsertJointData("q", pos);                         // 插入机器人关节位置数据，用于数据记录和分析
        ds_ptr_->InsertJointData("dq", vel);                        // 插入机器人关节速度数据，用于数据记录和分析
        ds_ptr_->InsertJointData("tau", tau);                       // 插入机器人关节力矩数据，用于数据记录和分析
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));               // 插入机器人关节命令中的期望位置数据，用于数据记录和分析
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));              // 插入机器人关节命令中的前馈力矩数据，用于数据记录和分析

        ds_ptr_->InsertImuData("rpy", rpy);                         // 插入机器人 IMU 的滚转、俯仰和偏航角数据，用于数据记录和分析
        ds_ptr_->InsertImuData("acc", acc);                         // 插入机器人 IMU 的加速度数据，用于数据记录和分析
        ds_ptr_->InsertImuData("omg", omg);                         // 插入机器人 IMU 的角速度数据，用于数据记录和分析

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();                       // 获取用户命令数据，用于控制机器人行为
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));  // 插入用户命令中的目标模式数据，用于数据记录和分析

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);  // 插入当前状态数据，用于数据记录和分析
       
        ds_ptr_->SendData();                                        // 发送数据流传输对象中的数据，用于实时传输和记录机器人状态和控制数据
    }

    //------------------------------------------------------------------------------------
    // @brief 根据状态名称获取对应的状态对象指针
    //------------------------------------------------------------------------------------
    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name)
    {
        switch(state_name){
            case StateName::kInvalid:{ return nullptr; }                // 如果状态名称无效，返回空指针

            case StateName::kIdle:{ return idle_controller_; }          // 空闲状态-->状态控制器指针

            case StateName::kStandUp:{ return standup_controller_; }    // 站立状态-->状态控制器指针

            case StateName::kRLControl:{                                // 首次选择 RL 时才加载 ONNX 模型
                if (!rl_controller_) {
                    try {
                        rl_controller_ = std::make_shared<RLControlStateONNX>(
                            robot_type_, "rl_control", controller_data_);
                    } catch (const std::exception& e) {
                        std::cerr << "[StateMachine] RL 控制器初始化失败，未切换到 RL："
                                  << e.what() << std::endl;
                        return nullptr;
                    }
                }
                return rl_controller_;
            }

            case StateName::kMPCControl:{ return mpc_controller_; }     // MPC+WBC 控制状态-->状态控制器指针

            case StateName::kJointDamping:{ return joint_damping_controller_; } // 关节阻尼状态-->状态控制器指针

            case StateName::kLieDown:{ return liedown_controller_; }    // 躺下状态-->状态控制器指针

            default:{
                std::cerr << "error state name" << std::endl;
            }
        }

        return nullptr;
    }

public:

    //------------------------------------------------------------------------------------
    // @brief 状态机构造函数：初始化机器人接口、用户命令接口、控制参数和各个状态控制器
    //------------------------------------------------------------------------------------
    StateMachine(RobotType robot_type)
    {
        robot_type_ = robot_type;
        // 默认使用键盘；设置 LITE3_USER_INPUT=usb 时读取飞智沙漠狐有线 XInput 手柄（Linux xpad，默认 /dev/input/js0）。
        const char* input_mode = std::getenv("LITE3_USER_INPUT");
        if (input_mode != nullptr && std::string(input_mode) == "usb") {
            std::cout << "Simulation input: USB gamepad" << std::endl;
            uc_ptr_ = std::make_shared<XboxGamepadInterface>();
        } else {
            std::cout << "Simulation input: KeyboardInterface" << std::endl;
            uc_ptr_ = std::make_shared<KeyboardInterface>();
        }
        if(robot_type == RobotType::Lite3){                                 // 如果机器人类型为 Lite3，创建仿真接口对象和控制参数对象
            ri_ptr_ = std::make_shared<SimulationInterface>("Lite3");       // 创建仿真接口对象，用于与 Lite3 机器人进行交互
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);      // 创建控制参数对象，用于存储和管理 Lite3 机器人的控制参数
        }
        else{
            std::cerr << "error" << std::endl;
        }

        // 创建 ControllerData 对象并初始化各个接口指针
        controller_data_ = std::make_shared<ControllerData>();             // 创建各状态共用的接口和参数集合
        controller_data_->ri_ptr = ri_ptr_;                                 // 初始化机器人接口指针
        controller_data_->uc_ptr = uc_ptr_;                                 // 初始化用户命令接口指针
        controller_data_->cp_ptr = cp_ptr_;                                 // 初始化控制参数指针
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);            // 创建数据流传输对象，用于实时传输和记录机器人状态和控制数据
        controller_data_->ds_ptr = ds_ptr_;                                 // 初始化数据流传输指针

        // 根据机器人类型和状态名称创建各个状态控制器对象，并传入 ControllerData 对象
        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", controller_data_);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", controller_data_);

        // MPC 可直接使用；RL 状态在用户选择 RL 时再创建并加载 ONNX 模型
        mpc_controller_ = std::make_shared<MPCControlState>(robot_type, "mpc_wbc_control", controller_data_);
        
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", controller_data_);
        liedown_controller_ = std::make_shared<LieDownState>(robot_type, "liedown_state", controller_data_);

        current_controller_ = idle_controller_;                             // 设置当前控制器为初始状态控制器（空闲状态）
        current_state_name_ = kIdle;                                        // 设置当前状态名称为初始状态名称（空闲状态）
        next_state_name_ = kIdle;                                           // 设置下一个状态名称为初始状态名称（空闲状态）
   
        std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));               //for safety 

        // 启动机器人接口和用户命令接口，并进入当前控制器的状态
        ri_ptr_->Start();                                                   // 启动机器人接口，开始与机器人硬件或仿真环境进行交互          
        uc_ptr_->Start();                                                   // 启动用户命令接口，开始接收用户输入命令
        current_controller_->OnEnter();                                     // 进入当前控制器的状态，执行状态进入逻辑
    }
    ~StateMachine(){}

    //------------------------------------------------------------------------------------
    // @brief 状态机运行函数：循环执行当前状态的运行逻辑，并根据用户命令和状态切换条件进行状态切换
    //------------------------------------------------------------------------------------
    void Run()
    {
        double time_record = 0;                                      // 记录上一次状态机运行的时间戳，用于判断是否需要执行状态切换逻辑
        while(true)
        {
            // 如果当前时间戳与上一次记录的时间戳不同，说明有新的状态数据需要处理
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){             // 如果当前时间戳与上一次记录的时间戳不同，说明有新的状态数据需要处理
                time_record = ri_ptr_->GetInterfaceTimeStamp();             // 更新记录的时间戳为当前时间戳
                current_controller_ -> Run();                               // 执行当前控制器的运行逻辑，处理机器人状态和控制数据
                
                // 状态切换逻辑：根据当前控制器的失控判断和下一个状态名称获取下一个状态对象指针
                if(current_controller_->LoseControlJudge()) {               // 如果当前控制器判断机器人失控，则切换到关节阻尼状态
                    next_state_name_ = StateName::kJointDamping;
                }
                else {                                                      // 如果当前控制器没有判断机器人失控，则获取下一个状态名称
                    next_state_name_ = current_controller_ -> GetNextStateName();
                }
                
                // 如果下一个状态名称与当前状态名称不同，则执行状态切换逻辑
                if(next_state_name_ != current_state_name_)
                {
                    std::shared_ptr<StateBase> next_controller = GetNextStatePtr(next_state_name_);
                    if (!next_controller) {
                        // 例如本机没有 ONNX 模型时，取消 RL 请求并留在当前状态
                        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
                        next_state_name_ = current_state_name_;
                    } else {
                        current_controller_->OnExit();                      // 执行当前控制器的状态退出逻辑
                        std::cout << current_controller_->state_name_ << " ------------> ";
                        current_controller_ = next_controller;              // 切换到目标状态控制器
                        std::cout << current_controller_->state_name_ << std::endl;
                        current_controller_->OnEnter();                     // 执行目标状态的进入逻辑
                        current_state_name_ = next_state_name_;             // 更新当前状态名称
                    }
                }

                // 插入数据流传输对象中的数据，用于实时传输和记录机器人状态和控制数据
                this->GetDataStreaming();
            }

            // 为了避免状态机运行过快，导致 CPU 占用过高，增加一个微秒级的睡眠时间
            std::this_thread::sleep_for(std::chrono::microseconds(500));    // 500 微秒（0.5 ms）睡眠，降低 CPU 占用率
        }

    }

};
