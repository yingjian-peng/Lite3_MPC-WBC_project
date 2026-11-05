/*===============================================================================================
 * @file state_base.h
 * @brief 状态控制基类及各状态共用的数据引用
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * @copyright Copyright (c) 2024 DeepRobotics
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：机器人接口、用户命令、控制参数和数据记录所需类型
 ===============================================================================================*/
#include "common_types.h"
#include "custom_types.h"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "data_streaming.hpp"
#include "basic_function.hpp"
#include "parameters/control_parameters.h"



// 引入工程中的常用命名空间，避免反复书写命名空间前缀
using namespace types;
using namespace interface;
using namespace functions;



//------------------------------------------------------------------------------------
// @brief 控制状态共用的数据集合，由状态机创建并传给各个状态
//------------------------------------------------------------------------------------
struct ControllerData{
    std::shared_ptr<RobotInterface> ri_ptr;             // 机器人接口：读取状态并发送关节命令
    std::shared_ptr<UserCommandInterface> uc_ptr;       // 用户命令接口：读取按键或其他控制命令
    std::shared_ptr<ControlParameters> cp_ptr;          // 控制参数：保存机器人和控制器配置
    std::shared_ptr<DataStreaming> ds_ptr;              // 数据流接口：整理并发送或记录运行数据
};



//------------------------------------------------------------------------------------
// @brief 所有机器人控制状态的抽象基类
// @details 每个具体状态实现进入、退出、运行、安全检查和下一状态选择逻辑。
//------------------------------------------------------------------------------------
class StateBase{
private:

public:
    // 保存状态机创建的公共数据，并从中取得各接口和参数对象
    // 构造时同时清零所有状态共用的运动状态反馈
    StateBase(const RobotType& robot_type, 
        const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):
        robot_type_(robot_type),
        state_name_(state_name),
        data_ptr_(data_ptr)
    {
            ri_ptr_ = data_ptr_->ri_ptr;
            uc_ptr_ = data_ptr_->uc_ptr;
            cp_ptr_ = data_ptr_->cp_ptr;
            ds_ptr_ = data_ptr_->ds_ptr;
            std::memset(&msfb_, 0, sizeof(msfb_));
    }
    ~StateBase(){}

    virtual void OnEnter() = 0;                         // 状态进入回调
    virtual void OnExit() = 0;                          // 状态退出回调
    virtual void Run() = 0;                             // 状态机每次循环调用 Run() 来执行该状态的运行逻辑
    virtual bool LoseControlJudge() = 0;                // 状态机在每次 Run() 后调用 LoseControlJudge() 来判断当前状态是否安全
    virtual StateName GetNextStateName() = 0;           // 安全检查通过后，状态机通过此函数获取状态逻辑建议的下一状态

    std::string state_name_;                            // 状态名称，用于状态切换日志输出
    RobotType robot_type_;                              // 当前机器人类型

    std::shared_ptr<ControllerData> data_ptr_;         // 共用数据集合
    std::shared_ptr<RobotInterface> ri_ptr_;            // 读取机器人状态并发送关节命令
    std::shared_ptr<UserCommandInterface> uc_ptr_;      // 读取用户命令
    std::shared_ptr<ControlParameters> cp_ptr_;         // 读取控制参数
    std::shared_ptr<DataStreaming> ds_ptr_;             // 写入运行数据或进行数据传输

    static MotionStateFeedback msfb_;                   // 所有状态共享的当前运动状态反馈
};





