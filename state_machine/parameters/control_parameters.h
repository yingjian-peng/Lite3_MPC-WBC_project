/*===============================================================================================
 * @file control_parameters.h
 * @brief basic control parameters
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的机器人类型、向量类型和标准库接口
 ===============================================================================================*/
#include "common_types.h"
#include "custom_types.h"



// 引入工程中的常用命名空间，避免反复书写命名空间前缀
using namespace types;



//------------------------------------------------------------------------------------
// @brief 机器人控制参数集合，包含机器人几何、关节约束和控制器配置
//------------------------------------------------------------------------------------
class ControlParameters
{
private:
    void GenerateLite3Parameters();                     // 生成 Lite3 机器人默认参数

public:
    ControlParameters(RobotType robot_type)             // 构造时根据机器人类型生成默认参数
    {
        if(robot_type==RobotType::Lite3) {
            GenerateLite3Parameters();
        }
        else{
            std::cerr << "Not Default Robot" << std::endl;
        }
    }
    ~ControlParameters(){}

    float body_len_x_, body_len_y_;                     // 机器人躯干长度和宽度 [m]
    float hip_len_, thigh_len_, shank_len_;             // 机器人髋关节、股骨和小腿长度 [m]
    float pre_height_, stand_height_;                   // 机器人预设站立高度和实际站立高度 [m]
    Vec3f swing_leg_kp_, swing_leg_kd_;                 // 摆动腿关节位置环和速度环增益，按 [hip,thigh,shank] 顺序排列
    Vec3f fl_joint_lower_, fl_joint_upper_;             // 左前腿关节角度下限和上限，按 [hip,thigh,shank] 顺序排列 [rad]
    Vec3f joint_vel_limit_;                             // 关节角速度限制，按 [hip,thigh,shank] 顺序排列 [rad/s]
    Vec3f torque_limit_;                                // 关节力矩限制，按 [hip,thigh,shank] 顺序排列 [Nm]
    float stand_duration_ = 1.5;                        // 站立状态持续时间 [s]
    std::string common_policy_path_;                    // 通用策略路径
    Vec3f common_policy_p_gain_, common_policy_d_gain_; // 通用策略关节位置环和速度环增益，按 [hip,thigh,shank] 顺序排列
};



