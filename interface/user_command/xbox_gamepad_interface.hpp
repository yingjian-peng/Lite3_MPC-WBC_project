#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define JOYSTICK_DEADZONE 0.1

using namespace interface;
using namespace types;

class XboxGamepadInterface : public UserCommandInterface
{
    // 支持飞智沙漠狐有线 XInput 模式（045e:028e，由 Linux xpad 识别为 Xbox 360 手柄）。
    // 按键映射：A 站立，B 躺下，X 选 RL，Y 选 MPC，LB 阻尼。
private:
    UserCommand usr_cmd_;
    bool start_thread_flag_;
    std::thread gp_thread_;
    int js_fd_;
    std::mutex mtx_;

    struct GamepadState {
        float left_axis_x = 0.0f;
        float left_axis_y = 0.0f;
        float right_axis_x = 0.0f;
        bool a_pressed = false;
        bool b_pressed = false;
        bool lb_pressed = false;
        bool x_pressed = false;
        bool y_pressed = false;
    } gp_state_;

    float ApplyDeadzone(float value, float deadzone = JOYSTICK_DEADZONE){
        value = std::max(-1.0f, std::min(1.0f, value));
        if(std::abs(value) < deadzone) return 0.0f;
        const float scaled = (value - std::copysign(deadzone, value)) / (1.0f - deadzone);
        return std::max(-1.0f, std::min(1.0f, scaled));
    }

public:
    XboxGamepadInterface(const std::string& js_device = "/dev/input/js0"){
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
        js_fd_ = -1;
        std::cout << "Using USB Gamepad Command Interface" << std::endl;
        std::cout << "Opening device: " << js_device << std::endl;
        
        js_fd_ = open(js_device.c_str(), O_RDONLY | O_NONBLOCK);
        if(js_fd_ < 0){
            std::cerr << "Failed to open gamepad device: " << js_device << std::endl;
            std::cerr << "Make sure the gamepad is connected and you have permission to access it." << std::endl;
        } else {
            std::cout << "Gamepad opened successfully!" << std::endl;
        }
    }

    ~XboxGamepadInterface(){
        if(js_fd_ >= 0) close(js_fd_);
    }

    virtual void Start(){
        if(js_fd_ < 0){
            std::cerr << "Cannot start gamepad thread: device not opened" << std::endl;
            return;
        }
        start_thread_flag_ = true;
        gp_thread_ = std::thread(std::bind(&XboxGamepadInterface::Run, this));
    }

    virtual void Stop(){
        start_thread_flag_ = false;
        if(gp_thread_.joinable()){
            gp_thread_.join();
        }
    }

    virtual UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return usr_cmd_;
    }

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        std::lock_guard<std::mutex> lock(mtx_);
        msfb_ = msfb;
        usr_cmd_.target_mode = msfb.current_state;
    }

    void Run(){
        js_event js_ev;
        bool a_prev = false;
        bool b_prev = false;
        bool lb_prev = false;
        bool x_prev = false;
        bool y_prev = false;

        std::cout << "Start Gamepad Listening" << std::endl;
        
        while (start_thread_flag_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            while(read(js_fd_, &js_ev, sizeof(js_event)) == sizeof(js_event)){
                std::lock_guard<std::mutex> lock(mtx_);
                
                switch(js_ev.type & ~JS_EVENT_INIT){
                    case JS_EVENT_AXIS:
                        switch(js_ev.number){
                            case 0: // Left stick X
                                gp_state_.left_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                break;
                            case 1: // Left stick Y
                                gp_state_.left_axis_y = ApplyDeadzone(js_ev.value / 32767.0f);
                                break;
                            case 3: // 右摇杆横轴：左右拨动控制转向
                                gp_state_.right_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                break;
                            case 4: // 右摇杆纵轴：当前不绑定控制命令
                            case 2: // LT；本控制方案不使用扳机键
                            case 5: // RT；本控制方案不使用扳机键
                                break;
                        }
                        break;
                        
                    case JS_EVENT_BUTTON:
                        switch(js_ev.number){
                            case 0: // A
                                gp_state_.a_pressed = (js_ev.value != 0);
                                break;
                            case 1: // B
                                gp_state_.b_pressed = (js_ev.value != 0);
                                break;
                            case 2: // X
                                gp_state_.x_pressed = (js_ev.value != 0);
                                break;
                            case 3: // Y
                                gp_state_.y_pressed = (js_ev.value != 0);
                                break;
                            case 4: // LB (Left Bumper)
                                gp_state_.lb_pressed = (js_ev.value != 0);
                                break;
                        }
                        break;
                }
            }
            
            std::lock_guard<std::mutex> lock(mtx_);

            // 三个控制轴始终跟随摇杆；回中进入死区后立即归零，满杆归一化为 ±1。
            usr_cmd_.forward_vel_scale = -gp_state_.left_axis_y;
            // 手柄横轴左负右正；机器人侧向速度左正右负，与键盘 a/d 保持一致。
            usr_cmd_.side_vel_scale = -gp_state_.left_axis_x;
            usr_cmd_.turnning_vel_scale = -gp_state_.right_axis_x;
            
            switch(msfb_.current_state) {
                case RobotMotionState::WaitingForStand:
                    if(gp_state_.a_pressed && !a_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        std::cout << "[Gamepad] A pressed -> StandingUp" << std::endl;
                    }
                    break;
                    
                case RobotMotionState::StandingUp:
                    if(gp_state_.x_pressed && !x_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                        std::cout << "[Gamepad] X pressed -> RLControlMode" << std::endl;
                    }else if(gp_state_.y_pressed && !y_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::MPCControlMode);
                        std::cout << "[Gamepad] Y pressed -> MPCControlMode" << std::endl;
                    }else if(gp_state_.b_pressed && !b_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        std::cout << "[Gamepad] B pressed -> LieDown" << std::endl;
                    }
                    break;
                    
                case RobotMotionState::RLControlMode:
                case RobotMotionState::MPCControlMode: {
                    if(gp_state_.lb_pressed && !lb_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
                        std::cout << "[Gamepad] LB pressed -> JointDamping" << std::endl;
                    }else if(gp_state_.b_pressed && !b_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        std::cout << "[Gamepad] B pressed -> LieDown" << std::endl;
                    }

                    break;
                }

                case RobotMotionState::JointDamping:
                    if(gp_state_.a_pressed && !a_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        std::cout << "[Gamepad] A pressed -> StandingUp" << std::endl;
                    }
                    break;
                    
                default:
                    break;
            }
            
            a_prev = gp_state_.a_pressed;
            b_prev = gp_state_.b_pressed;
            lb_prev = gp_state_.lb_pressed;
            x_prev = gp_state_.x_pressed;
            y_prev = gp_state_.y_pressed;
        }
        
        std::cout << "Gamepad thread stopped" << std::endl;
    }
};
