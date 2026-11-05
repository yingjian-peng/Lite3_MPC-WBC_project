/*===============================================================================================
 * @file 07_mpc_wbc_controller.hpp
 * @brief Lite3 MPC+WBC 整机控制计算、结果缓存和 MPC 工作线程
 * @details 本文件组织 00～06 模块，不持有 RobotInterface，也不直接发送关节命令。
 * 状态机接入见 state_machine/mpc_control_state.hpp：读接口→填 ControllerInput→发布 MPC 请求→
 * 读取最近的 MpcWrench→Step(in,wrench)→由状态机发送 ControllerOutput::joint_cmd。
 * 快速控制线程：根据新仿真状态更新步态、估计器、WBC 和关节命令，通常约 1 kHz。
 * MPC 工作线程：读取请求快照，求解 SRBD-MPC，写回基座加速度/接触力参考；默认休眠间隔 0.05 s。
 * Step(in,wrench) 使用工作线程发布的 MPC 参考，不在快速控制线程中求解 MPC。
 * 输出列顺序：[kp, q_des, kd, dq_des, tau_ff]，共 12×5；由仿真端执行 PD+前馈并限幅。
 * 默认 WBC 有效时 tau_ff=tau_wbc-PD；只有前馈未饱和且额外 PD 关闭时，总力矩才等于 tau_wbc。
 * 故障只在此处置标志，由外层状态机切换到阻尼；起立动作由外层站立状态负责。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include <algorithm>                                // 最值、夹取和排序等标准算法
#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <atomic>                                   // 线程运行标志和快照序列号
#include <chrono>                                   // 单调时钟、求解耗时和线程休眠时间
#include <cmath>                                    // 三角函数、绝对值和其他数学函数
#include <thread>                                   // MPC 工作线程与周期休眠
#include <vector>                                   // 随支撑数量或预测时域变化的动态数组

#include "00_lite3_model.hpp"                       // 统一坐标/关节约定及 Lite3 运动学、动力学模型
#include "01_state_estimator.hpp"                   // 基座速度、高度和状态有效性估计
#include "02_gait_trajectory.hpp"                   // 步态相位、速度分段和足端轨迹
#include "05_srbd_mpc.hpp"                          // 单刚体预测控制的输入输出和求解接口
#include "06_weighted_wbc.hpp"                      // 全身动力学 QP 和基座加速度参考换算



namespace mpc_wbc {

constexpr int kMaxContactPreview = 64;  // 支撑表一次最多向前看多少节点（须 ≥ SrbdMpcParams::N）



//===============================================================================================
// @brief 整机接管、命令约束、低层执行和子模块配置
//===============================================================================================
struct MpcWbcControllerParams 
{
    // 参数组 1：平滑接管与站立高度；机器人应在进入 MPC 状态前完成起立。
    Vec12d stand_q = Lite3Model::DefaultStandJointAngles();     // 接管后的 12 维站姿关节参考 [rad]
    double stand_ramp_time = 0.30;                              // 从进入时关节角平滑接管到 stand_q 的时长 [s]
    double stand_settle_time = 0.20;                            // 接管斜坡后继续保持位置环的时间 [s]
    double com_height = 0.0;                                    // 期望质心高度 [m]；非正时由站姿几何自动计算

    // 参数组 2：控制时序与 MPC 结果有效期。
    double mpc_period = 0.05;                                   // MPC 工作线程休眠间隔 [s]，默认约 20 Hz
    double max_dt = 0.05;                                       // 用于命令平滑的最大合法帧间隔 [s]，超限则该次 dt 取 0
    double wrench_max_age = 0.15;                               // 允许使用的 MPC 参考快照最大年龄 [s]

    // 参数组 3：命令低通及变化率限幅，减小手柄/键盘阶跃带来的冲击。
    double cmd_tau = 0.25;                                      // 速度命令一阶低通时间常数 [s]；非正时关闭低通
    double cmd_slew_v = 0.8;                                    // 线速度命令各轴最大变化率 [m/s²]；非正时关闭该限幅
    double cmd_slew_w = 2.0;                                    // 转向命令最大变化率 [rad/s²]；非正时关闭该限幅

    // 参数组 4：多轴组合约束；超限时三轴同比例缩小，保留相对方向。
    double combo_wz_ratio = 0.18;                               // 组合约束中把转向速率换算为等效线速度的系数
    double combo_max = 0.67;                                    // hypot(vx,vy)+ratio*abs(wz) 上限；非正时关闭组合约束

    // 参数组 5：保留的可选平方约束；当前主线通过 combo_turn_loss=0 关闭。
    double combo_turn_gain = 1.7;                               // 可选 vxy*wz² 约束的缩放系数，主线不启用该约束
    double combo_turn_loss = 0.0;                               // 可选 vxy*wz² 约束预算；非正时关闭
    int combo_priority = 0;                                     // 可选约束超限策略：0=三轴同比缩小，1=保转向，2=保平移

    // 参数组 6：单轴物理上限，位于偏置补偿和组合约束之前。
    double cmd_max_vx = 0.60;                                   // 前后单轴物理速度硬上限 [m/s]；非正时不作该轴限制
    double cmd_max_vy = 0.50;                                   // 横移单轴物理速度硬上限 [m/s]
    double cmd_max_wz = 1.00;                                   // 转向单轴物理速率硬上限 [rad/s]

    // 参数组 7：步幅反馈及偏置补偿，默认只启用共用命令补偿 cmd_vel_trim。
    double gait_vel_fb = 0.0;                                   // 步幅线速度误差反馈增益；0=关闭，不直接修改 MPC 参考
    double gait_yaw_fb = 0.0;                                   // 步幅偏航速度反馈增益；0=关闭
    double gait_vel_trim = 0.0;                                 // 仅对步幅命令沿平移方向减小的速度量 [m/s]；0=关闭
    double gait_cmd_limit_scale = 1.3;                          // 可选步幅反馈结果相对调度表全局上限的最大倍率
    double cmd_vel_trim = 0.04;                                 // 步态与 MPC 共用的平移命令补偿 [m/s]；当前 MuJoCo 标定值
    double mpc_vel_trim = 0.0;                                  // 仅对 MPC 请求减小的平移速度量 [m/s]；0=关闭
    double com_height_slew = 0.06;                              // 高度参考后续更新的最大变化率 [m/s]；初始化时直接采用目标

    // 参数组 8：低层 PD 与前馈反解；前馈饱和时总力矩可能偏离 WBC 输出。
    double kp_pos = 100.0;                                      // 位置环比例增益，进入前馈反解和下发命令
    double kd_pos = 2.5;                                        // 位置环速度增益
    double kp_scale_wbc = 1.0;                                  // WBC 工作时 kp_pos 的倍率，当前标定值为 1
    double kd_scale_wbc = 1.0;                                  // WBC 工作时 kd_pos 的倍率，当前标定值为 1
    double pd_extra = 0.0;                                      // 在 WBC 总力矩上额外叠加 PD 的比例；默认 0
    bool pd_extra_stance_only = true;                           // 额外 PD 是否只作用于支撑腿，默认开启
    bool swing_joint_vel_ff = false;                            // 是否用足端雅可比把规划足速反解为关节速度参考；默认关闭

    // 参数组 9：故障、WBC 使能及短时失效滞回。
    double tilt_limit = 0.8;                                    // 最大绝对 roll/pitch 故障阈值 [rad]，与 WBC 使能门限不同
    double wbc_enable_tilt = 0.35;                              // 允许新使能 WBC 的姿态门限 [rad]
    int max_consecutive_wbc_fail = 20;                          // WBC 连续求解失败达到该次数时置故障，未使能帧不计数
    int wbc_enable_hold_frames = 40;                            // WBC 已使能后允许估计/姿态门控短时失效的帧数
    bool swing_target_yaw_only = true;                          // 足端机体系目标转世界系时是否只保留偏航旋转

    // 参数组 10：子模块配置；实际速度段由 gait_schedule 选择。
    EstimatorParams estimator;                                  // 01 状态估计配置
    GaitParams gait;                                            // 固定步态配置与初始化参数
    bool use_gait_schedule = true;                              // 是否启用三段步态调度，主线默认开启
    SegmentedGaitParams gait_schedule = SegmentedGaitParams::BaselineClosedLoop();  // 三段速度配置及状态机输入映射使用的全局速度上限
    double nominal_x = -0.0133;                                 // 四腿标称足位的机体系前后平移 [m]，用于静态配平
    SrbdMpcParams mpc;                                          // 05 SRBD-MPC 模型/优化配置
    WbcParams wbc;                                              // 06 全身控制任务/约束/求解配置
};



//===============================================================================================
// @brief 由状态文件从机器人接口和用户命令整理的单帧输入
//===============================================================================================
struct ControllerInput {
    double t = 0.0;                                             // 机器人接口时间戳 [s]
    Vec3d rpy = Vec3d::Zero();                                  // 测量 ZYX 欧拉角 [rad]
    Vec3d omega_body = Vec3d::Zero();                           // 测量机体系角速度 [rad/s]
    Vec3d acc_body = Vec3d::Zero();                             // 测量机体系加速度 [m/s²]，当前估计器不融合
    Vec12d q = Vec12d::Zero();                                  // 12 维实际关节角 [rad]
    Vec12d dq = Vec12d::Zero();                                 // 12 维实际关节角速度 [rad/s]
    GaitCommand cmd;                                            // 机体系物理速度命令 vx/vy [m/s]、wz [rad/s]，已由输入层归一化换算
};



//===============================================================================================
// @brief 快速控制线程发布给 MPC 线程的固定容量输入快照
//===============================================================================================
struct MpcRequest {
    double t = 0.0;                                             // 生成请求时的接口时间戳 [s]
    Vec3d rpy = Vec3d::Zero();                                  // 本帧测量姿态 [rad]
    Vec3d omega_body = Vec3d::Zero();                           // 本帧机体系角速度 [rad/s]
    Vec12d q = Vec12d::Zero();                                  // 本帧 12 个实际关节角 [rad]
    Vec3d base_pos = Vec3d::Zero();                             // 最近估计的基座世界系位置 [m]
    Vec3d base_vel = Vec3d::Zero();                             // 最近估计的基座世界系速度 [m/s]
    Vec3d com_body = Vec3d::Zero();                             // 按本帧关节角计算的整机机体系质心 [m]
    bool est_valid = false;                                     // 生成请求时状态估计是否有效
    int num_support = 0;                                        // 估计器筛选后的一致支撑腿数
    GaitCommand cmd;                                            // 经平滑、收步缩放和可选补偿的物理速度参考
    double yaw_ref = 0.0;                                       // 独立积分的连续偏航参考 [rad]
    double yaw_meas = 0.0;                                      // 测量偏航展开后的连续角度 [rad]
    double height_cmd = 0.0;                                    // 当前使用的质心高度参考 [m]
    std::array<std::array<bool, kNumLegs>, kMaxContactPreview> contact{};  // 固定容量的未来四腿接触表
    int contact_steps = 0;                                      // 本次接触表有效节点数，最多 kMaxContactPreview
};



//===============================================================================================
// @brief MPC 线程返回的参考加速度、逐脚力与求解诊断
//===============================================================================================
struct MpcWrench {
    bool valid = false;                                         // 是否含可供快速线程使用的加速度/力参考
    double t = 0.0;                                             // 该参考对应请求的采样时间 [s]，用于判断年龄
    Vec3d a_des_world = Vec3d::Zero();                          // 世界系基座线加速度参考 [m/s²]
    Vec3d alpha_des_body = Vec3d::Zero();                       // 机体系角加速度参考 [rad/s²]
    std::array<Vec3d, kNumLegs> foot_force_ref{};               // MPC 首节点逐脚世界系力参考 [N]
    Vec3d net_force = Vec3d::Zero();                            // 首节点世界系合力 [N]
    Vec3d net_moment = Vec3d::Zero();                           // 首节点绕质心的世界系合力矩 [N·m]
    int status = -1;                                            // -1=未求解，0=新解，1=MPC 内部复用旧解，2=失败
    int reason = 0;                                             // 失败原因：1=估计无效，2=无支撑，3=MPC 无可用结果
    int iters = 0;                                              // MPC 盒约束 QP 记录的迭代数
    double qp_residual = 0.0;                                   // 盒约束 QP 的投影梯度残差
    double solve_ms = 0.0;                                      // MPC 求解耗时 [ms]
};



//===============================================================================================
// @brief 基于奇偶序列号检查的单写单读快照容器
// @details 写入中序号为奇数，写完为偶数；读端前后核对序号，最多尝试 8 次。
// @details 按当前实现复制 T，读操作返回 false 时调用者应忽略输出；不可改为多个并发写者。
//===============================================================================================
template <typename T>
class SeqLockBox 
{
public:
    //------------------------------------------------------------------------------------
    // @brief 由唯一写线程发布一份快照，先置奇序号、复制数据、再置偶序号
    //------------------------------------------------------------------------------------
    void Write(const T& value) {
        const uint64_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);   // 奇 = 正在写
        std::atomic_thread_fence(std::memory_order_release);
        value_ = value;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_relaxed);   // 偶 = 写完
    }

    //------------------------------------------------------------------------------------
    // @brief 尝试读取序号一致的快照；若连续 8 次受写入干扰则返回 false
    //------------------------------------------------------------------------------------
    bool Read(T* out) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const uint64_t s0 = seq_.load(std::memory_order_relaxed);
            if (s0 & 1U) continue;                       // 写者正在写
            std::atomic_thread_fence(std::memory_order_acquire);
            *out = value_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s0) return true;
        }
        return false;
    }

    //------------------------------------------------------------------------------------
    // @brief 读取快照及其版本号，供工作线程判断是否有尚未处理的新请求
    //------------------------------------------------------------------------------------
    bool ReadWithSeq(T* out, uint64_t* seq_out) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const uint64_t s0 = seq_.load(std::memory_order_relaxed);
            if (s0 & 1U) continue;
            std::atomic_thread_fence(std::memory_order_acquire);
            *out = value_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s0) {
                if (seq_out) *seq_out = s0;
                return true;
            }
        }
        return false;
    }

private:
    std::atomic<uint64_t> seq_{0};                              // 快照版本号；偶数表示复制完成，奇数表示正在写入
    T value_{};                                                 // 当前快照数据，读者需按序号检查结果是否可接受
};



//===============================================================================================
// @brief 整机控制器返回的关节命令、运行状态和诊断数据
//===============================================================================================
struct ControllerOutput 
{
    enum class Mode { kRamp, kStand, kWalk, kFault };

    // 控制输出与状态。
    Eigen::Matrix<float, 12, 5> joint_cmd = Eigen::Matrix<float, 12, 5>::Zero();  // 12×5，列为 [kp,q_des,kd,dq_des,tau_ff]，由状态文件发送
    Mode mode = Mode::kRamp;                                    // 当前控制阶段：接管、站立、行走或故障
    bool wbc_active = false;                                    // 本帧是否取得可用 WBC 结果并用于输出
    bool wbc_stale = false;                                     // 本帧是否接受了 WBC 内部的短时旧解
    bool fault = false;                                         // 控制器故障标志，外层状态机据此切入阻尼

    // 实际目标、力矩与机器人状态诊断。
    double dt = 0.0;                                            // 用于本次命令推进的合法时间间隔 [s]，异常时为 0
    Vec12d q_des = Vec12d::Zero();                              // 本帧关节目标 [rad]，为平滑接管或步态 IK 的结果
    Vec12d tau_wbc = Vec12d::Zero();                            // WBC 关节总力矩 [N·m]
    Vec12d tau_pd = Vec12d::Zero();                             // 按本帧实测 q/dq 和下发增益计算的 PD 力矩 [N·m]
    std::array<Vec3d, kNumLegs> foot_force{};                   // 本帧接受的 WBC 四脚世界系力 [N]
    std::array<bool, kNumLegs> stance{{true, true, true, true}};  // 本帧计划支撑标志
    Vec3d base_pos = Vec3d::Zero();                             // 估计的基座世界系位置 [m]
    Vec3d base_vel = Vec3d::Zero();                             // 估计的基座世界系速度 [m/s]
    Vec3d com_pos = Vec3d::Zero();                              // 按基座估计和当前关节角换算的世界系质心 [m]
    Vec3d a_des = Vec3d::Zero();                                // 本帧真正送入 WBC 的世界系线加速度参考 [m/s²]
    Vec3d alpha_des = Vec3d::Zero();                            // 本帧真正送入 WBC 的机体系角加速度参考 [rad/s²]
    double tilt = 0.0;                                          // max(abs(roll),abs(pitch)) [rad]
    double height = 0.0;                                        // 估计基座高度 [m]
    double est_confidence = 0.0;                                // 状态估计置信度 [0,1]

    // WBC 数值求解诊断。
    double wbc_eom_residual = 0.0;                              // WBC 动力学等式残差
    double wbc_contact_residual = 0.0;                          // WBC 接触/零力诊断残差
    double wbc_cone_violation = 0.0;                            // WBC 摩擦锥约束最大正违反
    double wbc_base_accel_error = 0.0;                          // WBC 基座加速度任务误差
    double wbc_swing_accel_error = 0.0;                         // WBC 摆动脚加速度任务残差
    double tau_abs_max = 0.0;                                   // WBC 关节力矩最大绝对值 [N·m]
    int wbc_iters = 0;                                          // WBC 内点法迭代数

    // 步态和 MPC 结果年龄诊断。
    std::array<double, kNumLegs> swing_des_z{};                 // 摆动脚期望世界系球心高度 [m]，支撑腿记 0
    std::array<double, kNumLegs> swing_now_z{};                 // 摆动脚实际世界系球心高度 [m]
    int stance_count = 0;                                       // 本帧进入 WBC 时的计划支撑腿数
    bool mpc_fresh = false;                                     // 快照 valid 且所用参考年龄小于 2*mpc_period 的诊断标志
    int mpc_status = -1;                                        // 本帧输入快照的求解状态；无可用参考时为 -1，应结合 mpc_held 判断
    int mpc_reason = 0;                                         // 参考不可用原因：1=估计无效，2=无支撑，3=无解，4=过期
    bool mpc_held = false;                                      // 是否沿用整机控制器缓存的最近一次有效参考
    double mpc_wrench_age = 0.0;                                // 本帧实际采用的 MPC 参考年龄 [s]
    double mpc_net_fz = 0.0;                                    // 所用 MPC 合力的世界系竖直分量 [N]
    int mpc_iters = 0;                                          // 当前输入快照记录的 MPC 迭代数
    double mpc_qp_residual = 0.0;                               // 当前输入快照记录的 MPC QP 残差
    double wbc_ms = 0.0;                                        // 本帧接受结果中记录的 WBC 求解耗时 [ms]
    double mpc_ms = 0.0;                                        // 当前输入快照记录的 MPC 求解耗时 [ms]
    double step_ms = 0.0;                                       // 本次快速控制计算耗时 [ms]
};



//===============================================================================================
// @brief 组织快速控制链和 MPC 求解接口，隔离算法计算与外部状态机/机器人 IO
//===============================================================================================
class MpcWbcController 
{
public:
    //-----------------------------------------------------------------------------------
    // @brief 构造模型、估计器、步态、MPC 与 WBC，并计算站姿几何
    //------------------------------------------------------------------------------------
    explicit MpcWbcController(const MpcWbcControllerParams& params = MpcWbcControllerParams())
        : params_(params), model_(), estimator_(params.estimator), gait_(params.gait),
          mpc_(params.mpc), wbc_(params.wbc) {
        ApplyGaitConfig();
        ComputeStandGeometry();
    }

    //------------------------------------------------------------------------------------
    // @brief 读取整机控制参数，供状态文件映射命令及配置工作线程
    //------------------------------------------------------------------------------------
    const MpcWbcControllerParams& params() const { return params_; }

    //------------------------------------------------------------------------------------
    // @brief 读取快速控制线程最近一次更新的状态估计
    //------------------------------------------------------------------------------------
    const StateEstimate& estimate() const { return estimate_; }

    //------------------------------------------------------------------------------------
    // @brief 读取本帧保存的 MPC 输入参考快照
    //------------------------------------------------------------------------------------
    const MpcWrench& wrench() const { return wrench_in_; }

    //------------------------------------------------------------------------------------
    // @brief 替换整机与子模块配置，并重新应用步态配平和站姿几何
    // @details 应在 MPC 工作线程停止后调用；重新构造估计器，其他运行缓存需要时另行 Reset。
    //------------------------------------------------------------------------------------
    void set_params(const MpcWbcControllerParams& p) {
        params_ = p;
        estimator_ = StateEstimator(p.estimator);
        gait_.set_params(p.gait);
        ApplyGaitConfig();
        mpc_.set_params(p.mpc);
        wbc_.set_params(p.wbc);
        ComputeStandGeometry();
    }

    //------------------------------------------------------------------------------------
    // @brief 先对速度做一阶低通，再按各轴变化率限幅
    // @param dt 合法控制时间间隔 [s]；dt<=0 时直接保留已有滤波状态，不做限幅或推进。
    //------------------------------------------------------------------------------------
    GaitCommand SmoothCommand(const GaitCommand& target, double dt) 
    {
        const MpcWbcControllerParams& p = params_;

        // 时间未前进时保留滤波状态，避免首次调用直接采用满幅目标。
        if (dt <= 0.0) return cmd_filtered_;
        auto slew = [&](double cur, double tgt, double rate) {
            if (rate <= 0.0) return tgt;
            const double max_step = rate * dt;
            return cur + std::max(-max_step, std::min(max_step, tgt - cur));
        };

        // 按当前滤波值先计算低通候选，再以同一当前值对候选的单帧变化量限幅。
        GaitCommand lp = target;
        if (p.cmd_tau > 0.0) {
            const double a = std::min(1.0, dt / p.cmd_tau);
            lp.vx = cmd_filtered_.vx + a * (target.vx - cmd_filtered_.vx);
            lp.vy = cmd_filtered_.vy + a * (target.vy - cmd_filtered_.vy);
            lp.wz = cmd_filtered_.wz + a * (target.wz - cmd_filtered_.wz);
        }
        cmd_filtered_.vx = slew(cmd_filtered_.vx, lp.vx, p.cmd_slew_v);
        cmd_filtered_.vy = slew(cmd_filtered_.vy, lp.vy, p.cmd_slew_v);
        cmd_filtered_.wz = slew(cmd_filtered_.wz, lp.wz, p.cmd_slew_w);
        return cmd_filtered_;
    }

    //------------------------------------------------------------------------------------
    // @brief 进入 MPC 状态前清空运行状态，并记录当前关节角用于平滑接管
    // @details 应在工作线程启动前执行；清空故障、收步、估计、WBC 旧结果和 MPC 参考缓存。
    // @details 偏航参考在后续请求中从实际测量初始化，水平位置原点由估计器重新设置。
    //------------------------------------------------------------------------------------
    void Reset(double t, const Vec12d& q_now) 
    {
        t_enter_ = t;
        last_t_ = t;
        q_start_ = q_now;
        q_des_ = q_now;
        cmd_filtered_ = GaitCommand();
        yaw_ref_ = 0.0;              // 清空缓存，首次请求会从实际偏航初始化参考
        yaw_ref_initialized_ = false;
        yaw_meas_cont_ = 0.0;
        yaw_meas_prev_ = 0.0;
        yaw_meas_initialized_ = false;
        height_cmd_ = 0.0;
        height_cmd_initialized_ = false;
        last_dt_ = 0.0;
        dq_des_.setZero();
        gait_.Reset(t);
        wbc_.Reset();
        estimator_.Reset(t);   // 重置水平位置原点；高度由后续支撑足运动学恢复
        estimate_ = StateEstimate();
        wrench_in_ = MpcWrench();
        last_good_wrench_ = MpcWrench();
        a_des_.setZero();
        alpha_des_.setZero();
        wbc_active_ = false;
        wbc_fail_count_ = 0;
        invalid_frames_ = 0;
        fault_ = false;
    }

    //------------------------------------------------------------------------------------
    // @brief 快速控制主入口：使用外部 MPC 参考计算本帧关节命令
    // @details 由状态文件调用；内部不求解 MPC、不启停线程，也不读写机器人接口。
    // @return 输出包含 WBC/位置环命令和故障标志，由状态文件负责发送及切换状态。
    //------------------------------------------------------------------------------------
    ControllerOutput Step(const ControllerInput& in, const MpcWrench& wrench) {
        wrench_in_ = wrench;
        return StepImpl(in);
    }

    //------------------------------------------------------------------------------------
    // @brief 在快速控制线程生成供 MPC 求解使用的独立请求快照
    // @details 不求解 QP；除复制测量/缓存外，还计算质心、更新连续偏航与高度参考，并预测接触计划。
    // @details 主线在本帧 Step 之前调用，因此估计器和滤波命令缓存通常来自上一控制帧。
    //------------------------------------------------------------------------------------
    MpcRequest BuildMpcRequest(const ControllerInput& in) 
    {
        // 步骤 1：复制当前测量、最近估计和按当前关节角计算的质心偏置。
        MpcRequest req;
        req.t = in.t;
        req.rpy = in.rpy;
        req.omega_body = in.omega_body;
        req.q = in.q;
        req.base_pos = estimate_.base_pos_w;
        req.base_vel = estimate_.base_vel_w;
        req.com_body = model_.CompositeComBody(in.q);
        req.est_valid = estimate_.valid;
        req.num_support = estimate_.num_support;

        // 步骤 2：使用最近滤波命令，并按步态收步系数同步缩小 MPC 速度参考。
        req.cmd = cmd_filtered_;
        // 软停步期间把 MPC 的速度/偏航参考按同一条步幅曲线缩下来：参考与足端规划
        // 同源，停步时才不会「参考已归零、腿还在扫」而互相顶。
        {
            const double ps = gait_.plan_scale();
            if (ps < 1.0) {
                req.cmd.vx *= ps;
                req.cmd.vy *= ps;
                req.cmd.wz *= ps;
            }
        }

        // 步骤 3：可选地仅对 MPC 平移参考减小幅值，每次最多削减一半，保持方向。
        if (params_.mpc_vel_trim > 0.0) {
            const double vm = std::hypot(req.cmd.vx, req.cmd.vy);
            if (vm > 1e-6) {
                // 最多修正一半，避免很小的指令被削成 0（那会变成「原地踏步但报运动指令」）。
                const double trim = std::min(params_.mpc_vel_trim, 0.5 * vm);
                const double k = (vm - trim) / vm;
                req.cmd.vx *= k;
                req.cmd.vy *= k;
            }
        }

        // 步骤 4：把测量偏航展开为连续角度，独立积分偏航参考，避免随测量漂移。
        if (!yaw_meas_initialized_) {
            yaw_meas_cont_ = in.rpy.z();
            yaw_meas_prev_ = in.rpy.z();
            yaw_meas_initialized_ = true;
        } else {
            yaw_meas_cont_ += WrapPi(in.rpy.z() - yaw_meas_prev_);
            yaw_meas_prev_ = in.rpy.z();
        }
        if (!yaw_ref_initialized_) {
            yaw_ref_ = yaw_meas_cont_;
            yaw_ref_initialized_ = true;
        } else {
            const double d = in.t - last_yaw_ref_t_;
            // 停步收步期间按同一条步幅曲线缩小偏航参考的推进速度，避免参考跑到机身前面。
            const double ps = gait_.plan_scale();
            if (d > 0.0 && d < 0.1) yaw_ref_ += ps * cmd_filtered_.wz * d;
        }
        last_yaw_ref_t_ = in.t;
        req.yaw_ref = yaw_ref_;
        req.yaw_meas = yaw_meas_cont_;

        // 步骤 5：更新高度参考；首次或估计高度很小时直接采用目标，其余时刻按配置限速。
        const double com_now = estimate_.base_pos_w.z() + model_.CompositeComBody(in.q).z();
        if (!height_cmd_initialized_ || com_now <= 0.05) {
            height_cmd_ = params_.com_height;   // 还没站起来：直接用目标高度
            height_cmd_initialized_ = true;
        } else if (params_.com_height_slew <= 0.0) {
            height_cmd_ = params_.com_height;
        } else {
            const double max_step = params_.com_height_slew *
                                    (last_dt_ > 0.0 ? last_dt_ : params_.mpc_period);
            const double err = params_.com_height - height_cmd_;
            height_cmd_ += std::max(-max_step, std::min(max_step, err));
        }
        req.height_cmd = height_cmd_;

        // 步骤 6：从步态模块取未来接触表，将有效长度限制在固定快照容量以内。
        const int N = std::max(1, std::min(kMaxContactPreview, params_.mpc.N));
        const auto sched = gait_.ContactSchedule<kMaxContactPreview>(in.t, params_.mpc.dt);
        for (int k = 0; k < N; ++k) req.contact[k] = sched[k];
        req.contact_steps = N;
        return req;
    }

    //------------------------------------------------------------------------------------
    // @brief 由 MPC 线程根据请求构造单刚体输入，求解后换算基座参考
    // @details 只读取请求快照和稳定配置，更新 MPC 自身求解缓存；同一控制器不能并发调用此函数。
    // @return 失败时返回 valid=false 及原因；快速控制链独立决定是否沿用未过期的有效缓存。
    //------------------------------------------------------------------------------------
    MpcWrench SolveWrench(const MpcRequest& req) 
    {
        // 步骤 1：初始化返回状态；估计无效或无支撑时不执行 MPC 求解。
        MpcWrench w;
        w.t = req.t;
        if (!req.est_valid) {
            w.status = 2;
            w.reason = 1;
            return w;  // 返回无效快照，由 StepImpl 检查独立的最近有效参考缓存
        }
        if (req.num_support < 1) {
            w.status = 2;
            w.reason = 2;
            return w;
        }

        // 步骤 2：由快照的基座和质心偏置构造 MPC 状态、惯量及世界系接触作用点。
        const Mat3d R = RpyToRotation(req.rpy);
        SrbdMpcInput mpc_in;
        const Vec3d com_world = req.base_pos + R * req.com_body;
        mpc_in.p_com = com_world;
        mpc_in.v_com = req.base_vel + req.omega_body.cross(com_world - req.base_pos);
        mpc_in.rpy = req.rpy;
        mpc_in.R_wb = R;
        // 偏航参考与测量都已是**连续不归一化**的角度（见 BuildMpcRequest 注释），
        // 这里直接相减即可，不需要任何 WrapPi 对齐。
        mpc_in.rpy.z() = req.yaw_meas;
        mpc_in.yaw_ref = req.yaw_ref;
        mpc_in.omega = req.omega_body;
        mpc_in.inertia_com_world = model_.CompositeInertiaWorld(req.q, R, req.com_body);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d foot_body = model_.FootPositionBody(leg, req.q.segment<3>(3 * leg));
            mpc_in.foot_pos_world[leg] =
                req.base_pos + R * foot_body + Vec3d(0.0, 0.0, -model_.geo.foot_radius);
        }

        // 步骤 3：将机体系水平速度参考按实际偏航转到世界系，填入高度和接触计划。
        const Mat3d R_yaw = YawOnly(R);
        mpc_in.v_cmd = R_yaw * Vec3d(req.cmd.vx, req.cmd.vy, 0.0);
        mpc_in.yaw_rate_cmd = req.cmd.wz;
        mpc_in.height_cmd = req.height_cmd;
        mpc_in.contact.assign(req.contact.begin(), req.contact.begin() + req.contact_steps);

        // 步骤 4：执行单刚体 MPC，记录求解状态；失败时保留返回值 valid=false。
        const SrbdMpcOutput out = mpc_.Solve(req.t, mpc_in);
        w.status = static_cast<int>(out.status);
        w.iters = out.iters;
        w.qp_residual = out.qp_residual;
        w.solve_ms = out.solve_ms;
        if (!out.ok()) {
            w.reason = 3;
            return w;  // 无可用解；是否沿用未过期缓存由 StepImpl 决定
        }

        // 步骤 5：保存可用接触力，并换算 WBC 所需的线/角加速度参考。
        w.valid = true;
        w.foot_force_ref = out.foot_force;
        w.net_force = out.net_force;
        w.net_moment = out.net_moment;
        ComputeBaseAccelDesired(out, mpc_in.inertia_com_world, req.omega_body, R,
                                model_.TotalMass(), params_.mpc.gravity, &w.a_des_world,
                                &w.alpha_des_body);
        return w;
    }

    //------------------------------------------------------------------------------------
    // @brief 读取按水平站立几何计算的整机质心高度 [m]
    //------------------------------------------------------------------------------------
    double stand_com_height() const { return stand_com_height_; }

    //------------------------------------------------------------------------------------
    // @brief 读取默认球足触地时基座原点的站立高度 [m]
    //------------------------------------------------------------------------------------
    double stand_base_height() const { return stand_base_height_; }

    //------------------------------------------------------------------------------------
    // @brief 读取步态和 MPC 共用的滤波后速度命令
    //------------------------------------------------------------------------------------
    const GaitCommand& smoothed_command() const { return cmd_filtered_; }

    //------------------------------------------------------------------------------------
    // @brief 读取锁存的控制器故障，由外层状态机据此切入阻尼
    //------------------------------------------------------------------------------------
    bool fault() const { return fault_; }




private:
    //------------------------------------------------------------------------------------
    // @brief 执行一帧速度约束、步态、估计、WBC 和关节命令计算
    // @details 本函数维护快速线程状态，不负责状态机切换；出现故障时只返回标志和兜底命令。
    //------------------------------------------------------------------------------------
    ControllerOutput StepImpl(const ControllerInput& in) 
    {
        // 步骤 1：开始计时并计算合法 dt、当前姿态和接管阶段。
        const auto t0 = std::chrono::steady_clock::now();
        ControllerOutput out;
        const MpcWbcControllerParams& p = params_;

        // dt 仅用于本帧滤波/限速等时间推进；异常时置 0，子模块仍按绝对时间戳检查输入。
        double dt = in.t - last_t_;
        if (!(dt > 0.0) || dt > p.max_dt) dt = 0.0;
        out.dt = dt;

        const Mat3d R = RpyToRotation(in.rpy);
        out.base_pos = estimate_.base_pos_w;
        out.base_vel = estimate_.base_vel_w;
        out.height = estimate_.height;
        out.est_confidence = estimate_.confidence;
        out.tilt = std::max(std::fabs(in.rpy(0)), std::fabs(in.rpy(1)));

        // ---- 阶段判定 ----
        const double rel = in.t - t_enter_;
        const bool ramping = rel < p.stand_ramp_time;
        const bool settling = !ramping && rel < p.stand_ramp_time + p.stand_settle_time;

        // 步骤 2：按单轴上限、平移偏置和多轴组合约束整理物理速度目标。
        GaitCommand target = in.cmd;
        // ---- 单轴硬上限（单轴指令就是这三条）----
        if (p.cmd_max_vx > 0.0) target.vx = std::max(-p.cmd_max_vx, std::min(p.cmd_max_vx, target.vx));
        if (p.cmd_max_vy > 0.0) target.vy = std::max(-p.cmd_max_vy, std::min(p.cmd_max_vy, target.vy));
        if (p.cmd_max_wz > 0.0) target.wz = std::max(-p.cmd_max_wz, std::min(p.cmd_max_wz, target.wz));
        if (p.cmd_vel_trim > 0.0) {
            const double vm = std::hypot(target.vx, target.vy);
            if (vm > 1e-6) {
                const double trim = std::min(p.cmd_vel_trim, 0.5 * vm);
                target.vx *= (vm - trim) / vm;
                target.vy *= (vm - trim) / vm;
            }
        }
        // 三轴组合约束：hypot(vx,vy)+combo_wz_ratio*abs(wz)<=combo_max，超限时保持比例缩小。
        if (p.combo_max > 0.0) {
            const double s = std::hypot(target.vx, target.vy) +
                             p.combo_wz_ratio * std::fabs(target.wz);
            if (s > p.combo_max) {
                const double k = p.combo_max / s;
                target.vx *= k;
                target.vy *= k;
                target.wz *= k;
            }
        }

        // 可选平方约束：限制 vxy*wz²；默认关闭，超限处理按 combo_priority 配置。
        if (p.combo_turn_loss > 0.0) {
            const double vxy = std::hypot(target.vx, target.vy);
            const double wz_abs = std::fabs(target.wz);
            const double w2 = target.wz * target.wz;
            const double budget = p.combo_turn_loss / std::max(1e-6, p.combo_turn_gain);
            if (vxy > 1e-9 && w2 > 1e-12 && vxy * w2 > budget) {
                if (p.combo_priority == 1) {
                    // 转向优先：保 wz，削平移
                    const double k = budget / (vxy * w2);   // <1
                    target.vx *= k;
                    target.vy *= k;
                } else if (p.combo_priority == 2) {
                    // 平移优先：保平移，削 wz（求 vxy·w² = budget）
                    const double k = std::sqrt(budget / vxy) / wz_abs;   // <1
                    target.wz *= k;
                } else {
                    // 同比缩比：三轴同乘 k，使 k·vxy·(k·wz)² = budget → k³
                    const double k = std::cbrt(budget / (vxy * w2));     // <1
                    target.vx *= k;
                    target.vy *= k;
                    target.wz *= k;
                }
            }
        }

        // 步骤 3：保存合法帧间隔，低通并限速生成步态/MPC 共用的速度命令。
        last_dt_ = dt;
        const GaitCommand cmd = SmoothCommand(target, dt);

        // 可选步幅反馈：使用最近一帧估计修正足端步幅，不直接改变 MPC 的共用速度参考。
        GaitCommand gait_cmd = cmd;
        if (p.gait_vel_fb != 0.0 || p.gait_yaw_fb != 0.0 || p.gait_vel_trim != 0.0) {
            const Vec3d v_body = R.transpose() * estimate_.base_vel_w;
            const double wz_meas = in.omega_body.z();
            if (p.gait_vel_trim != 0.0) {
                // 沿命令方向收缩步幅（抵消「实际恒快一个常量」的标定偏差）。
                const double vmag = std::hypot(cmd.vx, cmd.vy);
                if (vmag > 1e-6) {
                    const double k = std::max(0.0, vmag - p.gait_vel_trim) / vmag;
                    gait_cmd.vx *= k;
                    gait_cmd.vy *= k;
                }
            }
            if (p.gait_vel_fb != 0.0) {
                gait_cmd.vx += p.gait_vel_fb * (cmd.vx - v_body.x());
                gait_cmd.vy += p.gait_vel_fb * (cmd.vy - v_body.y());
            }
            if (p.gait_yaw_fb != 0.0) {
                gait_cmd.wz += p.gait_yaw_fb * (cmd.wz - wz_meas);
            }
            // 限幅：反馈最多把步幅放大到包络的 gait_cmd_limit_scale 倍。
            const double lim = std::max(1.0, p.gait_cmd_limit_scale);
            const double lvx = lim * p.gait_schedule.max_vx;
            const double lvy = lim * p.gait_schedule.max_vy;
            const double lwz = lim * p.gait_schedule.max_wz;
            gait_cmd.vx = std::max(-lvx, std::min(lvx, gait_cmd.vx));
            gait_cmd.vy = std::max(-lvy, std::min(lvy, gait_cmd.vy));
            gait_cmd.wz = std::max(-lwz, std::min(lwz, gait_cmd.wz));
        }

        // 步骤 4：根据未平滑命令请求停步，使用同一相位源更新足端目标和计划支撑集。
        const bool raw_cmd_zero =
            gait_.NormalizedSpeed(in.cmd) < gait_.params().stop_threshold;
        const auto legs = gait_.Update(in.t, gait_cmd, raw_cmd_zero);
        out.stance = {{legs[kFL].contact, legs[kFR].contact, legs[kHL].contact, legs[kHR].contact}};
        const bool stepping = gait_.stepping();

        // 步骤 5：锁存姿态故障；故障时保持最近关节目标并返回纯位置环，等待状态机切阻尼。
        if (out.tilt > p.tilt_limit) fault_ = true;
        const bool tilt_ok_for_wbc = out.tilt <= p.wbc_enable_tilt;
        if (fault_) {
            // 故障：保持上一帧目标角（或当前角），纯位置环兜底，等上层切到阻尼状态
            out.mode = ControllerOutput::Mode::kFault;
            out.q_des = q_des_;
            out.joint_cmd.col(0).setConstant(p.kp_pos);
            out.joint_cmd.col(1) = out.q_des.cast<float>();
            out.joint_cmd.col(2).setConstant(p.kd_pos);
            out.joint_cmd.col(3).setZero();
            out.joint_cmd.col(4).setZero();
            out.fault = true;
            last_t_ = in.t;
            out.step_ms = ElapsedMs(t0);
            return out;
        }

        // 步骤 6：接管阶段插值关节角；之后由规划足位做 IK，可选反解关节速度并统一限位。
        if (ramping) {
            const double s = SmoothStep01(rel / std::max(1e-6, p.stand_ramp_time));
            q_des_ = (1.0 - s) * q_start_ + s * p.stand_q;
            dq_des_.setZero();  // 斜坡段的关节速度不参与前馈，显式清零防旧值残留
        } else {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                Vec3d seed = q_des_.segment<3>(3 * leg);
                Vec3d q_leg = seed;
                if (model_.InverseKinematics(leg, legs[leg].foot_pos, &q_leg, &seed))
                    q_des_.segment<3>(3 * leg) = q_leg;

                // 启用时通过 J_leg*dq_des=v_foot_body 求解速度；不可逆或关闭时清零。
                if (p.swing_joint_vel_ff) {
                    const Mat3d J_leg = model_.LegJacobianBody(leg, q_leg);
                    Eigen::FullPivLU<Mat3d> lu(J_leg);
                    if (lu.isInvertible())
                        dq_des_.segment<3>(3 * leg) = lu.solve(legs[leg].foot_vel);
                    else
                        dq_des_.segment<3>(3 * leg).setZero();
                } else {
                    dq_des_.segment<3>(3 * leg).setZero();
                }
            }
        }
        ClipToJointLimits(&q_des_);
        out.q_des = q_des_;

        // 步骤 7：以本帧计划支撑集更新估计器，再组装 [v_world;omega_body;dq] 广义速度。
        EstimatorInput est_in;
        est_in.t = in.t;
        est_in.rpy = in.rpy;
        est_in.omega_body = in.omega_body;
        est_in.acc_body = in.acc_body;
        est_in.q = in.q;
        est_in.dq = in.dq;
        est_in.contact = out.stance;
        estimate_ = estimator_.Update(est_in);
        out.base_pos = estimate_.base_pos_w;
        out.base_vel = estimate_.base_vel_w;
        out.height = estimate_.height;
        out.est_confidence = estimate_.confidence;

        // 世界速度装配（WBC/MPC 共用）：qvel = [v_world; ω_body; dq]
        Vec18d qvel = Vec18d::Zero();
        qvel.head<3>() = estimate_.base_vel_w;
        qvel.segment<3>(3) = in.omega_body;
        qvel.tail<12>() = in.dq;

        const Vec3d com_body = model_.CompositeComBody(in.q);
        const Vec3d com_world = estimate_.base_pos_w + R * com_body;
        out.com_pos = com_world;

        // 步骤 8：接管结束、姿态合格且估计有效时使能 WBC；已使能时允许短时门控失效。
        const bool wbc_gate_open = !ramping && !settling && estimate_.valid &&
                                   estimate_.num_support >= 1 && tilt_ok_for_wbc;
        if (wbc_gate_open) {
            invalid_frames_ = 0;
            wbc_active_ = true;
        } else if (wbc_active_ && !ramping && !settling && !fault_ &&
                   ++invalid_frames_ <= p.wbc_enable_hold_frames &&
                   p.wbc_enable_hold_frames > 0) {

            // 门控短时失效仍保留 WBC；超过 tilt_limit 的情况已由前面的故障分支处理。
            wbc_active_ = true;
        } else {
            invalid_frames_ = 0;
            wbc_active_ = false;
        }
        const bool wbc_ready = wbc_active_;

        if (wbc_ready) {

            // 步骤 9：构造 WBC 的实际关节、姿态、接触和可选姿势参考输入。
            WbcInput win;
            win.q = in.q;
            win.qvel = qvel;
            win.R_wb = R;
            win.stance = out.stance;

            // 关节姿势参考采用当前步态 IK 目标；可选姿势任务不应在行走时强拉回固定站姿。
            win.q_nominal = q_des_;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const Vec3d foot_body = model_.FootPositionBody(leg, in.q.segment<3>(3 * leg));
                win.foot_pos_world[leg] = estimate_.base_pos_w + R * foot_body;
            }
            const Mat12x18d Jc = model_.ContactJacobianWorld(in.q, R);
            for (int leg = 0; leg < kNumLegs; ++leg)
                win.foot_vel_world[leg] = Jc.block<3, kWbcQdd>(3 * leg, 0) * qvel;

            // 参考选择：优先使用当前有效且未过期的快照，否则尝试最近一次有效参考缓存。
            const MpcWrench& w = wrench_in_;
            if (w.valid) last_good_wrench_ = w;
            const auto fresh_enough = [&](const MpcWrench& c) {
                const double age = in.t - c.t;
                return c.valid && age >= -1e-9 && age <= p.wrench_max_age;
            };
            const MpcWrench* src = nullptr;
            if (fresh_enough(w)) {
                src = &w;                       // 本帧快照
            } else if (fresh_enough(last_good_wrench_)) {
                src = &last_good_wrench_;       // 沿用最近一次有效 wrench
            }

            out.mpc_status = src ? w.status : -1;   // -1 ⇔ 本帧 a_des 被清零
            out.mpc_reason = src ? 0 : (w.valid ? 4 : (w.reason != 0 ? w.reason : 4));
            out.mpc_held = (src == &last_good_wrench_);
            out.mpc_wrench_age = src ? (in.t - src->t) : 0.0;
            out.mpc_fresh = false;
            out.mpc_ms = w.solve_ms;
            out.mpc_iters = w.iters;
            out.mpc_qp_residual = w.qp_residual;
            if (src != nullptr) {
                win.foot_force_ref = src->foot_force_ref;
                a_des_ = src->a_des_world;
                alpha_des_ = src->alpha_des_body;
                out.mpc_net_fz = src->net_force.z();
                out.mpc_fresh = (w.valid && out.mpc_wrench_age < 2.0 * p.mpc_period);
            } else {
                a_des_.setZero();
                alpha_des_.setZero();
            }
            win.a_des_world = a_des_;
            win.alpha_des_body = alpha_des_;
            out.a_des = a_des_;            // 诊断：本帧真正送进 WBC 的期望加速度
            out.alpha_des = alpha_des_;

            // 步骤 10：把四腿机体系规划足位/足速转为世界系输入；默认只取偏航旋转。
            const Mat3d R_ref = p.swing_target_yaw_only ? YawOnly(R) : R;

            // 摆动脚用这些目标构造足端任务；支撑脚可选反馈也使用同一规划输入。
            for (int leg = 0; leg < kNumLegs; ++leg) {
                win.foot_pos_des_world[leg] = estimate_.base_pos_w + R_ref * legs[leg].foot_pos;
                win.foot_vel_des_world[leg] = R_ref * legs[leg].foot_vel;
            }
            out.stance_count = 0;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (win.stance[leg]) {
                    ++out.stance_count;
                    continue;
                }
                win.foot_des_world[leg] = win.foot_pos_des_world[leg];
                out.swing_des_z[leg] = win.foot_des_world[leg].z();
                out.swing_now_z[leg] = win.foot_pos_world[leg].z();
            }

            // 步骤 11：求解 WBC；可用结果重置失败计数，无可用结果时退回位置环并累计故障。
            const WbcOutput wout = wbc_.Solve(in.t, win);
            out.wbc_stale = (wout.status == WbcOutput::Status::kStale);
            if (wout.ok()) {
                out.tau_wbc = wout.tau;
                out.foot_force = wout.foot_force;
                out.wbc_eom_residual = wout.eom_residual;
                out.wbc_contact_residual = wout.contact_residual;
                out.wbc_cone_violation = wout.cone_violation;
                out.wbc_base_accel_error = wout.base_accel_error;
                out.wbc_swing_accel_error = wout.swing_accel_error;
                out.tau_abs_max = wout.tau_abs_max;
                out.wbc_iters = wout.iters;
                out.wbc_ms = wout.solve_ms;
                wbc_fail_count_ = 0;
                out.wbc_active = true;
            } else {
                ++wbc_fail_count_;
                if (wbc_fail_count_ >= p.max_consecutive_wbc_fail) {
                    fault_ = true;
                    out.fault = true;
                }
                wbc_active_ = false;  // 本帧退回位置环
            }
        } else {
            out.mpc_status = -1;  // 本帧未使用 MPC 参考（未使能 WBC）
        }

        // 步骤 12：组装 12×5 命令；WBC 有效时按同一帧实测状态反解前馈，否则前馈为 0。
        const double kp = wbc_active_ ? p.kp_pos * p.kp_scale_wbc : p.kp_pos;
        const double kd = wbc_active_ ? p.kd_pos * p.kd_scale_wbc : p.kd_pos;
        out.joint_cmd.col(0).setConstant(kp);
        out.joint_cmd.col(1) = q_des_.cast<float>();
        out.joint_cmd.col(2).setConstant(kd);
        out.joint_cmd.col(3) = dq_des_.cast<float>();  // 期望关节速度（步态足端速度反解，站立为 0）
        for (int j = 0; j < 12; ++j) {
            const double pd = kp * (q_des_(j) - in.q(j)) + kd * (dq_des_(j) - in.dq(j));
            out.tau_pd(j) = pd;

            // 默认 tau_ff=tau_wbc-PD；关闭 WBC 时保留纯位置环，避免反向抵消其驱动力矩。
            double tau_ff = 0.0;
            if (wbc_active_) {

                // 先抵消下游 PD，再按配置叠加额外 PD，最后限制前馈绝对值。
                tau_ff = out.tau_wbc(j) - pd;

                // 可选额外 PD 关闭时，未饱和总力矩等于 WBC 输出；启用时额外叠加 pd_extra*PD。
                if (p.pd_extra > 0.0) {
                    const int leg = j / 3;
                    if (!p.pd_extra_stance_only || out.stance[static_cast<int>(leg)])
                        tau_ff += p.pd_extra * pd;
                }
                tau_ff = std::max(-p.wbc.tau_max, std::min(p.wbc.tau_max, tau_ff));
            }
            out.joint_cmd(j, 4) = static_cast<float>(tau_ff);
        }

        // 步骤 13：记录阶段、采样时间和本帧耗时并返回，由状态文件实际发送命令。
        out.mode = ramping ? ControllerOutput::Mode::kRamp
                           : (stepping ? ControllerOutput::Mode::kWalk : ControllerOutput::Mode::kStand);
        last_t_ = in.t;
        out.step_ms = ElapsedMs(t0);
        return out;
    }


private:
    //------------------------------------------------------------------------------------
    // @brief 使用单调时钟计算一帧耗时 [ms]
    //------------------------------------------------------------------------------------
    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }

    //------------------------------------------------------------------------------------
    // @brief 生成接管关节插值使用的三次平滑系数，输入夹取到 [0,1]
    //------------------------------------------------------------------------------------
    static double SmoothStep01(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    //------------------------------------------------------------------------------------
    // @brief 从完整旋转矩阵提取偏航，生成保持水平的足端轨迹参考系
    //------------------------------------------------------------------------------------
    static Mat3d YawOnly(const Mat3d& R) {
        const double yaw = std::atan2(R(1, 0), R(0, 0));
        return Rz(yaw);
    }

    //------------------------------------------------------------------------------------
    // @brief 生成绕世界竖直 z 轴旋转的矩阵，yaw 单位 rad
    //------------------------------------------------------------------------------------
    static Mat3d Rz(double yaw) {
        Mat3d R = Mat3d::Identity();
        const double c = std::cos(yaw), s = std::sin(yaw);
        R(0, 0) = c;
        R(0, 1) = -s;
        R(1, 0) = s;
        R(1, 1) = c;
        return R;
    }

    //------------------------------------------------------------------------------------
    // @brief 把角度差折叠到 (-π,π]，用于累加测量偏航的最短增量
    // @details 连续偏航参考本身不在每帧折叠，避免跨越 ±π 时产生参考分支跳变。
    //------------------------------------------------------------------------------------
    static double WrapPi(double a) {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0.0) a += 2.0 * M_PI;
        return a - M_PI;
    }

    //------------------------------------------------------------------------------------
    // @brief 按四腿三个关节的模型限位夹取关节目标，原地修改输入
    //------------------------------------------------------------------------------------
    void ClipToJointLimits(Vec12d* q) const 
    {
        for (int leg = 0; leg < kNumLegs; ++leg)
            for (int j = 0; j < 3; ++j) {
                const int idx = 3 * leg + j;
                const double lo = model_.geo.joint_lower(j), hi = model_.geo.joint_upper(j);
                (*q)(idx) = std::max(lo, std::min(hi, (*q)(idx)));
            }
    }

    //------------------------------------------------------------------------------------
    // @brief 绑定或关闭三段调度，并按 nominal_x 增量更新标称足位配平
    // @details 只应用相对上次偏移的差值，重复调用不会累加同一配平量。
    //------------------------------------------------------------------------------------
    void ApplyGaitConfig() {
        gait_.SetSchedule(params_.use_gait_schedule ? &params_.gait_schedule : nullptr);
        const double dx = params_.nominal_x - applied_nominal_x_;
        if (std::fabs(dx) > 1e-12) {
            gait_.ShiftNominal(Vec3d(dx, 0.0, 0.0));
            applied_nominal_x_ = params_.nominal_x;
        }
    }

    //------------------------------------------------------------------------------------
    // @brief 按站姿球心最低高度计算平地触地所需的基座与质心高度
    // @details 仅在 com_height 非正时用几何结果填入目标值；正目标按调用者配置保留。
    //------------------------------------------------------------------------------------
    void ComputeStandGeometry() {
        double min_foot_z = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg)
            min_foot_z = std::min(min_foot_z,
                                  model_.FootPositionBody(leg, params_.stand_q.segment<3>(3 * leg)).z());
        stand_base_height_ = -min_foot_z + model_.geo.foot_radius;
        stand_com_height_ = stand_base_height_ + model_.CompositeComBody(params_.stand_q).z();
        if (!(params_.com_height > 0.0)) params_.com_height = stand_com_height_;
    }

    // 缓存组 1：整机配置与各算法对象，由快速线程使用估计/步态/WBC，MPC 线程使用 mpc_。
    MpcWbcControllerParams params_;                             // 整机配置；工作线程运行时不应并发修改
    Lite3Model model_;                                          // 快速线程与 MPC 线程共用的只读机器人模型
    StateEstimator estimator_;                                  // 快速线程维护的状态估计器
    GaitTrajectory gait_;                                       // 快速线程维护的步态、目标足位和接触计划
    double applied_nominal_x_ = 0.0;  // ApplyGaitConfig 已施加的标称前后偏移
    SrbdPreviewMpc mpc_;                                        // 仅由 MPC 求解路径更新的单刚体优化器及其缓存
    WeightedWbc wbc_;                                           // 快速线程执行的全身控制器

    // 缓存组 2：最近估计、滤波命令、高度参考，以及当前/最近有效 MPC 结果。
    StateEstimate estimate_;                                    // 快速线程的最近状态估计
    GaitCommand cmd_filtered_;               // 平滑后的速度命令（步态/MPC 共用）
    double height_cmd_ = 0.0;                // 质心高度命令（软启动状态）
    bool height_cmd_initialized_ = false;                       // 是否已设置过高度参考
    double last_dt_ = 0.0;                                      // 上一快速控制帧采用的合法时间间隔 [s]
    MpcWrench wrench_in_;                    // 本帧使用的 wrench（由状态文件传入工作线程快照）
    MpcWrench last_good_wrench_;             // 最近一次有效 wrench（求解失败时按 age 上限沿用）
    Vec3d a_des_ = Vec3d::Zero();                               // 当前向 WBC 提供的世界系线加速度参考 [m/s²]
    Vec3d alpha_des_ = Vec3d::Zero();                           // 当前向 WBC 提供的机体系角加速度参考 [rad/s²]

    // 缓存组 3：进入时间、连续偏航、关节参考、WBC 失败计数和故障锁存。
    double t_enter_ = 0.0;                                      // 进入控制状态的接口时间 [s]
    double last_t_ = 0.0;                                       // 上一快速控制帧的接口时间 [s]
    double yaw_ref_ = 0.0;              // 偏航角参考（世界系，连续不归一化，1 kHz 积分）
    double last_yaw_ref_t_ = 0.0;                               // 上一次推进连续偏航参考的时间 [s]
    bool yaw_ref_initialized_ = false;                          // 偏航参考是否已按当前测量初始化
    double yaw_meas_cont_ = 0.0;        // 连续（不归一化）的测量偏航角
    double yaw_meas_prev_ = 0.0;                                // 上一帧未展开的测量偏航 [rad]
    bool yaw_meas_initialized_ = false;                         // 连续测量偏航是否已初始化
    Vec12d q_start_ = Vec12d::Zero();                           // 进入控制状态时的实测关节角 [rad]
    Vec12d q_des_ = Vec12d::Zero();                             // 当前关节位置目标 [rad]，也用作 IK 分支种子
    Vec12d dq_des_ = Vec12d::Zero();                            // 当前关节速度目标 [rad/s]，默认速度前馈关闭时为 0
    int wbc_fail_count_ = 0;                                    // 连续 WBC 求解失败次数；成功时清零，未使能帧不累加
    int invalid_frames_ = 0;                                    // WBC 已使能后的连续门控失效帧数
    bool wbc_active_ = false;                                   // 当前是否允许使用 WBC 力矩构造输出
    bool fault_ = false;                                        // 故障锁存标志，进入状态 Reset 时清除

    // 缓存组 4：按 stand_q 和模型算出的平地站立几何高度。
    double stand_base_height_ = 0.0;                            // 站姿球足接触平地时的基座高度 [m]
    double stand_com_height_ = 0.0;                             // 对应站姿的整机质心高度 [m]
};



//===============================================================================================
// @brief MPC 独立工作线程：读取最新请求、求解并发布参考结果
// @details 工作线程不发送关节命令；由状态文件在 OnEnter/OnExit 中启动和停止。
// @details 每轮计算结束后休眠 period；实际求解间隔还包含计算耗时和线程调度。
//===============================================================================================
class MpcWorkerThread 
{
public:
    //------------------------------------------------------------------------------------
    // @brief 保存非拥有型控制器指针和休眠间隔，构造时不启动线程
    // @details controller 的生命周期必须覆盖本工作线程。
    //------------------------------------------------------------------------------------
    MpcWorkerThread(MpcWbcController* controller, double period)
        : ctrl_(controller), period_(period) {}

    //------------------------------------------------------------------------------------
    // @brief 析构前停止并回收线程，防止后台继续访问控制器
    //------------------------------------------------------------------------------------
    ~MpcWorkerThread() { Stop(); }

    //------------------------------------------------------------------------------------
    // @brief 若尚未运行则启动 MPC 线程，重复调用不再新建线程
    //------------------------------------------------------------------------------------
    void Start() {
        if (running_) return;
        running_ = true;
        thread_ = std::thread(&MpcWorkerThread::Loop, this);
    }

    //------------------------------------------------------------------------------------
    // @brief 请求退出并等待线程结束，可能等待当前求解和休眠完成
    //------------------------------------------------------------------------------------
    void Stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    SeqLockBox<MpcRequest> requests;   // 1 kHz → MPC
    SeqLockBox<MpcWrench> wrenches;    // MPC → 1 kHz

private:
    //------------------------------------------------------------------------------------
    // @brief 读取版本更新的请求，调用 SolveWrench 发布结果，然后按 period 休眠
    // @details 同一请求版本不重复求解；多帧请求覆盖时处理最新的可读快照。
    //------------------------------------------------------------------------------------
    void Loop() {
        uint64_t last_seq = 0;
        while (running_) {
            MpcRequest req;
            uint64_t seq = 0;
            if (requests.ReadWithSeq(&req, &seq) && seq != last_seq) {
                last_seq = seq;
                wrenches.Write(ctrl_->SolveWrench(req));
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(period_));
        }
    }

    // 工作线程成员：控制器非拥有指针、休眠周期、线程句柄与原子运行标志。
    MpcWbcController* ctrl_;                                    // 非拥有型控制器指针，控制器应在工作线程停止后再销毁
    double period_;                                             // 每轮处理结束后的休眠时长 [s]
    std::thread thread_;                                        // MPC 后台求解线程句柄
    std::atomic<bool> running_{false};                          // 线程间共享的运行/停止标志
};

}  // namespace mpc_wbc



