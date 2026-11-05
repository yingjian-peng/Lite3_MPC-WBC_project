/*===============================================================================================
 * @file 02_gait_trajectory.hpp
 * @brief Lite3 对角小跑步态、速度分段调度和足端轨迹生成
 * @details 输入机体系速度命令 vx/vy [m/s]、wz [rad/s]；输出四腿支撑标志和球心位置/速度目标。
 * 机体系约定：x 前、y 左、z 上；四腿顺序 FL、FR、HL、HR。
 * 腿相位 phi_i = frac((t-t_start)/T + phase_offset_i)，phi_i<duty 时为支撑相。
 * 水平步幅由支撑时长 T*duty 和 vx/vy/wz 共同决定；摆动相用五次水平插值和余弦抬脚。
 * 支持三段速度参数插值和相位对齐软停步；不使用世界系足端锚点锁定。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include "00_lite3_model.hpp"                       // 统一坐标/关节约定及 Lite3 运动学、动力学模型

#include <algorithm>                                // 最值、夹取和排序等标准算法
#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <cmath>                                    // 三角函数、绝对值和其他数学函数



namespace mpc_wbc {
//===============================================================================================
// @brief 固定步态配置与起停阈值
//===============================================================================================
struct GaitParams {
    // 固定步态配置；三段调度开启后，由选中的速度段替换周期、占空比和抬脚高度
    double period = 0.60;                                       // 步态周期 T [s]，应为正数
    double duty = 0.68;                                         // 支撑占空比 d，应处于 (0,1)
    double swing_height = 0.10;                                 // 摆动相球心相对标称高度的最大抬升 [m]

    // 固定步态默认使用较低上限；主状态开启三段调度时，输入映射采用调度表的全局上限
    double max_vx = 0.20;                                       // 固定步态的前后速度归一化上限 [m/s]
    double max_vy = 0.15;                                       // 固定步态的横移速度归一化上限 [m/s]
    double max_wz = 0.30;                                       // 固定步态的偏航角速度归一化上限 [rad/s]

    // 起停判据和相位配置
    double start_threshold = 0.03;                              // 起步阈值：各轴相对上限的最大比例需大于此值
    double stop_threshold = 0.015;                              // 停步滞回阈值，小于起步阈值以避免反复切换
    std::array<double, kNumLegs> phase_offset = {{0.0, 0.5, 0.5, 0.0}};  // 四腿相位偏移；FL/HR 同相，FR/HL 与其相差半周期
    double stop_ramp_periods = 1.0;                             // 请求停步后，水平步幅从 1 收到 0 所需的周期数
};



//===============================================================================================
// @brief 已经换算为物理单位的机体系速度命令
//===============================================================================================
struct GaitCommand {
    double vx = 0.0;                                            // 机体系前后速度 [m/s]，正值向前
    double vy = 0.0;                                            // 机体系横移速度 [m/s]，正值向左
    double wz = 0.0;                                            // 绕机体系 z 轴的偏航角速度 [rad/s]，正值左转
};



//===============================================================================================
// @brief 按主运动方向和速度区间选择低、中、高速步态
//===============================================================================================
struct SegmentedGaitParams {
    // 三组步态参数与全局速度上限；本表本身不对物理速度命令限幅。
    GaitParams low;                                             // 低速段周期、支撑占比、抬脚高度等配置
    GaitParams mid;                                             // 中速段配置
    GaitParams high;                                            // 高速段配置
    double max_vx = 0.60;                                       // 前后全局上限 [m/s]，用于主方向归一化和状态机满杆映射
    double max_vy = 0.50;                                       // 横移全局上限 [m/s]
    double max_wz = 1.00;                                       // 转向全局上限 [rad/s]

    // 按主运动方向的绝对物理速度选段。
    double vx_low_mid = 0.20;                                   // 前后速度低/中速分界 [m/s]
    double vx_mid_high = 0.35;                                  // 前后速度中/高速分界 [m/s]
    double vy_low_mid = 0.15;                                   // 横移速度低/中速分界 [m/s]
    double vy_mid_high = 0.22;                                  // 横移速度中/高速分界 [m/s]
    double wz_low_mid = 0.20;                                   // 转向角速度低/中速分界 [rad/s]
    double wz_mid_high = 0.30;                                  // 转向角速度中/高速分界 [rad/s]

    // 边界附近平滑改变周期和步幅，减少减速时的突变。
    double blend_vx = 0.10;                                     // 前后速度段边界的平滑插值带宽 [m/s]
    double blend_vy = 0.075;                                    // 横移段边界插值带宽 [m/s]
    double blend_wz = 0.10;                                     // 转向段边界插值带宽 [rad/s]

    //------------------------------------------------------------------------------------
    // @brief 生成三段基础参数表
    // @details 低/中/高速周期分别为 1.20/0.75/0.44 s，支撑占比为 0.72/0.70/0.625；各段共享全局速度上限。
    //------------------------------------------------------------------------------------
    static SegmentedGaitParams Baseline() {
        SegmentedGaitParams s;
        s.low = s.mid = s.high = GaitParams();
        s.low.period = 1.20;
        s.low.duty = 0.72;
        s.low.swing_height = 0.07;
        s.mid.period = 0.75;
        s.mid.duty = 0.70;
        s.mid.swing_height = 0.07;
        s.high.period = 0.44;
        s.high.duty = 0.625;
        s.high.swing_height = 0.07;
        for (GaitParams* p : {&s.low, &s.mid, &s.high}) {
            p->max_vx = s.max_vx;
            p->max_vy = s.max_vy;
            p->max_wz = s.max_wz;
        }
        return s;
    }

    //------------------------------------------------------------------------------------
    // @brief 生成当前 MPC+WBC 主线使用的三段参数表
    // @details 在基础表上只将高速段抬脚高度设为 0.10 m，低、中速段保持 0.07 m。
    //------------------------------------------------------------------------------------
    static SegmentedGaitParams BaselineClosedLoop() {
        SegmentedGaitParams s = Baseline();
        s.high.swing_height = 0.10;
        return s;
    }

    //------------------------------------------------------------------------------------
    // @brief 将 [0,1] 输入映射为端点斜率为零的三次平滑曲线
    //------------------------------------------------------------------------------------
    static double SmoothStep01(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    //------------------------------------------------------------------------------------
    // @brief 在两组步态参数之间做连续插值
    // @details 周期、支撑占比、抬脚高度线性混合；速度上限和起停阈值取两组较大值，相位偏移沿用 a。
    //------------------------------------------------------------------------------------
    static GaitParams Blend(const GaitParams& a, const GaitParams& b, double alpha) {
        alpha = std::max(0.0, std::min(1.0, alpha));
        GaitParams r = a;
        r.period = a.period + alpha * (b.period - a.period);
        r.duty = a.duty + alpha * (b.duty - a.duty);
        r.swing_height = a.swing_height + alpha * (b.swing_height - a.swing_height);
        r.max_vx = std::max(a.max_vx, b.max_vx);
        r.max_vy = std::max(a.max_vy, b.max_vy);
        r.max_wz = std::max(a.max_wz, b.max_wz);
        r.start_threshold = std::max(a.start_threshold, b.start_threshold);
        r.stop_threshold = std::max(a.stop_threshold, b.stop_threshold);
        return r;
    }

    //------------------------------------------------------------------------------------
    // @brief 按单一主方向的速度大小选择或混合低、中、高速参数
    // @param s 当前方向速度绝对值；b1/b2 为两处分界；blend 为边界过渡带宽。
    // @details 各边界两侧半带宽范围内使用 SmoothStep01 混合相邻参数。
    //------------------------------------------------------------------------------------
    static GaitParams SelectAxis(double s, double b1, double b2, double blend,
                                 const GaitParams& low, const GaitParams& mid,
                                 const GaitParams& high) 
    {
        const double h = 0.5 * std::max(1e-9, blend);
        if (s <= b1 - h) return low;
        if (s < b1 + h)
            return Blend(low, mid, SmoothStep01((s - (b1 - h)) / (2.0 * h)));
        if (s <= b2 - h) return mid;
        if (s < b2 + h)
            return Blend(mid, high, SmoothStep01((s - (b2 - h)) / (2.0 * h)));
        return high;
    }

    //------------------------------------------------------------------------------------
    // @brief 先比较各轴相对全局上限的比例，再按占比最大的方向选段
    // @details 输入应为物理速度命令；比例只用于选择主轴，分段判断使用该轴的绝对物理速度。
    //------------------------------------------------------------------------------------
    GaitParams Select(const GaitCommand& cmd) const 
    {
        const double ax = std::fabs(cmd.vx) / std::max(1e-9, max_vx);
        const double ay = std::fabs(cmd.vy) / std::max(1e-9, max_vy);
        const double aw = std::fabs(cmd.wz) / std::max(1e-9, max_wz);
        if (ax >= ay && ax >= aw)
            return SelectAxis(std::fabs(cmd.vx), vx_low_mid, vx_mid_high, blend_vx, low, mid, high);
        if (ay >= aw)
            return SelectAxis(std::fabs(cmd.vy), vy_low_mid, vy_mid_high, blend_vy, low, mid, high);
        
        return SelectAxis(std::fabs(cmd.wz), wz_low_mid, wz_mid_high, blend_wz, low, mid, high);
    }

    //------------------------------------------------------------------------------------
    // @brief 返回离散速度段编号：0=低速、1=中速、2=高速
    // @details 不包含边界插值，仅供诊断或显示当前区间。
    //------------------------------------------------------------------------------------
    int SelectIndex(const GaitCommand& cmd) const 
    {
        const double ax = std::fabs(cmd.vx) / std::max(1e-9, max_vx);
        const double ay = std::fabs(cmd.vy) / std::max(1e-9, max_vy);
        const double aw = std::fabs(cmd.wz) / std::max(1e-9, max_wz);
        double s = 0.0, b1 = 0.0, b2 = 0.0;
        if (ax >= ay && ax >= aw) {
            s = std::fabs(cmd.vx);
            b1 = vx_low_mid;
            b2 = vx_mid_high;
        } 
        else if (ay >= aw) {
            s = std::fabs(cmd.vy);
            b1 = vy_low_mid;
            b2 = vy_mid_high;
        } 
        else {
            s = std::fabs(cmd.wz);
            b1 = wz_low_mid;
            b2 = wz_mid_high;
        }

        if (s <= b1) return 0;
        if (s <= b2) return 1;

        return 2;
    }
};



//===============================================================================================
// @brief 单腿的计划接触状态和机体系球心轨迹
//===============================================================================================
struct LegGaitState {
    bool contact = true;                                        // 计划支撑标志；true=支撑相，false=摆动相
    double phase = 0.0;                                         // 当前腿相位 [0,1)
    double progress = 0.0;                                      // 当前支撑相或摆动相的归一化进度 [0,1]
    Vec3d foot_pos = Vec3d::Zero();                             // 球心的机体系目标位置 [m]
    Vec3d foot_vel = Vec3d::Zero();                             // 球心相对机体系的规划速度 [m/s]
};



//===============================================================================================
// @brief 管理步态相位、三段参数和软停步状态，生成四腿轨迹
//===============================================================================================
class GaitTrajectory 
{
public:
    //------------------------------------------------------------------------------------
    // @brief 保存固定步态与可选调度表，并用默认站姿初始化标称足位
    //------------------------------------------------------------------------------------
    explicit GaitTrajectory(const GaitParams& params = GaitParams(), const SegmentedGaitParams* schedule = nullptr)
        : params_(params), schedule_(schedule) {
        InitNominal();
    }

    //------------------------------------------------------------------------------------
    // @brief 读取当前正在使用的步态参数
    //------------------------------------------------------------------------------------
    const GaitParams& params() const { return params_; }

    //------------------------------------------------------------------------------------
    // @brief 直接替换固定参数；运动中连续切换周期应使用 SetParamsAt
    //------------------------------------------------------------------------------------
    void set_params(const GaitParams& p) { params_ = p; }

    //------------------------------------------------------------------------------------
    // @brief 绑定三段调度表；传 nullptr 时使用固定步态参数
    // @details 保存的是外部指针，调度表的生命周期应覆盖本对象的使用期。
    //------------------------------------------------------------------------------------
    void SetSchedule(const SegmentedGaitParams* schedule) { schedule_ = schedule; }

    //------------------------------------------------------------------------------------
    // @brief 在指定时刻切换参数，同时调整起始时间以保持全局相位连续
    // @param t 当前时间 [s]；改变周期时不直接重启整个步态。
    //------------------------------------------------------------------------------------
    void SetParamsAt(double t, const GaitParams& p) 
    {
        if (stepping_ && params_.period > 1e-9 && p.period > 1e-9) {
            const double phase = Wrap01((t - t_start_) / params_.period);
            t_start_ = t - phase * p.period;
        }
        params_ = p;
    }

    //------------------------------------------------------------------------------------
    // @brief 读取四腿机体系标称球心位置，按四腿 xyz 排列
    //------------------------------------------------------------------------------------
    const Vec12d& nominal_feet() const { return nominal_feet_; }

    //------------------------------------------------------------------------------------
    // @brief 由给定的 12 个站立关节角重新计算标称球心位置
    //------------------------------------------------------------------------------------
    void SetNominalFromJointAngles(const Vec12d& q) {
        for (int leg = 0; leg < kNumLegs; ++leg){
            nominal_feet_.segment<3>(3 * leg) = model_.FootPositionBody(leg, q.segment<3>(3 * leg));
        }
    }

    //------------------------------------------------------------------------------------
    // @brief 整体平移标称球心位置，用于调整机身相对支撑区域的配平
    // @param delta 机体系位移 [m]；当前整机控制器用 nominal_x 调整前后位置。
    //------------------------------------------------------------------------------------
    void ShiftNominal(const Vec3d& delta) {
        for (int leg = 0; leg < kNumLegs; ++leg){
            nominal_feet_.segment<3>(3 * leg) += delta;
        }
    }

    //------------------------------------------------------------------------------------
    // @brief 将归一化控制量换算为各方向的物理速度命令
    // @param normalized 顺序为前后、横移、转向，各轴夹取到 [-1,1]。
    // @param deadband 额外零区，默认 0；此函数只置零，不重映射零区之外的比例。
    // @details USB 手柄的 10% 死区及其外侧线性重映射已经在输入接口中完成。
    //------------------------------------------------------------------------------------
    static GaitCommand ScaleCommand(const Vec3d& normalized, const GaitParams& p, double deadband = 0.0) 
    {
        auto scale = [deadband](double c, double limit) {
            if (std::fabs(c) < deadband) return 0.0;
            return std::max(-1.0, std::min(1.0, c)) * limit;
        };
        GaitCommand cmd;
        cmd.vx = scale(normalized(0), p.max_vx);
        cmd.vy = scale(normalized(1), p.max_vy);
        cmd.wz = scale(normalized(2), p.max_wz);
        
        return cmd;
    }

    //------------------------------------------------------------------------------------
    // @brief 取三个方向速度相对当前上限的最大比例，供起停判定
    //------------------------------------------------------------------------------------
    double NormalizedSpeed(const GaitCommand& c) const 
    {
        const double vx = std::fabs(c.vx) / std::max(1e-9, params_.max_vx);
        const double vy = std::fabs(c.vy) / std::max(1e-9, params_.max_vy);
        const double w = std::fabs(c.wz) / std::max(1e-9, params_.max_wz);
        
        return std::max({vx, vy, w});
    }

    //------------------------------------------------------------------------------------
    // @brief 读取是否正在执行周期步态
    //------------------------------------------------------------------------------------
    bool stepping() const { return stepping_; }

    //------------------------------------------------------------------------------------
    // @brief 读取当前全局相位零点的时间 [s]
    //------------------------------------------------------------------------------------
    double phase_start_time() const { return t_start_; }

    //------------------------------------------------------------------------------------
    // @brief 读取软停步期间的水平步幅系数，供 MPC 同步缩小速度参考
    // @details 收步时由 1 降到 0；其余阶段返回 1，抬脚高度不乘此系数。
    //------------------------------------------------------------------------------------
    double plan_scale() const { return (stepping_ && stopping_) ? stop_scale_ : 1.0; }

    //------------------------------------------------------------------------------------
    // @brief 复位为四腿静立，并清空停步锁存状态
    //------------------------------------------------------------------------------------
    void Reset(double t) {
        stepping_ = false;
        t_start_ = t;
        ResetStopLatch();
    }

    //------------------------------------------------------------------------------------
    // @brief 每帧更新速度选段、起停滞回和收步状态，再生成四腿目标
    // @param t 当前时间 [s]；cmd 为平滑后的物理速度；cmd_zero 由未平滑的原始命令判断。
    // @details 停步时锁定当前段参数，等待相位边界后逐步收小水平步幅，在后续边界退出周期步态。
    //------------------------------------------------------------------------------------
    std::array<LegGaitState, kNumLegs> Update(double t, const GaitCommand& cmd, bool cmd_zero = false) 
    {
        // 步骤 1：请求停步时锁定当前速度段参数，避免减速选段把收步周期拉长。
        if (cmd_zero && !stop_latched_) {
            stop_params_ = params_;
            stop_latched_ = true;
        } 
        else if (!cmd_zero) {
            stop_latched_ = false;
        }

        if (schedule_ != nullptr){
            SetParamsAt(t, stop_latched_ ? stop_params_ : schedule_->Select(cmd));
        }

        // 步骤 2：根据未平滑命令和归一化速度执行起停滞回。
        const double ns = NormalizedSpeed(cmd);
        if (!stepping_) {
            stopping_ = false;
            stop_scale_ = 1.0;
            // 重新起步同时要求原始命令非零，防止滤波尾值在收步结束后再次触发行走。
            if (!cmd_zero && ns > params_.start_threshold) {
                stepping_ = true;
                t_start_ = t;  // θ=0 时四足均处于支撑相，可从静立平稳起走
            }
        } 
        else if (!cmd_zero && ns >= params_.stop_threshold) {
            // 命令恢复：取消停车过程（s 突变只影响后续目标速度，位置本身连续）。
            stopping_ = false;
            stop_scale_ = 1.0;
        } 
        else {
            // 步骤 3：先对齐相位边界，再在指定周期数内把水平步幅收到 0，最后退出步态。
            const double T = std::max(1e-6, params_.period);
            const double theta = Wrap01((t - t_start_) / params_.period);
            if (!stopping_) {
                if (theta < 0.01) {  // 对齐相位边界再开始收步
                    stopping_ = true;
                    t_stop_ = t;
                }
            }
            if (stopping_) {
                const double ramp = T * std::max(0.1, params_.stop_ramp_periods);
                stop_scale_ = std::max(0.0, 1.0 - (t - t_stop_) / ramp);
                if (stop_scale_ <= 0.0 && theta < 0.01) {
                    stepping_ = false;
                    stopping_ = false;
                    stop_scale_ = 1.0;
                }
            }
        }
        return Evaluate(t, cmd);
    }

    //------------------------------------------------------------------------------------
    // @brief 在给定时刻只读计算四腿目标，不修改步态状态
    // @details 静立时返回标称足位和零速度；行走时按腿相位分别生成支撑后扫和摆动轨迹。
    // @details foot_vel 为按当前周期和步幅计算的相内速度，不含命令变化及收步系数变化的额外导数。
    //------------------------------------------------------------------------------------
    std::array<LegGaitState, kNumLegs> Evaluate(double t, const GaitCommand& cmd) const 
    {
        std::array<LegGaitState, kNumLegs> out;
        if (!stepping_) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                LegGaitState st;
                st.foot_pos = nominal_feet_.segment<3>(3 * leg);
                out[leg] = st;  // contact=true, vel=0
            }
            return out;
        }

        // 步骤 1：计算当前全局相位、周期及支撑相时长。
        const double T = params_.period;
        const double d = params_.duty;
        const double t_stance = T * d;
        const double theta = GlobalPhase(t);

        for (int leg = 0; leg < kNumLegs; ++leg) {
            LegGaitState st;
            st.phase = Wrap01(theta + params_.phase_offset[leg]);
            st.contact = st.phase < d;

            const Vec3d p0 = nominal_feet_.segment<3>(3 * leg);

            // 步骤 2：计算速度与转向共同产生的水平步幅；软停步仅缩放水平分量。
            const double dp_scale = stopping_ ? stop_scale_ : 1.0;
            const Vec3d dp(dp_scale * t_stance * (cmd.vx - cmd.wz * p0.y()), dp_scale * t_stance * (cmd.vy + cmd.wz * p0.x()), 0.0);

            // 步骤 3：支撑相线性后扫，摆动相五次水平插值并叠加余弦抬脚。
            if (st.contact) {
                st.progress = st.phase / d;
                st.foot_pos = p0 + (0.5 - st.progress) * dp;
                st.foot_vel = -dp / t_stance;  // d/dt[(1/2-ξ)Δp], dξ/dt = 1/T_stance
            } 
            else {
                st.progress = (st.phase - d) / (1.0 - d);
                const double xi = st.progress;
                st.foot_pos = p0 + (SwingCurve(xi) - 0.5) * dp;
                st.foot_pos.z() = p0.z() + SwingHeightCurve(xi, params_.swing_height);

                const double dxi_dt = 1.0 / (T * (1.0 - d));
                st.foot_vel.x() = dp.x() * SwingCurveDerivative(xi) * dxi_dt;
                st.foot_vel.y() = dp.y() * SwingCurveDerivative(xi) * dxi_dt;
                st.foot_vel.z() = SwingHeightDerivative(xi, params_.swing_height) * dxi_dt;
            }
            out[leg] = st;
        }
        return out;
    }

    //------------------------------------------------------------------------------------
    // @brief 根据同一相位源预测未来 N 个采样点的支撑标志
    // @param t 预测起始时间 [s]；dt 为预测节点间隔 [s]，不是状态机循环周期。
    // @details 按当前步态状态和参数向前展开，不在预测过程中重新执行起停或速度选段。
    //------------------------------------------------------------------------------------
    template <int N>
    std::array<std::array<bool, kNumLegs>, N> ContactSchedule(double t, double dt) const {
        std::array<std::array<bool, kNumLegs>, N> out{};
        for (int k = 0; k < N; ++k) {
            const double theta = stepping_ ? PhaseAt(t + k * dt) : 0.0;
            for (int leg = 0; leg < kNumLegs; ++leg)
                out[k][leg] = !stepping_ || (Wrap01(theta + params_.phase_offset[leg]) < params_.duty);
        }
        return out;
    }

    //------------------------------------------------------------------------------------
    // @brief 返回当前全局相位 [0,1)；静立时为 0
    //------------------------------------------------------------------------------------
    double GlobalPhase(double t) const { return stepping_ ? PhaseAt(t) : 0.0; }

    //------------------------------------------------------------------------------------
    // @brief 计算摆动相水平五次插值 s(xi)=10xi³-15xi⁴+6xi⁵
    // @details 端点位置分别为 0 和 1，端点一、二阶导数为 0；xi 为相内进度。
    //------------------------------------------------------------------------------------
    static double SwingCurve(double xi) {
        return xi * xi * xi * (10.0 + xi * (-15.0 + 6.0 * xi));
    }

    //------------------------------------------------------------------------------------
    // @brief 计算水平插值对相内进度 xi 的导数；再乘 dxi/dt 才是时间导数
    //------------------------------------------------------------------------------------
    static double SwingCurveDerivative(double xi) {
        return 30.0 * xi * xi * (1.0 - xi) * (1.0 - xi);  // 30ξ²−60ξ³+30ξ⁴
    }

    //------------------------------------------------------------------------------------
    // @brief 计算余弦抬脚高度 H/2*(1-cos(2πxi)) [m]
    //------------------------------------------------------------------------------------
    static double SwingHeightCurve(double xi, double H) {
        return 0.5 * H * (1.0 - std::cos(2.0 * kPi * xi));
    }

    //------------------------------------------------------------------------------------
    // @brief 计算抬脚高度对 xi 的导数，供竖直足速换算
    //------------------------------------------------------------------------------------
    static double SwingHeightDerivative(double xi, double H) {
        return kPi * H * std::sin(2.0 * kPi * xi);
    }


private:
    //------------------------------------------------------------------------------------
    // @brief 由 Lite3 默认站立角计算四腿标称足位
    //------------------------------------------------------------------------------------
    void InitNominal() {
        SetNominalFromJointAngles(Lite3Model::DefaultStandJointAngles());
    }

    //------------------------------------------------------------------------------------
    // @brief 将绝对时间换算为当前周期中的全局相位
    //------------------------------------------------------------------------------------
    double PhaseAt(double t) const { return Wrap01((t - t_start_) / params_.period); }

    //------------------------------------------------------------------------------------
    // @brief 把实数相位折叠到 [0,1)，支持负输入
    //------------------------------------------------------------------------------------
    static double Wrap01(double x) {
        x -= std::floor(x);
        return x;
    }

    // 内部状态：当前步态、外部调度表、标称足位，以及起步和软停步缓存。
    GaitParams params_;                                         // 当前实际生成轨迹所用的参数，可由调度表动态替换
    const SegmentedGaitParams* schedule_ = nullptr;             // 非拥有型调度表指针；nullptr 表示固定参数
    Lite3Model model_;                                          // 由站姿关节角计算四腿标称球心位置
    Vec12d nominal_feet_ = Vec12d::Zero();                      // FL/FR/HL/HR 四腿标称球心机体系位置 [m]
    bool stepping_ = false;                                     // 是否处于周期步态；false 时四足静立
    double t_start_ = 0.0;                                      // 全局相位零点的绝对时间 [s]
    // 软停步状态：stop_scale_ 为水平步幅系数（1 → 0），stopping_ 表示正在收步。
    bool stopping_ = false;                                     // 是否已经越过相位边界并开始收小水平步幅
    double t_stop_ = 0.0;                                       // 实际开始收步的时间 [s]
    double stop_scale_ = 1.0;                                   // 收步系数，控制水平步幅从 1 降到 0
    // 停步期间锁定的步态参数（防止命令衰减把主方向切到低速段、把收步拉长）。
    bool stop_latched_ = false;                                 // 是否已锁定停步时的速度段参数
    GaitParams stop_params_;                                    // 停步请求到来时保存的步态参数

    //------------------------------------------------------------------------------------
    // @brief 清空软停步标志、步幅缩放和参数锁存
    //------------------------------------------------------------------------------------
    void ResetStopLatch() {
        stopping_ = false;
        stop_scale_ = 1.0;
        stop_latched_ = false;
    }
};

}  // namespace mpc_wbc



