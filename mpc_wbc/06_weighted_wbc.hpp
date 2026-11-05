/*===============================================================================================
 * @file 06_weighted_wbc.hpp
 * @brief 基于全身动力学和加权任务 QP 的 Lite3 全身控制器
 * @details 输入机器人状态、计划支撑集、MPC 基座加速度参考和摆动脚轨迹；输出全身加速度、接触力和关节总力矩。
 * 决策变量共 42 维：[qdd(18); 四脚世界系接触力(12); 关节力矩(12)]。
 * qdd 排列：[世界系基座线加速度(3); 机体系基座角加速度(3); 关节加速度(12)]。
 * 等式固定 30 行：18 行全身动力学，加每腿 3 行支撑接触加速度约束或摆动脚零力约束。
 * 不等式固定 48 行：20 行摩擦金字塔、4 行法向力上限槽位、24 行关节力矩上下界。
 * 不启用的法向力槽位和摆动脚摩擦行用 0>=-1 占位，以保持固定矩阵布局。
 * 软任务通过 H+=w*Aᵀ*A、g-=w*Aᵀ*b 累加；调用 04_qp_ipm.hpp 求解。
 * 这里返回 tau_wbc 总力矩；07 整机控制器再按低层 PD 执行律反解前馈 tau_ff，避免重复叠加 PD。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include "00_lite3_model.hpp"                       // 统一坐标/关节约定及 Lite3 运动学、动力学模型
#include "05_srbd_mpc.hpp"                          // 单刚体预测控制的输入输出和求解接口
#include "04_qp_ipm.hpp"                            // WBC 等式与不等式 QP 的内点求解器

#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <chrono>                                   // 单调时钟、求解耗时和线程休眠时间
#include <cmath>                                    // 三角函数、绝对值和其他数学函数
#include <vector>                                   // 随支撑数量或预测时域变化的动态数组



namespace mpc_wbc {

// 决策变量与行数布局（固定）
constexpr int kWbcQdd = kNumGenVel;                    // 18
constexpr int kWbcNf = kNumContactForces;              // 12
constexpr int kWbcNtau = kNumJoints;                   // 12
constexpr int kWbcNvar = kWbcQdd + kWbcNf + kWbcNtau;  // 42
constexpr int kWbcNEq = kNumGenVel + kNumContactForces;  // 30
constexpr int kWbcNConeRows = 5 * kNumLegs;             // 20
constexpr int kWbcNCapRows = kNumLegs;                  // 4（可选）
constexpr int kWbcNTauRows = 2 * kNumJoints;            // 24
constexpr int kWbcNGe = kWbcNConeRows + kWbcNCapRows + kWbcNTauRows;



//===============================================================================================
// @brief WBC 任务权重、反馈增益、物理限幅和求解配置
//===============================================================================================
struct WbcParams {
    // 任务权重：数值越大越优先降低对应残差；各任务同时求解，不构成严格层级优先。
    double w_base = 1.0;                                        // 基座 6 维加速度任务权重
    double w_swing = 100.0;                                     // 摆动脚任务空间 PD 权重；当前主线高于基座任务权重
    double w_f_ref = 0.0;                                       // MPC 逐脚力参考跟踪权重；0=关闭，以免与基座任务竞争
    double w_posture = 0.0;                                     // 摆动腿关节位置参考权重；0=关闭，避免与足端任务竞争
    double w_posture_stance = 0.0;                              // 支撑腿关节姿势任务权重；当前主线关闭
    double kp_posture_stance = 20.0;                            // 支撑腿姿势任务位置增益 [1/s²]，仅启用该任务时生效
    double kd_posture_stance = 5.0;                             // 支撑腿姿势任务速度增益 [1/s]，仅启用该任务时生效
    double w_tau = 0.0;                                         // 关节力矩二次惩罚权重；0=关闭，非零会与跟踪任务折中

    // 任务空间和关节姿势增益。
    double kp_swing = 350.0;                                    // 摆动脚位置误差到加速度目标的增益 [1/s²]
    double kd_swing = 37.0;                                     // 摆动脚速度误差到加速度目标的增益 [1/s]
    double kp_posture = 25.0;                                   // 摆动腿关节姿势任务位置增益 [1/s²]
    double kd_posture = 2.0;                                    // 保留的姿势阻尼参数，当前摆动腿任务未使用此字段

    // 物理约束和足端目标限幅。
    double mu = 0.6;                                            // 支撑脚摩擦金字塔系数
    double tau_max = 30.0;                                      // 每个关节力矩绝对值上限 [N·m]
    double foot_force_max = 200.0;                              // 支撑脚法向力上限 [N]；非正时使用恒满足行关闭该限制
    double gravity = 9.81;                                      // 动力学偏置力使用的重力大小 [m/s²]，沿世界系 -z
    double contact_accel_max = 200.0;                           // 足端反馈/摆动加速度目标各分量的限幅 [m/s²]

    // 可选支撑反馈，目标速度采用输入的规划值；当前主线增益均为 0。
    double kv_stance_vel = 0.0;                                 // 支撑接触等式中的额外规划速度跟踪增益 [1/s]；0=关闭
    double kp_stance_pos = 0.0;                                 // 支撑接触等式中对规划世界足位的位置反馈增益 [1/s²]
    double kd_stance_pos = 0.0;                                 // 支撑接触等式中对规划世界足速的速度反馈增益 [1/s]

    // 求解配置与短时失败兜底。
    double reg = 1e-8;                                          // 代价 Hessian 的对角正则，改善数值求解条件
    QpIpmOptions qp;                                            // 原-对偶内点法的迭代精度和数值正则配置
    double max_solution_age = 0.01;                             // 本帧求解失败时允许使用旧结果的最长时间 [s]
};



//===============================================================================================
// @brief 一帧全身控制输入：实际状态、接触计划和各任务目标
//===============================================================================================
struct WbcInput {
    // 当前运动学/动力学状态。
    Vec12d q = Vec12d::Zero();                                  // 12 个实际关节角 [rad]
    Vec18d qvel = Vec18d::Zero();                               // 广义速度 [v_world;omega_body;dq]，18 维
    Mat3d R_wb = Mat3d::Identity();                             // 机体系到世界系的旋转矩阵
    std::array<bool, kNumLegs> stance = {{true, true, true, true}};  // 四腿计划支撑标志，顺序 FL/FR/HL/HR
    std::array<Vec3d, kNumLegs> foot_pos_world{};               // 当前球心世界系位置 [m]
    std::array<Vec3d, kNumLegs> foot_vel_world{};               // 当前输入的接触点世界系速度 [m/s]，主线由 Jc*qvel 得到

    // 足端规划目标；各位置/速度输入的几何定义由调用者保持一致。
    std::array<Vec3d, kNumLegs> foot_des_world{};               // 摆动脚球心世界系目标 [m]；支撑腿忽略此字段
    std::array<Vec3d, kNumLegs> foot_vel_des_world{};           // 规划世界系足速 [m/s]，供摆动任务和可选支撑速度反馈共用
    std::array<Vec3d, kNumLegs> foot_pos_des_world{};           // 规划世界系足位 [m]，供可选支撑位置反馈使用

    // 基座参考和可选软任务输入。
    Vec3d a_des_world = Vec3d::Zero();                          // MPC 净力换算的基座世界系线加速度参考 [m/s²]
    Vec3d alpha_des_body = Vec3d::Zero();                       // MPC 净力矩换算的机体系角加速度参考 [rad/s²]
    std::array<Vec3d, kNumLegs> foot_force_ref{};               // MPC 第 0 节点逐脚世界系力参考 [N]，仅 w_f_ref>0 时跟踪
    Vec12d q_nominal = Lite3Model::DefaultStandJointAngles();   // 可选关节姿势任务参考 [rad]；主线传步态 IK 目标
};



//===============================================================================================
// @brief 全身 QP 结果、可用性和残差诊断
//===============================================================================================
struct WbcOutput {
    enum class Status { kOk, kStale, kFailed };
    Status status = Status::kFailed;                            // kOk=本次求解成功；kStale=短时旧解；kFailed=无可用结果

    // 控制结果。
    Vec18d qdd = Vec18d::Zero();                                // 18 维广义加速度，坐标排列与文件头一致
    std::array<Vec3d, kNumLegs> foot_force{};                   // 四脚世界系接触力 [N]
    Vec12d tau = Vec12d::Zero();                                // 12 个关节总力矩 [N·m]，不是前馈 tau_ff

    // 诊断量：新解按当前输入重算；返回 kStale 时沿用旧帧诊断。
    double eom_residual = 0.0;                                  // 全身动力学等式残差的无穷范数
    double contact_residual = 0.0;                              // 支撑按 (Jc*qdd+Jdotv)-接触等式右端项计算；摆动按零力误差计算
    double cone_violation = 0.0;                                // 支撑脚五条摩擦金字塔约束的最大正违反值
    double tau_abs_max = 0.0;                                   // 关节总力矩最大绝对值 [N·m]
    double base_accel_error = 0.0;                              // 求解基座 6 维加速度与输入参考的最大分量误差
    double swing_accel_error = 0.0;                             // 摆动软任务残差，包含与 QP 相同的 Jdot*qvel 和目标限幅
    int iters = 0;                                              // 内点求解迭代次数
    double qp_gap = 0.0;                                        // 内点法平均互补量
    double qp_dual_res = 0.0;                                   // 内点法平稳性残差
    double solve_ms = 0.0;                                      // 本帧求解或返回旧解所耗时间 [ms]

    //------------------------------------------------------------------------------------
    // @brief 判断是否有可用 WBC 结果，包含未过期的旧结果
    //------------------------------------------------------------------------------------
    bool ok() const { return status != Status::kFailed; }

    //------------------------------------------------------------------------------------
    // @brief 判断当前返回的结果是否为本次新解
    //------------------------------------------------------------------------------------
    bool fresh() const { return status == Status::kOk; }
};



//------------------------------------------------------------------------------------
// @brief 将 MPC 合力和合力矩换算为 WBC 基座加速度参考
// @param mpc_out 首节点世界系净力/净力矩；mass 为整机质量 [kg]。
// @param inertia_com_world 绕整机质心的世界系惯量；omega_body 为输入的机体系角速度。
// @return 可选输出指针分别接收世界系线加速度和机体系角加速度。
// @details 线加速度按 F/m+gravity 计算；角加速度沿用当前惯量/陀螺项换算，再左乘 R_wbᵀ。
// @details 使用单刚体近似，未补偿基座原点相对整机质心偏置引起的加速度差。
//------------------------------------------------------------------------------------
inline void ComputeBaseAccelDesired(const SrbdMpcOutput& mpc_out, const Mat3d& inertia_com_world,
                                    const Vec3d& omega_body, const Mat3d& R_wb, double mass,
                                    double gravity, Vec3d* a_world_out, Vec3d* alpha_body_out) 
{
    const Vec3d I_omega = inertia_com_world * omega_body;
    const Vec3d alpha_world =
        inertia_com_world.ldlt().solve(mpc_out.net_moment - omega_body.cross(I_omega));
    if (a_world_out) *a_world_out = mpc_out.net_force / mass + Vec3d(0.0, 0.0, -gravity);
    if (alpha_body_out) *alpha_body_out = R_wb.transpose() * alpha_world;
}



//===============================================================================================
// @brief 组装并求解全身动力学 QP，维护初值和短时旧解缓存
//===============================================================================================
class WeightedWbc 
{
public:
    //------------------------------------------------------------------------------------
    // @brief 保存 WBC 参数并初始化内部动力学模型
    //------------------------------------------------------------------------------------
    explicit WeightedWbc(const WbcParams& params = WbcParams())
        : params_(params), model_() {}

    //------------------------------------------------------------------------------------
    // @brief 读取当前全身控制参数
    //------------------------------------------------------------------------------------
    const WbcParams& params() const { return params_; }

    //------------------------------------------------------------------------------------
    // @brief 替换任务和求解参数；需要清空有效结果缓存时另行调用 Reset
    //------------------------------------------------------------------------------------
    void set_params(const WbcParams& p) { params_ = p; }

    //------------------------------------------------------------------------------------
    // @brief 清空旧结果可用标志和旧解时间，不重置全部初值存储
    //------------------------------------------------------------------------------------
    void Reset() {
        have_solution_ = false;
        last_solution_time_ = 0.0;
        last_output_ = WbcOutput();
    }

    //===============================================================================================
    // @brief 组装 QP 所需的质量矩阵、偏置力和接触运动学量
    //===============================================================================================
    struct QpTerms {
        Mat18d M;                                               // 全身质量矩阵 18×18
        Vec18d h;                                               // 全身动力学偏置力 18 维
        Mat12x18d Jc;                                           // 四脚接触点雅可比 12×18，世界系
        VecContact12d Jdotv;                                    // 四脚接触点偏置加速度 12 维 [m/s²]
    };

    //===============================================================================================
    // @brief 全身 QP 的标准代价、等式、不等式和组装有效标志
    //===============================================================================================
    struct Qp {
        Eigen::MatrixXd H;                                      // 42×42 二次代价矩阵
        Eigen::VectorXd g;                                      // 42 维一次代价项
        Eigen::MatrixXd A_eq;                                   // 30×42 等式系数
        Eigen::VectorXd b_eq;                                   // 30 维等式右端项
        Eigen::MatrixXd A_ge;                                   // 48×42 不等式系数，方向为 >=
        Eigen::VectorXd b_ge;                                   // 48 维不等式右端项
        bool valid = false;                                     // 是否已通过组装保护并生成约束
    };

    //------------------------------------------------------------------------------------
    // @brief 由本帧状态计算动力学量并组装 QP，不执行优化求解
    // @param qp 接收代价与约束；terms 非空时同时接收中间动力学量。
    // @return 无支撑腿时返回 false；调用者应先判断返回值再使用输出。
    //------------------------------------------------------------------------------------
    bool BuildQp(const WbcInput& in, Qp* qp, QpTerms* terms = nullptr) const {

        // 步骤 1：计算全身质量矩阵 M、偏置力 h、接触雅可比 Jc 与偏置加速度 Jdot*qvel。
        QpTerms t;
        t.M = model_.MassMatrix(in.q, in.R_wb);
        const double gravity_vec[3] = {0.0, 0.0, -params_.gravity};
        t.h = model_.BiasForce(in.q, in.qvel, in.R_wb,
                               Vec3d(gravity_vec[0], gravity_vec[1], gravity_vec[2]));
        t.Jc = model_.ContactJacobianWorld(in.q, in.R_wb);
        t.Jdotv = model_.ContactBiasAccel(in.q, in.R_wb, in.qvel);
        if (!AssembleQp(in, t, qp)) return false;
        if (terms) *terms = t;
        return true;
    }

    //------------------------------------------------------------------------------------
    // @brief 计算一帧 WBC，失败时尝试返回未过期的旧解
    // @param t 本帧采样时间 [s]，决定旧解是否仍在 max_solution_age 范围内。
    //------------------------------------------------------------------------------------
    WbcOutput Solve(double t, const WbcInput& in) {
        const auto t0 = std::chrono::steady_clock::now();
        WbcOutput out = SolveOnce(in);
        if (!out.fresh()) return Finish(out, t0, t);

        have_solution_ = true;
        last_solution_time_ = t;
        last_output_ = out;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    //------------------------------------------------------------------------------------
    // @brief 执行一次 QP 组装与求解，不执行旧解兜底
    // @details 原始变量初值可用上一解；只有内点求解器报告收敛才提取结果并重算物理诊断。
    // @details 函数为 const，但 mutable 初值缓存会更新；同一对象不应被多个线程同时求解。
    //------------------------------------------------------------------------------------
    WbcOutput SolveOnce(const WbcInput& in) const {
        WbcOutput out;
        Qp qp;
        QpTerms terms;
        if (!BuildQp(in, &qp, &terms)) {
            // 保护：一条支撑腿都没有时，「接触力=0 的自由落体解」在数学上仍然可行，但对控制无意义
            // （参考工程同样显式拒绝）。置 kFailed，由上层的失败兜底处理。
            out.status = WbcOutput::Status::kFailed;
            return out;
        }
        const Mat18d& M = terms.M;
        const Vec18d& h = terms.h;
        const Mat12x18d& Jc = terms.Jc;
        const VecContact12d& Jdotv = terms.Jdotv;
        const WbcParams& p = params_;
        QpProblem qp_problem;
        qp_problem.H = qp.H;
        qp_problem.g = qp.g;
        qp_problem.A_eq = qp.A_eq;
        qp_problem.b_eq = qp.b_eq;
        qp_problem.A_ge = qp.A_ge;
        qp_problem.b_ge = qp.b_ge;

        // 步骤 2：把组装结果交给内点法，按配置提供上一原始解作为可选初值。
        const QpIpmResult res = SolveQpIpm(qp_problem, p.qp, warm_start_ ? &last_x_ref_ : nullptr);

        out.iters = res.info.iters;
        out.qp_gap = res.info.gap;
        out.qp_dual_res = res.info.dual_res;
        if (!res.ok()) {
            out.status = WbcOutput::Status::kFailed;
            return out;
        }

        // 步骤 3：按 [qdd;接触力;关节力矩] 布局拆解 QP 结果，并保存下一帧初值。
        out.qdd = res.x.head<kWbcQdd>();
        for (int leg = 0; leg < kNumLegs; ++leg)
            out.foot_force[leg] = res.x.segment<3>(kWbcQdd + 3 * leg);
        out.tau = res.x.segment<kWbcNtau>(kWbcQdd + kWbcNf);

        last_x_ref_ = res.x;  // 下一帧的热启动初值

        // 步骤 4：用求解结果和原始动力学量计算动力学、接触、摩擦和任务残差。
        Vec18d tau_gen = Vec18d::Zero();
        tau_gen.tail<kWbcNtau>() = out.tau;
        Eigen::VectorXd f_all(kWbcNf);
        for (int leg = 0; leg < kNumLegs; ++leg) f_all.segment<3>(3 * leg) = out.foot_force[leg];
        out.eom_residual =
            (M * out.qdd + h - Jc.transpose() * f_all - tau_gen).cwiseAbs().maxCoeff();

        double contact_res = 0.0, cone_viol = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d f = out.foot_force[leg];
            if (in.stance[leg]) {
                const Vec3d acc = Jc.block<3, kWbcQdd>(3 * leg, 0) * out.qdd +
                                  Jdotv.segment<3>(3 * leg);
                const Vec3d target =
                    StanceAccelTarget(in, p, Jdotv.segment<3>(3 * leg), leg) -
                    p.kv_stance_vel * (in.foot_vel_world[leg] - in.foot_vel_des_world[leg]);
                contact_res = std::max(contact_res, (acc - target).cwiseAbs().maxCoeff());
                const double viol[5] = {-f.z(),        f.x() - p.mu * f.z(), -f.x() - p.mu * f.z(),
                                        f.y() - p.mu * f.z(), -f.y() - p.mu * f.z()};
                for (double v : viol) cone_viol = std::max(cone_viol, v);
            } else {
                contact_res = std::max(contact_res, f.cwiseAbs().maxCoeff());
            }
        }
        out.contact_residual = contact_res;
        out.cone_violation = cone_viol;
        out.tau_abs_max = out.tau.cwiseAbs().maxCoeff();
        Eigen::Matrix<double, 6, 1> base_des;
        base_des << in.a_des_world, in.alpha_des_body;
        out.base_accel_error = (out.qdd.head<6>() - base_des).cwiseAbs().maxCoeff();
        double swing_err = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            const Vec3d err = in.foot_des_world[leg] - in.foot_pos_world[leg];
            const Vec3d edot = in.foot_vel_des_world[leg] - in.foot_vel_world[leg];

            // 摆动任务诊断使用与组装 QP 相同的偏置补偿和限幅，便于比较权重变化。
            Vec3d target = p.kp_swing * err + p.kd_swing * edot - Jdotv.segment<3>(3 * leg);
            for (int c = 0; c < 3; ++c)
                target(c) = std::min(std::max(target(c), -p.contact_accel_max), p.contact_accel_max);
            const Vec3d acc = Jc.block<3, kWbcQdd>(3 * leg, 0) * out.qdd;
            swing_err = std::max(swing_err, (acc - target).cwiseAbs().maxCoeff());
        }
        out.swing_accel_error = swing_err;

        out.status = WbcOutput::Status::kOk;
        return out;
    }

    //------------------------------------------------------------------------------------
    // @brief 把动力学硬约束和加权软任务写入固定维数的 QP
    // @param in 本帧状态和任务目标；t 为提前计算的 M、h、Jc、Jdot*qvel。
    // @details 等式方向为 A_eq*x=b_eq，不等式方向为 A_ge*x>=b_ge；无支撑时返回 false。
    //------------------------------------------------------------------------------------
    bool AssembleQp(const WbcInput& in, const QpTerms& t, Qp* qp) const {
        const WbcParams& p = params_;

        // 步骤 1：拒绝全摆动输入；自由落体零接触力解不属于当前控制器的有效支撑状态。
        bool any_stance = false;
        for (int leg = 0; leg < kNumLegs; ++leg) any_stance = any_stance || in.stance[leg];
        if (!any_stance) return false;
        qp->valid = true;
        const Mat18d& M = t.M;
        const Mat12x18d& Jc = t.Jc;
        const VecContact12d& Jdotv = t.Jdotv;

        // 步骤 2：组装 30 行等式，包括全身动力学和各腿支撑/零力条件。
        Eigen::MatrixXd& A_eq = qp->A_eq = Eigen::MatrixXd::Zero(kWbcNEq, kWbcNvar);
        Eigen::VectorXd& b_eq = qp->b_eq = Eigen::VectorXd::Zero(kWbcNEq);
        // EoM: M q̈ − J_allᵀ f − Sᵀ τ = −h
        A_eq.block<kWbcQdd, kWbcQdd>(0, 0) = M;
        A_eq.block<kWbcQdd, kWbcNf>(0, kWbcQdd) = -Jc.transpose();

        // S 为 12×18 的关节选择矩阵 [0 I]；Sᵀ 的非零块在广义动力学行 6..17。
        A_eq.block<kWbcNtau, kWbcNtau>(kWbcQdd - kWbcNtau, kWbcQdd + kWbcNf) =
            -Eigen::MatrixXd::Identity(kWbcNtau, kWbcNtau);
        b_eq.head<kWbcQdd>() = -t.h;
        // 每腿 3 行：支撑 → Jc q̈ = −Jdotc q̇；摆动 → f_i = 0
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int r = kWbcQdd + 3 * leg;
            if (in.stance[leg]) {
                A_eq.block<3, kWbcQdd>(r, 0) = Jc.block<3, kWbcQdd>(3 * leg, 0);
                b_eq.segment<3>(r) =
                    StanceAccelTarget(in, p, Jdotv.segment<3>(3 * leg), leg) -
                    p.kv_stance_vel * (in.foot_vel_world[leg] - in.foot_vel_des_world[leg]);
            } else {
                A_eq.block<3, 3>(r, kWbcQdd + 3 * leg).setIdentity();
            }
        }

        // 步骤 3：组装 >= 形式的不等式，按摩擦金字塔、法向力上限、力矩上下界固定排布。
        Eigen::MatrixXd& A_ge = qp->A_ge = Eigen::MatrixXd::Zero(kWbcNGe, kWbcNvar);
        Eigen::VectorXd& b_ge = qp->b_ge = Eigen::VectorXd::Zero(kWbcNGe);
        const double mu = p.mu;
        // 摩擦金字塔：fz ≥ 0；μfz − fx ≥ 0；μfz + fx ≥ 0；μfz − fy ≥ 0；μfz + fy ≥ 0
        const double cone[5][3] = {{0.0, 0.0, 1.0},
                                   {-1.0, 0.0, mu},
                                   {1.0, 0.0, mu},
                                   {0.0, -1.0, mu},
                                   {0.0, 1.0, mu}};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int r0 = 5 * leg;
            if (in.stance[leg]) {
                for (int k = 0; k < 5; ++k)
                    for (int c = 0; c < 3; ++c)
                        A_ge(r0 + k, kWbcQdd + 3 * leg + c) = cone[k][c];
            } else {
                // 空行：0 ≥ −1，恒满足；保持行数固定且不产生 0/0（s = 1 远离边界）
                for (int k = 0; k < 5; ++k) b_ge(r0 + k) = -1.0;
            }
        }
        // 单脚法向力上限
        const int r_cap = kWbcNConeRows;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (p.foot_force_max > 0.0 && in.stance[leg]) {
                A_ge(r_cap + leg, kWbcQdd + 3 * leg + 2) = -1.0;
                b_ge(r_cap + leg) = -p.foot_force_max;
            } else {
                b_ge(r_cap + leg) = -1.0;  // 空行
            }
        }
        // 关节力矩上下界：τ_j ≥ −τ_max 与 −τ_j ≥ −τ_max
        const int r_tau = kWbcNConeRows + kWbcNCapRows;
        for (int j = 0; j < kWbcNtau; ++j) {
            const int col = kWbcQdd + kWbcNf + j;
            A_ge(r_tau + j, col) = 1.0;
            A_ge(r_tau + kWbcNtau + j, col) = -1.0;
            b_ge(r_tau + j) = -p.tau_max;
            b_ge(r_tau + kWbcNtau + j) = -p.tau_max;
        }

        // 步骤 4：清空代价矩阵，通过 add_task 累加各项 w*Aᵀ*A 和 -w*Aᵀ*b。
        Eigen::MatrixXd& H = qp->H = Eigen::MatrixXd::Zero(kWbcNvar, kWbcNvar);
        Eigen::VectorXd& g = qp->g = Eigen::VectorXd::Zero(kWbcNvar);
        auto add_task = [&](const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double w) {
            if (w <= 0.0) return;
            H.noalias() += w * (A.transpose() * A);
            g.noalias() -= w * (A.transpose() * b);
        };

        // 任务 1：跟踪世界系线加速度与机体系角加速度组成的基座 6 维参考。
        {
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, kWbcNvar);
            A.block<6, 6>(0, 0).setIdentity();
            Eigen::VectorXd b(6);
            b << in.a_des_world, in.alpha_des_body;
            add_task(A, b, p.w_base);
        }

        // 任务 2：摆动脚笛卡尔 PD，目标为 kp*位置误差+kd*速度误差-Jdot*qvel。
        // 减去偏置项后，代价中的 Jc*qdd 才与实际足端加速度目标一致。
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, kWbcNvar);
            A.block<3, kWbcQdd>(0, 0) = Jc.block<3, kWbcQdd>(3 * leg, 0);
            const Vec3d err = in.foot_des_world[leg] - in.foot_pos_world[leg];
            const Vec3d edot = in.foot_vel_des_world[leg] - in.foot_vel_world[leg];
            Vec3d b = p.kp_swing * err + p.kd_swing * edot - Jdotv.segment<3>(3 * leg);
            for (int c = 0; c < 3; ++c)  // 限幅，防大误差时任务目标爆冲
                b(c) = std::min(std::max(b(c), -p.contact_accel_max), p.contact_accel_max);
            add_task(A, b, p.w_swing);
        }

        // 任务 3：可选地跟踪 MPC 逐脚力参考；仅对支撑腿添加。
        if (p.w_f_ref > 0.0) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.stance[leg]) continue;
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, kWbcNvar);
                A.block<3, 3>(0, kWbcQdd + 3 * leg).setIdentity();
                add_task(A, in.foot_force_ref[leg], p.w_f_ref);
            }
        }

        // 任务 4：可选摆动腿关节位置参考；当前仅使用 kp_posture，权重默认关闭。
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (in.stance[leg]) continue;
            for (int k = 0; k < 3; ++k) {
                const int j = 3 * leg + k;
                const double b = p.kp_posture * (in.q_nominal(j) - in.q(j));
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, kWbcNvar);
                A(0, 6 + j) = 1.0;
                Eigen::VectorXd bv = Eigen::VectorXd::Constant(1, b);
                add_task(A, bv, p.w_posture);
            }
        }

        // 任务 5：可选支撑腿关节姿势反馈，当前主线权重为 0。
        // 此分支速度项沿用现有 in.qvel(j) 索引；它并不等同于统一使用尾部 12 维关节速度。
        if (p.w_posture_stance > 0.0) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.stance[leg]) continue;
                for (int k = 0; k < 3; ++k) {
                    const int j = 3 * leg + k;
                    const double b = p.kp_posture_stance * (in.q_nominal(j) - in.q(j)) -
                                     p.kd_posture_stance * in.qvel(j);
                    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, kWbcNvar);
                    A(0, 6 + j) = 1.0;
                    Eigen::VectorXd bv = Eigen::VectorXd::Constant(1, b);
                    add_task(A, bv, p.w_posture_stance);
                }
            }
        }

        // 任务 6：可选关节力矩二次正则，与其他任务共同折中，不属于严格零空间投影。
        if (p.w_tau > 0.0) {
            for (int j = 0; j < kWbcNtau; ++j) {
                const int col = kWbcQdd + kWbcNf + j;
                H(col, col) += p.w_tau;
            }
        }

        // 步骤 5：对全部变量加入对角正则，完成 QP 代价组装。
        for (int i = 0; i < kWbcNvar; ++i) H(i, i) += p.reg;
        return true;
    }

    //------------------------------------------------------------------------------------
    // @brief 控制是否向内点法传入上一解的 x 初值；不复用完整原-对偶状态
    //------------------------------------------------------------------------------------
    void set_warm_start_enabled(bool en) { warm_start_ = en; }


private:
    //------------------------------------------------------------------------------------
    // @brief 构造支撑接触等式中的 -Jdot*qvel 与可选位置/速度反馈
    // @details 位置和速度误差相对输入的规划足位/足速计算；额外 kv_stance_vel 项由调用处再减去。
    // @return 加入反馈后的接触等式右端项 [m/s²]；反馈限幅不作用于 -Jdot*qvel 本身。
    //------------------------------------------------------------------------------------
    static Vec3d StanceAccelTarget(const WbcInput& in, const WbcParams& p,
                                   const Vec3d& jdotv, int leg) {
        Vec3d b = -jdotv;
        if (p.kp_stance_pos != 0.0 || p.kd_stance_pos != 0.0) {
            Vec3d fb = p.kp_stance_pos * (in.foot_pos_des_world[leg] - in.foot_pos_world[leg]) +
                       p.kd_stance_pos *
                           (in.foot_vel_des_world[leg] - in.foot_vel_world[leg]);
            for (int c = 0; c < 3; ++c)
                fb(c) = std::min(std::max(fb(c), -p.contact_accel_max), p.contact_accel_max);
            b += fb;
        }
        return b;
    }

    //------------------------------------------------------------------------------------
    // @brief 计算当前求解耗时 [ms]，使用单调时钟
    //------------------------------------------------------------------------------------
    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    //------------------------------------------------------------------------------------
    // @brief 无新解时返回有效期内的历史结果，否则报告失败
    // @details 旧解的力矩、接触力和诊断仍属于生成它的旧帧，由上层决定是否接受。
    //------------------------------------------------------------------------------------
    WbcOutput Finish(WbcOutput& out, const std::chrono::steady_clock::time_point& t0, double t) {
        if (have_solution_ && (t - last_solution_time_) <= params_.max_solution_age) {
            WbcOutput stale = last_output_;
            stale.status = WbcOutput::Status::kStale;
            stale.solve_ms = ElapsedMs(t0);
            return stale;
        }
        out.status = WbcOutput::Status::kFailed;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    // 内部状态：配置、模型、可选原始解初值，以及带有效期的上一帧成功结果。
    WbcParams params_;                                          // 软任务、物理约束和求解精度配置
    Lite3Model model_;                                          // 用于全身动力学和接触运动学的内部模型
    bool warm_start_ = true;                                    // 是否将上一次原始解传给内点法初始化 x
    mutable Eigen::VectorXd last_x_ref_;                        // 上一次 QP 原始变量；SolveOnce 为 const 但会更新此初值
    bool have_solution_ = false;                                // 是否存在可供 Solve 失败时短时复用的成功结果
    double last_solution_time_ = 0.0;                           // 成功结果的采样时间 [s]
    WbcOutput last_output_;                                     // 成功结果缓存，旧解复用时不按新测量重算诊断
};

}  // namespace mpc_wbc




