/*===============================================================================================
 * @file 05_srbd_mpc.hpp
 * @brief Lite3 单刚体滚动预测控制：优化未来支撑接触力
 * @details 将整机近似为单刚体，在 N 个等时间节点上预测质心位置/速度、欧拉角和机体系角速度。
 * 状态 x=[p_com_world(3); v_com_world(3); rpy(3); omega_body(3)]，共 12 维。
 * 每个支撑脚/预测节点使用四个非负摩擦锥生成系数 lambda；摆动脚不生成受力变量。
 * 生成方向为 (±mu,0,1)、(0,±mu,1)，得到保守的 |fx|+|fy|<=mu*fz 菱形截面，
 * 它包含于 WBC 使用的 |fx|<=mu*fz、|fy|<=mu*fz 方形金字塔之内，二者并不等价。
 * 建立自由轨迹和接触力响应矩阵后，调用 03_box_qp.hpp 求解；输出第 0 节点的逐脚力和净力矩。
 * 模型在一个预测时域内冻结部分姿态/接触位置量；结果不是完整多刚体动力学的精确预测。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include "00_lite3_model.hpp"                       // 统一坐标/关节约定及 Lite3 运动学、动力学模型
#include "03_box_qp.hpp"                            // MPC 接触力系数的盒约束 QP 求解器

#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <chrono>                                   // 单调时钟、求解耗时和线程休眠时间
#include <cmath>                                    // 三角函数、绝对值和其他数学函数
#include <unordered_map>                            // 上一解变量索引的哈希查找表
#include <vector>                                   // 随支撑数量或预测时域变化的动态数组



namespace mpc_wbc {
/// 每个摩擦锥的生成方向数（4 → 四棱锥）。
constexpr int kConeGenerators = 4;



//===============================================================================================
// @brief 单刚体预测模型、跟踪代价与求解配置
//===============================================================================================
struct SrbdMpcParams {
    // 预测模型与接触力约束。
    int N = 10;                                                 // 预测节点数量，实际长度不超过输入 contact 的长度
    double dt = 0.05;                                           // 预测节点间隔 [s]，总预测时长约为 N*dt
    double gravity = 9.81;                                      // 重力加速度大小 [m/s²]，重力沿世界系 -z
    double mu = 0.6;                                            // 摩擦锥生成方向使用的摩擦系数
    double foot_force_max = 200.0;                              // 单脚法向力上限 [N]，按生成方向数量平均限制 lambda 上界

    // 状态跟踪权重；各项单位不同，权重应配合状态量级理解。
    double q_p_xy = 10.0;                                       // 质心水平位置误差权重
    double q_p_z = 200.0;                                       // 质心竖直位置误差权重
    double q_v_xy = 10.0;                                       // 水平速度误差权重
    double q_v_z = 10.0;                                        // 竖直速度误差权重
    double q_rp = 50.0;                                         // 滚转/俯仰角误差权重
    double q_yaw = 20.0;                                        // 偏航角误差权重
    double q_w_rp = 5.0;                                        // 机体系 x/y 角速度误差权重
    double q_w_yaw = 2.0;                                       // 机体系 z 角速度误差权重

    // 数值求解及旧解复用。
    double r_lambda = 1e-6;                                     // 力生成系数的对角正则；过大会压低承重力
    BoxQpOptions qp;                                            // 盒约束 QP 迭代和精度配置
    bool warm_start = true;                                     // 是否按节点/腿/生成方向索引复用上次 lambda 初值
    double max_solution_age = 0.2;                              // 本帧失败时允许沿用上次有效结果的最长时间 [s]
};



//===============================================================================================
// @brief 单次 MPC 求解所需的状态、惯量、参考和接触计划
//===============================================================================================
struct SrbdMpcInput 
{
    // 本帧状态和模型量。
    Vec3d p_com = Vec3d::Zero();                                // 当前整机质心世界系位置 [m]
    Vec3d v_com = Vec3d::Zero();                                // 当前整机质心世界系速度 [m/s]
    Vec3d rpy = Vec3d::Zero();                                  // 当前 ZYX 欧拉角 [rad]；yaw 可使用与参考一致的连续分支
    Vec3d omega = Vec3d::Zero();                                // 当前机体系角速度 [rad/s]
    Mat3d R_wb = Mat3d::Identity();                             // 机体系到世界系旋转，必须与 rpy 表示同一姿态
    Mat3d inertia_com_world = Mat3d::Zero();                    // 绕整机质心的世界系惯量 [kg·m²]
    std::array<Vec3d, kNumLegs> foot_pos_world{};               // 四腿接触力作用点的世界系位置 [m]

    // 速度与高度参考；yaw_ref 与 rpy.z() 应采用一致的连续角度分支。
    Vec3d v_cmd = Vec3d::Zero();                                // 期望世界系质心速度 [m/s]，主线只给水平速度
    double yaw_rate_cmd = 0.0;                                  // 期望偏航角速度 [rad/s]
    double yaw_ref = 0.0;                                       // 独立积分的连续偏航角参考 [rad]，不能每帧替换为测量偏航
    double height_cmd = 0.0;                                    // 期望质心世界系高度 [m]

    // 未来接触计划，由步态模块的同一相位源生成。
    std::vector<std::array<bool, kNumLegs>> contact;            // 每个预测节点的四腿支撑标志，长度宜不小于 N
};



//===============================================================================================
// @brief MPC 首节点接触力、预测轨迹及求解状态
//===============================================================================================
struct SrbdMpcOutput 
{
    enum class Status { kOk, kStale, kFailed };
    Status status = Status::kFailed;                            // kOk=本次计算可用；kStale=复用旧解；kFailed=无可用结果

    // 提供给整机控制器/WBC 的首节点结果。
    std::array<Vec3d, kNumLegs> foot_force{};                   // 第 0 预测节点的逐脚接触力，世界系 [N]
    Vec3d net_force = Vec3d::Zero();                            // 第 0 节点四脚接触力之和，世界系 [N]
    Vec3d net_moment = Vec3d::Zero();                           // 第 0 节点接触力绕质心的合力矩，世界系 [N·m]

    // 预测轨迹，用于观察模型响应；旧解复用时轨迹也来自旧帧。
    std::vector<Vec3d> pred_p;                                  // 由本次 lambda 重建的预测质心世界系位置 [m]
    std::vector<Vec3d> pred_v;                                  // 预测质心世界系线速度 [m/s]
    std::vector<Vec3d> pred_rpy;                                // 预测 ZYX 欧拉角 [rad]
    std::vector<Vec3d> pred_omega;                              // 预测角速度状态 [rad/s]，排列沿用机体系状态定义

    // 数值求解诊断；当前实现以解向量维数和有限值作为缓存更新条件。
    int nvar = 0;                                               // 本时域内支撑脚生成系数的总数量
    int iters = 0;                                              // 盒 QP 主动集和坐标下降的迭代数之和
    double qp_residual = 0.0;                                   // 盒 QP 返回的投影梯度残差
    double cost = 0.0;                                          // 0.5*lambdaᵀ*H*lambda+gᵀ*lambda，不含常数项
    double solve_ms = 0.0;                                      // 本次求解或旧解返回所耗时间 [ms]

    //------------------------------------------------------------------------------------
    // @brief 判断是否有可用结果，包含未过期的旧解
    //------------------------------------------------------------------------------------
    bool ok() const { return status != Status::kFailed; }  // 有可用结果（含沿用旧解）

    //------------------------------------------------------------------------------------
    // @brief 判断结果是否来自本次计算
    //------------------------------------------------------------------------------------
    bool fresh() const { return status == Status::kOk; }   // 本帧真正解出
};



//===============================================================================================
// @brief 管理 SRBD 时域 QP、热启动索引和旧解缓存
//===============================================================================================
class SrbdPreviewMpc 
{
public:
    //------------------------------------------------------------------------------------
    // @brief 保存 MPC 参数，并从 Lite3 模型取得整机质量
    //------------------------------------------------------------------------------------
    explicit SrbdPreviewMpc(const SrbdMpcParams& params = SrbdMpcParams())
        : params_(params), mass_(Lite3Model().TotalMass()) {}

    //------------------------------------------------------------------------------------
    // @brief 读取当前预测模型与优化参数
    //------------------------------------------------------------------------------------
    const SrbdMpcParams& params() const { return params_; }

    //------------------------------------------------------------------------------------
    // @brief 更新 MPC 参数；需要清空旧解时由调用者另行 Reset
    //------------------------------------------------------------------------------------
    void set_params(const SrbdMpcParams& p) { params_ = p; }

    //------------------------------------------------------------------------------------
    // @brief 读取当前模型质量 [kg]
    //------------------------------------------------------------------------------------
    double mass() const { return mass_; }

    //------------------------------------------------------------------------------------
    // @brief 设置用于单刚体模型的整机质量 [kg]，应为正数
    //------------------------------------------------------------------------------------
    void set_mass(double m) { mass_ = m; }

    //------------------------------------------------------------------------------------
    // @brief 清空接触力初值、索引和旧解缓存
    //------------------------------------------------------------------------------------
    void Reset() {
        last_lambda_.resize(0);
        last_var_index_.clear();
        have_solution_ = false;
        last_solution_time_ = 0.0;
        last_output_ = SrbdMpcOutput();
    }

    //------------------------------------------------------------------------------------
    // @brief 计算 ZYX 欧拉角速率与机体系角速度的转换矩阵
    // @details rpy_dot=T(rpy)*omega_body；|pitch|>1.5 rad 时返回单位阵，避免奇异位置的数值放大。
    //------------------------------------------------------------------------------------
    static Mat3d EulerRateMatrix(const Vec3d& rpy) {
        const double r = rpy(0), p = rpy(1);
        if (std::fabs(p) > 1.5) return Mat3d::Identity();
        const double cp = std::cos(p), sp = std::sin(p);
        const double cr = std::cos(r), sr = std::sin(r);
        const double tp = sp / cp;
        Mat3d T;
        T << 1.0, sr * tp, cr * tp,
             0.0, cr, -sr,
             0.0, sr / cp, cr / cp;
        return T;
    }

    //------------------------------------------------------------------------------------
    // @brief 建立本帧时域接触力 QP，求解后生成首节点输出与预测轨迹
    // @param t 当前采样时间 [s]，用于旧解有效期判断。
    // @details 本函数会更新内部解缓存，只应由同一 MPC 求解线程调用。
    // @details 当前实现记录 BoxQpInfo 的收敛残差，但不会仅因 converged=false 拒绝有限维数正确的解。
    //------------------------------------------------------------------------------------
    SrbdMpcOutput Solve(double t, const SrbdMpcInput& in) 
    {
        // 步骤 1：检查预测时域、质量和节点间隔。
        const auto t0 = std::chrono::steady_clock::now();
        SrbdMpcOutput out;
        const int N = std::min(params_.N, static_cast<int>(in.contact.size()));
        const double dt = params_.dt;
        const double m = mass_;
        if (N < 1 || m <= 0.0 || dt <= 0.0) return Finish(out, t0, t);

        // 步骤 2：只为支撑脚建立 [预测节点,腿,生成方向] 接触力变量。
        struct Var {
            int k, leg, gen;
        };
        std::vector<Var> vars;
        vars.reserve(static_cast<size_t>(N) * kNumLegs * kConeGenerators);
        for (int k = 0; k < N; ++k) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!in.contact[k][leg]) continue;  // 摆动脚：不生成变量 → f_i ≡ 0
                for (int g = 0; g < kConeGenerators; ++g) vars.push_back({k, leg, g});
            }
        }
        const int V = static_cast<int>(vars.size());
        out.nvar = V;
        if (V == 0) return Finish(out, t0, t);  // 全摆动：无可行解

        // 步骤 3：生成四个摩擦锥方向；非负 lambda 结构性保证 fz>=0。
        static const double kConeXY[kConeGenerators][2] = {
            {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
        std::array<Vec3d, kConeGenerators> gens;
        for (int g = 0; g < kConeGenerators; ++g) {
            gens[g] = Vec3d(params_.mu * kConeXY[g][0], params_.mu * kConeXY[g][1], 1.0);
        }

        // 步骤 4：建立 12 维离散齐次状态矩阵，并将重力单独写成自由轨迹偏置。
        const Mat3d T = EulerRateMatrix(in.rpy);
        Eigen::Matrix<double, 12, 12> A = Eigen::Matrix<double, 12, 12>::Identity();
        A.block<3, 3>(0, 3) = dt * Mat3d::Identity();
        A.block<3, 3>(6, 9) = dt * T;

        Eigen::Matrix<double, 12, 1> x0;
        x0 << in.p_com, in.v_com, in.rpy, in.omega;

        Eigen::Matrix<double, 12, 1> gravity_term = Eigen::Matrix<double, 12, 1>::Zero();
        gravity_term.segment<3>(0) = Vec3d(0.0, 0.0, -0.5 * dt * dt * params_.gravity);
        gravity_term.segment<3>(3) = Vec3d(0.0, 0.0, -dt * params_.gravity);

        const Mat3d inertia_inv = in.inertia_com_world.inverse();
        if (!inertia_inv.allFinite() || !x0.allFinite()) return Finish(out, t0, t);

        // 步骤 5：不施加足端力时向前递推自由状态，作为 QP 的常数项。
        std::vector<Eigen::Matrix<double, 12, 1>> x_free(N);
        {
            Eigen::Matrix<double, 12, 1> x = x0;
            for (int k = 0; k < N; ++k) {
                x = A * x + gravity_term;
                if (!x.allFinite()) return Finish(out, t0, t);
                x_free[k] = x;
            }
        }

        // 步骤 6：计算每个接触力系数对整个时域状态的影响，形成 12N×V 灵敏度矩阵。
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(12 * N, V);
        for (int v = 0; v < V; ++v) {
            const Var& var = vars[v];
            const Vec3d d = gens[var.gen];
            const Vec3d r = in.foot_pos_world[var.leg] - in.p_com;

            // 坐标转换：接触力矩和惯量先在世界系计算角加速度，再转到机体系角速度状态通道。
            const Vec3d alpha_world = inertia_inv * r.cross(d);
            const Vec3d alpha = in.R_wb.transpose() * alpha_world;  // → 机体系

            Eigen::Matrix<double, 12, 1> B = Eigen::Matrix<double, 12, 1>::Zero();
            B.segment<3>(0) = (0.5 * dt * dt / m) * d;
            B.segment<3>(3) = (dt / m) * d;
            B.segment<3>(6) = 0.5 * dt * dt * (T * alpha);
            B.segment<3>(9) = dt * alpha;

            Eigen::Matrix<double, 12, 1> s = Eigen::Matrix<double, 12, 1>::Zero();
            for (int k = 0; k < N; ++k) {
                s = A * s;               // 先齐次推进
                if (k == var.k) s += B;  // 再叠加该步输入
                if (k >= var.k) S.block<12, 1>(12 * k, v) = s;
            }
        }

        // 步骤 7：组装状态权重、速度/高度/偏航参考，并累加 QP 的 H 和 g。
        Eigen::Matrix<double, 12, 12> W = Eigen::Matrix<double, 12, 12>::Zero();
        W.diagonal() << params_.q_p_xy, params_.q_p_xy, params_.q_p_z,
            params_.q_v_xy, params_.q_v_xy, params_.q_v_z,
            params_.q_rp, params_.q_rp, params_.q_yaw,
            params_.q_w_rp, params_.q_w_rp, params_.q_w_yaw;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(V, V);
        Eigen::VectorXd gvec = Eigen::VectorXd::Zero(V);
        for (int k = 0; k < N; ++k) {
            const double tk = dt * static_cast<double>(k + 1);
            // 参考位置按命令速度从当前质心推进；高度恒为期望值；姿态水平（yaw 按命令转）。
            Eigen::Matrix<double, 12, 1> ref;
            ref.segment<3>(0) = Vec3d(in.p_com.x() + in.v_cmd.x() * tk,
                                      in.p_com.y() + in.v_cmd.y() * tk, in.height_cmd);
            ref.segment<3>(3) = in.v_cmd;
            ref.segment<3>(6) = Vec3d(0.0, 0.0, in.yaw_ref + in.yaw_rate_cmd * tk);
            ref.segment<3>(9) = Vec3d(0.0, 0.0, in.yaw_rate_cmd);

            const Eigen::Matrix<double, 12, 1> e = x_free[k] - ref;
            const Eigen::MatrixXd Sk = S.block(12 * k, 0, 12, V);
            H.noalias() += Sk.transpose() * W * Sk;
            gvec.noalias() += Sk.transpose() * W * e;
        }
        for (int v = 0; v < V; ++v) H(v, v) += params_.r_lambda;

        // 步骤 8：每个 lambda 限制在 [0,foot_force_max/4]，保证单脚法向力不超过上限。
        const Eigen::VectorXd lb = Eigen::VectorXd::Zero(V);
        const Eigen::VectorXd ub =
            Eigen::VectorXd::Constant(V, params_.foot_force_max / kConeGenerators);

        // 步骤 9：按节点、腿和方向索引查找缓存初值，再交由盒 QP 投影/选用。
        // 当前节点 k 查找旧缓存 max(k-1,0)，这是现有实现的索引规则。
        Eigen::VectorXd warm;
        const Eigen::VectorXd* warm_ptr = nullptr;
        if (params_.warm_start && last_lambda_.size() > 0) {
            warm = Eigen::VectorXd::Zero(V);
            for (int v = 0; v < V; ++v) {
                const Var& var = vars[v];
                const int want_k = (var.k == 0) ? 0 : var.k - 1;
                const auto it = last_var_index_.find(Key(want_k, var.leg, var.gen));
                if (it != last_var_index_.end() && it->second < last_lambda_.size()) {
                    warm(v) = last_lambda_(it->second);
                }
            }
            warm_ptr = &warm;
        }

        // 步骤 10：调用盒约束 QP；维数错误或出现非有限值时进入旧解兜底。
        BoxQpInfo qp_info;
        const Eigen::VectorXd lambda =
            SolveBoxQp(H, gvec, lb, ub, warm_ptr, params_.qp, &qp_info);
        if (lambda.size() != V || !lambda.allFinite()) return Finish(out, t0, t);

        out.iters = qp_info.iters + qp_info.polish;
        out.qp_residual = qp_info.residual;

        // 步骤 11：提取第 0 节点逐脚力，累加得到绕当前质心的合力和合力矩。
        for (int leg = 0; leg < kNumLegs; ++leg) out.foot_force[leg].setZero();
        for (int v = 0; v < V; ++v) {
            if (vars[v].k != 0) continue;
            out.foot_force[vars[v].leg] += lambda(v) * gens[vars[v].gen];
        }
        for (int leg = 0; leg < kNumLegs; ++leg) {
            out.net_force += out.foot_force[leg];
            out.net_moment += (in.foot_pos_world[leg] - in.p_com).cross(out.foot_force[leg]);
        }

        // ---- 预测轨迹：用最优 λ 推进（含 ω×Iω 耦合与姿态相关的力矩换轴）----
        {

            // 步骤 12：使用本次系数和现有角速度耦合项重建诊断预测轨迹。
            // 该轨迹单独计算，不再用于反向修改已组装并求解的 QP。
            const Vec3d I_omega = in.inertia_com_world * in.omega;
            Eigen::Matrix<double, 12, 1> x = x0;
            for (int k = 0; k < N; ++k) {
                Vec3d f = Vec3d::Zero(), moment = Vec3d::Zero();
                for (int v = 0; v < V; ++v) {
                    if (vars[v].k != k) continue;
                    const Vec3d fj = lambda(v) * gens[vars[v].gen];
                    f += fj;
                    moment += (in.foot_pos_world[vars[v].leg] - in.p_com).cross(fj);
                }
                const Mat3d R = RpyToRotation(Eigen::Vector3d(x.segment<3>(6)));
                const Vec3d alpha =
                    inertia_inv * (moment - R.transpose() * in.omega.cross(I_omega));
                const Vec3d a = Vec3d(0.0, 0.0, -params_.gravity) + f / m;
                Eigen::Matrix<double, 12, 1> xn = x;
                xn.segment<3>(0) = x.segment<3>(0) + dt * x.segment<3>(3) + 0.5 * dt * dt * a;
                xn.segment<3>(3) = x.segment<3>(3) + dt * a;
                xn.segment<3>(6) =
                    x.segment<3>(6) + dt * (EulerRateMatrix(x.segment<3>(6)) * x.segment<3>(9));
                xn.segment<3>(9) = x.segment<3>(9) + dt * alpha;
                x = xn;
                out.pred_p.push_back(x.segment<3>(0));
                out.pred_v.push_back(x.segment<3>(3));
                out.pred_rpy.push_back(x.segment<3>(6));
                out.pred_omega.push_back(x.segment<3>(9));
            }
        }
        out.cost = 0.5 * lambda.dot(H * lambda) + gvec.dot(lambda);

        // 步骤 13：保存本次系数、变量索引和首节点输出，供下一次初始化与失败复用。
        last_lambda_ = lambda;
        last_var_index_.clear();
        for (int v = 0; v < V; ++v) last_var_index_[Key(vars[v].k, vars[v].leg, vars[v].gen)] = v;
        have_solution_ = true;
        last_solution_time_ = t;
        out.status = SrbdMpcOutput::Status::kOk;
        last_output_ = out;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }



private:
    //------------------------------------------------------------------------------------
    // @brief 生成用于缓存查找的 [节点,腿,锥方向] 三元索引
    //------------------------------------------------------------------------------------
    static std::array<int, 3> Key(int k, int leg, int gen) { return {k, leg, gen}; }

    //===============================================================================================
    // @brief 将三元索引映射为哈希值，供上一解变量索引表查找
    //===============================================================================================
    struct ArrayHash {
        //------------------------------------------------------------------------------------
        // @brief 计算节点、腿和生成方向组合的哈希值
        //------------------------------------------------------------------------------------
        size_t operator()(const std::array<int, 3>& a) const {
            return static_cast<size_t>(a[0]) * 1000003u + static_cast<size_t>(a[1]) * 10007u +
                   static_cast<size_t>(a[2]);
        }
    };

    //------------------------------------------------------------------------------------
    // @brief 返回从给定 steady_clock 起点到当前的耗时 [ms]
    //------------------------------------------------------------------------------------
    static double ElapsedMs(const std::chrono::steady_clock::time_point& t0) {
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    //------------------------------------------------------------------------------------
    // @brief 本帧无法继续求解时，返回未过期的旧结果或失败状态
    // @details 旧解返回时保留其接触力和预测轨迹，状态改为 kStale，不更新旧解生成时间。
    //------------------------------------------------------------------------------------
    SrbdMpcOutput Finish(SrbdMpcOutput& out, const std::chrono::steady_clock::time_point& t0, double t) 
    {
        if (have_solution_ && (t - last_solution_time_) <= params_.max_solution_age) {
            SrbdMpcOutput stale = last_output_;  // pred_* 为旧解对应的预测，属已知行为
            stale.status = SrbdMpcOutput::Status::kStale;
            stale.solve_ms = ElapsedMs(t0);
            stale.nvar = out.nvar;
            return stale;
        }
        out.status = SrbdMpcOutput::Status::kFailed;
        out.solve_ms = ElapsedMs(t0);
        return out;
    }

    // 内部缓存：模型配置、质量、上一组系数与变量索引，以及最近可用结果。
    SrbdMpcParams params_;                                      // 预测模型、约束和求解配置
    double mass_ = 11.9376;                                     // 当前整机质量 [kg]；构造时由 Lite3Model 总质量覆盖
    Eigen::VectorXd last_lambda_;                               // 上一次求解得到的接触力生成系数
    std::unordered_map<std::array<int, 3>, int, ArrayHash> last_var_index_;  // 上次 [节点,腿,方向] 到系数向量下标的映射
    bool have_solution_ = false;                                // 是否已缓存过可用的 MPC 输出
    double last_solution_time_ = 0.0;                           // 旧输出对应采样时间 [s]，用于判断是否过期
    SrbdMpcOutput last_output_;                                 // 最近输出缓存，包含接触力与对应的预测轨迹
};

}  // namespace mpc_wbc



