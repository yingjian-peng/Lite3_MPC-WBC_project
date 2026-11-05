/*===============================================================================================
 * @file 00_lite3_model.hpp
 * @brief Lite3 浮动基模型：坐标约定、运动学、全身动力学及质心量计算
 * @details 本文件提供估计器、步态、MPC 和 WBC 共用的机器人模型，不读取接口或发送命令。
 * 参数来源：robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3.xml。长度使用 m，质量使用 kg，惯量使用 kg·m²。
 * 坐标约定：机体系 B 的 x 向前、y 向左、z 向上；R_WB 将机体系向量旋转到世界系。
 * 广义速度共 18 维：[世界系基座线速度(3); 机体系角速度(3); 关节速度(12)]。
 * 关节顺序：FL、FR、HL、HR；每腿为 HipX、HipY、Knee，关节角使用 rad。
 * 关节轴来自 MJCF：HipX 为 (-1,0,0)，HipY/Knee 为 (0,-1,0)，因此旋转使用负关节角。
 * 全身动力学：M*qdd+h=Sᵀ*tau+Jcᵀ*f，所有矩阵遵循上述广义速度排列。
 * 足端位置默认指 FOOT 球心；接触雅可比另按球足最低点计算，调用时注意区分。
 ===============================================================================================*/
#pragma once



/*===============================================================================================
 * 头文件引用：本模块使用的模型、数值计算和标准库接口
 ===============================================================================================*/
#include <Eigen/Dense>                              // Eigen 矩阵、向量及线性方程求解
#include <array>                                    // 固定容量的四腿状态、矩阵或接触计划容器
#include <cmath>                                    // 三角函数、绝对值和其他数学函数


namespace mpc_wbc {
//===============================================================================================
// 固定维度与索引（对齐 MJCF 实测：nq=19, nv=18, nu=12）
//===============================================================================================
constexpr int kNumLegs = 4;                                     // 腿数量，索引顺序 FL、FR、HL、HR
constexpr int kNumJoints = 12;                                  // 12 个驱动关节
constexpr int kNumGenVel = 18;                                  // [v_world(3); omega_body(3); dq(12)]
constexpr int kNumGenPos = 19;                                  // 浮动基 7 (xyz + quat wxyz) + 12 关节
constexpr int kNumContactForces = 12;                           // 四脚 × 3 维接触力
constexpr int kNumActuators = 12;                               // 独立驱动关节数量
constexpr int kNumBodies = 17;                                  // TORSO + 4 腿 × (Hip,Thigh,Shank,Foot)

enum LegIndex { kFL = 0, kFR = 1, kHL = 2, kHR = 3 };
enum JointIndex { kHipX = 0, kHipY = 1, kKnee = 2 };

using Vec3d = Eigen::Vector3d;                                  // 三维位置、速度、力等向量，坐标系由函数约定
using Vec4d = Eigen::Vector4d;                                  // 四维向量；本文件四元数使用 wxyz 顺序
using Vec12d = Eigen::Matrix<double, kNumJoints, 1>;            // 四腿各三个关节组成的 12 维向量
using Vec18d = Eigen::Matrix<double, kNumGenVel, 1>;            // 18 维浮动基广义速度、加速度或广义力
using Mat3d = Eigen::Matrix3d;                                  // 三维旋转、惯量或单腿雅可比矩阵
using Mat18d = Eigen::Matrix<double, kNumGenVel, kNumGenVel>;   // 18×18 全身质量矩阵等固定维矩阵
using Mat3x18d = Eigen::Matrix<double, 3, kNumGenVel>;          // 单个刚体或足端的三维广义速度雅可比
using Mat12x18d = Eigen::Matrix<double, kNumContactForces, kNumGenVel>;  // 四个足端堆叠的 12×18 雅可比
using Mat3x12d = Eigen::Matrix<double, 3, kNumJoints>;          // 三维量对 12 个关节变量的矩阵
using VecContact12d = Eigen::Matrix<double, kNumContactForces, 1>;  // 四脚×3
using VecNv = Vec18d;                                           // 与广义速度维数一致的向量别名



//===============================================================================================
// @brief Lite3 几何参数和关节限位
// @details 髋安装位置按 FL/FR/HL/HR 排列；关节限位按 HipX/HipY/Knee 排列。
//===============================================================================================
struct Lite3Geometry 
{
    double thigh_len = 0.20;                                    // FL_THIGH -> FL_SHANK 的 z 偏移（大腿）
    double shank_len = 0.21012;                                 // FL_SHANK -> FL_FOOT 的 z 偏移（小腿）
    double hip_lateral = 0.09735;                               // FL_HIP -> FL_THIGH 的 |y| 偏移（髋侧向偏置）
    double foot_radius = 0.022;                                 // FL_FOOT 球足半径

    // 关节限位（MJCF range，弧度），每腿相同，顺序 HipX, HipY, Knee
    Vec3d joint_lower = Vec3d(-0.523, -2.67, 0.524);
    Vec3d joint_upper = Vec3d(0.523, 0.314, 2.792);

    // 髋关节安装点在机体系下的位置，MJCF 顺序 FL, FR, HL, HR
    std::array<Vec3d, kNumLegs> hip_pos_body = {{
        Vec3d(0.1745, 0.062, 0.0),                              // FL
        Vec3d(0.1745, -0.062, 0.0),                             // FR
        Vec3d(-0.1745, 0.062, 0.0),                             // HL
        Vec3d(-0.1745, -0.062, 0.0),                            // HR
    }};

    // 左右镜像符号：+1 为左侧（y>0），-1 为右侧
    std::array<double, kNumLegs> side_sign = {{+1.0, -1.0, +1.0, -1.0}};
};

//===============================================================================================
// @brief 刚体树中单个刚体的连接、质量、质心和惯量参数
// @details 每腿依次为 Hip、Thigh、Shank、Foot；刚体索引由下列索引函数生成。
//===============================================================================================
struct BodyParams {
    int parent = -1;                                            // 父刚体索引；躯干根节点为 -1
    Vec3d offset = Vec3d::Zero();                               // 在父刚体系表达的安装位移 [m]
    int joint_col = -1;                                         // 关节在 12 维关节向量中的列号；固定连接为 -1
    Vec3d axis = Vec3d::Zero();                                 // 在当前刚体系表达的关节旋转轴
    double mass = 0.0;                                          // 刚体质量 [kg]
    Vec3d com = Vec3d::Zero();                                  // 刚体质心相对当前刚体原点的位置 [m]
    Vec3d inertia_diag = Vec3d::Zero();                         // 质心主轴惯量 [kg·m²]；模型中的惯性坐标旋转为单位阵
};



//------------------------------------------------------------------------------------
// @brief 将腿编号映射到髋刚体索引
// @param leg 腿编号：0=FL、1=FR、2=HL、3=HR。
//------------------------------------------------------------------------------------
inline constexpr int HipBodyIndex(int leg) { return 1 + 4 * leg; }



//------------------------------------------------------------------------------------
// @brief 将腿编号映射到大腿刚体索引
//------------------------------------------------------------------------------------
inline constexpr int ThighBodyIndex(int leg) { return 2 + 4 * leg; }



//------------------------------------------------------------------------------------
// @brief 将腿编号映射到小腿刚体索引
//------------------------------------------------------------------------------------
inline constexpr int ShankBodyIndex(int leg) { return 3 + 4 * leg; }



//------------------------------------------------------------------------------------
// @brief 将腿编号映射到球足刚体索引
//------------------------------------------------------------------------------------
inline constexpr int FootBodyIndex(int leg) { return 4 + 4 * leg; }



//------------------------------------------------------------------------------------
// @brief 构造三维向量的叉乘矩阵
// @return 满足 Skew(p) * v = p × v 的 3×3 矩阵。
//------------------------------------------------------------------------------------
inline Mat3d Skew(const Vec3d& p) {
    Mat3d m;
    m << 0.0, -p.z(), p.y(),
         p.z(), 0.0, -p.x(),
         -p.y(), p.x(), 0.0;
    return m;
}



//------------------------------------------------------------------------------------
// @brief 生成绕 x 轴旋转 a 的旋转矩阵
// @param a 旋转角 [rad]，遵循右手定则
//------------------------------------------------------------------------------------
inline Mat3d RotationX(double a) {
    Mat3d m;
    const double c = std::cos(a), s = std::sin(a);
    m << 1.0, 0.0, 0.0,
         0.0, c, -s,
         0.0, s, c;
    return m;
}



//------------------------------------------------------------------------------------
// @brief 生成绕 y 轴旋转 a 的旋转矩阵
// @param a 旋转角 [rad]，遵循右手定则。
//------------------------------------------------------------------------------------
inline Mat3d RotationY(double a) {
    Mat3d m;
    const double c = std::cos(a), s = std::sin(a);
    m << c, 0.0, s,
         0.0, 1.0, 0.0,
         -s, 0.0, c;
    return m;
}



constexpr double kPi = 3.14159265358979323846;                  // 圆周率，供相位、角度折叠和轨迹函数使用



//------------------------------------------------------------------------------------
// @brief 将角度折叠到 (-π, π]，便于比较逆运动学分支
// @param a 待处理角度 [rad]。
//------------------------------------------------------------------------------------
inline double WrapToPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}



//------------------------------------------------------------------------------------
// @brief 将 ZYX 欧拉角转换为机体系到世界系的旋转矩阵
// @param rpy [roll, pitch, yaw]，单位 rad；R_WB = Rz(yaw) * Ry(pitch) * Rx(roll)。
//------------------------------------------------------------------------------------
inline Mat3d RpyToRotation(const Vec3d& rpy) {
    Mat3d Rz;
    const double c = std::cos(rpy(2)), s = std::sin(rpy(2));
    Rz << c, -s, 0.0,
          s, c, 0.0,
          0.0, 0.0, 1.0;
    return Rz * RotationY(rpy(1)) * RotationX(rpy(0));
}



//------------------------------------------------------------------------------------
// @brief 使用 Rodrigues 公式生成轴角旋转矩阵
// @param axis 单位旋转轴；调用者应保证其长度为 1。
// @param angle 旋转角 [rad]；angle=0 时结果为单位阵。
//------------------------------------------------------------------------------------
inline Mat3d AxisAngle(const Vec3d& axis, double angle) {
    const Mat3d K = Skew(axis);
    return Mat3d::Identity() + std::sin(angle) * K + (1.0 - std::cos(angle)) * (K * K);
}



//------------------------------------------------------------------------------------
// @brief 使用机体系角速度对姿态做指数映射积分
// @details R_new = R * Exp(Skew(omega_body) * dt)，支持正、负 dt。
// @param omega_body 机体系角速度 [rad/s]；dt 为积分时长 [s]。
//------------------------------------------------------------------------------------
inline Mat3d IntegrateBodyRotation(const Mat3d& R, const Vec3d& omega_body, double dt) {
    const double n = omega_body.norm();
    // 注意用 fabs：写成 n*dt < 1e-12 时，负 dt（向后积分）会恒为真而直接返回 R。
    if (std::fabs(n * dt) < 1e-12) return R;
    return R * AxisAngle(omega_body / n, n * dt);
}



//------------------------------------------------------------------------------------
// @brief 将 wxyz 顺序的四元数转换为 R_WB
// @details 先归一化四元数；范数非正时返回单位阵。
//------------------------------------------------------------------------------------
inline Mat3d QuatToRotation(const Vec4d& q_wxyz) {
    const double w = q_wxyz(0), x = q_wxyz(1), y = q_wxyz(2), z = q_wxyz(3);
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n <= 0.0) return Mat3d::Identity();
    const double iw = w / n, ix = x / n, iy = y / n, iz = z / n;
    Mat3d R;
    R << 1 - 2 * (iy * iy + iz * iz), 2 * (ix * iy - iz * iw), 2 * (ix * iz + iy * iw),
         2 * (ix * iy + iz * iw), 1 - 2 * (ix * ix + iz * iz), 2 * (iy * iz - ix * iw),
         2 * (ix * iz - iy * iw), 2 * (iy * iz + ix * iw), 1 - 2 * (ix * ix + iy * iy);
    return R;
}



//===============================================================================================
// @brief 共用机器人模型：几何、刚体树和运动学/动力学计算
// @details 函数只使用传入状态与模型参数；基座位置不影响雅可比和质量矩阵。
//===============================================================================================
class Lite3Model {
public:
    Lite3Geometry geo;                                          // 几何和关节限位配置；修改时需保持刚体树参数一致
    std::array<BodyParams, kNumBodies> bodies;                  // 17 个刚体的连接与惯性参数，构造时初始化

    //------------------------------------------------------------------------------------
    // @brief 根据内置 MJCF 参数初始化 17 个刚体的连接和惯性参数
    //------------------------------------------------------------------------------------
    Lite3Model() { InitBodies(); }

    //------------------------------------------------------------------------------------
    // @brief 正运动学：单腿关节角转换为球足中心的机体系位置
    // @param leg 腿编号；q_leg 为 [HipX, HipY, Knee]，单位 rad。
    // @return 相对基座原点的球心位置 [m]，不减球足半径。
    // @details p_foot = p_hip + Rx(-qx) * [l1*sin(qy)+l2*sin(qy+qk), side*d, -l1*cos(qy)-l2*cos(qy+qk)]。
    //------------------------------------------------------------------------------------
    Vec3d FootPositionBody(int leg, const Vec3d& q_leg) const {
        const double qx = q_leg(0), qy = q_leg(1), qk = q_leg(2);
        const double l1 = geo.thigh_len, l2 = geo.shank_len;
        const double x_sag = l1 * std::sin(qy) + l2 * std::sin(qy + qk);
        const double z_sag = -l1 * std::cos(qy) - l2 * std::cos(qy + qk);
        const Vec3d sagittal(x_sag, geo.side_sign[leg] * geo.hip_lateral, z_sag);
        return geo.hip_pos_body[leg] + RotationX(-qx) * sagittal;
    }

    //------------------------------------------------------------------------------------
    // @brief 计算四条腿的球心机体系位置
    // @return 3×4 矩阵，每列依次对应 FL、FR、HL、HR，单位 m。
    //------------------------------------------------------------------------------------
    Eigen::Matrix<double, 3, kNumLegs> FootPositionsBody(const Vec12d& q) const {
        Eigen::Matrix<double, 3, kNumLegs> out;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            out.col(leg) = FootPositionBody(leg, q.segment<3>(3 * leg));
        }
        return out;
    }

    //------------------------------------------------------------------------------------
    // @brief 逆运动学：由球心机体系目标位置计算单腿关节角
    // @param p_foot_body 相对基座原点的目标球心位置 [m]。
    // @param q_leg_out 非空时接收计算结果；q_seed 可选，用于选取最接近上一目标的分支。
    // @return 侧向目标不可达时返回 false，并保持输出不变。
    // @details 对平面两连杆的距离做数值夹取；返回 true 不代表通过全部关节限位检查，上层仍需限位。
    //------------------------------------------------------------------------------------
    bool InverseKinematics(int leg, const Vec3d& p_foot_body, Vec3d* q_leg_out, const Vec3d* q_seed = nullptr) const 
    {
        const double l1 = geo.thigh_len, l2 = geo.shank_len;
        const double L = geo.hip_lateral, s = geo.side_sign[leg];
        const double eps = 1e-9;

        const Vec3d r = p_foot_body - geo.hip_pos_body[leg];
        const double A = std::sqrt(r.y() * r.y() + r.z() * r.z());
        if (A < L - eps) return false;                              // 侧向偏置超出该腿可达范围

        // 解 HipX：cos(psi + qx) = s*L / A。候选角必须折叠到 (-pi, pi]，否则会漏掉 -phi-psi+2pi 分支（真解可能落在那里）
        const double psi = std::atan2(r.z(), r.y());
        const double phi = std::acos(std::max(-1.0, std::min(1.0, s * L / A)));
        const double qx_a = WrapToPi(phi - psi);
        const double qx_b = WrapToPi(-phi - psi);

        // 给定一侧摆角，解出完整单腿角（消去侧摆后是平面两连杆问题）
        auto solve_from_qx = [&](double qx) -> Vec3d 
        {
            const double z_sag = std::sin(qx) * r.y() + std::cos(qx) * r.z();
            const double x_sag = r.x();
            double rho = std::sqrt(x_sag * x_sag + z_sag * z_sag);
            const double rho_min = std::fabs(l1 - l2) + eps;
            const double rho_max = l1 + l2 - eps;
            if (rho < rho_min) rho = rho_min;
            if (rho > rho_max) rho = rho_max;

            double ck = (rho * rho - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
            ck = std::max(-1.0, std::min(1.0, ck));
            const double qk = std::acos(ck);

            const double z_down = -z_sag;
            const double qy = std::atan2(x_sag, z_down) -
                              std::atan2(l2 * std::sin(qk), l1 + l2 * std::cos(qk));
            return Vec3d(qx, qy, qk);
        };

        const Vec3d sol_a = solve_from_qx(qx_a);
        const Vec3d sol_b = solve_from_qx(qx_b);
        const Vec3d chosen = q_seed
            ? ((sol_a - *q_seed).norm() <= (sol_b - *q_seed).norm() ? sol_a : sol_b)
            : (std::fabs(qx_a) <= std::fabs(qx_b) ? sol_a : sol_b);

        if (q_leg_out) *q_leg_out = chosen;

        return true;
    }

    //------------------------------------------------------------------------------------
    // @brief 用中心差分计算单腿球心位置对三个关节角的雅可比
    // @return 3×3 机体系雅可比 J_leg，满足 v_foot_body = J_leg * dq_leg。
    // @details 差分步长为 1e-6 rad，沿用与正运动学一致的关节符号。
    //------------------------------------------------------------------------------------
    Mat3d LegJacobianBody(int leg, const Vec3d& q_leg) const 
    {
        Mat3d J;
        const double eps = 1e-6;
        for (int j = 0; j < 3; ++j) {
            Vec3d qp = q_leg, qm = q_leg;
            qp(j) += eps;
            qm(j) -= eps;
            J.col(j) = (FootPositionBody(leg, qp) - FootPositionBody(leg, qm)) / (2.0 * eps);
        }

        return J;
    }

    //------------------------------------------------------------------------------------
    // @brief 计算四足球心的世界系速度雅可比
    // @return 12×18 矩阵，行按四腿 xyz 排列，列按 [v_world; omega_body; dq] 排列。
    // @details 单腿速度：v_f = v_base - R_WB * Skew(p_f_body) * omega_body + R_WB * J_leg * dq_leg。
    //------------------------------------------------------------------------------------
    Mat12x18d FootJacobianWorld(const Vec12d& q, const Mat3d& R_WB) const 
    {
        Mat12x18d J = Mat12x18d::Zero();
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const Vec3d q_leg = q.segment<3>(3 * leg);
            const Vec3d p_b = FootPositionBody(leg, q_leg);
            const Mat3d J_leg = LegJacobianBody(leg, q_leg);
            const int row = 3 * leg;
            J.block<3, 3>(row, 0) = Mat3d::Identity();
            J.block<3, 3>(row, 3) = -R_WB * Skew(p_b);
            J.block<3, 3>(row, 6 + 3 * leg) = R_WB * J_leg;
        }

        return J;
    }

    //------------------------------------------------------------------------------------
    // @brief 通过各刚体质心动能叠加生成全身质量矩阵
    // @param R_B 机体系到世界系的旋转矩阵。
    // @return 对称 18×18 质量矩阵，与本文件的混合坐标广义速度排列一致。
    // @details 每个刚体贡献 m*Jvᵀ*Jv + Jwᵀ*I_world*Jw，末尾做数值对称化。
    //------------------------------------------------------------------------------------
    Mat18d MassMatrix(const Vec12d& q, const Mat3d& R_B) const 
    {
        // 步骤 1：计算刚体雅可比，再按质心线动能与转动动能累加质量矩阵
        std::array<Mat3d, kNumBodies> R;
        std::array<Mat3x18d, kNumBodies> Jp, Jr;
        ComputeBodyJacobians(q, R_B, &R, &Jp, &Jr);

        Mat18d M = Mat18d::Zero();
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            const Vec3d c_w = R[i] * b.com;
            const Mat3x18d Jp_com = Jp[i] - Skew(c_w) * Jr[i];
            const Mat3d I_world = R[i] * b.inertia_diag.asDiagonal() * R[i].transpose();
            M += b.mass * Jp_com.transpose() * Jp_com + Jr[i].transpose() * I_world * Jr[i];
        }
        return 0.5 * (M + M.transpose());
    }

    //------------------------------------------------------------------------------------
    // @brief 计算全身动力学偏置力 h，包含重力、科氏力和离心力
    // @param gravity 世界系重力加速度 [m/s²]，通常为 (0,0,-9.81)。
    // @return 18 维广义力；基座部分为力/力矩，关节部分为力矩。
    // @details 在 qdd=0 下递推刚体加速度，再用质心雅可比把惯性力和力矩投影到广义坐标。
    //------------------------------------------------------------------------------------
    Vec18d BiasForce(const Vec12d& q, const Vec18d& qvel, const Mat3d& R_B, const Vec3d& gravity) const 
    {
        std::array<Mat3d, kNumBodies> R;
        std::array<Mat3x18d, kNumBodies> Jp, Jr;
        ComputeBodyJacobians(q, R_B, &R, &Jp, &Jr);

        // 前向递推：速度、以及 qdd=0 时的加速度
        std::array<Vec3d, kNumBodies> v, omega, a, alpha;
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            if (b.parent < 0) {
                v[i] = qvel.segment<3>(0);
                omega[i] = R_B * qvel.segment<3>(3);
                a[i] = -gravity;  // 重力作为基座虚拟线加速度
                alpha[i] = Vec3d::Zero();
            } 
            else {
                const int par = b.parent;
                const Vec3d r = R[par] * b.offset;
                omega[i] = omega[par];
                alpha[i] = alpha[par];
                if (b.joint_col >= 0) {
                    const Vec3d axis_w = R[i] * b.axis;
                    const double qd = qvel(6 + b.joint_col);
                    omega[i] += axis_w * qd;
                    alpha[i] += omega[par].cross(axis_w) * qd;
                }
                v[i] = v[par] + omega[par].cross(r);
                a[i] = a[par] + alpha[par].cross(r) + omega[par].cross(omega[par].cross(r));
            }
        }

        // 将各刚体惯性力/力矩通过质心雅可比投影并累加，得到 C*qvel+G
        Vec18d h = Vec18d::Zero();
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            const Vec3d c_w = R[i] * b.com;
            const Mat3d I_world = R[i] * b.inertia_diag.asDiagonal() * R[i].transpose();
            const Vec3d a_com =
                a[i] + alpha[i].cross(c_w) + omega[i].cross(omega[i].cross(c_w));
            const Vec3d N = I_world * alpha[i] + omega[i].cross(I_world * omega[i]);
            const Mat3x18d Jp_com = Jp[i] - Skew(c_w) * Jr[i];
            h += Jp_com.transpose() * (b.mass * a_com) + Jr[i].transpose() * N;
        }

        return h;
    }

    //------------------------------------------------------------------------------------
    // @brief 计算四个球足最低接触点的世界系雅可比
    // @details 接触点偏移为世界系 (0,0,-foot_radius)，区别于球心雅可比 FootJacobianWorld。
    // @return 12×18 矩阵 Jc，供 WBC 的动力学与接触约束使用。
    //------------------------------------------------------------------------------------
    Mat12x18d ContactJacobianWorld(const Vec12d& q, const Mat3d& R_B) const 
    {
        std::array<Mat3d, kNumBodies> R;
        std::array<Mat3x18d, kNumBodies> Jp, Jr;
        ComputeBodyJacobians(q, R_B, &R, &Jp, &Jr);

        Mat12x18d Jc;
        const Vec3d d(0.0, 0.0, -geo.foot_radius);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int f = FootBodyIndex(leg);
            Jc.block<3, kNumGenVel>(3 * leg, 0) = Jp[f] - Skew(d) * Jr[f];
        }
        return Jc;
    }

    //------------------------------------------------------------------------------------
    // @brief 累加所有刚体质量，返回整机质量 [kg]
    //------------------------------------------------------------------------------------
    double TotalMass() const 
    {
        double m = 0.0;
        for (const auto& b : bodies) m += b.mass;
        return m;
    }

    //------------------------------------------------------------------------------------
    // @brief 通过中心差分计算接触点偏置加速度 Jdot_c * qvel
    // @details 沿关节速度和基座机体系角速度对 q、R_B 前后推进 eps 秒，再对接触雅可比差分。
    // @return 四腿 xyz 排列的 12 维世界系偏置加速度 [m/s²]。
    // @details 接触加速度为 Jc*qdd + Jdot_c*qvel；WBC 组装约束时应同时使用两项。
    //------------------------------------------------------------------------------------
    VecContact12d ContactBiasAccel(const Vec12d& q, const Mat3d& R_B, const Vec18d& qvel, double eps = 1e-6) const 
    {
        const Vec3d omega_body = qvel.segment<3>(3);
        const Vec12d dq = qvel.segment<kNumJoints>(6);
        const Mat3d R_p = IntegrateBodyRotation(R_B, omega_body, +eps);
        const Mat3d R_m = IntegrateBodyRotation(R_B, omega_body, -eps);
        const Mat12x18d J_p = ContactJacobianWorld(q + eps * dq, R_p);
        const Mat12x18d J_m = ContactJacobianWorld(q - eps * dq, R_m);
        return ((J_p - J_m) / (2.0 * eps)) * qvel;
    }

    //------------------------------------------------------------------------------------
    // @brief 按质量加权计算整机质心相对基座的机体系位置 [m]
    //------------------------------------------------------------------------------------
    Vec3d CompositeComBody(const Vec12d& q) const 
    {
        std::array<Mat3d, kNumBodies> R;
        std::array<Vec3d, kNumBodies> p;
        ComputeBodyPoses(q, &R, &p);
        Vec3d num = Vec3d::Zero();
        double m = 0.0;
        for (int i = 0; i < kNumBodies; ++i) {
            num += bodies[i].mass * (p[i] + R[i] * bodies[i].com);
            m += bodies[i].mass;
        }
        return num / m;
    }

    //------------------------------------------------------------------------------------
    // @brief 把整机质心位置转换为世界系位置 [m]
    // @param p_base_w 基座原点的世界系位置；结果为 p_base_w + R_B * com_body。
    //------------------------------------------------------------------------------------
    Vec3d CompositeComWorld(const Vec12d& q, const Mat3d& R_B, const Vec3d& p_base_w) const 
    {
        return p_base_w + R_B * CompositeComBody(q);
    }

    //------------------------------------------------------------------------------------
    // @brief 使用平行轴定理计算整机绕给定点的机体系惯量
    // @param point_body 参考点在机体系中的位置 [m]；传整机质心可得到 SRBD 惯量。
    // @return 3×3 惯量矩阵 [kg·m²]。
    //------------------------------------------------------------------------------------
    Mat3d CompositeInertiaBody(const Vec12d& q, const Vec3d& point_body) const 
    {
        std::array<Mat3d, kNumBodies> R;
        std::array<Vec3d, kNumBodies> p;
        ComputeBodyPoses(q, &R, &p);
        Mat3d I = Mat3d::Zero();
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            const Vec3d d = (p[i] + R[i] * b.com) - point_body;
            I += R[i] * b.inertia_diag.asDiagonal() * R[i].transpose() +
                 b.mass * (d.squaredNorm() * Mat3d::Identity() - d * d.transpose());
        }
        return I;
    }

    //------------------------------------------------------------------------------------
    // @brief 将整机绕给定点的惯量旋转到世界系表达
    // @details I_world = R_B * I_body * R_Bᵀ，参考点仍由 point_body 给出。
    //------------------------------------------------------------------------------------
    Mat3d CompositeInertiaWorld(const Vec12d& q, const Mat3d& R_B, const Vec3d& point_body) const 
    {
        return R_B * CompositeInertiaBody(q, point_body) * R_B.transpose();
    }

    //------------------------------------------------------------------------------------
    // @brief 返回单腿默认站立角 [0, -0.65, 1.3] rad
    //------------------------------------------------------------------------------------
    static Vec3d DefaultStandLegAngles() { 
        return Vec3d(0.0, -0.65, 1.3); 
    }

    //------------------------------------------------------------------------------------
    // @brief 把单腿默认站姿复制为 FL/FR/HL/HR 共 12 个关节目标
    //------------------------------------------------------------------------------------
    static Vec12d DefaultStandJointAngles() 
    {
        Vec12d q;
        for (int leg = 0; leg < kNumLegs; ++leg) q.segment<3>(3 * leg) = DefaultStandLegAngles();
        return q;
    }


private:
    //------------------------------------------------------------------------------------
    // @brief 建立躯干和四腿刚体树，并填写 MJCF 质量、质心和惯量参数
    // @details 根节点为躯干；每腿依次连接 Hip→Thigh→Shank→Foot，Foot 为固定连接。
    //------------------------------------------------------------------------------------
    void InitBodies() 
    {
        // 关节列：leg i -> 3i HipX、3i+1 HipY、3i+2 Knee
        const Vec3d hip_axis(-1.0, 0.0, 0.0);
        const Vec3d pitch_axis(0.0, -1.0, 0.0);

        BodyParams torso;
        torso.parent = -1;
        torso.mass = 5.6056;
        torso.com = Vec3d::Zero();
        torso.inertia_diag = Vec3d(0.02456, 0.05518, 0.07016);
        bodies[0] = torso;

        // 各腿 <inertial>（Lite3.xml）。顺序 FL, FR, HL, HR。
        const Vec3d hip_com[kNumLegs] = {
            Vec3d(-0.00601, -0.0066532, 0.00034295),
            Vec3d(-0.010579, 0.011358, 0.00048546),
            Vec3d(0.010905, -0.012636, 0.001051),
            Vec3d(0.010354, 0.011423, 0.00049498)
        };
        const Vec3d hip_inertia[kNumLegs] = {
            Vec3d(0.0003949, 0.0004028, 0.0004472),
            Vec3d(0.0004472, 0.0004028, 0.0003949),
            Vec3d(0.0003949, 0.0004028, 0.0004472),
            Vec3d(0.0003949, 0.0004028, 0.0004472)
        };
        const double thigh_com_y[kNumLegs] = {-0.014632, 0.014632, -0.014632, 0.014632};
        const double shank_com_y[kNumLegs] = {-1.4535e-06, -1.4552e-06, -1.4558e-06, -1.4529e-06};

        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double s = geo.side_sign[leg];
            BodyParams b;

            b.parent = 0;
            b.offset = geo.hip_pos_body[leg];
            b.joint_col = 3 * leg;
            b.axis = hip_axis;
            b.mass = 0.550;
            b.com = hip_com[leg];
            b.inertia_diag = hip_inertia[leg];
            bodies[HipBodyIndex(leg)] = b;

            b.parent = HipBodyIndex(leg);
            b.offset = Vec3d(0.0, s * geo.hip_lateral, 0.0);
            b.joint_col = 3 * leg + 1;
            b.axis = pitch_axis;
            b.mass = 0.86;
            b.com = Vec3d(-0.0039245, thigh_com_y[leg], -0.025146);
            b.inertia_diag = Vec3d(0.005736, 0.00496, 0.001436);
            bodies[ThighBodyIndex(leg)] = b;

            b.parent = ThighBodyIndex(leg);
            b.offset = Vec3d(0.0, 0.0, -geo.thigh_len);
            b.joint_col = 3 * leg + 2;
            b.axis = pitch_axis;
            b.mass = 0.153;
            b.com = Vec3d(0.0064794, shank_com_y[leg], -0.12157);
            b.inertia_diag = Vec3d(0.00089039, 0.00090672, 3.1266e-05);
            bodies[ShankBodyIndex(leg)] = b;

            b.parent = ShankBodyIndex(leg);
            b.offset = Vec3d(0.0, 0.0, -geo.shank_len);
            b.joint_col = -1;
            b.axis = Vec3d::Zero();
            b.mass = 0.02;
            b.com = Vec3d::Zero();
            b.inertia_diag = Vec3d::Zero();
            bodies[FootBodyIndex(leg)] = b;
        }
    }

    //------------------------------------------------------------------------------------
    // @brief 沿刚体树递推各刚体相对基座的位姿
    // @param R 输出各刚体系到机体系的旋转；p 输出各刚体原点的机体系位置 [m]。
    //------------------------------------------------------------------------------------
    void ComputeBodyPoses(const Vec12d& q, std::array<Mat3d, kNumBodies>* R, std::array<Vec3d, kNumBodies>* p) const 
    {
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            if (b.parent < 0) {
                (*R)[i] = Mat3d::Identity();
                (*p)[i] = Vec3d::Zero();
            } 
            else {
                const int par = b.parent;
                const double angle = (b.joint_col >= 0) ? q(b.joint_col) : 0.0;
                (*R)[i] = (*R)[par] * AxisAngle(b.axis, angle);
                (*p)[i] = (*p)[par] + (*R)[par] * b.offset;
            }
        }
    }

    //------------------------------------------------------------------------------------
    // @brief 递推各刚体姿态及原点的线速度/角速度雅可比
    // @param R 输出各刚体系到世界系的旋转；Jp、Jr 分别为 3×18 的线速度和角速度雅可比。
    // @details 子节点继承父节点雅可比，加入安装偏移的叉乘项和自身关节轴贡献。
    //------------------------------------------------------------------------------------
    void ComputeBodyJacobians(const Vec12d& q, const Mat3d& R_B,
                              std::array<Mat3d, kNumBodies>* R,
                              std::array<Mat3x18d, kNumBodies>* Jp,
                              std::array<Mat3x18d, kNumBodies>* Jr) const 
    {
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            if (b.parent < 0) {
                (*R)[i] = R_B;
                Mat3x18d jp = Mat3x18d::Zero();
                jp.block<3, 3>(0, 0) = Mat3d::Identity();
                Mat3x18d jr = Mat3x18d::Zero();
                jr.block<3, 3>(0, 3) = R_B;
                (*Jp)[i] = jp;
                (*Jr)[i] = jr;
            } 
            else {
                const int par = b.parent;
                const double angle = (b.joint_col >= 0) ? q(b.joint_col) : 0.0;
                (*R)[i] = (*R)[par] * AxisAngle(b.axis, angle);
                const Vec3d r = (*R)[par] * b.offset;
                (*Jp)[i] = (*Jp)[par] - Skew(r) * (*Jr)[par];
                (*Jr)[i] = (*Jr)[par];
                if (b.joint_col >= 0) (*Jr)[i].col(6 + b.joint_col) += (*R)[i] * b.axis;
            }
        }
    }
};

}  // namespace mpc_wbc



