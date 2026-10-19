/**
 * @file 00_lite3_model.hpp
 * @brief Lite3 平地基座模型：运动学（FK/IK/雅可比）。动力学（M/h/Jc）在后续步骤补入。
 *
 * 数值来源：robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3.xml（以 MuJoCo 对表为准）。
 * 坐标约定（见 docs/09_MPC_WBC重新推导与基础契约.md 第 1 节）：
 *   机体系 B：x 向前、y 向左、z 向上（右手系）。
 *   广义速度 qvel = [v_world(3); omega_body(3); dq(12)]，与 MuJoCo free joint 一致。
 * 关节顺序：FL, FR, HL, HR，每腿 HipX, HipY, Knee（与 MJCF / SDK 一致）。
 *
 * 符号约定（关键）：MuJoCo 中 HipX 轴为 (-1,0,0)，HipY/Knee 轴为 (0,-1,0)。
 *   因此本文件中的 qx/qy/qk 直接就是 MuJoCo 关节角，旋转分别为 Rx(-qx)、Ry(-qy)、Ry(-qk)。
 *   关节旋转符号以 MJCF 为准。
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace mpc_wbc {

// ---------------------------------------------------------------------------
// 固定维度与索引（对齐 MJCF 实测：nq=19, nv=18, nu=12）
// ---------------------------------------------------------------------------
constexpr int kNumLegs = 4;
constexpr int kNumJoints = 12;               // 12 个驱动关节
constexpr int kNumGenVel = 18;               // [v_world(3); omega_body(3); dq(12)]
constexpr int kNumGenPos = 19;               // 浮动基 7 (xyz + quat wxyz) + 12 关节
constexpr int kNumContactForces = 12;        // 四脚 × 3 维接触力
constexpr int kNumActuators = 12;
constexpr int kNumBodies = 17;               // TORSO + 4 腿 × (Hip,Thigh,Shank,Foot)

enum LegIndex { kFL = 0, kFR = 1, kHL = 2, kHR = 3 };
enum JointIndex { kHipX = 0, kHipY = 1, kKnee = 2 };

using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec12d = Eigen::Matrix<double, kNumJoints, 1>;
using Vec18d = Eigen::Matrix<double, kNumGenVel, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat18d = Eigen::Matrix<double, kNumGenVel, kNumGenVel>;
using Mat3x18d = Eigen::Matrix<double, 3, kNumGenVel>;
using Mat12x18d = Eigen::Matrix<double, kNumContactForces, kNumGenVel>;
using Mat3x12d = Eigen::Matrix<double, 3, kNumJoints>;
using VecContact12d = Eigen::Matrix<double, kNumContactForces, 1>;  // 四脚×3
using VecNv = Vec18d;

// ---------------------------------------------------------------------------
// 几何参数（MJCF 实测值）
// ---------------------------------------------------------------------------
struct Lite3Geometry {
    double thigh_len = 0.20;          // FL_THIGH -> FL_SHANK 的 z 偏移（大腿）
    double shank_len = 0.21012;       // FL_SHANK -> FL_FOOT 的 z 偏移（小腿）
    double hip_lateral = 0.09735;     // FL_HIP -> FL_THIGH 的 |y| 偏移（髋侧向偏置）
    double foot_radius = 0.022;       // FL_FOOT 球足半径

    // 关节限位（MJCF range，弧度），每腿相同，顺序 HipX, HipY, Knee
    Vec3d joint_lower = Vec3d(-0.523, -2.67, 0.524);
    Vec3d joint_upper = Vec3d(0.523, 0.314, 2.792);

    // 髋关节安装点在机体系下的位置，MJCF 顺序 FL, FR, HL, HR
    std::array<Vec3d, kNumLegs> hip_pos_body = {{
        Vec3d(0.1745, 0.062, 0.0),    // FL
        Vec3d(0.1745, -0.062, 0.0),   // FR
        Vec3d(-0.1745, 0.062, 0.0),   // HL
        Vec3d(-0.1745, -0.062, 0.0),  // HR
    }};
    // 左右镜像符号：+1 为左侧（y>0），-1 为右侧
    std::array<double, kNumLegs> side_sign = {{+1.0, -1.0, +1.0, -1.0}};
};

// ---------------------------------------------------------------------------
// 刚体参数（S2 全身动力学）。数值来源 Lite3.xml 的 <inertial>。
// 刚体顺序：0=TORSO；leg i(FL,FR,HL,HR) 依次为 1+4i=Hip, 2+4i=Thigh, 3+4i=Shank, 4+4i=Foot。
// ---------------------------------------------------------------------------
struct BodyParams {
    int parent = -1;                    // 父刚体索引；基座为 -1
    Vec3d offset = Vec3d::Zero();       // 相对父刚体，在父系下表达
    int joint_col = -1;                 // dq 列索引 (0..11)；固定刚体为 -1
    Vec3d axis = Vec3d::Zero();         // 关节轴，在本刚体系下
    double mass = 0.0;
    Vec3d com = Vec3d::Zero();          // 质心，在本刚体系下
    Vec3d inertia_diag = Vec3d::Zero(); // 主轴惯量（本刚体系，inertial quat 均为单位）
};

inline constexpr int HipBodyIndex(int leg) { return 1 + 4 * leg; }
inline constexpr int ThighBodyIndex(int leg) { return 2 + 4 * leg; }
inline constexpr int ShankBodyIndex(int leg) { return 3 + 4 * leg; }
inline constexpr int FootBodyIndex(int leg) { return 4 + 4 * leg; }

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------
inline Mat3d Skew(const Vec3d& p) {
    Mat3d m;
    m << 0.0, -p.z(), p.y(),
         p.z(), 0.0, -p.x(),
         -p.y(), p.x(), 0.0;
    return m;
}

inline Mat3d RotationX(double a) {
    Mat3d m;
    const double c = std::cos(a), s = std::sin(a);
    m << 1.0, 0.0, 0.0,
         0.0, c, -s,
         0.0, s, c;
    return m;
}

inline Mat3d RotationY(double a) {
    Mat3d m;
    const double c = std::cos(a), s = std::sin(a);
    m << c, 0.0, s,
         0.0, 1.0, 0.0,
         -s, 0.0, c;
    return m;
}

constexpr double kPi = 3.14159265358979323846;

/// 把角度折叠到 (-pi, pi]。
inline double WrapToPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

/// 欧拉角 (roll, pitch, yaw)（ZYX 顺序，与 MuJoCo/仿真一致）-> R_WB。
inline Mat3d RpyToRotation(const Vec3d& rpy) {
    Mat3d Rz;
    const double c = std::cos(rpy(2)), s = std::sin(rpy(2));
    Rz << c, -s, 0.0,
          s, c, 0.0,
          0.0, 0.0, 1.0;
    return Rz * RotationY(rpy(1)) * RotationX(rpy(0));
}

/// 绕单位轴 axis 旋转 angle 的旋转矩阵（Rodrigues 公式）。
inline Mat3d AxisAngle(const Vec3d& axis, double angle) {
    const Mat3d K = Skew(axis);
    return Mat3d::Identity() + std::sin(angle) * K + (1.0 - std::cos(angle)) * (K * K);
}

/// 在机体系角速度 omega_body 下把旋转向前积分 dt：R <- R * Exp([omega_body] dt)。
inline Mat3d IntegrateBodyRotation(const Mat3d& R, const Vec3d& omega_body, double dt) {
    const double n = omega_body.norm();
    // 注意用 fabs：写成 n*dt < 1e-12 时，负 dt（向后积分）会恒为真而直接返回 R。
    if (std::fabs(n * dt) < 1e-12) return R;
    return R * AxisAngle(omega_body / n, n * dt);
}

/// 四元数 (w,x,y,z) -> 旋转矩阵 R_WB（机体系向量转到世界系）。
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

// ---------------------------------------------------------------------------
// Lite3 平地基座模型
// ---------------------------------------------------------------------------
class Lite3Model {
public:
    Lite3Geometry geo;
    std::array<BodyParams, kNumBodies> bodies;

    Lite3Model() { InitBodies(); }

    // -----------------------------------------------------------------------
    // 正运动学 FK：单腿关节角 -> 足端在机体系下的位置
    //   p_foot^B = p_hip + Rx(-qx) * [ x_sag; s*d_h; z_sag ]
    //   x_sag = l1*sin(qy) + l2*sin(qy+qk)
    //   z_sag = -l1*cos(qy) - l2*cos(qy+qk)
    // 注意：返回的是 FOOT body 原点（球心）位置；触地点在球心下方 foot_radius 处。
    // -----------------------------------------------------------------------
    Vec3d FootPositionBody(int leg, const Vec3d& q_leg) const {
        const double qx = q_leg(0), qy = q_leg(1), qk = q_leg(2);
        const double l1 = geo.thigh_len, l2 = geo.shank_len;
        const double x_sag = l1 * std::sin(qy) + l2 * std::sin(qy + qk);
        const double z_sag = -l1 * std::cos(qy) - l2 * std::cos(qy + qk);
        const Vec3d sagittal(x_sag, geo.side_sign[leg] * geo.hip_lateral, z_sag);
        return geo.hip_pos_body[leg] + RotationX(-qx) * sagittal;
    }

    /// 四足足端（球心）在机体系下的位置，按 3x4 排布（每列一条腿）。
    Eigen::Matrix<double, 3, kNumLegs> FootPositionsBody(const Vec12d& q) const {
        Eigen::Matrix<double, 3, kNumLegs> out;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            out.col(leg) = FootPositionBody(leg, q.segment<3>(3 * leg));
        }
        return out;
    }

    // -----------------------------------------------------------------------
    // 逆运动学 IK：足端目标（机体系，FOOT 球心）-> 单腿关节角。
    // HipX 侧摆存在两个分支（对应腿平面的两种构型），都精确到达同一目标：
    //   - 传入 q_seed 时，取与 seed 最近的解（控制中保证连续性，推荐）；
    //   - 不传 seed 时，取 |qx| 较小的解。
    // 返回 false 表示目标不可达，此时不改动 q_leg_out。
    // -----------------------------------------------------------------------
    bool InverseKinematics(int leg, const Vec3d& p_foot_body, Vec3d* q_leg_out,
                           const Vec3d* q_seed = nullptr) const {
        const double l1 = geo.thigh_len, l2 = geo.shank_len;
        const double L = geo.hip_lateral, s = geo.side_sign[leg];
        const double eps = 1e-9;

        const Vec3d r = p_foot_body - geo.hip_pos_body[leg];
        const double A = std::sqrt(r.y() * r.y() + r.z() * r.z());
        if (A < L - eps) return false;  // 侧向偏置超出该腿可达范围

        // 解 HipX：cos(psi + qx) = s*L / A。候选角必须折叠到 (-pi, pi]，
        // 否则会漏掉 -phi-psi+2pi 分支（真解可能落在那里）。
        const double psi = std::atan2(r.z(), r.y());
        const double phi = std::acos(std::max(-1.0, std::min(1.0, s * L / A)));
        const double qx_a = WrapToPi(phi - psi);
        const double qx_b = WrapToPi(-phi - psi);

        // 给定一侧摆角，解出完整单腿角（消去侧摆后是平面两连杆问题）。
        auto solve_from_qx = [&](double qx) -> Vec3d {
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

    // -----------------------------------------------------------------------
    // 单腿雅可比（机体系，3x3）：中心差分 d p_foot^B / d q_leg
    // -----------------------------------------------------------------------
    Mat3d LegJacobianBody(int leg, const Vec3d& q_leg) const {
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

    // -----------------------------------------------------------------------
    // 全身足端雅可比（世界系，12x18）：把 qvel 映射到四足球心的世界线速度。
    //   v_fi^W = v_B^W - R_WB * [p_fi^B]x * omega_B + R_WB * J_leg^B * dq_leg
    // 行按 FL,FR,HL,HR 各 3 维；列按 [v_world(3); omega_body(3); dq(12)]。
    // -----------------------------------------------------------------------
    Mat12x18d FootJacobianWorld(const Vec12d& q, const Mat3d& R_WB) const {
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

    // -----------------------------------------------------------------------
    // 全身动力学（S2）
    //
    // 运动方程： M(q)·qdd + h(q, qvel) = Sᵀ·τ + Jcᵀ·f
    //   qvel = [v_world(3); omega_body(3); dq(12)]（与 MuJoCo free joint 一致）
    //   M(q) 由逐刚体动能组装；h 由 Newton-Euler（qdd=0、重力作为基座虚拟加速度）。
    // -----------------------------------------------------------------------

    /// 全身质量矩阵 M(q)（18×18）。仅依赖关节角与基座姿态。
    Mat18d MassMatrix(const Vec12d& q, const Mat3d& R_B) const {
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

    /// bias 项 h(q, qvel)（18），含科氏/离心/重力。
    Vec18d BiasForce(const Vec12d& q, const Vec18d& qvel, const Mat3d& R_B,
                     const Vec3d& gravity) const {
        std::array<Mat3d, kNumBodies> R;
        std::array<Mat3x18d, kNumBodies> Jp, Jr;
        ComputeBodyJacobians(q, R_B, &R, &Jp, &Jr);

        // 前向递推：速度、以及 qdd=0 时的加速度。
        std::array<Vec3d, kNumBodies> v, omega, a, alpha;
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            if (b.parent < 0) {
                v[i] = qvel.segment<3>(0);
                omega[i] = R_B * qvel.segment<3>(3);
                a[i] = -gravity;  // 重力作为基座虚拟线加速度
                alpha[i] = Vec3d::Zero();
            } else {
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

        // 反向求和：各刚体惯性广义力之和 = C·qvel + G。
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

    /// 四脚接触点雅可比（世界系 12×18）。接触点取球足最低点，即球心 + (0,0,-foot_radius)。
    Mat12x18d ContactJacobianWorld(const Vec12d& q, const Mat3d& R_B) const {
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

    /// 所有刚体质量之和。
    double TotalMass() const {
        double m = 0.0;
        for (const auto& b : bodies) m += b.mass;
        return m;
    }

    // -----------------------------------------------------------------------
    // 接触点偏置加速度 Jdot_c·qvel（S7 WBC 的支撑无滑约束需要）。
    //   a_contact = Jc(q)·qdd + Jdot_c(q,qvel)·qvel
    // 加速度恒等式 a=J*qdd+Jdot*v 要求保留 Jdot·v，否则支撑脚在加速时会被判成“应该是零加速度”。
    // Jdot_c·qvel 用中心差分算：把广义位置沿 qvel 用指数映射前后各推 ε，取
    //   Jdot·qvel ≈ (Jc(q⊕ε·qvel) − Jc(q⊖ε·qvel))·qvel / (2ε)
    // 注意：Jc 只依赖关节角与基座姿态（与基座位置无关），故只推 R_B 与 q。
    // -----------------------------------------------------------------------
    VecContact12d ContactBiasAccel(const Vec12d& q, const Mat3d& R_B,
                                   const Vec18d& qvel, double eps = 1e-6) const {
        const Vec3d omega_body = qvel.segment<3>(3);
        const Vec12d dq = qvel.segment<kNumJoints>(6);
        const Mat3d R_p = IntegrateBodyRotation(R_B, omega_body, +eps);
        const Mat3d R_m = IntegrateBodyRotation(R_B, omega_body, -eps);
        const Mat12x18d J_p = ContactJacobianWorld(q + eps * dq, R_p);
        const Mat12x18d J_m = ContactJacobianWorld(q - eps * dq, R_m);
        return ((J_p - J_m) / (2.0 * eps)) * qvel;
    }

    // -----------------------------------------------------------------------
    // 整机质心与合成惯量（S6 SRBD-MPC 用）。点坐标均在机体系下、以浮动基原点为参考。
    // -----------------------------------------------------------------------
    /// 整机质心在机体系下的位置（相对浮动基原点）。
    Vec3d CompositeComBody(const Vec12d& q) const {
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

    /// 整机质心在世界系下的位置（p_base_w 为浮动基原点在世界系下的位置）。
    Vec3d CompositeComWorld(const Vec12d& q, const Mat3d& R_B, const Vec3d& p_base_w) const {
        return p_base_w + R_B * CompositeComBody(q);
    }

    // Exact instantaneous CoM velocity, including joint motion. Columns follow
    // [v_base_W, omega_B, dq], matching the whole-body dynamics convention.
    Mat3x18d CompositeComJacobianWorld(const Vec12d& q, const Mat3d& R_B) const {
        std::array<Mat3d, kNumBodies> R;
        std::array<Mat3x18d, kNumBodies> Jp, Jr;
        ComputeBodyJacobians(q, R_B, &R, &Jp, &Jr);
        Mat3x18d J = Mat3x18d::Zero();
        for (int i = 0; i < kNumBodies; ++i)
            J += bodies[i].mass * (Jp[i] - Skew(R[i] * bodies[i].com) * Jr[i]);
        return J / TotalMass();
    }

    /// 整机绕机体系点 point_body 的合成惯量（表达在机体系）。平行轴定理逐刚体累加。
    Mat3d CompositeInertiaBody(const Vec12d& q, const Vec3d& point_body) const {
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

    /// 整机绕机体系点 point_body 的合成惯量，旋转到世界系表达。
    Mat3d CompositeInertiaWorld(const Vec12d& q, const Mat3d& R_B, const Vec3d& point_body) const {
        return R_B * CompositeInertiaBody(q, point_body) * R_B.transpose();
    }

    // -----------------------------------------------------------------------
    // 默认站立关节角（每腿 [0, -0.65, 1.3]）
    // -----------------------------------------------------------------------
    static Vec3d DefaultStandLegAngles() { return Vec3d(0.0, -0.65, 1.3); }
    static Vec12d DefaultStandJointAngles() {
        Vec12d q;
        for (int leg = 0; leg < kNumLegs; ++leg) q.segment<3>(3 * leg) = DefaultStandLegAngles();
        return q;
    }

private:
    void InitBodies() {
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
            Vec3d(0.010354, 0.011423, 0.00049498)};
        const Vec3d hip_inertia[kNumLegs] = {
            Vec3d(0.0003949, 0.0004028, 0.0004472),
            Vec3d(0.0004472, 0.0004028, 0.0003949),
            Vec3d(0.0003949, 0.0004028, 0.0004472),
            Vec3d(0.0003949, 0.0004028, 0.0004472)};
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

    /// 计算各刚体相对浮动基原点的位姿：R 用机体系表达（R_B 取单位阵），p 为机体系位置。
    void ComputeBodyPoses(const Vec12d& q, std::array<Mat3d, kNumBodies>* R,
                          std::array<Vec3d, kNumBodies>* p) const {
        for (int i = 0; i < kNumBodies; ++i) {
            const BodyParams& b = bodies[i];
            if (b.parent < 0) {
                (*R)[i] = Mat3d::Identity();
                (*p)[i] = Vec3d::Zero();
            } else {
                const int par = b.parent;
                const double angle = (b.joint_col >= 0) ? q(b.joint_col) : 0.0;
                (*R)[i] = (*R)[par] * AxisAngle(b.axis, angle);
                (*p)[i] = (*p)[par] + (*R)[par] * b.offset;
            }
        }
    }

    /// 计算各刚体的世界位姿 R 与足端/刚体雅可比 Jp（原点线速度）、Jr（角速度）。
    void ComputeBodyJacobians(const Vec12d& q, const Mat3d& R_B,
                              std::array<Mat3d, kNumBodies>* R,
                              std::array<Mat3x18d, kNumBodies>* Jp,
                              std::array<Mat3x18d, kNumBodies>* Jr) const {
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
            } else {
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
