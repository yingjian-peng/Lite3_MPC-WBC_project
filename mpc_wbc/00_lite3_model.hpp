/**
 * @file 00_lite3_model.hpp
 * @brief Lite3 平地基座模型：运动学（FK/IK/雅可比）。动力学（M/h/Jc）在后续步骤补入。
 *
 * 数值来源：robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3.xml（以 MuJoCo 对表为准）。
 * 坐标约定（与 docs/02 §3 一致）：
 *   机体系 B：x 向前、y 向左、z 向上（右手系）。
 *   广义速度 qvel = [v_world(3); omega_body(3); dq(12)]，与 MuJoCo free joint 一致。
 * 关节顺序：FL, FR, HL, HR，每腿 HipX, HipY, Knee（与 MJCF / SDK 一致）。
 *
 * 符号约定（关键）：MuJoCo 中 HipX 轴为 (-1,0,0)，HipY/Knee 轴为 (0,-1,0)。
 *   因此本文件中的 qx/qy/qk 直接就是 MuJoCo 关节角，旋转分别为 Rx(-qx)、Ry(-qy)、Ry(-qk)。
 *   这与 docs/03 §1.2 的公式在 HipX 侧摆符号上不同；本文件以 MJCF 为准。
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

enum LegIndex { kFL = 0, kFR = 1, kHL = 2, kHR = 3 };
enum JointIndex { kHipX = 0, kHipY = 1, kKnee = 2 };

using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec12d = Eigen::Matrix<double, kNumJoints, 1>;
using Vec18d = Eigen::Matrix<double, kNumGenVel, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat12x18d = Eigen::Matrix<double, kNumContactForces, kNumGenVel>;
using Mat3x12d = Eigen::Matrix<double, 3, kNumJoints>;

// ---------------------------------------------------------------------------
// 几何参数（MJCF 实测值）
// ---------------------------------------------------------------------------
struct Lite3Geometry {
    double thigh_len = 0.20;          // FL_THIGH -> FL_SHANK 的 z 偏移（大腿）
    double shank_len = 0.21012;       // FL_SHANK -> FL_FOOT 的 z 偏移（小腿）
    double hip_lateral = 0.09735;     // FL_HIP -> FL_THIGH 的 |y| 偏移（髋侧向偏置）
    double foot_radius = 0.022;       // FL_FOOT 球足半径

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
    //   - 不传 seed 时，取 |qx| 较小的解（docs/03 §1.3 的做法）。
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
    // 默认站立关节角（docs/03 附录 C：每腿 [0, -0.65, 1.3]）
    // -----------------------------------------------------------------------
    static Vec3d DefaultStandLegAngles() { return Vec3d(0.0, -0.65, 1.3); }
    static Vec12d DefaultStandJointAngles() {
        Vec12d q;
        for (int leg = 0; leg < kNumLegs; ++leg) q.segment<3>(3 * leg) = DefaultStandLegAngles();
        return q;
    }
};

}  // namespace mpc_wbc
