#pragma once

#include "00_lite3_model.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace mpc_wbc::closed_loop {

using ContactSet = std::array<bool, kNumLegs>;

struct BaseMotion {
    Mat3d R_WB = Mat3d::Identity();
    Vec3d position_W = Vec3d::Zero(), velocity_W = Vec3d::Zero();
    Vec3d acceleration_W = Vec3d::Zero();
    Vec3d omega_B = Vec3d::Zero(), alpha_B = Vec3d::Zero();
};

struct PointTrajectory {
    Vec3d position = Vec3d::Zero();
    Vec3d velocity = Vec3d::Zero();
    Vec3d acceleration = Vec3d::Zero();
};

// All derivatives refer to the SAME R_WB and base origin. If a yaw-only frame
// is chosen later, its own angular velocity/acceleration must be supplied.
inline PointTrajectory BodyPointToWorld(const BaseMotion& base, const PointTrajectory& point_B) {
    PointTrajectory out;
    const Vec3d r = base.R_WB * point_B.position;
    const Vec3d v_rel = base.R_WB * point_B.velocity;
    const Vec3d omega = base.R_WB * base.omega_B;
    const Vec3d alpha = base.R_WB * base.alpha_B;
    out.position = base.position_W + r;
    out.velocity = base.velocity_W + omega.cross(r) + v_rel;
    out.acceleration = base.acceleration_W + alpha.cross(r) + omega.cross(omega.cross(r))
                     + 2.0 * omega.cross(v_rel) + base.R_WB * point_B.acceleration;
    return out;
}

// Frozen-shape SRBD conversion only: I_W and every quantity in the Newton/Euler
// equation are world-frame. Joint momentum and I_dot are excluded assumptions.
inline Vec3d SrbdAngularAccelerationWorld(const Mat3d& inertia_W, const Vec3d& omega_W,
                                          const Vec3d& moment_W) {
    return inertia_W.ldlt().solve(moment_W - omega_W.cross(inertia_W * omega_W));
}

inline Vec3d SwingTaskRhs(const Vec3d& acceleration_ref_W, const Vec3d& position_error_W,
                          const Vec3d& velocity_error_W, const Vec3d& jdot_v_W,
                          double kp, double kd) {
    return acceleration_ref_W + kp * position_error_W + kd * velocity_error_W - jdot_v_W;
}

// Cache provenance must survive failed solves. Never restamp an old solution
// with the time of a new request. Epoch changes on reset, model or reference reset.
struct SolutionStamp {
    double state_time = 0.0;
    std::uint64_t epoch = 0;
    ContactSet contacts{{false, false, false, false}};
    bool valid = false;

    bool Usable(double now, double max_age, std::uint64_t current_epoch,
                const ContactSet& current_contacts) const {
        const double age = now - state_time;
        return valid && std::isfinite(age) && std::isfinite(max_age) && max_age >= 0.0
            && age >= 0.0 && age <= max_age && epoch == current_epoch && contacts == current_contacts;
    }
};

using JointCommand = Eigen::Matrix<float, kNumJoints, 5>;

// New WBC execution contract: tau is the TOTAL joint torque, not an extra torque
// superimposed on baseline PD. Zero gains avoid state-dependent cancellation in
// an asynchronous actuator. Switching/blending is a later separately tested layer.
inline JointCommand TotalTorqueCommand(const Vec12d& tau, double limit) {
    if (!tau.allFinite() || !std::isfinite(limit) || limit <= 0.0
        || tau.cwiseAbs().maxCoeff() > limit)
        throw std::invalid_argument("invalid or out-of-limit total torque");
    JointCommand cmd = JointCommand::Zero();
    cmd.col(4) = tau.cast<float>();
    return cmd;
}
}  // namespace mpc_wbc::closed_loop
