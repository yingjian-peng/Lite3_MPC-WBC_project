#pragma once

#include "00_lite3_model.hpp"
#include <stdexcept>

namespace mpc_wbc::closed_loop {
struct ActuatorDynamics {
    Vec12d armature = Vec12d::Zero();
    Vec12d damping = Vec12d::Zero();

    void Validate() const {
        if (!armature.allFinite() || !damping.allFinite()
            || armature.minCoeff() < 0.0 || damping.minCoeff() < 0.0)
            throw std::invalid_argument("armature/damping must be finite and nonnegative");
    }
};

struct DynamicsTerms { Mat18d M; Vec18d h; };

inline DynamicsTerms EffectiveDynamics(const Lite3Model& model, const Vec12d& q,
                                       const Vec18d& velocity, const Mat3d& R_WB,
                                       const Vec3d& gravity_W, const ActuatorDynamics& actuator) {
    actuator.Validate();
    DynamicsTerms out{model.MassMatrix(q, R_WB), model.BiasForce(q, velocity, R_WB, gravity_W)};
    out.M.bottomRightCorner<kNumJoints, kNumJoints>().diagonal() += actuator.armature;
    out.h.tail<kNumJoints>() += actuator.damping.cwiseProduct(velocity.tail<kNumJoints>());
    return out;
}
}  // namespace mpc_wbc::closed_loop
