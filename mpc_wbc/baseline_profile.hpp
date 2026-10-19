#pragma once

#include "02_gait_trajectory.hpp"
#include <algorithm>
#include <cmath>

namespace mpc_wbc {

// Frozen openloop-baseline-v1 configuration. New feedback controllers must use
// this profile explicitly rather than relying on GaitParams' historical defaults.
struct EnvelopeParams {
    double vx_max = 0.60, vy_max = 0.35, wz_max = 0.50;
    double s_max = 0.67, wz_weight = 0.18;
    bool enabled = true;
};

inline SegmentedGaitParams BaselineSchedule(const GaitParams& base = GaitParams()) {
    SegmentedGaitParams schedule;
    schedule.low = schedule.mid = schedule.high = base;
    schedule.low.period = 1.20;
    schedule.low.duty = 0.72;
    schedule.mid.period = 0.75;
    schedule.mid.duty = 0.70;
    schedule.high.period = 0.44;
    schedule.high.duty = 0.625;
    for (GaitParams* p : {&schedule.low, &schedule.mid, &schedule.high}) {
        p->swing_height = 0.07;
        p->max_vx = schedule.max_vx;
        p->max_vy = schedule.max_vy;
        p->max_wz = schedule.max_wz;
    }
    return schedule;
}

struct BaselineProfile {
    double kp = 200.0, kd = 2.5;
    double nominal_x = -0.0133;
    double ramp_accel = 1.5, ramp_decel = 0.75;
    double foot_blend_time = 0.8;
    EnvelopeParams envelope;
    SegmentedGaitParams schedule = BaselineSchedule();
};

inline double Approach(double current, double target, double max_delta) {
    const double d = target - current;
    if (std::fabs(d) <= max_delta) return target;
    return current + (d >= 0.0 ? max_delta : -max_delta);
}

inline GaitCommand LimitEnvelope(const GaitCommand& in, const EnvelopeParams& env) {
    GaitCommand out = in;
    if (!env.enabled) return out;
    if (env.vx_max > 0.0) out.vx = std::clamp(out.vx, -env.vx_max, env.vx_max);
    if (env.vy_max > 0.0) out.vy = std::clamp(out.vy, -env.vy_max, env.vy_max);
    if (env.wz_max > 0.0) out.wz = std::clamp(out.wz, -env.wz_max, env.wz_max);
    const double s = std::hypot(out.vx, out.vy) + env.wz_weight * std::fabs(out.wz);
    if (env.s_max > 0.0 && s > env.s_max) {
        const double k = env.s_max / s;
        out.vx *= k; out.vy *= k; out.wz *= k;
    }
    return out;
}

inline GaitCommand RampCommand(const GaitCommand& current, const GaitCommand& target,
                               double dt, double accel, double decel) {
    if (dt <= 0.0) return current;
    auto next = [&](double a, double b) {
        if (accel <= 0.0) return b;
        // Preserve the frozen baseline's reversal behavior exactly.
        return Approach(a, b, (std::fabs(b) < std::fabs(a) ? decel : accel) * dt);
    };
    return {next(current.vx, target.vx), next(current.vy, target.vy),
            next(current.wz, target.wz)};
}

inline GaitCommand LimitStep(const GaitCommand& in, double stance_time,
                              const Vec12d& nominal, double step_x, double step_y) {
    double s = 1.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3d p = nominal.segment<3>(3 * leg);
        const double dx = stance_time * (in.vx - in.wz * p.y());
        const double dy = stance_time * (in.vy + in.wz * p.x());
        if (step_x > 0.0 && std::fabs(dx) > step_x) s = std::min(s, step_x / std::fabs(dx));
        if (step_y > 0.0 && std::fabs(dy) > step_y) s = std::min(s, step_y / std::fabs(dy));
    }
    return {in.vx * s, in.vy * s, in.wz * s};
}
}  // namespace mpc_wbc
