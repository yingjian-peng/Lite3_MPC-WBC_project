#include "baseline_profile.hpp"
#include "closed_loop/contracts.hpp"
#include "closed_loop/model.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <limits>

using namespace mpc_wbc;
using namespace mpc_wbc::closed_loop;
int failures = 0;
void Check(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    if (!condition) ++failures;
}
Mat3d RotationVector(const Vec3d& v) {
    return v.norm() < 1e-14 ? Mat3d::Identity()
                          : Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
}

void Contracts() {
    BaseMotion base;
    base.R_WB = RpyToRotation(Vec3d(0.2, -0.1, 1.2));
    base.position_W = Vec3d(0.2, 0.1, 0.35);
    base.velocity_W = Vec3d(0.3, -0.1, 0.02);
    base.acceleration_W = Vec3d(0.2, 0.05, 0.1);
    base.omega_B = Vec3d(0.5, -0.3, 0.8);
    base.alpha_B = Vec3d(-0.2, 0.1, 0.4);
    PointTrajectory point;
    point.position = Vec3d(0.17, -0.15, -0.3);
    point.velocity = Vec3d(-0.1, 0.2, 0.05);
    point.acceleration = Vec3d(0.3, -0.1, 0.2);
    auto position = [&](double t) -> Vec3d {
        const Mat3d R = base.R_WB * RotationVector(base.omega_B * t + 0.5 * base.alpha_B * t * t);
        return base.position_W + t * base.velocity_W + 0.5 * t * t * base.acceleration_W
             + R * (point.position + t * point.velocity + 0.5 * t * t * point.acceleration);
    };
    const double h = 1e-4;
    const auto world = BodyPointToWorld(base, point);
    Check((world.velocity - (position(h) - position(-h)) / (2 * h)).norm() < 1e-7,
          "moving-frame foot velocity vs finite difference");
    Check((world.acceleration - (position(h) - 2 * position(0) + position(-h)) / (h * h)).norm() < 1e-6,
          "moving-frame foot acceleration vs finite difference (Coriolis included)");

    Mat3d Ib = Mat3d::Zero(); Ib.diagonal() << 0.12, 0.27, 0.35;
    const Mat3d Iw = base.R_WB * Ib * base.R_WB.transpose();
    const Vec3d moment(0.3, -0.2, 0.1), omega = base.R_WB * base.omega_B;
    const Vec3d alpha = SrbdAngularAccelerationWorld(Iw, omega, moment);
    const Vec3d body_expected = Ib.ldlt().solve(base.R_WB.transpose() * moment
                                               - base.omega_B.cross(Ib * base.omega_B));
    Check((base.R_WB.transpose() * alpha - body_expected).norm() < 1e-12,
          "world-frame Euler equation agrees with body-frame equation");
    const Mat3d Q = RpyToRotation(Vec3d(0.0, 0.0, 1.7));
    Check((SrbdAngularAccelerationWorld(Q * Iw * Q.transpose(), Q * omega, Q * moment)
            - Q * alpha).norm() < 1e-12, "angular dynamics invariant to initial heading");
    const Vec3d bias(0.1, -0.2, 0.3);
    Check((SwingTaskRhs(world.acceleration, Vec3d::Zero(), Vec3d::Zero(), bias, 200, 10)
            + bias - world.acceleration).norm() < 1e-12, "swing task cancels Jdot*v");

    ContactSet contacts{{true, false, false, true}};
    SolutionStamp stamp{1.0, 7, contacts, true};
    Check(stamp.Usable(1.05, 0.1, 7, contacts), "compatible fresh solution accepted");
    ContactSet switched{{false, true, true, false}};
    Check(!stamp.Usable(1.05, 0.1, 7, switched), "old contact-mode solution rejected");
    Check(!stamp.Usable(1.2, 0.1, 7, contacts) && !stamp.Usable(0.9, 0.1, 7, contacts)
          && !stamp.Usable(1.05, 0.1, 8, contacts), "expired, future and previous-epoch solutions rejected");

    Vec12d torque; torque.setLinSpaced(-25, 25);
    const auto command = TotalTorqueCommand(torque, 30);
    Vec12d applied;
    for (int i = 0; i < 12; ++i)
        applied(i) = command(i, 0) * (command(i, 1) - 10.0)
                   + command(i, 2) * (command(i, 3) + 5.0) + command(i, 4);
    Check((applied - torque).norm() < 1e-5, "total torque independent of delayed q/dq");
    bool rejected = false;
    try { (void)TotalTorqueCommand(Vec12d::Constant(30.1), 30); }
    catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "out-of-limit optimizer torque rejected before execution");

    const auto schedule = BaselineSchedule();
    Check(std::fabs(schedule.Select({0.15, 0, 0}).period - 1.20) < 1e-12
          && std::fabs(schedule.Select({0.3, 0, 0}).period - 0.75) < 1e-12
          && std::fabs(schedule.Select({0.6, 0, 0}).period - 0.44) < 1e-12,
          "frozen baseline speed segments preserved");
    const auto limited = LimitEnvelope({0.6, 0.35, 0.5}, EnvelopeParams());
    Check(std::fabs(std::hypot(limited.vx, limited.vy) + 0.18 * std::fabs(limited.wz) - 0.67) < 1e-12,
          "baseline three-axis envelope");
    Check(std::fabs(RampCommand({0.3, 0, 0}, {}, 0.1, 1.5, 0.75).vx - 0.225) < 1e-12,
          "baseline deceleration ramp");
}

template<class T> void Read(std::istream& input, T& value) {
    for (int r = 0; r < value.rows(); ++r)
        for (int c = 0; c < value.cols(); ++c) input >> value(r, c);
}

void Oracle(const char* path) {
    std::ifstream input(path);
    int count = 0; input >> count;
    Check(input.good() && count > 0, "oracle file available");
    if (count <= 0) return;
    Lite3Model model;
    double m_error = 0, h_error = 0, jac_error = 0, acceleration_error = 0;
    for (int i = 0; i < count; ++i) {
        Vec12d q; Vec18d velocity; Mat3d R; Vec3d gravity;
        ActuatorDynamics actuator;
        Mat18d mass; Vec18d bias, applied, qacc; Mat3x18d com_jac;
        Read(input, q); Read(input, velocity); Read(input, R); Read(input, gravity);
        Read(input, actuator.armature); Read(input, actuator.damping);
        Read(input, mass); Read(input, bias); Read(input, com_jac);
        Read(input, applied); Read(input, qacc);
        if (!input) { Check(false, "oracle is complete"); return; }
        const auto terms = EffectiveDynamics(model, q, velocity, R, gravity, actuator);
        m_error = std::max(m_error, (terms.M - mass).cwiseAbs().maxCoeff());
        h_error = std::max(h_error, (terms.h - bias).cwiseAbs().maxCoeff());
        jac_error = std::max(jac_error, (model.CompositeComJacobianWorld(q, R) - com_jac).cwiseAbs().maxCoeff());
        acceleration_error = std::max(acceleration_error,
            (terms.M.ldlt().solve(applied - terms.h) - qacc).cwiseAbs().maxCoeff());
    }
    std::cout << "oracle max errors M=" << m_error << " h=" << h_error
              << " Jcom=" << jac_error << " qacc=" << acceleration_error << '\n';
    Check(m_error < 1e-9 && h_error < 1e-9 && jac_error < 1e-9 && acceleration_error < 1e-7,
          "effective model vs MuJoCo (official, baseline and nonuniform actuators)");
}
int main(int argc, char** argv) {
    Contracts();
    if (argc > 1) Oracle(argv[1]);
    return failures ? 1 : 0;
}
