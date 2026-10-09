/**
 * @file lite3_state_estimator_test.cpp
 * @brief S5 状态估计验证：回放 MuJoCo rollout，比较估计与真值；并检查无效标志。
 *
 * 用法： ./lite3_state_estimator_test <rollout.csv>
 */
#include "01_state_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace mpc_wbc;

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "mpc_wbc/test/estimator_rollout.csv";
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open rollout: %s\n", path.c_str());
        return 2;
    }

    std::string line;
    std::getline(in, line);  // 跳过表头

    StateEstimator est;
    bool first = true;
    double max_h_err = 0.0, max_v_err = 0.0, max_xy_err = 0.0;
    double sum_h_err = 0.0, sum_v_err = 0.0;
    long n = 0, stat_n = 0, valid_count = 0;
    int invalid_steps = 0;
    double last_t = 0.0;
    double t_max_h = 0.0, t_max_v = 0.0, t_max_xy = 0.0;
    Vec3d dbg_est_v = Vec3d::Zero(), dbg_true_v = Vec3d::Zero();

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        auto next = [&](double& v) {
            std::getline(ss, tok, ',');
            v = std::stod(tok);
        };

        EstimatorInput e;
        Vec3d truth_pos, truth_vel, rpy, omega;
        next(e.t);
        next(truth_pos(0)); next(truth_pos(1)); next(truth_pos(2));
        next(truth_vel(0)); next(truth_vel(1)); next(truth_vel(2));
        next(rpy(0)); next(rpy(1)); next(rpy(2));
        next(omega(0)); next(omega(1)); next(omega(2));
        for (int i = 0; i < 12; ++i) next(e.q(i));
        for (int i = 0; i < 12; ++i) next(e.dq(i));
        for (int i = 0; i < 4; ++i) {
            double c = 0.0;
            next(c);
            e.contact[i] = (c != 0.0);
        }
        e.rpy = rpy;
        e.omega_body = omega;

        if (first) {
            est.Reset(e.t, Vec3d(truth_pos.x(), truth_pos.y(), 0.0));
            first = false;
        }

        const StateEstimate s = est.Update(e);
        if (!s.valid) ++invalid_steps;
        last_t = e.t;

        if (n > 20 && s.valid) {
            ++valid_count;
            const double h_err = std::fabs(s.height - truth_pos.z());
            const double v_err = (s.base_vel_w - truth_vel).norm();
            const double xy_err = (s.base_pos_w.head<2>() - truth_pos.head<2>()).norm();
            if (h_err > max_h_err) { max_h_err = h_err; t_max_h = e.t; }
            if (v_err > max_v_err) {
                max_v_err = v_err;
                t_max_v = e.t;
                dbg_est_v = s.base_vel_w;
                dbg_true_v = truth_vel;
            }
            if (xy_err > max_xy_err) { max_xy_err = xy_err; t_max_xy = e.t; }
            sum_h_err += h_err;
            sum_v_err += v_err;
            ++stat_n;
        }
        ++n;
    }

    std::printf("S5 state estimator vs MuJoCo truth (%ld steps)\n", n);
    std::printf("  valid steps: %ld / %ld (%.1f%%)\n", valid_count, n - 21,
                100.0 * valid_count / std::max(1L, n - 21));
    std::printf("  [valid-only] height   max=%.4f m @t=%.2f mean=%.4f m\n", max_h_err, t_max_h,
                sum_h_err / std::max(1L, stat_n));
    std::printf("  [valid-only] velocity max=%.4f m/s @t=%.2f mean=%.4f m/s\n", max_v_err, t_max_v,
                sum_v_err / std::max(1L, stat_n));
    std::printf("  [valid-only] position max=%.4f m @t=%.2f\n", max_xy_err, t_max_xy);
    std::printf("  invalid steps: %d / %ld\n", invalid_steps, n);
    std::printf("  worst velocity err @t=%.2f: est=(%.3f,%.3f,%.3f) true=(%.3f,%.3f,%.3f)\n", t_max_v,
                dbg_est_v.x(), dbg_est_v.y(), dbg_est_v.z(), dbg_true_v.x(), dbg_true_v.y(),
                dbg_true_v.z());

    bool ok = true;
    auto check = [&](bool cond, const char* name) {
        std::printf("  [%s] %s\n", cond ? "OK  " : "FAIL", name);
        ok = ok && cond;
    };
    check(max_h_err < 0.02, "height max error < 0.02 m (valid steps)");
    check(max_v_err < 0.20, "velocity max error < 0.20 m/s (valid steps)");
    check(max_xy_err < 0.15, "position max error < 0.15 m (valid steps)");
    check(valid_count >= 0.8 * (n - 21), "valid fraction >= 80%");

    // 无效标志：全摆动
    {
        EstimatorInput e;
        e.t = last_t + 0.01;
        e.q = Lite3Model::DefaultStandJointAngles();
        e.contact = {{false, false, false, false}};
        const StateEstimate s = est.Update(e);
        check(!s.valid && s.num_support == 0, "all-swing -> invalid");
    }
    // 无效标志：时间间隔过大（丢包）
    {
        EstimatorInput e;
        e.t = last_t + 0.5;
        e.q = Lite3Model::DefaultStandJointAngles();
        e.contact = {{true, true, true, true}};
        const StateEstimate s = est.Update(e);
        check(!s.valid, "large dt gap -> invalid");
    }

    std::printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
