// 复现控制器里的步态调用顺序：schedule 打开 + 一阶低通 + 原始命令判停。
#include "02_gait_trajectory.hpp"
#include <cstdio>

using namespace mpc_wbc;

int main() {
    SegmentedGaitParams sched = SegmentedGaitParams::BaselineClosedLoop();
    GaitTrajectory gait(GaitParams(), &sched);
    GaitCommand c;
    c.wz = 0.3;
    double t = 0.0;
    for (; t < 8.0; t += 0.001) gait.Update(t, c, false);
    std::printf("stepping=%d\n", gait.stepping() ? 1 : 0);

    GaitCommand filt = c;
    GaitCommand zero;
    bool exited = false;
    double t0 = t;
    for (; t < t0 + 4.0; t += 0.001) {
        filt.wz += (0.0 - filt.wz) * (0.001 / 0.25);
        const bool cmd_zero = gait.NormalizedSpeed(zero) < gait.params().stop_threshold;
        gait.Update(t, filt, cmd_zero);
        if (!gait.stepping()) {
            std::printf("exit at %.3f s after zero (T=%.3f)\n", t - t0, gait.params().period);
            exited = true;
            break;
        }
    }
    if (!exited) std::printf("NEVER EXITED within 4 s\n");
    return 0;
}
