# Lite3 运动控制基础与闭环重建

2026-10-19：旧 MPC、WBC、两个 QP 求解器、控制器编排、状态机接入、闭环仿真 app 和专属测试已直接删除。当前以 `openloop-baseline-v1` 的运动学/关节 PD 基线重建；新优化器尚未实现。

数学定义见 [重新推导与基础契约](../docs/09_MPC_WBC重新推导与基础契约.md)，实施状态见 [重建进度](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)。

## 当前文件

| 文件 | 用途 |
| --- | --- |
| `00_lite3_model.hpp` | FK/IK、雅可比、刚体动力学、精确质心速度雅可比 |
| `01_state_estimator.hpp` | 基础运动学估计器，仍需新闭环在线复验 |
| `02_gait_trajectory.hpp` | trot、连续选段、软停步 |
| `baseline_profile.hpp` | 冻结参数、命令包络、步幅与斜坡公共定义 |
| `closed_loop/contracts.hpp` | 重新推导的坐标导数、总力矩执行与解缓存契约 |
| `closed_loop/model.hpp` | 显式加入关节电枢惯量和阻尼的有效动力学 |
| `apps/position_loop_sim.cpp` | 当前可运行的开环位置控制 |
| `test/run_baseline.py` | 无头/可视化回归、场景副本、参数、源码/二进制指纹、分析结果 |

## 构建与检查

在工程根目录：

```bash
cmake -S . -B build -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_MPC_WBC_SIM=ON -DBUILD_MPC_WBC_TESTS=ON
cmake --build build -j4
cd build
ctest -R mpc_wbc --output-on-failure
```

当前 12 项检查覆盖基础运动学、原刚体动力学、步态、基础估计器，以及新坐标导数/执行/缓存契约、UDP 快照/超时和有效动力学。150 个新对表用例涵盖原模型、基线电枢/阻尼和非均匀关节参数；没有旧 MPC/WBC 验收项。

## 运行开环

```bash
python3 mpc_wbc/test/run_baseline.py --suite single10s --out build/baseline_single10s
python3 mpc_wbc/test/run_baseline.py --scenario mpc_wbc/test/scenarios/axis_fwd.csv --out build/baseline_fwd --viewer
```

输出目录必须新建或为空。`--repeat 5` 重复五轮；`--init-yaw 1.5707963` 改初始航向。suite 每档前/后/左/右/逆时针/顺时针各 10 s，段间停 2 s。保存原始 CSV、环境、源码差异和二进制指纹。`WARN/FAIL` 会返回非零，不能只看控制程序是否正常退出。

低/中/高的 `T/d/H` 为 `1.20/0.72/0.07`、`0.75/0.70/0.07`、`0.44/0.625/0.07`；kp/kd 为 200/2.5，足位 x 配平 -0.0133 m，起停混合 0.8 s，加速/减速斜坡 1.5/0.75。

速度包络为单轴 0.60/0.35/0.50，并满足 `hypot(vx,vy)+0.18*abs(wz)<=0.67`。`S4_KP/KD`、`S4_RAMP/RAMP_DECEL`、`S4_BLEND`、`S4_NOM_X`、`S4_LOW_*/MID_*/HIGH_*` 等实验覆盖仍有效。

仿真默认电枢/阻尼/足端接触配置为 0.01/0.1/4/0.03，源于仿真标定。Python 仿真和对表共用 `interface/robot/simulation/model_config.py`；C++ 新动力学显式接收每关节实际参数，不能默认当作真机模型。

C++ 与 Python 各自异步运行。状态整帧发布，控制器按新状态推进；命令 100 ms 不更新时转阻尼。观察日志中的 `command_age_wall`、`command_expired` 及实际 `atau`，不能用仿真时间 1 ms 单独证明墙钟实时性。

## 保留的开环调参工具

`sweep_gait_params.py`、`tune_s4_segments.py`、`energy_metrics.py`、`run_robustness.sh` 仍用于开环。例：

```bash
python3 mpc_wbc/test/sweep_gait_params.py --segment high \
  --scenario mpc_wbc/test/scenarios/verify_single_high.csv \
  --configs 0.44:0.625:0.07 --duration 68 --outdir build/check_high
```

旧 A/B 脚本和旧闭环入口已删除。新闭环从估计/接触复验、连续参考、新 MPC 和新 WBC 顺序实施，完成独立验证后再接状态机。
