# mpc_wbc 目录

Lite3 的 **SRBD-MPC + Weighted-QP WBC** 全闭环控制器（header-only C++17）。
控制链：`估计 → 步态/足端轨迹 → SRBD-MPC → WBC → 12×5 关节命令`。

工程总入口见 [根 README](../README.md)，指标与验收表见
[docs/00 设计指标](../docs/00_设计指标.md)。

## 代码入口

| 文件 | 用途 |
| --- | --- |
| `00_lite3_model.hpp` | 运动学、雅可比、全身动力学、质心惯量 |
| `01_state_estimator.hpp` | 基座速度、高度、估计有效性 |
| `02_gait_trajectory.hpp` | trot 相位、足端位置/速度目标、未来支撑表 |
| `03_srbd_mpc.hpp` + `box_qp.hpp` | SRBD 预测控制与接触力盒约束 QP |
| `04_weighted_wbc.hpp` + `qp_ipm.hpp` | 全身控制加权 QP 与内点求解器 |
| `05_mpc_wbc_controller.hpp` | 编排：命令级约束链 + 1 kHz Step + 独立 MPC 线程 |
| `apps/mpc_wbc_sim.cpp` | 离线闭环 harness（UDP ↔ MuJoCo） |
| `apps/position_loop_sim.cpp` | 开环位置环基线（对照用） |

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

产出 `build/mpc_wbc/mpc_wbc_mpc_wbc_sim` 与 `build/mpc_wbc/mpc_wbc_position_loop_sim`
（`-DBUILD_MPC_WBC_SIM=OFF` 可关闭）。

## 当前状态

指标已达标（详见 [docs/00 §5](../docs/00_设计指标.md)）：

- 单轴：$v_x$ 0.10–0.60 m/s、$v_y$ 0.10–0.50 m/s、$\omega_z$ 0.10–1.00 rad/s；
- 组合：一条线性归一化 $v_{xy}+0.18\lvert\omega_z\rvert\le0.67$（超限同比缩比）；
- 全部工况倾角 ≤8.6°，无翻车；实测失稳点 $\omega_z\approx2.5$ rad/s、$v_y\approx0.80$ m/s。

**本项目只面向 MuJoCo 仿真，不做真机部署**（边界见 [docs/00 §7](../docs/00_设计指标.md)）。

## 验收怎么跑

```bash
# 一键全套（单轴 / 组合 / 三轴满量程，默认开 viewer，约 12 分钟）
mpc_wbc/test/run_acceptance.sh

# 只跑指定项
mpc_wbc/test/run_acceptance.sh vx vy

# 单轴扫速（另一套场景，±4 档 × 8 s）
mpc_wbc/test/run_single_axis.sh wz
```

`mpc_wbc/test/` 下的文件清单与用途见 [test/README.md](test/README.md)。
已证伪的调参方向与失败记录见 [docs/05 优化日志](../docs/05_优化日志.md)。
