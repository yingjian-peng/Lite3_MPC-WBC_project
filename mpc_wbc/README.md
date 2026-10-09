# MPC+WBC 算法目录

本目录用于 Lite3 平地运动控制的 MPC+WBC 实现。实现与 ONNX 策略**互斥**运行，复用
`interface/` 中的机器人状态、用户命令和关节命令接口；状态切换由 `state_machine/` 管理。
施工步骤见 [`docs/04_Lite3_MPC_WBC实现步骤与里程碑.md`](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)。

## 当前进度

| 步骤 | 内容 | 状态 |
| --- | --- | --- |
| S0 | 约定冻结与骨架 | ✅ CMake target（`BUILD_MPC_WBC_TESTS`）已接入 |
| S1 | 运动学 FK / IK / 雅可比 + MuJoCo 对表 | ✅ 通过 |
| S2 | 全身动力学 M(q) / h(q,qvel) / Jc + MuJoCo 对表 | ✅ 通过 |
| S3 | 步态与足端轨迹（trot + 静立） | ✅ 通过 |
| S4 | 位置环跑步基线（MuJoCo） | ✅ 通过（见下） |
| S5 | 状态估计（基座速度/高度/触地可信度） | ✅ 通过 |

## 文件

| 文件 | 说明 |
| --- | --- |
| `00_lite3_model.hpp` | Lite3 平地基座模型：运动学（FK / IK / 单腿与全身足端雅可比）与全身动力学（`MassMatrix` / `BiasForce` / `ContactJacobianWorld`）。 |
| `01_state_estimator.hpp` | 状态估计：支撑腿零速反解基座速度（medoid 稳健选择）、支撑足恢复高度、水平短时积分、`valid/confidence`。 |
| `02_gait_trajectory.hpp` | trot 步态与足端轨迹：相位/支撑判据、支撑后扫、摆动五次曲线 + 余弦抬脚、停步滞回、未来支撑表。 |
| `apps/position_loop_sim.cpp` | S4 位置环跑步程序：站立斜坡 + 步态/IK + 关节 PD，通过 UDP 与仿真通信。 |
| `CMakeLists.txt` | `mpc_wbc` INTERFACE 库、对表/自检 target（`BUILD_MPC_WBC_TESTS`）与仿真程序（`BUILD_MPC_WBC_SIM`）。 |
| `test/gen_mujoco_cases.py` | 运动学真值：随机位姿 + `mj_forward` / `mj_jacBody`。 |
| `test/lite3_kinematics_test.cpp` | 比较 FK / 雅可比 / IK 回代。 |
| `test/gen_mujoco_dynamics_cases.py` | 动力学真值：随机位姿与广义速度 + `mj_fullM` / `qfrc_bias` / `mj_jacBody`。 |
| `test/lite3_dynamics_test.cpp` | 比较 `M(q)` / `h(q,qvel)` / `Jc`，并检查 M 的对称正定与质量块。 |
| `test/lite3_gait_trajectory_test.cpp` | 步态自检（停步、可达性/限位、连续性、支撑速度、支撑表）。 |
| `test/gen_estimator_rollout.py` | 离线 MuJoCo 小跑 rollout（含真值与计划接触），也可作 S6/S7 参考。 |
| `test/lite3_state_estimator_test.cpp` | 回放 rollout 比较估计与真值，并检查无效标志。 |
| `test/run_s4_position_loop.sh` + `analyze_s4_log.py` | S4 验收：无头仿真 + 位置环程序 + 日志分析。 |
| `test/run_*_test.sh` | 一键编译 + 运行各自测试/验证（独立于 CMake）。 |

## 约定（与 `docs/02` §3 一致）

- 机体系 `B`：`x` 向前、`y` 向左、`z` 向上。
- 广义速度 `qvel = [v_world(3); omega_body(3); dq(12)]`（与 MuJoCo free joint 一致）。
- 关节顺序 `FL, FR, HL, HR`，每腿 `HipX, HipY, Knee`。
- 关节角就是 MuJoCo 关节角：`HipX` 轴为 `(-1,0,0)`，`HipY/Knee` 轴为 `(0,-1,0)`。
- `FootPositionBody()` 返回 `FOOT` body 原点（球足球心）；真实触地点在球心下方 `foot_radius` 处。

## 运行对表

方式一（独立脚本，生成 + 编译 + 运行，不依赖 CMake 配置）：

```bash
mpc_wbc/test/run_kinematics_test.sh 500
mpc_wbc/test/run_dynamics_test.sh 200
mpc_wbc/test/run_gait_trajectory_test.sh
mpc_wbc/test/run_state_estimator_test.sh 10
```

方式二（纳入 CMake 构建，由 CTest 管理）：

```bash
cmake -S . -B build -DBUILD_MPC_WBC_TESTS=ON
cmake --build build -j
cd build && ctest -R mpc_wbc --output-on-failure
```

依赖：系统 `python3` + `mujoco`（3.x）、`g++`（C++17）、仓库内 Eigen。`BUILD_MPC_WBC_TESTS` 默认关闭，不影响 ONNX 仿真/真机构建。

### 运行 S4 位置环跑步（MuJoCo，无头）

```bash
cmake -S . -B build -DBUILD_MPC_WBC_SIM=ON
cmake --build build -j
mpc_wbc/test/run_s4_position_loop.sh 24     # 启动无头仿真 + 控制器 + 自动分析
```

脚本默认关闭 viewer（`MUJOCO_VIEWER=0`）；手动可视化时先 `python3 interface/robot/simulation/mujoco_simulation.py`，再运行 `build/mpc_wbc/mpc_wbc_position_loop_sim`。

### S1 对表结果（500 个随机位姿，覆盖真实关节范围）

| 项 | 误差 | 阈值 |
| --- | --- | --- |
| FK（机体系） | `3.3e-16` | `< 1e-9` |
| 单腿雅可比（机体系） | `8.5e-11` | `< 1e-7` |
| 全身足端雅可比（世界系） | `7.7e-11` | `< 1e-7` |
| IK 回代 `FK(IK(p))` | `1.7e-16` | `< 1e-9` |
| IK 带种子恢复关节角 | `2.2e-14` | `< 1e-9` |

### S2 对表结果（200 个随机位姿 + 广义速度）

| 项 | 误差 | 阈值 |
| --- | --- | --- |
| 质量矩阵 `M(q)` | `7.2e-16` | `< 1e-9` |
| bias `h(q,qvel)` | `5.7e-14` | `< 1e-9` |
| 接触雅可比 `Jc` | `5.0e-16` | `< 1e-9` |
| `M` 左上 3×3 = `m_total·I` | `0` | `< 1e-9` |
| `M` 不对称度 | `0` | `< 1e-9` |
| `M` 最小特征值 | `1.27e-3` | `> 0` |

### S3 自检结果

| 检查 | 结果 |
| --- | --- |
| 零命令停步（四足静立、足位 = 名义足位） | ✅ |
| 起步/停步滞回 | ✅ |
| 六种单轴 + 混合命令下足目标可达（IK 成功、无越限） | ✅ |
| 足端位置全程连续（每步 < `1.13e-4`，最大足速约 1.3 m/s） | ✅ |
| 支撑相机体系速度 = `-(v + ω×p0)`（误差 `7.4e-12`） | ✅ |
| 未来支撑表与逐点接触判据一致 | ✅ |

### S4 位置环跑步结果（无头 MuJoCo，24 s）

| 阶段 | Δx (m) | Δy (m) | Δyaw | max tilt | min base z |
| --- | --- | --- | --- | --- | --- |
| walk_fwd | `+0.186` | `0.005` | `2.2°` | `1.0°` | `0.331` |
| walk_back | `-0.340` | `-0.013` | `-0.7°` | `2.0°` | `0.332` |
| walk_left | `-0.137` | `+0.252` | `3.6°` | `1.5°` | `0.332` |
| turn | `-0.127` | `-0.047` | `+34.1°` | `1.2°` | `0.332` |

站立之后机身最大倾斜 `3.2°`、最低机身高 `0.331 m`，全程未跌倒，程序干净退出。位置环为开环、无姿态/力反馈，故侧移段存在少量前向漂移；这正是 S5–S7 要解决的问题。

### S5 状态估计结果（MuJoCo 小跑 rollout，10 s）

| 项 | 误差 | 阈值 |
| --- | --- | --- |
| 有效步占比 | `99.9%`（978/979） | `≥ 80%` |
| 基座高度 | max `1.2 mm` / mean `0.2 mm` | `< 0.02 m` |
| 基座速度 | max `0.144 m/s` / mean `0.017 m/s` | `< 0.20 m/s` |
| 基座水平位置（短时积分，10 s） | max `0.066 m` | `< 0.15 m` |
| 全摆动 → 无效 | ✅ | — |
| 时间间隔过大（丢包）→ 无效 | ✅ | — |

方案：支撑腿零速 `v_foot≈0` 反解 `v_B = −(J_all[:,3:6]ω_B + J_all[:,6:18]dq)`，用 **medoid + 一致子集** 稳健选择支撑腿；高度由 `p_B.z = r − (R·p_foot^B).z` 恢复并直接覆盖 z；水平位置仅在速度可信时短时积分。要求一致支撑 ≥2（或本就只有 1 条支撑腿）才判有效。

### 已知待办 / 边界

- `IK` 的 HipX 侧摆有两个分支（都精确到达同一目标）。控制中应传上一帧关节角作为 `q_seed`，取最近解以保证连续性。
- 触地点与 `FOOT` body 原点存在 `foot_radius` 偏移；接触雅可比与视觉轨迹的参考点需分别标注（S5/S7 处理）。
- 关节限位、腿长等参数来源是 MJCF；`ControlParameters` 中的数值（如 `shank_len_=0.21`、`hip_len_=0.0985`）与 MJCF（`0.21012`、`0.09735`）不一致，后续以 MJCF 对表结果为准。
- `BiasForce()` 的重力作为参数传入。MuJoCo 默认 `(0,0,-9.81)`，而 `support/types/common_types.h` 里 `gravity=9.815`；接入控制器时必须统一。
- `ContactJacobianWorld()` 用球足最低点（世界系 `-z` 偏移 `foot_radius`）作为接触点，仅对平地法向成立；WBC 需要 `Jdot·v` 项在 S7 补。
- 动力学目前用「各刚体惯性广义力求和」（与 RNEA 等价）实现；若需更快的在线计算，S7 再评估 CRBA/RNEA 优化。
- 步态支撑相用机体系线性后扫（开环）；世界系足端锚点锁定需要基座位置/速度估计，留到 S5 之后。
- 停步为「立即回到名义足位」，未做收步过渡（S9/S10 平滑）。
- 摆动起落点与支撑相速度在切换瞬间不连续（五次曲线端点速度为 0，支撑相为 `-v`）；位置连续，速度跳变属已知简化。
- `StateEstimator` 当前**不用 IMU 加速度**融合（其是否含重力的约定需实测核对），速度完全来自支撑腿零速；接触输入取“计划支撑集”，落地提前/延迟与足端滑移会带来误差（本轮小跑速度误差峰值 `0.144 m/s`）。
- 水平位置为开环短时积分，长时间会漂移；世界系足端锚点更新待后续版本。

