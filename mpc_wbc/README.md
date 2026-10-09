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

## 文件

| 文件 | 说明 |
| --- | --- |
| `00_lite3_model.hpp` | Lite3 平地基座模型：运动学（FK / IK / 单腿与全身足端雅可比）与全身动力学（`MassMatrix` / `BiasForce` / `ContactJacobianWorld`）。 |
| `CMakeLists.txt` | `mpc_wbc` INTERFACE 库与对表测试 target（`BUILD_MPC_WBC_TESTS`）。 |
| `test/gen_mujoco_cases.py` | 运动学真值：随机位姿 + `mj_forward` / `mj_jacBody`。 |
| `test/lite3_kinematics_test.cpp` | 比较 FK / 雅可比 / IK 回代。 |
| `test/gen_mujoco_dynamics_cases.py` | 动力学真值：随机位姿与广义速度 + `mj_fullM` / `qfrc_bias` / `mj_jacBody`。 |
| `test/lite3_dynamics_test.cpp` | 比较 `M(q)` / `h(q,qvel)` / `Jc`，并检查 M 的对称正定与质量块。 |
| `test/run_kinematics_test.sh`、`test/run_dynamics_test.sh` | 一键生成 + 编译 + 运行对表（独立于 CMake）。 |

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
```

方式二（纳入 CMake 构建，由 CTest 管理）：

```bash
cmake -S . -B build -DBUILD_MPC_WBC_TESTS=ON
cmake --build build -j
cd build && ctest -R mpc_wbc --output-on-failure
```

依赖：系统 `python3` + `mujoco`（3.x）、`g++`（C++17）、仓库内 Eigen。`BUILD_MPC_WBC_TESTS` 默认关闭，不影响 ONNX 仿真/真机构建。

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

### 已知待办 / 边界

- `IK` 的 HipX 侧摆有两个分支（都精确到达同一目标）。控制中应传上一帧关节角作为 `q_seed`，取最近解以保证连续性。
- 触地点与 `FOOT` body 原点存在 `foot_radius` 偏移；接触雅可比与视觉轨迹的参考点需分别标注（S5/S7 处理）。
- 关节限位、腿长等参数来源是 MJCF；`ControlParameters` 中的数值（如 `shank_len_=0.21`、`hip_len_=0.0985`）与 MJCF（`0.21012`、`0.09735`）不一致，后续以 MJCF 对表结果为准。
- `BiasForce()` 的重力作为参数传入。MuJoCo 默认 `(0,0,-9.81)`，而 `support/types/common_types.h` 里 `gravity=9.815`；接入控制器时必须统一。
- `ContactJacobianWorld()` 用球足最低点（世界系 `-z` 偏移 `foot_radius`）作为接触点，仅对平地法向成立；WBC 需要 `Jdot·v` 项在 S7 补。
- 动力学目前用「各刚体惯性广义力求和」（与 RNEA 等价）实现；若需更快的在线计算，S7 再评估 CRBA/RNEA 优化。

