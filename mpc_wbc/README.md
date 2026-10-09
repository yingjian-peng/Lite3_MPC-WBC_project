# MPC+WBC 算法目录

本目录用于 Lite3 平地运动控制的 MPC+WBC 实现。实现与 ONNX 策略**互斥**运行，复用
`interface/` 中的机器人状态、用户命令和关节命令接口；状态切换由 `state_machine/` 管理。
施工步骤见 [`docs/04_Lite3_MPC_WBC实现步骤与里程碑.md`](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)。

## 当前进度

| 步骤 | 内容 | 状态 |
| --- | --- | --- |
| S0 | 约定冻结与骨架 | ✅ CMake target（`BUILD_MPC_WBC_TESTS`）已接入 |
| S1 | 运动学 FK / IK / 雅可比 + MuJoCo 对表 | ✅ 通过 |

## 文件

| 文件 | 说明 |
| --- | --- |
| `00_lite3_model.hpp` | Lite3 平地基座模型。当前含运动学（FK / IK / 单腿与全身足端雅可比）；动力学（`M/h/Jc`）待 S2 补入。 |
| `CMakeLists.txt` | `mpc_wbc` INTERFACE 库与对表测试 target（`BUILD_MPC_WBC_TESTS`）。 |
| `test/gen_mujoco_cases.py` | 用 Python MuJoCo 生成随机位姿与真值（`mj_forward` / `mj_jacBody`）。 |
| `test/lite3_kinematics_test.cpp` | 读取真值，逐项比较本工程 FK / 雅可比 / IK 回代。 |
| `test/run_kinematics_test.sh` | 一键生成 + 编译 + 运行对表（独立于 CMake）。 |

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

### 已知待办 / 边界

- `IK` 的 HipX 侧摆有两个分支（都精确到达同一目标）。控制中应传上一帧关节角作为 `q_seed`，取最近解以保证连续性。
- 触地点与 `FOOT` body 原点存在 `foot_radius` 偏移；接触雅可比与视觉轨迹的参考点需分别标注（S5/S7 处理）。
- 关节限位、腿长等参数来源是 MJCF；`ControlParameters` 中的数值（如 `shank_len_=0.21`、`hip_len_=0.0985`）与 MJCF（`0.21012`、`0.09735`）不一致，后续以 MJCF 对表结果为准。

