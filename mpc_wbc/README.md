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
| S6 | SRBD 预测时域 MPC（逐脚力 + 期望 wrench） | ✅ 通过 |
| S7 | Weighted-QP WBC（自研 QP + MuJoCo 复核） | ✅ 通过 |
| S8 | 控制器编排 + 状态机线程 | ✅ 编排/线程/安全退化/站立通过；trot 行走稳定性移交 S9（见 [docs/04 §8](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)） |

## 文件

| 文件 | 说明 |
| --- | --- |
| `00_lite3_model.hpp` | Lite3 平地基座模型：运动学（FK / IK / 单腿与全身足端雅可比）、全身动力学（`MassMatrix` / `BiasForce` / `ContactJacobianWorld`）、整机质心与合成惯量（`CompositeComBody` / `CompositeInertiaBody`）。 |
| `01_state_estimator.hpp` | 状态估计：支撑腿零速反解基座速度（medoid 稳健选择）、支撑足恢复高度、水平短时积分、`valid/confidence`。 |
| `02_gait_trajectory.hpp` | trot 步态与足端轨迹：相位/支撑判据、支撑后扫、摆动五次曲线 + 余弦抬脚、停步滞回、未来支撑表。 |
| `03_srbd_mpc.hpp` | S6 预测时域 SRBD-MPC：N 步质心+姿态预测，决策为「支撑脚×节点×锥方向」的 λ≥0，输出第 0 节点逐脚力与世界系净力/净力矩。 |
| `box_qp.hpp` | 盒约束 QP（无约束解 → 主动集投影牛顿 → 坐标下降兜底），供 SRBD-MPC 求解接触力分配。 |
| `qp_ipm.hpp` | S7 自研稠密 QP（原-对偶内点 + Mehrotra 预测-校正）：`min ½xᵀHx+gᵀx s.t. A_eq x=b_eq, A_ge x≥b_ge`，零第三方依赖；返回 `x/y/z`（乘子）便于做 KKT 自检。 |
| `04_weighted_wbc.hpp` | S7 统一 Weighted-QP WBC：决策 `[q̈(18); f(12); τ(12)]`，硬约束 = EoM + 支撑无滑（含 `Jdot·v`）+ 摆动 `f=0` + 摩擦金字塔 + 法向力上限 + 力矩上下界；软任务 = 基座 6D + 摆动足 PD（+ 可选力参考/站姿/力矩正则）。`BuildQp` 可独立取出组装好的 QP 做最优性复核。 |
| `05_mpc_wbc_controller.hpp` | S8 控制器编排：估计 → 步态 → MPC 请求快照 → WBC → 12×5 命令（无线程、无 IO、不继承 `StateBase`）；`SeqLockBox<T>` 无锁快照 + `MpcWorkerThread`（MPC 独立线程）；命令低通 + 变化率限幅、WBC 使能滞回、质心高度软启动、`wrench_max_age` 过期降级、故障位。 |
| `apps/mpc_wbc_sim.cpp` | S8 离线整机 harness：复刻 `StandUpState` 两阶段起立 + 条件交棒，走 UDP 与 MuJoCo 通信，输出相位表与 32 列诊断 CSV；`S8_PHASES=stand|walk`、`S8_PD_SCALE/S8_W_SWING/S8_MPC_PERIOD/S8_HYST/S8_CMD_SCALE` 可调参。 |
| `apps/position_loop_sim.cpp` | S4 位置环跑步程序：站立斜坡 + 步态/IK + 关节 PD，通过 UDP 与仿真通信。 |
| `../state_machine/mpc_control_state.hpp` | S8 状态机：1 kHz `Run()` 唯一 `SetJointCommand()` 写入权 + MPC 工作线程；`LoseControlJudge`（姿态/故障）与 `GetNextStateName`（→ `kLieDown`/`kRLControl`）。 |
| `CMakeLists.txt` | `mpc_wbc` INTERFACE 库、对表/自检 target（`BUILD_MPC_WBC_TESTS`）与仿真程序（`BUILD_MPC_WBC_SIM`）。 |
| `test/gen_mujoco_cases.py` | 运动学真值：随机位姿 + `mj_forward` / `mj_jacBody`。 |
| `test/lite3_kinematics_test.cpp` | 比较 FK / 雅可比 / IK 回代。 |
| `test/gen_mujoco_dynamics_cases.py` | 动力学真值：随机位姿与广义速度 + `mj_fullM` / `qfrc_bias` / `mj_jacBody`。 |
| `test/lite3_dynamics_test.cpp` | 比较 `M(q)` / `h(q,qvel)` / `Jc` / 整机质心 / 合成惯量，并检查 M 的对称正定与质量块。 |
| `test/lite3_gait_trajectory_test.cpp` | 步态自检（停步、可达性/限位、连续性、支撑速度、支撑表）。 |
| `test/gen_estimator_rollout.py` | 离线 MuJoCo 小跑 rollout（含真值与计划接触），也可作 S6/S7 参考。 |
| `test/lite3_state_estimator_test.cpp` | 回放 rollout 比较估计与真值，并检查无效标志。 |
| `test/lite3_srbd_mpc_test.cpp` | S6 离线自检：悬停承重、速度/偏航响应、摩擦锥与接触计划、姿态恢复、求解耗时与失败兜底、闭环跟踪。 |
| `test/qp_ipm_test.cpp` | S7 QP 单测：解析解、与盒约束 QP 交叉、40 组随机 KKT、退化 NNLS、冗余等式、无不等式、不可行识别。 |
| `test/lite3_wbc_test.cpp` | S7 WBC 离线自检：`Jdot·v` 对表、静立承重、基座 6D 跟踪、摆动任务、KKT 最优性证书、权重单调性、极限工况、MPC 串联、耗时、失败兜底、PD 反解，并导出 MuJoCo 复核用例。 |
| `test/gen_mujoco_contact_accel_cases.py` | S7 真值：用 `mj_integratePos` + 中心差分生成接触点加速度 `Jc·q̈ + Jdot·q̇` 的 MuJoCo oracle。 |
| `test/check_wbc_hover_mujoco.py` | S7 独立复核：把 WBC 解（`q/qvel/qdd/f/τ/stance`）回灌 MuJoCo（`ctrl=τ` + `mj_applyFT` 施加逐脚接触力），验证 `qacc≈qdd`、支撑无滑、摆动 `f=0`。 |
| `test/lite3_controller_test.cpp` | S8 离线自检（8 段，不需要 MuJoCo）：起立退让（`τ_ff≡0`）、使能滞回、安全退化、MPC 过期降级、命令平滑、MPC 抖动不阻塞 1 kHz 路径、`τ_ff` 反解、连续失败兜底。 |
| `test/run_s8_mpc_wbc.sh` | S8 验收：离线自检（默认）+ 可选整机 harness（`stand` / `walk`）。 |
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
mpc_wbc/test/run_srbd_mpc_test.sh
mpc_wbc/test/run_wbc_test.sh
mpc_wbc/test/run_s8_mpc_wbc.sh
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

### 运行 S8 整机 MPC+WBC（MuJoCo，无头）

```bash
mpc_wbc/test/run_s8_mpc_wbc.sh            # 只跑离线确定性自检
mpc_wbc/test/run_s8_mpc_wbc.sh stand      # + 站立保持 20 s（整机闭环）
mpc_wbc/test/run_s8_mpc_wbc.sh walk       # + 全序列（前后/侧移/转向）
```

也可以直接起仿真再跑 harness（可视化加 `MUJOCO_VIEWER=1`）：

```bash
MUJOCO_VIEWER=0 SIM_DURATION=26 python3 interface/robot/simulation/mujoco_simulation.py &
S8_PHASES=stand ./build/mpc_wbc/mpc_wbc_mpc_wbc_sim build/s8_phases.csv build/s8_diag.csv
python3 mpc_wbc/test/analyze_s4_log.py build/s8_sim.csv build/s8_phases.csv
```

`mpc_wbc_sim` 的环境变量：`S8_PHASES=stand|walk`、`S8_PD_SCALE`、`S8_W_SWING`、`S8_MPC_PERIOD`、`S8_HYST`、`S8_CMD_SCALE`；诊断 CSV 列含义见 `apps/mpc_wbc_sim.cpp` 的表头（`step_ms/mpc_ms/wbc_iters/eom_res/cone_viol/a_des/alpha_des/num_support/wrench_age` 等）。**harness 交棒是条件式**（关节误差 < 0.15 rad、tilt < 0.2 rad；超 1.5 s 宽限则报「起立未收敛」退出），避免在半蹲状态把控制权交给 MPC 而误判成"控制律不稳"。

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
| 整机质心 vs `subtree_com` | `1.1e-16` | `< 1e-9` |
| 整机合成惯量绕基座原点 vs `M[3:6,3:6]` | `8.3e-16` | `< 1e-9` |
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

### S6 SRBD 预测时域 MPC 自检结果（离线，不需要 MuJoCo）

模型：`x=[p_com, v_com, rpy, ω]`，`N=10, Δt=0.05 s`（时域 0.5 s）；决策是「每支撑脚×每节点×4 个锥方向」的 `λ≥0`，摆动脚不生成变量 → 受力结构性为 0；单脚法向力上限由 `Σλ ≤ f_z,max` 保证（每方向上界取 `f_z,max/4`）。输出**第 0 节点逐脚力**与绕质心的净力/净力矩（S7 的期望 wrench）。

| 检查 | 结果 |
| --- | --- |
| 静立悬停净力 = `m·g` | `117.1081 N`（误差 `0.0002%`，`mg=117.108 N`） |
| 静立净力矩 / 水平足力 | `1.2e-7 N·m` / `< 1e-4 N` |
| 静立预测高度漂移（0.5 s 时域） | `3.6e-6 m` |
| 速度响应（前/后/横移 `0.1~0.3 m/s`） | 方向正确、随命令单调；预测末端速度误差 `< 1e-3 m/s` |
| 刹车（`v=0.3 → 0`） | 净力 `−64.4 N`，预测末端 `vx = −2e-4 m/s` |
| 偏航（`ω_z=±0.6 rad/s`） | `M_z = ±4.71 N·m`，预测末端 `ω_z=0.60 rad/s` |
| 接触计划（trot，k=0 时 FL/HR 摆动） | 摆动脚力恒为 `0`；两支撑腿合计 `117.10 N` |
| 摩擦锥 / 法向力上限 | 全部满足（逐脚 `f_z≥0`、`|f_xy|≤μf_z`、`f_z≤200 N`） |
| 姿态通道（`roll/pitch=0.1 rad`、`ω_y=0.5`） | 恢复/阻尼力矩均反向；预测姿态被拉回 |
| QP 收敛与耗时（trot 计划随时域前移，100 帧） | 全部收敛（残差 `≤1e-13`），mean `0.53 ms` / max `0.71 ms` |
| 求解失败兜底 | 旧解未过期 → `kStale`（沿用上帧力）；过期或从未解出 → `kFailed` |
| 闭环（用同一 SRBD 模型当被控对象，200 步） | `vx → 0.3000 m/s`，高度误差 `0 m`，roll `0` |

### S7 Weighted-QP WBC 自检结果（离线 + MuJoCo 正动力学复核）

`mpc_wbc/test/run_wbc_test.sh`：自研 QP 单测 + WBC 离线自检 + MuJoCo 独立复核，全部 0 failures。

| 检查 | 结果 |
| --- | --- |
| QP 单测（解析解 / 与盒约束 QP 交叉 / 40 组随机 KKT / 退化 NNLS / 冗余等式 / 不可行识别） | 交叉最大差 `6.5e-10` |
| 接触偏置加速度 `Jdot·v` 对表 MuJoCo（100 随机工况） | max `1.6e-9` |
| 静立悬停（四脚支撑） | `Σfz = 117.1079 N = mg`（误差 `0.0000%`），EoM `1.4e-14`，接触 `3.9e-18`，`|τ|max=3.86` |
| 基座 6D 跟踪（±x/±y/偏航，四脚支撑无摆动腿） | 误差 `~5e-9`（**无竞争任务时精确**） |
| 摆动足任务（对角支撑 + `qvel≠0`） | 残差 `0.033 m/s²`；关闭任务时 `15.07`（降 455 倍） |
| 支撑承重（两脚对角支撑，静止） | `Σfz = (M q̈ + h)_z` 精确到 `1e-6 N` |
| 加权最优性证书（`BuildQp` + `SolveQpIpm` 取乘子查 KKT） | 驻点 `7.2e-8`、互补 `6.9e-8`、`min z=1.4e-12` |
| 权重敏感性 | `w_base↑ ⇒ 基座误差↓`、`w_swing↑ ⇒ 摆动残差↓` |
| 极限工况 `a_des=(6,4,0), α_des=(0,0,5)` | 锥违反 `8.7e-10`、`|τ|max=24.8 < 30` |
| MPC 串联 | 竖直力误差 `1.40%`；开力参考任务后逐脚偏差 `2.46 N`（关 `2.93 N`） |
| 耗时（200 帧 trot） | mean `0.39 ms` / max `0.58 ms`，0 失败帧 |
| 失败兜底 / PD 反解 | `kStale`/`kFailed` 正确；反解复现误差 `4.4e-16 N·m` |
| MuJoCo 独立复核 | `max|qacc − qdd| = 1.8e-12`，支撑接触点加速度 `5.6e-11`，摆动 `f = 8e-17` |

**软任务的边界（重要，别再踩）**：软任务只承诺「加权最优」，不承诺「都精确」。`w_posture`（站姿）/`w_f_ref`（力参考）/`w_tau`（力矩正则）三项默认 **0**，因为它们与主任务抢同一组关节或同一组接触力——实测 `w_posture=1` 会让摆动任务残差与基座误差**同时**从 `0.033/0.193` 恶化到 `3.92/0.745`；`w_tau=1e-3` 会把无竞争时的基座任务从 `q̈_x=0.5000` 拉到 `0.4999869`。参考工程 `09_weighted_wbc` 的任务集只有「基座 6D + 摆动足」，力参考任务在 docs/03 §11.3 里被整条删除。`Σfz = m·g` 也只在**没有摆动腿**时严格成立（`qvel≠0` 有科氏偏置、腿部加速度经 `M` 非对角块耦合进基座 z 行），一般情形用 `Σfz = (M q̈ + h)_z`。

### S8 控制器编排自检结果（离线确定性 + MuJoCo 整机 harness）

`mpc_wbc/test/run_s8_mpc_wbc.sh`（离线 8 段，0 failures）：起立退让、使能滞回、安全退化、MPC 过期降级、命令平滑、1 kHz 路径不阻塞、`τ_ff` 反解、连续失败兜底。同一套控制器再跑 MuJoCo 闭环（`run_s8_mpc_wbc.sh stand|walk`）：

| 检查 | 结果 |
| --- | --- |
| 离线自检 8 段 | 8/8 PASS，0 failures |
| 起立退让 | `τ_ff ≡ 0`（同时 `|PD|max = 30 N·m`，证明位置环确实在出力） |
| 使能滞回 | 门关闭后仍保持 WBC 40 帧，之后退回位置环（`τ_ff` 归零） |
| 命令平滑 | 阶跃 0.2 m/s，0.2 s 后 0.1103（理论 0.1101），稳态 0.2000 |
| MPC 抖动不阻塞 | MPC 线程塞 20 ms 睡眠，1000 帧内 1 kHz 路径 p50 `0.533` / max `3.241 ms` |
| `τ_ff` 反解 | `|τ_applied − τ_wbc|max = 2.5e-6 N·m`（float 量化内） |
| 站立 20 s ×4（MuJoCo 闭环） | 全 PASS，max tilt `0.6/0.7/0.6/0.6°`，min bz `0.340 m` |
| 站立 1 kHz 单帧耗时 | mean `0.501–0.509` / p99 `0.58–0.74` / max `4.6–15.3 ms`；超 1 ms 帧 `0.17–0.21%`（OS 抢占） |
| 站立 MPC 线程 | mean `2.67–2.76` / max `3.05 ms`；`wrench_age` mean `29.6` / max `57 ms`（阈值 150） |
| 站立硬约束残差 | EoM `≤2.8e-14`、接触 `≤3.4e-16`、锥违反 `0` |
| 站立力矩 / 承重 | `|τ|max 4.5 N·m`（限 30）、`Σfz` mean `117 N`（`mg = 117.1 N`）、WBC 激活 `99.8%` |
| 状态机路径（`LITE3_CONTROL_PATH=mpc_wbc`） | `idle → standup → mpc_wbc_control`；站立 6 s（bz 0.349、tilt≈0）；前进 1.4 m（pitch<2°）；超限自动 `joint_damping → idle` |
| 交棒质量 | 关节误差 `0.017–0.018 rad`、tilt `0.2°`；接管 0.2 s 后估计高度 `0.3402–0.3405 m`、置信度 `1.00`、支撑腿 4 |
| **trot 行走（4 次）** | **全部失败**：`walk_fwd` 前进 `0.49–0.77 m` 后在 1.8–2.6 s 发散，或 `stop1`（命令回零）0.54 s 后发散 → 移交 S9（见下） |

---

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
- `SrbdPreviewMpc` 的姿态通道用「当前姿态冻结的 ZYX 欧拉角速率矩阵」线性化，且旋转方程忽略 `ω×Iω` 的时变（预测用当前值近似）；tilt 大或角速度高时精度下降，本工程运行包线（`±10°`、`ω<2 rad/s`）内可用。
- SRBD-MPC 的足端作用点在整个时域内视为不动（标准 SRBD 假设）；摆动腿惯性反作用力、接触切换的几何变化由 S7 WBC 与接触计划承担。
- 权重（`q_p_xy=10, q_p_z=200, q_v=10, q_rp=50, q_yaw=20, q_w_rp=5`）是**初值**，需在 S9/S10 用 7 段 A/B 整定。
- **S7 内点法不支持热启动**：单帧 mean `0.39 ms`（目标 20–50 Hz 富余大），S10 若要 1 kHz 必须换主动集 + 工作集缓存；WBC 的行布局已按固定行数设计好。
- S7 的摆动足任务用**接触点**雅可比（世界系），摆动目标位置来自 `FootPositionBody`（球心），相差一个 `foot_radius` 定值偏移；S9 接入轨迹层时要统一参考点标注。
- S7 支撑无滑用完整式 `J q̈ = −Jdot q̇`（参考工程用简化的 `J q̈ = 0`）；`Jdot·v` 已对表 MuJoCo（`1.6e-9`）。
- S7 的 `w_base=1e3 / w_swing=1e2` 沿用参考工程整定值，本工程尚未闭环整定（S9/S10）。
- MuJoCo ≥3.0 的 `data.xfrc_applied` **不被 `mj_forward` 消费**（只在 `mj_step` 生效）；离线正动力学复核要用 `mj_applyFT` 累加进 `data.qfrc_applied`。球足踩平地的接触点 = 球心 + **世界系** `(0,0,−r_foot)`，不是机体系 `-z` 乘 `R_foot`。
- `λ` 正则化必须远小于跟踪权重：取 `1e-3` 时会为了压小 `λ` 而把承重拉偏 `0.25%`（预测高度漂移 4 mm），已改用 `1e-6`。
- **S8 的 `τ_ff` 语义**：低层执行律是 `τ_applied = kp(q_des−q) + kd(dq_des−dq) + τ_ff`，所以 WBC 生效时 `τ_ff = τ_wbc − PD`；**WBC 未使能时必须 `τ_ff = 0`**（否则等于把 PD 反向叠加一次，再被 `±30` 夹住 → 凭空多出 `−30 N·m`，起立会被顶翻）。
- **S8 的命令平滑顺序**：必须「先一阶低通（`cmd_tau=0.25 s`）后变化率限幅（`0.8 m/s²`、`2.0 rad/s²`）」；反序等于两个滞后串联，命令几乎不动。`dt ≤ 0` 的帧不推进平滑器。
- **S8 的高度命令软启动**：`com_height_slew=0.06 m/s`，从实测质心高度爬到 `stand_com_height`；直接给目标会在 engage 瞬间要 ~30 m/s² 竖直加速度（τ 顶到 28.7 N·m，偶发顶翻）。
- **S8 的 WBC 使能滞回**：`wbc_enable_hold_frames=40`（1 kHz 下 40 ms），门关闭（估计失效/支撑不足/姿态偏大）时保持，只有超 `tilt_limit=0.8 rad` 判故障才立即失效——一帧一帧开关会同时改 PD 增益（×2）与 `τ_ff`，比撑着跑更糟。
- **S8 未通过项（移交 S9 第一项）**：trot 行走 4/4 失败——`walk_fwd` 前进 `0.49–0.77 m` 后 1.8–2.6 s 发散（tilt 46–155°），或 `stop1`（命令回零）0.54 s 后发散；失稳期实测 MPC 迭代 `0 → 75/2100`、残差 `1e-14 → 6e-5`、`Σfz` 峰值 `224 N`（≈1.9 mg）、`|τ_ff|` 打到 30 N·m 限幅、估计 `num_support` 掉到 1。参考工程 docs/03 §12.4/§12.5 记录过同类未解决问题。
- **S8 的接管条件不对称**：harness 已改成条件交棒（关节误差 < 0.15 rad、tilt < 0.2 rad，超 1.5 s 宽限报错退出），但真机状态机路径仍是**按时间**交棒（`standup_state.hpp` 的 `2*stand_duration_`），S9 应统一。
- 1 kHz 路径 p99 `≤0.74 ms`，但仍有 `0.17–0.21%` 帧超 1 ms（max 4.6–15.3 ms，OS 抢占，非算法）；真机需 RT 优先级或提频。

