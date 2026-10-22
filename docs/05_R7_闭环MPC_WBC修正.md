
---

## 2026-10-22 R7：对照 `legged_control` 修正 MPC+WBC，闭环包线从 0.05 m/s 拓到 ±0.45（前程到 0.60）

本轮先下载了参考工程 `legged_control`（legged_wbc / legged_interface 的 OCS2 SLQ 实现）
做逐项对照，再据此修正我们的实现。`legged_control/` 只作参考，未做任何修改，且已在
`.gitignore` 中排除（原来的 `.legged_control` 带前导点，与实际目录名不匹配，等于没忽略）。

### 对照结论

| 项 | legged_control (go1) | 我们（本轮修复前） | 判定 |
|---|---|---|---|
| 摆动任务 | `J q̈ = kp·e + kd·ė − J̇q̇` | 漏了 `− J̇q̇` | 真 bug |
| `w_swing / w_base` | 100 / 1 | 100 / 1000 | 真 bug（反了） |
| 摆动增益 kp/kd | 350 / 37 | 200 / 10 | 偏软，已对齐 |
| EoM 硬等式 + τ 为决策变量 | 是 | 是 | 本来就对 |
| 低层 PD 增益 | 整档，不打折 | WBC 工作时 ×0.5 | 真 bug（见下） |
| 基座任务 | 质心动量速率（`Ab⁻¹`） | 恒等 `q̈_base = a_des` | 简化，非错，保留 |
| MPC | OCS2 全身质心 SLQ，dt=0.015 | 线性 SRBD 箱式 QP，N=10，dt=0.05 | 架构差异，保留 |
| 摩擦系数 | 0.3 | 0.6 | 待评估 |
| MPC `q_p_xy` / `q_p_z` | 1000 / 1500 | 10 / 200 | 已加 env；扫大反而更差，暂不改 |

### 五个关键修复

1. **摆动任务补 `− J̇q̇`**（`04_weighted_wbc.hpp`）。足端加速度是 `a = J q̈ + J̇q̇`，
   漏掉这一项任务目标会偏一个与向心/科氏同量级的量，表现正是此前实测的
   「命令净空 0.10 m、实际只有 0.045 m，FL/HR 有 85–90% 的摆动时间在拖地」。
2. **权重比改为 `w_base=1.0 / w_swing=100`**，摆动增益对齐 `kp=350 / kd=37`。
3. **`swing_accel_error` 诊断口径对齐 QP**。原来诊断没减 `J̇q̇` 而 QP 减了，
   于是扫 `w_swing` 5 个数量级残差只降 1.4×，看起来像「权重失效」。
   修正后同一扫描降到 2.9e-06（约 95000×），证明权重一直有效。
4. **估计器跳变门限从「硬拒绝」改为「限幅跟随」**（`01_state_estimator.hpp`）。
   这是真正的拦路虎：原来超限时保留冻结的旧估计，一旦真值跑出
   「冻结值 ± max_accel·dt」，之后每一帧的新候选都必然超限 → **永久自锁**。
   实测 0.15 m/s 下 `v_est_x` 冻在 0.091 而真值已到 0.86 m/s，MPC 因此持续给
   前向加速度（正反馈），最后机身被足端拖下 6 cm、支撑腿数掉到 0、WBC 报 kFailed。
   改成限幅后伪尖峰只偏移一帧的量，真实加减速能贴物理上限跟上。
5. **`kp_scale_wbc` / `kd_scale_wbc` 0.5 → 1.0**。低层 PD 是内环、WBC 只通过 `τ_ff`
   把总力矩拉到 `τ_wbc`，两者职责正交，把内环打折只让关节跟踪变差
   （0.45 m/s 下 0.1–0.3 rad），`τ_ff = τ_wbc − pd` 才常年顶到 ±30 N·m。
   0.45 m/s 扫描：scale=0.25→12.9 s 倒；0.35→9.0 s 倒；0.5→12.9 s 倒；**1.0→全程通过**。

### 两个被证伪的猜想（已留注释，勿重试）

- **去掉 `τ_ff` 的 ±tau_max 夹取**：理论上多余（执行器只对总力矩限幅，`τ_wbc` 已被
  约束在 ±30 内），实测**更差**——τ_ff 峰值从 30 涨到 60 N·m，walk 段 6.5 s 就倾到
  13.6°、8.0 s 失稳。原因是 `pd` 本身有 5–16 N·m，放开夹取等于把 PD 失配整包灌进前馈。
  → `τ_ff` 顶限是「关节跟踪不足」的**症状**，不是独立 bug。
- **把步态 `foot_vel` 经腿雅可比反解成 `dq_des` 喂给 PD**：概念上更正确，但 A/B 实测
  无可测收益（互有胜负），且把 τ_ff 饱和率整体抬高（步态给的是**平均速度**，与真实
  关节速度不逐帧匹配，反而放大 kd 项）。代码保留并由 `swing_joint_vel_ff`（默认
  **false**，env `S8_DQ_FF=1` 可开）控制，供后续换更准的参考轨迹时复验。

### 新增工具

- `S8_VX_SCALE`：只放大前向速度。原来的 `S8_CMD_SCALE` 会连 `vy`/`ωz` 一起放大，
  `cs=3.0` 时 `ωz` 变成 1.2 rad/s（超 `docs/00` 的 0.50 上限），测出来的失稳其实是
  出包线造成的假象。
- MPC 权重 env：`S8_Q_PXY` / `S8_Q_PZ` / `S8_Q_VXY` / `S8_Q_VZ` / `S8_Q_RP` /
  `S8_Q_YAW` / `S8_FZ_MAX` / `S8_R_LAMBDA`。
- 场景 `mpc_wbc/test/scenarios/r7_fwd_sweep.csv`（0.15→0.60 带停）、`r7_back_sweep.csv`。
- `run_s8_case.py` / `sweep_gait_params.py` 的 `--python` 改为 `sys.executable`，
  `run_robustness.sh` 改读 `$PY`。原先把 `/tmp/lite3_venv/bin/python` 写死，
  `/tmp` 被清理后整条工具链失效（ctest 里 9 个用例直接 Not Run）。
  当前环境：`~/.venvs/lite3_mpc_wbc`（mujoco 3.16 + numpy + colorama + scipy）；
  CMake 侧需 `-DPYTHON3_EXECUTABLE=$HOME/.venvs/lite3_mpc_wbc/bin/python`
  （`find_program` 会把路径缓存在 `PYTHON3_EXECUTABLE`）。

### 结果

`ctest` 13/13 全绿。`--phases walk`（0.15 m/s 前进/后退/左移/转向 + 4 次制动）
在 0.15 / 0.30 / 0.45 各跑 2 次全部 rc=0，可重复。

**前进阶梯**（`r7_fwd_sweep.csv`，`docs/00` 包线上限 0.60）：全程 rc=0

| cmd vx | 倾角 max | 高度 p05 | τ_wbc p95 | Σfz p50 | vx 真值 |
|---|---|---|---|---|---|
| 0.15 | 1.86° | 0.340 | 10.1 | 117.5 | 0.179 |
| 0.30 | 3.40° | 0.339 | 10.8 | 117.0 | 0.341 |
| 0.45 | 3.10° | 0.346 | 12.1 | 118.4 | 0.488 |
| 0.60 | 5.98° | 0.344 | 13.2 | 119.9 | 0.652 |

**后退阶梯**（`r7_back_sweep.csv`）：到 **−0.45 通过，−0.60 失稳**。

### 已知未解（交接给下一轮）

1. **后退 −0.60 m/s 失稳**。失效点在 `back` 段内而不是 stop 段。试过
   `NOM_X = 0 / −0.020 / −0.0266` 均不稳定（−0.0266 有一次 rc=0 属偶然），
   所以不是足端名义偏置问题，机理未定位。当前闭环包线：**±0.45 稳健、0.60 仅前进**。
2. **`q_p_xy = 10` 比参考工程（1000）小两个量级**，但 0.60 m/s 下扫
   `q_p_xy=200/1000` 与 `q_v_xy=50`、`q_v_z=100`、`q_p_z=50`、`fz_max=150`
   **全部更差**，暂不改。
3. 速度跟踪有约 10–20% 超调（0.15→0.179、0.60→0.652），尚未加 `cmd_scale` 补偿层。

### 复现

    cmake --build build -j4 && ctest --test-dir build --output-on-failure
    ~/.venvs/lite3_mpc_wbc/bin/python mpc_wbc/test/run_s8_case.py \
        --outdir build/r7/walk --phases walk
    ~/.venvs/lite3_mpc_wbc/bin/python mpc_wbc/test/run_s8_case.py \
        --outdir build/r7/fwd --phases mpc_wbc/test/scenarios/r7_fwd_sweep.csv
