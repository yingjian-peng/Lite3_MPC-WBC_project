# MPC/WBC 目录与当前重建入口

本目录保留 Lite3 的运动学、动力学、状态估计、步态、SRBD-MPC、Weighted-QP WBC 及测试工具。现阶段先调通“足端轨迹 → IK → 关节 PD”的开环位置控制，前馈力矩为零；已有优化器暂不作为行走效果已经合格的证据。当前阶段、门槛与实验记录见 [重建计划](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)，旧实验见 [归档](../docs/archive/04_旧版实现与试验记录.md)。

## 代码入口

| 文件 | 用途 |
| --- | --- |
| 00_lite3_model.hpp | 运动学、雅可比、全身动力学与质心惯量 |
| 01_state_estimator.hpp | 基座速度、高度、估计有效性 |
| 02_gait_trajectory.hpp | trot 相位、足端位置/速度目标、未来支撑表 |
| 03_srbd_mpc.hpp、box_qp.hpp | SRBD 预测控制与接触力 QP |
| 04_weighted_wbc.hpp、qp_ipm.hpp | 全身控制 QP 与求解器 |
| 05_mpc_wbc_controller.hpp | 估计、步态、MPC 线程和 WBC 的已有编排 |
| apps/position_loop_sim.cpp | 当前开环调试主程序，无 MPC/WBC 前馈 |
| apps/mpc_wbc_sim.cpp | 已有优化器整机 harness，留待逐层复验 |
| test/scenarios/、test/analyze_s9_run.py | 场景与日志分析；旧 PASS 门槛需重新审定 |

## 构建与离线检查

在工程根目录执行：

    cmake -S . -B build -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_MPC_WBC_SIM=ON -DBUILD_MPC_WBC_TESTS=ON
    cmake --build build -j
    ctest --test-dir build --output-on-failure

CTest 检查模型对表、步态、估计、QP 和控制器离线行为。它不运行完整的实时 MuJoCo 行走场景，也不证明开环或 MPC+WBC 整机效果。

## 当前开环调试

无头采集可运行：

    mpc_wbc/test/run_s4_position_loop.sh 24

针对单个场景运行开环：

    mpc_wbc/test/run_s9_ab.sh --baseline --only axis_fwd

也可先运行 Python 仿真，再运行 build/mpc_wbc/mpc_wbc_position_loop_sim。仿真按墙钟约 1 ms 自主步进，C++ 接口独立约 1 ms 发送最近命令，与参考 /home/robot/pengyingjian/rl_deploy 的通信节拍一致。控制器按新状态时间戳更新目标；两端不锁步，重复运行应统计波动，不能期待逐位相同。

主要开环参数由 S4_KP、S4_KD、S4_SWING_HEIGHT、S4_PERIOD、S4_DUTY、S4_CMD_SCALE、S4_SWEEP_GAIN、S4_RAMP 等环境变量覆盖。S4_RAMP 是速度指令斜坡（命令单位/秒），默认 1.5；二轴混合换向时不能把命令直接阶跃，否则支撑脚目标会在一帧内跳变数厘米。S4_BLEND 是起步足端混合时间，默认 0.8 s；S4_NOM_X 是标称足位前后偏移，默认 **-0.0133 m**（按整机质心配平，见下）。

**仿真保真度（默认开启）**：官方 MJCF/URDF 未给关节阻尼与电枢惯量，足端接触也是 `condim=3`（无扭转摩擦）。`mujoco_simulation.py` 在默认参数下叠加关节电枢 0.01 kg·m²、阻尼 0.1 N·m·s/rad、足端 `condim=4` 与扭转摩擦 0.03，对应环境变量 `SIM_JOINT_ARM / SIM_JOINT_DAMP / SIM_FOOT_CONDIM / SIM_FOOT_TORSION`，四个都设 0 可回到官方原模型。地面使用官方 checker 纹理与单向光，平面范围放大到 40×40 m（官方 20×20 在 120 s 组合工况会走出边界）。

**速度包络（方案 B，默认开启）**：命令预处理链为「死区 → 扫掠增益 → 包络限幅 → 步幅限幅 → 斜坡」。包络先按单轴裁剪 `|vx|≤0.60`、`|vy|≤0.35`、`|wz|≤0.50`，再限制 `hypot(vx,vy)+0.18|wz| ≤ 0.67`，超限时三轴同比例缩比、方向不变。对应环境变量 `S4_ENV_VX`、`S4_ENV_VY`、`S4_ENV_WZ`、`S4_ENV_S_MAX`、`S4_ENV_WZ_WEIGHT`，`S4_ENV_LIMIT=0` 可整体关闭。注意 `phases.csv` 现在记录的是**限幅后的实际目标**，控制台会打印 `cmd -> applied`；想回到旧行为请显式关掉包络。S4_WZ_KP/S4_WZ_TAU/S4_WZ_CORR_MAX 是实验性的偏航角速度反馈（默认 0，纯开环；仅在平移指令时生效），隔离工况能压住 x+y− 的 yaw 振荡，但完整高速序列仍会出现平均偏航误差，暂不作为默认验收配置。当前默认值只代表代码中的旧配置，需要在新仿真节拍下重新测量，不能直接沿用旧报告里的“最优参数”。S4_TRACE 可记录每控制周期的目标命令。初始姿态与接触模式由 SIM_INIT_POSE、SIM_INIT_HEIGHT 和 SIM_LEG_COLLISION 设置；SIM_LOG、SIM_LOG_EVERY、SIM_DURATION 与 MUJOCO_VIEWER 控制日志、时长和显示。

二轴混合回归场景为 `test/scenarios/mix2_axes_10s.csv`：x+y、x+z、y+z 各 4 个正反组合，每段 10 s，段间停 1 s。高速边界场景为 `test/scenarios/mix2_axes_high_10s.csv`（0.50 / 0.30 / 0.40），1 s 停步；需要复现优化后的高速回归时用 `test/scenarios/mix2_axes_high_10s_stop2.csv`（2 s 停步），它能把最差 p2p 从 0.170/0.446 降到 0.034/0.112，但仍有 5 段偏航相关 WARN。当前开环可用高速二轴包线为 `test/scenarios/mix2_axes_high90_10s_stop2.csv`（0.45 / 0.27 / 0.36、2 s 停步）：12 段中 11 段 OK，仅左移+顺时针偏航误差 0.057 rad/s 略超 0.05。无头运行示例：

    S9_SCENARIO=mpc_wbc/test/scenarios/mix2_axes_10s.csv \
      S4_SEG_LOG=build/viewer_mix2_axes/segments.csv \
      ./build/mpc_wbc/mpc_wbc_position_loop_sim build/viewer_mix2_axes/phases.csv

`analyze_s4_steady.py` 在稳态速度/偏航误差或 p2p 超过 0.05 时标记 `WARN`，只有跌倒才判 `FELL`；不要只看最后的 `result=OK`。中速二轴 12 段已通过，高速二轴仍有前进+右移和左移+顺时针两段越限，见 ../docs/05_优化日志.md。

### 单轴 / 多轴速度边界场景

| 场景 | 内容 |
| --- | --- |
| `axis_x_limit.csv` / `axis_x_limit2.csv` | 单轴 vx 0.50–0.65 / 0.70–0.75 |
| `axis_y_limit.csv` / `axis_y_limit2.csv` | 单轴 vy 0.30–0.45 / 0.50–0.60 |
| `axis_z_limit.csv` / `axis_z_limit2.csv` | 单轴 wz 0.40–0.55 / 0.60–0.70 |
| `mix2_limit_cand.csv` | 二轴满速 0.65 / ±0.50 / ±0.60，12 段 |
| `mix3_limit_cand.csv` | 三轴满速 0.65 / ±0.50 / ±0.60，4 段 |
| `mix3_design.csv` | 三轴现行包络 0.50 / ±0.30 / ±0.40 |
| `mix3_boundary.csv` | 三轴 1.1× 现行包络 + 前向偏置包络 |
| `env_single.csv` | 方案 B 单轴区间 + 超限指令（验证限幅生效） |
| `mix2_env_full.csv` / `mix3_env_full.csv` | 方案 B 下二轴 / 三轴满指令（0.60/0.35/0.50） |
| `env_worst2.csv` / `env_worst_xy.csv` | 最差方向（前进+右移+逆时针）单点复现 |

> 包络默认开启后，上表中高于 0.60 / 0.35 / 0.50 或满足 `hypot+0.18|wz|>0.67` 的指令会被自动缩比。要复现早期的“超限扫速”（如 `axis_x_limit.csv` 的 0.65、`mix2_limit_cand.csv` 的 0.65/0.50），需加 `S4_ENV_LIMIT=0`。

任意包络可用生成器生成，不必手写 CSV：

    python3 mpc_wbc/test/gen_limit_mix.py --mode 3axis --mx 0.50 --my 0.30 --mz 0.40 \
      --out mpc_wbc/test/scenarios/mix3_design.csv
    python3 mpc_wbc/test/gen_limit_mix.py --mode 2axis --mx 0.65 --my 0.50 --mz 0.60 \
      --scale 0.9 --out mpc_wbc/test/scenarios/mix2_limit90.csv

实测结论（开环、高速段 T=0.45/d=0.625/H=0.07、每段 10 s + 2 s 停步）：

- 单轴上限：vx ≈ 0.70 m/s（0.75 开始 WARN）、vy ≥ 0.60 m/s（0.45 以上出现持续航向漂移 0.05–0.075 rad/s）、wz ≥ 0.70 rad/s。
- 三轴同时给满时：旧包线 0.50/0.30/0.40 已有 2/4 段 WARN（最差侧倾 13.1°）；上探到 0.55/0.33/0.44 为 16.6°，0.65/0.50/0.60 直接翻倒。
- 最差组合恒为 `+vx/−vy/+wz`，镜像 `+vx/+vy/−wz` 可通过，指向步态左右不对称；提高侧移包线前应先修这一点。
- 最终速度区间与组合约束见 ../docs/00_设计指标.md（单轴 0.60/0.35/0.50 + `hypot(vx,vy)+0.18|wz| ≤ 0.67`）。

方案 B + 保真度与配平后的实测（`build/final44_single/`、`build/final44_mix3/`、`build/final44_mix2_r2/`）：单轴高速 6/6 OK（最差偏航 0.038）；三轴 4/4 OK（最大侧倾 3.7°）；二轴 9/12 OK，最大侧倾 4.1°，残余为前进斜向 0.06–0.08 rad/s 偏航漂移与一段 0.051 m/s 跟踪误差。稳定性拐点在 $s_{max}\approx0.85$–$0.93$。

分段参数重扫（低速 1.20、中速 1.05，见下节）后的回归：`build/final6/{low,mid,high}` 三组单轴各 6/6 OK；`build/final6/mix3` 三轴 4/4 OK；`build/final8/mixed`（`segments_3x3_20s`，跨三个速度段 9 段）9/9 OK，最大侧倾 3.7°；`build/final6/mix2*`、`build/final7/mix2_wideblend` 二轴 7/12（三次一致，比冻结基线少 1 项，多出的一项落在 0.050–0.056 阈值边缘）。

先看站姿、关节跟踪和真实力矩，再看单腿净空、支撑滑移、三轴速度及停步。至少多次重复同一场景，保存原始 CSV、配置和代码版本。旧版锁步结果在 [历史记录](../docs/archive/04_旧版实现与试验记录.md) 中保留供追溯，但不能作为新时序的对照基线。

## 三段式开环调参

`02_gait_trajectory.hpp` 的 `SegmentedGaitParams` 按主运动方向把命令分到低/中/高速段，段内使用各自的周期 T、占空比 d 和抬腿高度 H，段边界做平滑插值；切换 T 时 `SetParamsAt()` 会保持相位连续。默认参数为：

| 段 | T (s) | d | H (m) | 步频 | 来源 |
| --- | --- | --- | --- | --- | --- |
| 低 | **1.20** | 0.72 | 0.07 | **0.83 Hz** | 速度区间更新后重扫（降步频） |
| 中 | **1.05** | 0.70 | 0.07 | **0.95 Hz** | 速度区间更新后重扫（降步频） |
| 高 | 0.44 | 0.625 | 0.07 | 2.27 Hz | 三轴偏航波动锁定，不能再降 |

2026-10-18 晚按“降步频、增步幅、降功耗”重扫：低/中速段步频分别降到 0.83 / 0.95 Hz（−38% / −40%），步幅增到 0.173 / 0.257 m（+60% / +67%），关节机械功率降到 9.1 / 16.1 W（−60% / −54%）；高速段受三轴组合的偏航波动限制，保持 0.44/0.625。

- 中/低速段再往上探仍稳定（中速 T=1.25 即 0.80 Hz、步幅 0.306 m 仍 OK），只是侧倾与滑移缓慢上升，取 1.05 为代价最小的一档；
- 高速段 T≥0.46 就会让 `verify_mix3` 掉项，且在相同步幅下降低占空比（0.46/0.63、0.50/0.58）同样失稳；
- 判定与功耗数据见 [优化日志](../docs/05_优化日志.md) 的“分段步态参数重扫”一节。

可用环境变量单段覆盖：

    S4_LOW_PERIOD=1.20 S4_LOW_DUTY=0.72 S4_LOW_SWING_HEIGHT=0.07
    S4_MID_PERIOD=1.05 S4_MID_DUTY=0.70 S4_MID_SWING_HEIGHT=0.07
    S4_HIGH_PERIOD=0.44 S4_HIGH_DUTY=0.625 S4_HIGH_SWING_HEIGHT=0.07

段边界可用 `S4_SEG_VX1/VX2`、`S4_SEG_VY1/VY2`、`S4_SEG_WZ1/WZ2` 覆盖，边界插值带宽用 `S4_SEG_BLEND_VX/VY/WZ`（默认 **0.10 / 0.075 / 0.10**，加宽是为了让减速穿段时周期不在一帧内跳变）；`S4_SEG_MAX_VX/VY/WZ` 只影响主运动方向归一化。`S4_SEG_LOG=<csv>` 记录选段与参数变化，便于核对（周期在带宽内连续变化时也会记行）。

长周期带来的两个瞬态问题已在代码里处理：

- **软停步**（`GaitTrajectory::Update`）：命令归零后不立即退出步态，等相位回到 θ<0.01（四足同时支撑、足端正好在标称位）再退出，避免目标位跳变；
- **独立减速斜坡** `S4_RAMP_DECEL`（默认 0.75 /s，加速仍 1.5 /s）：单轴高速场景停步倾角从 9.1° 降到 3.6°。

开环调参不追求起步/切换瞬态，只看稳态：`analyze_s4_steady.py` 会跳过每个运动相位开头的 `settle` 时间，用「一个步态周期的滑动平均」计算速度误差和低频波动。逐段搜索可运行：

    # 低/中/高各做一轮坐标搜索（每段约 7–8 次仿真）
    python3 mpc_wbc/test/tune_s4_segments.py

    # 只调高速段，跑两轮
    python3 mpc_wbc/test/tune_s4_segments.py --segments high --passes 2

    # 不启动仿真，只打印候选命令
    python3 mpc_wbc/test/tune_s4_segments.py --dry-run

    # 扩大候选范围（默认候选在 SEGMENTS 中；这里覆盖中速段）
    python3 mpc_wbc/test/tune_s4_segments.py --segments mid \
      --candidates "T=0.60,0.63,0.65;d=0.66,0.68,0.70;H=0.08,0.09,0.10"

产物在 `build/tune_s4/`：每个候选的原始 CSV、`results.csv` 和 `best.json`。搜索默认以稳态速度误差、周期平均速度峰峰值为主要代价，姿态、力矩饱和和足端滑移作为惩罚项；如果候选跌倒则直接淘汰。`--nominal` 可覆盖搜索起点。`--freq-weight`（默认 0.03）给步频加惩罚 `weight/T`，用于在跟踪误差接近时优先选择更慢、更省电的步态；设为 0 则只看稳态误差。

单段扫参推荐用 `sweep_gait_params.py`：它自己起停仿真、跑控制器再调分析器，一次给出整张对比表（`T/d/H/步频/步幅/结果/cost/侧倾/误差/滑移`），并写出 `sweep_summary.json`。

    # 高速段三档对比
    python3 mpc_wbc/test/sweep_gait_params.py --segment high \
      --scenario mpc_wbc/test/scenarios/verify_single_high.csv \
      --configs 0.44:0.625:0.07,0.52:0.68:0.07,0.62:0.68:0.07 \
      --duration 68 --outdir build/sweep_high

    # 跨速度段场景：按 segments.csv 逐相位取周期
    python3 mpc_wbc/test/sweep_gait_params.py --segment high \
      --scenario mpc_wbc/test/scenarios/segments_3x3_20s.csv \
      --configs 0.44:0.625:0.07 --duration 197 --settle 3.0 --seg-log \
      --outdir build/final8/mixed

    # 顺手加一个控制器环境变量做 A/B（可重复 --env）
    ... --env S4_NOM_X=-0.008 --env S4_RAMP_DECEL=1.5

功耗用 `energy_metrics.py` 从 `sim.csv` 的 `qd`/`atau` 估算：`P_abs` 为关节绝对机械功率（升降都算消耗），`P_cu` 为 $\sum\tau^2$ 铜损代理（真实电机损耗的主项，由承重决定，基本与步频无关）。

    python3 mpc_wbc/test/energy_metrics.py <sim.csv> <phases.csv> --label T0.44 --json o.json

## 后续优化器路径

优化器源码和 mpc_wbc_mpc_wbc_sim、state_machine/mpc_control_state.hpp 仍可构建。进入这些路径前，先完成 [R0–R3 开环验收](../docs/04_Lite3_MPC_WBC实现步骤与里程碑.md)；随后逐项打开高度/姿态反馈、状态估计、力前馈、MPC 和 WBC。每次只改变一个控制层，保留可回退的上一步配置，分别验证数学残差与真实足端/机身运动。
