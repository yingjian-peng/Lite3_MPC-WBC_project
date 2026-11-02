# mpc_wbc/test —— 离线调试与验收工具

**这个目录不参与构建，控制器也不依赖它。** `mpc_wbc/CMakeLists.txt` 与顶层
`CMakeLists.txt` 对这里零引用；删掉整个目录，`mpc_control` 仍能正常编译运行。

保留唯一理由：**让 `docs/00 §5/§6` 的验收数字可复现**。因此只留当前验收链用到的
文件，历史阶段（S4 开环 / S7 WBC 单测 / S9 A/B）的工具与一次性实验场景已在
2026-11-02 清理（如需追溯，见 git 历史与 `docs/archive/`）。

## 驱动脚本

| 文件 | 用途 |
| --- | --- |
| `run_s8_case.py` | **唯一**的场景驱动：起 MuJoCo → 跑 `mpc_wbc_mpc_wbc_sim` → 收数据（`sim.csv`/`diag.csv`/`phases.csv`）。下面三个脚本都调它 |
| `run_acceptance.sh` | 一键验收：单轴 / 组合 / 三轴满量程，默认开 viewer。`OUTROOT=` 改输出目录、`MUJOCO_VIEWER=0` 无头、`run_acceptance.sh vx vy` 只跑指定项 |
| `run_single_axis.sh` | 另一套单轴扫速（`axis_{vx,vy,wz}_sweep.csv`，±4 档 × 8 s + 2 s 停步） |
| `robustness_closed_loop.py` | 鲁棒性：侧推 / 坡面 / 速度阶跃 / 长时往返（`docs/00 §6`） |

## 分析工具

| 文件 | 用途 |
| --- | --- |
| `analyze_axis_sweep.py` | 逐段统计线性/偏航跟踪误差、纹波 p2p、倾角。**逐帧**用该帧偏航旋转世界系速度（旧版用整段平均偏航，长转向段会系统性偏小，曾伪造出「转向抢摩擦」的假结论） |
| `fit_combo_gain.py` | 逐帧口径重算组合工况的平移损失，反解竞争系数（若要重新启用 `combo_turn_gain` 才需要） |

## 场景 CSV（`scenarios/`）

相位表格式：`t, vx, vy, wz`（`#` 为注释）。

| 用途 | 文件 |
| --- | --- |
| 单轴验收（§5.1） | `spec_vx.csv`、`spec_vy.csv`、`spec_wz.csv` |
| 组合验收（§5.3） | `combo_fwd.csv`、`combo_yw.csv`、`combo2.csv`、`combo_3ax.csv`、`max3.csv`、`combo_bnd.csv` |
| 包线上探（§5.2） | `wz_up_{pos,neg}.csv`、`wz_hi_{pos,neg}.csv`、`wz_max_pos.csv`、`wz_lim.csv`、`wz_pm100.csv`、`wz_n{015,030}_long.csv`、`wz_{n,p}030_only.csv`、`wz_back_forth.csv`、`vy_up_{pos,neg}.csv`、`vy_hi_{pos,neg}.csv` |
| 单轴扫速 | `axis_{vx,vy,wz}_sweep.csv` |
| 鲁棒性（§6） | `robust_push.csv`、`robust_slope.csv`、`robust_step.csv`、`robust_drift.csv` |
| 其他 | `stand.csv`（起立）、`segments_3x3_20s.csv`（跨速度段连续运行） |

## 常用命令

```bash
PY=$HOME/.venvs/lite3_mpc_wbc/bin/python   # 必须用这个 venv（mujoco 3.16）

# 跑一个场景（长场景必须显式 --duration，默认只跑 80 s）
$PY mpc_wbc/test/run_s8_case.py --outdir build/r11/x \
    --phases mpc_wbc/test/scenarios/spec_vx.csv --duration 80 \
    --sim-env MUJOCO_VIEWER=1 --sim-log-every 10

# 分析结果
$PY mpc_wbc/test/analyze_axis_sweep.py build/r11/x
```

跑之前先清残留进程，否则新仿真抢不到 UDP 端口：

```bash
pgrep -af "mujoco_simulation.py|mpc_wbc_mpc_wbc_sim" | awk '{print $1}' | xargs -r kill -9
```
