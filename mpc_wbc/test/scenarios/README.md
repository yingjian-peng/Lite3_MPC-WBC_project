# 当前回归场景

仅保留 22 个用于当前基线和闭环重建的场景；早期超包络扫速、中间参数版本和重复场景已删除。

| 文件 | 用途 |
| --- | --- |
| `axis_fwd/back/left/right.csv`、`turn_left/right.csv` | 单方向快速检查，3 s 短段仅供诊断 |
| `stand.csv`、`stop_reverse.csv` | 静立及停步换向 |
| `verify_single_low/mid/high.csv` | 三档六方向，供调参及 `run_baseline.py --suite single10s` 使用 |
| `verify_mix2.csv`、`verify_mix3.csv` | 当前包络的双轴、三轴基线标定场景 |
| `mix2_env_full.csv`、`mix3_env_full.csv` | 当前包络的双轴、三轴 10 s 运动段 |
| `mix2_axes_10s.csv` | 中速双轴组合 |
| `env_single.csv` | 单轴包络与超限裁剪检查 |
| `segments_3x3_20s.csv` | 三轴跨速度段连续运行 |
| `robust_push/slope/step/drift.csv` | 推力、坡面、速度阶跃与连续运行 |

原地航向变化复验使用 `run_baseline.py --init-yaw`，无需复制方向相同的 CSV。其他组合场景可由 `gen_limit_mix.py` 按需生成。

基础数值检查统一运行 CTest；整机回归统一使用 `run_baseline.py`。旧独立编译脚本及旧分析器已删除，当前稳态分析器为 `analyze_s4_steady.py`。
