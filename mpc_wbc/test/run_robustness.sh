#!/usr/bin/env bash
# 开环基线鲁棒性测试套件（全部无头运行，约 10 分钟）。
#
# 覆盖：
#   1. 侧向推力扰动（30 N / −30 N / 45 N，各 0.15 s）
#   2. 2° 坡面（上坡 0.15、上坡 0.30、下坡 0.30）
#   3. 速度阶跃 0.20 ↔ 0.50 m/s（每 3 s 切换，共 10 段）
#   4. 5 分钟连续运行漂移（±0.30 m/s 各 30 s 交替 5 个来回）
#
# 用法：bash mpc_wbc/test/run_robustness.sh [输出根目录]
# 结果：<out>/push_y{30,-30,45}/、<out>/slope2/、<out>/step/、<out>/drift/
set -u

ROOT="${1:-build/robust}"
PY=${PY:-$(command -v python3)}
SWEEP=mpc_wbc/test/sweep_gait_params.py
SC=mpc_wbc/test/scenarios

run_push() {
  "$PY" "$SWEEP" --segment mid --scenario "$SC/robust_push.csv" \
    --configs 0.750:0.700:0.070 --duration 22 --settle 2.0 \
    --outdir "$ROOT/push_y$1" \
    --sim-env "SIM_PUSH_FY=$1" --sim-env SIM_PUSH_T0=8 --sim-env SIM_PUSH_DUR=0.15 \
    2>&1 | tail -3
}

for fy in 30 -30 45; do run_push "$fy"; done

"$PY" "$SWEEP" --segment mid --scenario "$SC/robust_slope.csv" \
  --configs 0.750:0.700:0.070 --duration 50 --settle 3.0 --seg-log \
  --outdir "$ROOT/slope2" --sim-env SIM_SLOPE_DEG=2 2>&1 | tail -3

"$PY" "$SWEEP" --segment high --scenario "$SC/robust_step.csv" \
  --configs 0.440:0.625:0.070 --duration 40 --settle 1.0 --seg-log \
  --outdir "$ROOT/step" 2>&1 | tail -3

"$PY" "$SWEEP" --segment mid --scenario "$SC/robust_drift.csv" \
  --configs 0.750:0.700:0.070 --duration 308 --settle 3.0 \
  --outdir "$ROOT/drift" 2>&1 | tail -3

echo
echo "完成。推力扰动用 mpc_wbc/test/analyze_push.py 分析峰倾角与恢复时间。"
