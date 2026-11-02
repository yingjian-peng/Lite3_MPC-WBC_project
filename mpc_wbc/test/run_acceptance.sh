#!/usr/bin/env bash
# 闭环 MPC+WBC 的完整验收跑批（单轴 / 组合 / 三轴满量程）。
#
# 用法：
#   mpc_wbc/test/run_acceptance.sh                # 全跑（约 12 分钟，全部开 viewer）
#   mpc_wbc/test/run_acceptance.sh vx vy          # 只跑指定项
#   OUTROOT=build/acc_20261101 mpc_wbc/test/run_acceptance.sh
#   MUJOCO_VIEWER=0 mpc_wbc/test/run_acceptance.sh all   # 无头快跑
#
# 每一项都：起 MuJoCo（默认 viewer）→ 跑场景 csv → analyze_axis_sweep.py 汇总。
# 判定口径见 docs/00 §4/§5：线速度误差 ≤0.05 m/s、偏航误差 ≤0.05 rad/s、倾角 ≤10°
# （>25° 或高度 <0.15 m 记 FELL）。
#
# 注意：脚本会先杀掉残留的 mujoco_simulation.py / mpc_wbc_mpc_wbc_sim，否则新仿真
# 抢不到 UDP 端口会报 "simulation failed to start"。
set -u
cd "$(dirname "$0")/../.." || exit 1

PY=${PY:-$HOME/.venvs/lite3_mpc_wbc/bin/python}
OUTROOT=${OUTROOT:-build/acceptance}
VIEWER=${MUJOCO_VIEWER:-1}
SC=mpc_wbc/test/scenarios

declare -A PHASES=(
  [vx]="spec_vx.csv"
  [vy]="spec_vy.csv"
  [wz]="spec_wz.csv"
  [wz_hi]="wz_pm100.csv"
  [combo_fwd]="combo_fwd.csv"
  [combo_yw]="combo_yw.csv"
  [combo_2ax]="combo2.csv"
  [combo_3ax]="combo_3ax.csv"
  [max3]="max3.csv"
  [bnd]="combo_bnd.csv"
)
declare -A DUR=(
  [vx]=80 [vy]=80 [wz]=80 [wz_hi]=40
  [combo_fwd]=70 [combo_yw]=70 [combo_2ax]=60 [combo_3ax]=60 [max3]=60 [bnd]=60
)
declare -A ENV=(
  [vx]=""          # 速度偏置补偿已固化在默认 cmd_vel_trim=0.04 里
  [vy]=""
  [wz]="" [wz_hi]=""
  [combo_fwd]="--env S8_COMBO_LOSS=0"     # 关掉竞争项（默认已关，这里显式写）
  [combo_yw]="--env S8_COMBO_LOSS=0"
  [combo_2ax]="" [combo_3ax]="" [max3]="" [bnd]=""
)

if [ $# -eq 0 ] || [ "${1:-}" = "all" ]; then
  ITEMS=(vx vy wz wz_hi combo_fwd combo_yw combo_2ax combo_3ax max3 bnd)
else
  ITEMS=("$@")
fi

mkdir -p "$OUTROOT"
FAIL=0
for it in "${ITEMS[@]}"; do
  csv=${PHASES[$it]:-}
  if [ -z "$csv" ]; then
    echo "[acceptance] 未知项 '$it'，可选：${!PHASES[*]}" >&2
    FAIL=1; continue
  fi
  echo "=============================================================="
  echo "[acceptance] $it  <- $csv  (viewer=$VIEWER)"
  pkill -9 -f "mujoco_simulation.py|mpc_wbc_mpc_wbc_sim" 2>/dev/null
  sleep 2
  # shellcheck disable=SC2086
  $PY mpc_wbc/test/run_s8_case.py \
      --outdir "$OUTROOT/$it" \
      --phases "$SC/$csv" \
      --duration "${DUR[$it]}" \
      --sim-env "MUJOCO_VIEWER=$VIEWER" \
      --sim-log-every 10 ${ENV[$it]} 2>&1 | tail -2
  $PY mpc_wbc/test/analyze_axis_sweep.py "$OUTROOT/$it"
  echo
done

pkill -9 -f "mujoco_simulation.py|mpc_wbc_mpc_wbc_sim" 2>/dev/null
echo "=============================================================="
echo "[acceptance] 全部完成，结果在 $OUTROOT/（每项一个目录，含 sim.csv/diag.csv）"
echo "[acceptance] 组合工况的平移损失请再用 fit_combo_gain.py 复核（逐帧口径）"
exit $FAIL
