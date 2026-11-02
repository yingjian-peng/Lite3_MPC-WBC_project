#!/usr/bin/env bash
# 单轴 vx / vy / wz 扫速回归（必须开 MuJoCo viewer，便于目视核对）。
#
# 用法：
#   mpc_wbc/test/run_single_axis.sh              # 三条全跑
#   mpc_wbc/test/run_single_axis.sh vx           # 只跑 vx
#   LITE3_PY=~/.venvs/lite3_mpc_wbc/bin/python mpc_wbc/test/run_single_axis.sh wz
#
# 产物：build/single_axis/final_{axis}/（sim.csv / phases.csv / diag.csv）
set -u
cd "$(dirname "$0")/../.." || exit 1

PY=${LITE3_PY:-$HOME/.venvs/lite3_mpc_wbc/bin/python}
OUTROOT=${OUTROOT:-build/single_axis/final}
AXES=${1:-"vx vy wz"}

run_one() {
  local axis="$1"; local scen="mpc_wbc/test/scenarios/axis_${axis}_sweep.csv"
  local out="${OUTROOT}_${axis}"
  echo "=== ${axis}: ${scen} -> ${out} (viewer ON) ==="
  "${PY}" mpc_wbc/test/run_s8_case.py \
      --outdir "${out}" --phases "${scen}" --duration 130 \
      --sim-env MUJOCO_VIEWER=1 --sim-log-every 10 \
      ${EXTRA_SIM_ENV:-} ${EXTRA_CTRL_ENV:-} 2>&1 | tail -4
  "${PY}" mpc_wbc/test/analyze_axis_sweep.py "${out}"
}

for a in ${AXES}; do
  run_one "${a}"
done
