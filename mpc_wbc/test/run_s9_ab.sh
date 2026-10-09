#!/usr/bin/env bash
# S9 A/B 试验台：**同一场景文件、同一仿真、同一分析口径**跑两条控制路径，量化力控收益。
#
#   baseline : S4 位置环（build/mpc_wbc/mpc_wbc_position_loop_sim）
#   mpc      : S8 MPC+WBC（build/mpc_wbc/mpc_wbc_mpc_wbc_sim）
#
# 用法：
#   mpc_wbc/test/run_s9_ab.sh                        # 全部场景 × 两条路径
#   mpc_wbc/test/run_s9_ab.sh --only axis_fwd        # 单个场景
#   mpc_wbc/test/run_s9_ab.sh --mpc                  # 只跑 MPC 侧
#   mpc_wbc/test/run_s9_ab.sh --init-yaw 1.5707963   # 起始航向（配合 --only axis_fwd 即 heading 用例）
#
# 产物：build/s9/<case>_<who>_{sim.csv,phases.csv,diag.csv,app.txt,json}
#       build/s9/report.md（由 s9_ab_report.py 生成）
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/build/s9"
SIM_SCRIPT="$ROOT/interface/robot/simulation/mujoco_simulation.py"
CASE_DIR="$ROOT/mpc_wbc/test/scenarios"
BASELINE_BIN="$ROOT/build/mpc_wbc/mpc_wbc_position_loop_sim"
MPC_BIN="$ROOT/build/mpc_wbc/mpc_wbc_mpc_wbc_sim"
ANALYZE="$ROOT/mpc_wbc/test/analyze_s9_run.py"
REPORT="$ROOT/mpc_wbc/test/s9_ab_report.py"

ONLY=""
WHO="both"
INIT_YAW=0
RUN_REPORT=1
CASES=(stand axis_fwd axis_back axis_left axis_right turn_left turn_right axis_fwd_long \
       mix_diag mix_yaw_fwd stop_reverse all_axes)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --only) ONLY="$2"; shift 2 ;;
    --baseline) WHO="baseline"; shift ;;
    --mpc) WHO="mpc"; shift ;;
    --init-yaw) INIT_YAW="$2"; shift 2 ;;
    --no-report) RUN_REPORT=0; shift ;;
    --cases) CASES=($2); shift 2 ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
done

mkdir -p "$OUT"
[[ -n "$ONLY" ]] && CASES=("$ONLY")

# 起始航向（非零时进文件名/报告名，避免覆盖 0° 的同一场景）
YAW_TAG=""
if [[ "$INIT_YAW" != "0" ]]; then
  YAW_TAG=$(python3 -c "import math;print('_yaw%d' % round(math.degrees($INIT_YAW)))")
fi

# 场景时长（运动段之和）
scenario_seconds() { awk -F, 'NR>1 && $1!="phase" {s+=$2} END{print s+0}' "$1"; }

# 起立段时长：S4 = settle+stand+hold0.5（=2.8s），S8 = settle+fold+raise+hold0.5（=4.3s）
PREFIX_SEC_BASELINE=2.8
PREFIX_SEC_MPC=4.3
PREFIX_ROWS_BASELINE=3
PREFIX_ROWS_MPC=4

wait_port_free() {
  for _ in $(seq 1 60); do
    ss -lun 2>/dev/null | grep -q ':30010 ' || return 0
    sleep 0.5
  done
  return 0
}

# run_one <case> <who> <bin> <env...>
run_one() {
  local case_name="$1" who="$2" bin="$3"; shift 3
  local scen="$CASE_DIR/$case_name.csv"
  local prefix=2.8 rows=3
  if [[ "$who" == "baseline" ]]; then prefix=$PREFIX_SEC_BASELINE; rows=$PREFIX_ROWS_BASELINE
  else prefix=$PREFIX_SEC_MPC; rows=$PREFIX_ROWS_MPC; fi
  local sim_dur
  sim_dur=$(python3 -c "print(f'{$(scenario_seconds "$scen") + $prefix + 3.0:.1f}')")

  local tag="${case_name}${YAW_TAG}_${who}"
  local case_label="${case_name}${YAW_TAG}"
  local sim_log="$OUT/${tag}_sim.csv" phases="$OUT/${tag}_phases.csv"
  local diag="$OUT/${tag}_diag.csv" applog="$OUT/${tag}_app.txt" js="$OUT/${tag}.json"
  rm -f "$sim_log" "$phases" "$diag" "$applog" "$js"

  wait_port_free
  MUJOCO_VIEWER=0 SIM_DEBUG=0 SIM_DURATION="$sim_dur" SIM_INIT_YAW="$INIT_YAW" \
    SIM_LOG="$sim_log" python3 "$SIM_SCRIPT" > "$OUT/${tag}_sim.txt" 2>&1 &
  local sim_pid=$!
  sleep 2

  echo "[s9] $tag: sim ${sim_dur}s, scenario $case_name"
  if [[ "$who" == "baseline" ]]; then
    env S9_SCENARIO="$scen" "$bin" "$phases" > "$applog" 2>&1
  else
    env S8_PHASES="$scen" "$bin" "$phases" "$diag" > "$applog" 2>&1
  fi
  local app_status=$?

  kill "$sim_pid" 2>/dev/null
  wait "$sim_pid" 2>/dev/null
  sleep 0.3

  local diag_arg=()
  [[ "$who" == "mpc" && -s "$diag" ]] && diag_arg=(--diag "$diag")
  python3 "$ANALYZE" "$sim_log" "$phases" --prefix-rows "$rows" \
      --label "$case_label/$who" "${diag_arg[@]}" --json "$js" \
      > "$OUT/${tag}_analyze.txt" 2>&1
  local analyze_status=$?
  tail -3 "$OUT/${tag}_analyze.txt" | sed 's/^/    /'
  echo "    app_status=$app_status analyze_status=$analyze_status"
}

for case_name in "${CASES[@]}"; do
  if ! [[ -f "$CASE_DIR/$case_name.csv" ]]; then
    echo "[s9] skip: no scenario $case_name"
    continue
  fi
  [[ "$WHO" == "both" || "$WHO" == "baseline" ]] && run_one "$case_name" baseline "$BASELINE_BIN"
  [[ "$WHO" == "both" || "$WHO" == "mpc" ]] && run_one "$case_name" mpc "$MPC_BIN"
done

if [[ $RUN_REPORT -eq 1 ]]; then
  python3 "$REPORT" --dir "$OUT" --out "$OUT/report.md" || echo "[s9] report failed"
fi
