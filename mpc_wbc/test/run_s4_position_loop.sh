#!/usr/bin/env bash
# S4 位置环跑步验收：启动无头 MuJoCo 仿真 + 位置环控制程序，跑完后分析。
# 用法： mpc_wbc/test/run_s4_position_loop.sh [sim_duration_s]
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$ROOT/build"
SIM_LOG="$BUILD_DIR/s4_sim_log.csv"
PHASES="$BUILD_DIR/s4_phases.csv"
SIM_OUT="$BUILD_DIR/s4_sim_stdout.txt"
APP="$BUILD_DIR/mpc_wbc/mpc_wbc_position_loop_sim"
SIM_DURATION="${1:-24}"

if [[ ! -x "$APP" ]]; then
  echo "[s4] building mpc_wbc_position_loop_sim ..."
  cmake -S "$ROOT" -B "$BUILD_DIR" -DBUILD_MPC_WBC_SIM=ON >/dev/null || exit 1
  cmake --build "$BUILD_DIR" --target mpc_wbc_position_loop_sim -j4 >/dev/null || exit 1
fi

rm -f "$SIM_LOG" "$PHASES" "$SIM_OUT"
cd "$ROOT"

echo "[s4] starting headless MuJoCo simulation (duration=${SIM_DURATION}s)..."
MUJOCO_VIEWER=0 SIM_DEBUG=0 SIM_DURATION="$SIM_DURATION" SIM_LOG="$SIM_LOG" \
  python3 interface/robot/simulation/mujoco_simulation.py > "$SIM_OUT" 2>&1 &
SIM_PID=$!

sleep 1.2
echo "[s4] running position-loop controller..."
"$APP" "$PHASES"
APP_STATUS=$?

wait "$SIM_PID" 2>/dev/null
SIM_STATUS=$?

echo "[s4] app_status=$APP_STATUS sim_status=$SIM_STATUS"
if [[ $APP_STATUS -ne 0 ]]; then
  echo "[s4] controller did not exit cleanly (fell or crashed)"
fi

echo "[s4] analyzing..."
python3 mpc_wbc/test/analyze_s4_log.py "$SIM_LOG" "$PHASES"
ANALYZE_STATUS=$?

if [[ $APP_STATUS -ne 0 ]]; then
  exit $APP_STATUS
fi
exit $ANALYZE_STATUS
