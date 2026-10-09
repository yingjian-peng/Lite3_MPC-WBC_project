#!/usr/bin/env bash
# S8 控制器编排验收：
#   [1] 离线确定性自检（不需要 MuJoCo，覆盖起立退让/滞回/安全退化/MPC 降级/命令平滑/不阻塞）
#   [2] 可选：无头 MuJoCo 整机 harness（真实 UDP 闭环，站立或全序列行走）
# 用法：
#   mpc_wbc/test/run_s8_mpc_wbc.sh              # 只跑离线自检
#   mpc_wbc/test/run_s8_mpc_wbc.sh stand        # 离线自检 + 站立保持 20 s harness
#   mpc_wbc/test/run_s8_mpc_wbc.sh walk         # 离线自检 + 全序列（前后/侧移/转向）harness
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$ROOT/build"
CTRL_BIN="$BUILD_DIR/lite3_controller_test"
APP="$BUILD_DIR/mpc_wbc/mpc_wbc_mpc_wbc_sim"
PHASE_SET="${1:-offline}"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[s8] [1/2] compiling C++ controller orchestration test..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_controller_test.cpp -o "$CTRL_BIN" -lpthread || exit 1

echo "[s8] [2/2] running offline self-check..."
"$CTRL_BIN"
CTRL_STATUS=$?
if [[ $CTRL_STATUS -ne 0 ]]; then
  echo "[s8] offline self-check FAILED (status=$CTRL_STATUS)"
  exit $CTRL_STATUS
fi

if [[ "$PHASE_SET" == "offline" ]]; then
  echo "[s8] done (offline only). Pass 'stand' or 'walk' to also run the MuJoCo harness."
  exit 0
fi

if [[ ! -x "$APP" ]]; then
  echo "[s8] building mpc_wbc_mpc_wbc_sim ..."
  cmake -S "$ROOT" -B "$BUILD_DIR" -DBUILD_MPC_WBC_SIM=ON -DBUILD_MPC_WBC_TESTS=ON >/dev/null || exit 1
  cmake --build "$BUILD_DIR" --target mpc_wbc_mpc_wbc_sim -j4 >/dev/null || exit 1
fi

# harness 的相位时长：stand = 0.8+1.5+1.5+20 = 23.8 s，walk = 20.8 s
case "$PHASE_SET" in
  stand) SIM_DURATION=26 ;;
  walk)  SIM_DURATION=24 ;;
  *) echo "[s8] unknown phase set: $PHASE_SET (expect offline|stand|walk)"; exit 2 ;;
esac

TAG="s8_${PHASE_SET}"
SIM_LOG="$BUILD_DIR/${TAG}_sim.csv"
PHASES="$BUILD_DIR/${TAG}_phases.csv"
DIAG="$BUILD_DIR/${TAG}_diag.csv"
SIM_OUT="$BUILD_DIR/${TAG}_sim_stdout.txt"
APP_OUT="$BUILD_DIR/${TAG}_app.txt"
rm -f "$SIM_LOG" "$PHASES" "$DIAG" "$SIM_OUT" "$APP_OUT"

SIM_PID=""
cleanup() { [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null; }
trap cleanup EXIT

echo "[s8] starting headless MuJoCo simulation (duration=${SIM_DURATION}s)..."
MUJOCO_VIEWER=0 SIM_DEBUG=0 SIM_DURATION="$SIM_DURATION" SIM_LOG="$SIM_LOG" \
  python3 interface/robot/simulation/mujoco_simulation.py > "$SIM_OUT" 2>&1 &
SIM_PID=$!

sleep 2
echo "[s8] running MPC+WBC controller (phases=$PHASE_SET)..."
S8_PHASES="$PHASE_SET" "$APP" "$PHASES" "$DIAG" > "$APP_OUT" 2>&1
APP_STATUS=$?

kill "$SIM_PID" 2>/dev/null
wait "$SIM_PID" 2>/dev/null
SIM_PID=""

echo "[s8] analyzing (app_status=$APP_STATUS)..."
python3 mpc_wbc/test/analyze_s4_log.py "$SIM_LOG" "$PHASES"
ANALYZE_STATUS=$?

if [[ $APP_STATUS -ne 0 ]]; then
  exit $APP_STATUS
fi
exit $ANALYZE_STATUS
