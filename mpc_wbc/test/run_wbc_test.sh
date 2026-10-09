#!/usr/bin/env bash
# S7 Weighted-QP WBC 离线自检（含 MuJoCo 正动力学独立复核）。
# 用法： mpc_wbc/test/run_wbc_test.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$ROOT/build"
QP_BIN="$BUILD_DIR/qp_ipm_test"
WBC_BIN="$BUILD_DIR/lite3_wbc_test"
CA_CASES="$BUILD_DIR/wbc_contact_accel_cases.txt"
WBC_CASES="$BUILD_DIR/wbc_mujoco_cases.csv"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[1/5] compiling C++ QP solver test..."
g++ -std=c++17 -O2 -Wall -Wextra -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/qp_ipm_test.cpp -o "$QP_BIN"

echo "[2/5] compiling C++ WBC test..."
g++ -std=c++17 -O2 -Wall -Wextra -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_wbc_test.cpp -o "$WBC_BIN"

echo "[3/5] generating MuJoCo contact-acceleration cases..."
python3 mpc_wbc/test/gen_mujoco_contact_accel_cases.py "$CA_CASES" 100

echo "[4/5] running QP + WBC self-checks..."
"$QP_BIN"
"$WBC_BIN" "$CA_CASES" "$WBC_CASES"

echo "[5/5] independent MuJoCo re-check of the WBC solution (forward dynamics)..."
python3 mpc_wbc/test/check_wbc_hover_mujoco.py "$WBC_CASES"
