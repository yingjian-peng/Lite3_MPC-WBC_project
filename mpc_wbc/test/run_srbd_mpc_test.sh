#!/usr/bin/env bash
# S6 SRBD 预测时域 MPC 离线自检（不需要 MuJoCo）。
# 用法： mpc_wbc/test/run_srbd_mpc_test.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$ROOT/build"
BIN="$BUILD_DIR/lite3_srbd_mpc_test"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[1/2] compiling C++ SRBD-MPC test..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_srbd_mpc_test.cpp -o "$BIN"

echo "[2/2] running self-check..."
"$BIN"
