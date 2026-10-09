#!/usr/bin/env bash
# S5 状态估计验证：生成 MuJoCo 小跑 rollout -> 编译测试 -> 回放对比。
# 用法： mpc_wbc/test/run_state_estimator_test.sh [duration_s]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DURATION="${1:-10}"
BUILD_DIR="$ROOT/build"
OUT="$BUILD_DIR/estimator_rollout.csv"
BIN="$BUILD_DIR/lite3_state_estimator_test"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[1/3] generating MuJoCo trot rollout (${DURATION}s)..."
python3 mpc_wbc/test/gen_estimator_rollout.py "$OUT" "$DURATION"

echo "[2/3] compiling C++ estimator test..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_state_estimator_test.cpp -o "$BIN"

echo "[3/3] replaying rollout..."
"$BIN" "$OUT"
