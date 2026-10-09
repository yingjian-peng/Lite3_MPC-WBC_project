#!/usr/bin/env bash
# S2 全身动力学对表：生成 MuJoCo 真值 -> 编译测试 -> 运行比较。
# 用法： mpc_wbc/test/run_dynamics_test.sh [num_cases]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM="${1:-200}"
BUILD_DIR="$ROOT/build"
OUT="$BUILD_DIR/dynamics_cases.txt"
BIN="$BUILD_DIR/lite3_dynamics_test"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[1/3] generating MuJoCo oracle cases (n=$NUM)..."
python3 mpc_wbc/test/gen_mujoco_dynamics_cases.py "$OUT" "$NUM"

echo "[2/3] compiling C++ dynamics test..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_dynamics_test.cpp -o "$BIN"

echo "[3/3] comparing against MuJoCo..."
"$BIN" "$OUT"
