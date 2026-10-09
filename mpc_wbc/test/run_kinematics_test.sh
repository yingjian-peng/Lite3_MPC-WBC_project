#!/usr/bin/env bash
# S1 运动学对表：生成 MuJoCo 真值 -> 编译测试 -> 运行比较。
# 用法： mpc_wbc/test/run_kinematics_test.sh [num_cases]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM="${1:-200}"
BUILD_DIR="$ROOT/build"
OUT="$BUILD_DIR/kinematics_cases.txt"
BIN="$BUILD_DIR/lite3_kinematics_test"

EIGEN="$ROOT/support/third_party/eigen"
if [[ ! -f "$EIGEN/Eigen/Dense" ]]; then
  EIGEN="$ROOT/support/third_party/eigen/eigen3"
fi

mkdir -p "$BUILD_DIR"
cd "$ROOT"

echo "[1/3] generating MuJoCo oracle cases (n=$NUM)..."
python3 mpc_wbc/test/gen_mujoco_cases.py "$OUT" "$NUM"

echo "[2/3] compiling C++ kinematics test..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I "$EIGEN" -I "$ROOT/mpc_wbc" \
    mpc_wbc/test/lite3_kinematics_test.cpp -o "$BIN"

echo "[3/3] comparing against MuJoCo..."
"$BIN" "$OUT"
