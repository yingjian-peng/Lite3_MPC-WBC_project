#!/usr/bin/env python3
"""跑一个 MPC+WBC（S8）场景：起仿真 → 跑控制器 → 收仿真。

与 sweep_gait_params.py 的区别：这里是 `mpc_wbc_mpc_wbc_sim`（估计 → 步态 →
SRBD-MPC → WBC），用于 R6/R7 的闭环迭代；命令来自 S8_PHASES（stand/walk/场景文件）。

用法：
    python3 mpc_wbc/test/run_s8_case.py --outdir build/r6/walk --phases walk
    python3 mpc_wbc/test/run_s8_case.py --outdir build/r6/a --phases walk \
        --env S8_GAIT_SCHEDULE=0 --sim-env SIM_SLOPE_DEG=2

产物：<outdir>/sim.csv、<outdir>/phases.csv、<outdir>/diag.csv
退出码：控制器的退出码（0 正常；3/4/5 分别对应姿态超限 / 控制器故障 / 仿真掉线）
"""

import argparse
import os
import subprocess
import sys
import time

SIM_SCRIPT = "interface/robot/simulation/mujoco_simulation.py"
CTRL_BIN = "./build/mpc_wbc/mpc_wbc_mpc_wbc_sim"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--phases", default="stand", help="stand | walk | <场景 csv 路径>")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="仿真时长 [s]；0 表示自动（起立 4.8 s + 场景时长 + 20 s 余量）")
    ap.add_argument("--sim-env", action="append", default=[], help="仿真侧环境变量 K=V")
    ap.add_argument("--env", action="append", default=[], help="控制器侧环境变量 K=V")
    ap.add_argument("--python", default="/tmp/lite3_venv/bin/python")
    ap.add_argument("--sim-log-every", default="10")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    paths = {k: os.path.join(args.outdir, f) for k, f in
             (("sim", "sim.csv"), ("phases", "phases.csv"), ("diag", "diag.csv"))}
    for p in paths.values():
        if os.path.exists(p):
            os.remove(p)

    duration = args.duration
    if duration <= 0.0:
        duration = 80.0  # 默认给足：控制器跑完就主动终止仿真

    sim_env = dict(os.environ)
    sim_env.update(MUJOCO_VIEWER="0", SIM_DEBUG="0", SIM_DURATION=str(duration),
                   SIM_LOG=paths["sim"], SIM_LOG_EVERY=args.sim_log_every,
                   SIM_INIT_POSE="stand")
    for kv in args.sim_env:
        k, v = kv.split("=", 1)
        sim_env[k.strip()] = v.strip()

    sim = subprocess.Popen([args.python, SIM_SCRIPT], env=sim_env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, bufsize=1)
    deadline = time.time() + 60.0
    ready = False
    while time.time() < deadline:
        line = sim.stdout.readline()
        if not line:
            break
        if "logging to" in line:
            ready = True
            break
    if not ready:
        sim.kill()
        print("[s8case] simulation failed to start", file=sys.stderr)
        return 2

    ctrl_env = dict(os.environ)
    ctrl_env["S8_PHASES"] = args.phases
    for kv in args.env:
        k, v = kv.split("=", 1)
        ctrl_env[k.strip()] = v.strip()

    t0 = time.time()
    ctrl = subprocess.run([CTRL_BIN, paths["phases"], paths["diag"]],
                          env=ctrl_env, capture_output=True, text=True)
    wall = time.time() - t0

    if sim.poll() is None:
        sim.terminate()
    try:
        sim.wait(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()
        sim.wait()

    print(ctrl.stdout.rstrip())
    if ctrl.stderr.strip():
        print(ctrl.stderr.rstrip(), file=sys.stderr)
    print(f"[s8case] rc={ctrl.returncode} wall={wall:.1f}s out={args.outdir}")
    return ctrl.returncode


if __name__ == "__main__":
    sys.exit(main())
