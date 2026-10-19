#!/usr/bin/env python3
"""串行扫参驱动：固定场景，遍历某个速度段的 (T, d, H)，汇总稳态指标。

用途：速度区间更新后重新整定分段步态参数。每次只做一段，避免多段耦合，
使 `--period` 与判定口径唯一。

用法：
    python3 mpc_wbc/test/sweep_gait_params.py \
        --segment high \
        --scenario mpc_wbc/test/scenarios/verify_single_high.csv \
        --configs 0.44:0.625:0.07,0.48:0.65:0.07,0.52:0.64:0.07 \
        --duration 68 --outdir build/sweep_high

约定：
- 仿真按实时节拍跑 1 kHz，所以 --duration 应略大于场景总时长；
- 一次只起一个仿真进程（UDP 端口独占）；
- 每个配置一份 report.json，最后打印横向对比表。
"""

import argparse
import json
import os
import subprocess
import sys
import time

SIM_SCRIPT = "interface/robot/simulation/mujoco_simulation.py"
CTRL_BIN = "./build/mpc_wbc/mpc_wbc_position_loop_sim"
ANALYZER = "mpc_wbc/test/analyze_s4_steady.py"

SEG_ENV = {
    "low": "S4_LOW",
    "mid": "S4_MID",
    "high": "S4_HIGH",
}

# 该段要覆盖的最大前进速度（用于换算最严苛步幅）。取段上界，high 取全局上限。
SEG_VMAX = {
    "low": 0.20,
    "mid": 0.35,
    "high": 0.60,
}


def parse_configs(text):
    out = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) != 3:
            raise SystemExit(f"bad config '{item}', expect T:d:H")
        out.append(tuple(float(x) for x in parts))
    return out


def run_one(cfg, args, tag):
    t_period, duty, height = cfg
    outdir = os.path.join(args.outdir, tag)
    os.makedirs(outdir, exist_ok=True)
    for name in ("sim.csv", "phases.csv", "segments.csv", "report.json"):
        path = os.path.join(outdir, name)
        if os.path.exists(path):
            os.remove(path)

    sim_env = dict(os.environ)
    sim_env.update(
        MUJOCO_VIEWER="0",
        SIM_DEBUG="0",
        SIM_DURATION=str(args.duration),
        SIM_LOG=os.path.join(outdir, "sim.csv"),
        SIM_LOG_EVERY="10",
        SIM_INIT_POSE="stand",
    )
    for kv in args.sim_env:
        if "=" not in kv:
            raise SystemExit(f"bad --sim-env '{kv}', expect K=V")
        k, v = kv.split("=", 1)
        sim_env[k.strip()] = v.strip()
    python = args.python
    sim = subprocess.Popen(
        [python, SIM_SCRIPT],
        env=sim_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    # 等仿真进入主循环（打印 "logging to" 之后才开始发观测）
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
        raise SystemExit(f"[{tag}] simulation failed to start")

    ctrl_env = dict(os.environ)
    ctrl_env["S9_SCENARIO"] = args.scenario
    ctrl_env["S4_SEG_LOG"] = os.path.join(outdir, "segments.csv")
    prefix = SEG_ENV[args.segment]
    ctrl_env[f"{prefix}_PERIOD"] = f"{t_period}"
    ctrl_env[f"{prefix}_DUTY"] = f"{duty}"
    ctrl_env[f"{prefix}_SWING_HEIGHT"] = f"{height}"
    for kv in args.env:
        if "=" not in kv:
            raise SystemExit(f"bad --env '{kv}', expect K=V")
        k, v = kv.split("=", 1)
        ctrl_env[k.strip()] = v.strip()

    t0 = time.time()
    ctrl = subprocess.run(
        [CTRL_BIN, os.path.join(outdir, "phases.csv")],
        env=ctrl_env,
        capture_output=True,
        text=True,
    )
    wall = time.time() - t0

    if sim.poll() is None:
        sim.terminate()
    try:
        sim.wait(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()
        sim.wait()

    report_path = os.path.join(outdir, "report.json")
    analyzer_cmd = [
        python,
        ANALYZER,
        os.path.join(outdir, "sim.csv"),
        os.path.join(outdir, "phases.csv"),
        "--prefix-rows",
        "3",
        "--settle",
        f"{args.settle}",
        "--period",
        f"{t_period}",
        "--label",
        tag,
        "--json",
        report_path,
    ]
    if args.seg_log:
        analyzer_cmd += ["--seg-log", os.path.join(outdir, "segments.csv")]
    subprocess.run(analyzer_cmd, capture_output=True, text=True)

    if not os.path.exists(report_path):
        return {
            "tag": tag,
            "T": t_period,
            "d": duty,
            "H": height,
            "result": "ERROR",
            "cost": float("nan"),
            "tilt": float("nan"),
            "err_lin": float("nan"),
            "err_yaw": float("nan"),
            "p2p_lin": float("nan"),
            "p2p_yaw": float("nan"),
            "slip": float("nan"),
            "bz": float("nan"),
            "wall": wall,
            "ctrl_rc": ctrl.returncode,
        }

    with open(report_path) as f:
        rep = json.load(f)

    motion = [p for p in rep["phases_out"] if p.get("is_motion")]
    if not motion:
        motion = rep["phases_out"]

    def worst(key):
        if not motion:
            return float("nan")
        return max(p[key] for p in motion)

    return {
        "tag": tag,
        "T": t_period,
        "d": duty,
        "H": height,
        "result": rep["result"],
        "cost": rep["cost"],
        "tilt": rep["max_tilt_deg"],
        "err_lin": worst("err_lin"),
        "err_yaw": worst("err_yaw"),
        "p2p_lin": worst("ma_p2p_lin"),
        "p2p_yaw": worst("ma_p2p_yaw"),
        "slip": worst("slip_mean_mps"),
        "bz": rep["min_bz"] if rep["min_bz"] is not None else float("nan"),
        "wall": wall,
        "ctrl_rc": ctrl.returncode,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--segment", required=True, choices=["low", "mid", "high"])
    ap.add_argument("--scenario", required=True)
    ap.add_argument(
        "--configs",
        required=True,
        help="T:d:H 逗号分隔，例如 0.44:0.625:0.07,0.52:0.64:0.07",
    )
    ap.add_argument("--duration", type=float, default=68.0)
    ap.add_argument(
        "--settle",
        type=float,
        default=2.0,
        help="稳态窗跳过的过渡时间 [s]；长周期段建议 >= 2T",
    )
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument(
        "--vmax",
        type=float,
        default=None,
        help="换算步幅用的速度（默认取该段上界）",
    )
    ap.add_argument(
        "--env",
        action="append",
        default=[],
        help="额外注入给控制器的环境变量 K=V（可重复）",
    )
    ap.add_argument(
        "--seg-log",
        action="store_true",
        help="混合速度场景：按 segments.csv 的逐相位周期分析",
    )
    ap.add_argument(
        "--sim-env",
        action="append",
        default=[],
        help="额外注入给仿真的环境变量 K=V（可重复），如 SIM_SLOPE_DEG=2"
    )
    args = ap.parse_args()

    if args.vmax is None:
        args.vmax = SEG_VMAX[args.segment]

    configs = parse_configs(args.configs)
    os.makedirs(args.outdir, exist_ok=True)

    rows = []
    for i, cfg in enumerate(configs):
        tag = f"T{cfg[0]:.3f}_d{cfg[1]:.3f}_H{cfg[2]:.3f}"
        if args.env:
            tag += "_" + "_".join(
                kv.replace("=", "") for kv in args.env
            )
        if args.sim_env:
            tag += "_" + "_".join(
                kv.replace("=", "") for kv in args.sim_env
            )
        print(f"\n=== [{i + 1}/{len(configs)}] {args.segment} {tag} ===", flush=True)
        row = run_one(cfg, args, tag)
        rows.append(row)
        print(
            f"  -> {row['result']:5s} cost={row['cost']:.3f} "
            f"tilt={row['tilt']:.1f} err_lin={row['err_lin']:.3f} "
            f"err_yaw={row['err_yaw']:.3f} slip={row['slip']:.3f}",
            flush=True,
        )

    header = (
        f"{'T':>6} {'d':>6} {'H':>6} {'步频Hz':>8} {'步幅m':>7} "
        f"{'结果':>6} {'cost':>6} {'tilt':>6} {'errLin':>7} {'errYaw':>7} "
        f"{'p2pLin':>7} {'p2pYaw':>7} {'slip':>6} {'bz':>6}"
    )
    print("\n" + "=" * len(header))
    print(f"segment={args.segment}  scenario={args.scenario}")
    print(header)
    print("-" * len(header))
    for r in rows:
        stride = args.vmax * r["T"] * r["d"] if args.vmax > 0 else float("nan")
        print(
            f"{r['T']:6.3f} {r['d']:6.3f} {r['H']:6.3f} {1.0 / r['T']:8.2f} "
            f"{stride:7.3f} {r['result']:>6} {r['cost']:6.3f} {r['tilt']:6.1f} "
            f"{r['err_lin']:7.3f} {r['err_yaw']:7.3f} {r['p2p_lin']:7.3f} "
            f"{r['p2p_yaw']:7.3f} {r['slip']:6.3f} {r['bz']:6.3f}"
        )
    print("=" * len(header))

    with open(os.path.join(args.outdir, "sweep_summary.json"), "w") as f:
        json.dump(
            {"segment": args.segment, "scenario": args.scenario, "runs": rows},
            f,
            indent=2,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
