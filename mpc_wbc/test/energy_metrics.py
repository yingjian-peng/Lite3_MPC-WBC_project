#!/usr/bin/env python3
"""从 sim.csv 估算关节侧机械功率与铜损代理，用于步态参数的功耗对比。

指标（只看运动相，跳过停步段）：
- P_abs = mean Σ|τ_i·qd_i|     关节绝对机械功率（W），升降都算消耗
- P_net = mean Σ τ_i·qd_i      净机械功率（W），负值表示回馈
- P_cu  ∝ mean Σ τ_i²          绕组铜损代理（W·N⁻²·m⁻² 归一化前）
- 同时给出 |τ| 均值，便于判断是否贴近力矩上限

用法：
    python3 mpc_wbc/test/energy_metrics.py <sim.csv> <phases.csv> \
        [--settle 2.0] [--json out.json]
"""

import argparse
import csv
import json
import math
import sys


def load_sim(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        cols = {
            "t": idx["t"],
            "qd": [idx[f"qd{i}"] for i in range(12)],
            "tau": [idx[f"atau{i}"] for i in range(12)],
        }
        rows = []
        for raw in reader:
            if len(raw) < len(header):
                continue
            try:
                rows.append(
                    (
                        float(raw[cols["t"]]),
                        [float(raw[i]) for i in cols["qd"]],
                        [float(raw[i]) for i in cols["tau"]],
                    )
                )
            except ValueError:
                continue
    return rows


def load_phases(path):
    """读取控制器输出的 phases.csv（列：phase,t_start,t_end,...,cmd 等）。"""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name.strip(): i for i, name in enumerate(header)}
        out = []
        for raw in reader:
            if len(raw) < len(header):
                continue
            try:
                out.append(
                    {
                        "phase": raw[idx["phase"]],
                        "t_start": float(raw[idx["t_start"]]),
                        "t_end": float(raw[idx["t_end"]]),
                        "vx": float(raw[idx["vx"]]),
                        "vy": float(raw[idx["vy"]]),
                        "wz": float(raw[idx["wz"]]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return out


def is_motion(p):
    return math.hypot(p["vx"], p["vy"]) > 1e-6 or abs(p["wz"]) > 1e-6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_csv")
    ap.add_argument("phases_csv")
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--json")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    sim = load_sim(args.sim_csv)
    phases = load_phases(args.phases_csv)

    out = {"label": args.label, "sim_log": args.sim_csv, "phases": []}
    tot_pabs = tot_pcu = tot_n = 0.0
    for ph in phases:
        t0 = ph["t_start"] + args.settle
        t1 = ph["t_end"]
        if t1 <= t0:
            continue
        sel = [r for r in sim if t0 <= r[0] <= t1]
        if not sel:
            continue
        p_abs = 0.0
        p_net = 0.0
        p_cu = 0.0
        tau_abs = 0.0
        for _, qd, tau in sel:
            p_abs += sum(abs(t * v) for t, v in zip(tau, qd))
            p_net += sum(t * v for t, v in zip(tau, qd))
            p_cu += sum(t * t for t in tau)
            tau_abs += sum(abs(t) for t in tau)
        n = len(sel)
        entry = {
            "phase": ph["phase"],
            "t_start": t0,
            "t_end": t1,
            "is_motion": is_motion(ph),
            "p_abs_w": p_abs / n,
            "p_net_w": p_net / n,
            "p_cu_proxy": p_cu / n,
            "tau_abs_mean": tau_abs / n,
            "samples": n,
        }
        out["phases"].append(entry)
        if entry["is_motion"]:
            tot_pabs += entry["p_abs_w"]
            tot_pcu += entry["p_cu_proxy"]
            tot_n += 1

    if tot_n:
        out["p_abs_w_mean"] = tot_pabs / tot_n
        out["p_cu_proxy_mean"] = tot_pcu / tot_n
    else:
        out["p_abs_w_mean"] = None
        out["p_cu_proxy_mean"] = None

    print(
        f"=== energy: {args.label or args.sim_csv} "
        f"(motion phases only, settle={args.settle}s) ==="
    )
    print(
        f"{'phase':16} {'P_abs[W]':>9} {'P_net[W]':>9} {'P_cu':>8} {'|tau|[Nm]':>10}"
    )
    for e in out["phases"]:
        flag = "" if e["is_motion"] else "  (stop)"
        print(
            f"{e['phase']:16} {e['p_abs_w']:9.2f} {e['p_net_w']:9.2f} "
            f"{e['p_cu_proxy']:8.2f} {e['tau_abs_mean']:10.2f}{flag}"
        )
    if out["p_abs_w_mean"] is not None:
        print(
            f"\nmean over motion phases: P_abs={out['p_abs_w_mean']:.2f} W, "
            f"P_cu_proxy={out['p_cu_proxy_mean']:.2f}"
        )

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[energy] wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
