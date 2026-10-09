#!/usr/bin/env python3
"""S9 A/B 报告：把 build/s9/*.json 的逐段指标汇成对照表（baseline = S4 位置环，mpc = S8 MPC+WBC）。

用法： s9_ab_report.py [--dir build/s9] [--out build/s9/report.md]
"""
import argparse
import glob
import json
import os
import sys


def load_runs(path):
    runs = {}
    for f in sorted(glob.glob(os.path.join(path, "*.json"))):
        try:
            with open(f) as fh:
                d = json.load(fh)
        except Exception as e:  # noqa: BLE001
            print(f"[warn] skip {f}: {e}")
            continue
        label = d.get("label") or ""
        if "/" not in label:
            continue
        case, who = label.split("/", 1)
        runs.setdefault(case, {})[who] = d
    return runs


def fmt(v, nd=2, suffix=""):
    if v is None:
        return "-"
    if isinstance(v, str):
        return v
    return f"{v:.{nd}f}{suffix}"


def per_phase_table(case, base, mpc, out):
    if mpc is None:
        return
    out.append(f"| 相位 | 路径 | along(m) | v_err(m/s) | tilt_max(°) | bz_min | 饱和% | 滑移(m/s) | 判定 |")
    out.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    base_phases = {p["phase"]: p for p in (base or {}).get("phases_out", [])}
    for p in mpc.get("phases_out", []):
        b = base_phases.get(p["phase"])
        for who, r in (("baseline", b), ("mpc", p)):
            if r is None:
                continue
            out.append(
                f"| {r['phase']} | {who} | {fmt(r['along_m'], 3)} | {fmt(r['v_err'])} | "
                f"{fmt(r['tilt_max_deg'], 1)} | {fmt(r['bz_min'], 3)} | {fmt(r['sat_pct'], 1)} | "
                f"{fmt(r['slip_mean_mps'], 3)} | {r['verdict']} |")
    out.append("")
    # MPC 侧求解侧指标
    diag_any = any("diag" in p for p in mpc.get("phases_out", []))
    if diag_any:
        out.append("| 相位 | step p99(ms) | step max(ms) | >1ms 帧 | mpc_ms mean/max | MPC iters max | a_des=0% | 支撑数 min | WBC 激活% |")
        out.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for p in mpc.get("phases_out", []):
            d = p.get("diag")
            if not d:
                continue
            out.append(f"| {p['phase']} | {fmt(d['step_ms_p99'], 2)} | {fmt(d['step_ms_max'], 2)} | "
                       f"{d['frames_over_1ms']} | {fmt(d['mpc_ms_mean'], 2)}/{fmt(d['mpc_ms_max'], 2)} | "
                       f"{d['mpc_iters_max']} | {fmt(d['mpc_a_des_zero_pct'], 1)} | "
                       f"{d['num_support_min']} | {fmt(d['wbc_active_pct'], 1)} |")
        out.append("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="build/s9")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    runs = load_runs(args.dir)
    if not runs:
        print(f"[s9] no run json in {args.dir}")
        return 1

    out = ["# S9 A/B 报告（S4 位置环基线 vs S8 MPC+WBC）", "",
           "同一份场景文件、同一个 MuJoCo 仿真（`SIM_DURATION`/`SIM_INIT_YAW` 由 runner 设定）、",
           "同一套分析口径（`analyze_s9_run.py`：命令帧内位移/速度、整段姿态、力矩饱和、足端滑移）。", "",
           "## 汇总", "",
           "| 场景 | baseline | mpc | 首段失败(tilt/bz) |",
           "| --- | --- | --- | --- |"]
    for case in sorted(runs):
        b = runs[case].get("baseline")
        m = runs[case].get("mpc")
        def verdict(d):
            if d is None:
                return "-"
            return d.get("result", "?")
        def trouble(d):
            if d is None:
                return "-"
            for r in d.get("phases_out", []):
                if r["verdict"] != "OK":
                    return f"{r['phase']}: " + "; ".join(r["problems"])
            if d.get("prefix_abnormal"):
                return (f"起立段异常(tilt_max={d['prefix_max_tilt_deg']:.0f}°, "
                        f"bz_min={d['prefix_min_bz']:.2f})")
            if d.get("fell_at") is not None:
                return f"fall@{d['fell_at']:.1f}s"
            return "—"
        out.append(f"| {case} | {verdict(b)} | {verdict(m)} | {trouble(b if b and b.get('result')=='FAIL' else m)} |")
    out.append("")

    for case in sorted(runs):
        out.append(f"## {case}")
        out.append("")
        per_phase_table(case, runs[case].get("baseline"), runs[case].get("mpc"), out)

    text = "\n".join(out) + "\n"
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        print(f"[s9] wrote {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
