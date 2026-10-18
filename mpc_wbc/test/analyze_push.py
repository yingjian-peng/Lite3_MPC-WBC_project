#!/usr/bin/env python3
"""分析推力扰动测试：峰值倾角、恢复时间、残余航向偏转。

用法：
    python3 mpc_wbc/test/analyze_push.py <run_dir> [<run_dir> ...]

<run_dir> 是 sweep_gait_params.py 的输出目录（内含 sim.csv / phases.csv）。
扰动时刻默认从 sim 日志里推断：取 tilt 梯度的最大处前一帧；也可用 --t0 指定。
"""

import argparse
import csv
import os
import sys

import numpy as np


def load(sim_csv):
    rows = list(csv.DictReader(open(sim_csv)))
    t = np.array([float(r["t"]) for r in rows])
    w, x, y, z = (np.array([float(r[k]) for r in rows]) for k in ("qw", "qx", "qy", "qz"))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    tilt = np.degrees(np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w * y + x * z), 1 - 2 * (y * y + z * z))))
    return t, tilt, yaw


def analyze(path, t0=None, window=3.0):
    t, tilt, yaw = load(os.path.join(path, "sim.csv"))
    if t0 is None:
        # 用基线期（1–5 s）的均值 + 5σ 找第一个越界点
        base_win = (t > 1.0) & (t < 5.0)
        base, sigma = tilt[base_win].mean(), tilt[base_win].std()
        thresh = base + max(5.0 * sigma, 1.0)
        hit = np.where(tilt > thresh)[0]
        t0 = float(t[hit[0]]) if len(hit) else float("nan")
    if not np.isfinite(t0):
        return {"run": path, "t0": None, "note": "未检测到扰动响应"}

    pre = (t > t0 - 1.5) & (t < t0)
    post = (t >= t0) & (t < t0 + window)
    base_tilt = float(tilt[pre].mean()) if pre.any() else 0.0
    peak = float(tilt[post].max())

    recover = float("nan")
    for i in range(np.argmax(t >= t0 + 0.5), len(t)):
        if tilt[i] < base_tilt + 1.0:
            recover = float(t[i] - t0)
            break

    yaw_pre = float(yaw[(t > t0 - 1.5) & (t < t0)].mean())
    yaw_post = float(yaw[(t > t0 + 2.0) & (t < t0 + 3.0)].mean())
    label = os.path.basename(os.path.normpath(path))
    for prefix in ("T0.750_d0.700_H0.070_", "T0.440_d0.625_H0.070_", "T1.200_d0.720_H0.070_"):
        label = label.replace(prefix, "")
    label = label.replace("SIM_PUSH_FY", "Fy").replace("SIM_PUSH_", "")
    return {
        "run": label,
        "t0": round(t0, 2),
        "peak_tilt_deg": round(peak, 1),
        "recover_s": round(recover, 2) if np.isfinite(recover) else None,
        "yaw_residual_deg": round(yaw_post - yaw_pre, 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--t0", type=float, default=None, help="扰动时刻 [s]，默认自动检测")
    args = ap.parse_args()

    print(f"{'run':28} {'t0[s]':>6} {'峰值倾角':>9} {'恢复[s]':>8} {'残余航向':>9}")
    for d in args.dirs:
        r = analyze(d, args.t0)
        if r.get("note"):
            print(f"{r['run']:28}   {r['note']}")
            continue
        print(
            f"{r['run']:28} {r['t0']:6.2f} {r['peak_tilt_deg']:8.1f}° "
            f"{(r['recover_s'] if r['recover_s'] is not None else float('nan')):8.2f} "
            f"{r['yaw_residual_deg']:+8.1f}°"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
