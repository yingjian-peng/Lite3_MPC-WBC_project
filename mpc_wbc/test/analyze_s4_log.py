#!/usr/bin/env python3
"""Analyze an S4 position-loop run: check uprightness and per-phase motion.

用法： python3 mpc_wbc/test/analyze_s4_log.py <sim_log.csv> <phases.csv>
返回 0 表示全部通过。
"""
import csv
import math
import sys

import numpy as np


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, k):
    return np.array([float(r[k]) for r in rows])


def euler(rows):
    w, x, y, z = col(rows, "qw"), col(rows, "qx"), col(rows, "qy"), col(rows, "qz")
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def main():
    sim = load(sys.argv[1])
    phases = load(sys.argv[2])
    t = col(sim, "t")
    roll, pitch, yaw = euler(sim)
    bx, by, bz = col(sim, "bx"), col(sim, "by"), col(sim, "bz")

    def window(t0, t1):
        m = (t >= t0) & (t <= t1)
        return m

    failures = 0
    print(f"{'phase':<10} {'dt':>5} {'dx':>8} {'dy':>8} {'dyaw(deg)':>10}  {'max|roll|(deg)':>14} {'max|pitch|(deg)':>15} {'min bz':>7}")

    # 行走阶段：跳过首尾 0.3s 过渡
    checks = {
        "walk_fwd": ("dx", lambda v: v > 0.08),
        "walk_back": ("dx", lambda v: v < -0.08),
        "walk_left": ("dy", lambda v: v > 0.05),
        "turn": ("dyaw", lambda v: v > math.radians(20.0)),
    }

    for ph in phases:
        name = ph["phase"]
        if name not in checks:
            continue
        t0, t1 = float(ph["t_start"]) + 0.3, float(ph["t_end"]) - 0.3
        m = window(t0, t1)
        if m.sum() < 2:
            print(f"{name:<10} insufficient samples")
            failures += 1
            continue
        dx = bx[m][-1] - bx[m][0]
        dy = by[m][-1] - by[m][0]
        dyaw = yaw[m][-1] - yaw[m][0]
        mr = math.degrees(np.abs(roll[m]).max())
        mp = math.degrees(np.abs(pitch[m]).max())
        mbz = bz[m].min()
        key, pred = checks[name]
        value = {"dx": dx, "dy": dy, "dyaw": dyaw}[key]
        ok = pred(value) and mr < 25.0 and mp < 25.0 and mbz > 0.15
        if not ok:
            failures += 1
        print(f"{name:<10} {t1-t0:5.2f} {dx:8.3f} {dy:8.3f} {math.degrees(dyaw):10.1f}  "
              f"{mr:14.1f} {mp:15.1f} {mbz:7.3f}  {'OK' if ok else 'FAIL'}")

    # 全局：站立之后应始终不跌倒
    stand_t0 = float([p for p in phases if p["phase"] == "hold"][0]["t_start"])
    m = t >= stand_t0
    max_tilt = max(math.degrees(np.abs(roll[m]).max()), math.degrees(np.abs(pitch[m]).max()))
    min_bz = bz[m].min()
    print(f"\nstanding phase: max tilt={max_tilt:.1f} deg, min base z={min_bz:.3f}")
    if max_tilt > 25.0 or min_bz < 0.15:
        failures += 1

    print("RESULT:", "PASS" if failures == 0 else "FAIL")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
