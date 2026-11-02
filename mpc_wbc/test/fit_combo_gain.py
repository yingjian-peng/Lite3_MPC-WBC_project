#!/usr/bin/env python3
"""按「稳态窗口 + 逐帧机体系」重算组合工况的平移损失，用于标定转向-平移竞争系数。

为什么要单独一个脚本：
  * `analyze_axis_sweep.py` 把整段（含起步斜坡）都算进均值，8 s 段的起步过渡会给
    "平移损失"多贡献 0.02-0.04 m/s；
  * 它把世界系速度用**该段平均偏航**旋转一次，而原地转向时机身一直在转（8 s 转 1.6 rad），
    同段样本会被系统性压小 -> 长转向段的平移量失真。

这里改为：**逐帧用该帧偏航把世界速度转到机体系**，只取每段最后 (duration-skip) 秒均值，
再按  dv ~= k * vxy * wz^2  反解 k，并按 |wz| 分档统计。

用法：
    python mpc_wbc/test/fit_combo_gain.py <outdir> [...] [--skip 2.0]
"""
import argparse
import csv
import math
import os
import sys

import numpy as np


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, k):
    return np.array([float(r[k]) for r in rows], dtype=float)


def yaw_of(rows):
    w, x, y, z = col(rows, "qw"), col(rows, "qx"), col(rows, "qy"), col(rows, "qz")
    return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdirs", nargs="+")
    ap.add_argument("--skip", type=float, default=2.0, help="每段跳过的起步时间 [s]")
    ap.add_argument("--prefix-rows", type=int, default=4)
    args = ap.parse_args()

    print(f"{'case':26s} {'cmd(vx,vy,wz)':20s} {'meas_body':18s} "
          f"{'vxy_cmd':>7s} {'|wz|':>5s} {'loss':>7s} {'k':>6s}")
    print("-" * 96)
    rows = []
    for d in args.outdirs:
        sim = load(os.path.join(d, "sim.csv"))
        if len(sim) < 100:
            print(f"{os.path.basename(d):26s} data insufficient")
            continue
        t = col(sim, "t")
        vx_w, vy_w = col(sim, "vx"), col(sim, "vy")
        yaw = yaw_of(sim)
        c, s = np.cos(yaw), np.sin(yaw)
        vx_b = c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w
        wz_b = col(sim, "wz")
        for ph in load(os.path.join(d, "phases.csv"))[args.prefix_rows:]:
            vx_c, vy_c, wz_c = float(ph["vx"]), float(ph["vy"]), float(ph["wz"])
            if abs(vx_c) < 1e-9 and abs(vy_c) < 1e-9 and abs(wz_c) < 1e-9:
                continue
            t0, t1 = float(ph["t_start"]), float(ph["t_end"])
            idx = np.where((t >= t0 + args.skip) & (t <= t1 - 0.2))[0]
            if idx.size < 50:
                continue
            mx, my, mw = vx_b[idx].mean(), vy_b[idx].mean(), wz_b[idx].mean()
            vxy_cmd = math.hypot(vx_c, vy_c)
            if vxy_cmd > 1e-9:
                ux, uy = vx_c / vxy_cmd, vy_c / vxy_cmd
                loss = vxy_cmd - (mx * ux + my * uy)
            else:
                loss = 0.0
            w2 = wz_c * wz_c
            k = loss / (vxy_cmd * w2) if vxy_cmd > 1e-9 and w2 > 1e-12 else float("nan")
            tag = f"{os.path.basename(d)}:{ph['phase']}"
            print(f"{tag:26s} ({vx_c:+.2f},{vy_c:+.2f},{wz_c:+.2f})  "
                  f"({mx:+.3f},{my:+.3f},{mw:+.3f})  "
                  f"{vxy_cmd:7.3f} {abs(wz_c):5.2f} {loss:7.3f} {k:6.2f}")
            rows.append((abs(wz_c), vxy_cmd, loss, k))

    print("\nband summary (|wz|), suggested = p95 of k")
    for lo, hi in [(0.0, 0.25), (0.25, 0.40), (0.40, 0.60), (0.60, 1.01)]:
        ks = sorted(r[3] for r in rows
                    if lo <= r[0] < hi and not math.isnan(r[3]))
        if not ks:
            print(f"  [{lo:.2f},{hi:.2f})  no sample")
            continue
        p95 = ks[min(len(ks) - 1, int(len(ks) * 0.95))]
        print(f"  [{lo:.2f},{hi:.2f})  n={len(ks):2d}  "
              f"k min={ks[0]:+.2f} p50={ks[len(ks)//2]:+.2f} p95={p95:+.2f} max={ks[-1]:+.2f}")


if __name__ == "__main__":
    sys.exit(main())
