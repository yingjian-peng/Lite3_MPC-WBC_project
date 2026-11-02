#!/usr/bin/env python3
"""单轴 / 多轴扫速的逐段指标汇总（闭环 S8 harness 专用）。

用法：
    analyze_axis_sweep.py <outdir> [--prefix-rows 4] [--settle 0.6] [--json out.json]

口径：
  * 只用 phases.csv 从第 `--prefix-rows` 行开始的**运动段**（settle/fold/raise/hold 之后）。
  * 线性速度用世界系速度按该段平均偏航旋转到机体系，再与 (vx,vy) 指令比较；
    角速度直接用 MuJoCo 机体系 qvel[3:6] 的 z 分量。
  * 稳态窗 = [t_start+settle, t_end]；倾角/高度看整段。
  * 跟踪误差：线速度向量差范数、角速度差绝对值；波动：5–95 分位峰峰值。
  * WARN 阈值：线速度误差 0.05 m/s、角速度误差 0.05 rad/s、倾角 10°；
    纹波阈值放宽到 0.30（正常 trot 单步纹波就有 0.1–0.25，不能当失稳）。
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, k):
    return np.array([float(r[k]) for r in rows], dtype=float)


def yaw_of(rows):
    w, x, y, z = col(rows, "qw"), col(rows, "qx"), col(rows, "qy"), col(rows, "qz")
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--prefix-rows", type=int, default=4)
    ap.add_argument("--settle", type=float, default=0.6)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    sim = load(os.path.join(args.outdir, "sim.csv"))
    phases = load(os.path.join(args.outdir, "phases.csv"))
    t = col(sim, "t")
    vx_w, vy_w = col(sim, "vx"), col(sim, "vy")
    wz_b = col(sim, "wz")
    bz = col(sim, "bz")
    roll, pitch, yaw = yaw_of(sim)
    tilt = np.degrees(np.maximum(np.abs(roll), np.abs(pitch)))

    print(f"=== {args.outdir} ===")
    print(f"{'phase':<10} {'cmd(vx,vy,wz)':<20} {'meas(vx,vy,wz)':<20} "
          f"{'err':<14} {'p2p':<14} {'tilt':<12} {'bz_min':<7} verdict")
    out = {"outdir": args.outdir, "phases": []}
    for i, ph in enumerate(phases):
        if i < args.prefix_rows:
            continue
        t0, t1 = float(ph["t_start"]), float(ph["t_end"])
        cmd = np.array([float(ph["vx"]), float(ph["vy"]), float(ph["wz"])])
        # 停步段要等减速瞬态走完（软停步≈0.9 s + 平滑），否则会把过渡算成跟踪误差
        settle = args.settle if np.any(np.abs(cmd) > 1e-9) else max(args.settle, 1.3)
        idx_all = np.where((t >= t0) & (t <= t1))[0]
        idx = np.where((t >= t0 + settle) & (t <= t1))[0]
        if idx.size < 5:
            print(f"{ph['phase']:<10} 数据不足（{idx_all.size} 点）")
            out["phases"].append({"phase": ph["phase"], "cmd": cmd.tolist(),
                                  "verdict": "NO_DATA"})
            continue
        # 必须**逐帧**用该帧偏航把世界系速度转到机体系。
        #
        # 原实现用「整段平均偏航」只转一次：原地转向时机身一直在转（0.5 rad/s × 8 s =
        # 4 rad），段内样本的方向跨度很大，均值会被系统性压小，看起来就像「转向抢了平移的
        # 摩擦预算」。实测对比（同一份 sim.csv）：
        #   f_x45_w50（0.45 + ωz 0.50，8 s 转 4 rad）
        #     平均偏航一次转 → vx 0.255（“损失 0.195”）
        #     逐帧转          → vx 0.499（无损失）
        # 两者差 2 倍。因此所有「组合工况平移损失」的结论必须在逐帧口径下重算。
        c, s = np.cos(yaw[idx]), np.sin(yaw[idx])
        vx_b = c * vx_w[idx] + s * vy_w[idx]
        vy_b = -s * vx_w[idx] + c * vy_w[idx]
        meas = np.array([float(np.mean(vx_b)), float(np.mean(vy_b)), float(np.mean(wz_b[idx]))])
        # 线速度沿指令方向投影 / 垂直分量；误差取向量差
        lin_err = float(np.linalg.norm(meas[:2] - cmd[:2]))
        yaw_err = abs(float(meas[2] - cmd[2]))
        # 峰峰值（5–95 分位，排除个别冲击）
        p2p_lin = float(np.hypot(np.percentile(vx_b, 95) - np.percentile(vx_b, 5),
                                 np.percentile(vy_b, 95) - np.percentile(vy_b, 5)))
        p2p_yaw = float(np.percentile(wz_b[idx], 95) - np.percentile(wz_b[idx], 5))
        tilt_max = float(tilt[idx_all].max())
        tilt_p95 = float(np.percentile(tilt[idx], 95))
        bz_min = float(bz[idx_all].min())

        problems = []
        if tilt_max > 25 or bz_min < 0.15:
            problems.append("FELL")
        if lin_err > 0.05:
            problems.append(f"lin_err>{0.05}")
        if yaw_err > 0.05:
            problems.append(f"yaw_err>{0.05}")
        if p2p_lin > 0.30:
            problems.append(f"p2p_lin>{0.30}")
        if p2p_yaw > 0.30:
            problems.append(f"p2p_yaw>{0.30}")
        if tilt_max > 10:
            problems.append("tilt>10")
        verdict = "OK" if not problems else ("FELL" if "FELL" in problems else "WARN")

        print(f"{ph['phase']:<10} "
              f"({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f})   "
              f"({meas[0]:+.3f},{meas[1]:+.3f},{meas[2]:+.3f}) "
              f"lin{lin_err:.3f}/yaw{yaw_err:.3f}  "
              f"lin{p2p_lin:.3f}/yaw{p2p_yaw:.3f}  "
              f"max{tilt_max:5.1f}/p95{tilt_p95:4.1f}  {bz_min:.3f}  {verdict}"
              + (f"  [{' '.join(problems)}]" if problems else ""))
        out["phases"].append({
            "phase": ph["phase"], "cmd": cmd.tolist(), "meas": meas.tolist(),
            "lin_err": lin_err, "yaw_err": yaw_err,
            "p2p_lin": p2p_lin, "p2p_yaw": p2p_yaw,
            "tilt_max": tilt_max, "tilt_p95": tilt_p95, "bz_min": bz_min,
            "verdict": verdict, "problems": problems,
        })
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
