#!/usr/bin/env python3
"""S4 开环分段调参分析器：只看稳态窗口，不计起步/切换瞬态。

用法：
    analyze_s4_steady.py <sim_log.csv> <phases.csv> [--prefix-rows 3]
                         [--settle -1] [--period 0.6] [--label NAME] [--json out.json]

口径：
  * phases.csv 前 `--prefix-rows` 行是起立/静置，跳过；
  * 每个运动相位从 t_start + settle 开始取稳态窗，settle 默认 max(1.0, 2*period)；
  * 世界系速度按逐样本偏航角转到机体系；
  * 平稳性用「一个步态周期的滑动平均」的峰峰值和标准差衡量，而不是原始速度的
    高频步态波动；
  * 停步相位只报告残余速度，不计入调参 cost。

退出码：0 = 有输出；1 = 输入/日志不完整。
"""
import argparse
import csv
import json
import math
import sys

import numpy as np


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, key, default=0.0):
    return np.array([float(r.get(key, default) or 0.0) for r in rows])


def euler_yaw(rows):
    w, x, y, z = col(rows, "qw"), col(rows, "qx"), col(rows, "qy"), col(rows, "qz")
    return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def moving_average(x, n):
    n = max(1, int(n))
    if n <= 1 or len(x) < n:
        return x.copy()
    return np.convolve(x, np.ones(n, dtype=float) / n, mode="valid")


def slip_rate(t, px, py, pz, foot_radius, clearance=0.004):
    """接触足几何判据下的水平位移速率均值/95 分位 [m/s]。"""
    contact = pz < foot_radius + clearance
    idx = np.where(contact[:-1] & contact[1:])[0]
    if idx.size == 0:
        return 0.0, 0.0
    dt = t[idx + 1] - t[idx]
    good = dt > 1e-9
    if not np.any(good):
        return 0.0, 0.0
    rates = np.hypot(px[idx + 1] - px[idx], py[idx + 1] - py[idx])[good] / dt[good]
    return float(rates.mean()), float(np.percentile(rates, 95))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_log")
    ap.add_argument("phases")
    ap.add_argument("--prefix-rows", type=int, default=3)
    ap.add_argument("--settle", type=float, default=-1.0,
                    help="稳态窗跳过的过渡时间 [s]；<0 表示 max(1.0, 2*period)")
    ap.add_argument("--period", type=float, default=0.6,
                    help="默认步态周期 [s]，用于滑动平均窗；--seg-log 可按相位覆盖")
    ap.add_argument("--seg-log", default=None,
                    help="position_loop_sim 的 S4_SEG_LOG CSV，可按 phase_idx 读取各段实际 T")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", default=None)
    ap.add_argument("--foot-radius", type=float, default=0.022)
    ap.add_argument("--tau-max", type=float, default=30.0)
    args = ap.parse_args()

    sim = load(args.sim_log)
    phases = load(args.phases)
    if not sim or not phases:
        print("[steady] empty sim log or phases", file=sys.stderr)
        return 1

    period_by_phase = {}
    if args.seg_log:
        try:
            for r in load(args.seg_log):
                period_by_phase[int(float(r["phase_idx"]))] = float(r["period"])
        except (FileNotFoundError, KeyError, ValueError) as e:
            print(f"[warn] cannot read seg-log {args.seg_log}: {e}", file=sys.stderr)

    t = col(sim, "t")
    vx_w, vy_w = col(sim, "vx"), col(sim, "vy")
    wz_b = col(sim, "wz")
    bz = col(sim, "bz")
    yaw = euler_yaw(sim)
    w, x, y, z = col(sim, "qw"), col(sim, "qx"), col(sim, "qy"), col(sim, "qz")
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    tilt = np.degrees(np.maximum(np.abs(roll), np.abs(pitch)))

    c, s = np.cos(yaw), np.sin(yaw)
    vx_b = c * vx_w + s * vy_w
    vy_b = -s * vx_w + c * vy_w

    tau_abs = np.zeros(len(sim))
    if "atau0" in sim[0]:
        tau_abs = np.max(np.abs(np.column_stack([col(sim, f"atau{i}") for i in range(12)])), axis=1)

    settle = args.settle
    if settle < 0.0:
        settle = max(1.0, 2.0 * args.period)
    default_period = max(1e-3, args.period)
    dt_nom = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    default_ma_n = max(1, int(round(default_period / max(dt_nom, 1e-6))))

    out = {
        "label": args.label,
        "sim_log": args.sim_log,
        "phases": args.phases,
        "period": default_period,
        "settle": settle,
        "moving_avg_samples": default_ma_n,
        "phases_out": [],
        "cost": 0.0,
        "fell": False,
        "max_tilt_deg": 0.0,
        "min_bz": float("inf"),
    }

    print(f"=== S4 steady: {args.label or args.sim_log} "
          f"(default T={default_period:.3f}s, settle={settle:.2f}s) ===")
    print(f"{'phase':<14} {'cmd(vx,vy,wz)':>20} {'v_mean':>16} {'v_err':>7} "
          f"{'ma_p2p':>7} {'ma_std':>7} {'yaw_err':>7} {'tilt°':>6} {'bz_min':>7} "
          f"{'sat%':>5} {'slip':>6} {'cost':>7}")

    cost_motion = 0.0
    n_motion = 0
    n_warn = 0
    fell = False
    hard_fail = False

    for i, ph in enumerate(phases):
        if i < args.prefix_rows:
            continue
        t0, t1 = float(ph["t_start"]), float(ph["t_end"])
        cmd = np.array([float(ph["vx"]), float(ph["vy"]), float(ph["wz"])])
        phase_motion = bool(abs(cmd[0]) > 1e-9 or abs(cmd[1]) > 1e-9 or abs(cmd[2]) > 1e-9)
        phase_period = max(1e-3, period_by_phase.get(i, default_period))
        phase_ma_n = max(1, int(round(phase_period / max(dt_nom, 1e-6))))
        idx_all = np.where((t >= t0) & (t <= t1))[0]
        idx = np.where((t >= t0 + settle) & (t <= t1))[0]
        short_window = idx.size < 2 <= idx_all.size
        if short_window:
            idx = idx_all
        if idx.size < 2:
            print(f"{ph['phase'][:14]:<14} no samples")
            if phase_motion:
                hard_fail = True
            out["phases_out"].append({
                "phase": ph["phase"], "t_start": t0, "t_end": t1,
                "cmd": cmd.tolist(), "is_motion": phase_motion, "verdict": "NO_SAMPLES",
            })
            continue

        mean_vx = float(np.mean(vx_b[idx]))
        mean_vy = float(np.mean(vy_b[idx]))
        mean_wz = float(np.mean(wz_b[idx]))
        err_x = mean_vx - cmd[0]
        err_y = mean_vy - cmd[1]
        err_lin = float(math.hypot(err_x, err_y))
        err_yaw = float(abs(mean_wz - cmd[2]))

        ma_x = moving_average(vx_b[idx], phase_ma_n)
        ma_y = moving_average(vy_b[idx], phase_ma_n)
        ma_w = moving_average(wz_b[idx], phase_ma_n)
        p2p_lin = float(max(np.ptp(ma_x), np.ptp(ma_y))) if ma_x.size else 0.0
        std_lin = float(math.hypot(np.std(ma_x), np.std(ma_y))) if ma_x.size else 0.0
        p2p_yaw = float(np.ptp(ma_w)) if ma_w.size else 0.0

        tilt_max = float(tilt[idx_all].max())
        bz_min = float(bz[idx_all].min())
        sat_pct = 100.0 * float(np.mean(tau_abs[idx] >= 0.95 * args.tau_max))
        slips = [slip_rate(t[idx_all], col(sim, f"px{leg}")[idx_all],
                           col(sim, f"py{leg}")[idx_all],
                           col(sim, f"pz{leg}")[idx_all], args.foot_radius)
                 for leg in range(4)]
        slip_mean = float(np.mean([r[0] for r in slips]))
        slip_p95 = float(np.max([r[1] for r in slips]))

        is_motion = phase_motion
        phase_fell = bool(tilt_max > 25.0 or bz_min < 0.15)
        fell = bool(fell or phase_fell)
        if phase_fell:
            hard_fail = True

        # 开环调参只看稳态运动：速度误差 + 低频波动为主，姿态/饱和/滑移作惩罚。
        cost = 0.0
        if is_motion:
            cost = err_lin + 0.5 * err_yaw + 0.5 * p2p_lin + 0.5 * p2p_yaw
            cost += 0.02 * max(0.0, tilt_max - 10.0)
            cost += 2.0 * sat_pct / 100.0
            cost += 2.0 * slip_mean
            cost += 20.0 if phase_fell else 0.0
            cost_motion += cost
            n_motion += 1

        verdict = "FELL" if phase_fell else ("OK" if not short_window else "SHORT")
        if is_motion and not short_window and not phase_fell:
            over_limit = (err_lin > 0.05 or err_yaw > 0.05 or
                          p2p_lin > 0.05 or p2p_yaw > 0.05)
            if over_limit:
                verdict = "WARN"
                n_warn += 1
        print(f"{ph['phase'][:14]:<14} "
              f"({cmd[0]:5.2f},{cmd[1]:5.2f},{cmd[2]:5.2f}) "
              f"({mean_vx:5.2f},{mean_vy:5.2f},{mean_wz:5.2f}) "
              f"{err_lin:7.3f} {p2p_lin:7.3f} {std_lin:7.3f} {err_yaw:7.3f} "
              f"{tilt_max:6.1f} {bz_min:7.3f} {sat_pct:5.1f} {slip_mean:6.3f} "
              f"{cost:7.3f}  {verdict}")

        out["phases_out"].append({
            "phase": ph["phase"],
            "t_start": t0,
            "t_end": t1,
            "period": phase_period,
            "moving_avg_samples": phase_ma_n,
            "cmd": cmd.tolist(),
            "is_motion": is_motion,
            "mean_velocity_body": [mean_vx, mean_vy, mean_wz],
            "err_lin": err_lin,
            "err_yaw": err_yaw,
            "ma_p2p_lin": p2p_lin,
            "ma_std_lin": std_lin,
            "ma_p2p_yaw": p2p_yaw,
            "tilt_max_deg": tilt_max,
            "bz_min": bz_min,
            "sat_pct": sat_pct,
            "slip_mean_mps": slip_mean,
            "slip_p95_mps": slip_p95,
            "short_window": short_window,
            "cost": cost,
            "verdict": verdict,
        })
        out["max_tilt_deg"] = max(out["max_tilt_deg"], tilt_max)
        out["min_bz"] = min(out["min_bz"], bz_min)

    if hard_fail or n_motion == 0:
        out["cost"] = 1.0e6
        out["result"] = "FAIL"
    else:
        out["cost"] = cost_motion / n_motion
        out["result"] = "FELL" if fell else ("WARN" if n_warn > 0 else "OK")
    out["fell"] = bool(fell or hard_fail)
    if math.isinf(out["min_bz"]):
        out["min_bz"] = None
    print(f"\nsteady cost={out['cost']:.3f}, max tilt={out['max_tilt_deg']:.1f}°, "
          f"min bz={out['min_bz'] if out['min_bz'] is not None else 'n/a'}, "
          f"result={out['result']}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[steady] wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
