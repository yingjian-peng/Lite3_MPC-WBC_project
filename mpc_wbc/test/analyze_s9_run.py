#!/usr/bin/env python3
"""S9 单次试验的统一分析器（S4 位置环基线与 S8 MPC+WBC 通用）。

用法：
    analyze_s9_run.py <sim_log.csv> <phases.csv> [--diag <diag.csv>]
                      [--prefix-rows N] [--label NAME] [--json out.json]
                      [--margin 0.4] [--foot-radius 0.022] [--tau-max 30]

口径（两条路径完全一致）：
  * 只分析 phases.csv 中第 `--prefix-rows` 行之后的**运动段**（前面的成段是起立/静置，脚本按行号
    跳过，不按名字匹配——S4 与 S8 的起立段名字不同、时长也不同）。
  * 逐段指标在「命令帧」里算：把世界位移/速度按段起始偏航旋转到机体系，再与段命令 (vx,vy,wz) 比对，
    因此不依赖起始航向（`SIM_INIT_YAW` 场景同样适用）。
  * 每段取 [t_start+margin, t_end] 作为稳态窗（跳开命令切换瞬态），但跌倒判定看整段。
  * 足端滑移用几何判据（足心 z < 足半径 + 4 mm）而不是接触力：名义站姿下 MuJoCo 里
    `*_SHANK_collision` 也压在地面上，只看足端接触力会漏掉承重腿。
退出码：0 = 全部段通过，1 = 有段失败。
"""
import argparse
import csv
import json
import math
import sys

import numpy as np

FOOT_RADIUS_DEFAULT = 0.022  # m（MJCF 球足）
TILT_FALL_DEG = 25.0
BZ_FALL = 0.15


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, k, default=0.0):
    return np.array([float(r.get(k, default) or 0.0) for r in rows])


def euler_yaw(rows):
    w, x, y, z = col(rows, "qw"), col(rows, "qx"), col(rows, "qy"), col(rows, "qz")
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def wrap_index(t, a, b):
    return np.where((t >= a) & (t <= b))[0]


def slip_rate(t, px, py, pz, foot_radius, clearance=0.004):
    """接触足（几何判据：足心 z < 足半径 + clearance）的水平位移速率均值/95 分位 [m/s]。"""
    rates = []
    contact = pz < foot_radius + clearance
    idx = np.where(contact[:-1] & contact[1:])[0]
    if idx.size:
        dt = t[idx + 1] - t[idx]
        good = dt > 1e-9
        rates.append(np.hypot(px[idx + 1] - px[idx], py[idx + 1] - py[idx])[good] / dt[good])
    if not rates:
        return 0.0, 0.0
    allr = np.concatenate(rates)
    return float(allr.mean()), float(np.percentile(allr, 95))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim_log")
    ap.add_argument("phases")
    ap.add_argument("--diag", default=None)
    ap.add_argument("--prefix-rows", type=int, default=3)
    ap.add_argument("--label", default="")
    ap.add_argument("--json", default=None)
    ap.add_argument("--margin", type=float, default=0.4)
    ap.add_argument("--foot-radius", type=float, default=FOOT_RADIUS_DEFAULT)
    ap.add_argument("--tau-max", type=float, default=30.0)
    args = ap.parse_args()

    sim = load(args.sim_log)
    phases = load(args.phases)
    t = col(sim, "t")
    bx, by, bz = col(sim, "bx"), col(sim, "by"), col(sim, "bz")
    roll, pitch, yaw = euler_yaw(sim)
    tilt = np.degrees(np.maximum(np.abs(roll), np.abs(pitch)))
    vx, vy, vz = col(sim, "vx"), col(sim, "vy"), col(sim, "vz")
    wx, wy, wz = col(sim, "wx"), col(sim, "wy"), col(sim, "wz")
    tau_act = np.zeros((len(sim), 12))
    for i in range(12):
        if f"atau{i}" in sim[0]:
            tau_act[:, i] = col(sim, f"atau{i}")
    tau_abs = np.abs(tau_act).max(axis=1) if tau_act.any() else np.zeros(len(sim))
    fzall = col(sim, "fzall") if "fzall" in sim[0] else np.full(len(sim), np.nan)

    diag = None
    if args.diag:
        try:
            diag = load(args.diag)
        except FileNotFoundError:
            print(f"[warn] diag not found: {args.diag}")

    out = {
        "label": args.label,
        "sim_log": args.sim_log,
        "phases": args.phases,
        "n_samples": len(sim),
        "duration": float(t[-1]) if len(t) else 0.0,
        "phases_out": [],
        "result": "PASS",
    }
    print(f"=== S9 run: {args.label or args.sim_log} ({len(sim)} samples, {out['duration']:.2f} s) ===")
    hdr = (f"{'phase':<14} {'dt':>5} {'along':>7} {'lateral':>8} {'dyaw°':>7} {'v_cmd':>6} "
           f"{'v_meas':>6} {'v_err':>6} {'tilt°':>6} {'bz_min':>7} {'sat%':>5} {'slip':>6}")
    print(hdr)

    failures = 0
    for i, ph in enumerate(phases):
        if i < args.prefix_rows:
            continue
        t0, t1 = float(ph["t_start"]), float(ph["t_end"])
        cmd = np.array([float(ph["vx"]), float(ph["vy"]), float(ph["wz"])])
        idx_all = wrap_index(t, t0, t1)
        idx = wrap_index(t, t0 + args.margin, t1)
        short_window = idx.size < 2 <= idx_all.size
        if short_window:
            idx = idx_all  # 段太短（例如本段刚开始就跌倒）：退回整段，并标注
        if idx.size < 2:
            print(f"{ph['phase'][:14]:<14} no samples in this phase (run ended early)")
            out["phases_out"].append({
                "phase": ph["phase"], "t_start": t0, "t_end": t1, "cmd": cmd.tolist(),
                "along_m": None, "lateral_m": None, "dyaw_deg": None, "v_cmd_norm": float(np.linalg.norm(cmd[:2])),
                "v_meas_norm": None, "v_err": None, "tilt_max_deg": None, "bz_min": None,
                "sat_pct": None, "slip_mean_mps": None, "slip_p95_mps": None, "fzall_mean": None,
                "verdict": "FAIL", "problems": ["no samples (run ended early)"],
            })
            failures += 1
            continue
        yaw0 = float(np.mean(yaw[idx_all]))  # 用整段平均偏航作为命令帧
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = bx[idx][-1] - bx[idx][0], by[idx][-1] - by[idx][0]
        along = c * dx + s * dy            # 命令帧 x（前向）
        lateral = -s * dx + c * dy         # 命令帧 y（左向）
        dyaw = yaw[idx][-1] - yaw[idx][0]
        v_along = c * vx[idx] + s * vy[idx]
        v_lat = -s * vx[idx] + c * vy[idx]
        v_meas = np.array([float(np.mean(v_along)), float(np.mean(v_lat))])
        v_cmd = cmd[:2]
        v_err = float(np.linalg.norm(v_meas - v_cmd))
        tilt_max = float(tilt[idx_all].max())
        bz_min = float(bz[idx_all].min())
        sat_pct = 100.0 * float(np.mean(tau_abs[idx] >= 0.95 * args.tau_max))
        slip_rates = [slip_rate(t[idx_all],
                               col(sim, f"px{leg}")[idx_all],
                               col(sim, f"py{leg}")[idx_all],
                               col(sim, f"pz{leg}")[idx_all],
                               args.foot_radius) for leg in range(4)]
        slip_mean = float(np.mean([r[0] for r in slip_rates]))
        slip_p95 = float(np.max([r[1] for r in slip_rates]))
        fell = tilt_max > TILT_FALL_DEG or bz_min < BZ_FALL

        # 判定：跌倒 / 方向 / 跟踪 / 饱和
        problems = []
        if fell:
            problems.append(f"fell(tilt={tilt_max:.0f}°,bz={bz_min:.2f})")
        moving = abs(v_cmd[0]) > 1e-6 or abs(v_cmd[1]) > 1e-6
        turning = abs(cmd[2]) > 1e-6
        if moving:
            expect = np.sign(v_cmd)
            if abs(v_cmd[0]) > 1e-6 and math.copysign(1, along) != (1 if v_cmd[0] > 0 else -1):
                problems.append("fwd/back sign")
            elif abs(v_cmd[1]) > 1e-6 and math.copysign(1, lateral) != (1 if v_cmd[1] > 0 else -1):
                problems.append("lateral sign")
            tol = max(0.08, 0.4 * float(np.linalg.norm(v_cmd)))
            if v_err > tol:
                problems.append(f"speed err {v_err:.2f}>{tol:.2f}")
        elif turning:
            if abs(dyaw) < math.radians(5.0):
                problems.append("no yaw motion")
            elif math.copysign(1, dyaw) != (1 if cmd[2] > 0 else -1):
                problems.append("yaw sign")
            tol = max(0.10, 0.4 * abs(cmd[2]))
            if abs(float(np.mean(wz[idx])) - cmd[2]) > tol:
                problems.append(f"yaw rate err {abs(float(np.mean(wz[idx])) - cmd[2]):.2f}")
        else:
            # 停步段：后 1/3 速度应衰减
            if idx.size > 9:
                tail = idx[int(idx.size * 2 / 3):]
                v_tail = float(np.mean(np.hypot(vx[tail], vy[tail])))
                if v_tail > 0.12:
                    problems.append(f"not stopping (|v|={v_tail:.2f})")
        if sat_pct > 10.0:
            problems.append(f"torque sat {sat_pct:.1f}%")
        if short_window:
            problems.append("short window (段内数据不足 0.4s)")

        verdict = "OK" if not problems else "FAIL"
        if problems:
            failures += 1
        print(f"{ph['phase'][:14]:<14} {t1 - t0:5.2f} {along:7.3f} {lateral:8.3f} "
              f"{math.degrees(dyaw):7.1f} {float(np.linalg.norm(v_cmd)):6.2f} "
              f"{float(np.linalg.norm(v_meas)):6.2f} {v_err:6.2f} {tilt_max:6.1f} {bz_min:7.3f} "
              f"{sat_pct:5.1f} {slip_mean:6.3f}  {verdict}"
              + (f"  [{' ; '.join(problems)}]" if problems else ""))

        rec = {
            "phase": ph["phase"], "t_start": t0, "t_end": t1,
            "cmd": cmd.tolist(),
            "along_m": along, "lateral_m": lateral, "dyaw_deg": math.degrees(dyaw),
            "v_cmd_norm": float(np.linalg.norm(v_cmd)), "v_meas_norm": float(np.linalg.norm(v_meas)),
            "v_err": v_err, "tilt_max_deg": tilt_max, "bz_min": bz_min,
            "sat_pct": sat_pct, "slip_mean_mps": slip_mean, "slip_p95_mps": slip_p95, "short_window": short_window,
            "fzall_mean": float(np.nanmean(fzall[idx_all])) if not np.isnan(fzall[idx_all]).all() else None,
            "verdict": verdict, "problems": problems,
        }
        if diag is not None:
            dt_all = col(diag, "t")
            di = wrap_index(dt_all, t0, t1)
            if di.size > 1:
                step_ms = col(diag, "step_ms")[di]
                rec["diag"] = {
                    "step_ms_mean": float(step_ms.mean()),
                    "step_ms_p99": float(np.percentile(step_ms, 99)),
                    "step_ms_max": float(step_ms.max()),
                    "frames_over_1ms": int((step_ms > 1.0).sum()),
                    "wbc_active_pct": 100.0 * float(np.mean(col(diag, "wbc")[di])),
                    "mpc_ms_mean": float(col(diag, "mpc_ms")[di].mean()),
                    "mpc_ms_max": float(col(diag, "mpc_ms")[di].max()),
                    "mpc_iters_max": int(col(diag, "mpc_iters")[di].max()),
                    "mpc_a_des_zero_pct": 100.0 * float(np.mean(col(diag, "mpc_status")[di] == -1)),
                    "mpc_held_pct": 100.0 * float(np.mean(col(diag, "mpc_held")[di]))
                    if "mpc_held" in diag[0] else 0.0,
                    "mpc_reason_pct": {int(r): round(100.0 * float(np.mean(col(diag, "mpc_reason")[di] == r)), 1)
                                       for r in range(5)} if "mpc_reason" in diag[0] else {},
                    "num_support_min": int(col(diag, "num_support")[di].min()),
                }
                if "mpc_reason" in diag[0]:
                    rs = col(diag, "mpc_reason")[di].astype(int)
                    held = 100.0 * float(np.mean(col(diag, "mpc_held")[di]))
                    zero = 100.0 * float(np.mean(col(diag, "mpc_status")[di] == -1))
                    reasons = " ".join(
                        f"r{r}={100.0 * float(np.mean(rs == r)):.0f}%"
                        for r in range(1, 5) if np.any(rs == r))
                    print(f"{'':14} mpc: 沿用={held:.1f}% 清零={zero:.1f}% {reasons}"
                          f" | 未激活={100.0 - float(np.mean(col(diag, 'wbc')[di])) * 100.0:.1f}%")
        out["phases_out"].append(rec)

    # 全局：每次跌倒的最早时刻（只看运动段；若场景本身只有静立段，则看最后一段）
    first_motion_row = min(args.prefix_rows, len(phases) - 1) if phases else 0
    after = t >= float(phases[first_motion_row]["t_start"]) if phases else np.ones(len(t), bool)
    # 起立/接管段（phases[1] 起，settle 的自由落体不算）的姿态跌倒单独记：两条路径的起立方式不同，
    # 混进运动段会把"没站起来"和"走不稳"混为一谈。
    motion_start = float(phases[args.prefix_rows]["t_start"]) if len(phases) > args.prefix_rows else None
    if motion_start is not None:
        pre = (t >= float(phases[1]["t_start"])) & (t < motion_start)
    else:
        # 运动段根本没开始（多半是起立就没站住）：整段都算起立段
        pre = t >= float(phases[1]["t_start"]) if len(phases) > 1 else np.zeros(len(t), bool)
    if pre.any():
        out["prefix_min_bz"] = float(bz[pre].min())
        out["prefix_max_tilt_deg"] = float(tilt[pre].max())
        # 注意：起立段本来就是从趴地抬起来（bz≈0.05），所以不能用 bz 判异常；
        # 只有「姿态翻掉」或「运动段根本没开始」才算起立段出问题。
        out["prefix_abnormal"] = bool(out["prefix_max_tilt_deg"] > TILT_FALL_DEG or
                                      motion_start is None)
    else:
        out["prefix_min_bz"] = None
        out["prefix_max_tilt_deg"] = None
        out["prefix_abnormal"] = False
    fall_idx = np.where(after & ((tilt > TILT_FALL_DEG) | (bz < BZ_FALL)))[0]
    out["fell_at"] = float(t[fall_idx[0]]) if fall_idx.size else None
    out["max_tilt_deg"] = float(tilt[after].max()) if after.any() else 0.0
    out["min_bz"] = float(bz[after].min()) if after.any() else 0.0
    if out["fell_at"] is not None:
        out["result"] = "FAIL"
        failures += 1
    print(f"\nmax tilt={out['max_tilt_deg']:.1f}°, min bz={out['min_bz']:.3f}"
          + (f", first fall at t={out['fell_at']:.2f} s" if out["fell_at"] is not None else "")
          + (f", 起立段异常 (tilt_max={out['prefix_max_tilt_deg']:.1f}°, "
             f"bz_min={out['prefix_min_bz']:.3f})" if out.get("prefix_abnormal") else ""))
    print("RESULT:", out["result"] if failures == 0 else "FAIL")
    out["result"] = "PASS" if failures == 0 else "FAIL"

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[s9] wrote {args.json}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
