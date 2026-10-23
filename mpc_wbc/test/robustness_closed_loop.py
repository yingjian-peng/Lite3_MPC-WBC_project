#!/usr/bin/env python3
"""闭环（MPC+WBC）鲁棒性测试套件。

复用开环基线的场景 CSV（scenarios/robust_*.csv）与仿真侧扰动注入（SIM_PUSH_* / SIM_SLOPE_DEG），
但驱动的是 S8 的 MPC+WBC 控制器（apps/mpc_wbc_sim.cpp），而不是 S4 的位置环。

用法：
    python mpc_wbc/test/robustness_closed_loop.py --group push
    python mpc_wbc/test/robustness_closed_loop.py --group slope
    python mpc_wbc/test/robustness_closed_loop.py --group step
    python mpc_wbc/test/robustness_closed_loop.py --group all --outdir build/r9/robust

指标口径与开环基线一致（docs/05 §"鲁棒性"）：
  peak_tilt   扰动窗口内（t0 .. t0+PUSH_WIN）的最大倾角
  recovery    从 t0 起，倾角重新降回 RECOVER_TILT 以下并保持 0.3 s 所需的时间
  residual    扰动前后航向残差 |yaw_end − yaw_before|
  PASS        峰值 < PEAK_LIMIT 且未触发控制器故障（rc==0）
"""

import argparse
import csv
import math
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNNER = os.path.join(REPO, "mpc_wbc", "test", "run_s8_case.py")
SC = os.path.join(REPO, "mpc_wbc", "test", "scenarios")

# 相位表的公共前缀时长：settle 0.8 + fold 1.5 + raise 1.5 + hold **0.5**（场景模式下
# hold 被覆盖成 0.5 s，实测 log 为 "t=4.30 enter phase '<第一个场景段>'"）。
# 注意内置 walk 曲线的 hold 是 1.0 s，所以它从 4.8 开始 —— 两者不同。
T_PHASES = 4.3
PUSH_WIN = 3.0       # 扰动后观察窗
RECOVER_TILT = 2.0   # （保留）绝对阀值，仅供参考
# 恢复判据必须相对「扰前基线」而不是固定角度：以 0.30 m/s 行走时倾角 p95 已接近 3°，
# 用固定 2° 会把正常行走误判成「未恢复」。
RECOVER_MARGIN = 0.5 # 回到 基线p95 + 0.5° 以内算恢复 [deg]
RECOVER_HOLD = 0.3   # 需保持的时长 [s]
PEAK_LIMIT = 15.0    # 峰值倾角判据 [deg]


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, ValueError):
            out.append(float("nan"))
    return out


def yaw_of(r):
    """从四元数取 yaw（sim.csv 有 qw,qx,qy,qz）。"""
    try:
        w, x, y, z = (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"]))
    except (KeyError, ValueError):
        return 0.0
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_series(outdir):
    """优先用控制器侧的 tilt_deg（与故障判据同源），缺失时用 sim.csv 的四元数回算。"""
    d = load(os.path.join(outdir, "diag.csv"))
    if d and d[0].get("tilt_deg") not in (None, ""):
        return [(float(r["t"]), float(r["tilt_deg"])) for r in d]
    s = load(os.path.join(outdir, "sim.csv"))
    out = []
    for r in s:
        w, x, y, z = (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"]))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sp)
        out.append((float(r["t"]), math.degrees(max(abs(roll), abs(pitch)))))
    return out


def push_metrics(outdir, t0, rc):
    ser = tilt_series(outdir)
    if not ser:
        return None
    win = [(t, v) for t, v in ser if t0 <= t <= t0 + PUSH_WIN]
    if not win:
        return None
    peak = max(v for _, v in win)
    # 扰前基线：t0 之前 1.5 s 的倾角 p95
    pre = sorted(v for t, v in ser if t0 - 1.5 <= t < t0)
    base = pre[int(0.95 * (len(pre) - 1))] if pre else 0.0
    thresh = base + RECOVER_MARGIN
    # 不做「倾角回落时刻」这类的恢复判据：行走本身就会周期性超阈值，那样只能量到噪声。
    # 改用**速度跟踪恢复**：扰动后 2.0–3.0 s 窗内的平均 vx 应当回到指令附近。
    # 这比「倾角回落」更能说明“扰动后还在正常行走”。
    rec = float("nan")
    s_tmp = load(os.path.join(outdir, "sim.csv"))
    late_vx = [float(r["vx"]) for r in s_tmp
               if t0 + 2.0 <= float(r["t"]) <= t0 + 3.0]
    if late_vx:
        rec = sum(late_vx) / len(late_vx)
    s = load(os.path.join(outdir, "sim.csv"))
    yaw0 = yaw_end = 0.0
    for r in s:
        t = float(r["t"])
        if t0 - 0.3 <= t <= t0:
            yaw0 = yaw_of(r)
        if t >= t0:
            yaw_end = yaw_of(r)
    dy = math.degrees(yaw_end - yaw0)
    dy = (dy + 180.0) % 360.0 - 180.0
    ok = (peak < PEAK_LIMIT) and (rc == 0) and (abs(dy) < 5.0) and (abs(rec - 0.30) < 0.10)
    return dict(peak=peak, base=base, rec=rec, dyaw=dy, rc=rc, ok=ok)


def run_case(name, outdir, scenario, sim_env=None, env=None, push_t0=None):
    argv = [sys.executable, RUNNER, "--outdir", outdir, "--phases", scenario]
    dur = None
    for f in load(scenario):
        dur = (dur or 0.0) + float(f["duration"])
    if dur:
        argv += ["--duration", f"{T_PHASES + dur + 3.0:.1f}"]
    for kv in sim_env or []:
        argv += ["--sim-env", kv]
    for kv in env or []:
        argv += ["--env", kv]
    print(f"[{name}] run ...", flush=True)
    p = subprocess.run(argv, capture_output=True, text=True)
    rc = p.returncode
    tail = [ln for ln in p.stdout.strip().splitlines() if ln.strip()]
    print(f"[{name}] rc={rc} {tail[-1] if tail else ''}", flush=True)
    if rc not in (0,) and push_t0 is not None:
        pass
    return rc


def group_push(a):
    cases = [
        ("push_y+30", "robust_push.csv", ["SIM_PUSH_FY=30"], 12.0),
        ("push_y-30", "robust_push.csv", ["SIM_PUSH_FY=-30"], 12.0),
        ("push_y+45", "robust_push.csv", ["SIM_PUSH_FY=45"], 12.0),
        ("push_x+30", "robust_push.csv", ["SIM_PUSH_FX=30"], 12.0),
        ("push_x-30", "robust_push.csv", ["SIM_PUSH_FX=-30"], 12.0),
    ]
    rows = []
    for name, sc, senv, t0 in cases:
        out = os.path.join(a.outdir, name)
        rc = run_case(name, out, os.path.join(SC, sc),
                      sim_env=senv + [f"SIM_PUSH_T0={t0}", "SIM_PUSH_DUR=0.15"], push_t0=t0)
        m = push_metrics(out, t0, rc)
        if m is None:
            print(f"[{name}] 无数据")
            continue
        rec = "  nan" if m["rec"] != m["rec"] else f"{m['rec']:5.2f}"
        print(f"  峰值倾角={m['peak']:5.2f}° (行走基线 p95={m['base']:4.2f}°, "
              f"超基线={m['peak'] - m['base']:4.2f}°)  "
              f"恢复后 vx={rec} m/s  ​航向残差={m['dyaw']:+5.2f}°  "
              f"{'PASS' if m['ok'] else 'FAIL'}")
        rows.append((name, m))
    return rows


def group_slope(a):
    cases = [("slope2_up_slow", 2.0), ("slope2_up_mid", 2.0), ("slope2_neg", -2.0)]
    rows = []
    for name, deg in cases:
        out = os.path.join(a.outdir, name)
        rc = run_case(name, out, os.path.join(SC, "robust_slope.csv"),
                      sim_env=[f"SIM_SLOPE_DEG={deg}"])
        s = load(os.path.join(out, "sim.csv"))
        d = load(os.path.join(out, "diag.csv"))
        if not s:
            print(f"[{name}] 无数据")
            continue
        bz = col(s, "bz")
        tilt = col(d, "tilt_deg") or [0.0]
        tilts = sorted(v for v in tilt if v == v)
        print(f"  倾角 p50/max={tilts[len(tilts) // 2]:5.2f}/{tilts[-1]:5.2f}°  "
              f"高度首末={bz[0]:.4f}->{bz[-1]:.4f} m  {'PASS' if rc == 0 else 'FAIL'}")
        rows.append((name, rc))
    return rows


def group_step(a):
    out = os.path.join(a.outdir, "step")
    rc = run_case("step_020_050_x10", out, os.path.join(SC, "robust_step.csv"))
    s = load(os.path.join(out, "sim.csv"))
    print(f"  10 段 0.20↔0.50 阶跃：{'PASS' if rc == 0 else 'FAIL'}")
    if s:
        vx = col(s, "vx")
        print(f"  vx 范围 {min(vx):+.3f} .. {max(vx):+.3f} m/s")
    return [("step", rc)]


def group_drift(a):
    out = os.path.join(a.outdir, "drift")
    rc = run_case("drift_5min", out, os.path.join(SC, "robust_drift.csv"))
    s = load(os.path.join(out, "sim.csv"))
    d = load(os.path.join(out, "diag.csv"))
    if d:
        tilt = [v for v in col(d, "tilt_deg") if v == v]
        tilt.sort()
        print(f"  倾角 p50/p99/max={tilt[len(tilt) // 2]:5.2f}/{tilt[int(0.99 * len(tilt))]:5.2f}/"
              f"{tilt[-1]:5.2f}°")
    if s:
        y0 = yaw_of(s[0])
        y1 = yaw_of(s[-1])
        dy = math.degrees(y1 - y0)
        dy = (dy + 180.0) % 360.0 - 180.0
        print(f"  航向漂移={dy:+.3f}°  sim 时长={float(s[-1]['t']):.1f}s")
    return [("drift", rc)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="push",
                    choices=["push", "slope", "step", "drift", "all"])
    ap.add_argument("--outdir", default=os.path.join(REPO, "build/r9/robust"))
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    groups = ["push", "slope", "step", "drift"] if a.group == "all" else [a.group]
    for g in groups:
        print(f"\n===== {g} =====", flush=True)
        {"push": group_push, "slope": group_slope,
         "step": group_step, "drift": group_drift}[g](a)


if __name__ == "__main__":
    main()
