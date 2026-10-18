#!/usr/bin/env python3
"""S4 开环三段式参数搜索：低/中/高速段分别整定 T、duty、抬腿高度。

每个候选参数跑一次「该速度段的六方向单轴场景」（x 正/反、y 左/右、z 顺/逆），
用 analyze_s4_steady.py 只取稳态窗口，以速度误差 + 周期平均速度波动为主、
姿态/饱和/滑移为惩罚项，选最优。

用法：
    # 默认：低/中/高三段各做一轮坐标搜索（每段约 8–9 次仿真）
    python3 mpc_wbc/test/tune_s4_segments.py

    # 只调中速段，跑两轮
    python3 mpc_wbc/test/tune_s4_segments.py --segments mid --passes 2

    # 只看会跑哪些命令，不启动仿真
    python3 mpc_wbc/test/tune_s4_segments.py --dry-run

    # 完整网格（每段 27 次）
    python3 mpc_wbc/test/tune_s4_segments.py --full-grid --segments high

产物：build/tune_s4/<segment>/<trial>/...、results.csv、best.json
"""
import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "build/mpc_wbc/mpc_wbc_position_loop_sim"
SIM = ROOT / "interface/robot/simulation/mujoco_simulation.py"
ANALYZER = ROOT / "mpc_wbc/test/analyze_s4_steady.py"

SEGMENTS = {
    "low": {
        "env": "LOW",
        "commands": {
            "x_pos": (0.15, 0.0, 0.0),
            "x_neg": (-0.15, 0.0, 0.0),
            "y_pos": (0.0, 0.125, 0.0),
            "y_neg": (0.0, -0.125, 0.0),
            "z_ccw": (0.0, 0.0, 0.15),
            "z_cw": (0.0, 0.0, -0.15),
        },
        "nominal": {"T": 0.75, "d": 0.72, "H": 0.09},
        "candidates": {
            "T": [0.75, 0.80, 0.85, 0.90],
            "d": [0.68, 0.70, 0.72],
            "H": [0.08, 0.09, 0.10, 0.11],
        },
        "settle": 1.50,
    },
    "mid": {
        "env": "MID",
        "commands": {
            "x_pos": (0.30, 0.0, 0.0),
            "x_neg": (-0.30, 0.0, 0.0),
            "y_pos": (0.0, 0.20, 0.0),
            "y_neg": (0.0, -0.20, 0.0),
            "z_ccw": (0.0, 0.0, 0.25),
            "z_cw": (0.0, 0.0, -0.25),
        },
        "nominal": {"T": 0.63, "d": 0.70, "H": 0.09},
        "candidates": {
            "T": [0.58, 0.60, 0.63, 0.66],
            "d": [0.66, 0.68, 0.70],
            "H": [0.08, 0.09, 0.10],
        },
        "settle": 1.30,
    },
    "high": {
        "env": "HIGH",
        "commands": {
            "x_pos": (0.50, 0.0, 0.0),
            "x_neg": (-0.50, 0.0, 0.0),
            "y_pos": (0.0, 0.30, 0.0),
            "y_neg": (0.0, -0.30, 0.0),
            "z_ccw": (0.0, 0.0, 0.40),
            "z_cw": (0.0, 0.0, -0.40),
        },
        "nominal": {"T": 0.45, "d": 0.625, "H": 0.07},
        "candidates": {
            "T": [0.42, 0.45, 0.48, 0.50],
            "d": [0.60, 0.61, 0.625],
            "H": [0.07, 0.08, 0.09],
        },
        "settle": 1.20,
    },
}


def parse_candidates(text):
    """解析 T=0.55,0.60;d=0.68,0.70;H=0.09,0.10 形式的候选覆盖。"""
    out = {}
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        key, value = part.split("=", 1)
        key = key.strip()
        if key not in ("T", "d", "H"):
            raise ValueError(f"unknown candidate key: {key}")
        out[key] = [float(x) for x in value.split(",") if x.strip()]
    return out


def parse_nominal(text):
    """解析 T=0.60;d=0.70;H=0.10 形式的起点覆盖。"""
    out = {}
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        key, value = part.split("=", 1)
        key = key.strip()
        if key not in ("T", "d", "H"):
            raise ValueError(f"unknown nominal key: {key}")
        out[key] = float(value)
    return out


def port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def wait_ports_free(timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port_free(20001) and port_free(30010):
            return True
        time.sleep(0.5)
    return False


def write_scenario(path, commands):
    rows = []
    for name, cmd in commands.items():
        rows.append((name, 3.5, *cmd))
        rows.append((f"stop_{name}", 0.8, 0.0, 0.0, 0.0))
    rows.append(("final_hold", 1.0, 0.0, 0.0, 0.0))
    with open(path, "w") as f:
        f.write("phase,duration,vx,vy,wz\n")
        for name, dur, vx, vy, wz in rows:
            f.write(f"{name},{dur:.3f},{vx:.6f},{vy:.6f},{wz:.6f}\n")
    return sum(r[1] for r in rows)


class Tuner:
    def __init__(self, args):
        self.args = args
        self.out = Path(args.workdir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.results = []

    def scenario_for(self, segment):
        seg_dir = self.out / segment
        seg_dir.mkdir(parents=True, exist_ok=True)
        path = seg_dir / "tune_scenario.csv"
        duration = write_scenario(path, SEGMENTS[segment]["commands"])
        return path, duration

    def env_for(self, segment, T, d, H):
        env = os.environ.copy()
        key = SEGMENTS[segment]["env"]
        env.update({
            "MUJOCO_VIEWER": "0",
            "SIM_DEBUG": "0",
            "SIM_INIT_POSE": "stand",
            "S4_CMD_SCALE": "1.0",
            f"S4_{key}_PERIOD": f"{T:.6f}",
            f"S4_{key}_DUTY": f"{d:.6f}",
            f"S4_{key}_SWING_HEIGHT": f"{H:.6f}",
        })
        return env

    def run_trial(self, segment, T, d, H, trial_idx):
        cfg = SEGMENTS[segment]
        work = self.out / segment / f"trial_{trial_idx:03d}"
        work.mkdir(parents=True, exist_ok=True)
        scenario, scenario_sec = self.scenario_for(segment)
        sim_duration = 2.8 + scenario_sec + 2.0
        sim_log = work / "sim.csv"
        phases = work / "app_phases.csv"
        seg_log = work / "segments.csv"
        app_log = work / "app.txt"
        sim_log_txt = work / "sim_stdout.txt"
        metrics_json = work / "metrics.json"

        if self.args.dry_run:
            print(f"[dry-run] {segment}: T={T:.3f} d={d:.3f} H={H:.3f} "
                  f"sim={sim_duration:.1f}s scenario={scenario}")
            return None

        if not wait_ports_free():
            print(f"[tune] ports 20001/30010 busy, skip {segment}/{trial_idx}", file=sys.stderr)
            return None

        env = self.env_for(segment, T, d, H)
        env.update({
            "SIM_DURATION": f"{sim_duration:.1f}",
            "SIM_LOG": str(sim_log),
            "SIM_LOG_EVERY": "10",
        })
        sim_out = open(sim_log_txt, "w")
        sim = subprocess.Popen([sys.executable, str(SIM)], cwd=str(ROOT), env=env,
                               stdout=sim_out, stderr=subprocess.STDOUT)
        time.sleep(1.8)
        if sim.poll() is not None:
            sim_out.close()
            print(f"[tune] sim exited before controller start: {sim_log_txt}", file=sys.stderr)
            return None

        app_env = self.env_for(segment, T, d, H)
        app_env["S9_SCENARIO"] = str(scenario)
        app_env["S4_SEG_LOG"] = str(seg_log)
        try:
            with open(app_log, "w") as f:
                subprocess.run([str(APP), str(phases)], cwd=str(ROOT), env=app_env,
                               stdout=f, stderr=subprocess.STDOUT,
                               timeout=sim_duration * 3.0 + 20.0)
        except subprocess.TimeoutExpired:
            print(f"[tune] controller timeout: {app_log}", file=sys.stderr)
        finally:
            try:
                sim.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                sim.terminate()
                try:
                    sim.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    sim.kill()
                    sim.wait()
            sim_out.close()

        if not sim_log.exists() or not phases.exists():
            print(f"[tune] missing logs: {work}", file=sys.stderr)
            return None

        label = f"{segment}/T{T:.3f}_d{d:.3f}_H{H:.3f}"
        subprocess.run([sys.executable, str(ANALYZER), str(sim_log), str(phases),
                        "--prefix-rows", "3", "--settle", f"{cfg['settle']:.3f}",
                        "--period", f"{T:.6f}", "--seg-log", str(seg_log),
                        "--label", label, "--json", str(metrics_json)],
                       cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not metrics_json.exists():
            print(f"[tune] analyzer failed: {work}", file=sys.stderr)
            return None
        with open(metrics_json) as f:
            metrics = json.load(f)
        steady_cost = float(metrics.get("cost", float("inf")))
        freq_hz = 1.0 / max(1e-9, T)
        freq_penalty = self.args.freq_weight * freq_hz
        rec = {
            "segment": segment,
            "T": T, "d": d, "H": H,
            "cost": steady_cost + freq_penalty,
            "steady_cost": steady_cost,
            "freq_hz": freq_hz,
            "freq_penalty": freq_penalty,
            "fell": bool(metrics.get("fell", True)),
            "max_tilt_deg": metrics.get("max_tilt_deg"),
            "min_bz": metrics.get("min_bz"),
            "workdir": str(work),
        }
        self.results.append(rec)
        print(f"[tune] {segment} T={T:.3f} d={d:.3f} H={H:.3f} "
              f"f={freq_hz:.2f}Hz steady={steady_cost:.3f} "
              f"+freq={freq_penalty:.3f} -> cost={rec['cost']:.3f} "
              f"fell={rec['fell']}")
        return rec

    def evaluate(self, segment, cfg, cache):
        key = (round(cfg["T"], 6), round(cfg["d"], 6), round(cfg["H"], 6))
        if key in cache:
            return cache[key]
        rec = self.run_trial(segment, cfg["T"], cfg["d"], cfg["H"], len(self.results))
        cache[key] = rec
        return rec

    def coordinate_search(self, segment):
        cfg = SEGMENTS[segment]
        current = dict(cfg["nominal"])
        cache = {}
        if self.args.dry_run:
            for key in ("T", "d", "H"):
                for value in cfg["candidates"][key]:
                    cand = dict(current)
                    cand[key] = value
                    self.run_trial(segment, cand["T"], cand["d"], cand["H"], 0)
            return None
        best_rec = self.evaluate(segment, current, cache)
        if best_rec is None:
            return None
        for pass_idx in range(self.args.passes):
            improved = False
            for key in ("T", "d", "H"):
                best_val = current[key]
                best_cost = best_rec["cost"]
                for value in cfg["candidates"][key]:
                    if abs(value - current[key]) < 1e-12:
                        continue
                    cand = dict(current)
                    cand[key] = value
                    rec = self.evaluate(segment, cand, cache)
                    if rec is not None and not rec["fell"] and rec["cost"] < best_cost:
                        best_val = value
                        best_cost = rec["cost"]
                        best_rec = rec
                        improved = True
                current[key] = best_val
            if not improved:
                break
        return current, best_rec

    def full_grid(self, segment):
        cfg = SEGMENTS[segment]
        cache = {}
        best = None
        for T in cfg["candidates"]["T"]:
            for d in cfg["candidates"]["d"]:
                for H in cfg["candidates"]["H"]:
                    cand = {"T": T, "d": d, "H": H}
                    rec = self.evaluate(segment, cand, cache)
                    if rec is not None and (best is None or rec["cost"] < best["cost"]):
                        if not rec["fell"]:
                            best = rec
        return best

    def save(self):
        results_csv = self.out / "results.csv"
        with open(results_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["segment", "T", "d", "H", "cost", "steady_cost",
                                              "freq_hz", "freq_penalty", "fell",
                                              "max_tilt_deg", "min_bz", "workdir"])
            w.writeheader()
            for r in self.results:
                w.writerow(r)
        best = {}
        for segment in SEGMENTS:
            recs = [r for r in self.results if r["segment"] == segment]
            if recs:
                best[segment] = min(recs, key=lambda r: (r["fell"], r["cost"]))
        with open(self.out / "best.json", "w") as f:
            json.dump(best, f, indent=2)
        print(f"\n[tune] wrote {results_csv} and {self.out / 'best.json'}")
        for segment, rec in best.items():
            key = SEGMENTS[segment]["env"]
            print(f"  {segment}: T={rec['T']:.3f} d={rec['d']:.3f} H={rec['H']:.3f} "
                  f"f={rec['freq_hz']:.2f}Hz steady={rec['steady_cost']:.3f} "
                  f"cost={rec['cost']:.3f}")
            print(f"    S4_{key}_PERIOD={rec['T']:.3f} S4_{key}_DUTY={rec['d']:.3f} "
                  f"S4_{key}_SWING_HEIGHT={rec['H']:.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--segments", default="all",
                    help="all 或逗号分隔的 low,mid,high")
    ap.add_argument("--passes", type=int, default=1,
                    help="坐标搜索轮数（默认 1）")
    ap.add_argument("--full-grid", action="store_true",
                    help="每段跑完整网格（更慢）")
    ap.add_argument("--candidates", default=None,
                    help="覆盖候选，如 T=0.55,0.60;d=0.68,0.70;H=0.09,0.10")
    ap.add_argument("--nominal", default=None,
                    help="覆盖起点，如 T=0.60;d=0.70;H=0.10")
    ap.add_argument("--freq-weight", type=float, default=0.03,
                    help="步频惩罚权重：cost += weight/T；0 表示只看稳态误差")
    ap.add_argument("--workdir", default=str(ROOT / "build/tune_s4"))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not args.dry_run and not APP.exists():
        print(f"[tune] missing executable: {APP}\n"
              f"  cmake -S . -B build -DBUILD_MPC_WBC_SIM=ON && "
              f"cmake --build build --target mpc_wbc_position_loop_sim -j4", file=sys.stderr)
        return 2

    names = list(SEGMENTS) if args.segments == "all" else [s.strip() for s in args.segments.split(",")]
    bad = [s for s in names if s not in SEGMENTS]
    if bad:
        print(f"[tune] unknown segment(s): {bad}", file=sys.stderr)
        return 2

    try:
        if args.candidates:
            cand = parse_candidates(args.candidates)
            for segment in names:
                SEGMENTS[segment]["candidates"].update(cand)
        if args.nominal:
            nom = parse_nominal(args.nominal)
            for segment in names:
                SEGMENTS[segment]["nominal"].update(nom)
    except ValueError as e:
        print(f"[tune] argument error: {e}", file=sys.stderr)
        return 2

    tuner = Tuner(args)
    for segment in names:
        print(f"\n=== tuning {segment} ===")
        if args.full_grid:
            best = tuner.full_grid(segment)
            print(f"[tune] {segment} best: {best}")
        else:
            out = tuner.coordinate_search(segment)
            if out:
                cfg, rec = out
                print(f"[tune] {segment} best: T={cfg['T']:.3f} d={cfg['d']:.3f} "
                      f"H={cfg['H']:.3f} cost={rec['cost']:.3f}")
    tuner.save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
