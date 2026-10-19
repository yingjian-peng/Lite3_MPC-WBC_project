#!/usr/bin/env python3
"""Run the active open-loop baseline; preserve configuration, logs and verdicts."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run(scenario, dest, args):
    with scenario.open() as stream:
        rows = list(csv.DictReader(stream))
    duration = 2.8 + sum(float(row["duration"]) for row in rows) + 5
    (dest / "scenario.csv").write_bytes(scenario.read_bytes())
    env = dict(os.environ, MUJOCO_VIEWER=str(int(args.viewer)), SIM_DEBUG="0",
               SIM_DURATION=str(duration), SIM_LOG=str(dest / "sim.csv"),
               SIM_LOG_EVERY="10", SIM_INIT_POSE="stand", SIM_INIT_YAW=str(args.init_yaw),
               PYTHONUNBUFFERED="1")
    controller_env = dict(env, S9_SCENARIO=str(dest / "scenario.csv"),
                          S4_SEG_LOG=str(dest / "segments.csv"))
    (dest / "environment.json").write_text(json.dumps(
        {k: v for k, v in controller_env.items() if k.startswith(("S4_", "SIM_", "S9_", "MUJOCO_"))}, indent=2))
    sim = controller = None
    print(f"start {dest.name}: {duration:.1f}s simulation budget", flush=True)
    try:
        with (dest / "sim.txt").open("w") as simlog, (dest / "app.txt").open("w") as applog:
            sim = subprocess.Popen([sys.executable, "-u", "interface/robot/simulation/mujoco_simulation.py"],
                                   env=env, stdout=simlog, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while "[INFO] logging to" not in (dest / "sim.txt").read_text():
                if sim.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"simulation startup failed; see {dest / 'sim.txt'}")
                time.sleep(0.02)
            controller = subprocess.Popen(["build/mpc_wbc/mpc_wbc_position_loop_sim", str(dest / "phases.csv")],
                                          env=controller_env, stdout=applog, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + duration * 4 + 30
            while controller.poll() is None:
                if sim.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"simulation exited or controller timed out; see {dest}")
                time.sleep(0.1)
            if controller.returncode:
                raise RuntimeError(f"controller exited {controller.returncode}; see {dest / 'app.txt'}")
    finally:
        stop(controller)
        stop(sim)
    analysis = subprocess.run([sys.executable, "mpc_wbc/test/analyze_s4_steady.py",
                               str(dest / "sim.csv"), str(dest / "phases.csv"),
                               "--settle", "2", "--seg-log", str(dest / "segments.csv"),
                               "--label", dest.name, "--json", str(dest / "report.json")],
                              capture_output=True, text=True)
    (dest / "analysis.txt").write_text(analysis.stdout + analysis.stderr)
    analysis.check_returncode()
    report = json.loads((dest / "report.json").read_text())
    motion = [p for p in report["phases_out"] if p["is_motion"]]
    expected_motion = sum(any(abs(float(r[k])) > 0 for k in ("vx", "vy", "wz")) for r in rows)
    if len(motion) != expected_motion or any(p["verdict"] in ("NO_SAMPLES", "SHORT") for p in motion):
        raise RuntimeError("incomplete motion log")
    ok = sum(p["verdict"] == "OK" for p in motion)
    print(f"complete {dest.name}: {ok}/{len(motion)} OK, result={report['result']}, "
          f"max tilt={report['max_tilt_deg']:.2f}deg", flush=True)
    return {"run": dest.name, "ok": ok, "motion_count": len(motion), "result": report["result"],
            "max_tilt_deg": report["max_tilt_deg"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scenario", type=Path)
    group.add_argument("--suite", choices=["single10s"])
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--init-yaw", type=float, default=0)
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    os.chdir(ROOT)
    dest = args.out.resolve()
    if dest.exists() and any(dest.iterdir()):
        parser.error("--out must be a new or empty directory")
    dest.mkdir(parents=True, exist_ok=True)
    if not (ROOT / "build/mpc_wbc/mpc_wbc_position_loop_sim").is_file():
        parser.error("build mpc_wbc_position_loop_sim first")
    diff = subprocess.check_output(["git", "diff", "HEAD", "--binary"])
    (dest / "source.patch").write_bytes(diff)
    untracked = subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard", "-z"]).split(b"\0")
    untracked_hashes = {os.fsdecode(p): hashlib.sha256((ROOT / os.fsdecode(p)).read_bytes()).hexdigest()
                        for p in untracked if p and (ROOT / os.fsdecode(p)).is_file()}
    (dest / "metadata.json").write_text(json.dumps({
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "tracked_diff_sha256": hashlib.sha256(diff).hexdigest(), "untracked_sha256": untracked_hashes,
        "binary_sha256": hashlib.sha256((ROOT / "build/mpc_wbc/mpc_wbc_position_loop_sim").read_bytes()).hexdigest(),
        "python": sys.executable, "suite": args.suite, "repeat": args.repeat,
        "viewer": args.viewer, "init_yaw": args.init_yaw}, indent=2))
    scenarios = []
    if args.suite:
        for level in ("low", "mid", "high"):
            source = ROOT / f"mpc_wbc/test/scenarios/verify_single_{level}.csv"
            with source.open() as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                if not row["phase"].startswith("stop_"):
                    row["duration"] = "10.0"
            target = dest / f"{level}_10s.csv"
            with target.open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            scenarios.append((level, target))
    else:
        scenarios = [(args.scenario.stem, args.scenario.resolve())]
    results = []
    for repeat in range(1, args.repeat + 1):
        for level, scenario in scenarios:
            run_dir = dest / f"r{repeat:02d}_{level}"
            run_dir.mkdir()
            results.append(run(scenario, run_dir, args))
            (dest / "summary.json").write_text(json.dumps(results, indent=2))
    lines = ["# 开环基础改造回归", "", "每个运动段 10 s（single10s 场景），段间停 2 s；稳态跳过前 2 s。", "",
             "| 运行 | 达标 | 最大倾角 | 结果 |", "| --- | --- | --- | --- |"]
    for result in results:
        lines.append(f"| {result['run']} | {result['ok']}/{result['motion_count']} | "
                     f"{result['max_tilt_deg']:.2f}° | {result['result']} |")
    (dest / "report.md").write_text("\n".join(lines) + "\n")
    return 0 if all(r["result"] == "OK" for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
