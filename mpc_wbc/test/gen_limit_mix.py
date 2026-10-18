#!/usr/bin/env python3
"""生成单轴/多轴限速验证场景 CSV。

用法示例：
  # 双轴满速（每个轴都用自己的实测上限）
  python3 gen_limit_mix.py --mode 2axis --mx 0.70 --my 0.50 --mz 0.60 \
      --out scenarios/mix2_limit.csv

  # 三轴满速，并对包络整体缩放 0.9
  python3 gen_limit_mix.py --mode 3axis --mx 0.70 --my 0.50 --mz 0.60 \
      --scale 0.9 --out scenarios/mix3_limit90.csv

输出列：phase,duration,vx,vy,wz（与 position_loop_sim 的 S9_SCENARIO 格式一致）。
"""
import argparse
import csv


def combos_2axis(mx, my, mz):
    """二轴组合：x+y、x+z、y+z，每个组合取正反两个方向。"""
    out = []
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            out.append((f"xy_{'p' if sx > 0 else 'n'}{'p' if sy > 0 else 'n'}",
                        sx * mx, sy * my, 0.0))
    for sx in (1.0, -1.0):
        for sz in (1.0, -1.0):
            out.append((f"xz_{'p' if sx > 0 else 'n'}{'p' if sz > 0 else 'n'}",
                        sx * mx, 0.0, sz * mz))
    for sy in (1.0, -1.0):
        for sz in (1.0, -1.0):
            out.append((f"yz_{'p' if sy > 0 else 'n'}{'p' if sz > 0 else 'n'}",
                        0.0, sy * my, sz * mz))
    return out


def combos_3axis(mx, my, mz):
    """三轴组合：覆盖同号/异号下的四种典型耦合。"""
    return [
        ("xyz_ppp", mx, my, mz),
        ("xyz_pnp", mx, -my, mz),
        ("xyz_ppn", mx, my, -mz),
        ("xyz_npn", -mx, my, -mz),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("2axis", "3axis"), required=True)
    ap.add_argument("--mx", type=float, required=True, help="vx 幅值 (m/s)")
    ap.add_argument("--my", type=float, required=True, help="vy 幅值 (m/s)")
    ap.add_argument("--mz", type=float, required=True, help="wz 幅值 (rad/s)")
    ap.add_argument("--scale", type=float, default=1.0, help="整体包络缩放系数")
    ap.add_argument("--duration", type=float, default=10.0, help="每段时长 (s)")
    ap.add_argument("--stop", type=float, default=2.0, help="段间停步时长 (s)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    mx = args.mx * args.scale
    my = args.my * args.scale
    mz = args.mz * args.scale
    segs = (combos_2axis if args.mode == "2axis" else combos_3axis)(mx, my, mz)

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase", "duration", "vx", "vy", "wz"])
        for name, vx, vy, wz in segs:
            tag = name.replace(".", "p")
            w.writerow([tag, f"{args.duration:.1f}", f"{vx:.3f}", f"{vy:.3f}", f"{wz:.3f}"])
            w.writerow([f"stop_{tag}", f"{args.stop:.1f}", "0.0", "0.0", "0.0"])

    total = len(segs) * (args.duration + args.stop)
    print(f"wrote {args.out}: {len(segs)} 段，包络=({mx:.3f},{my:.3f},{mz:.3f})，"
          f"运动时长 {total:.1f}s")


if __name__ == "__main__":
    main()
