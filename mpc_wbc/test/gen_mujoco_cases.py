#!/usr/bin/env python3
"""Generate MuJoCo oracle cases for the Lite3 kinematics table-check (S1).

用法：
    python3 mpc_wbc/test/gen_mujoco_cases.py <out_file> [num_cases]

输出文本格式（每个 case 18 行）：
    px py pz qw qx qy qz          # 浮动基位置 + 四元数(wxyz)，即 qpos[0:7]
    q0 ... q11                    # 12 个关节角（FL,FR,HL,HR × HipX,HipY,Knee）
    fx fy fz                      # 4 行：四足球心（FOOT body 原点）世界位置
    <18 个数>                     # 12 行：4 足 × 3 行的 jacp（世界系 3x18）
"""
import sys
import numpy as np
import mujoco

MODEL_PATH = "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
FOOT_BODIES = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
LEG_NAMES = ["FL", "FR", "HL", "HR"]


def random_quat(rng, tilt=0.3):
    rpy = rng.uniform(-tilt, tilt, size=3)
    r, p, y = rpy
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, yy, z])


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "mpc_wbc/test/cases.txt"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 200

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in FOOT_BODIES]
    assert all(i >= 0 for i in foot_ids), "foot bodies not found"

    # 关节范围（HipX, HipY, Knee），用于采样「合理位姿」
    jnt_ranges = []
    for leg in range(4):
        for jn in ["HipX", "HipY", "Knee"]:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{LEG_NAMES[leg]}_{jn}_joint")
            jnt_ranges.append(m.jnt_range[jid])
    jnt_ranges = np.array(jnt_ranges)

    rng = np.random.default_rng(20241009)
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))

    lines = [str(n)]
    for _ in range(n):
        base_pos = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0.25, 0.45)])
        quat = random_quat(rng, 0.3)

        # 在真实关节范围内采样（留一点边距），覆盖 walking 与较大偏摆
        lo = jnt_ranges[:, 0] + 1e-3
        hi = jnt_ranges[:, 1] - 1e-3
        q = rng.uniform(lo, hi)

        d.qpos[:] = 0.0
        d.qpos[0:3] = base_pos
        d.qpos[3:7] = quat
        d.qpos[7:19] = q
        mujoco.mj_forward(m, d)

        lines.append(" ".join(f"{v:.17g}" for v in np.concatenate([base_pos, quat])))
        lines.append(" ".join(f"{v:.17g}" for v in q))
        for bid in foot_ids:
            lines.append(" ".join(f"{v:.17g}" for v in d.xpos[bid]))
        for bid in foot_ids:
            jacp[:] = 0.0
            jacr[:] = 0.0
            mujoco.mj_jacBody(m, d, jacp, jacr, bid)
            for r in range(3):
                lines.append(" ".join(f"{v:.17g}" for v in jacp[r]))

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {n} cases to {out_path}")


if __name__ == "__main__":
    main()
