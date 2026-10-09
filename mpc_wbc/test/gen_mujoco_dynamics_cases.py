#!/usr/bin/env python3
"""Generate MuJoCo oracle cases for the Lite3 whole-body dynamics table-check (S2).

用法：
    python3 mpc_wbc/test/gen_mujoco_dynamics_cases.py <out_file> [num_cases]

每个 case 36 行：
    px py pz qw qx qy qz          # 浮动基位置 + 四元数(wxyz)
    q0 ... q11                    # 12 个关节角
    qv0 ... qv17                  # 广义速度 [v_world(3); omega_body(3); dq(12)]
    gx gy gz                      # MuJoCo 重力向量
    com_x com_y com_z             # 整机质心（世界系，data.subtree_com[0]）
    18 行：M(q)（每行 18 个数）
    1 行：qfrc_bias（18 个数）= C·qvel + G
    12 行：接触点雅可比 Jc（世界系，4 足 × 3 行）

Jc 由 mj_jacBody(FOOT) 换算：球足最低点相对球心在世界系下偏移 d=(0,0,-r)，
    Jc = jacp - skew(d) · jacr

M(q) 的浮动基旋转块 M[3:6, 3:6]（机体系表达）= 整机绕浮动基原点的合成惯量，
可用于校验 CompositeInertiaBody()（S6 用到的 SRBD 惯量）。
"""
import sys
import numpy as np
import mujoco

MODEL_PATH = "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
LEG_NAMES = ["FL", "FR", "HL", "HR"]
FOOT_BODIES = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
FOOT_RADIUS = 0.022


def skew(p):
    return np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])


def random_quat(rng, tilt=0.5):
    rpy = rng.uniform(-tilt, tilt, size=3)
    r, p, y = rpy
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "mpc_wbc/test/dyn_cases.txt"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 200

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    assert m.nv == 18 and m.nq == 19
    foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in FOOT_BODIES]

    jnt_ranges = []
    for leg in range(4):
        for jn in ["HipX", "HipY", "Knee"]:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{LEG_NAMES[leg]}_{jn}_joint")
            jnt_ranges.append(m.jnt_range[jid])
    lo = np.array(jnt_ranges)[:, 0] + 1e-3
    hi = np.array(jnt_ranges)[:, 1] - 1e-3

    rng = np.random.default_rng(20241009)
    M = np.zeros((m.nv, m.nv))
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    d_contact = np.array([0.0, 0.0, -FOOT_RADIUS])

    lines = [str(n)]
    for _ in range(n):
        base_pos = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(0.25, 0.5)])
        quat = random_quat(rng, 0.5)
        q = rng.uniform(lo, hi)

        d.qpos[:] = 0.0
        d.qpos[0:3] = base_pos
        d.qpos[3:7] = quat
        d.qpos[7:19] = q

        # 广义速度：[v_world(3); omega_body(3); dq(12)]
        qvel = np.concatenate([
            rng.uniform(-0.5, 0.5, 3),
            rng.uniform(-1.0, 1.0, 3),
            rng.uniform(-3.0, 3.0, 12),
        ])
        d.qvel[:] = qvel

        mujoco.mj_forward(m, d)
        mujoco.mj_fullM(m, M, d.qM)

        Jc = np.zeros((12, m.nv))
        for leg, bid in enumerate(foot_ids):
            jacp[:] = 0.0
            jacr[:] = 0.0
            mujoco.mj_jacBody(m, d, jacp, jacr, bid)
            Jc[3 * leg:3 * leg + 3, :] = jacp - skew(d_contact) @ jacr

        lines.append(" ".join(f"{v:.17g}" for v in np.concatenate([base_pos, quat])))
        lines.append(" ".join(f"{v:.17g}" for v in q))
        lines.append(" ".join(f"{v:.17g}" for v in qvel))
        lines.append(" ".join(f"{v:.17g}" for v in m.opt.gravity))
        lines.append(" ".join(f"{v:.17g}" for v in d.subtree_com[0]))  # 整机质心（世界系）
        for r in range(m.nv):
            lines.append(" ".join(f"{v:.17g}" for v in M[r]))
        lines.append(" ".join(f"{v:.17g}" for v in d.qfrc_bias))
        for r in range(12):
            lines.append(" ".join(f"{v:.17g}" for v in Jc[r]))

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {n} dynamics cases to {out_path}")


if __name__ == "__main__":
    main()
