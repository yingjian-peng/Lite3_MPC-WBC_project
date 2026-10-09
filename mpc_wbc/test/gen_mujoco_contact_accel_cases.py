#!/usr/bin/env python3
"""Generate MuJoCo oracle cases for the contact-point acceleration identity (S7).

用法：
    python3 mpc_wbc/test/gen_mujoco_contact_accel_cases.py <out_file> [num_cases]

每个 case 16 行：
    px py pz                          # 浮动基位置
    qw qx qy qz                       # 浮动基四元数(wxyz)
    q0 ... q11                        # 12 个关节角
    qv0 ... qv17                      # 广义速度 [v_world(3); omega_body(3); dq(12)]
    qdd0 ... qdd17                    # 广义加速度 [a_world(3); alpha_body(3); ddq(12)]
    12 行：接触点世界加速度 a_c（4 足 × 3），即真值

真值构造（与 C++ 实现相互独立）：
    a_c = Jc(q)·qdd + Jdot_c(q,q̇)·q̇
用「运动学积分 + 中心差分」测出整条：
    qpos(±h) = mj_integratePos(qpos, qvel, ±h)      # 只按广义速度积分（指数映射）
    qvel(±h) = qvel ± h·qdd
    v_c(±h)  = Jc(qpos(±h))·qvel(±h)                # Jc 由 mj_jacBody(FOOT) 换算到球足最低点
    a_c      = (v_c(+h) − v_c(−h)) / (2h)

这样 S7 的 `ContactBiasAccel()`（Jdot·v）就能被独立校验：C++ 侧比较
    Jc(q)·qdd + ContactBiasAccel(q, q̇)  vs  a_c
"""
import sys

import numpy as np
import mujoco

MODEL_PATH = "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
LEG_NAMES = ["FL", "FR", "HL", "HR"]
FOOT_BODIES = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
FOOT_RADIUS = 0.022
H = 1e-6


def skew(p):
    return np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])


def random_quat(rng, tilt=0.4):
    r, p, y = rng.uniform(-tilt, tilt, size=3)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def contact_velocity(m, d, qpos, qvel, foot_ids, jacp, jacr, d_contact):
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    out = np.zeros(12)
    for leg, bid in enumerate(foot_ids):
        jacp[:] = 0.0
        jacr[:] = 0.0
        mujoco.mj_jacBody(m, d, jacp, jacr, bid)
        Jc = jacp - skew(d_contact) @ jacr
        out[3 * leg:3 * leg + 3] = Jc @ qvel
    return out


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "mpc_wbc/test/contact_accel_cases.txt"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 100

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

    rng = np.random.default_rng(20241011)
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    d_contact = np.array([0.0, 0.0, -FOOT_RADIUS])

    lines = [str(n)]
    for _ in range(n):
        base_pos = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(0.25, 0.5)])
        quat = random_quat(rng)
        q = rng.uniform(lo, hi)
        qvel = np.concatenate([
            rng.uniform(-1.0, 1.0, 3),      # v_world
            rng.uniform(-2.0, 2.0, 3),      # omega_body
            rng.uniform(-4.0, 4.0, 12),     # dq
        ])
        qdd = np.concatenate([
            rng.uniform(-3.0, 3.0, 3),      # a_world
            rng.uniform(-6.0, 6.0, 3),      # alpha_body
            rng.uniform(-30.0, 30.0, 12),   # ddq
        ])

        qpos0 = np.concatenate([base_pos, quat, q])
        qpos_p = qpos0.copy()
        qpos_m = qpos0.copy()
        mujoco.mj_integratePos(m, qpos_p, qvel, +H)
        mujoco.mj_integratePos(m, qpos_m, qvel, -H)
        vc_p = contact_velocity(m, d, qpos_p, qvel + H * qdd, foot_ids, jacp, jacr, d_contact)
        vc_m = contact_velocity(m, d, qpos_m, qvel - H * qdd, foot_ids, jacp, jacr, d_contact)
        a_c = (vc_p - vc_m) / (2.0 * H)

        lines.append(" ".join(f"{v:.17g}" for v in base_pos))
        lines.append(" ".join(f"{v:.17g}" for v in quat))
        lines.append(" ".join(f"{v:.17g}" for v in q))
        lines.append(" ".join(f"{v:.17g}" for v in qvel))
        lines.append(" ".join(f"{v:.17g}" for v in qdd))
        for r in range(12):
            lines.append(f"{a_c[r]:.17g}")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {n} contact-acceleration cases to {out_path}")


if __name__ == "__main__":
    main()
