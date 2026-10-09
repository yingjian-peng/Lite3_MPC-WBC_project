#!/usr/bin/env python3
"""Generate a MuJoCo trot rollout for validating the S5 state estimator.

用 Python 直接驱动 MuJoCo（不走 UDP），复用与 C++ 相同的运动学/步态公式跑一段
对角小跑，记录真值与“控制器已知的计划接触”，供 C++ 估计器回放对比。
这份脚本也可作为 S6/S7 的离线 MuJoCo 参考。

用法： python3 mpc_wbc/test/gen_estimator_rollout.py <out.csv> [duration]
输出列： t,bx,by,bz,vx,vy,vz,roll,pitch,yaw,wx,wy,wz,q0..q11,dq0..dq11,c0..c3
"""
import sys
import numpy as np
import mujoco

MODEL_PATH = "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
L1, L2, LH, R_FOOT = 0.20, 0.21012, 0.09735, 0.022
HIP = [np.array([0.1745, 0.062, 0.0]), np.array([0.1745, -0.062, 0.0]),
       np.array([-0.1745, 0.062, 0.0]), np.array([-0.1745, -0.062, 0.0])]
SIDE = [1.0, -1.0, 1.0, -1.0]
STAND_LEG = np.array([0.0, -0.65, 1.3])
PHASE_OFF = [0.0, 0.5, 0.5, 0.0]
T_GAIT, DUTY, SWING_H = 0.60, 0.68, 0.06


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def fk(leg, q):
    qx, qy, qk = q
    xs = L1 * np.sin(qy) + L2 * np.sin(qy + qk)
    zs = -L1 * np.cos(qy) - L2 * np.cos(qy + qk)
    return HIP[leg] + Rx(-qx) @ np.array([xs, SIDE[leg] * LH, zs])


def ik(leg, p, seed):
    r = p - HIP[leg]
    s = SIDE[leg]
    A = np.hypot(r[1], r[2])
    if A < LH - 1e-9:
        return None
    psi = np.arctan2(r[2], r[1])
    phi = np.arccos(np.clip(s * LH / A, -1, 1))
    wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
    sols = []
    for qx in (wrap(phi - psi), wrap(-phi - psi)):
        zs = np.sin(qx) * r[1] + np.cos(qx) * r[2]
        xs = r[0]
        rho = np.clip(np.hypot(xs, zs), abs(L1 - L2) + 1e-9, L1 + L2 - 1e-9)
        qk = np.arccos(np.clip((rho ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2), -1, 1))
        qy = np.arctan2(xs, -zs) - np.arctan2(L2 * np.sin(qk), L1 + L2 * np.cos(qk))
        sols.append(np.array([qx, qy, qk]))
    return sols[0] if np.linalg.norm(sols[0] - seed) <= np.linalg.norm(sols[1] - seed) else sols[1]


class Gait:
    def __init__(self):
        self.nominal = [fk(leg, STAND_LEG) for leg in range(4)]
        self.stepping = False
        self.t0 = 0.0

    def update(self, t, cmd):
        vx, vy, wz = cmd
        if not self.stepping and (np.hypot(vx, vy) > 0.01 or abs(wz) > 0.01):
            self.stepping, self.t0 = True, t
        if self.stepping and np.hypot(vx, vy) < 0.005 and abs(wz) < 0.005:
            self.stepping = False
        if not self.stepping:
            return [p.copy() for p in self.nominal], [True] * 4
        theta = ((t - self.t0) / T_GAIT) % 1.0
        pos, contact = [], []
        for leg in range(4):
            p0 = self.nominal[leg]
            phi = (theta + PHASE_OFF[leg]) % 1.0
            dp = np.array([T_GAIT * DUTY * (vx - wz * p0[1]), T_GAIT * DUTY * (vy + wz * p0[0]), 0.0])
            if phi < DUTY:
                xi = phi / DUTY
                p = p0 + (0.5 - xi) * dp
                contact.append(True)
            else:
                xi = (phi - DUTY) / (1.0 - DUTY)
                s = 10 * xi ** 3 - 15 * xi ** 4 + 6 * xi ** 5
                p = p0 + (s - 0.5) * dp
                p = p.copy()
                p[2] = p0[2] + 0.5 * SWING_H * (1 - np.cos(2 * np.pi * xi))
                contact.append(False)
            pos.append(p)
        return pos, contact


def quat_to_rpy(q):
    w, x, y, z = q
    return np.array([
        np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
        np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "mpc_wbc/test/estimator_rollout.csv"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    m.opt.timestep = 0.001  # 与 interface/robot/simulation/mujoco_simulation.py 的 DT 一致
    dt = m.opt.timestep
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "TORSO")

    d.qpos[0:3] = [0.0, 0.0, 0.35]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    d.qpos[7:19] = np.tile(STAND_LEG, 4)
    mujoco.mj_forward(m, d)

    kp, kd = 100.0, 3.5
    gait = Gait()
    q_des = np.tile(STAND_LEG, 4)
    lines = ["t,bx,by,bz,vx,vy,vz,roll,pitch,yaw,wx,wy,wz,"
             + ",".join(f"q{i}" for i in range(12)) + ","
             + ",".join(f"dq{i}" for i in range(12)) + ","
             + ",".join(f"c{i}" for i in range(4))]

    def schedule(t):
        if t < 1.0:
            return (0.0, 0.0, 0.0)
        if t < 4.0:
            return (0.15, 0.0, 0.0)
        if t < 5.0:
            return (0.0, 0.0, 0.0)
        if t < 8.0:
            return (-0.15, 0.0, 0.0)
        if t < 9.0:
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, 0.2)

    n = int(duration / dt)
    for step in range(n):
        t = step * dt
        # 统一在步进前采集，保证 q/dq 与基座位姿/速度来自同一时刻
        base_pos = d.qpos[0:3].copy()
        quat = d.qpos[3:7].copy()
        q = d.qpos[7:19].copy()
        dq = d.qvel[6:18].copy()
        vB = d.qvel[0:3].copy()
        w = d.qvel[3:6].copy()

        cmd = schedule(t)
        pos, contact = gait.update(t, cmd)
        if cmd != (0.0, 0.0, 0.0) or gait.stepping:
            for leg in range(4):
                ql = ik(leg, pos[leg], q_des[3 * leg:3 * leg + 3])
                if ql is not None:
                    q_des[3 * leg:3 * leg + 3] = ql

        d.ctrl[:] = kp * (q_des - q) + kd * (0.0 - dq)

        d.xfrc_applied[:] = 0.0
        if 9.2 <= t < 9.6:
            d.xfrc_applied[torso, 1] = 20.0  # 原地转身阶段的侧推

        mujoco.mj_step(m, d)

        if step % 10 == 0:
            rpy = quat_to_rpy(quat)
            vals = ([t] + list(base_pos) + list(vB) + list(rpy) + list(w) + list(q) + list(dq)
                    + [1.0 if c else 0.0 for c in contact])
            lines.append(",".join(f"{v:.9g}" for v in vals))

    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {len(lines)-1} rows to {out}")


if __name__ == "__main__":
    main()
