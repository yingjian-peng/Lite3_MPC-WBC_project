#!/usr/bin/env python3
"""S7 Weighted-QP WBC 的 MuJoCo 独立复核（把 WBC 解回灌给 MuJoCo 做正动力学）。

用法：
    python3 mpc_wbc/test/check_wbc_hover_mujoco.py <wbc_mujoco_cases.csv>

输入由 `lite3_wbc_test` 的 [10] 导出，每行 85 个数：
    q(12)  qvel(18)  qdd(18)  f(12)  tau(12)  R(9)  stance(4)

复核内容（与被测 C++ 实现完全独立：动力学全部走 MuJoCo 自己）：
    ① 正动力学：τ 写进 data.ctrl，接触力 f 用 mj_applyFT 施加在**足端接触点**（累加进
       data.qfrc_applied），基座抬高 5 cm 避开地面接触；mj_forward 后应有 qacc ≈ qdd。
       注意：`data.xfrc_applied` 在 MuJoCo ≥3.0 里**不被 mj_forward 消费**（只在 mj_step
       里生效），照抄旧写法会得到"外力凭空消失"的假失败——这里用 mj_applyFT。
    ② 支撑腿无滑：用 mj_integratePos ±h 中心差分独立测接触点加速度，应有 Jc·q̈ + Jdot·q̇ ≈ 0；
    ③ 摆动腿不受力：f_i ≈ 0。

物理约定（与 00_lite3_model.hpp 一致，已由 S2/S3 对表验证）：
    qpos = [p_base(3); quat(wxyz); q(12)]，qvel = [v_world(3); ω_body(3); dq(12)]，
    MJCF 关节顺序 FL/FR/HL/HR × HipX/HipY/Knee。

接触点约定（新球足踩平地面）：接触点 = 球心（FOOT 体原点）+ **世界系** (0,0,-r_foot)，即"球心正下方"
    而不是"机体系 -z 方向 × R_foot"——球足与水平地面的切点永远在球心正下方，与足的姿态无关。
    `00_lite3_model.hpp` 用的就是世界系偏移（与 gen_mujoco_contact_accel_cases.py 的
    `jacp − skew(d_contact)·jacr`、d_contact=(0,0,−r) 一致），曾经在复核脚本里按体坐标系旋转
    偏移，导致 Jᵀf 的关节分量差 0.36 N·m（HipY/Knee），qacc 假失败到 ~1e2 rad/s²。
"""
import sys

import numpy as np
import mujoco

MODEL_PATH = "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"
FOOT_BODIES = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
FOOT_RADIUS = 0.022
BASE_LIFT = 0.05   # 抬高基座，避开地面接触（本检查只关心自由状态下的正动力学）
H = 1e-6           # 中心差分步长
NQ, NV, NJ = 19, 18, 12


def quat_from_matrix(R):
    """3x3 旋转矩阵 → (w, x, y, z)（Shepperd 法，数值稳定）。"""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def skew(p):
    return np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])


def contact_vel(m, d, qpos, qvel, foot_ids, jacp, jacr, d_contact):
    """接触点世界速度（4 足 × 3），由 mj_jacBody 换算到接触点。"""
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    out = np.zeros(12)
    for leg, bid in enumerate(foot_ids):
        jacp[:] = 0.0
        jacr[:] = 0.0
        mujoco.mj_jacBody(m, d, jacp, jacr, bid)
        out[3 * leg:3 * leg + 3] = (jacp - skew(d_contact) @ jacr) @ qvel
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "wbc_mujoco_cases.csv"
    rows = []
    with open(path) as f:
        for line in f:
            if line.strip():
                rows.append(np.array([float(x) for x in line.split()]))
    if not rows:
        print(f"FAIL: {path} 为空")
        return 1

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    assert m.nq == NQ and m.nv == NV, (m.nq, m.nv)
    foot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in FOOT_BODIES]
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    d_contact = np.array([0.0, 0.0, -FOOT_RADIUS])

    err_dyn = err_contact = err_swing_f = 0.0
    print(f"S7 WBC MuJoCo 独立复核：{len(rows)} 个用例（{path}）")
    for i, v in enumerate(rows):
        n_expect = NJ + 2 * NV + 2 * NJ + 9 + 4
        assert v.size == n_expect, (v.size, n_expect)
        q = v[0:NJ]
        qvel = v[NJ:NJ + NV]
        qdd = v[NJ + NV:NJ + 2 * NV]
        f = v[NJ + 2 * NV:NJ + 2 * NV + 12].reshape(4, 3)
        tau = v[NJ + 2 * NV + 12:NJ + 2 * NV + 24]
        R = v[NJ + 2 * NV + 24:NJ + 2 * NV + 33].reshape(3, 3)
        stance = v[NJ + 2 * NV + 33:NJ + 2 * NV + 37].astype(bool)

        # 基座名义高度 = 站立高度（球足触地时） + 抬升量
        base_z = 0.3485 + BASE_LIFT
        qpos0 = np.concatenate([[0.0, 0.0, base_z], quat_from_matrix(R), q])

        # ---- ① 正动力学：qacc ≈ qdd ----
        d.qpos[:] = qpos0
        d.qvel[:] = qvel
        d.ctrl[:] = tau
        d.qfrc_applied[:] = 0.0
        mujoco.mj_forward(m, d)   # 先拿足体姿态，用于把接触力施加到接触点
        qfrc = np.zeros(m.nv)
        for leg, bid in enumerate(foot_ids):
            if np.linalg.norm(f[leg]) == 0.0:
                continue
            point = d.xpos[bid] + d_contact  # 世界系接触点：球心正下方（见文件头约定说明）
            tf = np.zeros(m.nv)
            mujoco.mj_applyFT(m, d, f[leg], np.zeros(3), point, bid, tf)
            qfrc += tf
        d.qpos[:] = qpos0
        d.qvel[:] = qvel
        d.qfrc_applied[:] = qfrc
        mujoco.mj_forward(m, d)
        qacc = d.qacc.copy()
        err_dyn = max(err_dyn, float(np.abs(qacc - qdd).max()))

        # ---- ② 支撑腿无滑：独立测接触点加速度 ----
        if stance.any():
            qpos_p = qpos0.copy()
            qpos_m = qpos0.copy()
            mujoco.mj_integratePos(m, qpos_p, qvel, +H)
            mujoco.mj_integratePos(m, qpos_m, qvel, -H)
            vc_p = contact_vel(m, d, qpos_p, qvel + H * qdd, foot_ids, jacp, jacr, d_contact)
            vc_m = contact_vel(m, d, qpos_m, qvel - H * qdd, foot_ids, jacp, jacr, d_contact)
            a_c = (vc_p - vc_m) / (2.0 * H)
            for leg in range(4):
                if stance[leg]:
                    err_contact = max(err_contact, float(np.abs(a_c[3 * leg:3 * leg + 3]).max()))

        # ---- ③ 摆动腿不受力 ----
        for leg in range(4):
            if not stance[leg]:
                err_swing_f = max(err_swing_f, float(np.abs(f[leg]).max()))

        print(f"  case {i}: |qacc−qdd|∞={np.abs(qacc - qdd).max():.3e}  stance={stance.astype(int)}")

    print(f"\n  max|qacc − qdd|          = {err_dyn:.3e}")
    print(f"  max|接触点加速度|（支撑） = {err_contact:.3e}")
    print(f"  max|f|（摆动）            = {err_swing_f:.3e}")
    ok = err_dyn < 1e-8 and err_contact < 1e-4 and err_swing_f < 1e-12
    print("RESULT: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
