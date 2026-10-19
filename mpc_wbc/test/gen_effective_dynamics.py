"""Independent MuJoCo oracle: M, effective bias, CoM Jacobian and free acceleration."""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "interface/robot/simulation"))
from model_config import ModelOverrides, apply_model_overrides


def main():
    rng = np.random.default_rng(20261019)
    cases = []
    for armature, damping in [(0, 0), (0.01, 0.1), (0.025, 0.2)]:
        model = mujoco.MjModel.from_xml_path(str(ROOT / "robot_models/robots/Lite3/Lite3_mjcf/mjcf/Lite3_flat.xml"))
        apply_model_overrides(model, ModelOverrides(armature, damping, 4, 0.03))
        model.opt.disableflags |= (mujoco.mjtDisableBit.mjDSBL_CONTACT
                                   | mujoco.mjtDisableBit.mjDSBL_LIMIT)
        data = mujoco.MjData(model)
        if armature == 0.025:  # nonuniform parameters detect wrong joint indexing
            model.dof_armature[6:] *= rng.uniform(0.5, 1.5, 12)
            model.dof_damping[6:] *= rng.uniform(0.5, 1.5, 12)
        for _ in range(50):
            data.qpos[:3] = [0.2, -0.3, 1.0]
            quat = rng.normal(size=4)
            data.qpos[3:7] = quat / np.linalg.norm(quat)
            data.qpos[7:] = np.tile([0, -0.65, 1.3], 4) + rng.uniform(-0.12, 0.12, 12)
            data.qvel[:] = rng.uniform(-1.0, 1.0, 18)
            applied = rng.uniform(-5, 5, 18)
            data.qfrc_applied[:] = applied
            mujoco.mj_forward(model, data)
            mass = np.empty((18, 18))
            mujoco.mj_fullM(model, mass, data.qM)
            jac = np.zeros((3, 18))
            mujoco.mj_jacSubtreeCom(model, data, jac, 1)
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
            # Damping is passive force, NOT part of qfrc_bias in MuJoCo.
            effective_bias = data.qfrc_bias - data.qfrc_passive
            cases.append(np.concatenate([
                data.qpos[7:], data.qvel, rotation, model.opt.gravity,
                model.dof_armature[6:], model.dof_damping[6:], mass.ravel(),
                effective_bias, jac.ravel(), applied, data.qacc]))
    with Path(sys.argv[1]).open("w") as stream:
        stream.write(f"{len(cases)}\n")
        for case in cases:
            stream.write(" ".join(f"{x:.17g}" for x in case) + "\n")
    print(f"wrote {len(cases)} effective-dynamics oracle cases")


if __name__ == "__main__":
    main()
