"""Shared model overrides for simulation and independent dynamics oracles."""
from dataclasses import dataclass
import os

import mujoco
import numpy as np


@dataclass(frozen=True)
class ModelOverrides:
    armature: float = 0.01
    damping: float = 0.1
    foot_condim: int = 4
    foot_torsion: float = 0.03

    @classmethod
    def from_environment(cls, env=None):
        env = os.environ if env is None else env
        return cls(float(env.get("SIM_JOINT_ARM", "0.01") or 0),
                   float(env.get("SIM_JOINT_DAMP", "0.1") or 0),
                   int(env.get("SIM_FOOT_CONDIM", "4") or 0),
                   float(env.get("SIM_FOOT_TORSION", "0.03") or 0))


def apply_model_overrides(model, settings):
    values = (settings.armature, settings.damping, settings.foot_torsion)
    if not all(np.isfinite(v) and v >= 0 for v in values):
        raise ValueError("model overrides must be finite and nonnegative")
    if settings.foot_condim not in (0, 3, 4, 6):
        raise ValueError("foot condim must be 0 (unchanged), 3, 4 or 6")
    hinges = [int(model.jnt_dofadr[j]) for j in range(model.njnt)
              if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    # Zero preserves XML values, matching the frozen baseline's override convention.
    if settings.armature > 0:
        model.dof_armature[hinges] = settings.armature
    if settings.damping > 0:
        model.dof_damping[hinges] = settings.damping
    feet = [i for i in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").endswith("FOOT_collision")]
    if settings.foot_condim:
        model.geom_condim[feet] = settings.foot_condim
    if settings.foot_torsion > 0:
        model.geom_friction[feet, 1] = settings.foot_torsion
    return {"joint_dofs": hinges,
            "armature": model.dof_armature[hinges].tolist(),
            "damping": model.dof_damping[hinges].tolist(),
            "foot_condim": model.geom_condim[feet].tolist(),
            "foot_torsion": model.geom_friction[feet, 1].tolist()}
