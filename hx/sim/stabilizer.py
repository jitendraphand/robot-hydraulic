"""Balance stabilizer shared by the simulation and the on-robot brain.

The ZMP gait is planned with a linear inverted pendulum; the real multi-body robot
deviates from it, and without feedback the lateral CoM error grows step by step until the
robot rolls over the foot edge (this was found by rendering the walking video: with the
final CAD masses the open-loop gait fell at t = 7.6 s).

Two feedback terms are added to the joint references of both legs:

  * attitude:  hip_roll   += k_att * pelvis_roll + d_att * roll_rate
  * CoM / ZMP: ankle_roll -= k_com * e_y + d_com * e_vy        (e = measured - planned CoM)
               ankle_pitch += k_com * e_x + d_com * e_vx

The CoM term moves the centre of pressure under the CoM error (ankle strategy).  Gains
and signs were selected by sweep and verified across +-10 % torso mass (QA check SIM-8).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class StabilizerGains:
    enabled: bool = True
    k_att: float = 1.0      # rad / rad
    d_att: float = 0.05     # rad / (rad/s)
    k_com: float = 2.0      # rad / m
    d_com: float = 0.3      # rad / (m/s)
    limit: float = 0.15     # rad, clamp of every correction

    @classmethod
    def from_cfg(cls, cfg) -> "StabilizerGains":
        return cls(**cfg["sim"].get("stabilizer", {}))


class Stabilizer:
    def __init__(self, gains: StabilizerGains, joint_index: dict):
        self.g = gains
        self.idx = joint_index

    def corrections(self, roll: float, roll_rate: float, com_err, com_vel_err, n: int) -> np.ndarray:
        corr = np.zeros(n)
        if not self.g.enabled:
            return corr
        g = self.g
        ex, ey = float(com_err[0]), float(com_err[1])
        evx, evy = float(com_vel_err[0]), float(com_vel_err[1])
        for s in "LR":
            corr[self.idx[f"{s}_hip_roll"]] += g.k_att * roll + g.d_att * roll_rate
            corr[self.idx[f"{s}_ankle_roll"]] += -(g.k_com * ey + g.d_com * evy)
            corr[self.idx[f"{s}_ankle_pitch"]] += g.k_com * ex + g.d_com * evx
        return np.clip(corr, -g.limit, g.limit)


def pelvis_roll(xmat) -> float:
    R = np.asarray(xmat).reshape(3, 3)
    return math.atan2(R[2, 1], R[2, 2])


def com_relative_to_feet(model, data, qpos) -> np.ndarray:
    """CoM (x, y) relative to the midpoint of both soles, in the pelvis-heading frame.

    Used by the brain: the same quantity is computed for the measured state (joint encoders +
    IMU attitude) and for the reference, so no global position estimate is needed."""
    import mujoco
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    pel = model.body("pelvis").id
    mid = 0.5 * (data.site("L_foot_sole").xpos + data.site("R_foot_sole").xpos)
    return (data.subtree_com[pel] - mid)[:2].copy()
