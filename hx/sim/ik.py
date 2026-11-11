"""Whole-body inverse kinematics on the MuJoCo model (damped least squares).

Tasks: both sole sites (position + orientation), whole-body CoM (x, y),
pelvis height, pelvis orientation (upright).  Unknowns: floating base +
leg joints.  Arm / neck joints are held at prescribed values.
"""
from __future__ import annotations

import mujoco
import numpy as np


def _ori_err(R_cur: np.ndarray, R_tgt: np.ndarray) -> np.ndarray:
    return 0.5 * sum(np.cross(R_cur[:, i], R_tgt[:, i]) for i in range(3))


class WholeBodyIK:
    def __init__(self, model: mujoco.MjModel):
        self.m = model
        self.d = mujoco.MjData(model)
        m = model
        self.leg_joints = [m.joint(i).name for i in range(m.njnt)
                           if any(k in m.joint(i).name for k in
                                  ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"))]
        self.dofs = list(range(6)) + [m.joint(n).dofadr[0] for n in self.leg_joints]
        self.sites = {s: m.site(f"{s}_foot_sole").id for s in ("L", "R")}
        self.pelvis = m.body("pelvis").id
        self.jr = {n: m.joint(n).range.copy() for n in self.leg_joints}
        self.qadr = {n: m.joint(n).qposadr[0] for n in self.leg_joints}

    def com(self, qpos: np.ndarray) -> np.ndarray:
        self.d.qpos[:] = qpos
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        return self.d.subtree_com[self.pelvis].copy()

    def solve(self, qpos0: np.ndarray, feet: dict, com_xy, pelvis_z: float,
              fixed: dict | None = None, iters: int = 30, tol: float = 1e-6, yaw: float = 0.0):
        m, d = self.m, self.d
        q = qpos0.copy()
        for n in self.leg_joints:      # move away from the straight-knee singularity
            if n.endswith("knee") and q[self.qadr[n]] < 0.05:
                q[self.qadr[n]] = 0.35
                q[self.qadr[n.replace("knee", "hip_pitch")]] = -0.17
                q[self.qadr[n.replace("knee", "ankle_pitch")]] = -0.18
        if fixed:
            for n, v in fixed.items():
                q[m.joint(n).qposadr[0]] = v
        nv = m.nv
        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        jc = np.zeros((3, nv))
        Rt = np.eye(3)
        for it in range(iters):
            d.qpos[:] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            rows, errs = [], []
            for s in ("L", "R"):
                sid = self.sites[s]
                mujoco.mj_jacSite(m, d, jacp, jacr, sid)
                errs.append(np.asarray(feet[s]) - d.site_xpos[sid])
                rows.append(jacp.copy())
                errs.append(_ori_err(d.site_xmat[sid].reshape(3, 3), Rt))
                rows.append(jacr.copy())
            mujoco.mj_jacSubtreeCom(m, d, jc, self.pelvis)
            errs.append(np.asarray(com_xy) - d.subtree_com[self.pelvis][:2])
            rows.append(jc[:2].copy())
            mujoco.mj_jacBody(m, d, jacp, jacr, self.pelvis)
            errs.append(np.array([pelvis_z - d.xpos[self.pelvis][2]]))
            rows.append(jacp[2:3].copy())
            errs.append(_ori_err(d.xmat[self.pelvis].reshape(3, 3), Rt))
            rows.append(jacr.copy())
            e = np.concatenate(errs)
            J = np.vstack(rows)[:, self.dofs]
            if np.dot(e, e) < tol * tol:
                break
            lam = 1e-3 + 0.05 * float(np.dot(e, e))
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(J.shape[0]), e)
            nrm = np.linalg.norm(dq)
            if nrm > 0.15:
                dq *= 0.15 / nrm
            full = np.zeros(nv)
            full[self.dofs] = dq
            mujoco.mj_integratePos(m, q, full, 1.0)
            for n in self.leg_joints:
                a = self.qadr[n]
                q[a] = np.clip(q[a], *self.jr[n])
        return q, float(np.linalg.norm(e))
