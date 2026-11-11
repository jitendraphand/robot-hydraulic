"""ZMP preview-control walking pattern generator (Kajita 2003, LIPM).

Produces CoM, ZMP and foot trajectories for a straight walk; shared by the
simulation and the on-robot brain (hx.brain) so both run identical gaits.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are

G = 9.81


@dataclass
class GaitParams:
    step_length: float = 0.08
    step_width: float = 0.18
    step_time: float = 0.8
    double_support_fraction: float = 0.25
    swing_height: float = 0.035
    n_steps: int = 8
    preview_time: float = 1.6
    t_start: float = 1.2
    t_end: float = 1.8
    dt: float = 0.005
    zmp_forward_offset: float = 0.015

    @classmethod
    def from_cfg(cls, cfg) -> "GaitParams":
        g = cfg["sim"]["gait"]
        return cls(step_length=g["step_length"], step_width=g["step_width"], step_time=g["step_time"],
                   double_support_fraction=g["double_support_fraction"], swing_height=g["swing_height"],
                   n_steps=g["n_steps"], preview_time=g["preview_time"])


def preview_gains(zc: float, dt: float, n_preview: int, Qe: float = 1.0, R: float = 1e-6):
    A = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
    B = np.array([[dt ** 3 / 6], [dt * dt / 2], [dt]])
    C = np.array([[1, 0, -zc / G]])
    # augmented (Katayama) system: state [e_int; dx]
    A1 = np.block([[np.eye(1), C @ A], [np.zeros((3, 1)), A]])
    B1 = np.vstack([C @ B, B])
    Q = np.zeros((4, 4))
    Q[0, 0] = Qe
    P = solve_discrete_are(A1, B1, Q, R * np.eye(1))
    S = R + B1.T @ P @ B1
    K = np.linalg.solve(S, B1.T @ P @ A1)
    Gi, Gx = K[0, 0], K[0, 1:]
    Ac = A1 - B1 @ K
    I1 = np.array([[1.0], [0, ], [0], [0]])
    X = -Ac.T @ P @ I1
    Gd = np.zeros(n_preview)
    Gd[0] = -Gi
    for i in range(1, n_preview):
        Gd[i] = float(np.linalg.solve(S, B1.T @ X)[0, 0])
        X = Ac.T @ X
    return A, B, C, Gi, Gx, Gd


def footsteps(p: GaitParams, x0: float = 0.0):
    """Footprints advance by step_length each step; the last step closes the feet.

    Returns ([(support, swing, swing_start_xy, swing_target_xy, support_xy)], final_feet).
    """
    w = p.step_width / 2
    feet = {"L": np.array([x0, w]), "R": np.array([x0, -w])}
    seq = []
    swing = "L"
    for k in range(p.n_steps):
        sup = "R" if swing == "L" else "L"
        target = feet[swing].copy()
        target[0] = feet[sup][0] + (0.0 if k == p.n_steps - 1 else p.step_length)
        seq.append((sup, swing, feet[swing].copy(), target.copy(), feet[sup].copy()))
        feet[swing] = target
        swing = sup
    return seq, feet


def plan(p: GaitParams, zc: float, com0=(0.0, 0.0)):
    """Return dict of arrays at p.dt: t, com(x,y), zmp_ref, foot L/R (x,y,z), support phase."""
    seq, final = footsteps(p)
    Td = p.step_time * p.double_support_fraction
    Ts = p.step_time - Td
    total = p.t_start + p.n_steps * p.step_time + Td + p.t_end
    t = np.arange(0, total, p.dt)
    n = len(t)
    zmp = np.zeros((n, 2))
    fL = np.zeros((n, 3))
    fR = np.zeros((n, 3))
    phase = np.zeros(n, dtype=int)     # 0 DS, 1 L support, 2 R support
    w = p.step_width / 2
    feet = {"L": np.array([0.0, w]), "R": np.array([0.0, -w])}
    off = np.array([p.zmp_forward_offset, 0.0])
    centre = np.array(com0)
    # piecewise schedule
    events = []   # (t0, t1, kind, data)
    tc = p.t_start
    events.append((0.0, tc, "ds", (centre, centre)))
    prev_z = centre
    for sup, swing, start, target, supxy in seq:
        z_sup = supxy + off
        events.append((tc, tc + Td, "ds", (prev_z, z_sup)))
        events.append((tc + Td, tc + p.step_time, "ss", (sup, swing, start, target, z_sup)))
        prev_z = z_sup
        tc += p.step_time
    end_c = (final["L"] + final["R"]) / 2 + off
    events.append((tc, tc + Td, "ds", (prev_z, end_c)))
    events.append((tc + Td, total + 10, "ds", (end_c, end_c)))
    for (t0, t1, kind, data) in events:
        idx = np.nonzero((t >= t0) & (t < t1))[0]
        if len(idx) == 0:
            if kind == "ss":
                feet[data[1]] = data[3]
            continue
        sv = (t[idx] - t0) / max(t1 - t0, 1e-9)
        sm = (0.5 - 0.5 * np.cos(np.pi * sv))[:, None]
        if kind == "ds":
            a_, b_ = data
            zmp[idx] = a_ + (b_ - a_) * sm
            fL[idx, :2], fR[idx, :2] = feet["L"], feet["R"]
        else:
            sup, swing, start, target, zs = data
            zmp[idx] = zs
            phase[idx] = 1 if sup == "L" else 2
            sw = fL if swing == "L" else fR
            st = fR if swing == "L" else fL
            sw[idx, :2] = start + (target - start) * sm
            sw[idx, 2] = p.swing_height * np.sin(np.pi * sv)
            st[idx, :2] = feet[sup]
            feet[swing] = target
    # preview control per axis
    N = int(p.preview_time / p.dt)
    A, B, C, Gi, Gx, Gd = preview_gains(zc, p.dt, N)
    com = np.zeros((n, 2))
    comv = np.zeros((n, 2))
    zpad = np.vstack([zmp, np.repeat(zmp[-1:], N, 0)])
    for ax in range(2):
        x = np.array([com0[ax], 0.0, 0.0])
        e = 0.0
        for k in range(n):
            y = float((C @ x)[0])
            e += y - zpad[k, ax]
            u = float(-Gi * e - Gx @ x - Gd @ zpad[k + 1:k + 1 + N, ax])
            com[k, ax], comv[k, ax] = x[0], x[1]
            x = A @ x + (B * u).ravel()
    return dict(t=t, com=com, comv=comv, zmp=zmp, foot_L=fL, foot_R=fR, phase=phase,
                footsteps=[(s[0], s[3].tolist()) for s in seq])
