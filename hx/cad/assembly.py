"""Full-robot assembly from CAD + kinematics: interference checks and renders.

Parts are placed with MuJoCo forward kinematics (same model as the
simulation), cylinders are instantiated between their pins at the actual
pin-to-pin length, and every pair of solids is intersected exactly with
manifold3d.  A design passes when no pair overlaps by more than the
tolerance in any of the checked poses (zero pose, joint limits, and frames
sampled from the simulated scenarios).
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from ..config import ROOT, RobotConfig
from ..hydraulics import Cylinder, linkage_for
from ..sim.model import build_mjcf
from . import cylinder as CY
from .core import look_rotation, to_trimesh
from .features import JH
from .links import BODY_BUILDERS

M = 1000.0


class Assembly:
    def __init__(self, cfg: RobotConfig):
        self.cfg = cfg
        self.sizing = cfg.derived("sizing")
        jh = JH(cfg)
        self.parts = {}
        for base, fn in BODY_BUILDERS.items():
            m = fn(cfg, self.sizing, jh)
            if base in cfg.bodies:
                self.parts[base] = m
            else:
                self.parts[f"L_{base}"] = m
                self.parts[f"R_{base}"] = m.mirror([0, 1, 0])
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
        self.data = mujoco.MjData(self.model)
        self._cyl_cache = {}

    def _cyl_solid(self, c: Cylinder, L):
        key = (c.bore_mm, c.stroke_mm, round(L, 1))
        if key not in self._cyl_cache:
            self._cyl_cache[key] = CY.assembly_solid(self.cfg, c, L)
        return self._cyl_cache[key]

    def pose(self, qmap: dict | None = None):
        """Return {name: placed Manifold} (mm, world frame) and cylinder fit info."""
        m, d = self.model, self.data
        d.qpos[:] = m.qpos0
        g = self.cfg["geometry"]
        d.qpos[2] = g["hip_drop"] + g["hip_pitch_offset"] + g["thigh_length"] + g["shin_length"] + g["ankle_roll_offset"] + g["ankle_height"]
        for n, v in (qmap or {}).items():
            if n in [m.joint(i).name for i in range(m.njnt)]:
                d.qpos[m.joint(n).qposadr[0]] = v
        mujoco.mj_kinematics(m, d)
        X = {}
        for name in self.cfg.bodies:
            bid = m.body(name).id
            X[name] = (d.xmat[bid].reshape(3, 3).copy(), d.xpos[bid].copy() * M)
        placed = {}
        for name, part in self.parts.items():
            R, t = X[name]
            placed[name] = part.transform(np.hstack([R, t.reshape(3, 1)]))
        fits = {}
        for j in self.cfg.hydraulic_joints:
            child = self.cfg.bodies[self.cfg.body_of_joint(j.name).name]
            lk, c = linkage_for(self.cfg, j.name, self.sizing)
            Rp, tp = X[child.parent]
            Rc, tc = X[child.name]
            pa = Rp @ ((np.array(child.origin) + lk.parent) * M) + tp
            ca = Rc @ (lk.child * M) + tc
            v = ca - pa
            L = float(np.linalg.norm(v))
            z = v / L
            ax = Rp @ np.array(j.axis_vec)
            x = ax - np.dot(ax, z) * z
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            R = np.column_stack([x, y, z])
            placed[f"cyl_{j.name}"] = self._cyl_solid(c, L).transform(np.hstack([R, pa.reshape(3, 1)]))
            fits[j.name] = dict(L=L, lo=c.l_retracted * M, hi=c.l_extended * M,
                                ok=c.l_retracted * M - 0.5 <= L <= c.l_extended * M + 0.5)
        return placed, fits

    def interferences(self, placed: dict, tol_mm3: float = 2.0, skip_cross_legs: bool = False):
        names = list(placed)
        legs = ("hip_yaw", "hip_roll", "hip_pitch", "thigh", "knee", "shin", "ankle", "foot")
        bbs = {n: np.array(placed[n].bounding_box()) for n in names}
        hits = []
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                a, b = names[i], names[k]
                if skip_cross_legs and any(t in a for t in legs) and any(t in b for t in legs) \
                        and (("L_" in a and "R_" in b) or ("R_" in a and "L_" in b)):
                    continue
                A, B = bbs[a], bbs[b]
                if np.any(A[3:] < B[:3]) or np.any(B[3:] < A[:3]):
                    continue
                vol = (placed[a] ^ placed[b]).volume()
                if vol > tol_mm3:
                    hits.append((a, b, float(vol)))
        return hits


def nominal(cfg):
    from ..sim.scenarios import default_arms
    return {k: v for k, v in default_arms(cfg).items() if "elbow" not in k}


def check_poses(cfg: RobotConfig):
    """Poses: nominal, each joint at both limits (others nominal), squat/stride/arms,
    and every frame sampled from the simulated scenarios."""
    base = nominal(cfg)
    poses = {"zero": dict(base)}
    for j in cfg.joints.values():
        if not j.in_sim:
            continue
        lo, hi = j.range_rad
        poses[f"{j.name}@min"] = base | {j.name: lo}
        poses[f"{j.name}@max"] = base | {j.name: hi}
    for k, v in (cfg.derived("sim_poses") or {}).items():
        poses[f"sim:{k}"] = v
    sq = math.radians(cfg["sim"]["squat_knee_deg"])
    poses["squat"] = base | {f"{s}_{k}": v for s in "LR" for k, v in
                             (("hip_pitch", -0.55 * sq), ("knee", sq), ("ankle_pitch", -0.45 * sq))}
    poses["stride"] = base | {"L_hip_pitch": -0.45, "L_knee": 0.9, "L_ankle_pitch": -0.3,
                       "R_hip_pitch": 0.35, "R_knee": 0.35, "R_ankle_pitch": -0.25,
                       "L_hip_roll": 0.12, "R_hip_roll": 0.12, "L_ankle_roll": -0.12, "R_ankle_roll": -0.12}
    poses["arms_forward"] = base | {f"{s}_shoulder_pitch": -1.5 for s in "LR"} | {f"{s}_elbow": -1.2 for s in "LR"}
    return poses


def run_interference(cfg: RobotConfig, log=print, tol_mm3: float = 2.0):
    asm = Assembly(cfg)
    results = {}
    for pname, q in check_poses(cfg).items():
        placed, fits = asm.pose(q)
        hits = asm.interferences(placed, tol_mm3, skip_cross_legs="@" in pname)
        bad_fit = [k for k, v in fits.items() if not v["ok"]]
        results[pname] = {"hits": hits, "cyl_fit_fail": bad_fit}
        if hits or bad_fit:
            log(f"  [asm] {pname:28s} {len(hits)} interference(s) "
                + "; ".join(f"{a}<->{b} {v:.0f}mm3" for a, b, v in hits[:6])
                + (f" cyl-length out of range: {bad_fit}" if bad_fit else ""))
    return results


def render(cfg: RobotConfig, path: Path, qmap=None, views=((20, -60), (10, 0), (10, 90)), title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    asm = Assembly(cfg)
    placed, _ = asm.pose(qmap)
    fig = plt.figure(figsize=(5 * len(views), 9), dpi=120)
    light = np.array([0.4, -0.5, 0.75])
    light /= np.linalg.norm(light)
    meshes = []
    for n, mm in placed.items():
        tm = to_trimesh(mm.simplify(0.4))
        col = np.array([0.85, 0.45, 0.12]) if n.startswith("cyl_") else (
            np.array([0.35, 0.37, 0.42]) if any(k in n for k in ("foot", "pelvis", "torso", "head")) else np.array([0.62, 0.64, 0.68]))
        meshes.append((tm, col))
    for vi, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), vi + 1, projection="3d")
        for tm, col in meshes:
            tri = tm.vertices[tm.faces]
            nrm = tm.face_normals
            shade = 0.35 + 0.65 * np.clip(nrm @ light, 0, 1)
            pc = Poly3DCollection(tri, facecolors=np.clip(col[None, :] * shade[:, None], 0, 1), linewidths=0)
            ax.add_collection3d(pc)
        ax.set_xlim(-330, 330); ax.set_ylim(-330, 330); ax.set_zlim(0, 1260)
        ax.set_box_aspect((1, 1, 1.9))
        ax.view_init(elev=el, azim=az)
        ax.set_axis_off()
    if title:
        fig.suptitle(title)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.95, wspace=0)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path
