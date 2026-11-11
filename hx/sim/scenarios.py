"""Whole-body simulation scenarios -> joint demand envelopes.

Each scenario creates a reference motion with the whole-body IK, then runs the
floating-base MuJoCo model with stiff joint servos (idealised actuators) and
records q, qdot and actuator torque.  The torque envelope is the *demand* the
hydraulic and servo actuators are sized against.  Fall / slip / tracking
metrics are QA evidence that the gait itself is physically viable.
"""
from __future__ import annotations

import math
import time

import mujoco
import numpy as np

from ..config import RobotConfig
from .gait import GaitParams, plan
from .ik import WholeBodyIK
from .model import build_mjcf

REF_DT = 0.005


def default_arms(cfg: RobotConfig) -> dict:
    a = {}
    for s, sg in (("L", 1), ("R", -1)):
        a[f"{s}_shoulder_roll"] = sg * 0.16   # hands clear the hip-roll brackets (QA CAD-5)
        a[f"{s}_elbow"] = -0.25
        a[f"{s}_shoulder_pitch"] = 0.05
        a[f"{s}_shoulder_yaw"] = 0.0
    return a


class Scenario:
    name = "base"
    payload = 0.0

    def __init__(self, cfg: RobotConfig, meshes: dict | None = None):
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(cfg, self.payload, meshes=meshes))
        self.ik = WholeBodyIK(self.model)
        self.w = cfg["sim"]["gait"]["step_width"] / 2
        self.z0 = cfg["sim"]["standing_pelvis_height"]
        self.com_x0 = 0.015

    def frames(self):
        """Yield (t, feet{L,R}, com_xy, pelvis_z, fixed_joints)."""
        raise NotImplementedError

    def reference(self):
        m = self.model
        q = m.qpos0.copy()
        q[2] = self.z0
        refs, ts, errs, coms = [], [], [], []
        for t, feet, com, pz, fixed in self.frames():
            coms.append([float(com[0]), float(com[1])])
            q, err = self.ik.solve(q, feet, com, pz, fixed, iters=40 if not refs else 12)
            refs.append(q.copy())
            ts.append(t)
            errs.append(err)
        self.com_ref = np.array(coms)           # planned CoM (x, y) -> stabilizer
        return np.array(ts), np.array(refs), float(np.max(errs))


class Stand(Scenario):
    name = "stand"

    def frames(self):
        arms = default_arms(self.cfg)
        for t in np.arange(0, 3.0, REF_DT):
            yield t, {"L": [0, self.w, 0], "R": [0, -self.w, 0]}, [self.com_x0, 0], self.z0, arms


class Squat(Scenario):
    name = "squat"

    def frames(self):
        g = self.cfg["geometry"]
        kd = math.radians(self.cfg["sim"]["squat_knee_deg"])
        z_low = g["hip_drop"] + g["hip_pitch_offset"] + 2 * g["thigh_length"] * math.cos(kd / 2) + g["ankle_roll_offset"] + g["ankle_height"]
        arms = default_arms(self.cfg)
        for t in np.arange(0, 7.0, REF_DT):
            s = 0.0 if t < 1 else (0.5 - 0.5 * math.cos(math.pi * min(1, (t - 1) / 2.5)) if t < 3.5 else
                                   0.5 + 0.5 * math.cos(math.pi * min(1, (t - 3.5) / 2.5)))
            pz = self.z0 + (z_low - self.z0) * s
            yield t, {"L": [0, self.w, 0], "R": [0, -self.w, 0]}, [self.com_x0, 0], pz, arms


class Walk(Scenario):
    name = "walk"

    def frames(self):
        p = GaitParams.from_cfg(self.cfg)
        p.dt = REF_DT
        # LIPM height = CoM height in the standing pose
        q, _ = self.ik.solve(self.model.qpos0.copy(), {"L": [0, self.w, 0], "R": [0, -self.w, 0]},
                             [self.com_x0, 0], self.z0, default_arms(self.cfg), iters=60)
        zc = self.ik.com(q)[2]
        r = plan(p, zc, com0=(self.com_x0, 0.0))
        self.plan = r
        arms = default_arms(self.cfg)
        for k, t in enumerate(r["t"]):
            feet = {"L": r["foot_L"][k], "R": r["foot_R"][k]}
            yield t, feet, r["com"][k], self.z0, arms


class SingleLeg(Scenario):
    name = "single_leg"

    def frames(self):
        arms = default_arms(self.cfg)
        for t in np.arange(0, 8.0, REF_DT):
            shift = 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 0.5) / 1.5, 0, 1))
            back = 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 6.0) / 1.5, 0, 1))
            s = shift - back
            lift = 0.03 * (0.5 - 0.5 * math.cos(math.pi * np.clip((t - 2.2) / 0.6, 0, 1)))
            lift -= 0.03 * (0.5 - 0.5 * math.cos(math.pi * np.clip((t - 4.8) / 0.6, 0, 1)))
            com = [self.com_x0 + 0.015 * s, self.w * 0.97 * s]
            yield t, {"L": [0, self.w, 0], "R": [0, -self.w, max(0.0, lift)]}, com, self.z0, arms


class ArmLift(Scenario):
    name = "arm_lift"

    def __init__(self, cfg, meshes=None):
        self.payload = cfg["sim"]["payload_per_hand_kg"]
        super().__init__(cfg, meshes)

    def frames(self):
        sp = math.radians(self.cfg["sim"]["arm_lift_shoulder_deg"])
        for t in np.arange(0, 8.0, REF_DT):
            a = default_arms(self.cfg)
            up = 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 0.5) / 2.0, 0, 1))
            dn = 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 6.0) / 1.8, 0, 1))
            el = 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 3.0) / 1.0, 0, 1))
            el -= 0.5 - 0.5 * math.cos(math.pi * np.clip((t - 4.5) / 1.0, 0, 1))
            for s in ("L", "R"):
                a[f"{s}_shoulder_pitch"] = sp * (up - dn)
                a[f"{s}_elbow"] = -0.25 - 1.2 * el
            yield t, {"L": [0, self.w, 0], "R": [0, -self.w, 0]}, [self.com_x0, 0], self.z0, a


SCENARIOS = [Stand, Squat, Walk, SingleLeg, ArmLift]


def run(sc: Scenario, log=print, frame_cb=None, frame_dt: float = 1 / 30):
    """frame_cb(t, model, data) is called every frame_dt of simulated time (video capture)."""
    cfg = sc.cfg
    m = sc.model
    d = mujoco.MjData(m)
    t0 = time.time()
    tr, qr, ik_err = sc.reference()
    pd = cfg["sim"]["pd"]
    joints = [m.actuator(i).name for i in range(m.nu)]
    qadr = np.array([m.joint(n).qposadr[0] for n in joints])
    vadr = np.array([m.joint(n).dofadr[0] for n in joints])
    leg = np.array([any(k in n for k in ("hip", "knee", "ankle")) for n in joints])
    kp = np.where(leg, pd["kp"] * 3.0, pd["kp"] * 0.8)
    kd = np.where(leg, pd["kd"] * 2.0, pd["kd"] * 0.6)
    # joint reference & velocity at REF_DT
    qj = qr[:, qadr]
    vj = np.gradient(qj, REF_DT, axis=0)
    # settle
    d.qpos[:] = qr[0]
    d.qpos[2] += 0.004
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    settle = 0.6
    nsteps = int((tr[-1] + settle) / dt)
    rec_every = int(REF_DT / dt)
    rec = {n: {"q": [], "qdot": [], "tau": []} for n in joints}
    frames = []
    fell = False
    max_tilt = 0.0
    max_err = np.zeros(len(joints))
    pel = m.body("pelvis").id
    slip = 0.0
    feet_ids = {s: m.site(f"{s}_foot_sole").id for s in ("L", "R")}
    next_frame = 0.0
    from .stabilizer import Stabilizer, StabilizerGains, pelvis_roll
    stab = Stabilizer(StabilizerGains.from_cfg(cfg), {n: i for i, n in enumerate(joints)})
    com_ref = sc.com_ref
    com_vref = np.gradient(com_ref, REF_DT, axis=0)
    for k in range(nsteps):
        tt = max(0.0, k * dt - settle)
        i = min(int(tt / REF_DT), len(tr) - 2)
        a = (tt - tr[i]) / REF_DT
        qd = qj[i] * (1 - a) + qj[i + 1] * a
        vd = vj[i] * (1 - a) + vj[i + 1] * a
        q = d.qpos[qadr]
        # position actuators: tau = kp (ctrl - q) - kv qdot ; add velocity feed-forward via ctrl lead
        # balance stabilizer (attitude + CoM feedback), identical to the brain's
        mujoco.mj_comPos(m, d)
        mujoco.mj_subtreeVel(m, d)
        j_ = min(i, len(com_ref) - 1)
        corr = stab.corrections(pelvis_roll(d.xmat[pel]), d.qvel[3], d.subtree_com[pel][:2] - com_ref[j_],
                                d.subtree_linvel[pel][:2] - com_vref[j_], len(joints))
        d.ctrl[:] = qd + corr + (kd / kp) * vd
        mujoco.mj_step(m, d)
        if frame_cb is not None and k * dt >= next_frame:
            frame_cb(k * dt, m, d)
            next_frame += frame_dt
        if k * dt >= settle and k % rec_every == 0:
            for jj, n in enumerate(joints):
                rec[n]["q"].append(float(d.qpos[qadr[jj]]))
                rec[n]["qdot"].append(float(d.qvel[vadr[jj]]))
                rec[n]["tau"].append(float(d.actuator_force[jj]))
            max_err = np.maximum(max_err, np.abs(qd - q))
            if (k * dt - settle) % 0.5 < dt:
                frames.append({n: round(float(d.qpos[qadr[jj]]), 4) for jj, n in enumerate(joints)})
            R = d.xmat[pel].reshape(3, 3)
            tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
            max_tilt = max(max_tilt, tilt)
            if d.xpos[pel][2] < 0.45 or tilt > 35:
                fell = True
                break
    # final position error vs plan (walk) -> slip / drift
    final_err = None
    if sc.name == "walk":
        com_plan = sc.plan["com"][-1]
        mujoco.mj_comPos(m, d)
        final_err = float(np.linalg.norm(d.subtree_com[pel][:2] - com_plan))
    summary = dict(
        scenario=sc.name, fell=fell, duration=float(tr[-1]), max_tilt_deg=max_tilt,
        ik_residual=ik_err, final_com_error_m=final_err, payload_kg=sc.payload,
        mass_kg=float(sum(m.body_mass)), wall_s=time.time() - t0,
        max_tracking_err_deg={n: math.degrees(float(e)) for n, e in zip(joints, max_err)},
        peak_tau={n: float(np.max(np.abs(r["tau"]))) if r["tau"] else 0.0 for n, r in rec.items()},
        peak_qdot={n: float(np.max(np.abs(r["qdot"]))) if r["qdot"] else 0.0 for n, r in rec.items()},
        frames=frames,
    )
    log(f"  [sim] {sc.name:10s} fell={fell}  tilt={max_tilt:4.1f} deg  ik={ik_err:.1e}  "
        f"mass={summary['mass_kg']:.1f} kg  ({summary['wall_s']:.0f} s)")
    return rec, summary


def run_all(cfg: RobotConfig, log=print, only: list[str] | None = None):
    demand: dict[str, dict] = {}
    summaries = {}
    series = {}
    for S in SCENARIOS:
        if only and S.name not in only:
            continue
        sc = S(cfg)
        rec, summ = run(sc, log)
        summaries[sc.name] = summ
        series[sc.name] = rec
        for n, r in rec.items():
            dd = demand.setdefault(n, {"q": [], "qdot": [], "tau": []})
            for k in ("q", "qdot", "tau"):
                dd[k] += r[k]
    return demand, summaries, series
