"""Printed structural links, one solid per kinematic body (left side; right = mirror).

Every part is modelled in its body frame (mm) at the zero pose, so the assembly
is obtained by applying the MuJoCo forward-kinematics transforms directly.
"""
from __future__ import annotations

import math

import numpy as np

from ..config import RobotConfig
from ..hydraulics import linkage_for, rot
from .core import along, box, cyl, diff, hull, rbox, union
from .features import JH, Feature, beam_axis, beam_taper, fork, horn_flange, lugs, nearest_on_segments, servo_pocket, tongue

M = 1000.0


def anchors_on(cfg: RobotConfig, sizing: dict, body: str):
    """[(pin_mm, axis, dirs, role, joint)] for every cylinder pin on this body."""
    if body not in cfg.bodies:
        body = "L_" + body
    out = []
    for j in cfg.hydraulic_joints:
        if j.side == "R":
            continue
        child = cfg.body_of_joint(j.name)
        lk, cyl_ = linkage_for(cfg, j.name, sizing)
        lo, hi = j.range_rad
        qs = np.linspace(lo, hi, 7)
        if child.parent == body:
            pin = (np.array(child.origin) + lk.parent) * M
            dirs = [(rot(j.axis, q) @ lk.child - lk.parent) for q in qs]
            out.append((pin, j.axis, dirs, "parent", j.name))
        if child.name == body:
            pin = lk.child * M
            dirs = [(rot(j.axis, q).T @ lk.parent - lk.child) for q in qs]
            out.append((pin, j.axis, dirs, "child", j.name))
    return out


def anchor_lugs(jh: JH, axis, pin, dirs, segs, wall_pt=None, base_dir=None):
    """Lugs whose web sits on the far side of the pin from the cylinder, plus a
    strut from that web to the nearest point of the link skeleton."""
    pin = np.asarray(pin, float)
    ia = "xyz".index(axis)
    if base_dir is None:
        dm = np.mean([d / np.linalg.norm(d) for d in dirs], axis=0)
    else:
        dm = -np.asarray(base_dir, float)
    dm[ia] = 0.0
    dm = dm / max(np.linalg.norm(dm), 1e-9)
    base = pin - dm * (jh.lug_r + 12.0)
    f = lugs(jh, axis, pin, base, dirs)
    tgt = wall_pt if wall_pt is not None else (nearest_on_segments(base, segs) if segs else None)
    extra = []
    if tgt is not None and np.linalg.norm(np.asarray(tgt) - base) > 2:
        w = 2 * jh.lug_t + jh.lug_gap
        extra.append(beam_taper(base, tgt, axis, w, 12.0, 14))
    return f, extra


def _swing(cfg, jname, toward_child=True, n=7):
    """Directions swept by the child link neck in the parent frame (for fork slots)."""
    j = cfg.joints[jname]
    lo, hi = j.range_rad
    base = {"y": np.array([0, 0, -1.0]), "x": np.array([0, 0, -1.0]), "z": np.array([1.0, 0, 0])}[j.axis]
    return [rot(j.axis, q) @ base for q in np.linspace(lo, hi, n)]


def assemble(features: list[Feature], beams: list, anchor_specs, jh: JH, extra_cuts=None, anchor_segs=None):
    """Union features + beams + lugs (attached to nearest skeleton point), then cut."""
    segs = anchor_segs if anchor_segs is not None else [(a, b) for a, b, *_ in beams]
    solids = [f.solid for f in features]
    cuts = [f.cut for f in features if f.cut is not None]
    for a, b, ax, w, t in beams:
        solids.append(beam_axis(a, b, ax, w, t))
    for pin, axis, dirs, role, jn in anchor_specs:
        f, extra = anchor_lugs(jh, axis, pin, dirs, segs)
        solids += [f.solid] + extra
        cuts.append(f.cut)
    if extra_cuts:
        cuts += extra_cuts
    return diff(union(solids), cuts)


# ---------------------------------------------------------------- bodies
def pelvis(cfg: RobotConfig, sizing, jh: JH):
    g = cfg["geometry"]
    sm = cfg["servo_models"]["HV-150kgcm"]
    hy = g["hip_spacing"] / 2 * M
    hd = g["hip_drop"] * M
    zb = -(hd - 36.0)                    # pelvis bottom face (36 mm above the hip roll axis)
    zt = 22.0
    blk = rbox(98, 2 * hy + 96, zt - zb, 14, at=(13, 0, (zt + zb) / 2))
    solids, cuts = [blk], []
    L, W, H = sm["body"]
    b6808 = cfg["hardware"]["bearing_6808"]
    for s in (-1, 1):
        y = s * hy
        # servo pocket: shaft pointing down (-z) at hip yaw axis
        sc = (sm["shaft_offset"], y, zb + 7 + H / 2 + 0.5)
        cuts.append(box(L + 0.6, W + 0.6, H + 30, at=(sc[0], y, sc[2] + 15)))
        cuts.append(box(sm["ear_span"] + 0.6, W + 0.6, 3, at=(sc[0], y, zb + 7 + H - 9)))
        cuts.append(cyl(b6808["od"] / 2 + 0.1, b6808["w"] + 0.01, "z", (0, y, zb + b6808["w"] / 2)))
        cuts.append(cyl(17.0, 30, "z", (0, y, zb + 10)))
        for k in (-1, 1):
            cuts.append(cyl(1.8, 20, "z", (sc[0] + k * (sm["ear_span"] / 2 - 4.5), y, zb + 7 + H - 9 - 6)))
    # torso interface: 6x M4 inserts + central harness/hose pass-through
    for x in (-24, 50):
        for y in (-45, 0, 45):
            cuts.append(cyl(jh.insert_m4, 9, "z", (x, y, zt - 4)))
    cuts.append(rbox(30, 50, 80, 8, at=(13, 0, 0)))      # harness / hose pass-through
    return diff(union(solids), cuts)


def torso(cfg: RobotConfig, sizing, jh: JH):
    g = cfg["geometry"]
    t = g["torso"]
    D, Wd, Ht = t["depth"] * M, t["width"] * M, t["height"] * M
    z0, z1 = 22.0, Ht + 32.0
    wall = 4.0
    shell = rbox(D, Wd, z1 - z0, 16, at=(0, 0, (z0 + z1) / 2))
    inner = rbox(D - 2 * wall, Wd - 2 * wall, z1 - z0 - 2 * wall, 12, at=(0, 0, (z0 + z1) / 2))
    s = shell - inner
    feats, cuts = [], []
    # shelves (battery @ z~80, HPU @ z~170, electronics @ z~260)
    shelves = [box(D - 2, Wd - 2, 4, at=(0, 0, z)) for z in (130.0, 215.0)]
    # front/back service openings (covers are separate parts)
    for sx in (-1, 1):
        cuts.append(rbox(12, Wd - 40, z1 - z0 - 60, 10, at=(sx * D / 2, 0, (z0 + z1) / 2)))
    # side ventilation / hose ports
    for sy in (-1, 1):
        cuts.append(rbox(60, 12, 40, 8, at=(-30, sy * Wd / 2, 60)))
    # cover inserts
    for sx in (-1, 1):
        for zz in (z0 + 20, z1 - 20):
            for yy in (-Wd / 2 + 10, Wd / 2 - 10):
                cuts.append(cyl(jh.insert_m3, 7, "x", (sx * (D / 2 - 3), yy, zz)))
    # bottom bolt holes (to pelvis)
    for x in (-24, 50):
        for y in (-45, 0, 45):
            cuts.append(cyl(2.25, 12, "z", (x, y, z0 + 2)))
    cuts.append(rbox(30, 50, 12, 8, at=(13, 0, z0)))
    # shoulder forks: inner cheek boss from the wall, bridge front-down
    sh_y = g["shoulder_spacing"] / 2 * M
    sh_z = Ht
    sm = cfg["servo_models"]["MG996R"]
    for sgn in (1,):            # left; right side is the mirror of the whole part
        pass
    solids = [s] + shelves
    for sy in (-1, 1):
        at = np.array([0, sy * sh_y, sh_z])
        dp = np.array([-0.5, 0, -0.866])
        f = fork(jh, "y", at, dp, swing_dirs=[rot("y", q) @ np.array([0, 0, 1.0])
                                             for q in np.linspace(*cfg.joints["L_shoulder_pitch"].range_rad, 9)])
        solids.append(f.solid)
        cuts.append(f.cut)
        # bracket from the fork crossbar back to the torso wall
        solids.append(beam_axis(f.base, (f.base[0], sy * (Wd / 2 - 2), f.base[2]), "y", 22, 22))
        solids.append(beam_axis((f.base[0], sy * (Wd / 2 - 2), f.base[2]), (f.base[0], sy * (Wd / 2 - 2), f.base[2] - 60), "y", 8, 22))
    # neck servo pocket on top (yaw shaft up at neck_height)
    nf = servo_pocket(sm, "z", (0, 0, g["neck_height"] * M - 2.0), (1, 0, 0))
    solids.append(nf.solid)
    cuts.append(nf.cut)
    body = union(solids)
    # shoulder pitch cylinder lugs (parent side) on the torso walls
    specs = anchors_on(cfg, sizing, "torso")
    full = []
    for pin, axis, dirs, role, jn in specs:
        for sy in (1, -1):
            p = pin.copy()
            p[1] *= sy
            ds = [np.array([d[0], sy * d[1], d[2]]) for d in dirs]
            f, extra = anchor_lugs(jh, axis, p, ds, [], wall_pt=None)
            wall = np.array([f.base[0], sy * (Wd / 2 - 2), f.base[2]])
            full += [f.solid, beam_axis(f.base, wall, "y", 2 * jh.lug_t + jh.lug_gap, 18)]
            cuts.append(f.cut)
    body = union([body] + full)
    return diff(body, cuts)


def hip_yaw_link(cfg, sizing, jh: JH):
    """Hub in the 6808 bearing + servo horn interface on top, roll fork (x) below."""
    b6808 = cfg["hardware"]["bearing_6808"]
    sm = cfg["servo_models"]["HV-150kgcm"]
    ztop = 36.0                                  # pelvis bottom face in this frame
    hub = cyl(b6808["id"] / 2 - 0.05, b6808["w"] - 0.3, "z", (0, 0, ztop + (b6808["w"] - 0.3) / 2))
    flange = cyl(24.0, 6, "z", (0, 0, ztop - 3.0))
    plate_z = ztop - 6 - 3.5
    fk = fork(jh, "x", (0, 0, 0), (0, 0, 1), bridge_r=plate_z - 4, pad=(30, 14),
              swing_dirs=[rot("x", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_hip_roll"].range_rad, 9)])
    plate = rbox(46, 46, 7, 6, at=(0, 0, plate_z))
    solids = [hub, flange, plate, fk.solid]
    cuts = [fk.cut]
    pcd = sm["horn_d"] * 0.36
    for k in range(4):
        a_ = math.pi / 4 + k * math.pi / 2
        cuts.append(cyl(1.7, 30, "z", (pcd * math.cos(a_), pcd * math.sin(a_), ztop)))
    cuts.append(cyl(6.0, 30, "z", (0, 0, ztop)))
    for pin, axis, dirs, role, jn in anchors_on(cfg, sizing, "hip_yaw_link"):
        f, _ = anchor_lugs(jh, axis, pin, dirs, [])
        b = f.base
        w = 2 * jh.lug_t + jh.lug_gap
        p1 = np.array([b[0], 30.0, b[2]])               # inboard, above the cylinder
        p2 = np.array([b[0], 30.0, plate_z])            # down behind the pelvis
        solids += [f.solid, beam_axis(b, p1, "x", w, 14), beam_axis(p1, p2, "x", 12, 16),
                   beam_axis(p2, (0, 0, plate_z), "z", 7, 16)]
        cuts.append(f.cut)
    return diff(union(solids), cuts)


def hip_roll_link(cfg, sizing, jh: JH):
    """Roll tongue (x) at the top, hip-pitch fork (y) hip_pitch_offset below.

    Cylinder lugs are routed explicitly: out of the cylinder planes, then into
    the pitch-fork cheeks below the yaw-yoke cheeks (verified by the assembly check)."""
    off = cfg["geometry"]["hip_pitch_offset"] * M
    tg = tongue(jh, "x", (0, 0, 0), (0, 0, -1))
    sw = [rot("y", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_hip_pitch"].range_rad, 9)]
    fk = fork(jh, "y", (0, 0, -off), (0, 0, 1), swing_dirs=sw, bridge_r=off - jh.hub_r - 5, pad=(16, 12))
    solids = [tg.solid, fk.solid, beam_axis(tg.base, fk.base, "x", 16, 16)]
    cuts = [tg.cut, fk.cut]
    cheek_y = jh.tongue_w / 2 + jh.gap + jh.cheek_t / 2
    for pin, axis, dirs, role, jn in anchors_on(cfg, sizing, "hip_roll_link"):
        f, _ = anchor_lugs(jh, axis, pin, dirs, [])
        b = f.base
        sy = np.sign(b[1]) if abs(b[1]) > 1 else 1.0
        end = np.array([-12.0, sy * cheek_y, -off + 8])          # inside the pitch cheek
        if axis == "y":       # hip-pitch parent: step inboard of the cylinder plane, then down & forward
            y_in = sy * (abs(pin[1]) - (jh.lug_gap / 2 + jh.lug_t) - 2)
            p1 = np.array([b[0], y_in, b[2]])
            p2 = np.array([b[0], sy * (cheek_y + 4), end[2]])
            path = [(b, p1, "y", 2 * jh.lug_t + jh.lug_gap, 14), (p1, p2, "z", 12, 14), (p2, end, "z", 10, 14)]
        else:                 # hip-roll child (pin axis x): run low and forward, then up into the cheek
            p1 = np.array([end[0], b[1], b[2]])
            path = [(b, p1, "x", 2 * jh.lug_t + jh.lug_gap, 14), (p1, end, "x", 10, 14)]
        solids.append(f.solid)
        cuts.append(f.cut)
        for a_, b_, ax_, w_, t_ in path:
            solids.append(beam_axis(a_, b_, ax_, w_, t_))
    return diff(union(solids), cuts)


def _limb(cfg, sizing, jh, name, length, in_axis, out_axis, w=30, t=38, out_dir_parent=(0, 0, 1)):
    tg = tongue(jh, in_axis, (0, 0, 0), (0, 0, -1))
    fk = fork(jh, out_axis, (0, 0, -length), out_dir_parent, swing_dirs=None)
    beams = [(tg.base, fk.base, in_axis, w, t)]
    return assemble([tg, fk], beams, anchors_on(cfg, sizing, name), jh)


def thigh(cfg, sizing, jh):
    L = cfg["geometry"]["thigh_length"] * M
    sw = [rot("y", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_knee"].range_rad, 9)]
    tg = tongue(jh, "y", (0, 0, 0), (0, 0, -1))
    fk = fork(jh, "y", (0, 0, -L), (0, 0, 1), swing_dirs=sw)
    beams = [(tg.base, fk.base, "y", 16, 26)]
    beams.append(((0, 0, tg.base[2] - 45), (0, 0, fk.base[2]), "y", 30, 34))
    return assemble([tg, fk], beams, anchors_on(cfg, sizing, "thigh"), jh)


def shin(cfg, sizing, jh):
    L = cfg["geometry"]["shin_length"] * M
    sw = [rot("y", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_ankle_pitch"].range_rad, 9)]
    tg = tongue(jh, "y", (0, 0, 0), (0, 0, -1))
    fk = fork(jh, "y", (0, 0, -L), (0, 0, 1), swing_dirs=sw)
    beams = [(tg.base, fk.base, "y", 16, 38), ((0, 0, tg.base[2] - 25), (0, 0, fk.base[2]), "y", 28, 34)]
    return assemble([tg, fk], beams, anchors_on(cfg, sizing, "shin"), jh)


def ankle_link(cfg, sizing, jh):
    off = cfg["geometry"]["ankle_roll_offset"] * M
    tg = tongue(jh, "y", (0, 0, 0), (0, 0, -1))
    sw = [rot("x", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_ankle_roll"].range_rad, 9)]
    fk = fork(jh, "x", (0, 0, -off), (0, 0, 1), swing_dirs=sw, encoder=True, bridge_r=24.0, pad=(16, 12))
    beams = [(tg.base, fk.base, "y", 16, 24)]
    specs = anchors_on(cfg, sizing, "ankle_link")
    solids, cuts = [tg.solid, fk.solid], [tg.cut, fk.cut]
    sk = [(tg.base, fk.base)]
    for pin, axis, dirs, role, jn in specs:
        bd = None if jn.endswith("ankle_roll") else -np.array([pin[0], 0, pin[2] + 20.0])
        low = [(np.array([0, 0, -off + 5.0]), np.array([0, 0, -off + 5.0]))]   # below the shin-fork cheeks
        f, extra = anchor_lugs(jh, axis, pin, dirs, sk if bd is None else low, base_dir=bd)
        cuts.append(f.cut)
        solids.append(f.solid)
        if jn.endswith("ankle_roll"):
            # yoke routed in front of the roll cylinder: tongue -> forward arm -> outboard -> up
            c = linkage_for(cfg, jn, sizing)[1]
            sg = 1.0 if pin[0] >= 0 else -1.0
            xs = pin[0] + sg * (c.r_env * M + 11.0)
            p1 = np.array([sg * 30.0, 0, tg.base[2] + 8])
            p2 = np.array([xs, 0, p1[2]])
            p3 = np.array([xs, pin[1], p1[2]])
            p4 = np.array([xs, f.base[1], f.base[2]])
            for a_, b_, ax_, w_ in ((tg.base, p1, "y", 16), (p1, p2, "y", 16), (p2, p3, "z", 14), (p3, p4, "z", 14)):
                solids.append(beam_axis(a_, b_, ax_, w_, 16))
            solids.append(beam_axis(p4, f.base, "x", 2 * jh.lug_t + jh.lug_gap, 14))
        else:
            solids += extra
    for a_, b_, ax_, w_, t_ in beams:
        solids.append(beam_axis(a_, b_, ax_, w_, t_))
    return diff(union(solids), cuts)


def foot(cfg, sizing, jh):
    g = cfg["geometry"]
    f = g["foot"]
    ah = g["ankle_height"] * M
    tg = tongue(jh, "x", (0, 0, 0), (0, 0, -1))
    Lf, Wf, Tf = f["length"] * M, f["width"] * M, f["thickness"] * M
    cx = Lf / 2 - f["heel"] * M
    plate = rbox(Lf, Wf, Tf, 18, at=(cx, 0, -ah + Tf / 2))
    rib = rbox(70, 16, ah - Tf - 6, 4, at=(0, 0, -(ah + Tf) / 2 + 3))
    feats = [tg, Feature(plate, None, np.array([0, 0, -ah + Tf])), Feature(rib, None, np.zeros(3))]
    cuts = []
    # FSR pockets under the TPU pad (4 per foot) + wire channel
    for x in (-f["heel"] * M + 22, Lf - f["heel"] * M - 22):
        for y in (-Wf / 2 + 20, Wf / 2 - 20):
            cuts.append(cyl(7.0, 2.0, "z", (x, y, -ah + 0.99)))
    cuts.append(box(Lf - 60, 4, 2.0, at=(cx, 0, -ah + 0.99)))
    cuts.append(box(4, Wf - 40, 2.0, at=(-f["heel"] * M + 22, 0, -ah + 0.99)))
    cuts.append(box(4, Wf - 40, 2.0, at=(Lf - f["heel"] * M - 22, 0, -ah + 0.99)))
    # lightening pockets from top
    for x in (cx - 55, cx + 55):
        cuts.append(rbox(50, Wf - 34, Tf - 6, 8, at=(x, 0, -ah + Tf / 2 + 4)))
    beams = [(tg.base, (0, 0, -ah + Tf), "x", 16, 30)]
    specs = anchors_on(cfg, sizing, "foot")
    for pin, axis, dirs, role, jn in specs:
        # keep the post's faces off the plate's side faces (coplanar faces -> non-manifold STL edges)
        py = float(np.sign(pin[1]) * min(abs(pin[1]), Wf / 2 - 10.0))
        beams.append(((pin[0], py, -ah + Tf), (pin[0], py, pin[2] - 8), "x", 14, 14))
        beams.append(((pin[0], py, pin[2] - 8), (pin[0], pin[1], pin[2] - 8), "x", 14, 14))
    return assemble(feats, beams, specs, jh, cuts)


def foot_pad(cfg):
    g = cfg["geometry"]
    f = g["foot"]
    Lf, Wf = f["length"] * M, f["width"] * M
    pad = rbox(Lf, Wf, 3.0, 18, at=(Lf / 2 - f["heel"] * M, 0, 1.5))
    tread = [box(Lf - 20, 1.2, 1.0, at=(Lf / 2 - f["heel"] * M, y, 0.5)) for y in np.arange(-Wf / 2 + 12, Wf / 2 - 10, 9)]
    return diff(pad, tread)


def shoulder_link(cfg, sizing, jh):
    sm = cfg["servo_models"]["HV-60kgcm"]
    ry = cfg.bodies["L_upper_arm_roll"].origin[1] * M
    tg = tongue(jh, "y", (0, 0, 0), (0, 0, 1))
    top_z = jh.hub_r + 14
    bridge_a = np.array([0, 0, top_z])
    bridge_b = np.array([0, ry, top_z])
    sp = servo_pocket(sm, "x", (14.0, ry, 0.0), (0, 0, -1))
    beams = [(tg.base, bridge_a, "y", 16, 20), (bridge_a, bridge_b, "z", 14, 26), (bridge_b, sp.base, "z", 14, 22)]
    specs = anchors_on(cfg, sizing, "shoulder_link")
    for pin, axis, dirs, role, jn in specs:
        beams.append((bridge_a, (pin[0] * 0.3, pin[1], top_z), "z", 12, 18))
    return assemble([tg, sp], beams, specs, jh)


def upper_arm_roll(cfg, sizing, jh):
    sm = cfg["servo_models"]["HV-60kgcm"]
    sy = cfg["servo_models"]["HV-35kgcm"]
    hf = horn_flange("x", (14.0 + 1.0, 0, 0), sm)
    drop = cfg.bodies["L_upper_arm"].origin[2] * M
    sp = servo_pocket(sy, "z", (0, 0, drop + 1.0), (1, 0, 0), out_sign=-1)
    beams = [(hf.base, (hf.base[0], 0, drop + 22), "x", 6, 24)]
    return assemble([hf, sp], beams, [], jh)


def upper_arm(cfg, sizing, jh):
    sy = cfg["servo_models"]["HV-35kgcm"]
    L = -cfg.bodies["L_forearm"].origin[2] * M
    hf = horn_flange("z", (0, 0, -1.0), sy, out_sign=-1)
    sw = [rot("y", q) @ np.array([0, 0, -1.0]) for q in np.linspace(*cfg.joints["L_elbow"].range_rad, 9)]
    fk = fork(jh, "y", (0, 0, -L), (0, 0, 1), swing_dirs=sw)
    beams = [(hf.base, fk.base, "y", 26, 30)]
    return assemble([hf, fk], beams, anchors_on(cfg, sizing, "upper_arm"), jh)


def forearm(cfg, sizing, jh):
    L = cfg["geometry"]["forearm_length"] * M
    tg = tongue(jh, "y", (0, 0, 0), (0, 0, -1))
    fl = rbox(44, 44, 6, 6, at=(0, 0, -L + 3))
    feats = [tg, Feature(fl, union([cyl(1.7, 10, "z", (sx * 15, sy * 15, -L + 3)) for sx in (-1, 1) for sy in (-1, 1)]),
                         np.array([0, 0, -L + 6]))]
    beams = [(tg.base, (0, 0, -L + 6), "y", 16, 26)]
    return assemble(feats, beams, anchors_on(cfg, sizing, "forearm"), jh)


def hand(cfg, sizing, jh):
    sm = cfg["servo_models"]["MG996R"]
    palm = rbox(44, 44, 70, 8, at=(0, 0, -36.5))
    sp = servo_pocket(sm, "x", (24.0, 0, -50.0), (0, 0, 1))
    finger = hull(box(10, 40, 6, at=(-16, 0, -70)), box(8, 30, 6, at=(-10, 0, -118)))
    pad = box(4, 30, 30, at=(-8, 0, -104))
    cuts = [cyl(1.7, 10, "z", (sx * 15, sy * 15, -3)) for sx in (-1, 1) for sy in (-1, 1)]
    cuts.append(rbox(30, 30, 40, 6, at=(-4, 0, -31.5)))
    return diff(union(palm, sp.solid, finger, pad), sp.cut, cuts)


def finger(cfg, sizing, jh):
    sm = cfg["servo_models"]["MG996R"]
    hf = horn_flange("x", (0, 0, 0), sm)
    arm = hull(box(5, 30, 10, at=(hf.base[0], 0, 0)), box(6, 30, 8, at=(hf.base[0] - 30, 0, -58)))
    return diff(union(hf.solid, arm), hf.cut)


def neck_link(cfg, sizing, jh):
    sm = cfg["servo_models"]["MG996R"]
    hf = horn_flange("z", (0, 0, 0), sm)
    sp = servo_pocket(sm, "y", (0, 12.0, cfg.bodies["head"].origin[2] * M), (1, 0, 0))
    return assemble([hf, sp], [(hf.base, sp.base, "z", 10, 24)], [], jh)


def head(cfg, sizing, jh):
    sm = cfg["servo_models"]["MG996R"]
    h = [v * M for v in cfg["geometry"]["head"]["size"]]
    hf = horn_flange("y", (0, 12.5, 0), sm)
    z0 = 34.0                                      # shell bottom clears the neck servo when pitching
    shell = rbox(h[0], h[1], h[2], 20, at=(0, 0, h[2] / 2 + z0)) - rbox(h[0] - 6, h[1] - 6, h[2] - 6, 17, at=(0, 0, h[2] / 2 + z0))
    eyes = [cyl(9.0, 10, "x", (h[0] / 2, s * 30, h[2] * 0.6 + z0)) for s in (-1, 1)]
    cam = rbox(30, 30, 8, 3, at=(h[0] / 2, 0, h[2] * 0.75 + z0), axis="x")
    vents = [box(4, h[1] - 40, 3, at=(-h[0] / 2, 0, z)) for z in np.arange(z0 + 26, z0 + h[2], 12)]
    arm = hull(box(20, 6, 20, at=(0, hf.base[1] + 3, 0)), box(40, 6, 6, at=(0, hf.base[1] + 3, z0 + 3)))
    cable = box(30, 20, 10, at=(0, -20, z0))
    return diff(union(shell, arm, hf.solid), hf.cut, eyes, cam, vents, cable)


def torso_cover(cfg, sizing, jh):
    t = cfg["geometry"]["torso"]
    Wd, Ht = t["width"] * M, t["height"] * M
    h = Ht + 32 - 22 - 20
    plate = rbox(3.0, Wd - 14, h, 10, at=(0, 0, 0), axis="x")
    holes = [cyl(1.7, 6, "x", (0, yy, zz)) for yy in (-Wd / 2 + 10, Wd / 2 - 10) for zz in (-h / 2 + 10, h / 2 - 10)]
    vents = [box(6, Wd - 70, 3, at=(0, 0, z)) for z in np.arange(-h / 2 + 40, h / 2 - 40, 14)]
    return diff(plate, holes, vents)


BODY_BUILDERS = {
    "pelvis": pelvis, "torso": torso, "hip_yaw_link": hip_yaw_link, "hip_roll_link": hip_roll_link,
    "thigh": thigh, "shin": shin, "ankle_link": ankle_link, "foot": foot,
    "shoulder_link": shoulder_link, "upper_arm_roll": upper_arm_roll, "upper_arm": upper_arm,
    "forearm": forearm, "hand": hand, "neck_link": neck_link, "head": head,
}
