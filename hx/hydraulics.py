"""Hydraulic actuator sizing: tie-rod cylinders + joint linkage synthesis.

For every hydraulic joint we pick the smallest catalogue bore and the anchor
points (cylinder pin locations on the parent and child links) such that

* the cylinder's physical retracted/extended pin-to-pin lengths cover the
  geometric length range over the full joint range (stroke & dead length),
* torque capacity >= safety_factor * demand at every sampled (q, tau) pair
  coming from the whole-body simulation,
* torque capacity >= range_floor * peak demand everywhere in the joint range,
* moment arm never drops below min_moment_arm,
* the cylinder envelope keeps clear of the joint hardware and of the link
  beams when it lives in the link plane.

Sign convention: positive cylinder force = extension (pushes pins apart).
Generalised joint torque produced: tau = F * dL/dq.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from .config import PLANE, Joint, RobotConfig

MM = 1e-3


# ----------------------------------------------------------------- cylinder
@dataclass
class Cylinder:
    bore_mm: float
    rod_mm: float
    tube_od_mm: float
    stroke_mm: float
    dead_mm: float          # retracted pin-to-pin length minus stroke

    @property
    def a_bore(self) -> float:
        return math.pi / 4 * (self.bore_mm * MM) ** 2

    @property
    def a_annulus(self) -> float:
        return self.a_bore - math.pi / 4 * (self.rod_mm * MM) ** 2

    @property
    def l_retracted(self) -> float:
        return (self.dead_mm + self.stroke_mm) * MM

    @property
    def l_extended(self) -> float:
        return (self.dead_mm + 2 * self.stroke_mm) * MM

    @property
    def cap_od_mm(self) -> float:
        return round(self.tube_od_mm + 18, 1)          # M3 tie-rod bolt circle + wall

    @property
    def tie_rod_circle_mm(self) -> float:
        return round(self.tube_od_mm + 8, 1)

    @property
    def r_env(self) -> float:
        return self.cap_od_mm / 2 * MM

    @property
    def tube_length_mm(self) -> float:
        return self.stroke_mm + 14.0 + 2.0          # piston + cushion

    def swept_volume(self) -> float:
        return self.a_bore * self.stroke_mm * MM

    def mass(self, cfg: RobotConfig) -> float:
        cc = cfg.hyd["cylinder_construction"]
        rho_al = cc["tube_density_g_cm3"] * 1000
        rho_st = cc["rod_density_g_cm3"] * 1000
        lt = self.tube_length_mm * MM
        tube = math.pi / 4 * ((self.tube_od_mm * MM) ** 2 - (self.bore_mm * MM) ** 2) * lt * rho_al
        rod_len = (self.stroke_mm + cc["piston_width"] + cc["cap_thickness"] + 25) * MM
        rod = math.pi / 4 * (self.rod_mm * MM) ** 2 * rod_len * rho_st
        caps = 2 * math.pi / 4 * (self.cap_od_mm * MM) ** 2 * cc["cap_thickness"] * MM * 1150 * 0.85
        lug = 0.25 * caps
        tie = cc["tie_rods"] * math.pi / 4 * (3 * MM) ** 2 * (lt + 2 * cc["cap_thickness"] * MM + 0.012) * rho_st
        oil = self.a_bore * lt * cfg.hyd["fluid"]["density"]
        fixed = 0.030 + 0.008 + 2 * 0.015 + 0.012   # rod end, spherical brg, 2 fittings, seals/nuts
        return tube + rod + caps + lug + tie + oil + fixed


def dead_length_mm(cfg: RobotConfig) -> float:
    cc = cfg.hyd["cylinder_construction"]
    rod_protrusion = 15.0                     # wiper + jam nut clearance
    return (cc["blind_lug"] + 2 * cc["cap_thickness"] + cc["piston_width"] + 2.0
            + rod_protrusion + (cc["rod_end_c2c"] - cc["rod_thread_engagement"]))


def make_cylinder(cfg: RobotConfig, bore: int, stroke_mm: float) -> Cylinder:
    rod, od = cfg.hyd["cylinder_catalog"][bore]
    return Cylinder(bore_mm=float(bore), rod_mm=float(rod), tube_od_mm=float(od),
                    stroke_mm=float(stroke_mm), dead_mm=dead_length_mm(cfg))


# ------------------------------------------------------------------ linkage
def rot(axis: str, q):
    """Rotation matrices (..., 3, 3) about a principal axis."""
    q = np.asarray(q, dtype=float)
    c, s = np.cos(q), np.sin(q)
    o, z = np.ones_like(q), np.zeros_like(q)
    if axis == "x":
        m = [[o, z, z], [z, c, -s], [z, s, c]]
    elif axis == "y":
        m = [[c, z, s], [z, o, z], [-s, z, c]]
    else:
        m = [[c, -s, z], [s, c, z], [z, z, o]]
    return np.moveaxis(np.array(m), (0, 1), (-2, -1))


def plane_point(axis: str, offset: float, u: float, v: float) -> np.ndarray:
    p = np.zeros(3)
    i, j = PLANE[axis]
    p[i], p[j] = u, v
    p["xyz".index(axis)] = offset
    return p


@dataclass
class Linkage:
    axis: str
    parent: np.ndarray     # anchor in joint frame (parent side, fixed)
    child: np.ndarray      # anchor in joint frame at q = 0 (moves with child)

    def child_at(self, q) -> np.ndarray:
        return rot(self.axis, q) @ self.child

    def length(self, q) -> np.ndarray:
        q = np.atleast_1d(q)
        c = np.einsum("nij,j->ni", rot(self.axis, q), self.child)
        return np.linalg.norm(self.parent[None, :] - c, axis=1)

    def moment_arm(self, q) -> np.ndarray:
        """dL/dq (m/rad), analytic."""
        q = np.atleast_1d(q)
        R = rot(self.axis, q)
        c = np.einsum("nij,j->ni", R, self.child)
        a = np.array(np.eye(3)["xyz".index(self.axis)])
        dc = np.cross(a[None, :], c)               # d(Rc)/dq = a x (Rc)
        d = c - self.parent[None, :]
        L = np.linalg.norm(d, axis=1)
        return np.einsum("ni,ni->n", d, dc) / L

    def to_dict(self):
        return {"axis": self.axis, "parent": self.parent.tolist(), "child": self.child.tolist()}

    @staticmethod
    def from_dict(d):
        return Linkage(d["axis"], np.array(d["parent"]), np.array(d["child"]))


def torque_capacity(link: Linkage, cyl: Cylinder, dp: float, q):
    """(tau_pos_max, tau_neg_max) >= 0 at angles q."""
    m = link.moment_arm(q)
    fe, fr = dp * cyl.a_bore, dp * cyl.a_annulus
    pos = np.where(m > 0, fe, fr) * np.abs(m)
    neg = np.where(m > 0, fr, fe) * np.abs(m)
    return pos, neg


def _seg_point_dist(a, b, p):
    """Distance from points p (n,3) to segments a->b (n,3)."""
    ab = b - a
    t = np.clip(np.einsum("ni,ni->n", p - a, ab) / np.maximum(np.einsum("ni,ni->n", ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(a + ab * t[:, None] - p, axis=1)


def _seg_seg_dist(p1, q1, p2, q2, samples=9):
    """Approximate segment-segment distance by sampling segment 1 (vectorised over n)."""
    ts = np.linspace(0, 1, samples)
    best = np.full(p1.shape[0], np.inf)
    for t in ts:
        pt = p1 + (q1 - p1) * t
        best = np.minimum(best, _seg_point_dist(p2, q2, pt))
    return best


def barrel_segment(link: Linkage, cyl: Cylinder, q, cfg: RobotConfig):
    """Return (start, end) of the cylinder *body* (caps+tube) — rod side excluded."""
    cc = cfg.hyd["cylinder_construction"]
    q = np.atleast_1d(q)
    c = np.einsum("nij,j->ni", rot(link.axis, q), link.child)
    p = np.repeat(link.parent[None, :], len(q), 0)
    d = c - p
    L = np.linalg.norm(d, axis=1, keepdims=True)
    u = d / L
    s0 = cc["blind_lug"] * MM
    body = (2 * cc["cap_thickness"] + cyl.tube_length_mm) * MM
    return p + u * s0, p + u * (s0 + body)


# ------------------------------------------------------------- optimisation
def _keepouts(joint: Joint, cfg: RobotConfig):
    """Link beam capsules in the joint frame: list of (a, b, r, moves_with_child)."""
    g = cfg["geometry"]
    r = g["link_radius"]
    base = joint.base
    lst = []
    if joint.axis == "y" and abs(joint.hydraulic.plane_offset) < 0.055:
        if base == "knee":
            lst += [((0, 0, 0.0), (0, 0, g["thigh_length"]), r, False),
                    ((0, 0, 0.0), (0, 0, -g["shin_length"]), r, True)]
        if base == "elbow":
            lst += [((0, 0, 0.0), (0, 0, 0.18), r, False),
                    ((0, 0, 0.0), (0, 0, -g["forearm_length"]), r, True)]
        if base == "hip_pitch":
            lst += [((0, 0, 0.0), (0, 0, -g["thigh_length"]), r, True)]
        if base == "ankle_pitch":
            lst += [((0, 0, 0.0), (0, 0, g["shin_length"]), r, False)]
    return [(np.array(a, float), np.array(b, float), rr, mv) for a, b, rr, mv in lst]


def evaluate(joint: Joint, cfg: RobotConfig, x, bore: int, demand: dict | None,
             n_range: int = 61, detail: bool = False):
    """Score a candidate (x = [pu, pv, cu, cv]).  Returns (penalty, info)."""
    hs = cfg.hyd["sizing"]
    off = joint.hydraulic.plane_offset
    link = Linkage(joint.axis, plane_point(joint.axis, off, x[0], x[1]),
                   plane_point(joint.axis, off, x[2], x[3]))
    lo, hi = joint.range_rad
    qr = np.linspace(lo, hi, n_range)
    L = link.length(qr)
    lmin, lmax = float(L.min()), float(L.max())
    dead = dead_length_mm(cfg)
    rnd = cfg.hyd["cylinder_construction"]["stroke_round_mm"]
    stroke = math.ceil(((lmax - lmin) / MM + 4.0) / rnd) * rnd
    cyl = make_cylinder(cfg, bore, stroke)
    pen = 0.0
    # 1) geometric fit of the physical cylinder
    fit_lo = lmin - cyl.l_retracted                     # >= 0.002
    fit_hi = cyl.l_extended - lmax                      # >= 0.002
    fit = min(fit_lo, fit_hi) - 0.002
    if fit < 0:
        pen += 50 + 2000 * (-fit)
    # 2) moment arm floor
    m = link.moment_arm(qr)
    marm = float(np.min(np.abs(m)))
    sign_flip = bool(np.any(np.sign(m) != np.sign(m[0])))
    if sign_flip:
        pen += 40
    if marm < hs["min_moment_arm_mm"] * MM:
        pen += 20 + 3000 * (hs["min_moment_arm_mm"] * MM - marm)
    # 3) clearances
    clr = hs["cyl_clearance_mm"] * MM
    a, b = barrel_segment(link, cyl, qr, cfg)
    centre = np.repeat(plane_point(joint.axis, off, 0, 0)[None, :], len(qr), 0)
    dj = _seg_point_dist(a, b, centre)
    # joint hardware: bearing housing radius ~ 0.022 around the axis
    need = cyl.r_env + clr + 0.016 if abs(off) < 0.045 else cyl.r_env + clr
    jc = float(np.min(dj - need))
    if jc < 0:
        pen += 10 + 2000 * (-jc)
    # anchor lugs must clear the joint's fork/tongue hubs when in the link plane
    amin = 0.034 if abs(off) < 0.030 else 0.016
    i, k = PLANE[joint.axis]
    ad = min(math.hypot(x[0], x[1]), math.hypot(x[2], x[3]))
    if ad < amin:
        pen += 10 + 2000 * (amin - ad)
    kc = np.inf
    for ka, kb, kr, moves in _keepouts(joint, cfg):
        if moves:
            R = rot(joint.axis, qr)
            A = np.einsum("nij,j->ni", R, ka)
            B = np.einsum("nij,j->ni", R, kb)
        else:
            A = np.repeat(ka[None], len(qr), 0)
            B = np.repeat(kb[None], len(qr), 0)
        # compare in-plane only (project out axis coordinate)
        ax = "xyz".index(joint.axis)
        a2, b2, A2, B2 = a.copy(), b.copy(), A.copy(), B.copy()
        for arr in (a2, b2, A2, B2):
            arr[:, ax] = 0
        d = _seg_seg_dist(a2, b2, A2, B2)
        kc = min(kc, float(np.min(d - (cyl.r_env + kr + clr))))
    if kc < 0:
        pen += 10 + 2000 * (-kc)
    # 4) torque capacity
    dp = cfg.usable_dp
    pos, neg = torque_capacity(link, cyl, dp, qr)
    sf = hs["torque_safety_factor"]
    ratio_demand = np.inf
    peak = 1e-9
    if demand:
        qd = np.asarray(demand["q"])
        td = np.asarray(demand["tau"])
        peak = float(np.max(np.abs(td))) if len(td) else 1e-9
        p_d, n_d = torque_capacity(link, cyl, dp, qd)
        capd = np.where(td >= 0, p_d, n_d)
        need_t = sf * np.abs(td)
        mask = need_t > 0.05
        if np.any(mask):
            ratio_demand = float(np.min(capd[mask] / need_t[mask]))
        floor = hs["range_floor_fraction"] * peak
        ratio_floor = float(min(np.min(pos), np.min(neg)) / max(floor, 1e-9))
    else:
        ratio_floor = np.inf
    ratio = min(ratio_demand, ratio_floor)
    if ratio < 1.0:
        pen += 5 + 30 * (1.0 - ratio)
    # objective: among feasible, prefer larger margin, shorter stroke
    score = pen - 0.5 * min(ratio, 3.0) + 0.002 * stroke
    if not detail:
        return score, None
    v = demand_velocity(demand)
    flow = 0.0
    if v is not None:
        qd = np.asarray(demand["q"])
        mm_ = np.abs(link.moment_arm(qd))
        flow = float(np.max(mm_ * np.abs(np.asarray(demand["qdot"])) * cyl.a_bore))
    info = dict(
        joint=joint.base, bore=bore, cylinder=asdict(cyl), linkage=link.to_dict(),
        stroke_mm=stroke, l_min=lmin, l_max=lmax, fit_margin_mm=fit / MM,
        min_moment_arm_mm=marm / MM, max_moment_arm_mm=float(np.max(np.abs(m))) / MM,
        moment_sign_flip=sign_flip,
        joint_clearance_mm=jc / MM, link_clearance_mm=(kc / MM if np.isfinite(kc) else None),
        ratio_demand=(ratio_demand if np.isfinite(ratio_demand) else None),
        ratio_floor=(ratio_floor if np.isfinite(ratio_floor) else None),
        peak_demand_nm=peak, cap_pos_min_nm=float(np.min(pos)), cap_neg_min_nm=float(np.min(neg)),
        cap_pos_max_nm=float(np.max(pos)), cap_neg_max_nm=float(np.max(neg)),
        peak_flow_lpm=flow * 60000, mass_kg=cyl.mass(cfg), penalty=pen,
        feasible=pen == 0.0,
    )
    return score, info


def demand_velocity(demand):
    if not demand or "qdot" not in demand:
        return None
    return np.asarray(demand["qdot"])


def compress_demand(demand: dict | None, bins: int = 48) -> dict | None:
    """Keep the envelope of the demand: per angle bin, the extreme +/- torque and max |qdot|."""
    if not demand or not demand.get("q"):
        return demand
    q = np.asarray(demand["q"]); t = np.asarray(demand["tau"]); v = np.asarray(demand["qdot"])
    edges = np.linspace(q.min() - 1e-9, q.max() + 1e-9, bins + 1)
    idx = np.digitize(q, edges) - 1
    out = {"q": [], "tau": [], "qdot": []}
    for b in range(bins):
        m = idx == b
        if not np.any(m):
            continue
        for sel in (t[m] >= 0, t[m] < 0):
            if np.any(sel):
                qq, tt, vv = q[m][sel], t[m][sel], v[m][sel]
                k = int(np.argmax(np.abs(tt)))
                out["q"].append(float(qq[k])); out["tau"].append(float(tt[k]))
                out["qdot"].append(float(np.max(np.abs(vv))))
    return out


def _opt_task(args):
    from scipy.optimize import differential_evolution
    cfg_path, overrides, jname, bore, demand, seed = args
    from .config import RobotConfig
    cfg = RobotConfig(cfg_path, overrides)
    joint = cfg.joints[jname]
    b = joint.hydraulic.anchor_bounds
    bounds = [tuple(b["parent"][0]), tuple(b["parent"][1]), tuple(b["child"][0]), tuple(b["child"][1])]
    res = differential_evolution(
        lambda x: evaluate(joint, cfg, x, bore, demand, n_range=41)[0],
        bounds, seed=seed, maxiter=140, popsize=12, tol=1e-8, polish=False, init="sobol")
    _, info = evaluate(joint, cfg, res.x, bore, demand, n_range=121, detail=True)
    info["x"] = [float(v) for v in res.x]
    return jname, bore, info


def _pick(infos: list[dict], prev_bore: int | None = None, hysteresis: float = 1.10) -> dict:
    """Smallest feasible bore; else the least-bad candidate.

    Hysteresis keeps the design loop from limit-cycling: a bore smaller than the
    previous iteration's choice is only accepted with >= `hysteresis` margin."""
    feas = [i for i in sorted(infos, key=lambda i: i["bore"]) if i["feasible"]]
    if prev_bore is not None:
        feas = [i for i in feas if i["bore"] >= prev_bore or min(i["ratio_demand"] or 9, i["ratio_floor"] or 9) >= hysteresis]
    if feas:
        return feas[0]
    return min(infos, key=lambda i: i["penalty"])


def optimise_joint(joint: Joint, cfg: RobotConfig, demand: dict | None, seed: int = 7):
    infos = [_opt_task((str(cfg.path), getattr(cfg, "overrides", None), joint.name, bore, demand, seed))[2]
             for bore in sorted(cfg.hyd["cylinder_catalog"].keys())]
    return _pick(infos)


def mirror_demand(cfg: RobotConfig, demand_all: dict, joint: Joint) -> dict | None:
    """Combine L & R demand into the left-side convention (mirrored angles)."""
    out = {"q": [], "tau": [], "qdot": []}
    for side in ("L", "R"):
        name = f"{side}_{joint.base}"
        d = demand_all.get(name)
        if not d:
            continue
        s = -1.0 if (side == "R" and cfg.joints[name].axis in ("x", "z")) else 1.0
        out["q"] += [s * v for v in d["q"]]
        out["tau"] += [s * v for v in d["tau"]]
        out["qdot"] += [s * v for v in d["qdot"]]
    return out if out["q"] else None


def size_all(cfg: RobotConfig, demand_all: dict | None, log=print, workers: int | None = None,
             previous: dict | None = None) -> dict:
    """Size every hydraulic joint type (left side; right side = mirror), in parallel."""
    import os
    from concurrent.futures import ProcessPoolExecutor

    tasks, order = [], []
    for j in cfg.hydraulic_joints:
        if j.side == "R" or j.base in order:
            continue
        order.append(j.base)
        dem = compress_demand(mirror_demand(cfg, demand_all or {}, j))
        for bore in sorted(cfg.hyd["cylinder_catalog"].keys()):
            tasks.append((str(cfg.path), getattr(cfg, "overrides", None), j.name, bore, dem, 7))
    workers = workers or max(1, (os.cpu_count() or 2))
    by_joint: dict[str, list] = {}
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for jname, bore, info in ex.map(_opt_task, tasks):
            by_joint.setdefault(cfg.joints[jname].base, []).append(info)
    results = {}
    for base in order:
        prev = (previous or {}).get(base, {}).get("bore")
        info = _pick(by_joint[base], prev)
        results[base] = info
        log(f"  [size] {base:15s} bore {info['bore']:>2} stroke {info['stroke_mm']:>5.0f} mm  "
            f"arm {info['min_moment_arm_mm']:5.1f}-{info['max_moment_arm_mm']:5.1f} mm  "
            f"ratio {fmt(info['ratio_demand'])}/{fmt(info['ratio_floor'])}  "
            f"{'OK' if info['feasible'] else 'INFEASIBLE pen=%.2f' % info['penalty']}")
    return results


def fmt(v):
    return "  -  " if v is None else f"{v:5.2f}"


def linkage_for(cfg: RobotConfig, jname: str, sizing: dict) -> tuple[Linkage, Cylinder]:
    """Linkage + cylinder for a concrete (sided) joint name."""
    j = cfg.joints[jname]
    info = sizing[j.base]
    lk = Linkage.from_dict(info["linkage"])
    if j.side == "R":
        lk = mirror_linkage(lk)
    c = info["cylinder"]
    return lk, Cylinder(**c)


def mirror_linkage(lk: Linkage) -> Linkage:
    p, c = lk.parent.copy(), lk.child.copy()
    p[1], c[1] = -p[1], -c[1]
    return Linkage(lk.axis, p, c)


# ------------------------------------------------------------------- HPU sizing
def pump_flow_lpm(cfg: RobotConfig) -> float:
    h = cfg.hyd["hpu"]
    return h["pump_displacement_cc"] * h["pump_max_rpm"] * h["pump_volumetric_eff"] / 1000.0


def accumulator_usable_l(cfg: RobotConfig) -> float:
    """Isothermal gas law between pump-on threshold and lowest usable supply."""
    h = cfg.hyd["hpu"]
    p0 = h["accumulator_precharge_bar"] + 1.0
    p_hi = h["pressure_band_bar"][1] + 1.0
    p_lo = h["pressure_band_bar"][0] - 1.5 + 1.0    # allow 1.5 bar sag below band
    v0 = h["accumulator_volume_l"]
    return v0 * p0 / p_lo - v0 * p0 / p_hi
