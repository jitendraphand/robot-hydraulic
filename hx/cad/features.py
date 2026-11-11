"""Reusable mechanical features: forks, tongues, cylinder lugs, servo pockets.

Every feature returns (solid, cut, base_point):
  solid       material to add
  cut         material to subtract from the *final* part (holes, seats, slots)
  base_point  where the structural beam of the link attaches (mm, body frame)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from manifold3d import Manifold

from .core import along, box, cyl, diff, hull, rbox, union, axis_index


@dataclass
class Feature:
    solid: Manifold
    cut: Manifold | None
    base: np.ndarray
    name: str = ""
    keep: list = field(default_factory=list)    # protected volumes (cut from beams only)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _perp_box(axis, w_axis, s1, s2, at):
    """Axis-aligned box: w_axis along `axis`, s1/s2 along the remaining axes (in xyz order)."""
    dims = [s1, s2]
    out = []
    for i in range(3):
        out.append(w_axis if i == axis_index(axis) else dims.pop(0))
    return box(*out, at=at)


class JH:
    """Joint hardware constants (mm) from robot.yaml:joint_hardware."""

    def __init__(self, cfg):
        j = cfg["joint_hardware"]
        self.hub_r = j["hub_r"]
        self.tongue_w = j["tongue_w"]
        self.gap = j["gap"]
        self.cheek_t = j["cheek_t"]
        self.brg_r = j["bearing_fit_d"] / 2
        self.brg_w = cfg["hardware"]["bearing_608"]["w"]
        self.lip_r = j["lip_d"] / 2
        self.bolt_r = j["bolt_clear_d"] / 2
        self.bolt_tight_r = j["bolt_tight_d"] / 2
        self.lug_t = j["lug_t"]
        self.lug_r = j["lug_r"]
        self.lug_gap = j["lug_gap"]
        self.insert_m3 = cfg["hardware"]["insert_m3"]["hole_d"] / 2
        self.insert_m4 = cfg["hardware"]["insert_m4"]["hole_d"] / 2


def fork(jh: JH, axis: str, at, dir_parent, tongue_w=None, encoder=True, swing_dirs=None,
         bridge_r=None, pad=(22, 22)) -> Feature:
    """Parent-side clevis: two cheeks with 608 bearings, bridge toward the parent."""
    at = np.asarray(at, float)
    dp = _unit(dir_parent)
    tw = jh.tongue_w if tongue_w is None else tongue_w
    half = tw / 2 + jh.gap
    ct = jh.cheek_t
    solids, cuts = [], []
    bridge_r = jh.hub_r + 15.0 if bridge_r is None else bridge_r
    for sgn in (-1, 1):
        cc = at + along(axis, sgn * (half + ct / 2))
        disc = cyl(jh.hub_r, ct, axis, cc)
        pd = _perp_box(axis, ct, pad[0], pad[1], cc + dp * bridge_r)
        solids.append(hull(disc, pd))
        # 608 seat from the outer face, lip on the inner face
        seat_c = cc + along(axis, sgn * (ct / 2 - jh.brg_w / 2 + 0.01))
        cuts.append(cyl(jh.brg_r, jh.brg_w + 0.02, axis, seat_c, seg=64))
        cuts.append(cyl(jh.lip_r, ct + 2, axis, cc))
        if encoder and sgn == 1:
            # 2x M3 inserts on the outer face for the AS5048A bracket
            perp = np.cross(dp, along(axis, 1.0))
            for s2 in (-1, 1):
                p = cc + along(axis, ct / 2 - 3.0) + perp * s2 * 14.0
                cuts.append(cyl(jh.insert_m3, 6.2, axis, p))
    cross = _perp_box(axis, 2 * (half + ct), pad[0], pad[1], at + dp * bridge_r)
    solids.append(cross)
    # swing clearance for the tongue/neck of the child
    cuts.append(cyl(jh.hub_r + 2.0, 2 * half, axis, at))
    if swing_dirs is not None:
        for d in swing_dirs:
            d = _unit(d)
            cuts.append(hull(cyl(jh.hub_r - 2, 2 * half, axis, at),
                             cyl(12.5, 2 * half, axis, at + d * (jh.hub_r + 18))))
    else:
        cuts.append(cyl(jh.hub_r + 13.0, 2 * half, axis, at))
    return Feature(union(solids), union(cuts), at + dp * (bridge_r + min(pad) / 2 - 3), "fork")


def tongue(jh: JH, axis: str, at, dir_child, width=None) -> Feature:
    at = np.asarray(at, float)
    dc = _unit(dir_child)
    w = jh.tongue_w if width is None else width
    disc = cyl(jh.hub_r, w, axis, at)
    neck = _perp_box(axis, w, 20, 20, at + dc * (jh.hub_r + 8))
    s = hull(disc, neck)
    cut = cyl(jh.bolt_tight_r, w + 4, axis, at)
    return Feature(s, cut, at + dc * (jh.hub_r + 12), "tongue")


def lugs(jh: JH, axis: str, pin, base, cyl_dirs, gap=None) -> Feature:
    """Pair of anchor lugs around a cylinder pin (pin axis // joint axis)."""
    pin = np.asarray(pin, float)
    base = np.asarray(base, float)
    g = jh.lug_gap if gap is None else gap
    solids = []
    for sgn in (-1, 1):
        off = along(axis, sgn * (g / 2 + jh.lug_t / 2))
        disc = cyl(jh.lug_r, jh.lug_t, axis, pin + off)
        foot = _perp_box(axis, jh.lug_t, 16, 16, base + off)
        solids.append(hull(disc, foot))
    # slot slightly wider than the lug gap: no face of the cut is coplanar with a lug face
    # (coincident faces become non-manifold edges after STL vertex merging)
    gc = g + 0.4
    cuts = [cyl(jh.bolt_r, g + 2 * jh.lug_t + 4, axis, pin), cyl(jh.lug_r + 4.0, gc, axis, pin)]
    for d in cyl_dirs:
        d = _unit(d)
        cuts.append(hull(cyl(13.0, gc, axis, pin), cyl(13.0, gc, axis, pin + d * 45.0)))
    return Feature(union(solids), union(cuts), base, "lugs")


def servo_pocket(sm: dict, out_axis: str, shaft_at, long_dir, wall=4.0, out_sign=1) -> Feature:
    """Pocket that holds a hobby servo with its output shaft at `shaft_at` pointing
    along +out_axis (or -out_axis if out_sign=-1). `long_dir` is the servo body's
    long direction (unit, perpendicular to the shaft)."""
    L, W, H = sm["body"]
    ld = _unit(long_dir)
    ax = along(out_axis, 1.0) * out_sign
    wd = np.cross(ax, ld)
    shaft_at = np.asarray(shaft_at, float)
    centre = shaft_at - ax * (H / 2 + 2.0) - ld * sm["shaft_offset"]
    # build in local frame (x=long, y=width, z=out) then rotate
    R = np.column_stack([ld, wd, ax])

    def local_box(sx, sy, sz, c):
        b = box(sx, sy, sz)
        M = np.hstack([R, (centre + R @ np.asarray(c, float)).reshape(3, 1)])
        return b.transform(M)

    shell = local_box(L + 2 * wall, W + 2 * wall, H + wall, (0, 0, -wall / 2 + 0.0))
    ears = local_box(sm["ear_span"] + 8, W + 2 * wall, 6, (0, 0, H / 2 - 8))
    pocket = local_box(L + 0.6, W + 0.6, H + 0.2, (0, 0, 0.1))
    ear_slot = local_box(sm["ear_span"] + 0.6, W + 0.6, 2.8, (0, 0, H / 2 - 8 + 1.6))
    cable = local_box(10, 8, 12, (-L / 2 - wall / 2, 0, -H / 2 + 6))
    shaft_hole = cyl(sm["horn_d"] / 2 + 1.5, 2 * wall + 6, out_axis, shaft_at)
    screws = []
    for s1 in (-1, 1):
        for s2 in (-1, 1):
            screws.append(local_box(2.6, 2.6, 14, (s1 * (sm["ear_span"] / 2 - 4.5), s2 * 5.0, H / 2 - 8)))
    return Feature(union(shell, ears), union(pocket, ear_slot, cable, shaft_hole, screws),
                   centre - ax * (H / 2), "servo")


def horn_flange(axis: str, at, sm: dict, out_sign=1, thickness=5.0) -> Feature:
    """Child-side disc bolted to an aluminium servo disc horn (4x M3 on PCD)."""
    at = np.asarray(at, float)
    r = sm["horn_d"] / 2 + 4
    ax = along(axis, 1.0) * out_sign
    c = at + ax * (thickness / 2 + 0.5)
    s = cyl(r, thickness, axis, c)
    pcd = sm["horn_d"] * 0.36
    cuts = [cyl(4.5, thickness + 2, axis, c)]
    i = [k for k in range(3) if k != axis_index(axis)]
    for k in range(4):
        a = np.pi / 4 + k * np.pi / 2
        p = c.copy()
        p[i[0]] += pcd * np.cos(a)
        p[i[1]] += pcd * np.sin(a)
        cuts.append(cyl(1.7, thickness + 2, axis, p))
    return Feature(s, union(cuts), c + ax * (thickness / 2), "horn")


def beam(a, b, w, t, r=4.0):
    """Structural member between two points (rounded rectangular ends)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return hull(rbox(w, w, w, r, at=a), rbox(w, w, w, r, at=b)) if t is None else \
        hull(rbox(t, w, t, r, at=a, axis="y"), rbox(t, w, t, r, at=b, axis="y"))


def beam_axis(a, b, axis: str, w_axis: float, t: float, r=5.0):
    """Beam whose width along `axis` is w_axis and thickness elsewhere t.

    The end blocks are thin along the dominant beam direction so the beam
    does not overshoot its end points (important next to joints)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    dims = [t, t, t]
    dims[axis_index(axis)] = w_axis
    d = b - a
    k = int(np.argmax(np.abs(d))) if np.linalg.norm(d) > 1e-6 else 2
    if k != axis_index(axis):
        dims[k] = min(dims[k], 4.0)
    rr = min(r, min(dims) / 2 - 0.1)
    return hull(rbox(*dims, rr, at=a, axis=axis), rbox(*dims, rr, at=b, axis=axis))


def beam_taper(a, b, axis: str, w_a: float, w_b: float, t: float, r=4.0):
    """Beam whose width along `axis` tapers from w_a (at a) to w_b (at b)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    k = int(np.argmax(np.abs(d))) if np.linalg.norm(d) > 1e-6 else 2
    ends = []
    for p, w in ((a, w_a), (b, w_b)):
        dims = [t, t, t]
        dims[axis_index(axis)] = w
        if k != axis_index(axis):
            dims[k] = min(dims[k], 4.0)
        ends.append(rbox(*dims, min(r, min(dims) / 2 - 0.1), at=p, axis=axis))
    return hull(*ends)


def nearest_on_segments(p, segs):
    p = np.asarray(p, float)
    best, bd = None, np.inf
    for a, b in segs:
        a, b = np.asarray(a, float), np.asarray(b, float)
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
        q = a + t * ab
        d = np.linalg.norm(q - p)
        if d < bd:
            best, bd = q, d
    return best
