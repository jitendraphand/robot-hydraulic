"""Tiny CSG toolkit on top of manifold3d (units: millimetres).

All solids are guaranteed 2-manifold (watertight), which the QA verifies again
on the exported meshes.
"""
from __future__ import annotations

from functools import reduce

import numpy as np
from manifold3d import Manifold, OpType

SEG = 48            # circular segments for features (bearing seats etc.)
SEG_FINE = 72


def box(sx, sy, sz, at=(0, 0, 0)) -> Manifold:
    return Manifold.cube([sx, sy, sz], True).translate(list(at))


def cyl(r, h, axis="z", at=(0, 0, 0), seg=SEG, r2=None) -> Manifold:
    c = Manifold.cylinder(h, r, r if r2 is None else r2, seg, True)
    if axis == "x":
        c = c.rotate([0, 90, 0])
    elif axis == "y":
        c = c.rotate([90, 0, 0])
    return c.translate(list(at))


def sphere(r, at=(0, 0, 0), seg=24) -> Manifold:
    return Manifold.sphere(r, seg).translate(list(at))


def rbox(sx, sy, sz, r, at=(0, 0, 0), axis="z") -> Manifold:
    """Box with rounded edges parallel to `axis`."""
    if axis == "x":
        a, b, h = sz, sy, sx
    elif axis == "y":
        a, b, h = sx, sz, sy
    else:
        a, b, h = sx, sy, sz
    r = max(0.2, min(r, a / 2 - 0.05, b / 2 - 0.05))
    pts = [(-a / 2 + r, -b / 2 + r), (a / 2 - r, -b / 2 + r), (a / 2 - r, b / 2 - r), (-a / 2 + r, b / 2 - r)]
    m = Manifold.batch_hull([Manifold.cylinder(h, r, r, 24, True).translate([p[0], p[1], 0]) for p in pts])
    if axis == "x":
        m = m.rotate([0, 90, 0])
    elif axis == "y":
        m = m.rotate([90, 0, 0])
    return m.translate(list(at))


def hull(*ms) -> Manifold:
    ms = [m for m in _flat(ms) if m is not None]
    return Manifold.batch_hull(ms)


def union(*ms) -> Manifold:
    ms = [m for m in _flat(ms) if m is not None and not m.is_empty()]
    if not ms:
        return Manifold()
    return Manifold.batch_boolean(ms, OpType.Add)


def diff(a: Manifold, *cut) -> Manifold:
    cut = [m for m in _flat(cut) if m is not None and not m.is_empty()]
    if not cut:
        return a
    return Manifold.batch_boolean([a] + cut, OpType.Subtract)


def inter(a, b) -> Manifold:
    return a ^ b


def _flat(x):
    for i in x:
        if isinstance(i, (list, tuple)):
            yield from _flat(i)
        else:
            yield i


def axis_index(axis: str) -> int:
    return "xyz".index(axis)


def along(axis: str, d: float) -> np.ndarray:
    v = np.zeros(3)
    v[axis_index(axis)] = d
    return v


def place_on_axis(m: Manifold, axis: str, at) -> Manifold:
    """Rotate a z-aligned feature so that its z points along `axis`, then move."""
    if axis == "x":
        m = m.rotate([0, 90, 0])
    elif axis == "y":
        m = m.rotate([-90, 0, 0])
    return m.translate(list(at))


def look_rotation(direction) -> np.ndarray:
    """Rotation matrix that maps +z to `direction`."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, d)
    c = float(np.dot(z, d))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def transform(m: Manifold, R: np.ndarray, t) -> Manifold:
    M = np.hstack([np.asarray(R, float), np.asarray(t, float).reshape(3, 1)])
    return m.transform(M)


def to_trimesh(m: Manifold):
    import trimesh
    mg = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mg.vert_properties)[:, :3],
                           faces=np.asarray(mg.tri_verts), process=False)


def text_plate(*_a, **_k):   # placeholder: STL part labels are embossed via tags in slicer
    return None


def reduce_union(ms):
    return reduce(lambda a, b: a + b, ms)
