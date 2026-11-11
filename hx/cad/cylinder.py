"""Printed tie-rod hydraulic cylinder parts (caps, piston, valve saddle).

Purchased: honed Al barrel (cut to tube_length), hardened chromed rod with
turned thread, M3 stainless tie rods, M8 rod end, GE8E spherical bearing,
O-rings (NBR 70), M5 brass threaded inserts for ports.

Cylinder local frame: +z from blind lug (pin at z=0) toward the rod end.
"""
from __future__ import annotations

import math

import numpy as np

from ..hydraulics import Cylinder
from .core import box, cyl, diff, hull, union

ROD_THREAD = {6: 4.0, 8: 5.0, 10: 6.0, 12: 8.0}   # turned rod-end thread for piston nut


def _tie_holes(c: Cylinder, h, z0, r=1.7):
    pcd = c.tie_rod_circle_mm / 2
    return [cyl(r, h, "z", (pcd * math.cos(a), pcd * math.sin(a), z0))
            for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]


def _port(cfg, z, c: Cylinder, side=1):
    hp = cfg["hardware"]["hydraulic_port"]
    R = c.cap_od_mm / 2
    ins = cyl(hp["insert_hole_d"] / 2, hp["insert_depth"] * 2, "y", (0, side * R, z))
    pas = cyl(hp["passage_d"] / 2, R + 1, "y", (0, side * R / 2, z))
    return [ins, pas]


def blind_cap(cfg, c: Cylinder):
    cc = cfg["hyd"] if False else cfg.hyd["cylinder_construction"]
    st = cfg["hardware"]["oring_static"]
    T = cc["cap_thickness"]
    lug = cc["blind_lug"]
    R = c.cap_od_mm / 2
    rb = c.bore_mm / 2
    z0 = lug                                 # cap outer face
    body = cyl(R, T, "z", (0, 0, z0 + T / 2), seg=96)
    spigot = cyl(rb - 0.1, 6.0, "z", (0, 0, z0 + T + 3.0), seg=96)
    eye = cyl(13.0, 12.0, "x", (0, 0, 0))
    lugm = hull(eye, cyl(14.0, 12.0, "x", (0, 0, z0 + 2)))
    solid = union(body, spigot, lugm)
    cuts = _tie_holes(c, T + 2, z0 + T / 2)
    cuts += [cyl(rb - 0.1 + 1, st["groove_w"], "z", (0, 0, z0 + T + 3.0), seg=96)
             .__sub__(cyl(rb - 0.1 - st["groove_depth"], st["groove_w"] + 1, "z", (0, 0, z0 + T + 3.0), seg=96))]
    cuts += [cyl(8.05, 12.4, "x", (0, 0, 0))]          # GE8E seat (16 mm)
    port_z = z0 + T - 6
    cuts += _port(cfg, port_z, c)
    cuts += [cyl(1.25, 12, "z", (0, 0, z0 + T + 2)),     # passage to chamber face
             cyl(1.25, R, "y", (0, R / 2 - 0.5, port_z)).__add__(cyl(1.25, 7, "z", (0, 0, port_z + 3.5)))]
    # cushion relief: shallow recess so the port is never blocked by the piston
    cuts += [cyl(rb * 0.45, 1.2, "z", (0, 0, z0 + T + 6 - 0.6))]
    return diff(solid, cuts)


def rod_cap(cfg, c: Cylinder):
    cc = cfg.hyd["cylinder_construction"]
    st = cfg["hardware"]["oring_static"]
    T = cc["cap_thickness"]
    R = c.cap_od_mm / 2
    rb = c.bore_mm / 2
    rr = c.rod_mm / 2
    body = cyl(R, T, "z", (0, 0, T / 2), seg=96)
    spigot = cyl(rb - 0.1, 6.0, "z", (0, 0, -3.0), seg=96)
    solid = union(body, spigot)
    cuts = _tie_holes(c, T + 2, T / 2)
    ring = lambda z, w, ro, ri: cyl(ro, w, "z", (0, 0, z), seg=96) - cyl(ri, w + 1, "z", (0, 0, z), seg=96)
    cuts.append(ring(-3.0, st["groove_w"], rb + 1, rb - 0.1 - st["groove_depth"]))
    cuts.append(cyl(rr + 0.15, T + 14, "z", (0, 0, T / 2), seg=64))            # rod bore
    cuts.append(cyl(rr + 1.6, 2.7, "z", (0, 0, T * 0.45), seg=64))           # rod seal (O-ring / U-cup)
    cuts.append(cyl(rr + 1.6, 2.7, "z", (0, 0, T * 0.70), seg=64))           # second rod seal
    cuts.append(cyl(rr + 1.2, 2.0, "z", (0, 0, T - 1.6), seg=64))            # wiper
    cuts.append(cyl(rr + 0.8, 0.8, "z", (0, 0, T * 0.58), seg=64))           # leak-off/drain groove
    port_z = 6.0
    cuts += _port(cfg, port_z, c)
    cuts.append(cyl(1.25, 9, "z", (0, -rr - 3.0, port_z - 4.0)))
    cuts.append(cyl(1.25, R, "y", (0, R / 2 - 0.5, port_z)))
    # passage from port down to the inner face outside the rod
    cuts.append(hull(cyl(1.25, 0.5, "z", (0, rr + 3.2, port_z)), cyl(1.25, 0.5, "z", (0, rr + 3.2, -6.0))))
    return diff(solid, cuts)


def piston(cfg, c: Cylinder):
    dy = cfg["hardware"]["oring_dynamic"]
    w = cfg.hyd["cylinder_construction"]["piston_width"]
    rb = c.bore_mm / 2
    body = cyl(rb - 0.15, w, "z", (0, 0, 0), seg=96)
    groove = cyl(rb + 1, dy["groove_w"], "z", (0, 0, 0), seg=96) - \
        cyl(rb - 0.15 - dy["groove_depth"], dy["groove_w"] + 1, "z", (0, 0, 0), seg=96)
    th = ROD_THREAD.get(int(c.rod_mm), 6.0)
    bore = cyl(th / 2 + 0.2, w + 2, "z", seg=48)
    spot = cyl(c.rod_mm / 2 + 0.2, 3.0, "z", (0, 0, w / 2 - 1.4), seg=48)    # rod shoulder spigot
    oring_id = cyl(th / 2 + 1.4, 1.6, "z", (0, 0, -w / 2 + 0.7), seg=48)     # static seal on thread
    return diff(body, groove, bore, spot, oring_id)


def valve_saddle(cfg, c: Cylinder):
    """Split clamp around the barrel carrying one miniature spool valve + its micro servo."""
    ro = c.tube_od_mm / 2
    w = 22.0
    ring = cyl(ro + 5.0, w, "z", seg=96) - cyl(ro + 0.2, w + 2, "z", seg=96)
    split = box(1.6, 2 * ro + 12, w + 2, at=(0, -(ro + 4), 0))
    ears = box(14, 10, w, at=(0, -(ro + 7), 0))
    ear_hole = cyl(1.7, 16, "x", (0, -(ro + 8), 0))
    plat = box(58, 6, 34, at=(0, ro + 6, 0))
    web = hull(box(30, 1, w, at=(0, ro + 2, 0)), box(58, 1, 34, at=(0, ro + 6, 0)))
    holes = [cyl(1.7, 10, "y", (sx, ro + 6, sz)) for sx in (-22, -6, 10, 24) for sz in (-11, 11)]
    zt = [box(4, 8, 2.2, at=(sx, ro + 6, 0)) for sx in (-27, 27)]
    s = union(ring, ears, plat, web)
    return diff(s, split, ear_hole, holes, zt, cyl(ro + 0.2, w + 2, "z", seg=96))


def assembly_solid(cfg, c: Cylinder, L_mm: float):
    """Whole cylinder (lug..rod-end eye) at pin-to-pin length L, for interference checks."""
    cc = cfg.hyd["cylinder_construction"]
    T = cc["cap_thickness"]
    lug = cc["blind_lug"]
    z_bc = lug
    tube_end = lug + T + c.tube_length_mm
    caps = union(cyl(c.cap_od_mm / 2, T, "z", (0, 0, z_bc + T / 2), seg=32),
                 cyl(c.tube_od_mm / 2, c.tube_length_mm, "z", (0, 0, z_bc + T + c.tube_length_mm / 2), seg=32),
                 cyl(c.cap_od_mm / 2, T, "z", (0, 0, tube_end + T / 2), seg=32),
                 hull(cyl(11.0, 10.0, "x", (0, 0, 0), seg=24), cyl(12.0, 10.0, "x", (0, 0, lug), seg=24)))
    rod_start = tube_end + T
    rod_len = max(1.0, L_mm - rod_start - 12.0)
    rod = cyl(c.rod_mm / 2, rod_len, "z", (0, 0, rod_start + rod_len / 2), seg=24)
    eye = cyl(10.0, 10.0, "x", (0, 0, L_mm), seg=24)
    neck = hull(cyl(6.0, 4, "z", (0, 0, L_mm - 22)), cyl(7.0, 10.0, "x", (0, 0, L_mm), seg=24))
    return union(caps, rod, eye, neck)


def cut_list(cfg, c: Cylinder) -> dict:
    cc = cfg.hyd["cylinder_construction"]
    return {
        "barrel_mm": round(c.tube_length_mm, 1),
        "rod_mm": round(c.stroke_mm + cc["piston_width"] + 2 * cc["cap_thickness"] + 25, 1),
        "tie_rod_mm": round(c.tube_length_mm + 2 * cc["cap_thickness"] + 12, 1),
        "rod_piston_thread": f"M{int(ROD_THREAD.get(int(c.rod_mm), 6))}",
        "rod_end_thread": "M8" if c.rod_mm >= 10 else "M6",
    }
