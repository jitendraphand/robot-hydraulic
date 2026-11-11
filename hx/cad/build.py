"""Build every printed part -> STL, mass properties, print plan, plates, BOM.

Outputs
  cad/stl/<part>.stl                 one file per unique part (R_ = mirrored copy)
  cad/plates/<material>_plateNN.stl  parts packed onto the 420 x 420 bed
  cad/print_plan.md                  human readable plan (orientation, profile, time)
  cad/bom.csv                        purchased parts incl. cylinder cut list
  config/derived/mass_props.json     per-body printed mass/COM/inertia -> simulation
  config/derived/print_plan.json     machine-readable plan -> QA
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..config import ROOT, RobotConfig, save_derived
from ..hydraulics import Cylinder
from . import cylinder as CY
from .core import box, cyl, diff, hull, rbox, to_trimesh, union
from .features import JH
from .links import BODY_BUILDERS, finger, foot_pad, torso_cover

STL_DIR = ROOT / "cad" / "stl"
PLATE_DIR = ROOT / "cad" / "plates"
LINE_W = 0.45


@dataclass
class Part:
    name: str
    build: object                    # callable() -> Manifold
    qty: int
    material: str
    profile: str
    orient: tuple = (0, 0, 0)        # Euler (deg) applied before laying on the bed
    body: str | None = None          # kinematic body (for mass properties)
    mirror: bool = False             # also produce a mirrored R_ copy
    group: str = "structure"
    notes: str = ""
    mesh: object = field(default=None, repr=False)


# ------------------------------------------------------------- accessories
def magnet_cap(cfg):
    af = cfg["hardware"]["bolt_m8"]["nut_af"]
    mg = cfg["hardware"]["magnet_6x2_5"]
    body = cyl(10.5, 9.0, "z", (0, 0, 4.5))
    hexp = cyl(af / 2 / math.cos(math.pi / 6) + 0.15, 6.6, "z", (0, 0, 3.3), seg=6)
    mag = cyl(mg["d"] / 2 + 0.08, mg["h"] + 0.1, "z", (0, 0, 9.0 - (mg["h"] + 0.1) / 2 + 0.01))
    return diff(body, hexp, mag)


def encoder_bracket(cfg):
    plate = rbox(40, 16, 3, 3, at=(0, 0, 1.5))
    standoffs = [cyl(3.5, 11.5, "z", (s * 14, 0, 3 + 5.75)) for s in (-1, 1)]
    pcb = rbox(26, 16, 2, 2, at=(0, 0, 1))
    holes = [cyl(1.7, 30, "z", (s * 14, 0, 0)) for s in (-1, 1)]
    holes += [cyl(1.2, 10, "z", (s * 9, 0, 0)) for s in (-1, 1)]       # PCB screws (M2)
    win = box(12, 12, 10, at=(0, 0, 0))
    return diff(union(plate, standoffs), holes, win)


def node_enclosure(cfg, big=False):
    L, W, H = (130, 90, 32) if big else (100, 70, 28)
    shell = rbox(L, W, H, 6, at=(0, 0, H / 2)) - rbox(L - 4, W - 4, H, 4.5, at=(0, 0, H / 2 + 2))
    bosses = [cyl(3.5, 6, "z", (sx * (L / 2 - 9), sy * (W / 2 - 9), 5)) for sx in (-1, 1) for sy in (-1, 1)]
    holes = [cyl(1.6, 8, "z", (sx * (L / 2 - 9), sy * (W / 2 - 9), 6)) for sx in (-1, 1) for sy in (-1, 1)]
    ports = [rbox(14, 10, 12, 2, at=(L / 2, y, 14), axis="x") for y in (-18, 0, 18)]
    ports += [rbox(14, 10, 12, 2, at=(-L / 2, y, 14), axis="x") for y in (-12, 12)]
    vents = [box(2, W - 30, 1.5, at=(x, 0, 0)) for x in np.arange(-L / 2 + 20, L / 2 - 20, 6)]
    tabs = [diff(box(12, 10, 4, at=(sx * (L / 2 + 5), 0, 2)), cyl(2.2, 6, "z", (sx * (L / 2 + 6), 0, 2))) for sx in (-1, 1)]
    return diff(union(shell, bosses, tabs), holes, ports, vents)


def node_lid(cfg, big=False):
    L, W = (130, 90) if big else (100, 70)
    lid = rbox(L, W, 2.4, 6, at=(0, 0, 1.2))
    lip = rbox(L - 4.4, W - 4.4, 3, 4.3, at=(0, 0, 3.6)) - rbox(L - 8, W - 8, 4, 3, at=(0, 0, 3.6))
    holes = [cyl(1.7, 10, "z", (sx * (L / 2 - 9), sy * (W / 2 - 9), 1)) for sx in (-1, 1) for sy in (-1, 1)]
    return diff(union(lid, lip), holes)


def hose_clip(cfg):
    od = cfg.hyd["hose"]["od_mm"]
    body = rbox(2 * od + 14, 10, od + 5, 2, at=(0, 0, (od + 5) / 2))
    holes = [cyl(od / 2 + 0.25, 12, "y", (s * (od / 2 + 2), 0, od / 2 + 2.5)) for s in (-1, 1)]
    slots = [box(od - 1.2, 12, 4, at=(s * (od / 2 + 2), 0, od + 4)) for s in (-1, 1)]
    zt = box(4, 12, 1.8, at=(0, 0, 1.2))
    return diff(body, holes, slots, zt)


def fit_coupon(cfg):
    """Print FIRST: calibrates bearing press fit, inserts, O-ring glands, M5 ports."""
    base = rbox(150, 60, 10, 6, at=(0, 0, 5))
    cuts = []
    for i, d in enumerate((22.05, 22.15, 22.25)):
        cuts.append(cyl(d / 2, 7.02, "z", (-55 + i * 28, 12, 10 - 3.5), seg=64))
        cuts.append(cyl(7.5, 12, "z", (-55 + i * 28, 12, 5)))
    for i, d in enumerate((51.9, 52.1)):
        pass
    for i, d in enumerate((3.9, 4.0, 4.1)):
        cuts.append(cyl(d / 2, 6, "z", (35 + i * 10, 15, 10 - 2.9)))
    for i, d in enumerate((5.5, 5.6, 5.7)):
        cuts.append(cyl(d / 2, 9, "z", (35 + i * 10, -2, 10 - 4.4)))
    for i, d in enumerate((6.3, 6.4, 6.5)):
        cuts.append(cyl(d / 2, 9, "z", (35 + i * 10, -18, 10 - 4.4)))
    for i, d in enumerate((8.0, 8.2, 8.4)):
        cuts.append(cyl(d / 2, 12, "z", (-55 + i * 28, -15, 5)))
    # rod bore + O-ring gland test (10 mm rod)
    gland = cyl(10, 10, "z", (60, 12, 5)) if False else None
    return diff(base, cuts)


def cylinder_types(cfg, sizing):
    """Unique cylinder specs (bore, stroke) -> (Cylinder, [joint bases])."""
    types = {}
    for base, info in sizing.items():
        c = Cylinder(**info["cylinder"])
        key = f"b{int(c.bore_mm)}s{int(c.stroke_mm)}"
        types.setdefault(key, [c, []])[1].append(base)
    return types


def registry(cfg: RobotConfig, sizing: dict) -> list[Part]:
    jh = JH(cfg)
    P = []
    body_meta = {   # material, profile, print orientation, mirror
        "pelvis": ("PA12-CF", "structural", (0, 0, 0), False),
        "torso": ("PETG", "standard", (0, 0, 0), False),
        "hip_yaw_link": ("PA12-CF", "structural", (180, 0, 0), True),
        "hip_roll_link": ("PA12-CF", "structural", (0, 90, 0), True),
        "thigh": ("PA12-CF", "structural", (-90, 0, 0), True),
        "shin": ("PA12-CF", "structural", (-90, 0, 0), True),
        "ankle_link": ("PA12-CF", "structural", (-90, 0, 0), True),
        "foot": ("PA12-CF", "structural", (0, 0, 0), True),
        "shoulder_link": ("PA12-CF", "structural", (-90, 0, 0), True),
        "upper_arm_roll": ("PETG", "structural", (0, 0, 0), True),
        "upper_arm": ("PA12-CF", "structural", (-90, 0, 0), True),
        "forearm": ("PETG", "structural", (-90, 0, 0), True),
        "hand": ("PETG", "standard", (0, 90, 0), True),
        "neck_link": ("PETG", "standard", (0, 0, 0), False),
        "head": ("PETG", "standard", (0, 0, 0), False),
    }
    for base, fn in BODY_BUILDERS.items():
        mat, prof, ori, mir = body_meta[base]
        name = f"L_{base}" if mir else base
        P.append(Part(name, (lambda fn=fn: fn(cfg, sizing, jh)), 1, mat, prof, ori, body=name, mirror=mir))
    P.append(Part("torso_cover", lambda: torso_cover(cfg, sizing, jh), 2, "PETG", "standard", (0, 90, 0), group="covers"))
    P.append(Part("foot_pad", lambda: foot_pad(cfg), 2, "TPU95A", "flexible", group="structure",
                  notes="Glue under foot over the 4 FSRs (one per corner)."))
    P.append(Part("L_finger", lambda: finger(cfg, sizing, jh), 1, "PETG", "standard", (0, 90, 0), mirror=True))
    for key, (c, joints) in cylinder_types(cfg, sizing).items():
        n = 2 * len(joints)
        note = f"{c.bore_mm:.0f} mm bore x {c.stroke_mm:.0f} stroke, rod {c.rod_mm:.0f}: " + ", ".join(joints)
        P.append(Part(f"cyl_{key}_blind_cap", lambda c=c: CY.blind_cap(cfg, c), n, "PA12-CF", "pressure",
                      (0, 0, 0), group="hydraulic", notes=note))
        P.append(Part(f"cyl_{key}_rod_cap", lambda c=c: CY.rod_cap(cfg, c), n, "PA12-CF", "pressure",
                      (180, 0, 0), group="hydraulic", notes=note))
        P.append(Part(f"cyl_{key}_piston", lambda c=c: CY.piston(cfg, c), n, "PA12-CF", "pressure",
                      (0, 0, 0), group="hydraulic", notes=note))
        P.append(Part(f"cyl_{key}_valve_saddle", lambda c=c: CY.valve_saddle(cfg, c), n, "PETG", "structural",
                      (0, 0, 0), group="hydraulic", notes="Clamp on barrel; carries spool valve + micro servo."))
    nh = len(cfg.hydraulic_joints)
    P.append(Part("encoder_magnet_cap", lambda: magnet_cap(cfg), nh, "PETG", "standard", group="sensors"))
    P.append(Part("encoder_bracket", lambda: encoder_bracket(cfg), nh, "PETG", "standard", group="sensors"))
    P.append(Part("limb_node_enclosure", lambda: node_enclosure(cfg), 4, "PETG", "standard", group="electronics"))
    P.append(Part("limb_node_lid", lambda: node_lid(cfg), 4, "PETG", "standard", group="electronics"))
    P.append(Part("hub_enclosure", lambda: node_enclosure(cfg, True), 1, "PETG", "standard", group="electronics"))
    P.append(Part("hub_lid", lambda: node_lid(cfg, True), 1, "PETG", "standard", group="electronics"))
    P.append(Part("hose_clip", lambda: hose_clip(cfg), 40, "PETG", "standard", group="hydraulic"))
    P.append(Part("fit_coupon", lambda: fit_coupon(cfg), 1, "PA12-CF", "pressure", group="calibration",
                  notes="Print first in each material; record fits in qa/build_log.yaml"))
    return P


# ---------------------------------------------------------------- analysis
def merged_watertight(m) -> bool:
    import trimesh
    t = to_trimesh(m)
    t2 = trimesh.Trimesh(t.vertices, t.faces, process=True)    # what a slicer sees after STL load
    return bool(t2.is_watertight)


def heal(m):
    """Resolve features that touch along an edge (valid in manifold3d, ambiguous after
    STL vertex merging) by unioning with a 2 um shifted copy; verified afterwards."""
    from manifold3d import Manifold, OpType
    if merged_watertight(m):
        return m
    for d in ((0.002, 0.0013, 0.0007), (-0.003, 0.002, -0.0011), (0.004, -0.0031, 0.0023)):
        h = Manifold.batch_boolean([m, m.translate(list(d))], OpType.Add)
        if merged_watertight(h):
            return h
    return m


def printed_mass(mesh, cfg, material, profile) -> float:
    rho = cfg["materials"][material]["density_g_cm3"] * 1e-3      # g/mm^3
    pr = cfg["print_profiles"][profile]
    t = pr["perimeters"] * LINE_W
    V = float(mesh.volume)
    A = float(mesh.area)
    shell = min(V, A * t)
    return rho * (shell + pr["infill"] * max(0.0, V - shell)) / 1000.0     # kg


def lay_flat(m, orient):
    from manifold3d import Manifold
    r = m.rotate(list(orient))
    bb = r.bounding_box()
    return r.translate([-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]])


def fits_bed(m, cfg):
    p = cfg["printer"]
    bb = m.bounding_box()
    sx, sy, sz = bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]
    lim = p["bed_x_mm"] - 2 * p["edge_margin_mm"]
    ok_plain = sx <= lim and sy <= lim
    diag = (sx + sy) / math.sqrt(2) <= lim and max(sx, sy) <= lim * math.sqrt(2)
    return (ok_plain or diag) and sz <= p["max_z_mm"], (sx, sy, sz)


def pack(items, cfg):
    """Shelf packing of (name, w, d) rectangles onto beds; returns list of plates."""
    p = cfg["printer"]
    W = p["bed_x_mm"] - 2 * p["edge_margin_mm"]
    s = p["part_spacing_mm"]
    items = sorted(items, key=lambda t: -t[2])
    plates = []
    for it in items:
        placed = False
        for pl in plates:
            for sh in pl["shelves"]:
                if sh["x"] + it[1] <= W and it[2] <= sh["h"]:
                    pl["items"].append((it[0], sh["x"], sh["y"]))
                    sh["x"] += it[1] + s
                    placed = True
                    break
            if placed:
                break
            if pl["y"] + it[2] <= W:
                sh = {"x": it[1] + s, "y": pl["y"], "h": it[2]}
                pl["shelves"].append(sh)
                pl["items"].append((it[0], 0.0, pl["y"]))
                pl["y"] += it[2] + s
                placed = True
                break
        if not placed:
            pl = {"shelves": [{"x": it[1] + s, "y": 0.0, "h": it[2]}], "items": [(it[0], 0.0, 0.0)],
                  "y": it[2] + s}
            plates.append(pl)
    return plates


def build_all(cfg: RobotConfig | None = None, log=print, write_stl=True) -> dict:
    import trimesh

    cfg = cfg or RobotConfig()
    sizing = cfg.derived("sizing")
    if not sizing:
        raise RuntimeError("run sizing first (python -m hx size)")
    STL_DIR.mkdir(parents=True, exist_ok=True)
    PLATE_DIR.mkdir(parents=True, exist_ok=True)
    if write_stl:                       # never leave stale parts from earlier design iterations
        for d in (STL_DIR, PLATE_DIR):
            for f in d.glob("*.stl"):
                f.unlink()
    parts = registry(cfg, sizing)
    massprops, plan = {}, []
    meshes_bed = {}
    for p in parts:
        m = heal(p.build())
        if m.is_empty():
            raise RuntimeError(f"part {p.name} is empty")
        p.mesh = m
        tm = to_trimesh(m)
        mass = printed_mass(tm, cfg, p.material, p.profile)
        bedm = lay_flat(m, p.orient)
        ok, dims = fits_bed(bedm, cfg)
        btm = to_trimesh(bedm)
        names = [p.name] + ([p.name.replace("L_", "R_", 1)] if p.mirror else [])
        for nm in names:
            if write_stl:
                out = btm if nm == p.name else to_trimesh(lay_flat(m.mirror([0, 1, 0]), p.orient))
                out.export(STL_DIR / f"{nm}.stl")
            meshes_bed[nm] = bedm if nm == p.name else lay_flat(m.mirror([0, 1, 0]), p.orient)
        if p.body:
            com = tm.center_mass if tm.volume > 0 else np.zeros(3)
            I = tm.moment_inertia * (mass / max(tm.mass, 1e-12)) * 1e-6     # kg mm^2 -> kg m^2
            for nm in names:
                s = -1.0 if nm.startswith("R_") else 1.0
                c = (np.array(com) / 1000.0) * np.array([1, s, 1])
                Ii = I.copy()
                if s < 0:
                    Ii[0, 1] *= -1; Ii[1, 0] *= -1; Ii[1, 2] *= -1; Ii[2, 1] *= -1
                massprops[nm] = {"mass": mass, "com": c.tolist(), "inertia": Ii.tolist()}
        vol_cm3 = mass * 1000 / cfg["materials"][p.material]["density_g_cm3"]
        hours = vol_cm3 * 1000 / cfg["printer"]["volumetric_flow_mm3_s"] / 3600 * 1.35
        plan.append(dict(name=p.name, files=[f"cad/stl/{n}.stl" for n in names], qty=p.qty * len(names),
                         qty_each=p.qty, material=p.material, profile=p.profile, orient=list(p.orient),
                         group=p.group, fits=bool(ok), bed_dims_mm=[round(d, 1) for d in dims],
                         mass_kg_each=round(mass, 4), print_h_each=round(hours, 2),
                         watertight=bool(tm.is_watertight), volume_cm3=round(float(tm.volume) / 1000, 2),
                         body=p.body, notes=p.notes,
                         hardened_nozzle=cfg["materials"][p.material]["hardened_nozzle"]))
        log(f"  [cad] {p.name:32s} {dims[0]:6.1f} x {dims[1]:6.1f} x {dims[2]:6.1f} mm  "
            f"{mass*1000:7.1f} g  {'fits' if ok else 'DOES NOT FIT'}  {'' if tm.is_watertight else 'NOT WATERTIGHT'}")
    # plates per material
    plates_out = []
    for mat in sorted({p["material"] for p in plan}):
        items = []
        for e in plan:
            if e["material"] != mat:
                continue
            for f in e["files"]:
                nm = Path(f).stem
                for k in range(e["qty_each"]):
                    bb = meshes_bed[nm].bounding_box()
                    items.append((f"{nm}#{k}", bb[3] - bb[0], bb[4] - bb[1]))
        pls = pack(items, cfg)
        for i, pl in enumerate(pls):
            ms = []
            for (iid, x, y) in pl["items"]:
                nm = iid.split("#")[0]
                bm = meshes_bed[nm]
                bb = bm.bounding_box()
                ms.append(bm.translate([x - bb[0] + cfg["printer"]["edge_margin_mm"],
                                        y - bb[1] + cfg["printer"]["edge_margin_mm"], 0]))
            fn = PLATE_DIR / f"{mat}_plate{i + 1:02d}.stl"
            if write_stl:
                to_trimesh(union(ms)).export(fn)
            plates_out.append(dict(material=mat, file=str(fn.relative_to(ROOT)), parts=[it[0] for it in pl["items"]]))
    save_derived("mass_props", massprops)
    save_derived("print_plan", {"parts": plan, "plates": plates_out})
    write_plan_md(cfg, plan, plates_out)
    write_bom(cfg, sizing)
    return {"parts": plan, "plates": plates_out, "mass_props": massprops}


def write_plan_md(cfg, plan, plates):
    pr = cfg["print_profiles"]
    lines = ["# Print plan — Elegoo Neptune 4 Max (420 x 420 x 480 mm)", "",
             "Generated by `python -m hx cad`. Do not edit by hand.", "",
             "## Profiles", "", "| profile | layer | perimeters | top/bottom | infill |", "|---|---|---|---|---|"]
    for k, v in pr.items():
        lines.append(f"| {k} | {v['layer_mm']} mm | {v['perimeters']} | {v['top_bottom']} | {int(v['infill']*100)} % {v['pattern']} |")
    lines += ["", "## Materials", "", "| material | nozzle | bed | hardened nozzle | note |", "|---|---|---|---|---|"]
    for k, v in cfg["materials"].items():
        lines.append(f"| {k} | {v['nozzle_c']} C | {v['bed_c']} C | {'yes' if v['hardened_nozzle'] else 'no'} | {v.get('note','')} |")
    lines += ["", "## Parts", "", "| part | qty | material | profile | orient (deg) | bed footprint (mm) | g each | h each | fits |",
              "|---|---|---|---|---|---|---|---|---|"]
    tm, th = 0.0, 0.0
    for e in sorted(plan, key=lambda e: (e["group"], e["name"])):
        d = e["bed_dims_mm"]
        lines.append(f"| `{e['name']}` | {e['qty']} | {e['material']} | {e['profile']} | {tuple(e['orient'])} | "
                     f"{d[0]:.0f} x {d[1]:.0f} x {d[2]:.0f} | {e['mass_kg_each']*1000:.0f} | {e['print_h_each']:.1f} | "
                     f"{'yes' if e['fits'] else '**NO**'} |")
        tm += e["mass_kg_each"] * e["qty"]
        th += e["print_h_each"] * e["qty"]
    by_mat = {}
    for e in plan:
        by_mat[e["material"]] = by_mat.get(e["material"], 0) + e["mass_kg_each"] * e["qty"]
    lines += ["", f"**Total printed mass:** {tm:.2f} kg  —  **estimated print time:** {th:.0f} h", "",
              "Filament (+15 % waste): " + ", ".join(f"{k} {v*1.15:.2f} kg" for k, v in sorted(by_mat.items())), "",
              "## Plates", ""]
    for pl in plates:
        lines.append(f"- `{pl['file']}` — {len(pl['parts'])} parts: " + ", ".join(sorted(set(p.split('#')[0] for p in pl['parts']))))
    lines += ["", "## Rules", "",
              "1. Print `fit_coupon` first in every material; record results in `qa/build_log.yaml`.",
              "2. Pressure parts (`*_cap`, `*_piston`): 100 % infill, 8 perimeters, 0.16 mm layers, dry filament.",
              "3. Fibre-filled filaments need a hardened nozzle (see printer.nozzle_note in robot.yaml).",
              "4. Bed max is 85 C: PA12-CF on textured PEI + glue stick at 80 C, enclosure recommended.",
              "5. Install heat-set inserts at 245 C (PA-CF) / 220 C (PETG) before assembly.",
              ""]
    (ROOT / "cad" / "print_plan.md").write_text("\n".join(lines))


def write_bom(cfg, sizing):
    rows = []
    nh = len(cfg.hydraulic_joints)
    types = cylinder_types(cfg, sizing)
    for key, (c, joints) in types.items():
        n = 2 * len(joints)
        cl = CY.cut_list(cfg, c)
        rows += [
            ("hydraulic", f"Al honed barrel {c.bore_mm:.0f} ID x {c.tube_od_mm:.0f} OD, cut {cl['barrel_mm']} mm", n, key),
            ("hydraulic", f"Hardened chromed rod d{c.rod_mm:.0f}, cut {cl['rod_mm']} mm, turn {cl['rod_piston_thread']} piston end + {cl['rod_end_thread']} outer end", n, key),
            ("hydraulic", f"M3 A4 threaded rod {cl['tie_rod_mm']} mm (tie rods) + 8x M3 nyloc", 4 * n, key),
            ("hydraulic", f"NBR70 O-ring set for {c.bore_mm:.0f} bore (piston, 2x static, 2x rod, wiper)", n, key),
        ]
    rows += [
        ("hydraulic", f"{cfg.hyd['valve']['type']} ({cfg.hyd['valve']['ports']} ports, >= {cfg.hyd['valve']['rated_pressure_bar']} bar)", nh, "valves"),
        ("hydraulic", cfg.hyd["valve"]["servo"]["model"] + " (valve actuator)", nh, "valves"),
        ("hydraulic", f"Pressure transducer 0-{cfg.hyd['sensor']['range_bar']} bar, 0.5-4.5 V ratiometric", 2 * nh + 2, "sensors"),
        ("hydraulic", "M8 female rod end (rod side)", nh, "joints"),
        ("hydraulic", "GE8E spherical plain bearing (blind lug)", nh, "joints"),
        ("hydraulic", "M5 brass threaded insert (ports)", 2 * nh, "fittings"),
        ("hydraulic", "M5 -> 6 mm push-in / banjo fitting rated >= 30 bar", 4 * nh + 6, "fittings"),
        ("hydraulic", f"PA12 hose {cfg.hyd['hose']['od_mm']}x{cfg.hyd['hose']['id_mm']} mm rated {cfg.hyd['hose']['rated_bar']} bar (m)", 12, "fittings"),
        ("hpu", f"Gear pump {cfg.hyd['hpu']['pump_displacement_cc']} cc/rev, >= 30 bar (RC-hydraulics class)", 1, "hpu"),
        ("hpu", "BLDC motor 6S ~400 W + ESC with RPM telemetry", 1, "hpu"),
        ("hpu", f"Diaphragm accumulator {cfg.hyd['hpu']['accumulator_volume_l']} L, precharge {cfg.hyd['hpu']['accumulator_precharge_bar']} bar N2", 1, "hpu"),
        ("hpu", f"Relief valve adjustable, set {cfg.hyd['relief_pressure_bar']} bar", 1, "hpu"),
        ("hpu", "Normally-open 2/2 solenoid dump valve (accumulator -> tank on power loss)", 1, "hpu"),
        ("hpu", "Return filter 25 um + suction strainer", 1, "hpu"),
        ("hpu", "Check valve (pump outlet)", 1, "hpu"),
        ("hpu", f"Reservoir {cfg.hyd['hpu']['reservoir_volume_l']} L w/ breather, level switch, NTC", 1, "hpu"),
        ("joints", "608-2RS bearing", 2 * nh, "joints"),
        ("joints", "6808-2RS bearing (hip yaw)", 2, "joints"),
        ("joints", "M8 x 70 socket head bolt + nyloc (joint axes)", nh, "joints"),
        ("joints", "M8 x 30 bolt (hip roll stub axles) + M8 nut", 4, "joints"),
        ("joints", "M8 shoulder pins for cylinder ends (M8 x 45 + nyloc)", 2 * nh, "joints"),
        ("joints", "DIN 988 8x14x1 shim", 6 * nh, "joints"),
        ("sensors", "AS5048A magnetic encoder board + 6x2.5 diametric magnet", nh, "sensors"),
        ("sensors", "FSR 402 force sensor (feet)", 8, "sensors"),
        ("sensors", "IMU (BNO085 or ICM-42688 breakout)", 1, "sensors"),
        ("fasteners", "M3 heat-set insert (5.7 mm)", 200, "fasteners"),
        ("fasteners", "M4 heat-set insert (8.1 mm)", 20, "fasteners"),
        ("fasteners", "M3 / M4 socket head screw assortment (A2)", 1, "fasteners"),
    ]
    for j in cfg.servo_joints:
        rows.append(("servo", f"Servo {j.servo_model} ({j.name})", 1, "servos"))
    with open(ROOT / "cad" / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["category", "item", "qty", "ref"])
        merged = {}
        for r in rows:
            k = (r[0], r[1], r[3])
            merged[k] = merged.get(k, 0) + r[2]
        for (cat, item, ref), q in merged.items():
            w.writerow([cat, item, q, ref])
