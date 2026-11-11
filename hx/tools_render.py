"""Render documentation images: assembly poses + printed-part contact sheet."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .config import ROOT, RobotConfig

IMG = ROOT / "docs" / "img"


def render_all(cfg: RobotConfig, log=print):
    from .cad.assembly import nominal, render

    base = nominal(cfg)
    sq = math.radians(cfg["sim"]["squat_knee_deg"])
    poses = {
        "assembly_zero": (base, "HX-1 nominal pose"),
        "assembly_squat": (base | {f"{s}_{k}": v for s in "LR" for k, v in
                                   (("hip_pitch", -0.5 * sq), ("knee", sq), ("ankle_pitch", -0.5 * sq))},
                           "squat (knee %d deg)" % cfg["sim"]["squat_knee_deg"]),
        "assembly_stride": (base | {"L_hip_pitch": -0.4, "L_knee": 0.7, "L_ankle_pitch": -0.3,
                                    "R_hip_pitch": 0.25, "R_knee": 0.3, "R_ankle_pitch": -0.05,
                                    "L_shoulder_pitch": 0.3, "R_shoulder_pitch": -0.3}, "stride"),
        "assembly_arms": (base | {f"{s}_shoulder_pitch": -1.45 for s in "LR"} | {f"{s}_elbow": -1.0 for s in "LR"},
                          "arms forward, elbows flexed"),
    }
    for name, (q, title) in poses.items():
        render(cfg, IMG / f"{name}.png", qmap=q, views=((12, -50), (4, 0), (4, -90)), title=title)
        log(f"  [render] {name}")
    contact_sheet(cfg, IMG / "parts_sheet.png")
    log("  [render] parts_sheet")


def contact_sheet(cfg: RobotConfig, path: Path, cols: int = 6):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import trimesh

    plan = cfg.derived("print_plan")["parts"]
    files = [e["files"][0] for e in plan]
    rows = math.ceil(len(files) / cols)
    fig = plt.figure(figsize=(cols * 3.2, rows * 3.2), dpi=90)
    light = np.array([0.4, -0.5, 0.75])
    light /= np.linalg.norm(light)
    for i, f in enumerate(files):
        tm = trimesh.load(ROOT / f, force="mesh")
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d")
        tri = tm.vertices[tm.faces]
        shade = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
        col = np.array([0.85, 0.45, 0.12]) if "cyl_" in f else np.array([0.55, 0.58, 0.64])
        ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(col * shade[:, None], 0, 1), linewidths=0))
        lo, hi = tm.bounds
        c = (lo + hi) / 2
        r = max(hi - lo) / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=25, azim=-60)
        ax.set_axis_off()
        ax.set_title(Path(f).stem, fontsize=8)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.97, wspace=0, hspace=0.12)
    fig.savefig(path)
    plt.close(fig)
