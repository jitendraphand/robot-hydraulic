"""Simulation videos of HX-1 doing its tasks (stand, walk, squat, single-leg balance, arm lift).

The videos show the SAME physics runs the QA evaluates (hx.sim.scenarios.run), rendered with
the real CAD parts as visual meshes.  Hydraulic cylinders are drawn every frame between their
actual pin locations (cap / barrel / chromed rod), and a HUD shows each leg cylinder's load
as a percentage of its hydraulic capacity at the current joint angle.

    MUJOCO_GL=egl python -m hx video            # all tasks + montage + GIF previews
    MUJOCO_GL=egl python -m hx video --only walk

Outputs: docs/video/<task>.mp4, docs/video/hx1_tasks.mp4 (montage), docs/video/gif/<task>.gif
Needs an OpenGL backend for MuJoCo (EGL or OSMesa; `apt install libegl1 libosmesa6`) and ffmpeg.
"""
from __future__ import annotations

import math
import os
import shutil
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from ..config import BUILD, ROOT, RobotConfig  # noqa: E402

OUT = ROOT / "docs" / "video"
VIZ = BUILD / "viz"
W, H, FPS = 1280, 720, 30

COL_CF = (0.24, 0.25, 0.28, 1.0)       # PA12-CF parts
COL_PETG = (0.80, 0.81, 0.84, 1.0)     # PETG parts
COL_TPU = (0.12, 0.12, 0.14, 1.0)

TASKS = {   # scenario -> (title, subtitle, camera azimuth, elevation, distance)
    "walk": ("Walking", "ZMP preview-control gait + balance stabilizer, 8 steps of 0.08 m / 0.8 s", 135, -12, 2.75),
    "squat": ("Squat", "pelvis lowered to 75 deg knee flexion and back", 150, -10, 2.6),
    "single_leg": ("Single-leg balance", "CoM shifted over the left foot, right foot lifted 30 mm", 200, -10, 2.6),
    "arm_lift": ("Arm lift, 1 kg per hand", "shoulders to 87 deg, elbows flex; hips/ankles keep the CoM over the feet", 120, -8, 2.7),
    "stand": ("Standing", "quiet stance under hydraulic impedance control", 145, -10, 2.6),
}


# ------------------------------------------------------------------ meshes
def export_viz_meshes(cfg: RobotConfig) -> dict:
    """CAD links in their BODY frames (mm) -> build/viz/*.stl, with display colours."""
    from ..cad.build import registry
    from ..cad.core import to_trimesh
    from ..cad.features import JH
    from ..cad.links import BODY_BUILDERS

    VIZ.mkdir(parents=True, exist_ok=True)
    sizing = cfg.derived("sizing")
    jh = JH(cfg)
    mats = {p.name.replace("L_", ""): p.material for p in registry(cfg, sizing) if p.body}
    out = {}
    for base, fn in BODY_BUILDERS.items():
        m = fn(cfg, sizing, jh).simplify(0.25)
        col = COL_CF if mats.get(base) == "PA12-CF" else COL_PETG
        names = [base] if base in cfg.bodies else [f"L_{base}", f"R_{base}"]
        for nm in names:
            mm = m.mirror([0, 1, 0]) if nm.startswith("R_") else m
            path = VIZ / f"{nm}.stl"
            to_trimesh(mm).export(path)
            out[nm] = (str(path), col)
    return out


# ------------------------------------------------------------- cylinders
class CylinderPainter:
    """Adds cap/barrel/rod geoms for every hydraulic cylinder to a mjvScene."""

    def __init__(self, cfg: RobotConfig, model: mujoco.MjModel):
        from ..hydraulics import linkage_for, torque_capacity
        self.cfg = cfg
        self.tc = torque_capacity
        sizing = cfg.derived("sizing")
        cc = cfg.hyd["cylinder_construction"]
        self.items = []
        for j in cfg.hydraulic_joints:
            child = cfg.body_of_joint(j.name)
            lk, cyl = linkage_for(cfg, j.name, sizing)
            self.items.append(dict(
                joint=j.name, lk=lk, cyl=cyl,
                pid=model.body(child.parent).id, cid=model.body(child.name).id,
                porigin=np.array(child.origin), qadr=model.joint(j.name).qposadr[0],
                lug=cc["blind_lug"] / 1000, cap=cc["cap_thickness"] / 1000,
                tube=cyl.tube_length_mm / 1000))
        self.dp = cfg.usable_dp

    @staticmethod
    def _add(scn, gtype, width, a, b, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, gtype, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(rgba, np.float32))
        mujoco.mjv_connector(g, gtype, width, np.asarray(a, float), np.asarray(b, float))
        scn.ngeom += 1

    def paint(self, scn, d):
        for it in self.items:
            Rp, tp = d.xmat[it["pid"]].reshape(3, 3), d.xpos[it["pid"]]
            Rc, tc = d.xmat[it["cid"]].reshape(3, 3), d.xpos[it["cid"]]
            pa = Rp @ (it["porigin"] + it["lk"].parent) + tp
            ca = Rc @ it["lk"].child + tc
            u = (ca - pa) / max(np.linalg.norm(ca - pa), 1e-9)
            c = it["cyl"]
            r_tube, r_cap, r_rod = c.tube_od_mm / 2000, c.cap_od_mm / 2000, c.rod_mm / 2000
            s0 = it["lug"]
            s1 = s0 + it["cap"]
            s2 = s1 + it["tube"]
            s3 = s2 + it["cap"]
            orange, alu, chrome = (0.90, 0.45, 0.10, 1), (0.72, 0.74, 0.78, 1), (0.92, 0.93, 0.96, 1)
            self._add(scn, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.010, pa, pa + u * s0, orange)
            self._add(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, r_cap, pa + u * s0, pa + u * s1, orange)
            self._add(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, r_tube, pa + u * s1, pa + u * s2, alu)
            self._add(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, r_cap, pa + u * s2, pa + u * s3, orange)
            self._add(scn, mujoco.mjtGeom.mjGEOM_CAPSULE, r_rod, pa + u * s3, ca, chrome)

    def utilisation(self, d, model) -> dict:
        """|actuator torque| / hydraulic capacity in that direction at the current angle."""
        out = {}
        for it in self.items:
            q = np.array([d.qpos[it["qadr"]]])
            pos, neg = self.tc(it["lk"], it["cyl"], self.dp, q)
            tau = float(d.actuator_force[model.actuator(it["joint"]).id])
            cap = pos[0] if tau >= 0 else neg[0]
            out[it["joint"]] = abs(tau) / max(cap, 1e-6)
        return out


# --------------------------------------------------------------------- HUD
def _font(size):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"):
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default(size=size)


F_TITLE, F_SUB, F_SMALL = _font(34), _font(19), _font(15)
HUD_JOINTS = ["hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]


def hud(img: np.ndarray, title, subtitle, t, util: dict, pelvis_z: float, mass: float) -> np.ndarray:
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im, "RGBA")
    dr.rectangle([0, 0, W, 82], fill=(15, 18, 24, 170))
    dr.text((22, 8), f"HX-1  ·  {title}", font=F_TITLE, fill=(255, 255, 255, 255))
    dr.text((24, 52), subtitle, font=F_SUB, fill=(205, 212, 224, 255))
    dr.text((W - 250, 14), f"t = {t:5.2f} s", font=F_SUB, fill=(255, 255, 255, 255))
    dr.text((W - 250, 44), f"mass {mass:.1f} kg   pelvis {pelvis_z:.2f} m", font=F_SMALL, fill=(205, 212, 224, 255))
    # cylinder load panel
    x0, y0, lw, bw, cw = 24, H - 212, 118, 120, 320
    dr.rounded_rectangle([x0 - 12, y0 - 36, x0 + 2 * cw - 14, H - 16], 10, fill=(15, 18, 24, 170))
    dr.text((x0, y0 - 30), "leg cylinder load  (% of hydraulic capacity at current angle)", font=F_SMALL,
            fill=(230, 230, 235, 255))
    for col, side in enumerate(("L", "R")):
        for row, jb in enumerate(HUD_JOINTS):
            u = util.get(f"{side}_{jb}", 0.0)
            x = x0 + col * cw
            y = y0 + row * 34
            dr.text((x, y + 4), f"{side} {jb.replace('_', ' ')}", font=F_SMALL, fill=(220, 222, 228, 255))
            bx = x + lw
            dr.rectangle([bx, y + 6, bx + bw, y + 22], fill=(60, 64, 72, 220))
            c = (70, 200, 120, 255) if u < 0.5 else ((240, 190, 40, 255) if u < 0.8 else (235, 70, 60, 255))
            dr.rectangle([bx, y + 6, bx + int(bw * min(u, 1.0)), y + 22], fill=c)
            dr.text((bx + bw + 8, y + 4), f"{u * 100:3.0f}%", font=F_SMALL, fill=(235, 235, 240, 255))
    note = "MuJoCo physics · CAD meshes · cylinders drawn at live pin positions"
    tw = dr.textlength(note, font=F_SMALL)
    dr.rounded_rectangle([W - tw - 30, H - 40, W - 10, H - 12], 8, fill=(15, 18, 24, 150))
    dr.text((W - tw - 20, H - 36), note, font=F_SMALL, fill=(230, 232, 238, 255))
    return np.asarray(im)


# ------------------------------------------------------------------ render
class Recorder:
    def __init__(self, cfg, sc, task):
        self.cfg, self.sc = cfg, sc
        self.title, self.sub, az, el, dist = TASKS[task]
        m = sc.model
        m.vis.global_.offwidth, m.vis.global_.offheight = W, H
        m.vis.quality.shadowsize = 4096
        m.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
        m.vis.headlight.diffuse[:] = (0.55, 0.55, 0.55)
        self.r = mujoco.Renderer(m, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.azimuth, self.cam.elevation, self.cam.distance = az, el, dist
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        self.paint = CylinderPainter(cfg, m)
        self.pel = m.body("pelvis").id
        self.mass = float(sum(m.body_mass))
        self.frames = []
        self.look = None

    def __call__(self, t, m, d):
        target = d.xpos[self.pel].copy()
        target[2] = 0.70
        # shift the look-at point to the camera's left so the robot sits right of the HUD panel
        az = math.radians(self.cam.azimuth)
        target += 0.38 * np.array([-math.sin(az), math.cos(az), 0.0])
        self.look = target if self.look is None else 0.9 * self.look + 0.1 * target   # smooth tracking
        self.cam.lookat[:] = self.look
        self.r.update_scene(d, camera=self.cam, scene_option=self.opt)
        self.paint.paint(self.r.scene, d)
        img = self.r.render()
        self.frames.append(hud(img, self.title, self.sub, t, self.paint.utilisation(d, m), float(d.xpos[self.pel][2]),
                               self.mass))


def encode_mp4(frames, path: Path, fps=FPS):
    path.parent.mkdir(parents=True, exist_ok=True)
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "slow",
                          "-crf", "24", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
                         stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f).tobytes())
    p.stdin.close()
    if p.wait() != 0:
        raise RuntimeError(f"ffmpeg failed for {path}")


def make_gif(mp4: Path, gif: Path, width=480, fps=12):
    gif.parent.mkdir(parents=True, exist_ok=True)
    flt = f"fps={fps},scale={width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4), "-vf", flt, str(gif)], check=True)


def render_task(cfg, task: str, meshes: dict, log=print) -> dict:
    from .scenarios import SCENARIOS, run
    S = next(s for s in SCENARIOS if s.name == task)
    sc = S(cfg, meshes=meshes)
    rec = Recorder(cfg, sc, task)
    _, summ = run(sc, log=log, frame_cb=rec, frame_dt=1 / FPS)
    path = OUT / f"{task}.mp4"
    encode_mp4(rec.frames, path)
    make_gif(path, OUT / "gif" / f"{task}.gif")
    return {"file": str(path.relative_to(ROOT)), "frames": len(rec.frames), "fell": summ["fell"],
            "mb": path.stat().st_size / 1e6}


def _render_worker(cfg_path, task, meshes):
    return render_task(RobotConfig(cfg_path), task, meshes, log=lambda *_: None)


def montage(tasks):
    lst = OUT / "_concat.txt"
    lst.write_text("".join(f"file '{(OUT / f'{t}.mp4').as_posix()}'\n" for t in tasks))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst),
                    "-c", "copy", str(OUT / "hx1_tasks.mp4")], check=True)
    lst.unlink()


def render_all(cfg: RobotConfig, only: list[str] | None = None, log=print) -> dict:
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg not found")
    meshes = export_viz_meshes(cfg)
    order = ["walk", "squat", "single_leg", "arm_lift", "stand"]
    tasks = [t for t in order if not only or t in only]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=min(len(tasks), os.cpu_count() or 1)) as ex:
        futs = {t: ex.submit(_render_worker, str(cfg.path), t, meshes) for t in tasks}
        res = {t: f.result() for t, f in futs.items()}
    for t, r in res.items():
        log(f"  [video] {t}: {r['frames']} frames -> {r['file']} ({r['mb']:.1f} MB), fell={r['fell']}")
    if not only:
        montage(["walk", "squat", "single_leg", "arm_lift"])
        log(f"  [video] montage -> docs/video/hx1_tasks.mp4")
    return res
