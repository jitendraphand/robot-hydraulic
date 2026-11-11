"""Generate the MuJoCo MJCF model from robot.yaml (+ CAD mass properties).

Mass model per body =  printed parts (from CAD volume, density, infill)  +
purchased hardware (cylinders, valve manifolds, servos, bearings, sensors)  +
torso equipment (battery, HPU ...).  Until the CAD has been built the
``mass_guess`` table from robot.yaml is used instead of the printed mass.
"""
from __future__ import annotations

import math
from xml.sax.saxutils import quoteattr

import numpy as np

from ..config import RobotConfig

VALVE_PACK_EXTRA = 0.075     # mount + 2 pressure sensors + M5 fittings (kg)
JOINT_HW = 0.065             # 2x608, M8 bolt/nut, spacers, encoder + magnet (kg)


def _fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def body_mass_model(cfg: RobotConfig, payload_kg: float = 0.0) -> dict[str, dict]:
    """Return {body: {mass, com, inertia_diag, items: [...]}} in body frames."""
    from ..hydraulics import linkage_for

    sizing = cfg.derived("sizing")
    cadmp = cfg.derived("mass_props") or {}
    g = cfg["geometry"]
    out: dict[str, dict] = {}

    def add(body, name, m, pos, inertia=None):
        e = out.setdefault(body, {"items": []})
        e["items"].append({"name": name, "mass": float(m), "pos": [float(p) for p in pos],
                           "inertia": inertia})

    # printed structure
    for bname, b in cfg.bodies.items():
        if bname in cadmp:
            mp = cadmp[bname]
            add(bname, "printed", mp["mass"], mp["com"], mp.get("inertia"))
        else:
            m = cfg["mass_guess"].get(b.base, 0.2)
            add(bname, "guess", m, _default_com(cfg, b.base))
    # torso equipment
    for k, v in cfg["torso_equipment"].items():
        add("torso", k, v["mass"], v["pos"])
    # actuators
    for j in cfg.joints.values():
        child = cfg.body_of_joint(j.name)
        parent = child.parent
        if j.actuator_type == "servo":
            sm = cfg["servo_models"][j.servo_model]
            add(parent, f"servo_{j.name}", sm["mass"], list(child.origin))
        else:
            add(parent, f"jointhw_{j.name}", JOINT_HW, list(child.origin))
            if sizing and j.base in sizing:
                lk, cyl = linkage_for(cfg, j.name, sizing)
                m = cyl.mass(cfg)
                o = np.array(child.origin)
                pmid = o + 0.65 * lk.parent + 0.35 * lk.child
                add(parent, f"cyl_{j.name}", 0.6 * m, pmid)
                add(child.name, f"cylrod_{j.name}", 0.4 * m, 0.5 * lk.child)
                vm = (cfg.hyd["valve"]["mass_g"] + cfg.hyd["valve"]["servo"]["mass_g"]) / 1000 + VALVE_PACK_EXTRA
                add(parent, f"valves_{j.name}", vm, o + 1.1 * lk.parent)
            else:
                add(parent, f"cyl_{j.name}", 0.45, list(child.origin))
                add(parent, f"valves_{j.name}", 0.15, list(child.origin))
    if payload_kg:
        for s in ("L", "R"):
            add(f"{s}_hand", "payload", payload_kg, [0, 0, -0.06])
    # combine into mass, com, inertia (point masses + per-item inertia or a small sphere)
    for bname, e in out.items():
        M = sum(i["mass"] for i in e["items"])
        com = sum(np.array(i["pos"]) * i["mass"] for i in e["items"]) / max(M, 1e-9)
        I = np.zeros((3, 3))
        for i in e["items"]:
            r = np.array(i["pos"]) - com
            I += i["mass"] * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
            if i["inertia"] is not None:
                I += np.array(i["inertia"])
            else:
                rr = 0.03
                I += 0.4 * i["mass"] * rr * rr * np.eye(3)
        e.update(mass=float(M), com=com.tolist(), inertia=I.tolist())
    return out


def _default_com(cfg, base):
    g = cfg["geometry"]
    table = {
        "pelvis": [0, 0, -0.02], "torso": [0, 0, 0.18],
        "hip_yaw_link": [0, 0, 0.03], "hip_roll_link": [0, 0, 0],
        "thigh": [0, 0, -g["thigh_length"] * 0.4], "shin": [0, 0, -g["shin_length"] * 0.4],
        "ankle_link": [0, 0, -0.015], "foot": [0.03, 0, -0.04],
        "shoulder_link": [0, 0.02, 0], "upper_arm_roll": [0, 0, -0.03],
        "upper_arm": [0, 0, -0.09], "forearm": [0, 0, -0.10], "hand": [0, 0, -0.05],
        "neck_link": [0, 0, 0.015], "head": [0, 0, 0.07],
    }
    return table.get(base, [0, 0, 0])


def _visual_geoms(cfg: RobotConfig, b) -> list[str]:
    g = cfg["geometry"]
    base = b.base
    s = -1 if b.side == "R" else 1
    vis = 'contype="0" conaffinity="0" group="1" rgba="0.75 0.75 0.78 1"'
    hyd = 'contype="0" conaffinity="0" group="1" rgba="0.85 0.45 0.1 1"'
    out = []
    if base == "pelvis":
        out.append(f'<geom type="box" size="0.07 0.13 0.04" pos="0 0 -0.02" {vis}/>')
    elif base == "torso":
        t = g["torso"]
        out.append(f'<geom type="box" size="{t["depth"]/2} {t["width"]/2} {t["height"]/2 - 0.02}" '
                   f'pos="0 0 {t["height"]/2 + 0.01}" {vis}/>')
    elif base == "thigh":
        out.append(f'<geom type="capsule" fromto="0 0 0 0 0 {-g["thigh_length"]}" size="0.035" {vis}/>')
    elif base == "shin":
        out.append(f'<geom type="capsule" fromto="0 0 0 0 0 {-g["shin_length"]}" size="0.032" {vis}/>')
    elif base == "upper_arm":
        out.append(f'<geom type="capsule" fromto="0 0 0 0 0 -0.18" size="0.028" {vis}/>')
    elif base == "forearm":
        out.append(f'<geom type="capsule" fromto="0 0 0 0 0 {-g["forearm_length"]}" size="0.025" {vis}/>')
    elif base == "hand":
        out.append(f'<geom type="box" size="0.03 0.02 0.05" pos="0 0 -0.05" {vis}/>')
    elif base == "head":
        h = g["head"]["size"]
        out.append(f'<geom type="box" size="{h[0]/2} {h[1]/2} {h[2]/2}" pos="0 0 {h[2]/2 + 0.01}" {vis}/>')
    elif base in ("hip_yaw_link", "hip_roll_link", "ankle_link", "shoulder_link", "upper_arm_roll", "neck_link"):
        out.append(f'<geom type="sphere" size="0.03" {vis}/>')
    return out


def build_mjcf(cfg: RobotConfig, payload_kg: float = 0.0, floor: bool = True) -> str:
    g = cfg["geometry"]
    sim = cfg["sim"]
    mm = body_mass_model(cfg, payload_kg)
    foot = g["foot"]

    def body_xml(name: str, depth: int) -> str:
        b = cfg.bodies[name]
        ind = "  " * depth
        e = mm.get(name)
        lines = []
        if name == "pelvis":
            lines.append(f'{ind}<body name="pelvis" pos="0 0 {sim["standing_pelvis_height"] + 0.04}">')
            lines.append(f'{ind}  <freejoint name="root"/>')
            lines.append(f'{ind}  <site name="imu" pos="0 0 0.2" size="0.01"/>')
        else:
            lines.append(f'{ind}<body name="{name}" pos="{_fmt(b.origin)}">')
            if b.joint and not b.fixed and b.joint.in_sim:
                j = b.joint
                lo, hi = j.range_rad
                arm = 0.02 if j.actuator_type == "hydraulic" else 0.03
                lines.append(f'{ind}  <joint name="{j.name}" type="hinge" axis="{_fmt(j.axis_vec)}" '
                             f'range="{lo:.5f} {hi:.5f}" limited="true" damping="0.6" armature="{arm}"'
                             f' frictionloss="0.05"/>')
        if e and e["mass"] > 0:
            I = np.array(e["inertia"])
            I = 0.5 * (I + I.T) + 1e-6 * np.eye(3)
            lines.append(f'{ind}  <inertial pos="{_fmt(e["com"])}" mass="{e["mass"]:.5f}" '
                         f'fullinertia="{I[0,0]:.6g} {I[1,1]:.6g} {I[2,2]:.6g} {I[0,1]:.6g} {I[0,2]:.6g} {I[1,2]:.6g}"/>')
        for gx in _visual_geoms(cfg, b):
            lines.append(f"{ind}  {gx}")
        if b.base == "foot":
            cx = foot["length"] / 2 - foot["heel"]
            cz = -g["ankle_height"] + foot["thickness"] / 2
            lines.append(f'{ind}  <geom name="{name}_sole" type="box" size="{foot["length"]/2} {foot["width"]/2} '
                         f'{foot["thickness"]/2}" pos="{cx:.4f} 0 {cz:.4f}" friction="1.0 0.02 0.002" '
                         f'rgba="0.2 0.2 0.25 1" condim="3"/>')
            lines.append(f'{ind}  <site name="{name}_sole" pos="0 0 {-g["ankle_height"]:.4f}" size="0.01"/>')
        if b.base == "hand":
            lines.append(f'{ind}  <site name="{name}_grip" pos="0 0 -0.06" size="0.01"/>')
        for c in b.children:
            lines.append(body_xml(c, depth + 1))
        lines.append(f"{ind}</body>")
        return "\n".join(lines)

    # hand has a sim:false joint (gripper) -> attach hand as fixed part of forearm mass instead
    acts = []
    for j in cfg.joints.values():
        if not j.in_sim:
            continue
        leg = any(k in j.base for k in ("hip", "knee", "ankle"))
        kp = sim["pd"]["kp"] * (3.0 if leg else 0.8)
        kv = sim["pd"]["kd"] * (2.0 if leg else 0.6)
        lo, hi = j.range_rad
        acts.append(f'    <position name="{j.name}" joint="{j.name}" kp="{kp:.1f}" kv="{kv:.2f}" '
                    f'ctrlrange="{lo:.5f} {hi:.5f}" forcerange="-400 400"/>')
    xml = f"""<mujoco model={quoteattr(cfg['meta']['name'])}>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{sim['timestep']}" integrator="implicitfast" gravity="0 0 -9.81">
    <flag warmstart="enable"/>
  </option>
  <default>
    <geom solref="0.004 1" solimp="0.95 0.99 0.001"/>
  </default>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".85 .85 .85" rgb2=".7 .7 .7" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    {'<geom name="floor" type="plane" size="10 10 0.1" material="grid" friction="1.0 0.02 0.002"/>' if floor else ''}
{body_xml('pelvis', 2)}
  </worldbody>
  <actuator>
{chr(10).join(acts)}
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
  </sensor>
</mujoco>
"""
    return xml


def total_mass(cfg: RobotConfig, payload_kg: float = 0.0) -> float:
    return sum(e["mass"] for e in body_mass_model(cfg, payload_kg).values())


def load_model(cfg: RobotConfig, payload_kg: float = 0.0):
    import mujoco
    return mujoco.MjModel.from_xml_string(build_mjcf(cfg, payload_kg))
