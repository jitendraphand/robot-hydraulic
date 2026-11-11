"""Load, validate and expand config/robot.yaml (the single source of truth).

The raw YAML describes the left side of the robot; this module expands
``side: LR`` entries into mirrored ``L_`` and ``R_`` bodies/joints and exposes
convenient typed views used by every other subsystem.
"""
from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "robot.yaml"
BUILD = ROOT / "build"
DERIVED = ROOT / "config" / "derived"   # committed, generated design-loop outputs

AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
# in-plane coordinate indices (u, v) for a joint about each axis
PLANE = {"x": (1, 2), "y": (0, 2), "z": (0, 1)}


@dataclass
class HydraulicSpec:
    plane_offset: float
    valve_on: str
    anchor_bounds: dict[str, list[list[float]]]   # {'parent': [[ulo,uhi],[vlo,vhi]], 'child': ...}


@dataclass
class Joint:
    name: str              # e.g. L_knee
    base: str              # e.g. knee
    side: str              # 'L', 'R' or ''
    axis: str
    range_deg: tuple[float, float]
    actuator_type: str     # hydraulic | servo
    servo_model: str | None = None
    hydraulic: HydraulicSpec | None = None
    in_sim: bool = True

    @property
    def range_rad(self) -> tuple[float, float]:
        return (math.radians(self.range_deg[0]), math.radians(self.range_deg[1]))

    @property
    def axis_vec(self):
        return AXES[self.axis]


@dataclass
class Body:
    name: str
    base: str
    side: str
    parent: str
    origin: tuple[float, float, float]
    joint: Joint | None
    fixed: bool = False
    children: list[str] = field(default_factory=list)


def _mirror_body(raw: dict, side: str) -> dict:
    b = copy.deepcopy(raw)
    if side == "R":
        o = list(b.get("origin", [0, 0, 0]))
        o[1] = -o[1]
        b["origin"] = o
        j = b.get("joint")
        if j:
            ax = j["axis"]
            if j.get("mirror_range"):
                lo, hi = j["range_deg"]
                j["range_deg"] = [-hi, -lo]
            act = j.get("actuator", {})
            if act.get("type") == "hydraulic":
                if ax == "y":
                    act["plane_offset"] = -act["plane_offset"]
                elif ax == "x":
                    for k in ("parent", "child"):
                        lo, hi = act["anchors"][k]["u"]
                        act["anchors"][k]["u"] = [-hi, -lo]
    return b


class RobotConfig:
    """Typed, expanded view of robot.yaml."""

    def __init__(self, path: Path | str = CONFIG_PATH, overrides: dict | None = None):
        self.path = Path(path)
        with open(self.path) as f:
            self.raw: dict[str, Any] = yaml.safe_load(f)
        self.overrides = overrides
        if overrides:
            _deep_update(self.raw, overrides)
        self.bodies: dict[str, Body] = {}
        self.joints: dict[str, Joint] = {}
        self._expand()

    # ------------------------------------------------------------------ build
    def _expand(self) -> None:
        sided_bases = {b["name"] for b in self.raw["bodies"] if b.get("side") == "LR"}
        for rb in self.raw["bodies"]:
            sides = ["L", "R"] if rb.get("side") == "LR" else [""]
            for s in sides:
                b = _mirror_body(rb, s)
                pre = f"{s}_" if s else ""
                name = pre + b["name"]
                parent = b["parent"]
                if s and parent in sided_bases:
                    parent = pre + parent
                joint = None
                if "joint" in b:
                    jr = b["joint"]
                    act = jr.get("actuator", {"type": "servo"})
                    hyd = None
                    if act["type"] == "hydraulic":
                        hyd = HydraulicSpec(
                            plane_offset=float(act["plane_offset"]),
                            valve_on=act.get("valve_on", "parent"),
                            anchor_bounds={k: [list(map(float, act["anchors"][k]["u"])),
                                               list(map(float, act["anchors"][k]["v"]))]
                                           for k in ("parent", "child")},
                        )
                    joint = Joint(
                        name=pre + jr["name"], base=jr["name"], side=s, axis=jr["axis"],
                        range_deg=(float(jr["range_deg"][0]), float(jr["range_deg"][1])),
                        actuator_type=act["type"], servo_model=act.get("model"),
                        hydraulic=hyd, in_sim=jr.get("sim", True),
                    )
                    self.joints[joint.name] = joint
                self.bodies[name] = Body(
                    name=name, base=b["name"], side=s, parent=parent,
                    origin=tuple(float(v) for v in b.get("origin", [0, 0, 0])),
                    joint=joint, fixed=b.get("fixed", False),
                )
        for b in self.bodies.values():
            if b.parent in self.bodies:
                self.bodies[b.parent].children.append(b.name)

    # ------------------------------------------------------------- accessors
    def __getitem__(self, k):
        return self.raw[k]

    @property
    def hydraulic_joints(self) -> list[Joint]:
        return [j for j in self.joints.values() if j.actuator_type == "hydraulic"]

    @property
    def servo_joints(self) -> list[Joint]:
        return [j for j in self.joints.values() if j.actuator_type == "servo"]

    def body_of_joint(self, jname: str) -> Body:
        for b in self.bodies.values():
            if b.joint and b.joint.name == jname:
                return b
        raise KeyError(jname)

    @property
    def hyd(self) -> dict:
        return self.raw["hydraulics"]

    def pa(self, key: str) -> float:
        """Hydraulic pressure in Pa from a *_bar key."""
        return float(self.hyd[key]) * 1e5

    @property
    def usable_dp(self) -> float:
        return (self.pa("supply_pressure_bar") - self.pa("return_pressure_bar")) * \
            float(self.hyd["force_pressure_fraction"])

    def node_of_joint(self, jname: str) -> tuple[dict, int]:
        for n in self.raw["electrical"]["nodes"]:
            if jname in n["joints"]:
                return n, n["joints"].index(jname)
        raise KeyError(jname)

    def derived(self, name: str) -> dict | None:
        p = DERIVED / f"{name}.json"
        if p.exists():
            return json.loads(p.read_text())
        return None


def config_hash(path: Path | str = CONFIG_PATH) -> str:
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def save_derived(name: str, data: dict) -> Path:
    """Write a design-loop artefact and record which robot.yaml revision produced it."""
    DERIVED.mkdir(parents=True, exist_ok=True)
    p = DERIVED / f"{name}.json"
    p.write_text(json.dumps(data, indent=2, sort_keys=True, default=_jsonable) + "\n")
    man = DERIVED / "manifest.json"
    m = json.loads(man.read_text()) if man.exists() else {}
    m[name] = config_hash()
    man.write_text(json.dumps(m, indent=2, sort_keys=True) + "\n")
    return p


def _jsonable(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    raise TypeError(type(o))


def _deep_update(d: dict, u: dict) -> None:
    for k, v in u.items():
        if isinstance(v, dict) and isinstance(d.get(k), dict):
            _deep_update(d[k], v)
        else:
            d[k] = v


_cached: RobotConfig | None = None


def load(path: Path | str = CONFIG_PATH) -> RobotConfig:
    global _cached
    if _cached is None or Path(path) != _cached.path:
        _cached = RobotConfig(path)
    return _cached
