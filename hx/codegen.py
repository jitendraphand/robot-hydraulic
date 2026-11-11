"""Generate firmware headers from robot.yaml + design-loop outputs.

  firmware/core/include/hx/robot_config.h   per-node joint parameter tables, HPU params
  firmware/core/include/hx/can_protocol.h   CAN IDs + frame layouts (from hx/protocol.py)
  firmware/platform/esp32s3/include/pins.h  pin map (from electronics netlist)
"""
from __future__ import annotations

import math

import numpy as np

from .config import ROOT, RobotConfig, save_derived
from . import protocol

FW = ROOT / "firmware"


def _reflected_inertia(cfg: RobotConfig) -> dict:
    """Diagonal of the joint-space mass matrix in the standing pose (kg m^2)."""
    import mujoco
    from .sim.model import build_mjcf
    m = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    return {m.joint(i).name: float(M[m.joint(i).dofadr[0], m.joint(i).dofadr[0]])
            for i in range(m.njnt) if m.joint(i).type == mujoco.mjtJoint.mjJNT_HINGE}


def hose_volume(cfg) -> float:
    h = cfg.hyd["hose"]
    return math.pi / 4 * (h["id_mm"] * 1e-3) ** 2 * h["ab_length_m"] + 1.5e-6     # + fitting/port volume


def joint_params(cfg: RobotConfig, jname: str, sizing: dict, demand: dict, inertia: dict) -> dict:
    from .hydraulics import linkage_for
    j = cfg.joints[jname]
    c = cfg["control"]
    h = cfg.hyd
    lo, hi = j.range_rad
    peak = max((demand.get(jname) or {}).get("peak_tau", 1.0), 0.5)
    J = max(inertia.get(jname, 0.05), 0.01)
    kp = peak / math.radians(c["impedance_stiffness_deg"])
    kd = 2 * 0.8 * math.sqrt(kp * J)
    P = dict(q_min=lo, q_max=hi, soft_margin=math.radians(c["soft_limit_margin_deg"]),
             actuator=0 if j.actuator_type == "hydraulic" else 1, axis="xyz".index(j.axis),
             anchor_parent=[0, 0, 0], anchor_child=[0, 0, 0], a_bore=0.0, a_annulus=0.0, min_moment_arm=0.005,
             p_supply=cfg.pa("supply_pressure_bar"), p_return=cfg.pa("return_pressure_bar"),
             p_trip=c["pressure_trip_bar"] * 1e5, p_supply_min=0.6 * cfg.pa("supply_pressure_bar"),
             valve_q_rated=h["valve"]["rated_flow_lpm"] / 60000.0, valve_dp_rated=h["valve"]["rated_dp_bar"] * 1e5,
             valve_overlap=h["valve"]["overlap"], valve_deadband=c["valve_deadband"],
             valve_rate=2.0 / (h["valve"]["servo"]["rate_full_travel_ms"] / 1000.0),
             kp=kp, ki=0.0, kd=kd, kp_pos_valve=0.0, kd_pos_valve=0.0, ki_pos_valve=0.0, kp_force=0.0, ki_force=0.0,
             k_vel_ff=1.0, tau_max=1.0, max_following_err=math.radians(c["max_following_error_deg"]),
             following_time=0.25, servo_center_us=1500.0, servo_us_per_rad=636.6)
    if j.actuator_type == "hydraulic":
        lk, cyl = linkage_for(cfg, jname, sizing)
        info = sizing[j.base]
        fmax = cfg.usable_dp * cyl.a_bore
        dp = cfg.pa("supply_pressure_bar") - cfg.pa("return_pressure_bar")
        q_av = P["valve_q_rated"] * math.sqrt(dp / P["valve_dp_rated"])
        m_mean = 0.5 * (info["min_moment_arm_mm"] + info["max_moment_arm_mm"]) / 1000
        w_max = q_av / cyl.a_bore / m_mean               # rad/s at full valve
        # force loop: chamber pressure is an integrator, dF/dt ~= G u with
        # G = A * beta * Q / V (mid-stroke volume incl. hose) -> kp = w_c / G
        V = hose_volume(cfg) + cyl.a_bore * cyl.stroke_mm / 2000.0
        G = cyl.a_bore * h["fluid"]["effective_bulk_modulus_pa"] * q_av / V
        wf = 2 * math.pi * c["force_bandwidth_hz"]
        wp = 2 * math.pi * c["position_bandwidth_hz"]
        P.update(anchor_parent=[float(v) for v in lk.parent], anchor_child=[float(v) for v in lk.child],
                 a_bore=cyl.a_bore, a_annulus=cyl.a_annulus, min_moment_arm=0.5 * info["min_moment_arm_mm"] / 1000,
                 kp_force=wf / G, ki_force=0.5 * wf * wf / G / 10.0,
                 kp_pos_valve=wp / w_max, kd_pos_valve=0.0, ki_pos_valve=0.2 * wp * wp / w_max / 9.0,
                 tau_max=float(min(1.6 * peak, max(info["cap_pos_max_nm"], info["cap_neg_max_nm"]))),
                 servo_us_per_rad=h["valve"]["servo"]["pulse_span_us"])
    else:
        sm = cfg["servo_models"][j.servo_model]
        P["tau_max"] = sm["stall_nm"]
    return P


def _cfloat(v):
    t = f"{float(v):.7g}"
    if "." not in t and "e" not in t and "n" not in t:
        t += ".0"
    return t + "f"


def _params_c(P: dict) -> str:
    order = ["q_min", "q_max", "soft_margin", "actuator", "axis", "anchor_parent", "anchor_child", "a_bore",
             "a_annulus", "min_moment_arm", "p_supply", "p_return", "p_trip", "p_supply_min", "valve_q_rated",
             "valve_dp_rated", "valve_overlap", "valve_deadband", "valve_rate", "kp", "ki", "kd", "kp_pos_valve",
             "kd_pos_valve", "ki_pos_valve", "kp_force", "ki_force", "k_vel_ff", "tau_max", "max_following_err", "following_time",
             "servo_center_us", "servo_us_per_rad"]
    parts = []
    for k in order:
        v = P[k]
        if isinstance(v, list):
            parts.append("{" + ", ".join(_cfloat(x) for x in v) + "}")
        elif k in ("actuator", "axis"):
            parts.append(str(int(v)))
        else:
            parts.append(_cfloat(v))
    return "{" + ", ".join(parts) + "}"


def all_joint_params(cfg: RobotConfig) -> dict:
    sizing = cfg.derived("sizing") or {}
    demand = cfg.derived("demand") or {}
    inertia = _reflected_inertia(cfg)
    return {jn: joint_params(cfg, jn, sizing, demand, inertia) for jn in cfg.joints}


def robot_config_h(cfg: RobotConfig, params: dict) -> str:
    h = cfg.hyd
    c = cfg["control"]
    nodes = cfg["electrical"]["nodes"]
    L = ["// GENERATED by `python -m hx gen` from config/robot.yaml + config/derived/*.json.",
         "// Do not edit by hand: change robot.yaml and regenerate.",
         "#pragma once", "#include \"hx/joint_types.h\"", "#include \"hx/control.h\"", "",
         "namespace hx { namespace cfg {", "",
         f"constexpr const char* kRobotName = \"{cfg['meta']['name']}\";",
         f"constexpr const char* kConfigVersion = \"{cfg['meta']['version']}\";",
         f"constexpr float kLoopHz = {c['loop_hz']}.0f;",
         f"constexpr float kHeartbeatTimeout = {c['heartbeat_timeout_ms'] / 1000:.3f}f;",
         f"constexpr int kNodeCount = {len(nodes)};", "constexpr int kMaxJointsPerNode = 8;", ""]
    L.append("struct NodeDef { uint8_t id; const char* name; uint8_t joint_count; const char* joint_names[8]; hx_joint_params_t joints[8]; };")
    L.append("")
    L.append("constexpr NodeDef kNodes[kNodeCount] = {")
    for n in nodes:
        names = ", ".join(f"\"{j}\"" for j in n["joints"])
        L.append(f"  {{ {n['id']}, \"{n['name']}\", {len(n['joints'])}, {{ {names} }}, {{")
        for jn in n["joints"]:
            L.append(f"    /* {jn:18s} */ {_params_c(params[jn])},")
        L.append("  }},")
    L.append("};")
    hp = h["hpu"]
    ps = cfg.pa("supply_pressure_bar")
    L += ["", "constexpr HpuParams kHpu = {",
          f"  {_cfloat(hp['pressure_band_bar'][1] * 1e5)}, {_cfloat(hp['pressure_band_bar'][0] * 1e5)}, {_cfloat(hp['pressure_band_bar'][1] * 1e5 + 0.3e5)},",
          f"  {_cfloat(cfg.pa('relief_pressure_bar') + 0.5e5)}, {_cfloat(hp['max_oil_temp_c'])},",
          f"  {_cfloat(1.0 / (2.0 * ps * 0.1))}, {_cfloat(1.0 / (2.0 * ps * 0.1) * 4)}, 0.15f, 1.0f, 3.0f }};",
          "",
          f"constexpr float kPressureRangePa = {_cfloat(h['sensor']['range_bar'] * 1e5)};",
          f"constexpr float kPressureVlo = {_cfloat(h['sensor']['v_lo'])}, kPressureVhi = {_cfloat(h['sensor']['v_hi'])};",
          "constexpr float kPressureDivider = 0.6667f;   // 10k/20k divider into 3.3 V ADC",
          "", "} }  // namespace hx::cfg", ""]
    return "\n".join(L)


def generate_all(cfg: RobotConfig, log=print):
    inc = FW / "core" / "include" / "hx"
    params = all_joint_params(cfg)
    save_derived("joint_params", params)
    (inc / "robot_config.h").write_text(robot_config_h(cfg, params))
    (inc / "can_protocol.h").write_text(protocol.c_header())
    log("  [gen] robot_config.h, can_protocol.h")
    try:
        from .electronics.netlist import generate_outputs
        generate_outputs(cfg, log)
    except ImportError:
        pass
