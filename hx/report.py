"""Generate docs/00_design_summary.md from the design-loop outputs (never stale)."""
from __future__ import annotations

import math

from .config import ROOT, RobotConfig, config_hash


def design_summary(cfg: RobotConfig) -> str:
    from .hydraulics import accumulator_usable_l, pump_flow_lpm
    from .sim.model import body_mass_model
    sz = cfg.derived("sizing") or {}
    sim = cfg.derived("sim_summary") or {}
    hpu = cfg.derived("hpu") or {}
    sil = cfg.derived("sil") or {}
    el = cfg.derived("electronics") or {}
    plan = (cfg.derived("print_plan") or {}).get("parts", [])
    dl = cfg.derived("design_loop") or {}
    mm = body_mass_model(cfg)
    total = sum(v["mass"] for v in mm.values())
    printed = sum(e["mass_kg_each"] * e["qty"] for e in plan)
    hours = sum(e["print_h_each"] * e["qty"] for e in plan)
    L = [f"# {cfg['meta']['name']} design summary", "",
         f"Auto-generated from `config/robot.yaml` (`{config_hash()}`) and the design-loop outputs in "
         "`config/derived/`. Regenerate with `make all`.", "",
         "## Key figures", "",
         "| quantity | value |", "|---|---|",
         f"| total mass (no payload) | {total:.2f} kg |",
         f"| printed structure | {printed:.2f} kg, ~{hours:.0f} h printing on the Neptune 4 Max |",
         f"| height (standing) | ~{cfg['sim']['standing_pelvis_height'] + cfg['geometry']['neck_height'] + 0.18:.2f} m |",
         f"| actuated DOF | {len(cfg.joints)} ({len(cfg.hydraulic_joints)} hydraulic, {len(cfg.servo_joints)} servo) |",
         f"| supply / relief pressure | {cfg.hyd['supply_pressure_bar']} / {cfg.hyd['relief_pressure_bar']} bar |",
         f"| pump flow / accumulator usable | {pump_flow_lpm(cfg):.2f} L/min / {accumulator_usable_l(cfg) * 1000:.0f} mL |",
         f"| design loop | {'converged' if dl.get('converged') else 'NOT converged'} in {dl.get('iterations', '?')} iterations |", ""]
    L += ["## Hydraulic actuators (left side; right is mirrored)", "",
          "| joint | bore x stroke (mm) | rod | moment arm (mm) | peak demand (N m) | capacity ratio | peak flow (L/min) | cylinder mass (g) |",
          "|---|---|---|---|---|---|---|---|"]
    for k, v in sz.items():
        c = v["cylinder"]
        L.append(f"| {k} | {c['bore_mm']:.0f} x {c['stroke_mm']:.0f} | {c['rod_mm']:.0f} | "
                 f"{v['min_moment_arm_mm']:.0f}-{v['max_moment_arm_mm']:.0f} | {v['peak_demand_nm']:.1f} | "
                 f"{v['ratio_demand']:.2f} | {v['peak_flow_lpm']:.2f} | {v['mass_kg'] * 1000:.0f} |")
    L += ["", "Capacity ratio = min over every simulated (angle, torque) sample of capacity / "
          f"({cfg.hyd['sizing']['torque_safety_factor']} x demand); >= 1 required.", "",
          "## Simulation scenarios", "", "| scenario | stable | max tilt (deg) | mass (kg) |", "|---|---|---|---|"]
    for k, v in sim.items():
        L.append(f"| {k} | {'yes' if not v['fell'] else '**FELL**'} | {v['max_tilt_deg']:.1f} | {v['mass_kg']:.1f} |")
    if hpu:
        L += ["", "## HPU (pump + accumulator vs scenario flow)", "", "| scenario | peak flow L/min | mean flow L/min | min supply bar | pump duty |",
              "|---|---|---|---|---|"]
        for k, v in hpu.items():
            if isinstance(v, dict):
                L.append(f"| {k} | {v['peak_flow_lpm']:.2f} | {v['mean_flow_lpm']:.2f} | {v['min_supply_bar']:.1f} | {v['pump_duty']:.2f} |")
    if sil:
        L += ["", "## Hydraulic SIL (real firmware, lumped plant, 15 % feed-forward error)", "",
              "| joint | step overshoot | settle 5 % | walk RMS / max err (deg) | squat max err (deg) | peak p (bar) | result |",
              "|---|---|---|---|---|---|---|"]
        for k, r in sil.items():
            L.append(f"| {k} | {r['step']['overshoot'] * 100:.1f} % | {r['step']['settle_5pct_s']:.2f} s | "
                     f"{r['walk']['rms_err_deg']:.2f} / {r['walk']['max_err_deg']:.2f} | {r['squat']['max_err_deg']:.2f} | "
                     f"{max(r['walk']['peak_pressure_bar'], r['squat']['peak_pressure_bar']):.1f} | {'PASS' if r['pass'] else 'FAIL'} |")
        L += ["", "![SIL knee](img/sil_knee.png)"]
    if el:
        p = el["power"]
        L += ["", "## Electrical", "",
              f"- Battery: nominal {p['battery_nominal_w']:.0f} W, peak {p['battery_peak_w']:.0f} W, runtime ~{p['runtime_min']:.0f} min",
              "- CAN load: " + ", ".join(f"{k} {v['load'] * 100:.0f} %" for k, v in el["can"].items()),
              f"- ERC: {sum(1 for i in el['erc'] if i['severity'] == 'error')} errors, "
              f"{sum(1 for i in el['erc'] if i['severity'] == 'warning')} warnings"]
    L += ["", "## Mass by body", "", "| body | mass (kg) |", "|---|---|"]
    for k, v in sorted(mm.items()):
        L.append(f"| {k} | {v['mass']:.3f} |")
    L += ["", "## Renders", "", "![nominal](img/assembly_zero.png)", "![squat](img/assembly_squat.png)",
          "![stride](img/assembly_stride.png)", "![arms](img/assembly_arms.png)", "![parts](img/parts_sheet.png)", ""]
    return "\n".join(L)


def write(cfg: RobotConfig):
    (ROOT / "docs" / "00_design_summary.md").write_text(design_summary(cfg))
