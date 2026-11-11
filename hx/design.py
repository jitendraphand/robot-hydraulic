"""Recursive design loop (fixed-point iteration over the coupled subsystems).

    masses --> whole-body sim --> torque/velocity demand --> cylinder + linkage
      ^                                                        sizing
      |                                                          |
      +---- CAD mass properties <---- parametric CAD <-----------+

The loop iterates until cylinder selections are unchanged and the total
mass changes by less than ``tol`` (relative).  Every iteration is logged to
config/derived/design_loop.json so the convergence itself is auditable.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np

from .config import BUILD, RobotConfig, save_derived
from . import hydraulics as H


def filter_demand(rec: dict, window: int = 4) -> dict:
    """Moving-average the torque over the actuator bandwidth (20 ms @ 200 Hz).

    Foot-strike impulses shorter than the hydraulic bandwidth are absorbed by
    fluid/hose compliance and the structure, not by the valve-controlled force.
    """
    out = {}
    k = np.ones(window) / window
    for n, r in rec.items():
        tau = np.asarray(r["tau"])
        if len(tau) > window:
            tau = np.convolve(tau, k, mode="same")
        out[n] = {"q": list(map(float, r["q"])), "qdot": list(map(float, r["qdot"])),
                  "tau": list(map(float, tau))}
    return out


def run_sim(cfg: RobotConfig, log=print):
    from .sim.scenarios import SCENARIOS, run

    demand, summaries, series = {}, {}, {}
    for S in SCENARIOS:
        sc = S(cfg)
        rec, summ = run(sc, log)
        rec = filter_demand(rec)
        summaries[sc.name] = summ
        series[sc.name] = rec
        for n, r in rec.items():
            dd = demand.setdefault(n, {"q": [], "qdot": [], "tau": []})
            for k in ("q", "qdot", "tau"):
                dd[k] += r[k]
    return demand, summaries, series


def envelope(demand: dict) -> dict:
    """Compact per-joint envelope for committing (angle-binned extremes)."""
    out = {}
    for n, d in demand.items():
        c = H.compress_demand(d, bins=60)
        c["peak_tau"] = float(np.max(np.abs(d["tau"]))) if d["tau"] else 0.0
        c["rms_tau"] = float(np.sqrt(np.mean(np.square(d["tau"])))) if d["tau"] else 0.0
        c["peak_qdot"] = float(np.max(np.abs(d["qdot"]))) if d["qdot"] else 0.0
        c["q_range"] = [float(np.min(d["q"])), float(np.max(d["q"]))] if d["q"] else [0, 0]
        out[n] = c
    return out


def hpu_check(cfg: RobotConfig, series: dict, sizing: dict) -> dict:
    """Simulate supply flow vs pump + accumulator for every scenario."""
    h = cfg.hyd["hpu"]
    qp = H.pump_flow_lpm(cfg) / 60000.0              # m^3/s
    v0 = h["accumulator_volume_l"] / 1000
    p0 = (h["accumulator_precharge_bar"] + 1) * 1e5
    plo, phi = [(b + 1) * 1e5 for b in h["pressure_band_bar"]]
    leak = cfg.hyd["valve"]["leakage_lpm"] / 60000.0 * len(cfg.hydraulic_joints)
    res = {}
    for sname, rec in series.items():
        n = len(next(iter(rec.values()))["q"])
        dt = 0.005
        flow = np.zeros(n)
        for j in cfg.hydraulic_joints:
            if j.name not in rec:
                continue
            lk, cyl = H.linkage_for(cfg, j.name, sizing)
            q = np.asarray(rec[j.name]["q"])
            qd = np.asarray(rec[j.name]["qdot"])
            v = lk.moment_arm(q) * qd                 # piston speed (+ = extend)
            area = np.where(v > 0, cyl.a_bore, cyl.a_annulus)
            flow += np.abs(v) * area
        flow += leak
        # accumulator gas volume, isothermal; pump bang-bang on pressure band
        vg = v0 * p0 / phi
        pump_on = False
        pmin = phi
        duty = 0
        for k in range(n):
            p = p0 * v0 / vg
            if p < plo:
                pump_on = True
            if p > phi:
                pump_on = False
            qin = qp if pump_on else 0.0
            duty += pump_on
            vg += (flow[k] - qin) * dt              # oil out -> gas expands
            vg = min(max(vg, 1e-6), v0)
            pmin = min(pmin, p0 * v0 / vg)
        res[sname] = dict(peak_flow_lpm=float(flow.max() * 60000), mean_flow_lpm=float(flow.mean() * 60000),
                          min_supply_bar=float(pmin / 1e5 - 1), pump_duty=float(duty / max(n, 1)),
                          hydraulic_power_w=float(flow.mean() * cfg.pa("supply_pressure_bar")))
    res["pump_flow_lpm"] = H.pump_flow_lpm(cfg)
    res["accumulator_usable_l"] = H.accumulator_usable_l(cfg)
    return res


def save_traces(cfg, series: dict):
    """Per-joint reference traces (walk + squat, 100 Hz) for the hydraulic SIL tests."""
    out = {}
    for sc in ("walk", "squat"):
        rec = series.get(sc, {})
        for j in cfg.hydraulic_joints:
            if j.side != "L" or j.name not in rec:
                continue
            r = rec[j.name]
            out.setdefault(sc, {})[j.name] = {k: [round(v, 5) for v in r[k][::2]] for k in ("q", "qdot", "tau")}
    out["dt"] = 0.01
    save_derived("sim_traces", out)


def save_sim(summaries: dict):
    poses = {}
    for sn, sm in summaries.items():
        for i, f in enumerate(sm.pop("frames", [])):
            poses[f"{sn}_{i:02d}"] = f
    save_derived("sim_summary", summaries)
    save_derived("sim_poses", poses)


def loop(cfg: RobotConfig | None = None, max_iter: int = 4, tol: float = 0.02, build_cad: bool = True,
         log=print) -> dict:
    cfg = cfg or RobotConfig()
    history = []
    prev_mass, prev_sel = None, None
    for it in range(1, max_iter + 1):
        t0 = time.time()
        log(f"[design-loop] iteration {it}")
        cfg = RobotConfig(cfg.path)                     # reload derived inputs
        demand, summaries, series = run_sim(cfg, log)
        env = envelope(demand)
        save_derived("demand", env)
        save_sim(summaries)
        save_traces(cfg, series)
        sizing = H.size_all(cfg, demand, log, previous=cfg.derived("sizing"))
        save_derived("sizing", sizing)
        cfg = RobotConfig(cfg.path)
        hpu = hpu_check(cfg, series, sizing)
        save_derived("hpu", hpu)
        if build_cad:
            from .cad.build import build_all
            build_all(cfg, log=log)
            cfg = RobotConfig(cfg.path)
        from .sim.model import total_mass
        mass = total_mass(cfg)
        sel = {k: (v["bore"], v["stroke_mm"]) for k, v in sizing.items()}
        rel = abs(mass - prev_mass) / mass if prev_mass else math.inf
        entry = dict(iteration=it, total_mass_kg=mass, mass_change=rel if math.isfinite(rel) else None,
                     selection={k: list(v) for k, v in sel.items()},
                     all_feasible=all(v["feasible"] for v in sizing.values()),
                     fell=[k for k, v in summaries.items() if v["fell"]], wall_s=time.time() - t0)
        history.append(entry)
        log(f"[design-loop] it {it}: mass {mass:.2f} kg (d={entry['mass_change']}) "
            f"feasible={entry['all_feasible']} fell={entry['fell']}")
        same = prev_sel is not None and all(
            sel[k][0] == prev_sel[k][0] and abs(sel[k][1] - prev_sel[k][1]) <= 5 for k in sel)
        converged = same and rel < tol
        prev_mass, prev_sel = mass, sel
        if converged:
            break
    out = dict(history=history, converged=bool(converged), iterations=len(history))
    save_derived("design_loop", out)
    return out
