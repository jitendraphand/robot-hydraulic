"""Physical build QA: a recursive acceptance log for printing, sub-assembly and bring-up.

`qa/build_log.yaml` is generated from the design (parts, cylinders, joints,
limbs) and filled in by the builder.  Each item has measured fields; empty
fields are PENDING.  An item passes only when its own fields pass AND every
child item it depends on passes:

    part -> cylinder -> joint -> limb -> robot

so, e.g., a limb cannot be signed off while one of its cylinders has not
passed its proof test.  Existing entries are never overwritten.
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml

from ..config import ROOT, RobotConfig
from .framework import Check, Node, Result

LOG = ROOT / "qa" / "build_log.yaml"


def criteria(cfg: RobotConfig) -> dict:
    proof = cfg.hyd["relief_pressure_bar"] * cfg.hyd["proof_factor"]
    sil = cfg["qa"]["sil"]
    return {
        "printed": ("bool", True),
        "dimensions_ok": ("bool", True),
        "inserts_ok": ("bool", True),
        "bearing_fit_ok": ("bool", True),
        "proof_test_bar": ("min", round(proof, 1)),
        "proof_hold_min": ("min", 5),
        "external_leak_drops": ("max", 0),
        "breakaway_bar": ("max", 1.5),
        "internal_leak_ml_min": ("max", 0.5),
        "encoder_zeroed": ("bool", True),
        "range_error_deg": ("max", 2.0),
        "bench_step_overshoot": ("max", sil["max_overshoot"]),
        "bench_ss_error_deg": ("max", sil["max_ss_err_deg"]),
        "fault_lock_ok": ("bool", True),
        "node_flashed": ("bool", True),
        "can_ok": ("bool", True),
        "hold_test_ok": ("bool", True),
        "estop_test_ok": ("bool", True),
        "heartbeat_loss_test_ok": ("bool", True),
        "hpu_pressure_bar": ("min", cfg.hyd["hpu"]["pressure_band_bar"][0]),
        "relief_set_bar": ("range", (cfg.hyd["relief_pressure_bar"] - 0.5, cfg.hyd["relief_pressure_bar"] + 0.5)),
        "gantry_stand_ok": ("bool", True),
        "gantry_walk_ok": ("bool", True),
    }


def template(cfg: RobotConfig) -> dict:
    from ..cad.build import cylinder_types
    plan = cfg.derived("print_plan")["parts"]
    sizing = cfg.derived("sizing")
    T: dict = {"calibration": {}, "parts": {}, "cylinders": {}, "joints": {}, "limbs": {}, "system": {}}
    for mat in sorted({e["material"] for e in plan}):
        T["calibration"][f"fit_coupon_{mat}"] = {"printed": None, "bearing_fit_ok": None, "inserts_ok": None,
                                                 "note": "record the hole size that fits; update joint_hardware.bearing_fit_d"}
    for e in plan:
        if e["group"] in ("calibration",):
            continue
        for f in e["files"]:
            nm = Path(f).stem
            for k in range(e["qty_each"]):
                key = nm if e["qty_each"] == 1 else f"{nm}#{k + 1}"
                fields = {"printed": None, "dimensions_ok": None}
                if e["body"] or e["group"] == "hydraulic":
                    fields["inserts_ok"] = None
                if e["body"]:
                    fields["bearing_fit_ok"] = None
                T["parts"][key] = fields
    types = cylinder_types(cfg, sizing)
    counters = {}
    for j in cfg.hydraulic_joints:
        key = next(k for k, (c, js) in types.items() if j.base in js)
        counters[key] = counters.get(key, 0) + 1
        n = counters[key]
        T["cylinders"][f"cyl_{j.name}"] = {
            "parts": [f"cyl_{key}_blind_cap#{n}", f"cyl_{key}_rod_cap#{n}", f"cyl_{key}_piston#{n}"],
            "proof_test_bar": None, "proof_hold_min": None, "external_leak_drops": None,
            "breakaway_bar": None, "internal_leak_ml_min": None}
    for j in cfg.joints.values():
        child = cfg.body_of_joint(j.name)
        parts = [p for p in (child.parent, child.name) if p in T["parts"] or p.replace("R_", "L_") in T["parts"]]
        e = {"parts": parts, "range_error_deg": None}
        if j.actuator_type == "hydraulic":
            e.update(cylinder=f"cyl_{j.name}", encoder_zeroed=None, bench_step_overshoot=None,
                     bench_ss_error_deg=None, fault_lock_ok=None)
        T["joints"][j.name] = e
    for n in cfg["electrical"]["nodes"]:
        T["limbs"][n["name"]] = {"joints": n["joints"], "node_flashed": None, "can_ok": None, "hold_test_ok": None}
    T["system"]["HX-1"] = {"limbs": list(T["limbs"]), "estop_test_ok": None, "heartbeat_loss_test_ok": None,
                           "hpu_pressure_bar": None, "relief_set_bar": None, "gantry_stand_ok": None,
                           "gantry_walk_ok": None}
    return T


def ensure_log(cfg: RobotConfig) -> dict:
    T = template(cfg)
    cur = yaml.safe_load(LOG.read_text()) if LOG.exists() else {}
    cur = cur or {}
    for sec, items in T.items():
        cur.setdefault(sec, {})
        for k, v in items.items():
            if k not in cur[sec]:
                cur[sec][k] = v
            else:
                for f, val in v.items():
                    cur[sec][k].setdefault(f, val)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    head = ("# HX-1 physical build QA log (recursive acceptance).\n"
            "# Fill measured values as you build; leave null for 'not done yet'.\n"
            "# `python -m hx qa` evaluates every item against criteria derived from robot.yaml.\n"
            "# Roll-up: part -> cylinder -> joint -> limb -> system. See docs/08_quality_assurance.md.\n")
    LOG.write_text(head + yaml.safe_dump(cur, sort_keys=False, width=120))
    return cur


def _eval_fields(item: dict, crit: dict) -> tuple[str, list]:
    st, msgs = "PASS", []
    for f, val in item.items():
        if f not in crit:
            continue
        kind, lim = crit[f]
        if val is None:
            st = "PENDING" if st == "PASS" else st
            continue
        good = (kind == "bool" and val is True) or (kind == "min" and float(val) >= lim) or \
               (kind == "max" and float(val) <= lim) or (kind == "range" and lim[0] <= float(val) <= lim[1])
        if not good:
            st = "FAIL"
            msgs.append(f"{f}={val} (need {kind} {lim})")
    return st, msgs


def evaluate(cfg: RobotConfig) -> Node:
    log = ensure_log(cfg)
    crit = criteria(cfg)
    memo: dict = {}

    def status(sec: str, key: str) -> tuple[str, str]:
        if (sec, key) in memo:
            return memo[(sec, key)]
        item = log.get(sec, {}).get(key)
        if item is None:
            if sec == "parts":    # mirrored parts share one entry name pattern
                alt = [k for k in log["parts"] if k.split("#")[0] == key.split("#")[0]]
                if alt:
                    sts = [status("parts", a)[0] for a in alt]
                    r = ("FAIL" if "FAIL" in sts else "PENDING" if "PENDING" in sts else "PASS", "")
                    memo[(sec, key)] = r
                    return r
            memo[(sec, key)] = ("PENDING", "not in log")
            return memo[(sec, key)]
        st, msgs = _eval_fields(item, crit)
        deps = []
        for p in item.get("parts", []):
            deps.append(status("parts", p))
        if "cylinder" in item:
            deps.append(status("cylinders", item["cylinder"]))
        for j in item.get("joints", []):
            deps.append(status("joints", j))
        for lb in item.get("limbs", []):
            deps.append(status("limbs", lb))
        dsts = [d[0] for d in deps]
        if "FAIL" in dsts:
            st = "FAIL"
            msgs.append("a dependency failed")
        elif "PENDING" in dsts and st == "PASS":
            st = "PENDING"
        memo[(sec, key)] = (st, "; ".join(msgs))
        return memo[(sec, key)]

    root = Node("Physical build (qa/build_log.yaml)")
    for sec, title in (("calibration", "Calibration coupons"), ("parts", "Printed parts"), ("cylinders", "Cylinders (proof-tested)"),
                       ("joints", "Joints"), ("limbs", "Limbs / nodes"), ("system", "System bring-up")):
        n = Node(title)
        counts = {"PASS": 0, "PENDING": 0, "FAIL": 0}
        fails = []
        for key in log.get(sec, {}):
            s, m = status(sec, key)
            counts[s] += 1
            if s == "FAIL":
                fails.append(f"{key}: {m}")
        st = "FAIL" if counts["FAIL"] else ("PENDING" if counts["PENDING"] else "PASS")
        detail = f"{counts['PASS']} pass, {counts['PENDING']} pending, {counts['FAIL']} fail" + \
                 (f" — {fails[:3]}" if fails else "")
        n.add(Check(f"BLD-{sec[:4].upper()}", title, (lambda st=st, detail=detail: (lambda ctx: Result(st, detail)))(), []))
        root.add(n)
    return root
