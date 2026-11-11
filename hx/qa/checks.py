"""The HX-1 QA tree: system -> subsystems -> components."""
from __future__ import annotations

import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import yaml

from ..config import DERIVED, ROOT, RobotConfig, config_hash
from .framework import Check, Node, Result, check_bool, fail, ok, warn

FW = ROOT / "firmware"


def _d(ctx, name):
    return ctx["cfg"].derived(name) or {}


# ------------------------------------------------------------------ config
def c_schema(ctx):
    import jsonschema
    schema = json.loads((ROOT / "config" / "robot.schema.json").read_text())
    try:
        jsonschema.validate(ctx["cfg"].raw, schema)
    except jsonschema.ValidationError as e:
        return fail(f"{'/'.join(map(str, e.path))}: {e.message}")
    return ok("robot.yaml validates against robot.schema.json")


def c_kinematics(ctx):
    cfg = ctx["cfg"]
    nh = len(cfg.hydraulic_joints)
    legs = [j for j in cfg.joints.values() if any(k in j.base for k in ("hip", "knee", "ankle"))]
    arms = [j for j in cfg.joints.values() if any(k in j.base for k in ("shoulder", "elbow"))]
    bad = [j.name for j in cfg.joints.values() if not j.range_deg[0] < j.range_deg[1]]
    g = cfg["geometry"]
    consistency = abs(cfg.bodies["L_thigh"].origin[2] + g["hip_pitch_offset"]) < 1e-9 and \
        abs(cfg.bodies["L_ankle_link"].origin[2] + g["shin_length"]) < 1e-9 and \
        abs(cfg.bodies["L_foot"].origin[2] + g["ankle_roll_offset"]) < 1e-9 and \
        abs(cfg.bodies["L_hip_yaw_link"].origin[2] + g["hip_drop"]) < 1e-9
    good = len(legs) >= 12 and len(arms) >= 4 and not bad and consistency
    return check_bool(good, f"{len(legs)} leg DOF, {len(arms)} arm DOF, {nh} hydraulic; body origins match geometry",
                      f"legs {len(legs)}, arms {len(arms)}, bad ranges {bad}, geometry/origin consistent={consistency}")


def c_fresh(ctx):
    man = json.loads((DERIVED / "manifest.json").read_text()) if (DERIVED / "manifest.json").exists() else {}
    h = config_hash()
    need = ["demand", "sim_summary", "sizing", "hpu", "mass_props", "print_plan", "joint_params", "electronics",
            "design_loop", "sim_traces"]
    stale = [k for k in need if man.get(k) != h]
    return check_bool(not stale, f"all {len(need)} derived artefacts generated from robot.yaml {h}",
                      f"stale vs robot.yaml {h}: {stale} -> run `make loop` / `make all`")


def c_loop(ctx):
    dl = _d(ctx, "design_loop")
    if not dl:
        return fail("design loop has not been run")
    last = dl["history"][-1]
    return check_bool(dl["converged"] and last["all_feasible"] and not last["fell"],
                      f"converged in {dl['iterations']} iterations, mass {last['total_mass_kg']:.2f} kg",
                      f"converged={dl['converged']} feasible={last['all_feasible']} fell={last['fell']}",
                      iterations=dl["iterations"], mass=last["total_mass_kg"])


def c_mass(ctx):
    from ..sim.model import total_mass
    m = total_mass(ctx["cfg"])
    lim = ctx["cfg"]["meta"]["mass_max_kg"]
    return check_bool(m <= lim, f"{m:.2f} kg <= {lim} kg", f"{m:.2f} kg > {lim} kg", mass_kg=m)


# --------------------------------------------------------------------- CAD
def _plan(ctx):
    return _d(ctx, "print_plan").get("parts", [])


def c_bed(ctx):
    plan = _plan(ctx)
    bad = [e["name"] for e in plan if not e["fits"]]
    big = max(plan, key=lambda e: max(e["bed_dims_mm"]))
    return check_bool(plan and not bad, f"{len(plan)} part designs fit; largest {big['name']} {big['bed_dims_mm']} mm",
                      f"do not fit: {bad}")


def c_watertight(ctx):
    import trimesh
    plan = _plan(ctx)
    files = [f for e in plan for f in e["files"]]
    bad = []
    for f in files:
        p = ROOT / f
        if not p.exists():
            bad.append(f"{f} missing")
            continue
        m = trimesh.load(p, force="mesh")
        if not m.is_watertight or m.volume <= 0:
            bad.append(f)
    return check_bool(not bad, f"{len(files)} STL files exist, watertight, positive volume", f"bad: {bad[:6]}")


def c_materials(ctx):
    cfg = ctx["cfg"]
    pr = cfg["printer"]
    bad = [k for k, m in cfg["materials"].items() if m["nozzle_c"] > pr["max_nozzle_c"] or m["bed_c"] > pr["max_bed_c"]]
    cf = sorted({e["material"] for e in _plan(ctx) if e["hardened_nozzle"]})
    if bad:
        return fail(f"materials exceed printer limits: {bad}")
    return warn(f"temperatures within N4 Max limits; {cf} need a hardened-steel nozzle") if cf else ok("within limits")


def c_interference(ctx):
    from ..cad.assembly import run_interference
    tol = ctx["cfg"]["qa"]["cad"]["interference_tol_mm3"]
    res = run_interference(ctx["cfg"], log=lambda *_: None, tol_mm3=tol)
    ctx["asm"] = res
    bad = {k: v["hits"][:2] for k, v in res.items() if v["hits"]}
    return check_bool(not bad, f"{len(res)} poses, 0 interferences (> {tol} mm3)",
                      f"{len(bad)} poses with interference, e.g. {list(bad.items())[:2]}", poses=len(res))


def c_cyl_fit(ctx):
    res = ctx.get("asm")
    if res is None:
        return fail("interference check did not run")
    bad = {k: v["cyl_fit_fail"] for k, v in res.items() if v["cyl_fit_fail"]}
    return check_bool(not bad, f"all cylinder lengths within stroke in {len(res)} poses", f"out of stroke: {bad}")


def c_lugs(ctx):
    cfg = ctx["cfg"]
    sizing = _d(ctx, "sizing")
    jh = cfg["joint_hardware"]
    sfmin = cfg["qa"]["cad"]["min_lug_bearing_sf"]
    worst = (math.inf, "")
    for base, info in sizing.items():
        a = math.pi / 4 * (info["cylinder"]["bore_mm"] * 1e-3) ** 2
        F = cfg.pa("relief_pressure_bar") * a
        stress = F / (8e-3 * jh["lug_t"] * 1e-3 * 2) / 1e6      # 2 lugs, M8 pin
        sf = cfg["materials"]["PA12-CF"]["tensile_mpa"] / stress
        worst = min(worst, (sf, base))
    return check_bool(worst[0] >= sfmin, f"min lug bearing SF {worst[0]:.1f} ({worst[1]}) >= {sfmin}",
                      f"lug bearing SF {worst[0]:.1f} ({worst[1]}) < {sfmin}", sf=worst[0])


def c_outputs(ctx):
    need = [ROOT / "cad" / "print_plan.md", ROOT / "cad" / "bom.csv"]
    plates = _d(ctx, "print_plan").get("plates", [])
    missing = [str(p) for p in need if not p.exists()] + [p["file"] for p in plates if not (ROOT / p["file"]).exists()]
    bom = (ROOT / "cad" / "bom.csv").read_text() if need[1].exists() else ""
    return check_bool(not missing and "cut" in bom, f"print plan, {len(plates)} plates, BOM with cylinder cut list",
                      f"missing: {missing}")


# --------------------------------------------------------------- hydraulics
def c_torque(ctx):
    s = _d(ctx, "sizing")
    bad = {k: v["ratio_demand"] for k, v in s.items() if not v["feasible"] or (v["ratio_demand"] or 0) < 1.0}
    worst = min(s.items(), key=lambda kv: kv[1]["ratio_demand"] or 9)
    return check_bool(s and not bad, f"all {len(s)} joint types >= SF x demand; tightest {worst[0]} "
                                     f"{worst[1]['ratio_demand']:.2f}", f"insufficient: {bad}")


def c_range(ctx):
    s = _d(ctx, "sizing")
    mn = ctx["cfg"].hyd["sizing"]["min_moment_arm_mm"]
    bad = {k: (round(v["ratio_floor"] or 0, 2), round(v["min_moment_arm_mm"], 1), v["moment_sign_flip"])
           for k, v in s.items() if (v["ratio_floor"] or 0) < 1 or v["min_moment_arm_mm"] < mn - 0.05 or v["moment_sign_flip"]}
    return check_bool(not bad, f"full-range floor and moment arm >= {mn} mm for all joints", f"violations: {bad}")


def c_valve_flow(ctx):
    cfg = ctx["cfg"]
    v = cfg.hyd["valve"]
    dp = (cfg.pa("supply_pressure_bar") - cfg.pa("return_pressure_bar")) / 1e5
    qmax = v["rated_flow_lpm"] * math.sqrt(dp / v["rated_dp_bar"])
    margin = cfg.hyd["sizing"]["valve_flow_margin"]
    s = _d(ctx, "sizing")
    worst = max(s.items(), key=lambda kv: kv[1]["peak_flow_lpm"])
    need = worst[1]["peak_flow_lpm"] * margin
    return check_bool(qmax >= need, f"valve {qmax:.2f} L/min >= {margin} x peak {worst[1]['peak_flow_lpm']:.2f} L/min ({worst[0]})",
                      f"valve {qmax:.2f} L/min < {margin} x peak {worst[1]['peak_flow_lpm']:.2f} L/min ({worst[0]})")


def c_hpu(ctx):
    cfg = ctx["cfg"]
    h = _d(ctx, "hpu")
    lo = cfg.hyd["hpu"]["pressure_band_bar"][0] - 1.5 + cfg["qa"]["hpu"]["min_supply_margin_bar"]
    sc = {k: v for k, v in h.items() if isinstance(v, dict)}
    worst = min(sc.items(), key=lambda kv: kv[1]["min_supply_bar"])
    duty = max(v["pump_duty"] for v in sc.values())
    return check_bool(worst[1]["min_supply_bar"] >= lo and duty < 0.9,
                      f"min supply {worst[1]['min_supply_bar']:.1f} bar ({worst[0]}) >= {lo:.1f}; max pump duty {duty:.2f}; "
                      f"pump {h['pump_flow_lpm']:.2f} L/min, accumulator {h['accumulator_usable_l'] * 1000:.0f} mL usable",
                      f"supply dips to {worst[1]['min_supply_bar']:.1f} bar in {worst[0]} (< {lo:.1f}) or duty {duty:.2f}")


def c_ratings(ctx):
    h = ctx["cfg"].hyd
    relief = h["relief_pressure_bar"]
    proof = relief * h["proof_factor"]
    trip = ctx["cfg"]["control"]["pressure_trip_bar"]
    errs = []
    if h["valve"]["rated_pressure_bar"] < proof:
        errs.append(f"valve {h['valve']['rated_pressure_bar']} < proof {proof}")
    if h["hose"]["rated_bar"] < proof:
        errs.append(f"hose {h['hose']['rated_bar']} < proof {proof}")
    if h["sensor"]["range_bar"] < trip:
        errs.append("sensor range < trip")
    if not (relief < trip < h["sensor"]["range_bar"]):
        errs.append("need relief < trip < sensor range")
    if not (h["supply_pressure_bar"] < relief):
        errs.append("supply >= relief")
    return check_bool(not errs, f"supply {h['supply_pressure_bar']} < relief {relief} < trip {trip} < sensor {h['sensor']['range_bar']}; "
                                f"proof {proof:.1f} bar <= valve/hose ratings", "; ".join(errs))


def c_proof_doc(ctx):
    t = (ROOT / "docs" / "08_quality_assurance.md").read_text() if (ROOT / "docs" / "08_quality_assurance.md").exists() else ""
    return check_bool("proof" in t.lower() and "cylinder" in t.lower(), "proof-test procedure documented",
                      "docs/08_quality_assurance.md lacks proof-test procedure")


# --------------------------------------------------------------- simulation
def _sim(ctx):
    return _d(ctx, "sim_summary")


def c_scenario(name):
    def f(ctx):
        q = ctx["cfg"]["qa"]["sim"]
        s = _sim(ctx).get(name)
        if not s:
            return fail(f"scenario {name} missing")
        hyd = {j.name for j in ctx["cfg"].hydraulic_joints}
        err = max(v for k, v in s["max_tracking_err_deg"].items() if k in hyd)
        errs = []
        if s["fell"]:
            errs.append("fell")
        if s["max_tilt_deg"] > q["max_tilt_deg"]:
            errs.append(f"tilt {s['max_tilt_deg']:.1f}")
        if err > q["max_tracking_err_deg"]:
            errs.append(f"tracking {err:.1f} deg")
        if s["ik_residual"] > 0.01:
            errs.append(f"IK residual {s['ik_residual']:.3f}")
        if s.get("final_com_error_m") is not None and s["final_com_error_m"] > q["max_final_com_error_m"]:
            errs.append(f"CoM drift {s['final_com_error_m']:.3f} m")
        return check_bool(not errs, f"stable, tilt {s['max_tilt_deg']:.1f} deg, max hydraulic tracking {err:.2f} deg, "
                                    f"mass {s['mass_kg']:.1f} kg", ", ".join(errs))
    return f


def c_servo(ctx):
    cfg = ctx["cfg"]
    dem = _d(ctx, "demand")
    lim = cfg["servo_sizing"]["torque_utilisation_max"]
    worst = (0.0, "")
    for j in cfg.servo_joints:
        if j.name in dem:
            u = dem[j.name]["peak_tau"] / cfg["servo_models"][j.servo_model]["stall_nm"]
            worst = max(worst, (u, j.name))
    return check_bool(worst[0] <= lim, f"max servo utilisation {worst[0] * 100:.0f} % ({worst[1]}) <= {lim * 100:.0f} %",
                      f"{worst[1]} uses {worst[0] * 100:.0f} % of stall")


def c_ranges_used(ctx):
    cfg = ctx["cfg"]
    dem = _d(ctx, "demand")
    m = math.radians(cfg["control"]["soft_limit_margin_deg"])
    close = []
    for n, d in dem.items():
        lo, hi = cfg.joints[n].range_rad
        if d["q_range"][0] < lo + m - 1e-3 or d["q_range"][1] > hi - m + 1e-3:
            close.append(f"{n} [{math.degrees(d['q_range'][0]):.0f},{math.degrees(d['q_range'][1]):.0f}]")
    return ok("simulated motion stays inside soft limits") if not close else \
        warn(f"motion reaches soft-limit band: {close[:4]}")


# ---------------------------------------------------------------- firmware
def c_fw_tests(ctx):
    b = FW / "build"
    r1 = subprocess.run(["cmake", "-S", str(FW), "-B", str(b), "-DCMAKE_BUILD_TYPE=Release"], capture_output=True, text=True)
    r2 = subprocess.run(["cmake", "--build", str(b), "-j4"], capture_output=True, text=True)
    if r1.returncode or r2.returncode:
        return fail("build failed: " + (r2.stderr or r1.stderr)[-400:])
    warns = r2.stdout.count("warning:") + r2.stderr.count("warning:")
    r3 = subprocess.run([str(b / "hx_tests")], capture_output=True, text=True)
    last = r3.stdout.strip().splitlines()[-1] if r3.stdout.strip() else ""
    return check_bool(r3.returncode == 0 and warns == 0, f"-Werror build, {last}", f"tests rc={r3.returncode}: {last}; warnings {warns}")


def c_fw_platform(ctx):
    plat = FW / "platform" / "esp32s3"
    nodes = ctx["cfg"]["electrical"]["nodes"]
    bad = []
    for n in nodes:
        flags = [f"-DHX_NODE_ID={n['id']}"] + (["-DHX_HUB=1"] if n["kind"] == "hub" else [])
        r = subprocess.run(["g++", "-std=c++17", "-fsyntax-only", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                            *flags, "-I", str(plat / "host_stubs"), "-I", str(plat / "include"), "-I", str(FW / "core" / "include"),
                            str(plat / "src" / "main.cpp"), str(plat / "src" / "hal_esp32.cpp")], capture_output=True, text=True)
        if r.returncode:
            bad.append(f"{n['name']}: {r.stderr[-300:]}")
    if bad:
        return fail("; ".join(bad))
    return Result("PASS", f"{len(nodes)} node builds compile against the Arduino-ESP32 API (host check; "
                          "full ESP32 toolchain build runs in CI: `pio run`)")


def c_fw_generated(ctx):
    from ..codegen import all_joint_params, robot_config_h
    from .. import protocol
    from ..electronics.netlist import EDIR, pins_h
    B = yaml.safe_load(open(EDIR / "boards.yaml"))
    want = {FW / "core" / "include" / "hx" / "robot_config.h": robot_config_h(ctx["cfg"], all_joint_params(ctx["cfg"])),
            FW / "core" / "include" / "hx" / "can_protocol.h": protocol.c_header(),
            FW / "platform" / "esp32s3" / "include" / "pins.h": pins_h(B)}
    stale = [p.name for p, t in want.items() if not p.exists() or p.read_text() != t]
    return check_bool(not stale, "robot_config.h, can_protocol.h, pins.h match the configuration",
                      f"stale generated headers: {stale} -> run `python -m hx gen`")


def _sil(ctx):
    if "sil" not in ctx:
        man = json.loads((DERIVED / "manifest.json").read_text()) if (DERIVED / "manifest.json").exists() else {}
        cached = ctx["cfg"].derived("sil")
        if ctx.get("fast") and cached and man.get("sil") == config_hash():
            ctx["sil"] = cached
        else:
            from ..config import save_derived
            from ..sim.sil import run_all
            ctx["sil"] = run_all(ctx["cfg"], log=lambda *_: None)
            save_derived("sil", ctx["sil"])
    return ctx["sil"]


def c_sil_joint(base):
    def f(ctx):
        r = _sil(ctx).get(base)
        if not r:
            return fail("no SIL result")
        s, w, q = r["step"], r["walk"], r["squat"]
        return check_bool(r["pass"], f"step os {s['overshoot'] * 100:.1f} % ts {s['settle_5pct_s']:.2f} s ss {s['ss_err_deg']:.2f} deg; "
                                     f"walk rms {w['rms_err_deg']:.2f} max {w['max_err_deg']:.2f} deg; squat max {q['max_err_deg']:.2f} deg; "
                                     f"pmax {max(w['peak_pressure_bar'], q['peak_pressure_bar']):.1f} bar",
                          "criteria not met: " + json.dumps({k: (r[k] if k in ('step', 'walk', 'squat') else None) for k in ('step', 'walk', 'squat')}, default=float)[:300])
    return f


def c_sil_faults(ctx):
    res = _sil(ctx)
    lim = ctx["cfg"]["qa"]["sil"]["max_fault_detect_s"]
    bad = []
    for base, r in res.items():
        for k in ("encoder", "pressure", "supply_loss"):
            fr = r[f"fault_{k}"]
            if not fr["faults"] or fr["fault_time"] is None or fr["fault_time"] - 0.3 > lim:
                bad.append(f"{base}:{k}")
    return check_bool(not bad, f"{3 * len(res)} injected faults detected within {lim * 1000:.0f} ms, valve locked",
                      f"undetected/slow: {bad}")


def c_brain(ctx):
    from ..brain.runtime import Brain, NodeView, VirtualBus
    cfg = ctx["cfg"]
    nodes = {n["name"]: NodeView(n["id"], n["name"], "", n["joints"]) for n in cfg["electrical"]["nodes"]}
    bus = VirtualBus(nodes)
    br = Brain(cfg, bus)
    armed = br.arm(timeout=3.0, log=lambda *_: None)
    # negative test: a node that never leaves IDLE must block arming
    bus2 = VirtualBus(nodes)
    bus2.state[1] = 5          # leg_L in FAULT
    br2 = Brain(cfg, bus2)
    refused = not br2.arm(timeout=0.5, log=lambda *_: None)
    return check_bool(armed and refused, "arms with healthy nodes and pressure; refuses with a faulted node",
                      f"armed={armed} refused_on_fault={refused}")


# -------------------------------------------------------------- electronics
def _elec(ctx):
    return _d(ctx, "electronics")


def c_erc(ctx):
    e = _elec(ctx)
    errs = [i for i in e.get("erc", []) if i["severity"] == "error"]
    warns = [i for i in e.get("erc", []) if i["severity"] == "warning"]
    cnt = ", ".join(f"{k}: {v['parts']} parts / {v['nets']} nets" for k, v in e.get("counts", {}).items())
    if errs:
        return fail(f"{len(errs)} ERC errors: {errs[0]['msg']}")
    return (warn if warns else ok)(f"0 errors, {len(warns)} warnings ({cnt})")


def c_power(ctx):
    p = _elec(ctx)["power"]
    worst = min((v["margin"], k) for k, v in p["rails"].items() if "margin" in v)
    return check_bool(not p["issues"], f"min regulator margin {worst[0]:.2f}x ({worst[1]}); battery peak {p['battery_peak_a']:.0f} A "
                                       f"of {p['battery_max_a']:.0f} A; runtime ~{p['runtime_min']:.0f} min", "; ".join(p["issues"]))


def c_harness(ctx):
    h = _elec(ctx)["harness"]
    return check_bool(not h, "all wires fused within ampacity and <= 80 % loaded", "; ".join(h))


def c_can(ctx):
    c = _elec(ctx)["can"]
    worst = max(c.items(), key=lambda kv: kv[1]["load"])
    return check_bool(worst[1]["load"] <= 0.6, f"max bus load {worst[1]['load'] * 100:.0f} % ({worst[0]})",
                      f"{worst[0]} load {worst[1]['load'] * 100:.0f} % > 60 %")


def c_estop_arch(ctx):
    r = ctx["cfg"]["electrical"]["rails"]
    ok_ = r["PUMP_BAT"]["estop_switched"] and r["SERVO_7V4"]["estop_switched"] and not r["VALVE_6V"]["estop_switched"] \
        and not r["LOGIC_5V"]["estop_switched"]
    bom = (ROOT / "cad" / "bom.csv").read_text() if (ROOT / "cad" / "bom.csv").exists() else ""
    dump = "Normally-open" in bom
    return check_bool(ok_ and dump, "pump+servo power e-stop switched; logic+valve servos always on; NO dump valve in BOM",
                      f"rails ok={ok_}, NO dump valve in BOM={dump}")


# ------------------------------------------------------------------- docs
DOCS = ["README.md", "docs/01_architecture.md", "docs/02_hydraulics.md", "docs/03_printing.md", "docs/04_assembly.md",
        "docs/05_electronics.md", "docs/06_firmware.md", "docs/07_simulation.md", "docs/08_quality_assurance.md",
        "docs/09_safety.md"]


def c_docs(ctx):
    missing = [d for d in DOCS if not (ROOT / d).exists()]
    return check_bool(not missing, f"{len(DOCS)} guides present", f"missing: {missing}")


def c_safety_doc(ctx):
    p = ROOT / "docs" / "09_safety.md"
    t = p.read_text().lower() if p.exists() else ""
    need = ["injection", "e-stop", "proof", "ppe", "gantry"]
    miss = [k for k in need if k not in t]
    return check_bool(not miss, "hazards, PPE, proof testing, e-stop, gantry bring-up covered", f"missing topics: {miss}")


def c_trace(ctx):
    reqs = yaml.safe_load(open(ROOT / "config" / "requirements.yaml"))
    ids = {k for grp in reqs.values() for k in grp}
    covered = set()
    unknown = set()
    for c in ctx["root"].all_checks():
        for r in c.reqs:
            (covered if r in ids else unknown).add(r)
    missing = sorted(ids - covered)
    return check_bool(not missing and not unknown, f"{len(ids)} requirements, all verified by >= 1 check",
                      f"unverified: {missing}; unknown ids: {sorted(unknown)}")


# ------------------------------------------------------------------- build
def build_tree(cfg: RobotConfig) -> Node:
    C = Check
    root = Node(f"{cfg['meta']['name']} system")
    conf = Node("Configuration & design loop").add(
        C("CFG-1", "robot.yaml schema", c_schema, ["SYS-006"]),
        C("CFG-2", "kinematic topology & geometry consistency", c_kinematics, ["SYS-001"]),
        C("CFG-3", "derived artefacts up to date", c_fresh, ["SYS-006"]),
        C("CFG-4", "design loop converged", c_loop, ["SYS-007"]),
        C("CFG-5", "total mass budget", c_mass, ["SYS-002"]))
    cad = Node("Mechanical (CAD / printing)").add(
        Node("Printability").add(
            C("CAD-1", "parts fit Neptune 4 Max", c_bed, ["MEC-001"]),
            C("CAD-2", "meshes watertight", c_watertight, ["MEC-002"]),
            C("CAD-3", "materials vs printer", c_materials, ["MEC-005"]),
            C("CAD-4", "print plan, plates, BOM", c_outputs, ["MEC-007"])),
        Node("Assembly").add(
            C("CAD-5", "assembly interference (all poses)", c_interference, ["MEC-003"]),
            C("CAD-6", "cylinder strokes in all poses", c_cyl_fit, ["MEC-004"]),
            C("CAD-7", "printed lug bearing stress", c_lugs, ["MEC-006"])))
    hyd = Node("Hydraulics").add(
        C("HYD-1", "torque capacity vs simulated demand", c_torque, ["HYD-001"]),
        C("HYD-2", "full-range capacity & moment arm", c_range, ["HYD-002"]),
        C("HYD-3", "valve flow capacity", c_valve_flow, ["HYD-003"]),
        C("HYD-4", "HPU pump + accumulator", c_hpu, ["HYD-004"]),
        C("HYD-5", "pressure ratings chain", c_ratings, ["HYD-005"]),
        C("HYD-6", "proof-test procedure", c_proof_doc, ["HYD-006"]))
    sim = Node("Whole-body simulation").add(
        C("SIM-1", "stand", c_scenario("stand"), ["SYS-003"]),
        C("SIM-2", "walk", c_scenario("walk"), ["SYS-003"]),
        C("SIM-3", "squat", c_scenario("squat"), ["SYS-004"]),
        C("SIM-4", "single-leg balance", c_scenario("single_leg"), ["SYS-004"]),
        C("SIM-5", "arm lift with payload", c_scenario("arm_lift"), ["SYS-005"]),
        C("SIM-6", "servo torque utilisation", c_servo, ["ELE-005"]),
        C("SIM-7", "motion inside soft limits", c_ranges_used, ["SAF-002"]))
    sil = Node("Hydraulic SIL (real firmware vs plant model)")
    for j in cfg.hydraulic_joints:
        if j.side == "L":
            sil.add(C(f"SIL-{j.base}", f"{j.base} step + walk/squat replay", c_sil_joint(j.base), ["CTL-004", "CTL-005"]))
    sil.add(C("SIL-F", "fault injection (encoder, pressure, supply)", c_sil_faults, ["CTL-006", "SAF-002"]))
    fw = Node("Firmware & control").add(
        C("FW-1", "host build (-Werror) + unit tests", c_fw_tests, ["CTL-001", "CTL-007", "SAF-002"]),
        C("FW-2", "node firmware compile check", c_fw_platform, ["CTL-002"]),
        C("FW-3", "generated headers current", c_fw_generated, ["CTL-003"]),
        C("FW-4", "brain arming interlocks", c_brain, ["CTL-008"]),
        sil)
    ele = Node("Electronics").add(
        C("ELE-1", "ERC (limb + hub boards)", c_erc, ["ELE-001"]),
        C("ELE-2", "power budget", c_power, ["ELE-002"]),
        C("ELE-3", "harness ampacity & fusing", c_harness, ["ELE-003"]),
        C("ELE-4", "CAN bus load", c_can, ["ELE-004"]),
        C("ELE-5", "e-stop power architecture", c_estop_arch, ["SAF-001"]))
    doc = Node("Documentation & safety").add(
        C("DOC-1", "guides present", c_docs, ["DOC-001"]),
        C("DOC-2", "safety guide content", c_safety_doc, ["SAF-003"]))
    root.add(conf, cad, hyd, sim, fw, ele, doc)
    root.add(C("TRACE", "requirements traceability", c_trace, []))
    return root
