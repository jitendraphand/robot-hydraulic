"""Hydraulic joint software-in-the-loop (SIL) tests.

The *real* firmware joint controller (firmware/core, compiled to
firmware/build/libhx_sil.so) runs at 1 kHz against a lumped-parameter plant:

  * micro-servo-driven spool: first-order lag + slew limit, closed-centre overlap
  * 4 metering edges: orifice equation Q = K a sqrt(dp), laminar leakage
  * two compressible chambers (effective bulk modulus incl. hose compliance)
  * piston on the joint linkage, seal friction (Coulomb + viscous)
  * joint inertia from the MuJoCo mass matrix; external torque reproduces the
    whole-body simulation demand:  tau_ext = J * qdd_ref - tau_demand
  * sensors: 12-bit pressure ADC with noise, 14-bit encoder, finite-difference qdot
  * supply: HPU pressure with line drop

Scenarios per hydraulic joint type: position step under load, impedance-mode
walking/squat trace replay with torque feed-forward, and fault injection
(sensor failure, supply loss).
"""
from __future__ import annotations

import ctypes as C
import math
import subprocess
from pathlib import Path

import numpy as np

from ..config import ROOT, RobotConfig
from ..hydraulics import Linkage, linkage_for

LIB = ROOT / "firmware" / "build" / "libhx_sil.so"


class Params(C.Structure):
    _fields_ = [("q_min", C.c_float), ("q_max", C.c_float), ("soft_margin", C.c_float), ("actuator", C.c_int32),
                ("axis", C.c_int32), ("anchor_parent", C.c_float * 3), ("anchor_child", C.c_float * 3),
                ("a_bore", C.c_float), ("a_annulus", C.c_float), ("min_moment_arm", C.c_float),
                ("p_supply", C.c_float), ("p_return", C.c_float), ("p_trip", C.c_float), ("p_supply_min", C.c_float),
                ("valve_q_rated", C.c_float), ("valve_dp_rated", C.c_float), ("valve_overlap", C.c_float),
                ("valve_deadband", C.c_float), ("valve_rate", C.c_float), ("kp", C.c_float), ("ki", C.c_float),
                ("kd", C.c_float), ("kp_pos_valve", C.c_float), ("kd_pos_valve", C.c_float),
                ("ki_pos_valve", C.c_float), ("kp_force", C.c_float),
                ("ki_force", C.c_float), ("k_vel_ff", C.c_float), ("tau_max", C.c_float),
                ("max_following_err", C.c_float), ("following_time", C.c_float), ("servo_center_us", C.c_float),
                ("servo_us_per_rad", C.c_float)]


class In(C.Structure):
    _fields_ = [("dt", C.c_float), ("mode", C.c_int32), ("q", C.c_float), ("qd", C.c_float), ("p_a", C.c_float),
                ("p_b", C.c_float), ("p_supply", C.c_float), ("q_valid", C.c_int32), ("p_valid", C.c_int32),
                ("q_ref", C.c_float), ("qd_ref", C.c_float), ("tau_ff", C.c_float), ("kp_scale", C.c_float),
                ("kd_scale", C.c_float)]


class Out(C.Structure):
    _fields_ = [("valve_cmd", C.c_float), ("spool", C.c_float), ("pulse_us", C.c_float), ("tau_cmd", C.c_float),
                ("force_cmd", C.c_float), ("force_meas", C.c_float), ("tau_meas", C.c_float),
                ("moment_arm", C.c_float), ("faults", C.c_uint32)]


FF_MODEL_ERROR = 0.85
MODE = {"OFF": 0, "HOLD": 1, "POSITION": 2, "IMPEDANCE": 3, "FORCE": 4}


def build_lib(log=print) -> Path:
    fw = ROOT / "firmware"
    subprocess.run(["cmake", "-S", str(fw), "-B", str(fw / "build"), "-DCMAKE_BUILD_TYPE=Release"],
                   check=True, capture_output=True)
    subprocess.run(["cmake", "--build", str(fw / "build"), "-j4"], check=True, capture_output=True)
    return LIB


def load_lib():
    if not LIB.exists():
        build_lib()
    lib = C.CDLL(str(LIB))
    lib.hx_joint_new.restype = C.c_void_p
    lib.hx_joint_new.argtypes = [C.POINTER(Params)]
    lib.hx_joint_step.argtypes = [C.c_void_p, C.POINTER(In), C.POINTER(Out)]
    lib.hx_joint_free.argtypes = [C.c_void_p]
    lib.hx_sizeof_params.restype = C.c_uint32
    lib.hx_sizeof_in.restype = C.c_uint32
    lib.hx_sizeof_out.restype = C.c_uint32
    assert lib.hx_sizeof_params() == C.sizeof(Params), "Params layout mismatch C<->Python"
    assert lib.hx_sizeof_in() == C.sizeof(In), "In layout mismatch"
    assert lib.hx_sizeof_out() == C.sizeof(Out), "Out layout mismatch"
    return lib


def make_params(d: dict) -> Params:
    p = Params()
    for k, _ in Params._fields_:
        v = d[k]
        if isinstance(v, list):
            getattr(p, k)[:] = v
        else:
            setattr(p, k, v)
    return p


class Plant:
    """Lumped hydraulic joint plant (state integrated at dt_sub)."""

    def __init__(self, cfg: RobotConfig, jname: str, J: float, q0: float, seed: int = 1):
        sizing = cfg.derived("sizing")
        self.lk, self.cyl = linkage_for(cfg, jname, sizing)
        h = cfg.hyd
        j = cfg.joints[jname]
        self.q_lo, self.q_hi = j.range_rad
        self.beta = h["fluid"]["effective_bulk_modulus_pa"]
        self.ps = cfg.pa("supply_pressure_bar") - 0.5e5     # mid-band supply
        self.pt = cfg.pa("return_pressure_bar")
        v = h["valve"]
        self.K = (v["rated_flow_lpm"] / 60000.0) / math.sqrt(v["rated_dp_bar"] * 1e5 / 2)
        self.ov = v["overlap"]
        self.cl = (v["leakage_lpm"] / 60000.0) / self.ps
        sv = v["servo"]
        self.tau_s = sv["time_const_ms"] / 1000
        self.rate_s = 2.0 / (sv["rate_full_travel_ms"] / 1000)
        self.span = sv["pulse_span_us"]
        from ..codegen import hose_volume
        self.V0 = hose_volume(cfg)
        self.J = J
        self.b = 0.15                    # joint viscous (bearings, seals) N m s/rad
        self.Fc = 25.0                   # seal Coulomb friction (N)
        self.cv = 300.0                  # seal viscous (N s/m)
        self.q, self.qd = q0, 0.0
        self.pa = self.pb = 0.5 * self.ps
        self.spool = 0.0
        self.rng = np.random.default_rng(seed)
        self.p_lsb = 30e5 / 4095 / 0.6667 * 3.3 / 5.0
        self.q_lsb = 2 * math.pi / 16384
        self.q_prev_meas = q0
        self.supply_ok = True

    @staticmethod
    def _ssqrt(x):
        a = abs(x)
        return math.copysign(math.sqrt(a), x) if a > 2e3 else x / math.sqrt(2e3)

    def step(self, pulse_us: float, tau_ext: float, dt: float, n_sub: int = 10):
        h = dt / n_sub
        ps = self.ps if self.supply_ok else 0.8e5
        s_cmd = 0.0 if pulse_us == 0 else (pulse_us - 1500.0) / self.span
        s_cmd = max(-1.0, min(1.0, s_cmd))
        Lmin = self.cyl.l_retracted
        stroke = self.cyl.stroke_mm / 1000
        for _ in range(n_sub):
            # servo-positioned spool
            ds = (s_cmd - self.spool) / self.tau_s
            ds = max(-self.rate_s, min(self.rate_s, ds))
            self.spool += ds * h
            a = max(0.0, abs(self.spool) - self.ov) / (1 - self.ov)
            pa, pb = self.pa, self.pb
            if self.spool >= 0:
                qa = self.K * a * self._ssqrt(ps - pa)
                qb = -self.K * a * self._ssqrt(pb - self.pt)
            else:
                qa = -self.K * a * self._ssqrt(pa - self.pt)
                qb = self.K * a * self._ssqrt(ps - pb)
            qa += self.cl * ((ps - pa) - (pa - self.pt))
            qb += self.cl * ((ps - pb) - (pb - self.pt))
            m = float(self.lk.moment_arm(self.q)[0])
            L = float(self.lk.length(self.q)[0])
            xp = min(max(L - Lmin, 0.0), stroke)
            xv = m * self.qd
            VA = self.V0 + self.cyl.a_bore * xp
            VB = self.V0 + self.cyl.a_annulus * (stroke - xp)
            self.pa = max(0.0, pa + self.beta / VA * (qa - self.cyl.a_bore * xv) * h)
            self.pb = max(0.0, pb + self.beta / VB * (qb + self.cyl.a_annulus * xv) * h)
            F = self.pa * self.cyl.a_bore - self.pb * self.cyl.a_annulus
            F -= self.cv * xv + self.Fc * math.tanh(xv / 0.002)
            tau = F * m - self.b * self.qd + tau_ext
            # hard stops
            if self.q > self.q_hi:
                tau -= 5e3 * (self.q - self.q_hi) + 50 * self.qd
            if self.q < self.q_lo:
                tau -= 5e3 * (self.q - self.q_lo) + 50 * self.qd
            self.qd += tau / self.J * h
            self.q += self.qd * h

    def sense(self, dt):
        qm = round((self.q + self.rng.normal(0, 0.5 * self.q_lsb)) / self.q_lsb) * self.q_lsb
        qd = (qm - self.q_prev_meas) / dt
        self.q_prev_meas = qm
        pa = max(0.0, round((self.pa + self.rng.normal(0, self.p_lsb)) / self.p_lsb) * self.p_lsb)
        pb = max(0.0, round((self.pb + self.rng.normal(0, self.p_lsb)) / self.p_lsb) * self.p_lsb)
        return qm, qd, pa, pb


def smooth(x, n):
    """Zero-phase moving average (edge-padded)."""
    if n <= 1:
        return np.asarray(x, float)
    k = np.ones(n) / n
    xp = np.pad(np.asarray(x, float), (n, n), mode="edge")
    return np.convolve(xp, k, mode="same")[n:-n]


def reference(trace: dict, src_dt: float, dt: float, win_s: float = 0.04):
    """Smooth, mutually consistent (q, qd, qdd, tau) reference from a sim trace.

    The whole-body simulation contains contact chatter; the brain streams smooth
    trajectories, so the SIL replays a 40 ms zero-phase-filtered version."""
    src = np.arange(len(trace["q"])) * src_dt
    tt = np.arange(0, src[-1], dt)
    n = max(1, int(win_s / dt))
    q = smooth(smooth(np.interp(tt, src, trace["q"]), n), n)
    qd = np.gradient(q, dt)
    qdd = smooth(np.gradient(qd, dt), n)
    tau = smooth(np.interp(tt, src, trace["tau"]), n)
    return tt, q, qd, qdd, tau


def run_case(lib, cfg, jname, params, J, t, q_ref, qd_ref, tau_dem, mode, inject=None, dt=1e-3, qdd_ref=None):
    """Replay a reference; returns metrics dict + downsampled time series."""
    if qdd_ref is None:
        qdd_ref = np.gradient(qd_ref, dt)
    plant = Plant(cfg, jname, J, float(q_ref[0]))
    h = lib.hx_joint_new(C.byref(make_params(params)))
    # pre-pressurise chambers to hold the initial load
    lk, cyl = plant.lk, plant.cyl
    m0 = float(lk.moment_arm(q_ref[0])[0])
    F0 = tau_dem[0] / m0
    plant.pa = max(1e5, min(plant.ps, (F0 + 0.4 * plant.ps * cyl.a_annulus) / cyl.a_bore))
    plant.pb = max(1e5, (plant.pa * cyl.a_bore - F0) / cyl.a_annulus)
    inp, out = In(), Out()
    inp.dt, inp.kp_scale, inp.kd_scale = dt, 1.0, 1.0
    err, pmax, faults, valves = [], 0.0, 0, []
    log = {"t": [], "q": [], "q_ref": [], "pa": [], "pb": [], "spool": []}
    fault_t = None
    for k in range(len(t)):
        if inject and t[k] >= inject[1]:
            if inject[0] == "supply_loss":
                plant.supply_ok = False
        qm, qd, pa, pb = plant.sense(dt)
        inp.mode = MODE[mode]
        inp.q, inp.qd, inp.p_a, inp.p_b = qm, qd, pa, pb
        inp.p_supply = plant.ps if plant.supply_ok else 0.8e5
        inp.q_valid = 0 if (inject and inject[0] == "encoder" and t[k] >= inject[1]) else 1
        inp.p_valid = 0 if (inject and inject[0] == "pressure" and t[k] >= inject[1]) else 1
        inp.q_ref, inp.qd_ref = float(q_ref[k]), float(qd_ref[k])
        # the brain's whole-body model is imperfect: feed-forward = 85 % of the true load torque
        inp.tau_ff = FF_MODEL_ERROR * float(tau_dem[k]) if mode in ("IMPEDANCE", "FORCE") else 0.0
        lib.hx_joint_step(h, C.byref(inp), C.byref(out))
        if out.faults and fault_t is None:
            fault_t = t[k]
        faults |= out.faults
        tau_ext = J * qdd_ref[k] - tau_dem[k]
        plant.step(out.pulse_us if out.pulse_us else 1500.0, tau_ext, dt)
        err.append(plant.q - q_ref[k])
        pmax = max(pmax, plant.pa, plant.pb)
        valves.append(out.spool)
        if k % 10 == 0:
            for key, v in (("t", t[k]), ("q", plant.q), ("q_ref", q_ref[k]), ("pa", plant.pa), ("pb", plant.pb),
                           ("spool", out.spool)):
                log[key].append(float(v))
    lib.hx_joint_free(h)
    e = np.degrees(np.asarray(err))
    settle = int(0.3 / dt)
    return dict(rms_err_deg=float(np.sqrt(np.mean(e[settle:] ** 2))), max_err_deg=float(np.max(np.abs(e[settle:]))),
                peak_pressure_bar=pmax / 1e5, faults=int(faults), fault_time=fault_t,
                valve_saturation=float(np.mean(np.abs(valves) > 0.98))), log


def step_metrics(t, q, q0, q1):
    y = (np.asarray(q) - q0) / (q1 - q0)
    t = np.asarray(t)
    i10 = np.argmax(y >= 0.1)
    i90 = np.argmax(y >= 0.9)
    over = max(0.0, float(np.max(y) - 1.0))
    band = np.abs(y - 1.0) > 0.05
    settle = float(t[np.nonzero(band)[0][-1]]) if np.any(band) else 0.0
    return dict(rise_s=float(t[i90] - t[i10]), overshoot=over, settle_5pct_s=settle,
                ss_err_deg=float(abs(math.degrees((q[-1] - q1)))))


def run_all(cfg: RobotConfig, log=print, plot=True) -> dict:
    from ..codegen import _reflected_inertia
    build_lib()
    lib = load_lib()
    params = cfg.derived("joint_params")
    traces = cfg.derived("sim_traces")
    crit = cfg["qa"]["sil"]
    inertia = _reflected_inertia(cfg)
    dt = 1e-3
    results = {}
    for j in cfg.hydraulic_joints:
        if j.side != "L":
            continue
        P = params[j.name]
        J = max(inertia[j.name], 0.01)
        res = {}
        # 1) position step under the mean walking load
        tr = traces["walk"][j.name]
        tau_mean = float(np.mean(tr["tau"]))
        q0 = float(np.mean(tr["q"]))
        lo, hi = j.range_rad
        q1 = min(hi - 0.1, q0 + math.radians(8)) if q0 + math.radians(8) < hi - 0.1 else q0 - math.radians(8)
        t = np.arange(0, 1.2, dt)
        qr = np.where(t < 0.2, q0, q1)
        m, lg = run_case(lib, cfg, j.name, P, J, t, qr, np.zeros_like(t), np.full_like(t, tau_mean), "POSITION")
        sm = step_metrics(np.array(lg["t"])[20:] - 0.2, np.array(lg["q"])[20:], q0, q1)
        res["step"] = m | sm
        # 2) trace replay (walk, squat) in impedance mode with torque feed-forward
        for sc in ("walk", "squat"):
            tt, qr, qdr, qddr, taur = reference(traces[sc][j.name], traces["dt"], dt)
            m, lg = run_case(lib, cfg, j.name, P, J, tt, qr, qdr, taur, "IMPEDANCE", qdd_ref=qddr)
            res[sc] = m
            if plot and sc == "walk":
                _plot(lg, ROOT / "docs" / "img" / f"sil_{j.base}.png", f"SIL {j.base}: walk replay (impedance mode)")
        # 3) fault injection: encoder failure & supply loss must lock the valve
        t = np.arange(0, 0.6, dt)
        for kind in ("encoder", "pressure", "supply_loss"):
            m, _ = run_case(lib, cfg, j.name, P, J, t, np.full_like(t, q0), np.zeros_like(t),
                            np.full_like(t, tau_mean), "IMPEDANCE", inject=(kind, 0.3))
            res[f"fault_{kind}"] = m
        ok = (res["step"]["overshoot"] <= crit["max_overshoot"] and res["step"]["settle_5pct_s"] <= crit["max_settle_s"]
              and res["step"]["ss_err_deg"] <= crit["max_ss_err_deg"]
              and res["walk"]["rms_err_deg"] <= crit["max_rms_err_deg"] and res["walk"]["max_err_deg"] <= crit["max_err_deg"]
              and res["squat"]["max_err_deg"] <= crit["max_err_deg"]
              and max(res["walk"]["peak_pressure_bar"], res["squat"]["peak_pressure_bar"]) < cfg["control"]["pressure_trip_bar"]
              and res["walk"]["faults"] == 0 and res["squat"]["faults"] == 0
              and all(res[f"fault_{k}"]["faults"] != 0 and res[f"fault_{k}"]["fault_time"] is not None
                      and res[f"fault_{k}"]["fault_time"] < 0.3 + crit["max_fault_detect_s"]
                      for k in ("encoder", "pressure", "supply_loss")))
        res["pass"] = bool(ok)
        results[j.base] = res
        log(f"  [sil] {j.base:15s} step: os {res['step']['overshoot']*100:4.1f}% ts {res['step']['settle_5pct_s']:.2f}s "
            f"ss {res['step']['ss_err_deg']:.2f}deg | walk rms {res['walk']['rms_err_deg']:.2f} max {res['walk']['max_err_deg']:.2f}deg "
            f"| squat max {res['squat']['max_err_deg']:.2f}deg | pmax {max(res['walk']['peak_pressure_bar'], res['squat']['peak_pressure_bar']):.1f} bar "
            f"| {'PASS' if ok else 'FAIL'}")
    return results


def _plot(lg, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True, dpi=100)
    ax[0].plot(lg["t"], np.degrees(lg["q_ref"]), label="reference", lw=1.2)
    ax[0].plot(lg["t"], np.degrees(lg["q"]), label="SIL plant", lw=1.0)
    ax[0].set_ylabel("angle (deg)"); ax[0].legend(loc="upper right")
    ax[1].plot(lg["t"], np.array(lg["pa"]) / 1e5, label="p_A (bore)")
    ax[1].plot(lg["t"], np.array(lg["pb"]) / 1e5, label="p_B (annulus)")
    ax[1].set_ylabel("bar"); ax[1].legend(loc="upper right")
    ax[2].plot(lg["t"], lg["spool"]); ax[2].set_ylabel("spool"); ax[2].set_xlabel("t (s)")
    fig.suptitle(title)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
