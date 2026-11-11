"""Python unit tests for the HX-1 design package."""
import math

import numpy as np
import pytest

from hx import protocol as P
from hx.config import RobotConfig
from hx.hydraulics import Linkage, make_cylinder, mirror_linkage, plane_point, torque_capacity


@pytest.fixture(scope="module")
def cfg():
    return RobotConfig()


def test_config_expands_left_right_mirrored(cfg):
    for j in cfg.joints.values():
        if j.side != "L":
            continue
        r = cfg.joints["R_" + j.base]
        assert r.actuator_type == j.actuator_type and r.axis == j.axis
    lk = cfg.joints["L_hip_roll"].range_deg
    rk = cfg.joints["R_hip_roll"].range_deg
    assert rk == (-lk[1], -lk[0])
    assert cfg.joints["L_knee"].range_deg == cfg.joints["R_knee"].range_deg
    assert cfg.bodies["R_hip_yaw_link"].origin[1] == -cfg.bodies["L_hip_yaw_link"].origin[1]
    assert len(cfg.hydraulic_joints) == 14


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_moment_arm_is_derivative_of_length(axis):
    lk = Linkage(axis, plane_point(axis, 0.03, 0.08, 0.2), plane_point(axis, 0.03, 0.05, -0.03))
    q = np.linspace(-1.0, 1.0, 41)
    h = 1e-5
    fd = (lk.length(q + h) - lk.length(q - h)) / (2 * h)
    assert np.allclose(lk.moment_arm(q), fd, atol=1e-6)


def test_mirror_linkage_preserves_length_under_mirrored_angle():
    lk = Linkage("x", np.array([0.0, 0.08, 0.15]), np.array([0.0, 0.07, -0.05]))
    mr = mirror_linkage(lk)
    for q in np.linspace(-0.3, 0.3, 7):
        assert lk.length(q)[0] == pytest.approx(mr.length(-q)[0])


def test_torque_capacity_uses_correct_area(cfg):
    cyl = make_cylinder(cfg, 32, 100)
    lk = Linkage("y", np.array([0.1, 0, 0.2]), np.array([0.05, 0, -0.03]))
    pos, neg = torque_capacity(lk, cyl, 1e6, np.array([0.3]))
    m = lk.moment_arm(np.array([0.3]))[0]
    big, small = 1e6 * cyl.a_bore * abs(m), 1e6 * cyl.a_annulus * abs(m)
    assert sorted([pos[0], neg[0]]) == pytest.approx([small, big])


def test_protocol_roundtrip():
    d = P.pack_setpoint(0.5, -1.2, 12.34, kp_scale=1.5, kd_scale=0.75, mode="POSITION")
    r = P.unpack("setpoint", d)
    assert r["q_ref"] == pytest.approx(0.5, abs=1e-4)
    assert r["qd_ref"] == pytest.approx(-1.2, abs=1e-3)
    assert r["tau_ff"] == pytest.approx(12.34, abs=1e-2)
    assert int(r["mode_kd"]) >> 5 == P.MODE["POSITION"]
    assert len(d) == 8


def test_gait_plan_zmp_tracking():
    from hx.sim.gait import GaitParams, plan
    p = GaitParams()
    r = plan(p, 0.62)
    acc = np.gradient(np.gradient(r["com"][:, 0], p.dt), p.dt)
    zmp = r["com"][:, 0] - 0.62 / 9.81 * acc
    err = np.abs(zmp - r["zmp"][:, 0])[200:-200]
    assert err.max() < 5e-3
    assert r["foot_L"][-1, 0] == pytest.approx(r["foot_R"][-1, 0])


def test_power_and_erc_clean(cfg):
    from hx.electronics.netlist import EDIR, build_hub, build_limb, erc
    import yaml
    B = yaml.safe_load(open(EDIR / "boards.yaml"))
    issues = erc(build_limb(B["boards"]["limb"])) + erc(build_hub(B["boards"]["hub"]))
    assert not [i for i in issues if i["severity"] == "error"]


def test_erc_detects_overvoltage():
    from hx.electronics.netlist import EDIR, build_limb, erc
    import yaml
    B = yaml.safe_load(open(EDIR / "boards.yaml"))
    n = build_limb(B["boards"]["limb"])
    # wire a raw 4.5 V pressure output straight into the ADC (bypassing the divider)
    n.nets["P_RAW0"].append("U3.CH7")
    n.nets["P_ADC7"].remove("U3.CH7")
    msgs = [i["msg"] for i in erc(n)]
    assert any("abs max" in m for m in msgs)


def test_stabilizer_signs_and_brain_noop(cfg):
    from hx.sim.stabilizer import Stabilizer, StabilizerGains
    idx = {f"{s}_{j}": i for i, (s, j) in enumerate((s, j) for s in "LR" for j in ("hip_roll", "ankle_roll", "ankle_pitch"))}
    st = Stabilizer(StabilizerGains(), idx)
    c = st.corrections(0.0, 0.0, [0.0, 0.01], [0.0, 0.0], len(idx))
    assert c[idx["L_ankle_roll"]] < 0 and c[idx["R_ankle_roll"]] < 0      # CoM too far left -> push ZMP left
    c = st.corrections(0.05, 0.0, [0, 0], [0, 0], len(idx))
    assert c[idx["L_hip_roll"]] > 0
    assert abs(st.corrections(9, 9, [9, 9], [9, 9], len(idx))).max() <= StabilizerGains().limit + 1e-12


def test_brain_stabilize_zero_when_on_reference(cfg):
    from hx.brain.runtime import Brain, NodeView, VirtualBus
    nodes = {n["name"]: NodeView(n["id"], n["name"], "", n["joints"]) for n in cfg["electrical"]["nodes"]}
    br = Brain(cfg, VirtualBus(nodes))
    q = {j: 0.1 if "knee" in j else 0.0 for j in br.jnames}
    for n in br.nodes.values():
        n.q = {j: q[j] for j in n.joints if j in q}
    br.quat = np.array([1.0, 0, 0, 0])
    out = br.stabilize(q, 0.01)
    assert max(abs(out[j] - q[j]) for j in out) < 1e-9
