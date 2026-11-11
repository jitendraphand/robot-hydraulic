// Unit tests for the firmware core (run on host: `make fw-test`).
#include "hx/control.h"
#include "test.h"

using namespace hx;

static hx_joint_params_t knee_params() {
    hx_joint_params_t p{};
    p.q_min = 0.f; p.q_max = 1.6f; p.soft_margin = 0.05f;
    p.actuator = HX_ACT_HYDRAULIC; p.axis = 1;
    p.anchor_parent[0] = 0.13f; p.anchor_parent[2] = 0.24f;
    p.anchor_child[0] = 0.07f; p.anchor_child[2] = 0.02f;
    p.a_bore = 4.9e-4f; p.a_annulus = 4.1e-4f; p.min_moment_arm = 0.01f;
    p.p_supply = 15e5f; p.p_return = 0.5e5f; p.p_trip = 18.5e5f; p.p_supply_min = 9e5f;
    p.valve_q_rated = 5e-5f; p.valve_dp_rated = 10e5f; p.valve_overlap = 0.1f; p.valve_deadband = 0.02f;
    p.valve_rate = 57.f;
    p.kp = 240.f; p.kd = 10.f; p.kp_pos_valve = 5.f; p.kd_pos_valve = 0.05f;
    p.kp_force = 0.007f; p.ki_force = 0.08f; p.k_vel_ff = 0.9f; p.tau_max = 40.f;
    p.max_following_err = 0.2f; p.following_time = 0.25f;
    p.servo_center_us = 1500.f; p.servo_us_per_rad = 400.f;
    return p;
}

static hx_joint_in_t in_default(float q = 0.5f) {
    hx_joint_in_t in{};
    in.dt = 1e-3f; in.mode = HX_MODE_POSITION; in.q = q; in.q_valid = 1; in.p_valid = 1;
    in.p_a = 5e5f; in.p_b = 5e5f; in.p_supply = 15e5f; in.q_ref = q; in.kp_scale = 1; in.kd_scale = 1;
    return in;
}

TEST(linkage_moment_arm_matches_finite_difference) {
    for (int axis = 0; axis < 3; ++axis) {
        Linkage l;
        l.axis = axis;
        const float p[3] = {0.05f, -0.07f, 0.2f}, c[3] = {0.06f, 0.03f, -0.04f};
        for (int i = 0; i < 3; ++i) { l.p[i] = p[i]; l.c[i] = c[i]; }
        for (float q = -1.2f; q < 1.2f; q += 0.1f) {
            const float h = 1e-3f;
            const float fd = (l.length(q + h) - l.length(q - h)) / (2 * h);
            CHECK_NEAR(l.moment_arm(q), fd, 2e-4);
        }
    }
}

TEST(spool_map_deadband_overlap_monotonic) {
    SpoolValve v;
    v.overlap = 0.1f; v.deadband = 0.02f;
    CHECK(v.map(0.f) == 0.f);
    CHECK(v.map(0.019f) == 0.f);
    CHECK(v.map(0.03f) >= 0.1f);           // jumps over the overlap
    CHECK(v.map(-0.03f) <= -0.1f);
    CHECK_NEAR(v.map(1.f), 1.f, 1e-6);
    float prev = -2.f;
    for (float u = -1.f; u <= 1.f; u += 0.01f) { CHECK(v.map(u) >= prev); prev = v.map(u); }
}

TEST(spool_slew_rate_limited) {
    SpoolValve v;
    v.rate = 50.f;
    float s = 0;
    for (int i = 0; i < 10; ++i) s = v.step(1.f, 1e-3f);
    CHECK_NEAR(s, 0.5f, 1e-4);              // 10 ms * 50 /s
}

TEST(pid_antiwindup) {
    Pid p;
    p.kp = 1; p.ki = 100; p.out_min = -1; p.out_max = 1;
    for (int i = 0; i < 1000; ++i) p.step(10.f, 0, 1e-3f);
    CHECK(p.integ <= 1.f);
    float u = 0;
    for (int i = 0; i < 30; ++i) u = p.step(-0.5f, 0, 1e-3f);
    CHECK(u < 1.f);                          // recovers immediately, no windup tail
}

TEST(joint_off_mode_centres_valve) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default();
    in.mode = HX_MODE_OFF; in.q_ref = 1.0f;
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(o.valve_cmd == 0.f);
    CHECK(o.spool == 0.f);
    CHECK_NEAR(o.pulse_us, 1500.f, 1e-3);
}

TEST(joint_position_mode_drives_toward_reference) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default(0.5f);
    in.q_ref = 0.7f;
    hx_joint_out_t o;
    j.step(in, o);
    Linkage l; l.axis = 1; l.p[0] = 0.13f; l.p[2] = 0.24f; l.c[0] = 0.07f; l.c[2] = 0.02f;
    // the commanded valve direction must move q toward q_ref
    const float qdir = signf(o.valve_cmd) * signf(l.moment_arm(0.5f));
    CHECK(qdir > 0);
    in.q_ref = 0.3f;
    j.step(in, o);
    CHECK(signf(o.valve_cmd) * signf(l.moment_arm(0.5f)) < 0);
}

TEST(joint_overpressure_trips_and_latches) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default();
    in.q_ref = 1.0f;
    in.p_a = 19e5f;
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(o.faults & HX_JF_OVERPRESSURE);
    CHECK(o.valve_cmd == 0.f);
    in.p_a = 5e5f;
    j.step(in, o);
    CHECK(o.faults & HX_JF_OVERPRESSURE);    // latched
    CHECK(o.valve_cmd == 0.f);
    j.clear_faults();
    j.step(in, o);
    CHECK(o.faults == 0);
}

TEST(joint_sensor_faults) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default();
    in.q_valid = 0;
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(o.faults & HX_JF_ENCODER);
    j.clear_faults();
    in = in_default();
    in.p_valid = 0;
    j.step(in, o);
    CHECK(o.faults & HX_JF_PRESSURE_SENSOR);
}

TEST(joint_supply_low_fault) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default();
    in.p_supply = 5e5f;
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(o.faults & HX_JF_SUPPLY_LOW);
    CHECK(o.valve_cmd == 0.f);
}

TEST(joint_soft_limit_blocks_further_motion) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default(1.58f);           // inside q_max - soft_margin band
    in.mode = HX_MODE_FORCE;
    in.p_a = 1e5f; in.p_b = 1e5f;
    in.tau_ff = 20.f;                      // torque pushing further into +q limit
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(o.valve_cmd == 0.f);
    in.tau_ff = -20.f;                     // pushing away from the limit is allowed
    j.step(in, o);
    CHECK(o.valve_cmd != 0.f);
    // position mode clamps the reference inside the soft band -> moves back out
    in = in_default(1.58f);
    in.q_ref = 2.5f;
    j.reset();
    j.step(in, o);
    Linkage l; l.axis = 1; l.p[0] = 0.13f; l.p[2] = 0.24f; l.c[0] = 0.07f; l.c[2] = 0.02f;
    CHECK(signf(o.valve_cmd) * signf(l.moment_arm(1.58f)) <= 0);
}

TEST(joint_following_error_fault_after_timeout) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default(0.5f);
    in.q_ref = 1.2f;                       // 0.7 rad error, never moves (stuck joint)
    hx_joint_out_t o;
    for (int i = 0; i < 200; ++i) j.step(in, o);
    CHECK((o.faults & HX_JF_FOLLOWING) == 0);
    for (int i = 0; i < 100; ++i) j.step(in, o);
    CHECK(o.faults & HX_JF_FOLLOWING);
}

TEST(joint_hold_captures_position) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default(0.8f);
    in.mode = HX_MODE_HOLD; in.q_ref = 0.1f;   // q_ref ignored in HOLD
    hx_joint_out_t o;
    j.step(in, o);
    CHECK(fabsf(o.valve_cmd) < 1e-6f);
    in.q = 0.75f;                               // drifts down -> should push back up
    j.step(in, o);
    Linkage l; l.axis = 1; l.p[0] = 0.13f; l.p[2] = 0.24f; l.c[0] = 0.07f; l.c[2] = 0.02f;
    CHECK(signf(o.valve_cmd) * signf(l.moment_arm(0.75f)) > 0);
}

TEST(joint_impedance_force_loop_sign) {
    JointController j;
    j.configure(knee_params());
    auto in = in_default(0.5f);
    in.mode = HX_MODE_FORCE;
    in.tau_ff = 20.f;
    in.p_a = 1e5f; in.p_b = 1e5f;             // measured force ~ 0 -> must increase tau
    hx_joint_out_t o;
    for (int i = 0; i < 5; ++i) j.step(in, o);
    CHECK(o.tau_cmd > 19.f);
    // increasing tau along +q requires force sign = sign(m): extension if m>0
    CHECK(signf(o.valve_cmd) == signf(o.force_cmd));
    CHECK_NEAR(o.force_cmd * o.moment_arm, 20.f, 0.5f);
}

TEST(servo_joint_pulse_mapping) {
    hx_joint_params_t p{};
    p.actuator = HX_ACT_SERVO; p.q_min = -0.5f; p.q_max = 0.5f; p.soft_margin = 0.f;
    p.servo_center_us = 1500.f; p.servo_us_per_rad = 636.6f; p.max_following_err = 10; p.following_time = 1;
    JointController j;
    j.configure(p);
    auto in = in_default(0.f);
    in.q_ref = 0.25f;
    hx_joint_out_t o;
    j.step(in, o);
    CHECK_NEAR(o.pulse_us, 1500.f + 636.6f * 0.25f, 0.1f);
    in.q_ref = 3.f;                        // clamped to limit
    j.step(in, o);
    CHECK_NEAR(o.pulse_us, 1500.f + 636.6f * 0.5f, 0.1f);
}

TEST(supervisor_nominal_and_heartbeat_loss) {
    Supervisor s(0.05f, 1.0f);
    SupervisorInputs in{1e-3f, true, true, true, 0, NodeCmd::NONE};
    s.step(in);
    CHECK(s.state() == NodeState::IDLE);
    in.cmd = NodeCmd::ARM; s.step(in);
    CHECK(s.state() == NodeState::ARMED);
    CHECK(s.joint_mode(HX_MODE_IMPEDANCE) == HX_MODE_HOLD);
    in.cmd = NodeCmd::RUN; s.step(in);
    CHECK(s.state() == NodeState::RUNNING);
    CHECK(s.joint_mode(HX_MODE_IMPEDANCE) == HX_MODE_IMPEDANCE);
    in.cmd = NodeCmd::NONE; in.heartbeat_seen = false;
    for (int i = 0; i < 60; ++i) s.step(in);
    CHECK(s.state() == NodeState::HOLD);
    CHECK(s.joint_mode(HX_MODE_IMPEDANCE) == HX_MODE_HOLD);
    for (int i = 0; i < 1100; ++i) s.step(in);
    CHECK(s.state() == NodeState::IDLE);
    CHECK(s.joint_mode(HX_MODE_IMPEDANCE) == HX_MODE_OFF);
}

TEST(supervisor_estop_and_fault) {
    Supervisor s;
    SupervisorInputs in{1e-3f, true, true, true, 0, NodeCmd::NONE};
    s.step(in);
    in.cmd = NodeCmd::ARM; s.step(in);
    in.cmd = NodeCmd::RUN; s.step(in);
    in.cmd = NodeCmd::NONE; in.estop_line_ok = false; s.step(in);
    CHECK(s.state() == NodeState::ESTOP);
    CHECK(s.joint_mode(HX_MODE_POSITION) == HX_MODE_OFF);
    in.estop_line_ok = true; s.step(in);
    CHECK(s.state() == NodeState::ESTOP);          // needs explicit clear
    in.cmd = NodeCmd::CLEAR_FAULT; s.step(in);
    CHECK(s.state() == NodeState::IDLE);
    in.cmd = NodeCmd::ARM; s.step(in);
    in.cmd = NodeCmd::NONE; in.joint_faults = HX_JF_OVERPRESSURE; s.step(in);
    CHECK(s.state() == NodeState::FAULT);
    in.cmd = NodeCmd::CLEAR_FAULT; s.step(in);
    CHECK(s.state() == NodeState::FAULT);          // cannot clear while fault present
    in.joint_faults = 0; s.step(in);
    CHECK(s.state() == NodeState::IDLE);
}

TEST(supervisor_arm_requires_heartbeat) {
    Supervisor s;
    SupervisorInputs in{1e-3f, false, true, true, 0, NodeCmd::NONE};
    for (int i = 0; i < 100; ++i) s.step(in);
    in.cmd = NodeCmd::ARM; s.step(in);
    CHECK(s.state() == NodeState::IDLE);
}

TEST(hpu_prime_run_unload_and_trip) {
    HpuParams p{15e5f, 13.5e5f, 15.3e5f, 17.5e5f, 60.f, 1e-6f, 4e-6f, 0.15f, 1.f, 0.5f};
    HpuController h(p);
    HpuOut o = h.step(1e-3f, true, 0.f, true, 25.f, true);
    CHECK(o.state == HpuState::PRIME);
    CHECK(!o.dump_closed);
    for (int i = 0; i < 600; ++i) o = h.step(1e-3f, true, 0.f, true, 25.f, true);
    CHECK(o.state == HpuState::RUN);
    CHECK(o.dump_closed);
    CHECK(o.motor_duty > 0.1f);
    o = h.step(1e-3f, true, 15.5e5f, true, 25.f, true);   // above band -> unload
    CHECK(o.motor_duty == 0.f);
    o = h.step(1e-3f, true, 14.0e5f, true, 25.f, true);   // inside band -> still unloading (hysteresis)
    CHECK(o.motor_duty == 0.f);
    o = h.step(1e-3f, true, 13.0e5f, true, 25.f, true);   // below band -> pump
    CHECK(o.motor_duty > 0.f);
    o = h.step(1e-3f, true, 18e5f, true, 25.f, true);     // over-pressure
    CHECK(o.state == HpuState::FAULT);
    CHECK(!o.dump_closed);
    CHECK(o.motor_duty == 0.f);
}

TEST(hpu_overtemp_and_level) {
    HpuParams p{15e5f, 13.5e5f, 15.3e5f, 17.5e5f, 60.f, 1e-6f, 4e-6f, 0.15f, 1.f, 0.5f};
    HpuController h(p);
    HpuOut o = h.step(1e-3f, true, 14e5f, true, 75.f, true);
    CHECK(o.faults & 2);
    HpuController h2(p);
    o = h2.step(1e-3f, true, 14e5f, true, 25.f, false);
    CHECK(o.faults & 4);
}

TEST(as5048_roundtrip_and_parity) {
    for (float q = -3.f; q < 3.f; q += 0.37f) {
        float r = 0;
        CHECK(as5048_decode(as5048_encode_for_test(q), 0.f, 1, &r));
        CHECK_NEAR(r, q, 1e-3);
    }
    float r;
    const uint16_t f = as5048_encode_for_test(1.0f);
    CHECK(!as5048_decode(f ^ 0x0001u, 0.f, 1, &r));         // single bit flip -> parity error
    CHECK(!as5048_decode((uint16_t)((f | 0x4000u) ^ 0x8000u), 0.f, 1, &r));   // error flag set
}

TEST(pressure_sensor_scaling) {
    PressureSensor s{30e5f, 0.5f, 4.5f, 0.6667f, 3.3f};
    const uint16_t raw_lo = (uint16_t)(0.5f * 0.6667f / 3.3f * 4095.f);
    const uint16_t raw_hi = (uint16_t)(4.5f * 0.6667f / 3.3f * 4095.f);
    CHECK_NEAR(s.pa(raw_lo), 0.f, 0.01e5);
    CHECK_NEAR(s.pa(raw_hi), 30e5f, 0.05e5);
    CHECK(s.valid(raw_lo));
    CHECK(!s.valid(0));                   // open wire
    CHECK(!s.valid(4095));                // short to supply
}
