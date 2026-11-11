// HX-1 firmware core implementation.
#include "hx/control.h"

#include <string.h>

namespace hx {

static constexpr float kPi = 3.14159265358979f;

// ------------------------------------------------------------------ Linkage
void Linkage::child_at(float q, float o[3]) const {
    const float cq = cosf(q), sq = sinf(q);
    switch (axis) {
        case 0:  o[0] = c[0];                  o[1] = cq * c[1] - sq * c[2]; o[2] = sq * c[1] + cq * c[2]; break;
        case 1:  o[0] = cq * c[0] + sq * c[2]; o[1] = c[1];                  o[2] = -sq * c[0] + cq * c[2]; break;
        default: o[0] = cq * c[0] - sq * c[1]; o[1] = sq * c[0] + cq * c[1]; o[2] = c[2]; break;
    }
}

float Linkage::length(float q) const {
    float r[3];
    child_at(q, r);
    const float d0 = r[0] - p[0], d1 = r[1] - p[1], d2 = r[2] - p[2];
    return sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
}

float Linkage::moment_arm(float q) const {
    float r[3];
    child_at(q, r);
    // d(Rc)/dq = a x (Rc)
    float dr[3];
    switch (axis) {
        case 0:  dr[0] = 0;     dr[1] = -r[2]; dr[2] = r[1]; break;
        case 1:  dr[0] = r[2];  dr[1] = 0;     dr[2] = -r[0]; break;
        default: dr[0] = -r[1]; dr[1] = r[0];  dr[2] = 0; break;
    }
    const float d0 = r[0] - p[0], d1 = r[1] - p[1], d2 = r[2] - p[2];
    const float L = sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
    if (L < 1e-6f) return 0.f;
    return (d0 * dr[0] + d1 * dr[1] + d2 * dr[2]) / L;
}

// --------------------------------------------------------- JointController
void JointController::configure(const hx_joint_params_t& p) {
    p_ = p;
    link_.axis = p.axis;
    for (int i = 0; i < 3; ++i) { link_.p[i] = p.anchor_parent[i]; link_.c[i] = p.anchor_child[i]; }
    valve_.overlap = p.valve_overlap;
    valve_.deadband = p.valve_deadband;
    valve_.rate = p.valve_rate;
    pos_pid_.kp = p.kp; pos_pid_.ki = p.ki; pos_pid_.kd = p.kd;
    pos_pid_.out_min = -p.tau_max; pos_pid_.out_max = p.tau_max;
    force_pid_.kp = p.kp_force; force_pid_.ki = p.ki_force; force_pid_.kd = 0;
    force_pid_.out_min = -1.f; force_pid_.out_max = 1.f;
    reset();
}

void JointController::reset() {
    pos_pid_.reset();
    force_pid_.reset();
    qd_filt_.reset();
    f_filt_.reset();
    valve_.reset();
    last_mode_ = HX_MODE_OFF;
    following_t_ = 0;
    pos_int_ = 0;
}

float JointController::valve_from_force(float f_cmd, float f_meas, float v_ref, float dt) {
    const float area = v_ref > 0 ? p_.a_bore : p_.a_annulus;
    const float dp = fmaxf(p_.p_supply - p_.p_return, 1e5f);
    const float q_avail = p_.valve_q_rated * sqrtf(dp / fmaxf(p_.valve_dp_rated, 1e3f));
    const float ff = p_.k_vel_ff * area * v_ref / fmaxf(q_avail, 1e-9f);
    return clampf(force_pid_.step(f_cmd - f_meas, 0.f, dt) + ff, -1.f, 1.f);
}

void JointController::step(const hx_joint_in_t& in, hx_joint_out_t& out) {
    memset(&out, 0, sizeof(out));
    const float dt = in.dt > 0 ? in.dt : 1e-3f;
    const bool hyd = p_.actuator == HX_ACT_HYDRAULIC;

    // ---- plausibility / protection (latched)
    if (!in.q_valid) faults_ |= HX_JF_ENCODER;
    if (hyd && !in.p_valid) faults_ |= HX_JF_PRESSURE_SENSOR;
    if (hyd && in.p_valid && (in.p_a > p_.p_trip || in.p_b > p_.p_trip)) faults_ |= HX_JF_OVERPRESSURE;
    if (in.q_valid && (in.q > p_.q_max + 0.035f || in.q < p_.q_min - 0.035f)) faults_ |= HX_JF_LIMIT;
    int32_t mode = faults_ ? HX_MODE_OFF : in.mode;
    if (hyd && mode != HX_MODE_OFF && in.p_valid && in.p_supply < p_.p_supply_min) {
        faults_ |= HX_JF_SUPPLY_LOW;
        mode = HX_MODE_OFF;
    }

    const float qd = qd_filt_.step(in.qd, dt, 0.004f);
    float m = hyd ? link_.moment_arm(in.q) : 1.f;
    out.moment_arm = m;
    const float f_meas = hyd ? f_filt_.step(in.p_a * p_.a_bore - in.p_b * p_.a_annulus, dt, 0.002f) : 0.f;
    out.force_meas = f_meas;
    out.tau_meas = f_meas * m;

    if (mode != last_mode_) {
        if (mode == HX_MODE_HOLD) hold_q_ = in.q;
        pos_pid_.reset();
        force_pid_.reset();
        following_t_ = 0;
        pos_int_ = 0;
    }
    last_mode_ = mode;

    float q_ref = in.q_ref, qd_ref = in.qd_ref, tau_ff = in.tau_ff;
    if (mode == HX_MODE_HOLD) { q_ref = hold_q_; qd_ref = 0; tau_ff = 0; }
    q_ref = clampf(q_ref, p_.q_min + p_.soft_margin, p_.q_max - p_.soft_margin);
    const float kps = in.kp_scale > 0 ? in.kp_scale : 1.f;
    const float kds = in.kd_scale > 0 ? in.kd_scale : 1.f;
    const float e = q_ref - in.q;

    // ---- following error supervision
    if (mode == HX_MODE_POSITION || mode == HX_MODE_IMPEDANCE || mode == HX_MODE_HOLD) {
        following_t_ = fabsf(e) > p_.max_following_err ? following_t_ + dt : 0.f;
        if (following_t_ > p_.following_time) faults_ |= HX_JF_FOLLOWING;
    }

    if (!hyd) {   // hobby/industrial servo: position command only
        if (mode == HX_MODE_OFF) { out.pulse_us = 0; return; }
        out.pulse_us = p_.servo_center_us + p_.servo_us_per_rad * q_ref;
        out.tau_cmd = 0;
        out.faults = faults_;
        return;
    }

    float u = 0.f;
    if (fabsf(m) < p_.min_moment_arm) m = signf(m == 0 ? 1.f : m) * p_.min_moment_arm;
    switch (mode) {
        case HX_MODE_HOLD:
        case HX_MODE_POSITION: {
            const float area = (e * m) > 0 ? p_.a_bore : p_.a_annulus;
            const float dp = fmaxf(p_.p_supply - p_.p_return, 1e5f);
            const float q_av = p_.valve_q_rated * sqrtf(dp / fmaxf(p_.valve_dp_rated, 1e3f));
            const float ff = p_.k_vel_ff * area * m * qd_ref / fmaxf(q_av, 1e-9f);
            // integral action overcomes spool overlap, seal friction and load (clamped, frozen in limits)
            pos_int_ = clampf(pos_int_ + p_.ki_pos_valve * e * dt, -0.3f, 0.3f);
            // valve opening -> piston velocity -> joint velocity scales with 1/m: normalise by m
            u = signf(m) * (kps * p_.kp_pos_valve * e + kds * p_.kd_pos_valve * (qd_ref - qd) + pos_int_) + ff;
            out.tau_cmd = 0;
            break;
        }
        case HX_MODE_IMPEDANCE:
        case HX_MODE_FORCE: {
            float tau = tau_ff;
            if (mode == HX_MODE_IMPEDANCE) {
                pos_pid_.kp = kps * p_.kp;
                pos_pid_.kd = kds * p_.kd;
                tau += pos_pid_.step(e, qd_ref - qd, dt);
            }
            tau = clampf(tau, -p_.tau_max, p_.tau_max);
            out.tau_cmd = tau;
            const float f_cmd = tau / m;
            out.force_cmd = f_cmd;
            u = valve_from_force(f_cmd, f_meas, m * qd_ref, dt);
            break;
        }
        default:
            u = 0.f;
            pos_pid_.reset();
            force_pid_.reset();
            break;
    }

    // ---- soft limits: never drive further into a limit
    const float q_dir = signf(u) * signf(m);
    if ((in.q >= p_.q_max - p_.soft_margin && q_dir > 0) || (in.q <= p_.q_min + p_.soft_margin && q_dir < 0)) u = 0.f;

    out.valve_cmd = clampf(u, -1.f, 1.f);
    out.spool = valve_.step(out.valve_cmd, dt);
    out.pulse_us = p_.servo_center_us + out.spool * p_.servo_us_per_rad;   // hydraulic: us per unit spool
    out.faults = faults_;
}

// ------------------------------------------------------------- Supervisor
NodeState Supervisor::step(const SupervisorInputs& in) {
    since_hb_ = in.heartbeat_seen ? 0.f : since_hb_ + in.dt;
    state_t_ += in.dt;
    const bool hb_ok = since_hb_ < hb_timeout_;
    if (in.cmd == NodeCmd::ESTOP || !in.estop_line_ok) {
        if (s_ != NodeState::BOOT) go(NodeState::ESTOP);
        return s_;
    }
    switch (s_) {
        case NodeState::BOOT:
            if (in.selftest_ok) go(NodeState::IDLE);
            break;
        case NodeState::ESTOP:
            if (in.cmd == NodeCmd::CLEAR_FAULT) go(NodeState::IDLE);
            break;
        case NodeState::IDLE:
            if (in.joint_faults) go(NodeState::FAULT);
            else if (in.cmd == NodeCmd::ARM && hb_ok) go(NodeState::ARMED);
            break;
        case NodeState::ARMED:
            if (in.joint_faults) go(NodeState::FAULT);
            else if (in.cmd == NodeCmd::DISARM || !hb_ok) go(NodeState::IDLE);
            else if (in.cmd == NodeCmd::RUN) go(NodeState::RUNNING);
            break;
        case NodeState::RUNNING:
            if (in.joint_faults) go(NodeState::FAULT);
            else if (in.cmd == NodeCmd::DISARM) go(NodeState::IDLE);
            else if (!hb_ok) go(NodeState::HOLD);
            break;
        case NodeState::HOLD:
            if (in.joint_faults) go(NodeState::FAULT);
            else if (in.cmd == NodeCmd::DISARM || state_t_ > hold_timeout_) go(NodeState::IDLE);
            else if (hb_ok && in.cmd == NodeCmd::RUN) go(NodeState::RUNNING);
            break;
        case NodeState::FAULT:
            if (in.cmd == NodeCmd::CLEAR_FAULT && !in.joint_faults) go(NodeState::IDLE);
            break;
    }
    return s_;
}

int32_t Supervisor::joint_mode(int32_t requested) const {
    switch (s_) {
        case NodeState::ARMED:
        case NodeState::HOLD: return HX_MODE_HOLD;
        case NodeState::RUNNING: return requested;
        default: return HX_MODE_OFF;
    }
}

// ---------------------------------------------------------- HpuController
HpuOut HpuController::step(float dt, bool enable, float p, bool p_valid, float temp_c, bool level_ok) {
    HpuOut o{0.f, false, s_, faults_};
    if (!p_valid) faults_ |= 8;
    if (p_valid && p > p_.p_trip) faults_ |= 1;
    if (temp_c > p_.t_max) faults_ |= 2;
    if (!level_ok) faults_ |= 4;
    if (faults_) s_ = HpuState::FAULT;
    t_ += dt;
    switch (s_) {
        case HpuState::OFF:
            pi_.reset();
            if (enable) { s_ = HpuState::PRIME; t_ = 0; }
            break;
        case HpuState::PRIME:
            if (!enable) { s_ = HpuState::OFF; break; }
            o.motor_duty = p_.min_duty;
            o.dump_closed = false;            // circulate to tank, purge air
            if (t_ > p_.prime_time) { s_ = HpuState::RUN; unloading_ = false; }
            break;
        case HpuState::RUN: {
            if (!enable) { s_ = HpuState::OFF; break; }
            o.dump_closed = true;
            if (p > p_.p_band_hi) unloading_ = true;
            if (p < p_.p_band_lo) unloading_ = false;
            if (unloading_) { o.motor_duty = 0.f; pi_.reset(); }
            else o.motor_duty = clampf(pi_.step(p_.p_set - p, 0.f, dt), p_.min_duty, p_.max_duty);
            break;
        }
        case HpuState::FAULT:
            o.motor_duty = 0.f;
            o.dump_closed = false;            // depressurise
            break;
    }
    o.state = s_;
    o.faults = faults_;
    return o;
}

// ---------------------------------------------------------------- sensors
static bool even_parity(uint16_t v) {
    v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return (v & 1u) == 0;
}

bool as5048_decode(uint16_t frame, float zero, int8_t dir, float* q) {
    if (!even_parity(frame)) return false;
    if (frame & 0x4000u) return false;
    const float a = (frame & 0x3FFFu) * (2.f * kPi / 16384.f);
    float r = dir * (a - zero);
    while (r >= kPi) r -= 2.f * kPi;
    while (r < -kPi) r += 2.f * kPi;
    *q = r;
    return true;
}

uint16_t as5048_encode_for_test(float q) {
    while (q < 0) q += 2.f * kPi;
    uint16_t v = static_cast<uint16_t>(q / (2.f * kPi) * 16384.f + 0.5f) & 0x3FFFu;
    if (!even_parity(v)) v |= 0x8000u;
    return v;
}

}  // namespace hx

