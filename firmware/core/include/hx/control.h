// HX-1 firmware core: control primitives, joint controller, node supervisor,
// HPU controller.  Header-light, no heap, no exceptions, deterministic.
#pragma once

#include <math.h>
#include <stdint.h>

#include "hx/joint_types.h"

namespace hx {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float signf(float v) { return v > 0.f ? 1.f : (v < 0.f ? -1.f : 0.f); }

// ------------------------------------------------------------------ filters
struct LowPass {
    float y = 0.f;
    bool init = false;
    float step(float x, float dt, float tau) {
        if (!init) { y = x; init = true; return y; }
        const float a = dt / (tau + dt);
        y += a * (x - y);
        return y;
    }
    void reset() { init = false; y = 0.f; }
};

struct RateLimiter {
    float y = 0.f;
    float step(float x, float rate, float dt) {
        const float d = clampf(x - y, -rate * dt, rate * dt);
        y += d;
        return y;
    }
};

// PI(D) with conditional-integration anti-windup and output clamp.
struct Pid {
    float kp = 0, ki = 0, kd = 0, out_min = -1, out_max = 1;
    float integ = 0;
    float step(float err, float derr, float dt) {
        const float p = kp * err + kd * derr;
        const float u_unsat = p + integ + ki * err * dt;
        const float u = clampf(u_unsat, out_min, out_max);
        if (u == u_unsat || signf(err) != signf(u_unsat)) integ += ki * err * dt;   // no windup
        integ = clampf(integ, out_min, out_max);
        return clampf(p + integ, out_min, out_max);
    }
    void reset() { integ = 0; }
};

// ------------------------------------------------------------------ linkage
// Cylinder pin-to-pin length and moment arm dL/dq for a joint about a
// principal axis; anchors given in the joint frame (child anchor at q = 0).
struct Linkage {
    int axis = 1;
    float p[3] = {0, 0, 0};
    float c[3] = {0, 0, 0};
    void child_at(float q, float out[3]) const;
    float length(float q) const;
    float moment_arm(float q) const;   // m, tau = F * moment_arm
};

// --------------------------------------------------------- valve (spool) map
// Maps a normalised flow command (-1..1) to spool position, compensating the
// closed-centre overlap, then applies the servo slew limit.
struct SpoolValve {
    float overlap = 0.1f, deadband = 0.02f, rate = 30.f;
    RateLimiter slew;
    float map(float u) const {
        if (fabsf(u) < deadband) return 0.f;
        const float a = (fabsf(u) - deadband) / (1.f - deadband);
        return signf(u) * (overlap + (1.f - overlap) * clampf(a, 0.f, 1.f));
    }
    float step(float u, float dt) { return slew.step(map(clampf(u, -1.f, 1.f)), rate, dt); }
    void reset() { slew.y = 0.f; }
};

// --------------------------------------------------------- joint controller
class JointController {
public:
    void configure(const hx_joint_params_t& p);
    void reset();
    void step(const hx_joint_in_t& in, hx_joint_out_t& out);
    uint32_t faults() const { return faults_; }
    void clear_faults() { faults_ = 0; following_t_ = 0; }
    const hx_joint_params_t& params() const { return p_; }

private:
    float valve_from_force(float f_cmd, float f_meas, float v_piston_ref, float dt);
    hx_joint_params_t p_{};
    Linkage link_;
    SpoolValve valve_;
    Pid pos_pid_, force_pid_;
    LowPass qd_filt_, f_filt_;
    float hold_q_ = 0;
    float pos_int_ = 0;
    int32_t last_mode_ = HX_MODE_OFF;
    uint32_t faults_ = 0;
    float following_t_ = 0;
};

// --------------------------------------------------------- node supervisor
enum class NodeState : uint8_t { BOOT = 0, IDLE = 1, ARMED = 2, RUNNING = 3, HOLD = 4, FAULT = 5, ESTOP = 6 };
enum class NodeCmd : uint8_t { NONE = 0, ARM = 1, RUN = 2, DISARM = 3, CLEAR_FAULT = 4, ESTOP = 5 };

struct SupervisorInputs {
    float dt;
    bool heartbeat_seen;       // brain heartbeat received this cycle
    bool estop_line_ok;        // hardware e-stop loop closed (actuator power present)
    bool selftest_ok;          // sensors plausible after boot
    uint32_t joint_faults;     // OR of all joint faults
    NodeCmd cmd;
};

class Supervisor {
public:
    explicit Supervisor(float heartbeat_timeout_s = 0.05f, float hold_timeout_s = 1.0f)
        : hb_timeout_(heartbeat_timeout_s), hold_timeout_(hold_timeout_s) {}
    NodeState step(const SupervisorInputs& in);
    NodeState state() const { return s_; }
    // Joint mode the node must apply given the state and the brain's request.
    int32_t joint_mode(int32_t requested) const;
    uint32_t transitions() const { return transitions_; }

private:
    void go(NodeState n) { if (n != s_) { s_ = n; ++transitions_; state_t_ = 0; } }
    NodeState s_ = NodeState::BOOT;
    float hb_timeout_, hold_timeout_;
    float since_hb_ = 0, state_t_ = 0;
    uint32_t transitions_ = 0;
};

// --------------------------------------------------------- HPU controller
struct HpuParams {
    float p_set, p_band_lo, p_band_hi;   // Pa
    float p_trip;                        // Pa: relief-backup software trip
    float t_max;                         // C oil temperature
    float kp, ki;                        // motor duty per Pa, per Pa s
    float min_duty, max_duty;            // 0..1
    float prime_time;                    // s: pump runs with dump open to purge
};

enum class HpuState : uint8_t { OFF = 0, PRIME = 1, RUN = 2, FAULT = 3 };

struct HpuOut {
    float motor_duty;   // 0..1 to ESC
    bool dump_closed;   // energise NO dump valve (closed = accumulator holds pressure)
    HpuState state;
    uint32_t faults;    // 1 overpressure, 2 overtemp, 4 low level, 8 sensor
};

class HpuController {
public:
    explicit HpuController(const HpuParams& p) : p_(p) { pi_.kp = p.kp; pi_.ki = p.ki; pi_.out_min = 0; pi_.out_max = p.max_duty; }
    HpuOut step(float dt, bool enable, float p_supply, bool p_valid, float oil_temp_c, bool level_ok);
    void clear() { faults_ = 0; s_ = HpuState::OFF; pi_.reset(); }
    HpuState state() const { return s_; }

private:
    HpuParams p_;
    Pid pi_;
    HpuState s_ = HpuState::OFF;
    float t_ = 0;
    uint32_t faults_ = 0;
    bool unloading_ = false;
};

// ---------------------------------------------------------------- sensors
// 0.5-4.5 V ratiometric pressure transducer behind a divider into a 12-bit ADC.
struct PressureSensor {
    float range_pa, v_lo, v_hi, divider, vref;
    bool valid(uint16_t raw) const {
        const float v = raw * vref / 4095.f / divider;
        return v > v_lo - 0.25f && v < v_hi + 0.25f;
    }
    float pa(uint16_t raw) const {
        const float v = raw * vref / 4095.f / divider;
        return clampf((v - v_lo) / (v_hi - v_lo), -0.05f, 1.2f) * range_pa;
    }
};

// AS5048A 14-bit SPI frame: parity, error flag, angle. Returns false on error.
bool as5048_decode(uint16_t frame, float zero_offset_rad, int8_t direction, float* q_rad);
uint16_t as5048_encode_for_test(float q_rad);

}  // namespace hx
