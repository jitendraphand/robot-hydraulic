// HX-1 node application (limb nodes 1-4, hub node 5).
//
// Core 1: 1 kHz control task (sensors -> supervisor -> joint controllers -> PWM, CAN I/O)
// Core 0: Arduino loop(): serial console, IMU (hub), housekeeping.
//
// Safety model (see docs/09_safety.md):
//   * hardware e-stop removes pump/servo power and opens the NO dump valve;
//   * this firmware detects the e-stop line and centres every valve spool (hydraulic lock);
//   * brain heartbeat loss -> HOLD (lock position) -> IDLE after 1 s;
//   * any latched joint fault -> FAULT state, valves centred, explicit CLEAR required.
#include <Arduino.h>
#include <Preferences.h>

#include "hal.h"
#include "hx/can_protocol.h"
#include "hx/control.h"
#include "hx/robot_config.h"
#include "pins.h"

#ifndef HX_NODE_ID
#error "HX_NODE_ID must be defined (see platformio.ini)"
#endif
#ifdef HX_HUB
#include <Adafruit_BNO08x.h>
#endif

using namespace hx;

static Hal hal;
static const cfg::NodeDef* node = nullptr;
static JointController ctl[cfg::kMaxJointsPerNode];
static Supervisor sup(cfg::kHeartbeatTimeout, 1.0f);
static Preferences prefs;
static constexpr float kDt = 1.0f / cfg::kLoopHz;
static const PressureSensor kPs{cfg::kPressureRangePa, cfg::kPressureVlo, cfg::kPressureVhi, cfg::kPressureDivider, 3.3f};

struct JointIo {
    int enc = -1, adc_a = -1, adc_b = -1, pwm = -1;
    float zero = 0.f;
    int8_t dir = 1;
    float q = 0, q_prev = 0, qd = 0;
    float p_a = 0, p_b = 0;
    bool q_ok = false;
};
struct Setpoint {
    float q_ref = 0, qd_ref = 0, tau_ff = 0, kp = 1, kd = 1;
    int32_t mode = HX_MODE_HOLD;
    bool fresh = false;
};

static JointIo io[cfg::kMaxJointsPerNode];
static Setpoint sp[cfg::kMaxJointsPerNode];
static hx_joint_out_t jout[cfg::kMaxJointsPerNode];
static volatile NodeCmd pending_cmd = NodeCmd::NONE;
static volatile uint32_t loop_overruns = 0, loop_max_us = 0;
static float p_supply = 0.f;
static bool p_supply_ok = false;
static uint32_t hub_rx_tick = 0, now_tick = 0;     // limb nodes: supply pressure from the hub HPU frame
static float hub_p_supply = 0.f;

#ifdef HX_HUB
static HpuController hpu(cfg::kHpu);
static HpuOut hpu_out{};
static float oil_temp_c = 25.f, vbat = 0.f;
static Adafruit_BNO08x imu(pins::hub::IMU_RST);
static volatile float quat[4] = {1, 0, 0, 0};
#endif

// --------------------------------------------------------------- setup helpers
static void map_io() {
    int h = 0, s = 0;
    (void)h;
    for (int j = 0; j < node->joint_count; ++j) {
        const auto& p = node->joints[j];
        ctl[j].configure(p);
        char key[12];
        snprintf(key, sizeof(key), "z%d", j);
        io[j].zero = prefs.getFloat(key, 0.f);
        snprintf(key, sizeof(key), "d%d", j);
        io[j].dir = (int8_t)prefs.getChar(key, 1);
#ifndef HX_HUB
        if (p.actuator == HX_ACT_HYDRAULIC) {
            io[j].enc = h;
            io[j].adc_a = 2 * h;
            io[j].adc_b = 2 * h + 1;
            io[j].pwm = h;            // PWM0-4: valve servos
            ++h;
        } else {
            io[j].pwm = 5 + s;        // PWM5-7: joint servos
            ++s;
        }
#else
        io[j].pwm = 2 + s;            // neck servos on LEDC ch2/3
        ++s;
#endif
        sp[j].q_ref = 0.5f * (p.q_min + p.q_max);
    }
}

// ------------------------------------------------------------------ CAN I/O
static void handle_can() {
    CanFrame f;
    bool hb = false;
    while (hal.can_receive(f)) {
        if (f.id == can::kIdCmd && f.len >= 2) {
            if (f.data[1] & (1u << (node->id - 1))) pending_cmd = (NodeCmd)f.data[0];
        } else if (f.id == can::kIdSync) {
            hb = true;
        } else if (f.id == can::kIdHpu && f.len >= sizeof(can::HpuRaw)) {
            can::HpuRaw r;
            memcpy(&r, f.data, sizeof(r));
            hub_p_supply = r.p_supply * can::kHpu_p_supply;
            hub_rx_tick = now_tick;
        } else if (f.id >= can::setpoint_id(node->id, 0) && f.id < can::setpoint_id(node->id, node->joint_count)) {
            const int j = f.id - can::setpoint_id(node->id, 0);
            can::SetpointRaw r;
            memcpy(&r, f.data, sizeof(r));
            sp[j].q_ref = r.q_ref * can::kSetpoint_q_ref;
            sp[j].qd_ref = r.qd_ref * can::kSetpoint_qd_ref;
            sp[j].tau_ff = r.tau_ff * can::kSetpoint_tau_ff;
            sp[j].kp = r.kp_scale * can::kSetpoint_kp_scale;
            sp[j].kd = (r.mode_kd & 0x1F) / 16.f;
            sp[j].mode = r.mode_kd >> 5;
            sp[j].fresh = true;
        }
    }
    SupervisorInputs si{kDt, hb, hal.estop_line_ok(), true, 0, pending_cmd};
    uint32_t jf = 0;
    for (int j = 0; j < node->joint_count; ++j) jf |= ctl[j].faults();
    si.joint_faults = jf;
    const NodeState before = sup.state();
    sup.step(si);
    if (pending_cmd == NodeCmd::CLEAR_FAULT && sup.state() == NodeState::FAULT) {
        for (int j = 0; j < node->joint_count; ++j) ctl[j].clear_faults();
    }
    if (pending_cmd == NodeCmd::CLEAR_FAULT && before == NodeState::ESTOP) {
        for (int j = 0; j < node->joint_count; ++j) ctl[j].clear_faults();
    }
    pending_cmd = NodeCmd::NONE;
}

static void send_state(uint32_t tick) {
    CanFrame f;
    if (tick % 10 == 0) {                                // 100 Hz joint state
        for (int j = 0; j < node->joint_count; ++j) {
            can::StateRaw r;
            r.q = can::sat16(io[j].q / can::kState_q);
            r.qd = can::sat16(io[j].qd / can::kState_qd);
            r.tau = can::sat16(jout[j].tau_meas / can::kState_tau);
            r.p_a = can::sat8(io[j].p_a / can::kState_p_a);
            r.p_b = can::sat8(io[j].p_b / can::kState_p_b);
            f.id = can::state_id(node->id, (uint8_t)j);
            f.len = 8;
            memcpy(f.data, &r, 8);
            hal.can_send(f);
        }
    }
    if (tick % 50 == 0) {                                // 20 Hz status
        can::StatusRaw s;
        s.state = (uint8_t)sup.state();
        s.joint_count = node->joint_count;
        uint32_t jf = 0;
        for (int j = 0; j < node->joint_count; ++j) jf |= ctl[j].faults() << (j * 2 % 16);
        s.faults = (uint16_t)jf;
        s.p_supply = (uint16_t)(p_supply / can::kStatus_p_supply);
        s.temp = (int16_t)(temperatureRead() / can::kStatus_temp);
        f.id = can::kIdStatus + node->id;
        f.len = sizeof(s);
        memcpy(f.data, &s, sizeof(s));
        hal.can_send(f);
#ifdef HX_HUB
        can::HpuRaw hr;
        hr.state = (uint8_t)hpu_out.state;
        hr.faults = (uint8_t)hpu_out.faults;
        hr.p_supply = (uint16_t)(p_supply / can::kHpu_p_supply);
        hr.duty = can::sat8(hpu_out.motor_duty / can::kHpu_duty);
        hr.oil_temp = (int8_t)oil_temp_c;
        hr.vbat = (uint16_t)(vbat / can::kHpu_vbat);
        f.id = can::kIdHpu;
        f.len = sizeof(hr);
        memcpy(f.data, &hr, sizeof(hr));
        hal.can_send(f);
#endif
    }
#ifdef HX_HUB
    if (tick % 5 == 0) {                                 // 200 Hz IMU quaternion
        int16_t qv[4];
        for (int i = 0; i < 4; ++i) qv[i] = (int16_t)(quat[i] * 16384.f);
        f.id = can::kIdImu;
        f.len = 8;
        memcpy(f.data, qv, 8);
        hal.can_send(f);
    }
#endif
}

// ------------------------------------------------------------ control step
static void control_step(uint32_t tick) {
    uint16_t enc[6] = {0};
    uint16_t adc[16] = {0};
#ifndef HX_HUB
    hal.read_encoders(enc, 6);
    hal.read_adc(adc, 16);
    // supply pressure is measured by the hub and broadcast at 20 Hz; if that is stale
    // (> 250 ms) assume nominal so a hub fault does not drop the limbs (status flags it)
    now_tick = tick;
    const bool hub_fresh = (tick - hub_rx_tick) < 250 && hub_rx_tick != 0;
    p_supply_ok = true;
    p_supply = hub_fresh ? hub_p_supply : node->joints[node->joint_count - 1].p_supply;
#else
    hal.read_adc(adc, 3);
    p_supply_ok = kPs.valid(adc[0]);
    p_supply = kPs.pa(adc[0]);
#endif
    handle_can();

    for (int j = 0; j < node->joint_count; ++j) {
        const auto& P = node->joints[j];
        hx_joint_in_t in{};
        in.dt = kDt;
        if (io[j].enc >= 0) {
            float q;
            io[j].q_ok = as5048_decode(enc[io[j].enc], io[j].zero, io[j].dir, &q);
            if (io[j].q_ok) {
                io[j].qd = 0.8f * io[j].qd + 0.2f * (q - io[j].q_prev) / kDt;
                io[j].q_prev = q;
                io[j].q = q;
            }
        } else {
            io[j].q = sp[j].q_ref;     // hobby servo: no feedback, report command
            io[j].q_ok = true;
        }
        in.q = io[j].q;
        in.qd = io[j].qd;
        in.q_valid = io[j].q_ok;
        if (io[j].adc_a >= 0) {
            in.p_valid = kPs.valid(adc[io[j].adc_a]) && kPs.valid(adc[io[j].adc_b]);
            in.p_a = io[j].p_a = kPs.pa(adc[io[j].adc_a]);
            in.p_b = io[j].p_b = kPs.pa(adc[io[j].adc_b]);
        } else {
            in.p_valid = 1;
        }
        in.p_supply = p_supply;
        in.mode = sup.joint_mode(sp[j].mode);
        in.q_ref = sp[j].q_ref;
        in.qd_ref = sp[j].qd_ref;
        in.tau_ff = sp[j].tau_ff;
        in.kp_scale = sp[j].kp;
        in.kd_scale = sp[j].kd;
        ctl[j].step(in, jout[j]);
        if (P.actuator == HX_ACT_HYDRAULIC) {
            // OFF/FAULT/ESTOP: centred spool = closed centre = joint locked
            hal.set_pulse_us(io[j].pwm, in.mode == HX_MODE_OFF ? P.servo_center_us : jout[j].pulse_us);
        } else {
            hal.set_pulse_us(io[j].pwm, jout[j].pulse_us);   // 0 -> no pulses -> servo limp
        }
    }
#ifdef HX_HUB
    const bool enable = sup.state() == NodeState::ARMED || sup.state() == NodeState::RUNNING || sup.state() == NodeState::HOLD;
    hpu_out = hpu.step(kDt, enable, p_supply, p_supply_ok, oil_temp_c, hal.digital_in(pins::hub::LEVEL_SW));
    hal.set_pulse_us(0, 1000.f + 1000.f * hpu_out.motor_duty);
    hal.set_gpio(pins::hub::DUMP_VALVE, hpu_out.dump_closed);
    hal.set_pulse_us(4, oil_temp_c > 40.f ? 40.f : 0.f);
#endif
    send_state(tick);
    hal.led((tick / (sup.state() == NodeState::RUNNING ? 100 : 500)) & 1);
}

static void control_task(void*) {
    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        const uint32_t t0 = hal.micros32();
        control_step(tick++);
        const uint32_t dt = hal.micros32() - t0;
        if (dt > loop_max_us) loop_max_us = dt;
        if (dt > 900) ++loop_overruns;
    }
}

// ---------------------------------------------------------------- console
static void console() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "status") {
        Serial.printf("node %s id %d state %d overruns %lu max_loop %lu us can_err %lu\n", node->name, node->id,
                      (int)sup.state(), (unsigned long)loop_overruns, (unsigned long)loop_max_us,
                      (unsigned long)hal.can_errors());
        for (int j = 0; j < node->joint_count; ++j)
            Serial.printf("  %-18s q %+7.3f qd %+7.3f tau %+7.2f spool %+5.2f faults 0x%02lx\n", node->joint_names[j],
                          io[j].q, io[j].qd, jout[j].tau_meas, jout[j].spool, (unsigned long)ctl[j].faults());
    } else if (line.startsWith("cal ")) {
        // place the joint in the zero jig, then: cal <joint index>
        const int j = line.substring(4).toInt();
        if (j >= 0 && j < node->joint_count && sup.state() == NodeState::IDLE) {
            char key[12];
            snprintf(key, sizeof(key), "z%d", j);
            io[j].zero = prefs.getFloat(key, 0.f) + io[j].q * io[j].dir;
            prefs.putFloat(key, io[j].zero);
            Serial.printf("joint %d zero = %.4f rad (saved)\n", j, io[j].zero);
        } else {
            Serial.println("cal refused: bad joint or node not IDLE");
        }
    } else if (line.startsWith("dir ")) {
        const int j = line.substring(4).toInt();
        char key[12];
        snprintf(key, sizeof(key), "d%d", j);
        io[j].dir = (int8_t)-io[j].dir;
        prefs.putChar(key, io[j].dir);
        Serial.printf("joint %d direction %d (saved)\n", j, io[j].dir);
    } else if (line == "clear") {
        pending_cmd = NodeCmd::CLEAR_FAULT;
    } else {
        Serial.println("commands: status | cal <j> | dir <j> | clear");
    }
}

void setup() {
    Serial.begin(921600);
    for (int i = 0; i < cfg::kNodeCount; ++i)
        if (cfg::kNodes[i].id == HX_NODE_ID) node = &cfg::kNodes[i];
    if (!node) {
        for (;;) { Serial.println("HX: unknown node id"); delay(1000); }
    }
    prefs.begin("hx", false);
#ifdef HX_HUB
    const bool ok = hal.begin(true);
    Wire.begin(pins::hub::I2C_SDA, pins::hub::I2C_SCL, 400000);
    if (imu.begin_I2C()) imu.enableReport(SH2_ROTATION_VECTOR, 5000);
#else
    const bool ok = hal.begin(false);
#endif
    map_io();
    Serial.printf("HX-1 %s node %s (id %d) config %s: %s\n", cfg::kRobotName, node->name, node->id,
                  cfg::kConfigVersion, ok ? "ready" : "CAN INIT FAILED");
    xTaskCreatePinnedToCore(control_task, "hx_ctl", 8192, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
}

void loop() {
    console();
#ifdef HX_HUB
    sh2_SensorValue_t v;
    if (imu.getSensorEvent(&v) && v.sensorId == SH2_ROTATION_VECTOR) {
        quat[0] = v.un.rotationVector.real;
        quat[1] = v.un.rotationVector.i;
        quat[2] = v.un.rotationVector.j;
        quat[3] = v.un.rotationVector.k;
    }
    // NTC (10k B3950, 10k top resistor) and battery divider (ADC1)
    const float vn = hal.analog_mv(pins::hub::NTC_ADC) / 1000.f;
    const float r = 10000.f * vn / fmaxf(3.3f - vn, 0.01f);
    oil_temp_c = 1.f / (1.f / 298.15f + logf(fmaxf(r, 1.f) / 10000.f) / 3950.f) - 273.15f;
    vbat = hal.analog_mv(pins::hub::VBAT_ADC) / 1000.f / 0.1071f;
#endif
    delay(5);
}
