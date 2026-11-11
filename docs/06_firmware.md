# 06 · Firmware

```
firmware/
  core/include/hx/joint_types.h   C ABI (shared with Python SIL via ctypes)
  core/include/hx/control.h       Linkage, SpoolValve, Pid, JointController, Supervisor, HpuController, sensors
  core/include/hx/robot_config.h  GENERATED per-node joint tables (limits, linkage, areas, gains)
  core/include/hx/can_protocol.h  GENERATED CAN IDs / frames
  core/src/control.cpp            implementation (no heap, no exceptions)
  sil/hx_sil.cpp                  C API for the SIL harness (host only)
  tests/                          24 unit tests (~4600 checks)
  platform/esp32s3/               PlatformIO app: HAL (TWAI, SPI, LEDC), 1 kHz task, console
```

## Joint control law

```
IMPEDANCE (default while walking)
  tau_cmd = kp*(q_ref - q) + kd*(qd_ref - qd) + tau_ff          (clamped to tau_max)
  F_cmd   = tau_cmd / m(q)                 m = dL/dq from the cylinder linkage (generated)
  F_meas  = p_A*A_bore - p_B*A_annulus     (filtered 2 ms)
  u       = PI_force(F_cmd - F_meas) + A*m*qd_ref / Q_avail        (flow feed-forward)
POSITION (bench tests, HOLD)
  u = sign(m) * (kpv*e + kiv*∫e + kdv*ė) + flow feed-forward
spool = overlap-compensated map(u) -> slew limit -> servo pulse 1500 ± 400 us
```

**Gain derivation (in `hx/codegen.py`, not hand-tuned).** The chamber is an integrator:
dF/dt ≈ G·u with G = A·β_eff·Q/V. The force-loop gain is k_p = ω_f/G, with ω_f = 2π·12 Hz below
the valve-servo bandwidth. The first attempt used a rule-of-thumb gain about 100× higher;
the SIL showed it limit-cycling between supply and tank and tripping over-pressure. Position
mode uses k = ω_p/ω_max (4.5 Hz). Impedance stiffness puts the peak demand torque at 2.5° error.

## Protections (all latched, unit-tested)

| fault | condition | action |
|---|---|---|
| ENCODER | AS5048A parity/error flag | valve centred (lock) |
| PRESSURE_SENSOR | ADC outside 0.25–4.75 V (open/short) | lock |
| OVERPRESSURE | chamber > 18.5 bar | lock |
| SUPPLY_LOW | supply < 60 % nominal while active | lock |
| FOLLOWING | error > 12° for 250 ms | lock |
| LIMIT | > 2° beyond a hard limit | lock |
| soft limit | inside 3° of a limit | command pushing further is zeroed |

## Node state machine

```
BOOT ─selftest─► IDLE ─ARM (+heartbeat)─► ARMED(HOLD) ─RUN─► RUNNING
                  ▲  ◄──DISARM──────────────┘                 │ heartbeat lost (50 ms)
                  │                                           ▼
                  └──────────── 1 s ─────────────────────── HOLD
 any joint fault ─► FAULT ─CLEAR (faults gone)─► IDLE      e-stop line ─► ESTOP ─CLEAR─► IDLE
```

The hub runs the HPU only in ARMED/RUNNING/HOLD. The NO dump valve opens in any other state.

## Building and flashing

```bash
make fw-test
```

```bash
cd firmware/platform/esp32s3 && pio run -e leg_L -t upload && pio device monitor -e leg_L
```

Environments: `leg_L` (id 1), `leg_R` (2), `arm_L` (3), `arm_R` (4), `hub` (5). Pins come from the
generated `include/pins.h`.

## Calibration (per joint, node in IDLE)

1. Put the joint in its zero position using the printed jig or the CAD reference: links collinear, foot flat.
2. Console: `cal <j>` stores the encoder zero in NVS. If the joint moves the wrong way, use `dir <j>`.
3. Run the bench step test (docs/08) and record overshoot and steady-state error in `qa/build_log.yaml`.

## Brain-side balance stabilizer

`hx/brain/runtime.py` runs the same `Stabilizer` as the simulation. It reads the hub's IMU
quaternion (CAN 0x3F1) and the joint states, then computes the CoM relative to the feet for both
the measured state and the reference with the MuJoCo kinematic model. It adds the hip-roll and
ankle corrections (clamped to ±0.15 rad) to the streamed setpoints. Until the IMU and every joint
state have arrived it is a no-op.

## CAN protocol

See `hx/protocol.py`. Setpoint frames are `q_ref` (1e-4 rad), `qd_ref` (1e-3 rad/s), `tau_ff`
(0.01 N m), `kp_scale` (0.01), and `mode|kd_scale`. State frames carry `q`, `qd`, `tau`
(from pressures), `p_A` and `p_B`. A sync/heartbeat goes out at 100 Hz, and status + HPU frames
at 20 Hz.
