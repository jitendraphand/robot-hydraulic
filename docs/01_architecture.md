# 01 · System architecture

## Kinematics

```
pelvis (floating base) ─┬─ torso (rigid) ─┬─ L/R shoulder_pitch [H] ─ shoulder_roll [S] ─ shoulder_yaw [S] ─ elbow [H] ─ hand (gripper [S])
                        │                 └─ neck_yaw [S] ─ neck_pitch [S] ─ head
                        └─ L/R hip_yaw [S] ─ hip_roll [H] ─(50 mm)─ hip_pitch [H] ─ knee [H] ─ ankle_pitch [H] ─(45 mm)─ ankle_roll [H] ─ foot
[H] hydraulic cylinder   [S] servo
```

* Frames: x forward, y left, z up; every body frame sits on its joint axis and is aligned with
  the world at the zero pose, so CAD parts, MuJoCo bodies and the firmware linkage tables all
  share the same coordinates.
* The hip is deliberately **non-intersecting** (pitch axis 50 mm below roll) and the ankle
  likewise (roll 45 mm below pitch). An intersecting design needs a cross-block inside a yoke;
  the assembly interference check showed the cylinder brackets cannot be routed around such a
  yoke. The IK and simulation handle the offsets exactly.
* Joint ranges are limited by the linkage geometry of linear cylinders (e.g. knee 0–95°,
  hip pitch −50…30°) and verified against the simulated motions plus the soft-limit margin.

## Actuation

| joint | actuator | why |
|---|---|---|
| hip roll/pitch, knee, ankle pitch/roll | hydraulic cylinder | high torque, high stiffness, back-drivable through valve |
| shoulder pitch, elbow | hydraulic cylinder | lifting payload at arm's length |
| hip yaw | 150 kg·cm servo + 6808 bearing | low torque, a cylinder linkage would need > 60° rotation |
| shoulder roll/yaw, neck, gripper | servos | low torque |

Each hydraulic joint: printed tie-rod cylinder (Al barrel, chromed rod) with a **servo-actuated
miniature proportional 4/3 closed-centre spool valve** clamped on its barrel, two pressure
transducers (chambers A/B) and an AS5048A magnetic encoder on the joint bolt.

## Electronics and computation

```
Raspberry Pi 5 (brain) ── USB-CAN ─┬─ CAN_A 1 Mbit ── leg_L node ── leg_R node
   gait + whole-body IK            └─ CAN_B 1 Mbit ── arm_L node ── arm_R node ── hub node (HPU, IMU, neck)
   100 Hz setpoints, sync/heartbeat
```

* Node = ESP32-S3 DevKitC + SN65HVD230 + 2× MCP3208 + up to 6 encoders + 8 servo PWM outputs.
* 1 kHz control loop per node (core 1), cascaded impedance → force control per hydraulic joint.
* Brain streams `q_ref, qd_ref, tau_ff` and gain scales; nodes are autonomous for safety
  (heartbeat loss → HOLD → IDLE).

## Software architecture: one config, many generated artefacts

| generator | input | output |
|---|---|---|
| `hx.config` | robot.yaml | expanded L/R bodies & joints |
| `hx.sim.model` | config + `mass_props.json` | MuJoCo MJCF |
| `hx.sim.scenarios` | MJCF | demand envelopes, traces, frames |
| `hx.hydraulics` | config + demand | `sizing.json` (bore, stroke, pins) |
| `hx.cad.*` | config + sizing | STL, plates, print plan, BOM, `mass_props.json` |
| `hx.codegen` | config + sizing + inertia | `robot_config.h`, `can_protocol.h` |
| `hx.electronics` | boards.yaml + parts.yaml | `pins.h`, netlists, power/harness/CAN reports |
| `hx.qa` | everything | `qa/report.md`, build QA |

Every derived artefact records the hash of `robot.yaml` that produced it
(`config/derived/manifest.json`); QA check `CFG-3` fails if anything is stale.
