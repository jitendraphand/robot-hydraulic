# HX-1 QA report

Generated 2026-11-11 09:29 UTC by `python -m hx qa` for robot.yaml `393db12be9dd23c9`.

Status of a node = worst of its own checks and all children (recursive roll-up). Design QA gates the repository; the build QA tracks the physical build from `qa/build_log.yaml`.

# Design QA

**Overall: ⚠️ WARN**

## Tree

- ⚠️ **HX-1 system**
  - ✅ `TRACE` requirements traceability — 37 requirements, all verified by >= 1 check
  - ✅ **Configuration & design loop**
    - ✅ `CFG-1` robot.yaml schema `SYS-006` — robot.yaml validates against robot.schema.json
    - ✅ `CFG-2` kinematic topology & geometry consistency `SYS-001` — 12 leg DOF, 8 arm DOF, 14 hydraulic; body origins match geometry
    - ✅ `CFG-3` derived artefacts up to date `SYS-006` — all 10 derived artefacts generated from robot.yaml 393db12be9dd23c9
    - ✅ `CFG-4` design loop converged `SYS-007` — converged in 3 iterations, mass 19.60 kg
    - ✅ `CFG-5` total mass budget `SYS-002` — 19.60 kg <= 25.0 kg
  - ⚠️ **Mechanical (CAD / printing)**
    - ⚠️ **Printability**
      - ✅ `CAD-1` parts fit Neptune 4 Max `MEC-001` — 54 part designs fit; largest torso [190.0, 378.0, 384.0] mm
      - ✅ `CAD-2` meshes watertight `MEC-002` — 66 STL files exist, watertight, positive volume
      - ⚠️ `CAD-3` materials vs printer `MEC-005` — temperatures within N4 Max limits; ['PA12-CF'] need a hardened-steel nozzle
      - ✅ `CAD-4` print plan, plates, BOM `MEC-007` — print plan, 10 plates, BOM with cylinder cut list
    - ✅ **Assembly**
      - ✅ `CAD-5` assembly interference (all poses) `MEC-003` — 116 poses, 0 interferences (> 2.0 mm3)
      - ✅ `CAD-6` cylinder strokes in all poses `MEC-004` — all cylinder lengths within stroke in 116 poses
      - ✅ `CAD-7` printed lug bearing stress `MEC-006` — min lug bearing SF 6.1 (ankle_pitch) >= 3.0
  - ✅ **Hydraulics**
    - ✅ `HYD-1` torque capacity vs simulated demand `HYD-001` — all 7 joint types >= SF x demand; tightest ankle_roll 1.17
    - ✅ `HYD-2` full-range capacity & moment arm `HYD-002` — full-range floor and moment arm >= 12 mm for all joints
    - ✅ `HYD-3` valve flow capacity `HYD-003` — valve 7.22 L/min >= 1.2 x peak 5.85 L/min (knee)
    - ✅ `HYD-4` HPU pump + accumulator `HYD-004` — min supply 13.5 bar (walk) >= 13.0; max pump duty 0.67; pump 7.65 L/min, accumulator 87 mL usable
    - ✅ `HYD-5` pressure ratings chain `HYD-005` — supply 15.0 < relief 17.0 < trip 18.5 < sensor 30; proof 25.5 bar <= valve/hose ratings
    - ✅ `HYD-6` proof-test procedure `HYD-006` — proof-test procedure documented
  - ✅ **Whole-body simulation**
    - ✅ `SIM-1` stand `SYS-003` — stable, tilt 0.2 deg, max hydraulic tracking 0.13 deg, mass 19.6 kg
    - ✅ `SIM-2` walk `SYS-003` — stable, tilt 1.6 deg, max hydraulic tracking 0.46 deg, mass 19.6 kg
    - ✅ `SIM-3` squat `SYS-004` — stable, tilt 0.3 deg, max hydraulic tracking 0.31 deg, mass 19.6 kg
    - ✅ `SIM-4` single-leg balance `SYS-004` — stable, tilt 0.9 deg, max hydraulic tracking 0.45 deg, mass 19.6 kg
    - ✅ `SIM-5` arm lift with payload `SYS-005` — stable, tilt 0.2 deg, max hydraulic tracking 0.70 deg, mass 21.6 kg
    - ✅ `SIM-6` servo torque utilisation `ELE-005` — max servo utilisation 31 % (L_hip_yaw) <= 50 %
    - ✅ `SIM-7` motion inside soft limits `SAF-002` — simulated motion stays inside soft limits
  - ✅ **Firmware & control**
    - ✅ `FW-1` host build (-Werror) + unit tests `CTL-001, CTL-007, SAF-002` — -Werror build, 24 tests, 4612 checks, 0 failed checks, 0 failed tests
    - ✅ `FW-2` node firmware compile check `CTL-002` — 5 node builds compile against the Arduino-ESP32 API (host check; full ESP32 toolchain build runs in CI: `pio run`)
    - ✅ `FW-3` generated headers current `CTL-003` — robot_config.h, can_protocol.h, pins.h match the configuration
    - ✅ `FW-4` brain arming interlocks `CTL-008` — arms with healthy nodes and pressure; refuses with a faulted node
    - ✅ **Hydraulic SIL (real firmware vs plant model)**
      - ✅ `SIL-hip_roll` hip_roll step + walk/squat replay `CTL-004, CTL-005` — step os 4.0 % ts 0.10 s ss 0.23 deg; walk rms 0.79 max 1.68 deg; squat max 0.17 deg; pmax 11.1 bar
      - ✅ `SIL-hip_pitch` hip_pitch step + walk/squat replay `CTL-004, CTL-005` — step os 5.0 % ts 0.11 s ss 0.16 deg; walk rms 1.12 max 2.36 deg; squat max 1.16 deg; pmax 11.9 bar
      - ✅ `SIL-knee` knee step + walk/squat replay `CTL-004, CTL-005` — step os 3.4 % ts 0.09 s ss 0.25 deg; walk rms 0.93 max 3.09 deg; squat max 1.43 deg; pmax 11.8 bar
      - ✅ `SIL-ankle_pitch` ankle_pitch step + walk/squat replay `CTL-004, CTL-005` — step os 3.1 % ts 0.09 s ss 0.23 deg; walk rms 0.38 max 1.14 deg; squat max 0.59 deg; pmax 9.9 bar
      - ✅ `SIL-ankle_roll` ankle_roll step + walk/squat replay `CTL-004, CTL-005` — step os 1.7 % ts 0.08 s ss 0.12 deg; walk rms 1.32 max 2.36 deg; squat max 0.24 deg; pmax 10.9 bar
      - ✅ `SIL-shoulder_pitch` shoulder_pitch step + walk/squat replay `CTL-004, CTL-005` — step os 1.8 % ts 0.10 s ss 0.08 deg; walk rms 0.07 max 0.20 deg; squat max 0.20 deg; pmax 8.3 bar
      - ✅ `SIL-elbow` elbow step + walk/squat replay `CTL-004, CTL-005` — step os 0.0 % ts 0.11 s ss 0.10 deg; walk rms 0.11 max 0.41 deg; squat max 0.41 deg; pmax 8.3 bar
      - ✅ `SIL-F` fault injection (encoder, pressure, supply) `CTL-006, SAF-002` — 21 injected faults detected within 50 ms, valve locked
  - ✅ **Electronics**
    - ✅ `ELE-1` ERC (limb + hub boards) `ELE-001` — 0 errors, 0 warnings (hub: 33 parts / 37 nets, limb: 53 parts / 56 nets)
    - ✅ `ELE-2` power budget `ELE-002` — min regulator margin 1.40x (SERVO_7V4); battery peak 42 A of 200 A; runtime ~41 min
    - ✅ `ELE-3` harness ampacity & fusing `ELE-003` — all wires fused within ampacity and <= 80 % loaded
    - ✅ `ELE-4` CAN bus load `ELE-004` — max bus load 38 % (CAN_B)
    - ✅ `ELE-5` e-stop power architecture `SAF-001` — pump+servo power e-stop switched; logic+valve servos always on; NO dump valve in BOM
  - ✅ **Documentation & safety**
    - ✅ `DOC-1` guides present `DOC-001` — 10 guides present
    - ✅ `DOC-2` safety guide content `SAF-003` — hazards, PPE, proof testing, e-stop, gantry bring-up covered

# Physical build QA

**Overall: ⏳ PENDING**

## Tree

- ⏳ **Physical build (qa/build_log.yaml)**
  - ⏳ **Calibration coupons**
    - ⏳ `BLD-CALI` Calibration coupons — 0 pass, 3 pending, 0 fail
  - ⏳ **Printed parts**
    - ⏳ `BLD-PART` Printed parts — 0 pass, 206 pending, 0 fail
  - ⏳ **Cylinders (proof-tested)**
    - ⏳ `BLD-CYLI` Cylinders (proof-tested) — 0 pass, 14 pending, 0 fail
  - ⏳ **Joints**
    - ⏳ `BLD-JOIN` Joints — 0 pass, 24 pending, 0 fail
  - ⏳ **Limbs / nodes**
    - ⏳ `BLD-LIMB` Limbs / nodes — 0 pass, 5 pending, 0 fail
  - ⏳ **System bring-up**
    - ⏳ `BLD-SYST` System bring-up — 0 pass, 1 pending, 0 fail
