# 08 · Recursive quality assurance & control

QA here is a **tree**. Every node's status is the worst of its own checks and all of its
children, recursively, so the robot can only be "green" if every subsystem, component and
part beneath it is green. The same principle runs at two levels:

1. **Design QA** (automatic, `python -m hx qa`): verifies the design and every generated artefact.
   It gates the repository and CI.
2. **Build QA** (your measurements in `qa/build_log.yaml`): verifies the physical robot as you
   build it, rolling up **part → cylinder → joint → limb → robot**.

The latest report is in [`qa/report.md`](../qa/report.md).

## Design QA tree

```
HX-1 system
├─ Configuration & design loop   schema · topology/geometry consistency · artefacts fresh (config hash) · loop converged · mass budget
├─ Mechanical
│  ├─ Printability               bed fit (N4 Max) · watertight meshes · materials vs printer · plan/plates/BOM
│  └─ Assembly                   exact-boolean interference over ~120 poses · cylinder strokes · lug bearing stress
├─ Hydraulics                    torque vs demand at every sample · full-range floor & moment arm · valve flow · HPU · pressure ratings · proof procedure
├─ Whole-body simulation         stand · walk · squat · single-leg · arm lift · servo utilisation · soft-limit use
├─ Firmware & control            -Werror build + unit tests · node compile · generated headers current · brain interlocks
│  └─ Hydraulic SIL              per joint: step + walk/squat replay · fault injection
├─ Electronics                   ERC · power budget · harness · CAN load · e-stop architecture
├─ Documentation & safety
└─ Requirements traceability     every requirement in config/requirements.yaml verified by >= 1 check
```

### The control loop around the QA ("recursive control")

QA findings feed back into the design inputs, and the generators re-derive everything downstream:

* **Design loop.** masses → simulation → sizing → CAD → masses, iterated to a fixed point with
  hysteresis. `CFG-4` fails if it did not converge.
* **Freshness.** Every artefact stores the hash of `robot.yaml`, so editing the config without
  regenerating fails `CFG-3`.
* **Derived gains.** Controller gains come from the sized hardware (areas, volumes, valve flow,
  inertia), so a hardware change automatically retunes the firmware.
* **Real examples found by this QA during development:**
  * the interference check reshaped the hip (non-intersecting axes);
  * the SIL drove the force-loop gain derivation and the valve upsizing;
  * the ERC moved pins off the DevKit's RGB-LED line;
  * the power check upsized the servo regulator;
  * the harness check moved the main leads to AWG8.

## Build QA (physical)

`qa/build_log.yaml` is generated from the design and never overwritten; new items are merged in.
Fill in measurements as you go. `null` means pending. Acceptance limits are derived from
robot.yaml:

| level | fields (criterion) | depends on |
|---|---|---|
| calibration | coupon printed, bearing fit OK, inserts OK | — |
| part | printed, dimensions OK (±0.2 mm on bores), inserts OK, bearing fit OK | coupon of its material |
| cylinder | proof_test_bar ≥ 1.5 × relief, proof_hold_min ≥ 5, external_leak_drops = 0, breakaway_bar ≤ 1.5, internal_leak_ml_min ≤ 0.5 | its 3 printed parts |
| joint | range_error_deg ≤ 2, encoder_zeroed, bench_step_overshoot ≤ 15 %, bench_ss_error_deg ≤ 0.6, fault_lock_ok | its links + cylinder |
| limb | node_flashed, can_ok, hold_test_ok | its joints |
| system | estop_test_ok, heartbeat_loss_test_ok, hpu_pressure_bar ≥ band low, relief_set_bar = relief ± 0.5, gantry_stand_ok, gantry_walk_ok | all limbs |

Any FAIL propagates upward. A limb with a failed cylinder cannot pass, and the robot cannot pass
until every limb does.

## Procedures

### Cylinder proof test (every cylinder, before installation)

1. Fill the cylinder with oil and bleed all air (air stores energy, oil does not).
2. Put it behind a polycarbonate shield. Wear nitrile gloves and safety glasses. Keep hands out of the line of the rod.
3. Use a hand pump with gauge (0–40 bar) on port A, port B open to a cup. Extend fully against the end stop.
4. Raise slowly to **25.5 bar (1.5 × relief)** and hold **5 min**. Pass: no external drop, no pressure loss > 0.5 bar, no visible cap deformation.
5. Repeat on port B with the cylinder fully retracted.
6. **Breakaway**: with both ports at low pressure, raise A until the rod moves. Record the bar; ≤ 1.5 bar passes.
7. **Internal leak**: hold the rod at mid-stroke, pressurise A to 15 bar and measure B outflow over 1 min. ≤ 0.5 mL/min passes.
8. Record the values in `qa/build_log.yaml` under `cylinders.cyl_<joint>`.

### Joint bench test (node on bench supply, robot on gantry, HPU at 15 bar)

1. `cal <j>`, then sweep by console to both soft limits and record the range error vs config.
2. Step +8° in POSITION mode; overshoot and steady-state error must meet the SIL criteria.
3. Unplug the encoder connector: the joint must lock within 50 ms (`fault_lock_ok`).

### System bring-up (on a gantry/harness, always)

1. E-stop test: press while RUNNING. The pump must stop, supply must fall below 2 bar within 5 s,
   joints must lock, and every node must report ESTOP.
2. Heartbeat loss: kill the brain process. Nodes go HOLD, then IDLE after 1 s, with no motion.
3. Relief setting: dead-head the pump and adjust relief to 17 ± 0.5 bar.
4. Stand on the gantry with the feet loaded through a scale for 5 min, then walk 8 steps with the harness slack but attached.

## Running

```bash
make qa
```

```bash
make qa-fast     # reuse cached SIL results if robot.yaml is unchanged
```

CI runs the full QA on every push (`.github/workflows/ci.yml`) and adds the ESP32 cross-compile.
