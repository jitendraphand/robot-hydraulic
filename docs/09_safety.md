# 09 · Safety

A 19 kg robot with stored hydraulic energy can injure people. This design keeps pressure low
(15 bar) and provides layered protection, but **you** are responsible for building and testing
it safely. If in doubt, don't pressurise.

## Hazards

| hazard | mitigation |
|---|---|
| **Fluid injection injury**: a pinhole jet can penetrate skin even at modest pressure | never search for leaks with your hand (use cardboard). Depressurise before touching fittings. Any injection injury is a **medical emergency**: go to hospital immediately even if it looks minor |
| printed part rupture | 100 % infill pressure profile, tie-rod construction, per-cylinder **proof test** at 1.5 × relief behind a shield, re-test after any rework |
| stored energy in the accumulator | NO dump valve opens on e-stop/power loss. Check the gauge reads 0 before opening any line |
| pinch/crush in joints and cylinders | gantry/harness for all powered tests; keep clear of the leg envelope; soft limits + following-error trip |
| unexpected motion | arming sequence (IDLE→ARMED→RUN); heartbeat loss → HOLD → IDLE; faults latch until cleared |
| falls | gantry/harness until `gantry_walk_ok` passes; robot never walks untethered near people |
| LiPo fire | charge in a LiPo bag/box, never unattended; balance-charge; storage voltage when idle; 60 A main fuse |
| hot oil / motor | 60 °C oil trip; fan; let the HPU cool before service |
| slippery floor from oil | absorbent mat under the test area; clean spills immediately |

## PPE

Safety glasses at all times when the HPU is armed, nitrile gloves when handling oil, and closed
shoes. Use a polycarbonate shield for proof tests. Keep a fire extinguisher (class B/electrical)
next to the charging and test area.

## E-stop

* A hardwired mushroom button, plus an optional wireless e-stop, in series with the contactor
  coil. It is a Category 0 stop of the actuator power.
* Effect: pump off, joint servos unpowered, accumulator dumped through the NO valve. The always-on
  logic centres the spools, locking the joints so the robot does not collapse.
* The e-stop must be **tested at the start of every session** (build QA `estop_test_ok`).
* Software stop: `DISARM` / `ESTOP` CAN commands from the brain; heartbeat loss.

## Bring-up gating

The build QA enforces the order: no joint passes before its cylinder passed the proof test, no
limb before its joints, no system test before all limbs. Do not skip ahead. The first powered
tests of each limb are done with the HPU at 5 bar (temporarily set the band low in robot.yaml
and regenerate).

## Proof testing

See [08_quality_assurance.md](08_quality_assurance.md#cylinder-proof-test-every-cylinder-before-installation).
Proof = 1.5 × relief = 25.5 bar for 5 min, oil-filled and bled, behind a shield.
