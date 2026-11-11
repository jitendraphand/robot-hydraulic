# HX-1 design summary

Auto-generated from `config/robot.yaml` (`393db12be9dd23c9`) and the design-loop outputs in `config/derived/`. Regenerate with `make all`.

## Key figures

| quantity | value |
|---|---|
| total mass (no payload) | 19.60 kg |
| printed structure | 8.40 kg, ~191 h printing on the Neptune 4 Max |
| height (standing) | ~1.40 m |
| actuated DOF | 24 (14 hydraulic, 10 servo) |
| supply / relief pressure | 15.0 / 17.0 bar |
| pump flow / accumulator usable | 7.65 L/min / 87 mL |
| design loop | converged in 3 iterations |

## Hydraulic actuators (left side; right is mirrored)

| joint | bore x stroke (mm) | rod | moment arm (mm) | peak demand (N m) | capacity ratio | peak flow (L/min) | cylinder mass (g) |
|---|---|---|---|---|---|---|---|
| ankle_pitch | 32 x 100 | 12 | 46-67 | 25.0 | 1.67 | 4.76 | 490 |
| ankle_roll | 20 x 55 | 8 | 64-71 | 14.4 | 1.17 | 1.05 | 239 |
| elbow | 20 x 40 | 8 | 12-24 | 4.4 | 1.27 | 0.84 | 222 |
| hip_pitch | 16 x 140 | 6 | 35-151 | 10.6 | 1.47 | 1.55 | 271 |
| hip_roll | 25 x 75 | 10 | 82-119 | 21.4 | 1.32 | 1.75 | 338 |
| knee | 25 x 115 | 10 | 37-79 | 20.6 | 1.19 | 5.85 | 407 |
| shoulder_pitch | 32 x 75 | 12 | 12-39 | 8.8 | 1.31 | 2.33 | 430 |

Capacity ratio = min over every simulated (angle, torque) sample of capacity / (1.5 x demand); >= 1 required.

## Simulation scenarios

| scenario | stable | max tilt (deg) | mass (kg) |
|---|---|---|---|
| arm_lift | yes | 0.2 | 21.6 |
| single_leg | yes | 0.9 | 19.6 |
| squat | yes | 0.3 | 19.6 |
| stand | yes | 0.2 | 19.6 |
| walk | yes | 1.6 | 19.6 |

## HPU (pump + accumulator vs scenario flow)

| scenario | peak flow L/min | mean flow L/min | min supply bar | pump duty |
|---|---|---|---|---|
| arm_lift | 6.17 | 2.30 | 13.5 | 0.30 |
| single_leg | 4.70 | 1.44 | 13.5 | 0.18 |
| squat | 4.39 | 2.11 | 13.5 | 0.24 |
| stand | 0.44 | 0.42 | 14.1 | 0.00 |
| walk | 16.07 | 5.25 | 13.5 | 0.67 |

## Hydraulic SIL (real firmware, lumped plant, 15 % feed-forward error)

| joint | step overshoot | settle 5 % | walk RMS / max err (deg) | squat max err (deg) | peak p (bar) | result |
|---|---|---|---|---|---|---|
| ankle_pitch | 3.1 % | 0.09 s | 0.38 / 1.14 | 0.59 | 9.9 | PASS |
| ankle_roll | 1.7 % | 0.08 s | 1.32 / 2.36 | 0.24 | 10.9 | PASS |
| elbow | 0.0 % | 0.11 s | 0.11 / 0.41 | 0.41 | 8.3 | PASS |
| hip_pitch | 5.0 % | 0.11 s | 1.12 / 2.36 | 1.16 | 11.9 | PASS |
| hip_roll | 4.0 % | 0.10 s | 0.79 / 1.68 | 0.17 | 11.1 | PASS |
| knee | 3.4 % | 0.09 s | 0.93 / 3.09 | 1.43 | 11.8 | PASS |
| shoulder_pitch | 1.8 % | 0.10 s | 0.07 / 0.20 | 0.20 | 8.3 | PASS |

![SIL knee](img/sil_knee.png)

## Electrical

- Battery: nominal 263 W, peak 828 W, runtime ~41 min
- CAN load: CAN_A 34 %, CAN_B 38 %
- ERC: 0 errors, 0 warnings

## Mass by body

| body | mass (kg) |
|---|---|
| L_ankle_link | 0.687 |
| L_foot | 0.462 |
| L_forearm | 0.269 |
| L_hand | 0.067 |
| L_hip_roll_link | 0.633 |
| L_hip_yaw_link | 0.522 |
| L_shin | 0.913 |
| L_shoulder_link | 0.348 |
| L_thigh | 0.798 |
| L_upper_arm | 0.483 |
| L_upper_arm_roll | 0.117 |
| R_ankle_link | 0.687 |
| R_foot | 0.462 |
| R_forearm | 0.269 |
| R_hand | 0.067 |
| R_hip_roll_link | 0.633 |
| R_hip_yaw_link | 0.522 |
| R_shin | 0.913 |
| R_shoulder_link | 0.348 |
| R_thigh | 0.798 |
| R_upper_arm | 0.483 |
| R_upper_arm_roll | 0.117 |
| head | 0.386 |
| neck_link | 0.088 |
| pelvis | 1.175 |
| torso | 7.354 |

## Renders

![nominal](img/assembly_zero.png)
![squat](img/assembly_squat.png)
![stride](img/assembly_stride.png)
![arms](img/assembly_arms.png)
![parts](img/parts_sheet.png)
