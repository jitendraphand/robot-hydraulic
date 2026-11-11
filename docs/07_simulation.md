# 07 · Simulation

## Whole-body (MuJoCo)

`hx/sim/model.py` builds the MJCF from the config. Masses come from **CAD mass properties**
(volume × density with the slicer profile's shell/infill split), plus purchased hardware:
cylinders split 60/40 between parent and child, valve packs, servos, joint hardware, and torso
equipment. Contacts are only between the feet and the floor. Actuators are implicit PD position
servos (stable at 1 ms), used to measure the torque each joint *must* produce.

Scenarios (`hx/sim/scenarios.py`):

| scenario | what | used for |
|---|---|---|
| stand | 3 s quiet stance | baseline load |
| squat | pelvis down to the configured knee angle and back | knee/hip/ankle peak torque |
| walk | 8 steps, ZMP preview control (Kajita), 0.08 m / 0.8 s | demand, velocity, flow, traces |
| single_leg | CoM over one foot, other foot lifted 30 mm | hip/ankle roll torque |
| arm_lift | 1 kg per hand, shoulders to horizontal, elbows flex | arm sizing, CoM compensation |

Reference motions come from a damped-least-squares **whole-body IK**. It tracks both soles
(position + orientation), the whole-body CoM, pelvis height and pelvis attitude. Torque is
low-pass filtered over the actuator bandwidth (20 ms) before sizing.

```bash
python -m hx sim
```

## Balance stabilizer

The ZMP gait is planned on a linear inverted pendulum, but the real multi-body robot deviates from
it. Rendering the walking video showed that, with the final CAD masses, the **open-loop** gait
developed a growing lateral rocking mode: pelvis roll went ±0.5° → ±2.4° → 5.7°, and the robot
rolled over the foot edge at t = 7.6 s. The QA had passed it, because the stable result came from
the previous design-loop iteration's masses, 1 % lighter.

`hx/sim/stabilizer.py` now adds two feedback terms to both legs' references. The same code is
run by the brain on the robot, using the IMU and joint encoders:

| term | joint | law |
|---|---|---|
| attitude | hip roll | `+ 1.0·roll + 0.05·roll_rate` |
| CoM / ZMP (ankle strategy) | ankle roll, ankle pitch | `∓ (2.0·e_com + 0.3·e_com_dot)` |

Gains and signs were chosen by sweep:

* without the CoM term, walking fell at 0.95× torso mass;
* the opposite CoM sign fell immediately;
* 2× gain fell at 1.1×.

The selected gains keep maximum tilt at 0.4–2.1° from 0.9× to 1.1× torso mass. Two QA checks now
cover this. `SIM-8` re-runs the walk at ±10 % torso mass. The scenario checks also fail if a
scenario was simulated with masses that differ from the current CAD by more than 2 %.

## Videos

`python -m hx video` (or `make videos`) re-runs each scenario with a frame hook. It renders the
real CAD links as MuJoCo visual meshes in their body frames; physics is unchanged, because the
meshes are massless and non-colliding. Each frame it draws every cylinder (blind cap, barrel, rod
cap, chromed rod) between its live pin positions. A HUD shows per-cylinder load as `|τ| /
capacity(q)` in the direction of the torque.

Outputs go to `docs/video/<task>.mp4` (1280×720, 30 fps, H.264), `docs/video/hx1_tasks.mp4` (montage),
and `docs/video/gif/<task>.gif` (README previews). Tasks render in parallel, about 2.5 min on 4 cores
with software OpenGL (`MUJOCO_GL=egl`).

## Hydraulic software-in-the-loop

`hx/sim/sil.py` loads `firmware/build/libhx_sil.so` (the real `JointController`) and runs it at
1 kHz against a lumped plant:

* micro-servo spool (12 ms lag, slew limit), closed-centre overlap, orifice flow on 4 metering edges, laminar leakage;
* two compressible chambers (β_eff = 250 MPa incl. hose) with volumes depending on piston position;
* seal friction (Coulomb + viscous), joint inertia from the MuJoCo mass matrix, hard stops;
* 12-bit pressure ADC noise and quantisation, a 14-bit encoder, finite-difference velocity.

External torque reproduces the whole-body demand. The brain's feed-forward deliberately carries
**15 % model error**, so the feedback loops must do real work. Per joint type it runs:

1. a position step under the mean walking load (overshoot, settling, steady-state error);
2. walk and squat replays in impedance mode (RMS/max tracking, peak chamber pressure, no faults);
3. fault injection of encoder failure, pressure-sensor failure and supply loss (detected, latched, valve locked).

```bash
python -m hx sil
```

The SIL found three real problems during development:

* an unstable force loop (gains now derived from chamber stiffness);
* a proportional-only position loop stalling inside the spool overlap (integral added, deadband halved);
* valve flow saturation in knee swing (valve class raised to 6 L/min after a new QA flow-margin check;
  the HPU flow check likewise raised the pump from 0.6 to 1.5 cc/rev).
