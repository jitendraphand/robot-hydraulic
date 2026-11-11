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
