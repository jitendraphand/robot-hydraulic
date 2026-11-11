# 04 · Assembly

Work bottom-up and **record every step in `qa/build_log.yaml`**. The QA roll-up prevents
signing off an assembly whose sub-assemblies have not passed. Part names below match
`cad/stl/`, and quantities and cut lengths are in [`cad/bom.csv`](../cad/bom.csv).

## Tools

Heat-set insert iron, 8 mm reamer, M3/M4/M8 hex keys, torque driver (0.5–10 N m), O-ring pick,
PTFE tape is **not** used (M5 ports seal with bonded washers/O-rings). You also need a hand pump
or regulated test rig to 26 bar with gauge, a safety shield, nitrile gloves and safety glasses.

## 1. Cylinders (×14)

1. Cut the barrel, rod and tie rods to the BOM lengths. Deburr and chamfer the barrel ends 0.5 mm × 15°.
2. Turn the rod ends: piston thread on one end (M6 for 10/12 mm rods, M5 for 8 mm), M8 for the rod end on the other.
3. Insert the M5 brass inserts into both caps' radial ports (245 °C, flush).
4. Fit the O-rings. Piston: groove O-ring (lubricated). Caps: spigot static O-rings, two rod seals and a wiper.
5. Piston on rod: thread-locker, then torque the nut to 3 N m (M5) or 5 N m (M6).
6. Stack: blind cap → barrel → piston/rod → barrel → rod cap. Fit the 4 tie rods and tighten the nyloc nuts in a cross pattern to **0.8 N m** (printed caps creep under higher preload).
7. Press the GE8E bearing into the blind lug. Screw the rod end onto the rod with a jam nut.
8. **Proof test** (docs/08). Do not continue with a cylinder that has not passed.
9. Clamp the `valve_saddle` 25 mm from the blind cap. Mount the spool valve and its micro servo, and connect A/B to the cap ports with short PA12 hoses.

## 2. Joints

Each revolute joint is a **fork (parent) + tongue (child)**:

```
 bolt head ─ shim ─ [608 in cheek] ─ shim ─ tongue ─ shim ─ [608 in cheek] ─ shim ─ nut + magnet cap
```

1. Press a 608-2RS into each fork cheek (outer face flush).
2. Slide the tongue between the cheeks with 8×14×1 shims each side. Pass the M8 bolt through and clamp with a nyloc nut at 6 N m. The bolt rotates with the tongue, so the bearings carry the joint.
3. Press `encoder_magnet_cap` over the nut and glue a 6×2.5 mm diametric magnet in its pocket.
4. Fit the AS5048A board on `encoder_bracket` using the M3 inserts on the cheek outer face (1.5–2 mm air gap).
5. Hip yaw: press the 6808 into the pelvis seat, insert the `hip_yaw_link` hub, and screw the HV-150 servo horn through the hub (4× M3).
6. Hip roll / ankle roll use the same fork + tongue arrangement, with the roll axis along x.

## 3. Cylinders onto links

Pin the blind lug into the parent link's lug pair and the rod end into the child link's lug pair
with M8 × 45 bolts and nylocs (snug, then back off 1/8 turn; the spherical bearings take the
misalignment). Cycle each joint through its full range **by hand with the valves open to tank**
and check that nothing binds. The assembly check verified clearance at every pose, so binding
means a print or assembly error.

## 4. Legs, pelvis, torso

1. Pelvis: both HV-150 servos into their pockets (ears on the ledge, 4× M3 each).
2. Legs: hip_yaw_link → hip_roll_link → thigh → shin → ankle_link → foot, plus the TPU foot pads (glued over the 4 FSRs).
3. Torso onto the pelvis with 6× M4 into inserts. Run the hydraulic P/T lines and the harness through the central pass-through.
4. Arms: shoulder link on the torso fork, servos, upper arm, forearm, hand.
5. Neck and head.

## 5. HPU (torso)

Lower shelf: battery tray (2× 6S). Middle shelf: BLDC motor + gear pump with a printed coupling
guard, accumulator and reservoir. Upper shelf: hub node, power distribution, Raspberry Pi.
Plumbing: suction strainer → pump → check valve → accumulator tee → P manifold. The relief valve,
NO dump valve and return filter go back to the reservoir. Keep the suction line short and rising.

## 6. Plumbing

* Fill each line, then bleed: run the HPU at 5 bar and cycle every joint slowly 10× with its bleed screw cracked.
* All hoses get `hose_clip`s at ≤ 120 mm spacing. A hose must never rub a moving part.
  Check this at the squat and stride poses.

## 7. Wiring

Follow [05_electronics](05_electronics.md) and the generated harness table. Fit the CAN
terminations (jumper `JP1`) **only** on the two end nodes of each bus.
