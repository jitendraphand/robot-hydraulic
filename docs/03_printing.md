# 03 · Printing on the Elegoo Neptune 4 Max

Machine limits used by the design (`printer` in robot.yaml): build volume **420 × 420 × 480 mm**,
nozzle ≤ **300 °C**, bed ≤ **85 °C**, 0.4 mm nozzle. Every part is checked to fit with an 8 mm edge
margin (QA `CAD-1`). The generated [print plan](../cad/print_plan.md) lists each part with its
material, profile, orientation, mass and time.

## Materials

| use | material | notes |
|---|---|---|
| loaded links, cylinder caps, pistons, hips, feet | **PA12-CF** | dry 8 h at 80 °C, print from a dry box; **hardened-steel nozzle required** (fit one compatible with the Neptune 4 hotend); bed 80 °C on textured PEI + glue; enclosure strongly recommended |
| torso, covers, enclosures, saddles, clips | **PETG** | stock brass nozzle is fine |
| foot pads | **TPU 95A** | slow (≤ 40 mm/s), direct drive |
| fit-check prototypes only | PLA | never for loaded or wetted parts |

If you cannot print PA12-CF, PETG works for every *structural* link at reduced safety factor.
Pressure parts must then be PETG at 100 % infill and **must still pass the proof test**.

## Profiles (OrcaSlicer / Elegoo Cura)

| profile | layer | perimeters | top/bottom | infill | used for |
|---|---|---|---|---|---|
| structural | 0.20 | 5 | 6 | 35 % gyroid | links |
| pressure | 0.16 | 8 | 10 | 100 % rectilinear | caps, pistons, coupon |
| standard | 0.20 | 3 | 4 | 20 % gyroid | torso, enclosures |
| flexible | 0.20 | 3 | 4 | 25 % | TPU pads |

Orientation is already baked into the STLs, which are laid flat on the bed. Rules used:

* Links are printed **on their side (+y up)**, so layers lie in the bending plane of the link and
  fork cheeks are not split by layer lines.
* Cylinder caps are printed with the bore axis vertical so glands and spigots are round. The rod
  cap prints with its outer face down.
* Pistons are printed flat; ream the O-ring groove floor if it is rough.

## Calibration first: `fit_coupon.stl`

Print the coupon in every material, then record results in `qa/build_log.yaml`:

1. 608 bearing seats at Ø22.05 / 22.15 / 22.25. Choose the one that needs firm thumb pressure,
   then set `joint_hardware.bearing_fit_d` and regenerate (`make all`).
2. M3 / M4 heat-set insert holes. M5 port insert holes (Ø6.3/6.4/6.5).
3. Ø8.0 / 8.2 / 8.4 bolt holes (8.4 = clearance, 8.0 = tight tongue fit).

## Plates

`cad/plates/<material>_plateNN.stl` are pre-packed beds (6 mm spacing, 8 mm edge margin).
Load a plate as one object, or load the individual STLs from `cad/stl/` if you prefer your
slicer's arrange tool. Quantities in the plan include left/right copies.

## Post-processing

* Install heat-set inserts at 245 °C (PA12-CF) / 220 °C (PETG) with a flat tip, perpendicular.
* Ream the bearing seats only if the coupon showed it is needed. Bearings must not spin in their seat.
* For cylinder caps, lightly sand the spigot O-ring groove with 600 grit and clean with IPA.
  Apply O-ring grease before assembly.
