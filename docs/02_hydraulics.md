# 02 · Hydraulics

## Why low pressure (15 bar)

Printed cylinder caps, pistons and valve saddles are only trustworthy at modest pressure. At
15 bar supply / 17 bar relief, a tie-rod construction puts all hoop stress into the aluminium
barrel; the printed caps see only face pressure (≤ 2.2 kN on a 40 mm bore), carried by four
stainless M3 tie rods. Larger bores compensate for the low pressure. The sizing loop picks the
smallest bore that meets the torque demand.

## Circuit

```
            ┌──────────── relief 17 bar ───────────┐
 tank ─ strainer ─ gear pump ─ check ─┬─ accumulator (0.5 L, 11 bar N2)
  ▲          (BLDC, 1.5 cc/rev)        ├─ P sensor (hub)
  │                                    ├─ NO dump valve ─────────────────► tank   (opens on e-stop/power loss)
  │                                    └─ P manifold ──► 14 × [4/3 closed-centre spool valve ─ A/B ─ cylinder]
  └──── return filter 25 µm ◄──── T manifold ◄───────────────────────────┘          ▲  2 × pressure sensor per joint
```

* **Fluid**: ISO VG 15 mineral hydraulic oil with NBR seals. PETG and PA12-CF are oil compatible.
  Do not use PLA for anything wetted.
* **Pressure chain** (checked by QA `HYD-5`): supply 15 < relief 17 < software trip 18.5 < sensor
  range 30 bar; proof 25.5 bar ≤ valve (30) and hose (40) ratings.
* **HPU control** (hub node): pressure band 13.5–15 bar with PI-controlled pump speed. It unloads
  above the band, primes for 3 s with the dump valve open, and faults on over-pressure,
  over-temperature, low oil level or sensor failure.

## Valve trade study

| option | per joint mass | proportional | lock at rest | verdict |
|---|---|---|---|---|
| 4 × 2/2 solenoid "H-bridge" (independent metering) | ~0.40 kg | PWM only (10–20 ms valves) | yes | rejected: 5.6 kg of valves |
| servo-actuated miniature 4/3 spool (RC-hydraulics class) | ~0.07 kg + saddle | yes | yes (closed centre) | **chosen** |
| per-joint pump (EHA) | ~0.3 kg | yes | needs brake | rejected: 14 pumps |

The spool overlap (closed centre) is compensated in firmware (`SpoolValve::map`). The micro-servo
lag (12 ms) and slew limit are modelled in the SIL. The SIL showed that a 3 L/min valve
**saturates during knee swing**. A 4.5 L/min valve then failed the QA flow margin (`HYD-3`, 1.2 × peak
joint flow), so the design specifies the **6 L/min class**.

## HPU sizing (found by the QA loop)

The HPU check integrates the summed cylinder flow of every scenario against pump flow plus
accumulator gas volume. Walking is the worst case: about 4.5 L/min mean and 13 L/min peak.

| pump | accumulator | min supply while walking | verdict |
|---|---|---|---|
| 0.6 cc/rev (3.1 L/min) | 0.32 L / 9 bar | 9.0 bar | fail |
| 1.2 cc/rev (6.1 L/min) | 0.5 L / 11 bar | 12.5 bar | fail |
| **1.5 cc/rev (7.7 L/min)** | **0.5 L / 11 bar** | **13.4 bar**, 58 % duty | **pass** (≥ 13.0) |

A larger accumulator barely helps, because the gas-volume swing inside a 1.5 bar band is small.
Pump flow is what matters.

## Cylinder construction (printed tie-rod)

```
 GE8E ─┐   blind cap (PA12-CF, 100 %)      honed Al barrel        rod cap (PA12-CF, 100 %)    rod end M8
  lug ─┤▐█▌[O-ring]═══════════[piston O-ring]══════════════[O-ring]▐█▌[2× rod seal][wiper]═══rod═══○
       └── M5 port (brass insert)            4 × M3 A4 tie rods          M5 port ──┘
```

* Dead length (pin-to-pin minus stroke) = 115 mm drives the linkage geometry.
* Glands: static O-rings on the cap spigots, a dynamic O-ring on the piston, two rod seals, a
  wiper, and a leak-off groove between the rod seals.
* **Every cylinder is proof tested** at 1.5 × relief before installation (procedure in docs/08).

## Sizing method (`hx/hydraulics.py`)

For each hydraulic joint the optimiser chooses the parent/child pin locations inside the
configured bounds, the stroke and the smallest catalogue bore so that:

1. the physical cylinder (dead length + stroke) covers the geometric pin distance over the whole range;
2. capacity ≥ 1.5 × demand at **every** simulated (angle, torque) sample (pull and push areas differ);
3. capacity everywhere in the range ≥ 35 % of peak demand, and the moment arm ≥ 12 mm with no sign change;
4. the barrel clears the joint hardware and link beams.

Differential evolution runs in parallel per (joint, bore). Hysteresis prevents bore limit-cycles
in the design loop. The results are in [00_design_summary.md](00_design_summary.md).

## Maintenance

* Check oil level and filter ΔP every 5 h of operation; change oil every 50 h or when dark.
* Inspect rod seals (leak-off groove) before every session; any external drop is a fail.
* Re-proof-test any cylinder after reprinting or disassembling a cap.
