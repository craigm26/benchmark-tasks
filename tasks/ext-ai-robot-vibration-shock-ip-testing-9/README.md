# Tilt-slide a quick-acting knurled knob down a stud, square to engage threads, and snug

AI hardware reliability engineers and qualification technicians clamp edge compute boxes and GPU carrier modules onto
vibration and HALT shaker subplates using slotted aluminum crossbars, vertical M8 studs, and quick-acting tilt
knobs (Carr Lane CLM-8-QKK on a Qualmark HALT Fixture Kit 750-0169 / 750-0170 assembly). Pushing a quick-acting knob
straight down an M8 stud jams its internal half-thread lugs on the top thread crests; the operator must hold an
off-axis tilt (~15 to 18 degrees) to slide the relieved bore freely down 55 mm of exposed stud, square the knob flat
against the aluminum crossbar so the half-threads catch, and turn clockwise at least half a turn to snug it.

## Parts, dimensions, and materials

All coordinates are in the station frame (table top `TOP = 0.7475 m`; `+x` forward, `+y` left, `+z` up):

- **T-slot subplate and edge compute module (`plate`, fixed):** 6061-T6 aluminum subplate, 160 x 200 x 12 mm, at
  `(0.260, 0.0, TOP)` carrying a dark-anodized extruded aluminum edge compute enclosure (90 x 103 x 35 mm, modeled
  after a Hammond 1455-series housing) with silver heatsink stripes and a green PCB edge indicator.
- **Crossbar and M8x1.25 stud (`crossbar`, fixed base):** 6061-T6 aluminum crossbar, 25 x 180 x 12 mm (top at
  `TOP + 0.059 m`), spanning `y = -0.090..+0.090 m` on two 35 mm aluminum spacer sleeves at `y = +0.075` and
  `y = -0.075 m`. The left stud (`+0.075 m`) is pre-clamped by a hex nut; the active right M8x1.25 steel stud at
  `(0.260, -0.075 m)` rises 55 mm above the crossbar to `TOP + 0.114 m` (5.0 mm core diameter with 8.0 mm
  thread-crest collars).
- **Staging block (`stage`, fixed):** anodized blue aluminum block, 44 x 44 x 12 mm, at `(0.200, -0.150, TOP)` with
  an 18 mm OD x 10 mm pedestal (`STAGE_TOP = TOP + 0.022 m`) and a 4.2 mm diameter, 8 mm tall brass staging peg.
- **Quick-acting knurled knob (`knob`, free, 28 g, friction 0.6):** modeled after the **Carr Lane CLM-8-QKK** metric
  quick-acting knurled knob (M8x1.25, 22 mm OD; 16 mm nominal height, modeled at 18 mm total height with 18.0 mm outer
  bore, opposing half-thread lugs at 3.25 mm inner radial offset, 12.8 mm Y-guide bore width, and a gold top index tab
  on the `-x` tilt-relief side; 12L14 black-oxide steel, ~3.02 cm^3 solid volume at 7.87 g/cm^3 -> ~24 g, modeled at
  28 g). Starts upright on the staging peg at `(0.200, -0.150, TOP + 0.031 m)`.

## Scripted demo and state reads

The right arm approaches the staging peg, grasps the 22 mm black-oxide knob across its OD (TCP 8.0 mm above knob
center so pad tips clear the pedestal and crossbar by 4.0 mm), lifts clear, tilts +19 degrees off axis high above the
M8 stud tip, trims wrist sag, centers the knob directly over the stud tip, slides straight down along the 55 mm exposed
stud to the crossbar face under continuous closed-loop sag tracking while holding the 19 degree tilt, pivots smoothly
to 0 degrees (square) against the crossbar, rotates clockwise >= 0.55 turns in two wrist passes (re-indexing at 39.4 mm
finger gap to stay within joint6 limits), opens the gripper, lifts clear above the stud tip, returns to `READY`, and
settles for 2.5 s (left arm holds `READY` throughout).

The demo reads the live simulator position of the knob once hovered above the stud tip to zero out any in-finger grasp
offset, alongside its joint encoders and gripper opening.

## Fidelity note

M8x1.25 thread crests are approximated as discrete 8.0 mm cylindrical collars and the knob's relieved half-threads as
internal opposing lugs, so holding 19 degrees off-axis tilt clears the collars (> 1.2 mm radial gap) while upright
pushing jams. Axial helical thread draw and clamping preload are not contact-resolved: tightening is scored from
cumulative clockwise turns while seated flat on the crossbar, and all bodies are rigid. The simulator scores the final
step without opening the grippers; `released()` checks >= 30 mm finger-pad clearance so a held knob never scores
above 0.75.

## Run a trial

```bash
INSTR="Tilt and slide the quick acting knob down the stud, square it on the crossbar, and snug it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ext_ai_robot_vibration_shock_ip_testing_9.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/vib-knob-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ext_ai_robot_vibration_shock_ip_testing_9.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 800 --store-frames --log-dir ~/yam-eval/vib-knob-demo
```

- **Seed and layout:** `-E object_jitter=0` is required so the staging peg, stud axis, and crossbar sit at the exact
  nominal coordinates.
- **Runtime per trial:** the scripted demo finishes in 32.8 s simulated time (656 steps at 20 Hz).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 130 s for the demo and about 320 s for a full Dream-1 trial.

## Scoring

At the end of the run (grippers opened >= 3 cm from the knob):

1. **Score 0.0** if a sustained latch (>= 3 steps / 0.15 s) fires for **tip/drop** (knob COM `z < TOP - 0.05 m` or
   tilt > 30 deg) or **straight-push jam** (knob within 1.4 cm of stud axis, bottom within 0.6 cm of stud top
   `TOP + 0.114 m`, and tilt <= 5 deg).
2. **0.25** if the knob is on the stud (radial offset <= 0.6 cm) and slid down at least 3.0 cm (`bottom <= TOP + 0.084 m`,
   i.e., `83.15 cm`) without tipping, but not yet squared on the crossbar.
3. **0.50** if seated flat on the crossbar (`|bottom - CROSSBAR_TOP| <= 0.002 m`, tilt <= 3 deg, radial offset
   <= 0.6 cm) with fewer than 0.5 cumulative clockwise turns while seated.
4. **0.75** if seated with >= 0.5 cumulative clockwise turns while seated, but fingers are still within 3 cm of the knob
   or the knob is still moving.
5. **1.00** if snugged (>= 0.5 clockwise turns while seated), released (`>= 3 cm` clear of both grippers), and still
   (linear speed `< 2 mm/s`) for the last 1.5 s (30 steps).

## Real-world build, parts list, and reset

All required actions stay well below 40 N per finger: gripping a 35 g steel knob requires ~1 to 2 N normal force,
tilted sliding has nominal bore clearance (< 2 N friction), and snugging half a turn against dry aluminum requires
< 5 N tangential finger force.

### Bill of materials (low-cost reproducible build)

| Item | Commercial product / source | Qty | Est. price (USD) |
| --- | --- | --- | --- |
| Quick-acting knurled knob (M8x1.25) | Carr Lane `CLM-8-QKK` (All Industrial / Geisler Co.) | 1 | $10.59 |
| M8x1.25 threaded rod (zinc-plated steel, 1 m, cut to 102 mm & 70 mm) | Bolt Depot / McMaster-Carr | 1 | $5.50 |
| Aluminum crossbar (6061-T6 flat bar, 25 x 12 x 180 mm, 9 mm bore at y +-75 mm) | McMaster-Carr (or printed via `fabricate.py`) | 1 | $6.00 |
| Aluminum T-slot subplate (160 x 200 x 12 mm) + 35 mm spacer sleeves & M8 hex nut | McMaster-Carr / local shop (or printed base via `fabricate.py`) | 1 set | $28.00 |
| Edge compute enclosure (103 mm wide extruded aluminum, trimmed to 90 mm, ballasted to 450 g) | Hammond `1455L1201` / `1455K1201` series | 1 | $23.05 |
| Staging block + 4.2 mm brass dowel pin (or printed via `fabricate.py`) | Hardware store / PLA print | 1 | $4.00 |
| **Total** | | | **~$77.14** |

An optional higher-cost industrial Qualmark HALT starter kit (`750-0169` SAE / `750-0170` Metric) provides the same
slotted crossbar and stud hardware if already available in an environmental test lab. Run `python3 fabricate.py` to
export millimeter-scale ASCII STLs (`knob_replica.stl`, `crossbar.stl`, `staging_block.stl`, `subplate.stl`) for the
printed fixture parts.

### Assembly and real-world reset

1. Bolt the subplate to the table at `(0.260, 0.0)` and mount both vertical M8 studs at `y = +0.075` (70 mm tall) and
   `y = -0.075` (102 mm tall) with 35 mm spacer sleeves. Place the ballasted enclosure centered on the subplate, set the
   crossbar over both studs, and snug the `+0.075 m` stud with an M8 hex nut.
2. Fasten the staging block at `(0.200, -0.150)` with its 10 mm pedestal and 4.2 mm upright brass pin.
3. **Between trials:** tilt the Carr Lane knob ~15 to 18 degrees off axis to slide it off the right M8 stud, wipe any
   dust from the stud crests and crossbar face with a dry cloth, and seat the knob upright on the staging peg with its
   gold index mark facing `-x` (toward the robot bases).

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
