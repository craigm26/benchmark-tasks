# Seat a locking teapot lid: pitch its underside hook under the spout-side rim, then rock flat

Porcelain teapots use a rigid underside locking nose (German: *Haltenase*) on the lid skirt so the lid stays captive
when tipped forward to pour. Anyone who has made tea knows this catch: set the lid straight down level and the
protruding hook hits the top of the circular rim, leaving the lid cocked. Because the overall span across the skirt
and hook (75 mm) is 5 mm wider than the 70 mm mouth opening, the robot must steady the loose teapot by its handle,
infer the hidden hook's direction from the dark steam vent directly opposite it on the lid dome, rotate the lid to
aim the hook toward the spout, pitch it nose-down to dip and tuck the hook beneath the overhanging rim ledge, and
rock the rear edge flat onto the rim seat.

## Parts (all full scale)

- **Teapot** (loose, glossy white hard porcelain, 740 g, friction 0.45): modeled directly from the **Seltmann Weiden
  Trio White 1.25 L Porcelain Teapot** (Article No. `001.686297`, EAN `4003106642743`; Trio Highline decor variant
  EAN `4003106902717`; matching white replacement lid with Haltenase EAN `4052212082301`). Widely available US/UK
  equivalent: **Price & Kensington 6-Cup Gloss White Stoneware Teapot** (1.1 L, SKU `0056.719U`, ~235 x 145 x 140 mm,
  ~70 to 75 mm mouth with underside locking tab). Dimensions: 217 mm spout-to-handle span along y (spout along -y,
  C-handle along +y), 153 mm belly diameter, 109 mm body rim height above tray (143 mm total seated height with lid).
  Overhanging 5 mm thick gallery collar with 70 mm circular mouth bore (1 mm top lead-in chamfer; rim underside at
  104 mm above tray). Tapered spout (28 to 17 mm dia) extending to 110 mm along -y; C-handle grip bar 14 x 12 x 67 mm
  at +101 mm along +y (18.5 mm clear of belly). Starts on tray at x 27.0, y -1.5 cm.
- **Locking lid** (loose, glossy white hard porcelain, 115 g, friction 0.45): 81 mm cap diameter, 53 mm total height
  (19 mm skirt depth below flange, 34 mm hollow dome and 24 mm diameter knob above flange). Circular skirt 63 mm outer
  diameter (54 mm inner), leaving 3.5 mm radial fit clearance inside the 70 mm mouth. Underside locking hook
  15 mm wide x 22 mm long x 5 mm thick, coplanar with the skirt base so the lid sits flat on the tray, protruding
  12 mm past the skirt outer radius to 43.5 mm (75 mm span vs 70 mm mouth) with a 14 mm vertical slot below the cap
  flange to clear the 5 mm rim. Look-only 9 mm dark charcoal steam vent on the lid dome directly opposite the hook.
  Starts right-side up on the tray at x 27.0, y -19.5 cm, yaw +90 deg (hook pointing +x, steam vent facing -x).
- **Serving tray** (fixed, dark walnut wood, friction 0.6): base 220 x 460 x 10 mm centred at x 27.0, y -3.5 cm
  (top 10 mm above the table), with four 10 mm wide x 14 mm tall rim walls and taped start-position rings for pot
  and lid.

## What the scripted demo does and what it reads

Both arms work together in closed loop (`demo_script` in `seat_a_locking_teapot_lid_hook_under_rim.py`). The left arm
approaches the teapot's C-handle from +y at a 45 deg downward pitch and closes gently on the 14 mm grip bar to steady
the loose pot while the right arm hovers straight down over the lid knob, grasps it, confirms grip from its finger
gap (one retry 2.5 mm lower), and lifts to 18.5 cm above the table. While still clear of the spout, the right arm
rotates the lid -90 deg (aiming the hook toward -y) and pre-pitches 28 deg nose-down, then arcs forward of and above
the protruding spout (x 21.5 cm, z 19.2 cm above table) to poise over the mouth at 30 deg nose-down. It dips the
hook nose down into the 70 mm mouth until below the rim underside, slides -y to tuck the hook 3.8 mm under the
spout-side rim ledge, smoothly rocks the pitch from 28 deg to 0 deg down to 2.5 mm above the rim seat, opens the
fingers to let the lid settle flat, and rises clear. The left arm then opens and withdraws along +y/+z, and both arms
park at the ready pose with grippers open for 4 s so the final frames show the seated lid and both arms well clear.

The demo reads the live poses of the teapot and lid from the simulator (to locate the handle and knob at pickup,
measure the lid's actual offset in hand above the mouth, and trim the dip, tuck, and rock-flat trajectory in the
teapot's live frame), together with both arms' joint encoders and gripper opening. It never teleports objects or
edits the simulation.

## Fidelity

Every part is full size and real porcelain mass (modeled collider volumes give ~2.24 g/cm3 density for the 740 g pot
and 115 g lid). The circular mouth collar, hollow teapot belly, lid skirt, and hollow cap dome use 24-sided convex
prism colliders (`mechkit.py`) with 3.5 mm radial clearance between the 63 mm skirt and 70 mm mouth for reliable
PhysX seating. The teapot is dry and empty (no water, heat, or pouring), and the porcelain is modeled as rigid and
unbreakable (no fracture or chipping model) with uniform 0.45 friction and default contact offsets. Overall teapot
and lid dimensions match the Seltmann Weiden Trio 1.25 L specification (217 x 153 x 143 mm), while the modeled 70 mm
mouth bore and 12 mm underside locking nose (5 mm thick, 19 mm skirt depth) represent standard Haltenase locking-lid
geometry and may differ by a few millimetres from fired ceramic batch tolerances on an individual purchased pot.
Because the simulator scores the final step without opening the grippers, the scorer checks live finger pad clearance
(>= 3 cm) and airborne height (> 1 cm above every support) so any held part scores 0; offline replays without finger
sensors fall back to the airborne check.

## Run a trial

On a machine set up by the simulation walkthrough, from this folder:

```bash
INSTR="Put the lid on the teapot so its underside hook (opposite the dark vent) locks under the spout-side rim and sits flat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=seat_a_locking_teapot_lid_hook_under_rim.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir yam-eval/teapot-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=seat_a_locking_teapot_lid_hook_under_rim.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir yam-eval/teapot-demo
```

- **Seed and layout:** `-E object_jitter=0` is required to hold the fixed tray layout and start marks; no seed is
  needed.
- **Runtime per trial:** the demo finishes its motions in about 32 s of simulated time (~640 steps) and holds at the
  ready pose (~37 s / ~740 steps total; episode ends 8 s after full seated release); a Dream-1 trial runs up to 2,400
  steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 120 s for the demo and about 350 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` holds continuously for 8 s (`SUCCESS_HOLD_S = 8.0`), otherwise
  `score()` on the final step.

## Scoring

Evaluated in the teapot's live local frame (on a real bench: ruler, straightedge across rim, top photo, video, and an
optional gentle 45 deg pour-tilt check confirming the hook retains the lid):

1. **Whole-run violations (0.0 for the whole trial):**
   - Teapot tilted > 15 deg, lowest point < 5 mm above table, or base centre outside the tray xy footprint for 0.3 s.
   - Lid centre outside the tray xy footprint or lowest point < 5 mm above table (dropped off tray) for 0.3 s.
   - Lid speed > 0.9 m/s for 0.1 s (dropped or thrown).
2. **Release and rest requirement (0.0 if unmet at end):** both grippers >= 3 cm clear of both teapot and lid,
   neither part airborne (> 1 cm above every support), and both parts < 1 cm/s and < 0.5 rad/s for 1.0 s.
3. **Final outcome:**
   - **1.0:** lid upright (tilt <= 3 deg), flange underside within 2.5 mm of rim top (109 mm in pot frame), lid axis
     within 4 mm of teapot axis, hook tip >= 5 mm radially past the 35 mm inner mouth rim with hook top >= 1 mm below
     the 104 mm rim underside, hook heading within 25 deg of the spout (-y), and teapot upright (tilt <= 3 deg) on the
     tray within 3 cm of its start mark.
   - **0.5:** lid seated flat with hook engaged under the rim and teapot upright on mark, but hook heading misaligned
     > 25 deg from the spout.
   - **0.0:** anything else, including the lid cocked on the rim after a flat drop, upside down, left on the tray, or
     teapot displaced > 3 cm.

Additional documentation (not scored by `score()`): `ARM_KEEP_OUT` declares a 15 mm padded box around the protruding
porcelain spout (x 24.0 to 30.0 cm, y -14.1 to -5.5 cm, up to 13.75 cm above the table); the scripted demo routes
clear of it and keeps tool speeds below 0.15 m/s.

## Real-world build, parts list, and reset

- **Parts list and sources:**
  1. **Porcelain locking-lid teapot:** Seltmann Weiden Trio White 1.25 L Porcelain Teapot (Article No. `001.686297`,
     EAN `4003106642743`, ~$50; replacement lid EAN `4052212082301`), or Price & Kensington 6-Cup Gloss White
     Stoneware Teapot (SKU `0056.719U`, ~$30). Measure the purchased unit's exact fired mouth diameter if tuning
     scripted waypoints.
  2. **Optional 3D-printable calibration replica:** run `python3 fabricate.py` to export ASCII STLs (in mm) of the exact
     modeled teapot body (`teapot_body.stl`) and locking lid (`teapot_locking_lid.stl`) matching the simulated 70 mm
     mouth and 12 mm hook; ballast the printed pot to 740 g and lid to 115 g with sand/epoxy in the base and cap if
     used for non-fragile rehearsal.
  3. **Serving tray:** standard dark walnut wood serving tray, ~22 x 46 cm, 1 cm base thickness with low ~1.4 cm rims,
     taped or clamped flat to the station table at x 27.0, y -3.5 cm.
  4. **Tape markers:** low-tack masking tape for the teapot and lid start rings on the tray floor; optional small dark
     paint/tape dot around the lid steam vent if the commercial pot's steam pinhole is under 4 mm.
- **Assembly and safety:** secure the tray to the table with double-sided tape, mark the pot and lid start outlines,
  and run completely dry (no water or heat). Peak grip and insertion forces are well under 10 N per finger (holding a
  115 g lid and steadying a 740 g empty pot, far below the YAM's ~40 N per finger limit).
- **Reset and cleanliness:** tilt the lid ~30 deg toward the spout to unhook and lift it out, place it right-side up
  on its start ring at x 27.0, y -19.5 cm with the dark steam vent facing -x, and verify the empty teapot rests on
  its mark at x 27.0, y -1.5 cm with spout along -y. Wipe ceramic dust or finger-pad marks with a dry microfibre
  cloth between sessions; nothing is consumed.

## Files

- `seat_a_locking_teapot_lid_hook_under_rim.py`: task file for `yam_arena` (scene, `score()`, `success()`,
  `demo_script()`).
- `task.json`: task metadata.
- `fabricate.py`: optional standard-library script to export ASCII STL files of the modeled teapot and lid.
- `mechkit.py`, `closedloop.py`, `belimits.py`: procedural geometry, closed-loop demo, and scoring helpers.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
