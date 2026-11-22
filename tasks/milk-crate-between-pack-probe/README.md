# Take a milk delivery's temperature: flat probe between two packed cartons

A receiving check that school, hospital and restaurant kitchens do on every chilled delivery. A crate of half-pint
milk cartons arrives; the receiver slides a flat between-pack thermometer probe down the seam between two cartons
in the middle of the crate, holds it still for the reading, draws it out and puts the probe away. FDA Food Code
3-202.11 sets the limit (41 F, milk 45 F), and school receiving SOPs (Wisconsin DPI, "Receiving Deliveries") say
for "packaged products including cartons of milk, the thermometer is inserted between two items without
puncturing the wrapper". It is a listed O*NET task of food preparation workers (35-2021, about 0.9M US workers:
"Take and record temperature of food and food storage areas") and institution and cafeteria cooks (35-2012, about
0.4M: "Monitor and record food temperatures to ensure food safety"). The hard part is physical: the cartons are
loose and packed, so the 2 mm blade has to find the closed seam at the gable roofs, wedge it open by its own
thickness against the cartons' weight, reach a depth band with no floor to stop on, and stay still. Lifting or
tipping a carton (opening a gap by hand) scores 0.

## Parts (real sizes, as built in the scene)

| Part | Real product | Size and material |
|---|---|---|
| Milk crate | 16 qt square milk crate, 13 x 13 x 11 in (WebstaurantStore 227MC16GRN, green) | HDPE, 330 x 330 x 279 mm outside, 305 mm inside, walls 12.5 mm, floor 8 mm; fixed |
| Cartons, 50 | half-pint (8 fl oz, 236 mL) gable-top school milk cartons | coated paperboard, 60 x 60 mm base, 95 mm body, 20 mm gable roof (ridge and fin front to back, along x), 6 mm fin, 121 mm tall, 250 g filled; lower layer of 25 fixed, top layer of 25 free |
| Probe | ThermoWorks Between Pack Probe, PRB-K-060 (SKU THS-103-060), type K | stainless blade 127 x 3.8 x 2.0 mm (5 x 0.15 x 0.08 in) with a rounded tip, black handle 19 mm x 100 mm, 50 g |
| Probe holster | printed (fabricate.py) | PLA, 40 x 40 x 135 mm block, 12 mm round hole 130 mm deep with a 2 mm entry chamfer |

Top-layer cartons stand in 5 rows (x 13.8 to 38.2 cm) by 5 columns (y -12.2 to +12.2 cm). The crate is 305 mm
inside and five cartons take 300 mm, so the 5 mm of slack is shared out as a delivered crate settles: 0.8 mm at each
seam and at each wall, every seam narrower than the 2.0 mm blade. Carton shoulders are 22.4 cm and fin tops 25.0
cm above the table. The holster stands at x 27, y -23.5 cm, outside the crate on the right arm's side.

## What the scripted demo does and what it reads

The demo reads the live poses of the probe (to aim and depth its blade tip) and of the cartons (the seam's middle,
and any carton rising with the blade) from the simulator, besides the fixed layout (object_jitter=0), its joint
encoders and gripper opening. Right arm only (the left arm holds READY): a level
side grip (gripper pointing +x, pads closing across the handle 40 mm above its bottom, at the probe's centre of mass),
lift straight out of the holster to a carry height with the tip above the crate rim, move over the seam between the
centre carton and the one to its right (x 26.1, y -2.9 cm), where the two gable roofs form a V. The round handle can
hang a few mm off or turned in the pads, so the demo reads where the blade tip really is and moves the hand (at most
2 cm, and up to 14 deg of wrist yaw) until the tip is over the seam with the blade's flat faces toward the cartons.
It lowers the tip into the V, then feels it down: the command goes down 6 mm/s while the live tip is short of 42 mm
below the shoulders, never more than 34 mm ahead of the measured hand (that caps the push at a few newtons), steering
the tip to the middle of the seam. At depth it takes the push off (command back to the measured hand plus its
free-air sag) so the hand holds the probe still, and holds 20 s. It draws the blade out at 1 cm/s; if any carton
starts to rise with it (1.5 mm), it stops, lowers back and eases the blade 0.4 mm off that carton. It then aims the
tip over the holster's hole, lowers until the probe reads seated (retrying up to three times), takes the push off,
opens slowly, rises 13 cm straight up and returns home. The hand never goes below 34.9 cm over the crate (7 cm above
the rim) and never moves faster than 0.10 m/s.

## Fidelity

Cartons are rigid boxes with real gable roofs and a 2 mm shoulder chamfer. Real paperboard gives a little; here the
blade slides whole cartons apart within the crate's 5 mm row slack (0.8 mm per seam), and nothing squeezes back on
the blade. A rigid blade drawn out can still drag a neighbour up by friction, so the lift rule is live in the
simulation too. The
lower layer is one fixed block; the crate's grid walls are solid. There is no temperature: the reading is a 15 s
still hold at depth (ThermoWorks lists a time constant under 3 s in liquid and a full reading under 15 s). The
probe's cord and meter are not modelled. The simulator scores the last step without opening the grippers, so a
probe still held at the end never counts as returned (score 0.75 at most), and a probe held above the holster is
not in it.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.

## Files

- `milk_crate_between_pack_probe.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: geometry and joint kit, closed-loop demo runner, scoring helpers.
- `fabricate.py`: writes `probe_holster.stl` (ASCII, mm) from the scene's dimensions: `python3 fabricate.py out/`.
- `task.json`: the task's details.

## Run a trial

From this folder, on an instance set up by the simulation walkthrough:

```bash
INSTR="Take this milk delivery's temperature: slide the probe 3 to 7 cm down between two middle cartons, lifting or tipping none, hold it for the reading, then put it back."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=milk_crate_between_pack_probe.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/milk-dream

# Scripted demo (closed loop, right arm)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=milk_crate_between_pack_probe.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/milk-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; carton lift, tilt and shift are measured from the fixed
  layout. `-E control_hz=20` is required; the reading hold is timed in 20 Hz control steps.
- **Runtime per trial:** the demo takes about 1,760 steps (88 s simulated); a Dream-1 trial runs up to 3,600 steps
  (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 670 s for the demo and about 1060 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` (full score and both hands 10 cm clear of the probe) has held
  for 8 s, otherwise `score()` at the end of the run.

## Scoring

A **seam** is the gap between the facing walls of two cartons of the inner 3 x 3, with the tip at least 1 cm in from
their corners; the gaps at the crate walls, seams of the outer ring and a carton's own body never count. **Depth**
is how far the tip is below the carton shoulders (22.4 cm above the table).

1. **Score 0 for the run** if any carton's base rises 5 mm or more, or a carton tilts over 10 deg, or a carton's
   centre leaves the crate, for 3 control steps in a row (0.15 s).
2. Otherwise the best stage reached at any step:
   - **0.25:** tip in a seam, 1.5 cm or more deep.
   - **0.5:** tip in a seam 3 to 7 cm deep, probe within 15 deg of vertical.
   - **0.75:** reading taken: 0.5 held with the tip moving under 3 mm for 15 s.
3. **1.0:** after the reading, the probe back in its holster (blade in the hole, handle on the block, within 10 deg
   of vertical), released (no finger pad within 3 cm), still for the last 2 s, and every carton within 1 cm of its
   start.

Doing nothing scores 0. Offline checks cover the wall gap, an outer-ring seam, a punctured carton, a tip near the
corner, 2 versus 3 steps of a 6 mm lift, a 12 deg tip, a carton pulled out or slid over the wall, a 4.9 s and a
14.9 s hold, a probe left in the seam, held or dropped, and a leaning probe.

**Real trial:** mark tape bands 3 and 7 cm up the blade from the tip and film the seam from the side; the meter shows
the reading; a straightedge laid across the carton tops in a photo shows a lift of 5 mm. A stopwatch on the video
times the 15 s hold.

## Real-world build and cost

| Item | Source | Price |
|---|---|---|
| Between Pack Probe PRB-K-060 | thermoworks.com/products/prb-k-060 | $59.00 list |
| Type K meter, Therma K | thermoworks.com/products/therma-k | $129.00 |
| 16 qt square milk crate, 13 x 13 x 11 in | webstaurantstore.com 227MC16 series (minimum order 6, about $10.49 each) | about $63 |
| 50 half-pint milk cartons | any grocery or school supplier, emptied, refilled with 236 mL water, taped shut | about $40 |
| Holster | `fabricate.py`, about 90 g PLA | about $3 |
| Foam board spacers (5 mm), tape | craft store | about $8 |
| **Total** | | **about $302** |

Assembly: print the holster upright, 3 walls, 30 % infill, and stand it on the table at x 27, y -23.5 cm with
double-sided tape. Fill and seal the cartons, pack the lower layer of 25, then the top layer of 25 with the ridges
running front to back, set out with a 0.8 mm card strip in each seam and at each wall, then pull the strips; if
the cartons are narrower than 60 mm, stand foam-board spacers along the near and right walls first. Plug the probe into the meter and stand it in
the holster with the cord laid away from the crate. The push the blade needs (about 1.4 N to slide two 250 g
cartons aside on the wedge) and the 50 g probe are far under 40 N per finger.

## Real-world reset

Set the top-layer cartons back in their grid with the 0.8 mm card strips, then pull the strips; dry any carton
that sweated and replace any that leaks. Wipe the blade with a sanitising wipe and stand the probe in its holster.
Cartons and crate are reused.
