# Open a flush push-to-open EV charge port door past 90 degrees and flip down its orange DC pin flap

Every driver of an electric car at a DC fast charger does this: the charge port door on the fender is flush and has
no handle, so you press its edge in, it pops ajar on its push-push latch, you take it by its narrow popped edge
(without pushing it back shut, which latches it again), swing it wide, then reach into the recessed pocket and fold
down the orange cover over the CCS1 DC pins. A robot that can do it can plug a car in. Unlike opening a handled
cabinet door or closing one that is already open, the door here gives nothing to grip until it has been pressed and
released, the popped edge stands only about 30 mm proud, and the second step is a small flap pinched inside a
pocket under a rigid shroud.

## Parts (full scale)

Positions are in cm from the midpoint between the arm bases (x forward, y left), heights above the table top.

| Part | Real product, size and material | Position |
|---|---|---|
| Charge port door | GM 42676631 (2017-2021 Chevrolet Bolt EV charge port door): painted ABS, 17.0 x 17.5 cm, 6 mm thick with a dark inner face, 180 g, no handle, gooseneck hinge on its left edge, opens 100 degrees toward the robot | Shut and flush in the fender opening, outer face x 35.5, y -20.1 to -3.1, z 1.0 to 18.5; hinge axis x 35.1, y -2.6 |
| Pocket with push-push latch | GM 42780310 (Bolt EV charge port housing with the push-to-open latch): matte black moulded pocket 4.6 cm deep; its latch plunger (15 mm cap) sits behind the door's right part. About 7 N pressed 5 mm past flush releases it; the plunger then pops the door about 12 degrees (35 mm at its edge) ajar | Plunger at y -18.0, z 9.75 |
| CCS1 inlet | GM 24062486 (Bolt EV CCS1 combo inlet), unwired; or a salvage 2017-2021 Bolt EV charge port assembly with the harness cut off (about US$100 to 180 used): 82 x 118 mm black flange, silver J1772 AC shroud 43.8 mm across and 14 mm deep, two 8 mm copper DC pins 32 mm apart in white rings | Flange face 3.8 cm behind the fender face, centred y -12.1; AC shroud centre z 11.7; DC pins z 6.0 |
| DC pin flap | GM 24290558 (Bolt EV DC terminal cover): safety-orange plastic, 60 wide x 40 tall x 5 mm, 25 g, hinged at its bottom edge, clicks shut (upright) and open (folded 100 degrees down), 22 mm wide x 12 mm tall pull tab standing 16 mm proud along its top edge | Hinge line x 38.9, z 4.0; tab z 6.8 to 8.0 |
| Fender mock-up | 14 mm MDF painted glacial blue, 25.6 wide x 19.8 tall, with a 17.6 x 18.1 cm opening (the door sits in a 3 mm reveal); steel foot plate 9 x 25.6 cm clamped to the table | Face at x 35.5, y -24.4 to +1.2 |

All dimensions are the scene's; the door, pocket, inlet and flap are taken from the Bolt EV parts above.

## What the scripted demo does

Only the right arm moves; the left arm holds its ready pose. The demo reads the live outer_door, plunger and dc_flap
joint values from the simulator (to know when the press has armed the latch, how far the door popped and how far
the flap has turned), plus its joint encoders and gripper opening; it never moves an object or writes a joint.

1. With closed fingers, level and leaned 10 degrees down, it presses the door's right part in over the plunger (up to
   16 mm past the face, stopping once the door reads +2.6 degrees), holds 0.2 s and backs off; the latch pops the
   door about 12 degrees ajar.
2. It turns the gripper straight down with the pads opened 16 mm and closing across the door, comes in along the
   door's plane from beyond its trailing edge 10 mm below the door's top, so the rear pad slips into the 29 mm gap
   behind the popped edge without pushing the door, and pinches the door 15 mm in from its edge (retrying once).
3. It swings the door round its hinge to 98 degrees over 7 s, the pads turning with it, opens the pads and slides
   them off past the door's edge.
4. Level and leaned 10 degrees down again, pads open 50 mm across the orange pull tab on the flap's top edge (the
   finger blades pass 2.5 mm under the AC shroud), it squeezes the tab's sides lightly (pads 3 mm under its width, so
   the tab can turn between them) and draws it round the flap hinge to 80 degrees, its hand following the arc of
   the tab's root so the flap below swings away from the fingertips. If the flap lags behind the detent's 3 N peak
   it leads it on (up to 110 degrees commanded) until the flap reads past 70 degrees, then lets go; the detent spring
   folds the flap onto its open stop.
5. It backs out of the pocket to the side of the fender panel and parks at its ready pose with the gripper open,
   10 cm or more from the door and flap. Tool speeds stay under 0.10 m/s.

## Fidelity

Approximate. The push-push latch (a heart-shaped cam track in the real pocket) is modelled by a real sprung plunger
(350 N/m, 14 N s/m) that touches the door, plus a scorer-side state machine: pressed past +2.0 degrees with the plunger
in 4 mm, the door's hinge friction drops from 0.90 to 0.04 N m; when the door eases back under +1.4 degrees the
plunger's spring rest moves 31 mm out, so the plunger pushes the door about 12 degrees ajar. Pushed back past +2.0
degrees it re-latches. The hinge friction (hold) values are not yet confirmed in Isaac Sim, so no score depends on
their exact value. The DC flap's catch is a two-position detent spring (0.14 N m/rad, flipping at its 50 degree
midpoint) resting on rigid stops. The door and flap pass through the fixed fender and pocket (a joint body never
touches its own base), so their travel is set by joint stops. Inlet pins and AC pins are look-only. The simulator
scores the final step without opening the grippers, while a real trial opens them first; any finger pad within 3 cm
of the door or the flap scores 0, so a door or flap still held never scores.

## Run a trial

From this folder, on an instance set up by the simulation walkthrough:

```bash
INSTR="Open the flush EV charge port door past 90 degrees, then flip down the inner orange DC pin flap and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_ev_charge_port_door_and_dc_flap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/ev-port-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_ev_charge_port_door_and_dc_flap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/ev-port-demo
```

- **Seed and layout:** `-E object_jitter=0` keeps the fixed charge port mock-up at its layout coordinates.
- **Runtime per trial:** the demo's motion takes about 50 s of simulated time (about 1,000 steps) and then holds
  still at its ready pose; `success()` also waits for both hands to be 10 cm clear, and with `SUCCESS_HOLD_S = 8.0`
  the episode ends about 8 s later, about 58 s (1,160 steps) in all. A Dream-1 trial runs up to
  2,400 steps (2 minutes at 20 Hz).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 200 s for the demo and about 360 s for a full Dream-1 trial.
- **Score:** `SCORE_OVER = "end"`; `task_score` reports 1.0 when `success()` (a full score) holds for 8 s, otherwise
  `score()` at the end of the run.

## Scoring

Checked every step; the final state counts. In a real trial, open the grippers, then judge from an overhead photo
with a protractor at each hinge (or the printed hinge protractor below) and the trial video. Let phi be the door angle
(0 shut, -100 at its open stop) and alpha the flap angle (0 shut, -100 folded down).

1. **Score 0** if the door or flap is forced past a stop for 2 steps or more (real: bent or broken); if the door was
   opened without first being pressed in so its latch popped it at least 6 degrees ajar (18 mm at its edge; real:
   pried open past the latch); or unless both hands have been 3 cm clear of the door and the flap, and both have been
   still (under 2 degrees per second), for the last 1.0 s.
2. **Otherwise:** 1.0 if phi <= -90 and alpha <= -85; 0.65 if phi <= -90 and alpha <= -45, or phi <= -75 and
   alpha <= -85; 0.45 if phi <= -90 (flap left up); 0.2 if phi <= -6 (popped, or partly opened); else 0 (shut or
   pushed back until it re-latched).

Possible scores: 0, 0.2, 0.45, 0.65, 1.0. Doing nothing scores 0.

## Real-world build and reset

- **Parts list:** the four Bolt EV parts above (new, or one salvage charge port assembly that includes the door,
  pocket, inlet and flap; unwired); one 600 x 600 mm sheet of 14 mm MDF, cut to 256 x 198 mm with a 176 x 181 mm
  opening, painted blue; a steel or plywood foot plate 90 x 256 x 8 mm; four wood screws and two C-clamps.
- **Bill of materials (approximate US retail, checked against seller listings before buying):** one salvage
  2017-2021 Bolt EV charge port assembly with door, pocket, CCS1 inlet and DC flap, harness cut off, about US$100 to
  180 used (the new GM parts above cost more); 14 mm MDF sheet about US$15; spray paint about US$8; foot plate,
  screws and two C-clamps about US$20; printed frame and protractor (about 150 g of PLA) about US$5. Total about
  US$150 to 230.
- **Print-ready files:** `python3 fabricate.py` writes `stl/pocket_mount_frame.stl` (a 4 mm frame screwed behind the
  opening, with two ears that clamp the pocket's flange so the shut door sits flush in a 3 mm reveal) and
  `stl/hinge_protractor.stl` (a 3 mm scoring plate with posts at 0, -6, -75, -90 and -100 degrees about the door
  hinge). Nothing is written on import.
- **Assembly:** screw the frame behind the MDF opening, clamp the pocket in it, check the shut door sits flush with a
  3 mm reveal, fit the inlet and its flap in the pocket as on the car, screw the panel upright onto the foot plate and
  clamp it to the table with the panel face at x 35.5 cm and the opening's right edge at y -20.4 cm.
- **Forces:** pressing the door to release the latch takes about 7 N; the door swings with under 0.5 N; the flap
  needs about 3 N at its tab. All are far under the YAM's roughly 40 N per finger.
- **Reset (under 10 s, nothing consumed):** fold the flap up until it clicks shut, swing the door shut and press it once
  until it latches flush. The inlet is unwired, nothing is wet or sharp, and the parts only need dusting.

## Files

- `open_ev_charge_port_door_and_dc_flap.py`: the task file (scene, latch, `score()`, `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (procedural geometry and joints, the closed-loop demo
  controller, geometry helpers).
- `fabricate.py`: STL writer for the two custom printed parts.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
