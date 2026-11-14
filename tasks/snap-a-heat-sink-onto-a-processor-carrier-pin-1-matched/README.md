# Snap a heat sink onto a processor carrier in its tray, pin 1 marks matched, until all four corner clips catch

A server processor is never put in its socket bare: the technician first builds a processor heat sink module. The
processor sits in its plastic carrier in the shipping tray, the heat sink is lowered onto it with the pin 1 triangles
at the same corner, and the corners are pressed until the carrier's four clips snap over the heat sink base. Server
service manuals give it this way (Lenovo ThinkSystem SR650 V2, "Install a processor and heat sink"; Dell PowerEdge
R750, "Installing the processor"), and it is done on every processor install or replacement. Unlike
a light lid pressed into a fixed case, the part is 0.45 kg and gripped with 8 mm to spare a side, the quarter turn that
matches pin 1 cannot be undone once the clips catch, the carrier is loose and must stay in its tray, and four clips
only catch under an even press at the corners.

## Parts (real sizes)

- **Shipping tray:** blue-grey moulded plastic, fixed, 147 x 100 x 24 mm, pocket 127 x 70 mm with a rim 6 mm tall.
- **Processor in its carrier:** loose in the pocket, 0.13 kg in all. Black plastic frame 109 x 64 x 7 mm, green
  substrate 77.5 x 56.5 mm, silver lid 64 x 50 mm, its top 9 mm above the pocket floor (the seat). Play in the pocket:
  3 mm a side across, 4 mm (clips) along.
- **Four corner clips:** dark plastic arms 12 x 5 x 20 mm, 20 g each, at the frame's ends, 24 mm either side of the
  centre line, hinged at the foot, sprung shut against a stop (0.45 N m/rad), opening outward up to 0.40 rad. Each nose
  reaches 3.5 mm inward: 0.5 mm of gap to the heat sink base end, then 3.0 mm over it. Catch face 16 mm above the pocket
  floor (1 mm above the seated base top), 55 degree ramp on top.
- **Heat sink:** 0.45 kg, 108 x 78 x 25 mm: copper base 108 x 78 x 6 mm, aluminium fin stack 92 x 78 x 19 mm (13 fins
  along the long side), notched 14 x 9 mm at each corner round a steel captive nut 9 mm across that stands 2 mm above
  the fins. White label with a gold 20 mm pin 1 triangle. Seated, the base overhangs the pocket rim by 4 mm, 3 mm above it.
- **Stand:** dark grey block 90 x 120 x 30 mm, fixed, holding the heat sink a quarter turn away from the carrier.
- Gold pin 1 triangles: 20 mm on the heat sink label, 14 mm on the processor substrate, 12 mm on the tray rim.

Resting on the four ramps the base is about 12 mm above the lid. By wedge arithmetic (ramp 55 degrees, lever 16 mm) the
heat sink's weight opens each clip 0.3 to 0.4 mm of the 1.25 mm a pass needs, and a press of 4 to 6 N per clip
(friction 0.3 to 0.5), 11 to 19 N from the two hands on top of the weight, takes it through. Computed, not measured.

## What the scripted demo does

Both arms, closed loop (`demo_script` in the task file, run by the standard `yam_scripted` policy). The left hand grips
the heat sink across its 78 mm of fins (jaws open 94 mm), lifts it, turns it a quarter turn counterclockwise (seen from
above) while carrying it over the tray, lines its centre and direction up with the carrier, lowers it until the base is
1 mm above the clip ramps, sets it on them and lets go. Then both hands, fingers closed and leaned 20 degrees outward,
press down on the corner nuts, one diagonal pair of corners and then the other, until those clips catch; any corner
not caught is pressed again, up to twice. Both arms then park at the ready pose with the grippers open.

The demo reads the live poses of the heat sink and the carrier and the four clip angles from the simulator, and its own
joint encoders and gripper opening. It never moves an object or writes a joint.

## Fidelity

Approximate. Sizes and masses are real. The clips are rigid arms on sprung hinges with a stop, standing in for moulded
flexing clips; the carrier and processor are one rigid body. The fins are drawn; the fin stack collides as one block
notched at the corners. Density is uniform over the colliders, so the heat sink's centre of mass is higher than a
copper-based part's. No thermal paste and no force limit on the processor lid. The simulator scores the last step
without opening the grippers, so a heat sink still held or in the air scores 0, as it would once released. About 46
shapes, 25 of them colliding.

## Files

- `snap_a_heat_sink_onto_a_processor_carrier_pin_1_matched.py`: scene, `score()`, `success()`, `demo_script()`.
- `mechkit.py`, `closedloop.py`, `belimits.py`: shared helpers (multi-part objects and joints, closed-loop demo, limits).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="Fit the heat sink onto the processor carrier in the tray, gold triangles together, and press till all four clips catch."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=snap_a_heat_sink_onto_a_processor_carrier_pin_1_matched.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/hsm-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=snap_a_heat_sink_onto_a_processor_carrier_pin_1_matched.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/hsm-demo
```

- **Layout:** `-E object_jitter=0` is required (fixed layout). No seeds.
- **Runtime per trial:** the demo takes about 45 to 60 s of simulated time (900 to 1,200 steps, longer when a press
  needs its full travel) of the 3,600 steps allowed
  (3 minutes at 20 Hz). Wall-clock time on one L40S, including Isaac Sim start-up: about 220 s for the demo; the Dream-1 run ended after about 30 s with no score and no recording (see the Dream-1 run result below).

**Dream-1 run result:** no score and no video. Dream-1's first action was out of the allowed range, so the policy adapter rejected it and the episode ended before the robot moved. The error was `PolicyError: q_r_chunk gripper leaves 0..1: [-0.113, 0.938]`, on all 3 attempts on 2026-11-14. The scripted demo scores 1.0 and shows the task can be done.
- **Score:** `task_score` reports 1.0 once `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

Measured after the run with both grippers at least 3 cm from the heat sink and carrier and everything still for 1 s.

- **Matched:** the heat sink within 5 degrees of the carrier's direction with the gold triangles at the same corner.
- **Caught (per clip):** the clip upright on its stop (within 0.05 rad), its nose over the copper base, and the base
  underside there within 1 mm of the processor lid.
- **Score, if matched:** 0.25 per clip caught. 1.0 needs all four and the heat sink flat on the lid within 1.5 degrees
  (otherwise 0.75).
- **Score 0:** not matched or reversed; no clip caught (heat sink on the stand, resting on the ramps, held up, or off
  the carrier); a gripper still on a part; or, at any time in the run: a clip caught a reversed heat sink; the carrier
  was lifted 5 mm off the pocket floor or tilted 10 degrees; the heat sink was let go more than 15 mm above the lid or
  tilted over 15 degrees.

The robot's hands stay out of the tray pocket (`ARM_KEEP_OUT`): they work on the heat sink only.

## Real-world reset

Open the four clips by hand, lift the heat sink off and put it back on the stand with its triangle at the far left;
centre the carrier in the pocket. Check the clips for wear every few trials.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
