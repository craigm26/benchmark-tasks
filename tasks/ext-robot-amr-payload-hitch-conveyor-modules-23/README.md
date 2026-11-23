# Drop a motorized roller's hex ends point-up into its frame slots without pulling its cable, then fit its torque plate

Building the roller-conveyor top module of a mobile robot (AMR) means fitting 24 V motorized drive rollers into the
module's side frame. A drop-in frame has an open-top slot in each rail; the roller's two hex shaft ends must go into
both slots at once, blind under the tube, and the hex only drops to its V seat when it is turned point-up. The
roller's power cable leaves through the motor-side shaft and is short: a pulled lead strains the seal and conductors
where it enters the shaft, so the roller is handled by its tube, never by or against its cable. Finally an anti-rotation (torque) plate slides down a guide on the rail and locks over the hex, which
takes the motor's reaction torque. The hard parts are physical: a 0.7 kg round part tethered by a short flexible
cable, two shaft ends aligned and clocked by feel into slots the hand cannot see, then a second part dropped over the
seated hex. The nearest accepted task, feeding a hex shaft through two bearings, slides a bare shaft along its own
axis; here a tethered 0.7 kg round part must have both shaft ends clocked and dropped blind into two slots at once
while its own cable stays slack, and then a second part is dropped over the seated hex.

## The parts (real size)

| Part | Size and material | Real product |
|---|---|---|
| Motorized drive roller | Zinc-plated steel tube, 38 mm OD x 225 mm, an 11 mm hex stub 27.5 mm long at each end (280 mm overall), 0.70 kg | Itoh Denki Power Moller 24, PM380LS (38 mm tube, 24 V DC, hexagon shaft standard). The hex size, stub length and mass are design values: measure the roller you buy and size the slots to it |
| Power cable | Black, 6 mm OD, 180 mm from the motor shaft to its connector, leaving the left shaft end along the axis | The roller's own lead |
| Driver card housing | Green box 60 x 22 x 32 mm on the left rail's outer face; the cable's connector (22 x 14 x 10 mm) is fixed beside it | Itoh Denki HB-510 driver card (a housing of the same footprint is enough for a trial) |
| Side rails | Two dark blue anodised aluminium plates 170 x 75 x 6 mm, inside faces 236 mm apart (between-frame width: tube + 11 mm, as the Itoh catalogue sizes it); open-top slot at mid-length, 13 mm wide, chamfered to 22 mm at the top, 120 deg V seat, apex 44 mm up, hex axis seated 50 mm up | `side_rail.stl` (fabricate.py), or cut from 6 mm plate; two 12 x 236 x 20 mm cross ties |
| Torque plate | SUS304 stainless, 34 x 70 x 4 mm, 69 g (7.93 g/cm3 x 8.77 cm3), a 13 mm wide notch at the bottom (its mouth flared to 18 mm over the lowest 5 mm) ending in a point-up 120 deg apex | Modelled on Ultimation's hex-point-up bracket MBB-081 for Itoh Denki rollers; `torque_plate.stl` (laser-cut from 4 mm SUS304) |
| Plate guide | Two posts 8 x 14 mm with 6 x 4 mm front lips (28 mm opening), 28 to 70 mm up, over a floor ledge; pocket 40 x 10 mm (3 mm a side on the plate) | `plate_guide.stl` |
| Roller V blocks | Two grey 90 deg V blocks, 60 x 16 x 28 mm (V apex 12 mm up), at y +-7.5 cm | `v_block.stl` |
| Plate rack | Grey slotted stand, 52 x 30 mm base, 30 mm walls, 6 mm slot | `plate_rack.stl` |

Layout (cm from the midpoint between the arm bases, x forward, y left): rails from x 21.5 to 38.5, slots at x 30;
roller on its V blocks at x 16.5, y 0, axis 3.9 up, hex stubs flat-up (a flat on top, 30 deg from point-up); cable
from the left shaft end (y +14.0) out to y 21.5, back along x to 23.5 and down to the connector at x 23.5, y 15 to 17;
torque plate upright in its rack at x 35.5, y 21.5.

## What the scripted demo does

Only the left arm moves; the right arm holds its ready pose. The left arm grasps the roller tube at its middle with
its pads across the tube and its wrist turned 30 deg about the roller's axis, lifts it off the V blocks and levels the
wrist, which turns the hex stubs point-up, and carries it over the rails at 12.5 cm (the cable spans at most 12.3 cm
of its 18.0 cm). Over the slots it servoes its measured tool pose (encoder forward kinematics) onto the target,
cancelling the sag the 0.7 kg roller puts on the wrist, including its roll about the roller's axis, then lowers both
hex ends into the slots together and presses until the arm lags its command (the hex on its V seats). It opens,
rises, picks the torque plate from its rack by its two 34 mm-apart edges (the pinch centres it in x), lowers it into the guide over the hex until it
stops, opens, and swings back to its ready pose. The demo reads only the fixed layout (object_jitter=0), its joint
encoders and gripper opening.

## Fidelity

Full size. The roller is one rigid body welded to the end of a 9-link cable chain (20 mm links, 6 mm, soft bend
springs) whose other end is anchored at the connector; the chain cannot stretch, so a pull shows as its span
reaching 95% of its length, which score() latches. No motor, bearing or roller surface turns. Slots, V seats and the
plate's notch are contact geometry with 1 mm clearance on the hex side flats; the V seat and notch flanks match the
hex flats, so a hex turned 30 deg off point-up rests 1 mm high and a plate over it 1 mm high (check.py). The
simulator scores the last step without opening the grippers, so a part held more than 1 cm above its seat counts as
not seated, and the last 0.25 needs both hands 3 cm clear of every part. The roller's 0.70 kg is a design mass (tube
shell 245 g plus stubs, motor and gearbox).

## Files

- `ext_robot_amr_payload_hitch_conveyor_modules_23.py`: the task file for `yam_arena` (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `ropekit.py`, `closedloop.py`, `belimits.py`: the shared kit (geometry, the cable, the closed-loop
  demo controller, geometry helpers), unchanged.
- `fabricate.py`: writes the STL files of the custom parts (`python3 fabricate.py out_dir`).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="Seat the roller's hex ends point-up in the frame slots without pulling its cable, then drop the torque plate over the hex."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/roller-dream

# Scripted demo (left arm only)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/roller-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the cable is never jittered.
- **Runtime per trial:** the demo's script runs about 40 s of simulated time (800 steps), then `success()` must hold
  8 s; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 210 s for the demo and about 680 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (score 1.0 with the plate and cable still),
  otherwise `score()` at the end of the run.

## Scoring

At the end, with both grippers open:

1. **Score 0** if the cable was ever pulled straight (its span from connector to shaft end over 95% of its 18.0 cm
   length for 0.25 s or more; judged from the video in a real trial), or it ends between the rails, in a slot or in
   the torque plate's pocket.
2. **0.25** when both hex stubs sit in their slots: axes within 1.5 mm of the seat height (50 mm up) and 2 mm of the
   slot centre line, each stub across its own rail (the tube ends allow 5.5 mm either way) and level within 3 deg.
3. **0.50** when also the hex is point-up within 5 deg (a digital angle gauge on an end flat reads 90 +- 5 deg).
4. **0.75** when also the torque plate is down in its guide over the hex: its notch apex within 1.5 mm of its seat,
   upright within 10 deg.
5. **1.0** when also both hands are 3 cm clear of the roller, cable and plate.

In a real trial the seat heights are judged with a 1.5 mm feeler gauge: it must not pass under either hex stub's
lower corner at its V seat, nor under the plate's notch apex on the hex; the slot centre line and clocking are read
with a steel rule and a digital angle gauge.

Doing nothing scores 0 (the roller stays on its V blocks). Seating the roller flat-up scores 0.25; a plate dropped
with no roller seated falls to its ledge, 12 mm under its seat, and scores nothing.

## Real-world build and reset

Bill of materials (a real trial needs no power; leave the roller unplugged from any supply):

| Item | Source | Price (estimate unless a listing is cited) |
|---|---|---|
| Motorized roller, Itoh Denki PM380LS (or any 24 V Itoh roller; the PM486FE, 48.6 mm, lists at $231 for a 17 in BF roller at ultimationinc.com) | Itoh Denki distributor, quote | about $230 to $300 |
| Cheaper inert route: 38 mm OD x 1.2 mm steel tube 225 mm, two 11 mm hex bar stubs pressed into printed end plugs, 6 mm two-core cable 18 cm, lead shot sealed inside to 0.70 kg | hardware store, metal supplier | about $30 |
| Side rails, plate guide, V blocks, rack (fabricate.py, PETG, about 150 g) | any printer | about $5 |
| Torque plate, 4 mm SUS304 laser-cut from `torque_plate.stl` | online laser-cut service | about $15 |
| Base board 40 x 40 cm, M4 screws, double-sided tape | hardware store | about $15 |

Total about $65 with the inert roller, about $270 with a real roller.

Assembly: screw both rails to the base board 236 mm apart (inside faces), slots opposite each other at x 30 cm, with
the two cross ties between them; screw the plate guide to the left rail's outer face, centred on its slot, ledge
28 mm up; fix the driver-card housing beside it and tape the cable connector to the board at x 23.5, y 15 to 17; tape
the V blocks and the plate rack down at their marks and tape the cable's start outline on the board.

Reset: lift the plate back into its rack and the roller back onto the V blocks with a flat on top of each stub, and
lay the cable along its outline. Nothing is consumed and nothing is wet; every force is under 10 N (a 7 N roller, a
0.7 N plate), well inside the YAM's 40 N per finger.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder; no third-party assets.
