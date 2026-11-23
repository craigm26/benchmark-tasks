# Move a trigger sprayer to the refill bottle: steer its curved dip tube down the neck, land the collar square

Everyone who buys a cleaner refill moves the trigger head from the empty spray bottle to the full one. The hard part
is the dip tube: it keeps the curve it was made with, so its tip hangs 20 mm off the head's axis. Centre the head over
the 20 mm neck and the tip lands on the shoulder. The robot must draw the tube out of the empty bottle (the curve drags
through its neck), carry the sprayer over the full bottle, aim the off-axis tip into the neck from 18 cm above it,
lead the head back over the neck as the curve passes through (the tube flexes and springs back inside), and leave the
28 mm collar sitting square on the neck finish, released. A kinked tube, a tube left outside the bottle or a knocked
bottle scores 0. Running the collar down its thread (the covered hold-and-turn ability) and spraying are not tested.

## Parts (real sizes and materials)

| Part | Real product | Size and material | In the scene |
|---|---|---|---|
| Full bottle | Impact Products 5016, 16 oz bottle, 28/400 finish | PET, 70 mm body, 190 mm to the neck top, 20 mm bore, 25 mm finish root, 33 mm support bead; 473 mL water, 0.50 kg | free, 0.50 kg, pale blue, x 27.0, y -3.5 |
| Empty bottle | the same bottle, empty | 35 g | free, pale grey, x 24.0, y -16.5 |
| Trigger sprayer | Impact Products 5816 trigger sprayer, 28/400, 7-1/4 in (184 mm) dip tube | PP head 40 g, closure 33 mm OD, 29 mm skirt bore, 8.5 mm skirt | rigid body: closure, stem 26 x 30 mm, shroud, trigger, nozzle; seal face on the empty bottle's neck, nozzle +x |
| Dip tube | part of the 5816 | LDPE, 6 mm OD x 4 mm ID, 184 mm, 2.7 g; factory curve: straight 100 mm under the collar, then R 170 mm toward +x for 84 mm (28 deg), tip 20 mm off the axis, 5 mm above the bottom | ropekit plastic rope, 9 segments |
| Drip tray | any 20 x 24 cm plastic tray with 12 mm rims | charcoal plastic, 4 mm floor | fixed, centred x 25.5, y -9.5 |

Positions are cm from the midpoint between the arm bases, x forward, y left. The 5016 bottle (16 oz, 28/400) and the
5816 trigger (7-1/4 in tube, made to pair with the 5016) are sold together as Zoro G0087477 and separately by
janitorial distributors. Confirm the bore and finish height on the bought bottle before the first trial.

## The scripted demo

Closed loop, `closedloop.py`, right arm only (the left arm holds READY; no arm crosses y = 0). The demo reads the dip
tube's measured tip position and the full bottle's pose from the simulator (two reads), and otherwise the fixed
layout (object_jitter=0), its joint encoders and gripper opening. Gripper level, pointing +x, pads across the stem's
30 mm sides at 32 mm above the seal face, centred in x on the measured centre of mass of sprayer plus tube.

1. Approaches from -x, trims wrist sag with `trim_to_measured`, closes on the stem and checks `holding()` (retries once).
2. Draws the sprayer straight up at about 0.024 m/s until the tube tip is 19 mm over the neck, and waits 1 s.
3. Carries it (peak 0.08 m/s) so the head is 20 mm off the full bottle's axis, with the curved tip over the neck;
   waits 1.5 s, trims, reads the tube tip and corrects the head by up to 15 mm so the tip is over the neck centre.
4. Feeds at 0.02 m/s: as the head goes down, it moves toward the neck axis so the tube point at the neck stays on
   the axis (head offset = R (1 - cos phi), sin phi = (h - 100 mm) / R). With 100 mm left the head is over the axis.
5. Lowers to 3 mm over the neck and presses (`press_until`, 2 to 6 mm, stops on tracking error) until the seal face
   stops on the neck top.
6. Opens, backs out 60 mm along -x, rises 10 cm and swings home over 8 s.

Every tool-point command stays under the declared 0.12 m/s limit (peak 0.10 m/s in the offline replay).

## Fidelity

Approximate. The dip tube is nine 20.4 mm rigid segments with elastic bend joints from beam theory (LDPE, E 0.25 GPa,
I 51 mm^4: 0.64 N m/rad per joint, yield 15.5 deg), perfectly plastic past the yield (no hardening); its laid curve is
the joints' zero. Straightening the curve through the neck needs about 7 deg per joint, inside the elastic range.
Bottles are rigid, opaque and 16-sided (inner flats at the nominal radius); the full bottle has no liquid, only its
mass. The 28/400 thread is look-only, so in simulation the seal face rests on the neck top, while a real unturned
collar rests 3 to 5 mm up on the thread start; the 0 to 6 mm seating band covers both. The simulator scores the last
step without opening the gripper, so a sprayer still held in the air or on the neck always scores 0. The tube is
coloured orange (real tubes are white or clear) so it shows at 320 x 176. The bottles are opaque in simulation, so
the camera cannot see the tube once it is inside.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Move the sprayer onto the full bottle: steer its curved dip tube all the way down the neck, then sit the collar on the neck."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=move_a_trigger_sprayer_to_the_refill_bottle_curved_dip_tube.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/sprayer-dream

# Scripted demo (closed loop, right arm only)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=move_a_trigger_sprayer_to_the_refill_bottle_curved_dip_tube.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1400 --store-frames --log-dir ~/yam-eval/sprayer-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the rope is never jittered and the bottle limits are measured
  from the marked positions.
- **Runtime per trial:** the demo runs about 48 s of simulated time (about 950 steps) plus the 8 s success hold; a
  Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 200 s for the demo and about 670 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (`SUCCESS_HOLD_S = 8.0`), otherwise
  `score()` at the end of the run.

## Scoring

L is the top of the full bottle's neck (the land). Measured after the gripper opens.

1. **Score 0** if, at any time for 0.5 s, either bottle tilts over 15 deg, slides over 30 mm or lifts over 10 mm;
   if the tube has a new permanent bend over 10 deg at any point (real: it no longer lies in the 1:1 template); or if
   the sprayer is still held (a finger pad within 30 mm of the sprayer or tube) or still moving (over 1 cm/s).
2. **1.0:** tube tip 160 mm or more below L, every point of the tube inside the full bottle, and the collar seated:
   seal face 0 to 6 mm above L (simulation: -1 to +6 mm), its centre within 2.5 mm of the neck axis, tilted 5 deg or
   less (real: no visible gap under one side of the collar).
3. **0.5:** the tube that deep and inside, but the collar not seated (on the thread crest, tilted or off centre).
4. **0.25:** tip 60 mm or more below L with the sprayer resting on the full bottle (seal face within 40 mm above L
   and 20 mm of its axis).
5. **0** otherwise: sprayer left in the empty bottle, tube outside the full bottle, sprayer dropped.

Real trial: a steel ruler read through the clear bottle gives the tip depth, an angle card or the printed seat gauge
the collar, the 1:1 template the bend, a video the bottle limits. The collar is placed by a light press (under 5 N);
nothing in the task needs more than a few newtons per finger.

## Real-world build and reset

Parts list (prices approximate, from distributor listings): Impact Products 5016 bottle with 5816 trigger, 2 sets
(about $3 each, Zoro G0087477); one plastic drip tray 20 x 24 cm (about $8); 473 mL tap water; printed parts from
`fabricate.py` (tube template, two bottle locator rings, seat gauge; about 40 g PLA, about $2). Total about $20.

Assembly: tape the tray to the table at the marked spot, tape a locator ring under each bottle position, fill one
bottle with 473 mL water, unscrew the sprayer and stand it in the empty bottle with the nozzle pointing +x, its tube
curve toward +x. Check the tube lies in the template first; a tube with a different curve is replaced.

Reset: lift the sprayer out of the full bottle, check its tube in the template, stand it back in the empty bottle,
stand both bottles in their rings and top the full one up to 473 mL. The tray catches drips; wipe the tube and tray
dry between trials. The robot only places the collar; nobody screws it down during trials.

## Files

- `move_a_trigger_sprayer_to_the_refill_bottle_curved_dip_tube.py`: the task (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `ropekit.py`, `closedloop.py`, `belimits.py`: the shared kit, unchanged.
- `fabricate.py`: writes the printed parts' STL files when run as a script.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder; no third-party assets.
