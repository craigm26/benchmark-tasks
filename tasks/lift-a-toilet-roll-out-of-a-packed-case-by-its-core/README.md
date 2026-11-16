# Lift a toilet roll out of a packed case by its core

Janitors and cleaners restock toilet paper from a case on every restroom round (O*NET 37-2011: service, clean and
supply restrooms; 2.2 million US workers). A fresh case is packed wall to wall, so no finger gets down the side of a
roll without crushing paper, and people hook their fingers into the cardboard core of the first roll and draw it
straight up. This gripper has the same problem twice over: a 114 mm roll is 20 mm wider than its 94 mm opening. So
the robot must put its closed fingers about 2 cm into the 42 mm core, open them until their backs bear on the core wall,
lift the roll square out from between two walls and two touching rolls, carry it hanging on two contact patches and
stand it in the cart bin, leaving the other three standing in the case. A neighbour that rises or rocks on the way and
settles back standing costs nothing; a neighbour left lying, leaning or propped up lowers the score, and a second
roll taken out scores 0.

This is not the packed-shelf book task: that one makes an outside grasp by tipping the book out first. Here there is
no pre-grasp and no outside grasp at all; the grip is an expansion inside a bore.

## Parts (real size, real shape)

Positions are in cm from the midpoint between the arm bases, x forward, y to the robot's left.

- **Four toilet rolls**, loose: standard institutional 2-ply rolls, 4.5 x 4 in, each 11.4 diameter x 10.2 tall, 150 g,
  white, wound on a brown cardboard core 4.2 inside diameter. Built as hollow cylinders (a 24-sided tube collider, a
  smooth visual), not blocks. They stand on end at x 27.5 and 39.0, y -18.0 and -6.5, 1 mm apart and 1 mm from the
  walls, so every roll touches two walls and two rolls, as the first roll out of any freshly opened case does. The
  demo takes roll_a (x 27.5, y -18.0), the one nearest the right arm; the scorer accepts any one roll.
- **Case**, kraft single-wall corrugated board, 4 mm, taped down: a 2 x 2 corner cut from a one-layer case (such
  rolls ship 24 to 96 to a case, in layers of the same wall-to-wall pattern). Outside 23.9 x 23.9 x 10.7 tall, inside
  23.1 x 23.1 x 10.3 deep, outer faces at x 21.3 to 45.2 and y -24.2 to -0.3. The rim stands 1 mm over the roll
  tops. The four top flaps are folded down flat outside (look only).
- **Bin** (a cart caddy), charcoal grey plastic, taped down: outside 15.8 x 15.8 x 6.4 tall, 4 mm walls and floor,
  inside 15.0 x 15.0 (1.8 cm a side round a standing roll), centred at x 10.4, y -10.0, 3 cm from the case on the
  robot's side.
- **Wrapping.** Institutional cases ship with rolls individually wrapped or unwrapped. This is an unwrapped case;
  wrappers are not modelled.

## What the scripted demo does

Right arm only, about 32 s (636 steps in the offline dry run); the left arm stays at its ready pose.

1. **Approach.** Gripper pointing down and leaned 9 degrees about the pad line (the lean lifts the arm's reach ceiling from 22 to 26 cm), fingers closed, pads opening along y, to 3 cm above the rim over
   roll_a's core; it then lines up on the core's axis.
2. **Enter.** Straight down at 2 cm/s until the fingertips are 19 mm inside the core (if the encoders show the hand held up short, resting on the roll, it backs off 1.5 mm). The closed pair is 14.1 mm in
   circumradius there, 6.9 mm a side inside the 21 mm bore.
3. **Expand.** The fingers open at 1 cm/s until the encoder gap stops 11 mm short of the command, which means their backs bear on the core wall (22 mm on the gripper
   encoder in the contact replay). The hand then moves until the roll is back on the line it stood on, so it presses on no neighbour. The grasp is accepted when the fingers bear and the roll
   ends under 3 mm sideways and 2 mm up; otherwise the hand closes, comes out, lines up again and tries once more.
4. **Lift.** Straight up at 2.5 cm/s until the roll's foot reads 12 mm over the rim, holding the roll's axis on its
   start line and slowing if it tilts more than 3 degrees. If the foot cannot be brought 4 mm
   over the rim the hand lowers the roll back into its place and lets go instead of dragging it over the wall.
5. **Carry.** At 9 cm/s over the case's near wall, holding the roll's foot 12 mm over the rim by its read pose until the whole roll is past the wall, then down to 2 cm over the bin's rim on the way to its centre; 1 s for the swing to die.
6. **Set down.** Down at 3 cm/s until the roll's foot reads 3 mm above the bin floor.
7. **Release.** The fingers close (the roll drops 3 mm), rise straight out of the core at 4 cm/s and the arm parks
   at its ready pose, over 25 cm from the bin. The last frames show one white roll standing in the charcoal bin, three
   rolls in the case and both arms clear.

The demo reads roll_a's live pose from the simulator (its core axis, its tilt and the height of its foot) to line up
on the core, to stop the lift and to stop the set-down; everything else comes from its own joint encoders and gripper
opening. It never moves, teleports or edits an object.

In the contact replay (MuJoCo, the same arm and finger collision hulls) only the two fingertips touched roll_a, no
part of the arm touched anything else, the three other rolls stayed where they stood (under 0.5 mm), and the run scored 1.0. The
tool point is never commanded faster than 0.10 m/s (the task's declared limit is 0.15 m/s).

## Fidelity

Rigid bodies. Each roll is a rigid 24-sided hollow cylinder with friction 0.6; a real roll gives a little and real
rolls stand touching, slightly squeezed, so real drag on a neighbour is if anything larger than the simulated 1 mm
gap gives. Rolls are unwrapped. The case and the bin are fixed, as if taped to the table. The case is one layer deep
because the arm reaches only 22 cm above the table pointing straight down and 26 cm leaned 9 degrees: the roll's
foot clears the 10.7 cm rim by 12 mm at the top of the lift. The simulator scores the last step without opening the grippers, so a roll still
held, in the air or within 3 cm of a gripper scores 0. Speeds are read frame to frame at 20 Hz, as from video. A rigid
roll cannot dent or unwind, so the simulator reads a displaced roll (N) from its final pose alone; the dent and
unwound-paper limits apply to real trials.

## Files

- `lift_a_toilet_roll_out_of_a_packed_case_by_its_core.py`: the task file for the `yam_arena` embodiment (scene,
  `score()`, `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (multi-part objects and geometry, the closed-loop
  demo controller, footprint checks), unchanged.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take one toilet roll out of the case and stand it upright in the grey bin. Leave the other rolls in the case."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lift_a_toilet_roll_out_of_a_packed_case_by_its_core.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/roll-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lift_a_toilet_roll_out_of_a_packed_case_by_its_core.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/roll-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the rolls are packed to 1 mm, and the demo's lift has no room for a
  shifted roll.
- **Runtime per trial:** the demo needs about 32 s of simulated time (636 steps); a Dream-1 trial runs up to 2,400
  steps (2 minutes simulated), about 3.3 times the demo. Wall-clock time on one L40S, including Isaac Sim start-up: about 130 s for the demo and about 370 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (the roll released and standing still in the
  bin), otherwise `score()` at the end of the run.

## Scoring

Read at the end of the run, from the last frames of the video (overhead and a side view) with a tape and a
protractor. Both grippers must be 3 cm or more from every roll and every roll still (under 5 mm/s for 2 s);
otherwise the score is 0. Only D looks at the whole run.

- **Out of the case:** a roll whose centre, seen from above, is outside the case's inner walls. Exactly one roll must
  be out. None out (nothing taken, or the roll put back) scores 0; two or more out (a second roll removed) scores 0.
- **R:** the one roll that is out.
- **N, a roll left displaced in the case:** any of the three rolls still in the case ends more than 15 degrees from
  upright (lying or leaning), or with its lowest point more than 6 mm above the case floor (propped on a wall, on
  another roll or on its own edge). N lowers the score one row: 1.0 to 0.5, 0.5 to 0.25, 0.25 to 0. A roll that
  slid sideways into the emptied place and stands on the case floor is not displaced: loose rolls in an opened case
  shift, and nothing is harmed. In a real trial N also covers a damaged roll: a dent deeper than 5 mm (straightedge
  across the dent and a ruler) or more than 10 cm of paper unwound on any roll left in the case.
- **D, dropped or flung:** the R roll's centre ever faster than 0.5 m/s, which is 2.5 cm between two frames of 20 fps
  video. The demo's 3 mm release reaches about 0.24 m/s; a fall from 2 cm or more does not pass.

| Score | Outcome for the R roll (before N) |
|---|---|
| 1.0 | in the bin (centre inside the inner walls, lowest point within 6 mm of the bin floor), within 15 degrees of upright, no D |
| 0.5 | in the bin lying or leaning over, or in the bin after D |
| 0.25 | at rest on the bare table (lowest point within 6 mm of it, centre outside the case and the bin): taken out, not delivered |
| 0 | none or two or more rolls out of the case, still held, in the air, perched on the case wall or the bin rim, off the table |

What the other rolls do on the way is not scored. A neighbour that rises 6 mm with the roll, rocks 11 degrees, or
is lifted and set back, and comes to rest standing on the case floor, leaves the score as it is: the rolls are
unharmed and the case is as a person would leave it. Lifting one roll clear, standing it back and taking another is
scored the same way, by where every roll ends. The penalty is for what is left behind: a second roll out of the case
(0), or a roll left lying, leaning, propped up or damaged in it (one row lower).

A roll put back in the case scores 0, not 0.25: nothing was restocked. Pushing or sweeping cannot take a roll out
(the walls are as tall as the rolls). In the demo's contact replay the three neighbours moved under 0.5 mm and
roll_a's own peak tilt was 0.9 degrees. The offline checker scores about 40 synthetic runs covering each row, the D
latch, N at each row and the borderline cases (a neighbour up 6 mm or over 11 degrees on the way, ending standing:
1.0; a neighbour left leaning 14 degrees: 1.0, 20 degrees: 0.5; left 5 mm off the case floor: 1.0, 8 mm: 0.5; 14
degrees in the bin is upright, 20 is not).

## Real-world reset

Stand the roll back in its corner of the case with its core vertical and square the four rolls: a card slips between
any two, a finger does not. Empty the bin. The case and the bin stay taped down. Replace a roll whose core is crushed
or whose paper has started to unwind.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
