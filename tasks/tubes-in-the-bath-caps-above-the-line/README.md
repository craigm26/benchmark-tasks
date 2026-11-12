# Hold floating sample tubes down in a water bath, caps above the water line

Lab technicians warm sample tubes in a water bath. A part-filled tube is lighter than the water it pushes aside, so it
floats: it rides up out of the rack, leans, and its cap ends up in the water. The fix is a hold-down plate that sits on
the caps and keeps the tubes on their seats, where the sample is under water and the cap is above it.

Here a steel bath stands in front of the arms with a tube rack inside it. Four orange tubes (20 g each, light enough to
float) wait on the bench, and a white hold-down plate (180 g) lies beside the bath. The robot has to stand each tube in a
rack opening, where it floats 2.2 cm above its seat, and then lower the plate over the rack's centre post onto the four
caps. The plate's weight pushes all four tubes down and keeps them there.

What makes this different from loading a rigid rack: the load pushes back. A tube let go in its opening does not stay
where it was put, the four caps the plate lands on give way under it, and the job is not done until the plate is on its
post holding all four down.

## Files

- `tubes_in_the_bath_caps_above_the_line.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`: multi-part objects and sprung joints for task files (the rack with its water discs, the plate).
- `closedloop.py`: lets the scripted demo read the arms' own joint encoders while it runs.
- `belimits.py`: geometry helpers the task file imports (outline inside an area, tilt, top and bottom heights).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Load the four sample tubes into the water bath rack and fit the hold-down plate over them."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tubes_in_the_bath_caps_above_the_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/tub-dream

# Scripted demo (full score), one arm at a time. The left arm stands its two tubes in the left openings and parks;
# the right arm stands its two in the right openings and parks; then the left arm lowers the plate over the post
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tubes_in_the_bath_caps_above_the_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/tub-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it, and
  the demo's grasps are for those positions.
- **Runtime per trial:** the demo's plan is 1742 steps (87 s of simulated time) before any trimming; with the trims it
  ran about 1,750 steps in an offline contact replay, and each trim may take up to 1.5 s longer. The run ends 6 s after
  the plate is on, when `success()` has held that long. A Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- **Score:** `task_score` reports 1.0 when `success()` holds for 6 s (all four tubes held down under the plate,
  everything still), otherwise `score()` at the end of the run. The 6 s hold keeps the run going after the plate is
  let go, so the video shows the gripper open, rise and park.
- **What the demo reads:** the fixed layout in the task file and the arms' own joint encoders, which it uses to trim
  the hand onto its target before each grasp and release. It reads no simulator object state.
- **Both arms work, one at a time:** the left arm places two tubes and the plate, the right arm places two tubes.

Wall-clock time on one L40S, including Isaac Sim start-up: about 440 s for the demo and about 660 s for a full Dream-1 trial.

## Scoring

At the end of the run, open both grippers, let everything settle, and measure heights from the table top with a ruler.
The bath rim is 7 cm above the table, so a height can also be read from the rim.

1. **Score 0** if any tube lies in the bath (tilted more than 45 degrees with any part of it, seen from above, inside
   the walls: its cap is in the water), or a tube or the plate has left the table.
2. **Otherwise each tube earns 0.125** if it stands in a rack opening: wholly inside the 5 cm opening seen from above,
   its underside below the grid (its top under 13 cm), tilted 10 degrees or less (high and low edges of its top within
   0.5 cm; 0.5 cm is 3 cm x sin 10 degrees), and still.
3. **It earns 0.125 more** if it is also held down: its top is 9.4 cm or lower (2.4 cm above the rim; a seated tube's
   top is at 9.0 cm) and the plate is on the post, with the post through its hole, within 10 degrees of level (opposite
   edges within 2 cm in height) and still.

Possible scores: 0 to 1.0 in steps of 0.125. Doing nothing scores 0.

| Outcome | Score |
| --- | --- |
| Four tubes standing in the rack, floating (tops at 11.2 cm), plate still on the bench | 0.5 |
| Two tubes in the rack, two on the bench, no plate | 0.25 |
| Four tubes in the rack, plate on the post holding all four down | 1.0 |
| Three tubes in the rack with the plate on the post holding them down, one tube on the bench | 0.75 |
| Plate on the post, no tubes in the rack | 0 |
| A tube dropped flat into the bath, whatever else is done | 0 |
| A tube standing on the grid between openings, or lying on the bench | nothing for that tube |
| Plate laid on the caps beside the post, or leaning on the post more than 10 degrees | no hold-down credit |

The bath's 7 cm walls stop a tube being pushed in along the table: it has to be lifted over them. A tube pushed down
by a gripper without the plate earns no hold-down credit, and floats back up when the gripper opens.

In simulation the checks use each object's actual pose. The simulator cannot open the grippers before scoring, so a
tube still held more than 1 cm above the disc under it counts as not placed, and the plate counts as on the post only
once its underside is at least 3 mm below the post's top. The arms' path is not scored; only where the tubes and the
plate end up.

## How the water is simulated

The simulator has no fluids. The push of water on an upright tube grows in proportion to how deep its underside is
below the surface: the weight of the water it displaces, which for a 3 cm tube is 6.9 N per metre of depth. The task
file gives each rack opening a blue disc on a vertical spring of exactly that stiffness, resting at the water line, with
damping standing in for the water's drag. A 20 g tube stood on a disc sinks 2.8 cm and stays there, floating with its
underside 3.2 cm above the table and its top at 11.2 cm, as it would in water 5 cm deep over its seat. Pushed down to
its seat (the disc's bottom stop, 1 cm above the table) a tube pushes back with 0.15 N; four of them lift with 0.6 N,
which the 180 g plate (1.8 N) outweighs.

What this leaves out: a floating tube leaning against the side of its opening, splashing, and the water level rising
about 0.6 cm (to 6.6 cm, below the 7 cm rim) when all four tubes are pushed down.

## Real-world reset

Mark the four tube spots (x 12 cm; y +15, +6, -6, -15) and the plate's spot (x 25, y +21) on the table before the first
trial. Fill each tube with water to a total of 20 g (about 10 mL in a 10 g tube) and cap it. Fill the bath to the blue
line with room-temperature tap water with the rack in and no tubes. Between trials, lift the plate off, dry the tubes
and the plate, stand them back on their spots, and top the water up to the line. The bath stays clamped to the table
and the rack stays fixed in the bath.

Real trials: tap water only, room temperature; keep cables and electronics clear of the bath; the operator stops the run
if any part of an arm other than the finger pads reaches the water, and wipes spills before the next trial. A stopped
run is scored as it stands after both grippers open. The simulation does not show or score this; in the demo the finger
pads stay 2.4 cm or more above the water line.
