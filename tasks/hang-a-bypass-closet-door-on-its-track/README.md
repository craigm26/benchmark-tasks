# Hang a bypass closet door on its track and run it shut

A white closet door stands on its shoes on the table in front of the left arm. Behind it is a bypass door track: a dark grey
head track overhead (a plate on two end posts, with a front lip and a rear lip) and a thin orange guide
rib on the table. The door has a dark shoe (two cheeks) at each end of its bottom and a blue handle on its front face.
The robot must get the door's top up into the head track, swing the bottom in so the shoes straddle the guide rib, let
the door down onto the rib, then run the hung door along the track until its edge meets the jamb (the end post on the
left). Set straight down, the shoes land on the rib and the door falls out. A door that is only leaned against the
track scores little.

## Files

- `hang_a_bypass_closet_door_on_its_track.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, the closed-loop `demo_script`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers for multi-part objects, geometry checks and the
  closed-loop scripted demo, kept beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Hang the white closet door on its track and slide it shut."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_a_bypass_closet_door_on_its_track.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/door-dream

# Scripted demo (full score): the left arm lifts the door, tilts its top into the track, swings the
# foot in over the rib, lowers it, and runs it shut by the handle; the right arm stays parked
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_a_bypass_closet_door_on_its_track.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/door-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it.
- **Runtime per trial:** the demo takes about 70 s of simulated time (about 1,400 steps); a Dream-1 trial runs up to 3,600 steps
  (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 190 s for the demo and about 440 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds (hung, shut and still), otherwise `score()` at the end of
  the run.

## Scoring

At the end of the run (grippers opened):

| Score | Condition | Check on a real door |
|---|---|---|
| 1.0 | hung and shut | the shoes straddle the rib, the top is in the track, the edge is at the jamb |
| 0.5 | hung, not shut | a ruler from the door's edge to the jamb shows a gap over 3 mm |
| 0.25 | at some point the door top was between the lips 1 cm or more above their bottoms for 1 s with no part of the door on the table | a photo or video |
| 0 | anything else | |

Hung means all of: both shoes straddle the rib (the rib lies between the two cheeks, and the cheeks overlap the rib's
height by 8 mm or more), the door top is between the lips (touching one counts) and at least 1.5 cm above the lips' bottoms (z 15 cm or
more above the table), and the door is within 10 degrees of upright. It may lean on a lip. Shut means the door's +y
edge is within 3 mm of the jamb. A door leaned on the track with its shoes beside or on the rib is not hung. A door
whose lowest point is more than 3 cm above the table (still held) scores 0. Doing nothing scores 0. Pushing the door
along the table cannot score: its top has to be lifted into the channel and its shoes lifted over the rib.

## Real-world reset

Stand the door on its shoes at x 17 cm, y +6 cm, handle toward the arms. Nothing else moves between trials. A real
build uses a stock bypass door track and floor guide with a half-scale door.
