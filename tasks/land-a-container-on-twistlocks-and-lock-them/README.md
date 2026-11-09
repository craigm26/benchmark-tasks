# Land a container on two deck twistlocks, then turn both locks shut

A red container stands on the table with its long side along x. The robot must turn it, lift it by the white lug on its
top, and land it on a dark blue deck so that two twistlock cones, 11 cm apart, go up through two holes in its floor. The
container's own body hides the cones as it comes down. Then each cone gets a quarter turn with its orange handle, which
stands well out beside the container's end (the gripper is 6.8 cm wide, so the handles are 12.5 cm off the middle line). The order is forced: a head turned before landing blocks its hole and the container
sits 2 cm too high on it. A container 3 mm off, or askew, sits on a head too.

A full score needs the container down on both flanges with both locks turned shut and the container let go.

## Files

- `land_a_container_on_twistlocks_and_lock_them.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, scripted demo). `mechkit.py`, `closedloop.py` and `belimits.py` are helper modules it imports; keep them
  beside it.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Land the red container on the deck twistlocks and lock it down."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=land_a_container_on_twistlocks_and_lock_them.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/twistlock-dream

# Scripted demo (full score): the right arm carries and lands the container, the left arm turns the left lock and
# the right arm then turns the right lock
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=land_a_container_on_twistlocks_and_lock_them.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/twistlock-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the container at its start spot, which fixes the
  scored positions.
- **Runtime per trial:** the demo succeeds after about 94 s of simulated time (about 1,900 steps) and the episode then
  holds for 8 s so the release shows; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 390 s for the demo and about 690 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (landed, both locks locked, container still),
  otherwise `score()` at the end of the run.
- **Licence:** every object is a plain box or cylinder written in the task file; no third-party assets.

## Scoring

At the end of the run (in the real world: grippers opened first), in this order:

1. **Lifted** (kept over the whole run): the container's underside has been at least 3 cm above the table. Without it
   the score is 0, so pushing the container along the table never scores. A container whose underside is more than
   1 cm above every support (table, flange tops, head tops) at the end counts as not placed: 0.
2. **Landed:** the container's underside within 3 mm of the flange tops (2.0 cm above the table), tilt at most 2
   degrees, centre within 6 mm of the deck's centre, square to its corner marks within 3 degrees (half a turn looks
   the same). The cones can only be in the holes then; a head in the way holds the floor 2 cm higher.
3. **Locked:** a lock's angle within 0.15 rad of a quarter turn (1.57 rad). Tape equivalent: the handle within 1.5 cm of
   the centre of its white locked mark.
4. **1.0** landed, both locks locked, container still. **0.5** landed with one or no lock locked. **0.25** the
   container is set down upright (tilt at most 15 degrees) with both cones under its footprint, but not landed: it sits
   on a head turned before landing, or is off by more than 3 mm. **0** otherwise.

Doing nothing scores 0 (the container never lifts). Turning both locks first scores 0.25 at best. In simulation the
tests use the container's actual pose, and the locks' angles come from the hinge joints.

## Real-world reset

Turn both handles back to their open positions (heads with their long side along y), and stand the container at its
start spot, upright, long side along x. Nothing is consumed. Mark the deck's four corner marks and two locked marks
with paint or tape before the first trial; use a ruler at the container's corners to read its height.
