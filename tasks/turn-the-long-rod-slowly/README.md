# Turn two long rods onto their brackets without their ends moving faster than 20 cm/s

Two 30 cm rods lie along x, one in front of each arm. Each is to be turned a quarter turn and laid across a pair of
dark grey brackets 16 cm apart, so that it ends along y. Someone is standing by the table, so neither end of a rod
may ever move faster than 20 cm/s. A turn swings the ends at the turn rate times 15 cm on top of the carry, so a
quick turn breaks the limit even when the hand barely moves. A full score needs both rods hung with no end ever over
the limit. Breaking the limit once scores 0 for the whole run.

## Files

- `turn_the_long_rod_slowly.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `closedloop.py`, `belimits.py`: shared helpers (closed-loop demo controller, geometry checks), unchanged.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Turn each rod across and lay it on a pair of grey brackets, one rod to a pair. Someone is standing by the table, so neither end of a rod may ever move faster than 20 cm/s."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_the_long_rod_slowly.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/rod-dream

# Scripted demo (full score): the left arm hangs its rod, then the right arm hangs the other
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_the_long_rod_slowly.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1000 --store-frames --log-dir ~/yam-eval/rod-demo
```

- **Seed and layout:** `-E object_jitter=0` is required (the rods start at their marked spots). `-E control_hz=20` is
  required: speeds are measured over two control steps, which is 0.10 s only at 20 Hz (at a lower rate the reading is
  higher, so a slower rate fails safe).
- **Runtime per trial:** the demo takes about 42 s of simulated time (840 steps); a Dream-1 trial runs up to 2,400
  steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 140 s for the demo and about 320 s for a full Dream-1 trial.
- **Demo speed:** the demo grips each rod at its middle and turns it during a 4.5 s carry. Its largest measured end speed
  is about 0.13 m/s (turn 0.08 plus carry 0.05), about a third under the limit. The same plan run 2.6 times faster
  reaches about 0.37 m/s and scores 0.
- **Score:** `task_score` reports 1.0 when `success()` holds for 1 s (both rods hung, still, no end ever over the
  limit), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened). In the simulator the final step is scored without opening the grippers, so a
rod still held in the air counts as neither hung nor turned, which is never better than releasing it.

1. **Speed.** Each rod end is the rod's centre plus or minus 0.15 m along its own long axis, from its simulated pose.
   Its speed is its displacement over two control steps (0.10 s) divided by 0.10. Any speed over 0.20 m/s sets a
   breach that lasts the whole run. A breach, or a rod whose lowest point falls 5 cm below the table top, scores 0.
2. **Hung, 0.5 per rod.** Still (under 1 cm/s), centre within 1.5 cm of a bracket pair's midpoint (x 32, y 13 or x 24,
   y -13), long axis within 10 degrees of y, rolled at most 20 degrees, lowest point within 1 cm of the bracket tops (3 cm above the table).
3. **Turned, 0.25 per rod.** Still, long axis within 10 degrees of y, rolled at most 20 degrees, lowest point within
   1 cm of the table top (or of the bracket tops with its middle within 4 cm of a pair's midpoint), and carried at least
   2 cm off the table at some point. Sliding a rod round on the table, or perching it on the other rod, scores 0.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0 (the rods lie along x). Either rod may be hung on
either pair, one rod to a pair.

**Reset between trials.** The tracker clears on the first step, when both rods return to their start poses (2 mm, 3
degrees) after one was more than 3 cm away (an environment reset), and when both rods are at their start poses, still,
and neither has been more than 3 cm from it since the last clear. The last path is the only residual one: a rod jerked
quickly less than 3 cm and put back exactly is forgiven. The offline tests cover it.

## Real-world reset

Lay both rods back along x at their marked spots. The four brackets and the post stay down. In a real trial, open the
grippers before measuring and film from above at 30 fps: an end that moves more than 20 mm in 3 frames breaks the limit.
