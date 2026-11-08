# Line up the chute gates for the green bin, then release the batch

A short chute slopes down toward the arms. At its high end a closed hopper (a 2 cm slot in its roof shows the balls but is
too narrow for one to pass) holds three yellow balls behind an orange slide gate, which a spring keeps shut. Below it two grey diverter gates stand across the chute, each with an upright
handle outside the chute wall. At the low end the chute feeds three bins: grey, grey and green. As the scene starts the
left gate sends the balls to the left-hand grey bin and the right gate is pulled out of the way. The robot must set both
grey gates so the balls run into the green bin, and only then pull the orange gate open and hold it until all three
balls have left the hopper. A ball that goes into a grey bin is lost and cannot be put back. Doing nothing scores 0.

## Files

- `chute_gates_to_green_bin.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `mechkit.py`: the shared helper that builds the chute and its three sliding gates as one object with joints.
- `belimits.py`: the shared geometry helpers (footprints, a latch that survives from step to step).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Set the two grey chute gates so the balls run into the green bin, then pull the orange slide gate open and hold it until all three have left the hopper."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=chute_gates_to_green_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cggb-dream

# Scripted demo (full score): the right arm pushes the right gate in, the left arm pulls the left gate out,
# then the left arm pulls the orange gate open, holds it while the balls run, eases it shut and lets go
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=chute_gates_to_green_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cggb-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the balls in their row against the slide gate and
  the gates at their marked starts. The latch that remembers a lost ball clears when the simulator resets the scene.
- **Runtime per trial:** the demo finishes in about 32 s of simulated time (about 640 steps, the last 4 s waiting for
  the hand to clear); a Dream-1 trial runs up to 2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 180 s for the demo and about 490 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 4 s (all three balls settled in the green bin and the
  slide gate shut), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened, the slide gate let go so its spring shuts it), in this order:

1. **Score 0** if any ball is ever lifted out over the bins (its centre more than 7 cm above the table while over a
   bin). A ball held in the air counts as not placed.
2. **Balls in the green bin:** count the balls resting in it (centre x 5 to 14 cm, y -10 to -4 cm, under 4 cm above
   the table, still, on its floor or on another ball there). A ball that was ever over a grey bin, at any height,
   is lost for good and never counts.
3. **Score:** 1.0 for three balls with the slide gate shut (handle within 5 mm of its start); 0.75 for three balls
   with it open, or for two; 0.5 for one; 0.25 for none, if no ball is lost and both grey gates are set (each handle
   moved 8.5 cm or more from its start); otherwise 0.

The gates count as set at 8.5 cm of travel or more (the stops are at 9 cm; 8.5 cm keeps the right gate's tip 3.5 cm
across the ball stream and the left gate's tip 5.5 cm clear of it). Pushing or dragging a ball never scores: the
hopper is closed, the balls leave it only through the slide gate, and the green bin counts only balls that came down
the chute without going over a grey bin.

## Real-world reset

Push the left gate back across the chute, pull the right gate out beyond the wall, and let the slide gate spring shut.
Pick the balls out of the bins and put them back in a row in the hopper against the slide gate (pull the gate open a
little, drop them in through the mouth, release). Mark each handle's start on the table.
