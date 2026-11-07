# Take a temperature with a no-touch thermometer held above the forehead

A white block stands on a grey pad. It stands in for an infrared forehead thermometer. The robot must pick it up and
hold its bottom end 2 to 5 cm straight above a dark mark on a manikin's forehead for 3 seconds, without ever coming
within 2 cm of the head, then stand it back on its pad. It is the steady, no-contact hold that a home or clinic aid
robot needs: a 3 cm height band to stay in, a floor that must not be crossed, and a still hold for 3 s.

## Files

- `no_touch_thermometer_hover.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `belimits.py`: shared geometry helpers (footprints, tilt, bottom height).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="With the white no-touch thermometer upright, hold its bottom end 2 to 5 cm straight above the manikin's dark forehead mark for 3 seconds, never closer than 2 cm to the head, then stand it back on its grey pad."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=no_touch_thermometer_hover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/nth-dream

# Scripted demo (full score): the left arm carries the thermometer over the mark, holds it 4.5 s, and stands it back
# on the pad; the right arm stays still
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=no_touch_thermometer_hover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/nth-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts
  it; the head, the reading band and the pad are measured from those positions.
- **Runtime per trial:** the demo finishes in about 20 s of simulated time (400 steps); a Dream-1 trial runs up to
  1,200 steps (1 minute simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 90 s for the demo and about 160 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 1 s (read, then home and still), otherwise `score()`
  at the end of the run.

## Scoring

A reading is made when, for 60 control steps in a row (3.0 s at 20 Hz), all of these hold:

- the thermometer's centre is within 2 cm of the forehead mark, seen from above;
- its bottom end is 2 to 5 cm above the forehead's top (the mark's height);
- it is tilted at most 20 degrees from upright.

Then, in this order:

1. **Score 0** if at any time during the run the thermometer's bounding box came within 2 cm of the head sphere (the
   latch is never cleared by moving away, or by putting the thermometer back) or it fell below the table top.
2. **Otherwise 0.5** once a reading was made, **plus 0.5** if the thermometer then stands still (under 1 cm/s) and
   upright (tilted at most 15 degrees) on the pad, its centre within 3 cm of the pad's centre and its bottom within
   8 mm of the table top.

Possible scores: 0, 0.5, 1.0. Doing nothing, touching the head, a 2 s hold, or a hold on its side scores 0; a
reading never put back scores 0.5. A thermometer still held in the air at the end never counts as home. In simulation
only the thermometer is scored; in a real trial open both grippers first, and any robot part touching the head also
scores 0.

## Real-world reset

Fix a 12 cm foam head to the table at x 22, y 0 with a 4 cm felt dot on its top, and tape a grey pad at x 16, y +16.
Stand a 4 x 4 x 8 cm white block (or a real infrared thermometer of that size) on the pad. Read the hover height with
a ruler held beside the head. Between trials, stand the thermometer back on the pad.
