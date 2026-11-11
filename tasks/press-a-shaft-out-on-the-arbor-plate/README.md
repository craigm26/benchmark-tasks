# Turn an arbor press plate to the slot that backs the hub, set the pulley on it and press its shaft out

A pulley with a press-fit shaft stands beside an arbor press. Under the ram is a press plate that turns on a post and
has three slots: 1.4 cm, 3.0 cm and 5.4 cm wide. The 1.4 cm slot is under the ram at the start; it is narrower than
the 1.8 cm shaft, so the shaft cannot pass. The 5.4 cm slot is wider than the 4.4 cm hub, so the hub falls through.
Only the 3.0 cm slot clears the shaft and still carries the hub. The robot has to turn the plate a quarter turn by
its orange tab, set the hub on the plate with the shaft on the ram axis (within 3 mm), and drive the ram down by its
blue handle until the shaft top is level with the hub top. The instruction names only the job; which slot to use is
left to the scene.

## Files

- `press_a_shaft_out_on_the_arbor_plate.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`: multi-part objects with slides and hinges (the ram, the plate, the shaft in its hub).
- `closedloop.py`: runs `demo_script()` inside the standard `yam_scripted` policy with live object positions.
- `belimits.py`: shared geometry checks used by `score()`.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Press the shaft out of the red pulley hub on the arbor press."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=press_a_shaft_out_on_the_arbor_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/arbor-dream

# Scripted demo (full score): the left arm turns the plate by its tab; the right arm carries the pulley in below
# the ram face and seats it; the left arm drives the ram down by its handle, lifts it and lets go
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=press_a_shaft_out_on_the_arbor_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/arbor-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts
  it, and the 3 mm limit is measured from the ram axis of the press at that position. No random seed is used.
- **Runtime per trial:** the demo takes about 95 s of simulated time (about 1,900 steps); a Dream-1 trial runs up
  to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 390 s for the demo and about 560 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score, pulley and plate still; the long
  hold keeps the video running until the hand has let go and moved away), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened), from a top-down photo and a ruler:

1. **Score 0** if the hub is on the table or off it (dropped, pushed off its stand, or fallen through the 5.4 cm
   slot), or if the 3.0 cm slot is not under the ram: the plate must be within 0.05 rad of a quarter turn from its
   start, which is 4 mm at the tab.
2. **0.25** for the plate turned to the 3.0 cm slot, nothing else done.
3. **0.5** when also the hub is seated: flat on the plate (tilt at most 3 degrees, which is 2 mm across the hub;
   bottom within 2 mm of the plate top) with the shaft centre within 3 mm of the ram axis. A hub held above the
   plate, still on the stand or leaning is not seated.
4. **1.0** when also the shaft is pressed (its top at most 5 mm above the hub top, driven there by the ram) and the
   ram face is back at least 3 cm above the hub, with everything still for 1 s.

The press must be done by the ram. Over the whole run, shaft travel that happens while the ram face is not on the
shaft top (a finger or hand pushing it, the hub lifted off it) is added up; more than 1.2 cm of it voids the press
for the rest of the run, even if things are put back, so the score stays at 0.5 at most. Doing nothing scores 0.
The pulley has to be lifted off its stand and set on the plate 12 cm away, so pushing it cannot score.

Ruler checks: shaft top to hub top (at most 5 mm), and the shaft standing about 4 cm out below the plate. Top photo:
which slot is under the ram, and the shaft centre against the ram axis.

The scorer sees only the objects. The scripted demo also keeps every arm part clear of the press column, carries
the pulley in below the ram face, and never puts a hand under the ram while it comes down; in a real trial that
last point is a rule for the operator to watch.

## Real-world reset

Press the shaft back until its bottom is flush with the hub bottom, stand the pulley on its stand, turn the plate
back so the 1.4 cm slot is under the ram, and let the ram rest at the top of its stroke. Mark the stand's outline
and the plate's start position on the table before the first trial. A real build uses a 1 ton arbor press with its
pinion lever replaced by a handle on the ram; keep people's hands out from under the ram during trials.
