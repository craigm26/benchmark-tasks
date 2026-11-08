# Fit two brake pads facing the rotor, then swing the caliper down over them

A disc brake is shown with its caliper swung up. The robot fits two new pads into the bracket's ear slots, one each
side of the rotor, with each pad's friction lining toward the rotor, then swings the red caliper down over both. It is
the pad change automotive service technicians do on every wheel: a keyed drop-in (the steel ears of a pad go into narrow
slots), a quarter turn of each pad the right way (the two pads are mirror images), and an ordered close (the caliper
cannot shut until both pads are fully down, and once it is shut the slots are covered). The two pads start in holders
with their linings facing the same way, so one must be turned each way. A pad turned lining-outward cannot get its ears
down: the lining hits a pad stop and the pad perches on the bracket.

Closest board tasks: the sand-core and cope task (insert, then close) and the filter-cartridge flow-arrow task (one
part, one direction). This one adds two mirror parts, each turned to face a shared rotor, and a closure that fails if
either part is high.

## Files

- `fit_brake_pads_then_swing_the_caliper_down.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, and the closed-loop `demo_script`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: shared helpers the task file imports (multi-part objects with a hinge,
  the closed-loop demo controller, and footprint tests). Keep them beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Fit both new brake pads in the bracket with their linings facing the rotor, then swing the red caliper down over them."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fit_brake_pads_then_swing_the_caliper_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/brake-pads-dream

# Scripted demo (full score): the right arm seats both pads, the left arm swings the caliper down
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_brake_pads_then_swing_the_caliper_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/brake-pads-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it, and
  the slots are only 3 mm wider than the plates on each side.
- **Runtime per trial:** the demo takes about 80 s of simulated time (about 1,600 steps) and then holds the finished
  state for 8 s; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 280 s for the demo and about 540 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (both pads seated, caliper closed, everything
  still), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened on a real trial), with a ruler and a top photo:

1. **Score 0** if any part of a pad is over the rotor (x 26 to 28 cm, y -12 to 4 cm), at any height.
2. A pad is **seated** when its steel ears are in both slots of one pair: the plate's bottom within 3 mm of the slot floor
   (3.5 cm above the table), the plate's middle within 4 mm of the slot's middle in x and 3 mm of y -4, the pad leaning at
   most 10 degrees, and its lining turned to the rotor (within 6 degrees). One pair of slots takes one pad.
3. **0.25** for one pad seated, **0.5** for both seated, **1.0** for both seated and the caliper **closed** (the red
   post within 4 mm of resting on its stop, 0.05 rad), with the pads still.

A pad turned lining-outward, perched on the pad stops or the abutments, standing 6 mm or more up, or held in the air
is not seated. Pads sit in holders 3 cm tall and their ears are 3.5 cm up, so pushing a pad along the table cannot seat
it: it has to be lifted out. Closing the caliper first covers the slots, so the pads cannot be fitted afterwards. Doing
nothing scores 0. In the simulation the outline test uses each pad's real pose; the arms' path is not scored (a real
trial keeps hands and tools off the rotor face), and the task file's `ARM_KEEP_OUT` is checked offline.

Offline, the scorer is checked on 34 synthetic end states (start 0, intended end 1.0, each constraint broken, pads
pushed, held, reversed, perched, leaning, off by 5 to 8 mm).

## Real-world reset

Stand both pads upright in their holders with their linings toward +y (the left of the table), put the grip tabs on top,
and swing the caliper back up against its stop. Nothing is consumed. A scrap caliper bracket, a rotor and two new pads
(or a printed copy) will do; the bracket, rotor and pad stops stay fixed to the table.
