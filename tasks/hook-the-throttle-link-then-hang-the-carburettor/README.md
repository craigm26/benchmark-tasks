# Hook a small engine's throttle link into the carburettor, then slide it onto its studs

A carburettor stands on the table in front of the right arm. The engine has two steel studs and a sliding throttle link
with an orange pin. The pin enters the carburettor's yellow lever plate sideways, along y, and the carburettor can only be
slid onto the studs along x. Once it is on the studs it cannot move sideways, so the pin has to be hooked first, with the
carburettor held in the air, and the hooked carburettor then carried 5 cm along onto the studs, taking the link with it.
Mounted first, the pin cannot be hooked. A rust-brown muffler stands beside the engine. A full score needs the pin through the
lever hole and the carburettor seated on the gasket.

## Files

- `hook_the_throttle_link_then_hang_the_carburettor.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, the closed-loop `demo_script()`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers the task file imports (moving parts, geometry checks,
  closed-loop arm control).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Refit the carburettor to the engine and hook up its throttle link."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hook_the_throttle_link_then_hang_the_carburettor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/hook-dream

# Scripted demo (full score): the right arm grips the green tab, hooks the pin into the lever, slides the carburettor onto the
# studs, lets go and parks clear; the left arm stays at its ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hook_the_throttle_link_then_hang_the_carburettor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/hook-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it.
- **Runtime per trial:** the demo takes about 40 s of simulated time (about 800 steps) and then ends 8 s after the full score; a
  Dream-1 trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 140 s for the demo and about 530 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held (hooked, seated, still), otherwise `score()` at the end of the run.

## Scoring

The hooked and on-the-studs flags are kept over the whole run; the carburettor is judged at the end.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | hooked and seated, nothing held | the pin shows through the lever hole in a top photo; a ruler between the flange and the gasket reads 3 mm or less; both studs through the flange |
| 0.5 | hooked at some point with the flange within 3 cm of the gasket | a ruler at the gasket |
| 0.25 | hooked for 1 s while the carburettor is lifted at least 1 cm off the table | a top photo, a phone timer |
| 0 | anything else | |

Hooked means the pin passes through the lever plate's hole (its axis within 5 mm of the hole's centre) with the tip past the
plate's inner face. Seated means the flange is within 3 mm of the gasket with both studs through the holes (hole lines within
4 mm of the studs) and the carburettor tilted at most 3 degrees. Seated but never hooked scores 0.

Doing nothing scores 0. Pushing the carburettor along the table scores 0: the studs are 7 cm above the table, so it has to be
lifted. A carburettor held up at the end counts as not placed (its bottom must be within 4 mm of the seat height). The
arms' final pose and their path are not scored; only where the parts end up.

The muffler is a hot part in a real engine; keep hands and tools off it. The simulation does not score arm contact with it.

## Real-world reset

Stand the carburettor back at its start with the flange toward the engine, and push the link back to the -x end of its guide.
A printed intake block with two studs, a pad under the carburettor, and a link in a slot is enough to rebuild the scene.
