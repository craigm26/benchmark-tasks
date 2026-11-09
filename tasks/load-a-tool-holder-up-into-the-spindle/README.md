# Load a tool holder up into a mill spindle, keys aligned, while holding the drawbar release

A blue tool holder stands in a small grey cup in front of the right arm, turned a quarter turn from where it has to go.
A mill's spindle head hangs on a grey column at the back, its square bore opening downward, with two unequal black drive
keys under the face and an orange release tab on its side. The holder's yellow flange has two key slots, one wide and
one narrow. To load it, the release has to be held out (the spring-loaded tab pulled away from the head) while the holder
is turned to its keys and pushed straight up until its flange meets the face. Letting the release go must then leave the
holder hanging on the latch. A holder turned the wrong way stops 8 mm short, a release let go too early stops the shank
1.8 cm into the bore, and nothing can be pushed up into an overhead bore, so pushing cannot score.

CNC operators and machinists load tool holders this way (O*NET 51-9161.00 and 51-4041.00). Not tested: taper fit, drawbar
force, tool length offset.

## Files

- `load_a_tool_holder_up_into_the_spindle.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, and the scripted demo as `demo_script`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: small helper modules the task file imports (multi-part objects with
  spring joints, a closed-loop demo controller, and geometry checks). Keep them beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Load the blue tool holder into the mill spindle."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_a_tool_holder_up_into_the_spindle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/spindle-dream

# Scripted demo (full score): the right arm carries the holder from the cup, turns it a quarter turn and pushes it up on its
# measured pose; the left arm pulls the release out and holds it, then eases it back
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_a_tool_holder_up_into_the_spindle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/spindle-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the holder exactly where the task file puts it, in the
  cup and a quarter turn from keyed.
- **Runtime per trial:** the demo takes about 85 s of simulated time (about 1,700 steps) and the run ends 8 s after the
  holder hangs; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 330 s for the demo and about 580 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (the holder hanging on the latch, still), otherwise
  `score()` at the end of the run.

## Scoring

Progress flags stay set for the whole run; the end state is judged with the grippers open.

| Score | Condition | Check on a real build |
|---|---|---|
| 1.0 | the holder hangs on the latch: flange top 1 to 3 mm under the spindle face, slots on the keys (yaw within 4 degrees of keyed), tilt within 3 degrees, the release tab back at rest (within 2 mm) with the tongue in the notch, holder still | ruler at the side for the flange gap; top photo for the tab at rest; a tug on the holder shows it hangs |
| 0.5 | at any time the holder was seated: flange top within 3 mm of the face, yaw within 4 degrees, tilt within 3 degrees (the release was held out) | ruler at the side |
| 0.25 | at any time the shank was at least 1 cm up inside the bore | ruler on the shank |
| 0 | anything else | |

Doing nothing scores 0. A holder turned 90 or 180 degrees stops 8 mm short with its flange on a key (0.25); a release let
go early stops the shank 1.8 cm in (0.25); a holder seated and then dropped keeps 0.5. The simulator scores the final step
without opening the grippers: a holder held up against the face has no 1 mm drop onto the tongue, so it scores 0.5 and never
more than it would if released. The arms' paths are not scored; the demo keeps both hands clear of the mill's column.

## Real-world reset

Stand the holder back in the cup, turned a quarter turn from keyed (wide slot toward the left arm's side), and check the
release tab slides freely and returns to rest. Nothing is consumed. No third-party assets are used.
