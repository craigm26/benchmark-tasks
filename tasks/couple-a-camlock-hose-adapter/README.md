# Couple a hose adapter into a cam-and-groove coupler and close both cam arms

A blue hose adapter stands on a white pad in front of the right arm, its square shank pointing at a coupler fixed to a
tank end wall. The robot has to lift it, push the shank sideways into the coupler's square bore until the collar meets
the mouth, and then swing both red cam arms closed along the body. Each arm's lobe passes through a slot in the wall
into a groove in the shank, so an arm closes only if the shank is far enough in; a shank that is more than about 5 mm
short stops the lobes at its shoulder, and arms swung closed before the insertion block the bore. The score rewards the
insertion first (0.5) and then each closed arm (0.25 each), with credit for a shank that has gone in 3 cm.

## Files

- `couple_a_camlock_hose_adapter.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers for hinged parts, geometry checks and the closed-loop
  scripted demo. Keep them beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Couple the blue hose adapter to the tank's coupler and close both red cam arms."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=couple_a_camlock_hose_adapter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/camlock-dream

# Scripted demo (full score): the right arm brings the adapter to the mouth and pushes it home, lets go and leaves;
# the left arm closes its cam arm, then the right arm closes the other
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=couple_a_camlock_hose_adapter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/camlock-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the start is recognised by the adapter standing on its pad.
- **Runtime per trial:** the demo takes about 70 s of simulated time (about 1,400 steps) and the trial ends in success
  8 s after the last arm closes; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 290 s for the demo and about 530 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds (home, both arms closed, still) for 8 s, otherwise `score()`
  at the end of the run.

## Scoring

At the end of the run (grippers opened on a real trial), G is the gap between the adapter's collar and the coupler's
mouth, measured with a ruler.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | home and both cam arms closed, for 1 s | both grip blocks within 7 mm of the body, collar at the mouth |
| 0.75 | home, one cam arm closed | one grip block within 7 mm of the body |
| 0.5 | home, no cam arm closed | collar within 5 mm of the mouth |
| 0.25 | not home, but the shank tip was at least 3 cm into the bore, on its axis, for 1 s | the shank's tip 3 cm past the mouth |
| 0 | anything else | |

- **Home:** G is 5 mm or less, the shank tip is on the bore axis (within 10 mm), the riser leans at most 15 degrees (its
  top within 2.4 cm of plumb over its 9 cm height) and the adapter is still.
- **A cam arm is closed** when its grip block lies within 7 mm of the coupler body (the arm within 0.1 rad of a quarter turn).
- **Not placed:** an adapter held up with its underside more than 1 cm above the pad or the table, and not in the bore,
  scores 0, as does one knocked off the table. The simulator scores the final step without opening the grippers, so this
  keeps a held adapter from scoring better than a released one.
- **No shortcuts:** the bore is 5.5 cm above the table, so an adapter pushed along the table cannot enter it; arms closed
  first block the bore (their lobes stand 2.2 cm apart, the shank is 3 cm wide); a shank more than about 5 mm short stops
  a lobe at the shank's shoulder, so that arm cannot close. Putting the adapter back on its pad restarts the trial.

Doing nothing scores 0. The arms' paths are not scored; only where the adapter and the cam arms end up.

## Real-world reset

Stand the adapter back on its pad, shank toward the coupler, and swing both cam arms open until their bars point straight
out. A real camlock coupler and adapter (2 in) on a plywood tank end would do; the slots, the 4 mm clearances and the
neck, head and nose of the shank are what the simulation models.
