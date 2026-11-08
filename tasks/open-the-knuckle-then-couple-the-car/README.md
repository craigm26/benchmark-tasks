# Open a coupler knuckle by hand, then roll the car in until the knuckle closes and locks

A short red car stands on a track in front of the left arm; a grey coupler head stands at the far end, in front of
the right arm. The head's mouth is shut by a black sliding knuckle jaw, held shut by a yellow spring lock standing in a
notch in the jaw. The robot has to lift the lock out of its notch, pull the jaw open by its red lever, and then push
the car in by its blue handle. With the jaw open the car's pin goes into the head's throat, and a red striker finger
on the car rides up a ramp on the jaw and pushes it shut behind the pin; the lock drops back into its notch. With the
jaw left shut the pin only butts against it, 10 cm short of its seat. Yard crews do this by hand before a shove (two
closed knuckles push the cars apart) and the lock dropping is the check that the coupling took.

## Files

- `open_the_knuckle_then_couple_the_car.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers for jointed parts, geometry checks and the closed-loop
  scripted demo. Keep them beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Couple the red car to the grey coupler head and leave them locked together."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_the_knuckle_then_couple_the_car.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/knuckle-dream

# Scripted demo (full score): the right arm lifts the lock by its yellow tab, the left arm pulls the jaw open by the red
# lever, both let go and leave, then the left arm pushes the car in by its blue handle at 3 cm/s until the jaw has shut
# behind the pin and the lock has dropped
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_the_knuckle_then_couple_the_car.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/knuckle-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: every part is a jointed piece of one mock-up and starts shut,
  dropped and at rest.
- **Runtime per trial:** the demo takes about 45 s of simulated time (about 900 steps) and the trial ends in success
  8 s after the lock drops; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 160 s for the demo and about 510 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds (coupled, no butt, still) for 8 s, otherwise `score()` at the
  end of the run.

## Scoring

At the end of the run (grippers opened on a real trial), read from a top photo and a ruler. K is how far the jaw is
pulled open, L how far the lock is lifted above its notch floor, J how far the car has moved toward the head.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | coupled, no butt at any time, still for 1 s | pin head in the throat (tip at y -7.35 cm), jaw shut, lock down in its notch |
| 0.75 | coupled after a butt, or coupled but not yet still for 1 s | same, but the pin was once shoved against the shut jaw |
| 0.5 | not coupled, but at any time K was 2 cm or more while J was under 2 cm | jaw was pulled open before the car moved |
| 0.25 | otherwise, L was 1.2 cm or more for 0.5 s at any time | lock was lifted clear of its notch |
| 0 | anything else | |

- **Coupled:** K 3 mm or less (jaw shut), L 3 mm or less (lock dropped), J between 13.35 and 15.35 cm (pin within 1 cm of
  its seat, car 14.35 cm from its start).
- **Butt:** the car 9.2 to 10.8 cm from its start while K is 3 mm or less. The shut jaw stops the pin head there. It is
  latched for the whole run, so backing the car off and trying again still scores 0.75 at best.
- **No shortcuts:** the lock's foot stands in the notch with 3 mm to spare, so the jaw cannot open until the lock is lifted
  1.2 cm; the jaw cannot be pushed open from the car; a jaw shut by hand with the pin short of its seat is not coupled; a
  lock held up at the end is not dropped, so a held part never scores better than a released one (the simulator scores
  the last step without opening the grippers). The car is bound to its track and cannot be carried off.

Doing nothing scores 0. Where the arms are is not scored: on a real trial nobody stands between the cars while the
car moves, and the arms stay out of the gap between the car and the head (x 24 to 30 cm, y -5.5 to 7.5 cm) below 12 cm;
the lock tab, the lever block and the car handle are all reached from outside it.

## Real-world reset

Slide the car back until the pin tip is 7 cm from the head's mouth line (y 7 cm), push the jaw shut by its lever and let
the lock drop into its notch. Mark the car's nose and the jaw's tip on the track with tape. A printed knuckle, lock and
pin on a drawer slide would do; the 3 mm clearances and the striker ramp are what the simulation models.
