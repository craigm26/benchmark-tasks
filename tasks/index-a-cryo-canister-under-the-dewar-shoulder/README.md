# Return a cryo canister to its marked place in a dewar

A small orange canister stands beside a white dewar. The robot must lower it down the dewar's 8 cm neck by the top of
its purple handle, slide it sideways under the shoulder into the floor slot below the green-marked notch, and set it
down so that its body drops into the slot and the lip of the handle's hook sits in the green notch of the index ring.
It is the blind-placement step a lab technician repeats at a liquid nitrogen dewar: the place cannot be seen from above,
so the position is found by feel and by the notch. A canister left in the neck, resting on a slot wall, hung with its
hook on the ring, or put in the other slot, scores less.

## Files

- `index_a_cryo_canister_under_the_dewar_shoulder.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, the scripted demo).
- `mechkit.py`, `closedloop.py`, `belimits.py`: helper modules the task file imports (multi-part objects, the
  closed-loop demo controller, geometry checks).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Return the orange canister to the dewar at the green-marked position."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=index_a_cryo_canister_under_the_dewar_shoulder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/cryo-dream

# Scripted demo (full score): the left arm carries the canister by its handle; the right arm holds the ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=index_a_cryo_canister_under_the_dewar_shoulder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/cryo-demo
```

- **Seed and layout:** `-E object_jitter=0` is required. The slot and the notch have millimetre clearances, so the
  layout must be exactly the one in `task.json`.
- **Runtime per trial:** the demo takes about 34 s of simulated time plus an 8 s hold after the canister is seated; a
  Dream-1 trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 130 s for the demo and about 430 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (the canister seated and still, so the video shows
  the release and the arm moving clear), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened), the canister is **seated** when all of these hold:

- its body underside is within 3 mm of the table inside the green slot, and the body centre is within 6 mm of the slot
  centre (x 23.5 cm, y 13.85 cm);
- it is tilted at most 4 degrees;
- the centre of the hook's lip is within 6 mm of the notch centre in x, and the lip's bottom is below the top of the ring.

Then:

| Result | Score |
| --- | --- |
| Seated, and still for 1 s | 1.0 |
| Not seated, but upright (at most 15 degrees) inside the dewar with its centre at least 5 cm toward +y of the neck centre, under the green-side shoulder: on a slot wall, or hung by its hook on the ring | 0.5 |
| Inside the dewar with its top under the deck, tilted at most 45 degrees (left in the neck, or in the -y slot) | 0.25 |
| Anything else: still outside, on the deck, off the table, lying on its side | 0 |

A canister whose underside is more than 1.6 cm above the table counts as not placed, since the simulator scores the final
step without opening the grippers and a real trial opens them first. The dewar is closed apart from the neck, so the
canister cannot be pushed in: it has to be lifted over the 8.6 cm ring. On a real table: a ruler gives the handle top
at 14 cm when seated and 14.6 cm when hung on the ring, and a photo from above shows the lip in the green notch.

Doing nothing scores 0 (the canister stays outside).

## Real-world reset

Stand the canister at its start mark, about 3 cm from the dewar wall with the handle toward -y, and check that both slots are
clear. The dewar, ring and slots are fixed. Use a printed dewar top and canister (the handle can be bent aluminium
strip); no liquid nitrogen is involved.
