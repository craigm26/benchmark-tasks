# Hang a forklift fork on the carriage and set its spacing

A half-scale forklift carriage faces the arms with one fork already fitted. A second fork stands loose on the table
in front of it, and a pallet lies in front of the forks. The robot must fit the loose fork the way a truck operator or
mechanic does: hang it on the carriage at the removal gap (the only place its bottom hook goes down past the lower
bar), slide it along the bars, and drop its lock pin into the notch that puts its tine in line with the pallet's free
opening. A fork offered up anywhere else stays perched on the keeper strip and cannot hang. A fork locked one notch off
would spear a pallet block and scores 0.

## Files

- `hang_a_forklift_fork_and_set_its_spacing.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`: multi-part objects and slide joints (the carriage, the forks and their lock pins).
- `closedloop.py`: runs `demo_script()` inside the standard `yam_scripted` policy with live object positions.
- `belimits.py`: geometry helpers used by `score()`.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Fit the loose fork to the forklift carriage, spaced for the pallet."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_a_forklift_fork_and_set_its_spacing.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/fork-dream

# Scripted demo (full score): the right arm hangs the fork at the gap by its green tab, lifts the pin by its red knob,
# slides the fork to the notch in line with the pallet's free opening and lowers the pin; the left arm stays still
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_a_forklift_fork_and_set_its_spacing.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/fork-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it.
  There is one fixed layout, so the seed does not change the start.
- **Runtime per trial:** the demo takes about 50 s of simulated time (about 1,000 steps), then the trial runs 8 s more
  and ends with success; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 220 s for the demo and about 540 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (the new fork locked in the right notch, both
  forks still), otherwise `score()` at the end of the run.

## Scoring

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left). At the end of the run
(grippers opened), with a ruler and a photo from the front:

1. **Hung** means the fork's top plate rests on the rib (its underside within 3 mm of the rib's top), its back lip is
   behind the top bar, its bottom hook's lip is behind the lower bar's front strip, and it tilts under 5 degrees.
2. **Score 0** unless the reference fork is still hung with its pin down in the y +2 notch and the new fork is hung.
   A fork on the table, perched on the keeper strip, leaning, or held above the rib is not hung.
3. **1.0** if the new fork's pin is down in the y -18 notch: pin within 3 mm of its lowest point and the fork's
   centre within 4 mm of y -18, which is 20 cm centre to centre from the reference fork. In the front photo both
   tines point into the pallet's openings.
4. **0** if the new fork's pin is down in the y -14 or y -2 notch: its tine meets a pallet block.
5. **Otherwise 0.5** if the fork's centre is 5 cm or more from the gap's centre (y -8), so the keeper strip holds
   its bottom hook down and it can no longer lift off; **0.25** if it is nearer (hung at the gap).

Doing nothing scores 0. Pushing cannot hang a fork: its hooks only pass the bars when it is lifted 2.5 cm and moved in
over them, and its bottom hook only goes down at the gap. In simulation a fork still held in the gripper is judged
where it is: held above the rib it is not hung, and a pin still held up is not down.

The scripted demo also keeps the whole right arm out of the space behind the carriage's back plate, where a real
truck's mast rails and lift chains run. The scorer sees only the objects, so this is not scored; on a real trial keep
hands and grippers in front of the back plate.

## Real-world reset

Lift the new fork's pin by its knob, slide the fork back to the gap, lift it off the bars and stand it on its mark on
the table (y -13, shank front at x 28). Check that the reference fork's pin is down in the y +2 notch. The carriage and
the pallet stay fixed.
