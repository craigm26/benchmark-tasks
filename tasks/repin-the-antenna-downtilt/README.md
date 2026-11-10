# Re-pin an antenna's downtilt bracket to the marked hole

A panel antenna hangs on a hinge from a mast and is pinned plumb. Its red bracket arm is held by an orange lock pin
that passes through the arm and the plumb hole of a grey tilt plate. The plate has four square holes on an arc at 0
(plumb), 5, 10 and 15 degrees; the 10 degree hole is marked in green. The job is to draw the pin out, swing the panel's
foot out by its blue handle until the arm's hole lines up with the green-marked plate hole, hold it there against
gravity, and push the pin through both. Let go and the panel rests on the pin at the new tilt. The holes are 1.4 cm
square and the rod is 0.8 cm, so there are 3 mm of play a side. Only the green-marked hole scores a full mark; a pin
left in any other hole scores 0.

## Files

- `repin_the_antenna_downtilt.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: helper modules the task file imports (hinge and free-body objects,
  closed-loop scripted demo, geometry checks).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Reset the antenna's downtilt to the hole marked in green on its tilt plate."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=repin_the_antenna_downtilt.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/antenna-dream

# Scripted demo (full score): the right arm draws the pin out by its head; the left arm swings the panel by its
# handle to 10 degrees and holds it; the right arm pushes the pin through the plate and the arm; both let go and park
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=repin_the_antenna_downtilt.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/antenna-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it.
- **Runtime per trial:** the demo takes about 52 s of simulated time (515 steps) and holds for 8 s after success. A
  Dream-1 trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 340 s for the demo and about 680 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds (the pin through the marked hole and the arm, the panel
  resting on it, the pin still), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened), in this order. T is the panel's tilt; on a real build, at 10 degrees the
arm's hole sits 3.5 cm toward the arms from where it hangs plumb (ruler on the table).

| Score | Condition | Check on a real build |
|---|---|---|
| 0 | the pin ends through the plumb, 5 degree or 15 degree plate hole and the arm (a wrong tilt goes live), or it was never drawn clear of the plate | look at which hole the pin is in |
| 1.0 | the pin is through the green-marked plate hole and the arm (rod axis within 3.5 mm of both hole centres), T is 7.5 to 12 degrees, and the pin is not held up (its lowest point is not over 1 cm above the table or the marked hole's bottom) | the pin passes through both plates; arm hole 2.6 to 4.2 cm toward the arms from plumb (ruler) |
| 0.5 | the pin was drawn out and its tip entered the green-marked hole by 5 mm or more while T was 7.5 to 12 degrees, but the end state is not 1.0 | side photo of the pin tip in the green-marked hole |
| 0.25 | the pin was drawn clear of the plate (tip 2 mm or more behind the plate's outer face, or the pin lost off the table) and nothing more | tip behind the plate |

"Drawn out" and "entered" are kept for the whole run, so returning to the start does not erase them. A panel resting on
the pin sits at 8.3 to 10 degrees because of the 3 mm play, inside the 7.5 to 12 degree window. Pushing cannot score: the
pin has to be drawn out 16 cm and aimed along y into a 1.4 cm hole at the right height, and a pin lying on the table is far too low
for any hole. A pin held in the air at the end counts as not placed, whatever its pose.

The simulator sees only the pin and the panel. The arms' path is not scored.

## Real-world reset

Let the panel hang plumb on its stop and push the pin back through the plumb hole and the arm's hole. Nothing is
consumed.
