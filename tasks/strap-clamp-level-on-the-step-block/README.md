# Clamp a part to the mill table with a strap clamp, its heel on the step-block tread that levels it

A blue 6 x 6 x 4 cm part sits against two stops on a small mill table. A red slotted strap and a yellow step block
lie loose on the bare table, one in front of each arm. A stud stands beside the part with an orange nut raised near its
top. The robot builds the clamp the way a machinist does: it sets the step block on the mill table so the tread that
matches the part's height is under the strap's heel, threads the strap's open slot onto the stud under the raised nut
with its toe on the part's edge, and then runs the nut down. A block one tread off tilts the strap about 5 degrees;
a nut run down first blocks the slot. The part has to stay where it was, against both stops.

## Files

- `strap_clamp_level_on_the_step_block.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers the task file imports (moving parts, geometry checks,
  closed-loop scripted demo). Keep them beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Clamp the blue part to the mill table with the red strap clamp."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strap_clamp_level_on_the_step_block.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/strap-clamp-dream

# Scripted demo (full score): the left arm sets the step block on the mill table, the right arm threads the strap
# onto the stud and then runs the nut down
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strap_clamp_level_on_the_step_block.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/strap-clamp-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it,
  and the part's 3 mm limit is measured from that position.
- **Runtime per trial:** the demo finishes in about 72 s of simulated time (about 1,450 steps) and the trial ends once
  the clamp has held for 8 s; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 300 s for the demo and about 620 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (clamp level, nut down, everything still),
  otherwise `score()` at the end of the run.

## Scoring

The part counts as kept if it never moved more than 3 mm from its start at any step of the run (latched, so putting it
back does not clear it). Then, at the end:

1. **Score 0** if the part moved, or the strap is off the table or airborne (its underside more than 1 cm above every
   support: the table, the mill table, the part's top, the tread tops of a block that is itself on the table).
2. **1.0** if the strap is level and the nut is down.
3. **0.5** if the strap is level and the nut is not down.
4. **0.25** if the strap's toe rests on the part (toe underside within 3 mm of the part's top, at least 0.5 cm of toe
   over its edge) and its heel on any tread (within 3 mm of that tread's top), tilted at most 8 degrees.
5. Otherwise 0.

**Level:** the stud is inside the slot; the toe's underside is within 2 mm of the part's top with 1.0 to 2.5 cm of toe
over the part's edge; the heel end rests on a tread (underside within 2 mm of its top); the heel end's underside is
0 to 4 mm higher than the toe end's (about 0 to 2 degrees; 1.5 mm of sag below is allowed in the simulator); the strap
is tilted at most 3 degrees. Only the 4 cm tread meets this: the 3 cm tread leaves the heel 1 cm low and the 5 cm
tread 1 cm high.

**Nut down:** the nut's underside is within 3 mm of the strap's top at the stud.

Doing nothing scores 0. Pushing the strap along the table cannot score: it has to be lifted onto two supports 4 cm up.
A nut run down before the strap is threaded leaves the slot blocked and scores 0. On a real table, measure the toe and
heel heights with a ruler and check the toe overlap and the nut from a top photo.

## Real-world reset

Put the part against both stops, the strap and step block at their start spots (strap at x 30, y -17, step block at
x 24, y 15 cm, both with their long side along x), and the nut back at the top of the stud. In the simulation the nut
holds its height by damping; on a real stud use a nut that stays where it is wound.
