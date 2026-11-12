# Clamp a part to the mill table with a strap clamp, its heel on the step-block tread that levels it

A blue 6 x 6 x 4 cm part sits against two stops on a small mill table. A red slotted strap and a yellow step block
lie loose on the bare table, one in front of each arm. An M12 threaded stud stands beside the part. It carries a lift
spring with a spring washer, a brass thrust washer, and an orange nut with a yellow handle, backed off so a strap can
pass under the thrust washer. The robot builds the clamp the way a machinist does: it sets the step block on the mill
table so the tread that matches the part's height is under the strap's heel, threads the strap's open slot onto the
stud under the washer with its toe over the part's edge, and then turns the nut clockwise until the strap is clamped.

Until the nut is tight the lift spring holds the strap's toe 5 to 6 mm off the part, as on a spring-loaded strap
clamp, so a strap that is only laid in place, or a nut that only touches it, does not hold the part. A block one tread
off leaves the strap 6 to 11 degrees out of level. The part has to stay where it was, against both stops.

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

# Scripted demo (full score): the left arm sets the step block on the mill table; the right arm threads the strap
# onto the stud, then pushes the nut's handle round with closed fingers until the strap is clamped
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strap_clamp_level_on_the_step_block.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/strap-clamp-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it,
  and the part's 3 mm limit is measured from that position.
- **Runtime per trial:** the demo finishes in about 76 s of simulated time (about 1,520 steps), and the trial ends once
  the clamp has held for 12 s, a few seconds after the arm has parked; a Dream-1 trial runs up to 3,600 steps
  (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 380 s for the demo and about 720 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 12 s (strap set and clamped, everything still),
  otherwise `score()` at the end of the run.
- **What the demo reads:** the positions of the step block and the strap, the nut's angle and the height of the
  spring washer under the strap, all of which a camera over the table can see. It stops turning the nut when the
  spring washer is almost down to the part's height (3.7 mm, 5.6 N) or the nut stops turning with the strap clamped.
  It pushes the handle by aiming up to 12 degrees past it; if the nut stops before the strap is clamped it leans
  further, up to 24 degrees past. While it turns the nut it holds its fingertips' height from the arm's own joint
  readings.

## Scoring

The part counts as kept if it never moved more than 3 mm from its start at any step of the run (latched, so putting it
back does not clear it). Then, at the end:

1. **Score 0** if the part moved, or the strap is off the table or airborne (its underside more than 1 cm above every
   support: the table, the mill table, the part's top, the spring washer, the tread tops of a block that is itself on
   the table).
2. **1.0** if the strap is set and clamped.
3. **0.5** if the strap is set and not clamped.
4. **0.25** if the strap's toe is over the part (at least 0.5 cm of toe over its edge) and its heel rests on any
   tread (within 3 mm of that tread's top), tilted at most 15 degrees.
5. Otherwise 0.

**Set:** the stud is inside the slot; 1.0 to 2.5 cm of toe is over the part's edge, its underside at most 1 cm above
the part's top; the heel end rests on a tread (underside within 2 mm of its top) whose top is 0 to 4 mm above the
part's top (1.5 mm below is allowed in the simulator). Only the 4 cm tread meets this: the 3 cm tread is 1 cm low and
the 5 cm tread 1 cm high.

**Clamped:** the strap is set, its toe is down on the part (underside within 2 mm of the part's top), the heel end's
underside is 0 to 4 mm higher than the toe end's (about 0 to 2 degrees; 1.5 mm of sag below is allowed in the
simulator), it is tilted at most 3 degrees, the thrust washer is on the strap (gap at most 1 mm), and the strap is
held:

- **On a real table:** after the run, with the robot clear, pull across the strap's heel end (along y) with a spring
  scale up to 5 N. The strap is held if it moves less than 1 mm. Check the washer and the toe overlap from a top
  photo and the toe and heel heights with a ruler.
- **In the simulator:** nothing pulls on the strap, so the scorer reads two things at the end of the run instead.
  1. **The nut did the work.** The nut has turned 110 degrees or more (about 140 seats a level strap), and the thrust
     washer is as far down as that angle puts it, within 3 mm: 0.16 mm per degree, less the 1 mm the lug starts above
     the ramp. The washer rests up against the nut's lug on its own light spring, so the only way it can be lower than
     the thread puts it is that something is pushing it down. A washer pushed onto the strap by a gripper has no
     matching nut angle and is not clamped.
  2. **The clamp force covers the pull.** The spring washer under the strap is pressed down 3 mm or more, which is
     4.5 N or more at 1.5 N per mm. The strap's own weight (1.1 N) presses it about 1 mm. Taking moments about the
     heel (spring 7.0 cm away, nut 6.5 cm, the strap's weight 5.75 cm), the nut presses the strap with
     (4.5 x 7.0 - 1.1 x 5.75) / 6.5 = 3.8 N or more. A pull across the strap is resisted by friction on its top face
     (the washer, 3.8 N) and on its underside (the part, the tread and the spring washer, 3.8 N plus the strap's
     weight): at 0.8 on both faces that is 3.8 x 0.8 x 2 = 6 N, plus 0.9 N from the weight, 7 N. With the top face
     at 0.475, midway between the washer's 0.15 and the strap's 0.8, it is still 3.8 x 0.475 + 5.0 x 0.8 = 5.8 N.
     So by this estimate (it is arithmetic, not a measurement) a clamp that passes in the simulator carries the 5 N
     pull. A strap seated on the part presses the spring 4 mm (6 N). A real M12 nut clamps far harder than this; the
     5 N pull is the least a clamp must hold to count, and the simulated clamp is sized to it because the arm can put
     only about 3 to 4 N on the handle.

The scorer cannot see the grippers, so it does not ask for them to be open. It does not need to: a strap held up scores
0, and a strap, washer or nut that a gripper is pressing does not pass the nut test unless the nut itself has been
turned to the clamping angle, and then the flat step under the lug holds the clamp without the gripper.

Doing nothing scores 0. Pushing the strap along the table cannot score: it has to be lifted onto two supports 4 cm
up. A nut that is near the strap but does not load it scores 0.5 at most, because the lift spring keeps the toe off
the part. A nut tightened before the strap is threaded clamps nothing (with no strap its washer stops 4 mm above the
spring washer) and leaves no room to thread the strap. While the nut is part tightened the washer first presses the
strap flat against the lift spring, which can lift its heel 2 to 3 mm off the tread for a few degrees of turn; a run
that stops there scores 0.25, because the heel is not on the tread.

## How the thread is simulated

A two-finger gripper cannot spin a nut twelve times within the run limit in simulation, and the simulator has no
screw joint, so the thread is built from contact. The nut turns on the stud. A lug under it climbs a helical ramp on
the thrust washer, which is keyed to the stud and can only move down. The ramp is coarse (2.4 cm of travel in 150
degrees of turn, where the M12 thread gives 1.75 mm per turn) and is cut as 80 flat steps of 0.34 mm. Turning the
handle wedges the washer down onto the strap, and the washer cannot push the nut back because the lug rests on a flat
step. The three angles in this file agree: the ramp's whole 150 degrees give 2.4 cm, a level strap needs 1.9 cm of
washer travel plus the slack, which is about 140 degrees, and the scorer asks for 110 or more. The washer hangs 2 mm
below the lug, so the first 15 degrees or so take up that slack before the washer moves. Only the lug's swept toe
meets the ramp. The nut and washer have friction 0.15 (an oiled thread) where the other parts have 0.8. That is less
than the slope of the lug's toe (tan 25 degrees = 0.47), so a lug left part way up a step's edge does not hang there
on friction: it settles back onto the flat below, at most 0.34 mm, and rests. Two things differ from the real nut:
the nut stays at one height while its washer travels, and less than half a turn does the work of about 11 turns. A
light spring (100 N/m) carries the 20 g washer under the nut, as the real nut would. Every shape of the nut starts
clear of the washer's ramp (0.4 mm or more), so the scene builds with nothing overlapping.

## Real-world reset

Turn the nut back up the stud until the thrust washer's underside is 8.1 cm above the table and the handle points to
the robot's left (x 28.0, y 3.4 cm). Put the part against both stops, and the strap and step block at their start
spots (strap at x 30, y -17, step block at x 24, y 15 cm, both with their long side along x). Use an M12 x 1.75 stud,
a flange nut with a 5.5 cm handle arm, a keyed thrust washer, and a lift spring of 1.5 N per mm, which gives 6 N with
its spring washer at the part's height.
