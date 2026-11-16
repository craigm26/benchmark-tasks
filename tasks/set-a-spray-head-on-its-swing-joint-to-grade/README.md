# Set a pop-up spray head on its three-elbow swing joint: over the stake, plumb, its top flush with grade

Every spray head on a new irrigation zone stands on a swing joint: three swivel elbows and a short nipple between the
buried lateral's tee and the head. Landscaping and groundskeeping crews (O*NET 37-3011.00, about 1.0M US workers)
turn the swivels until the head stands plumb over its stake with its top flush with finished grade, let go to check
it, then backfill. Here the head starts plumb but 35.6 mm proud, elbow 2 pointing along the trench. Only one
combination of the three swivels is plumb, flush and over the stake: swing elbow 2 back 90 degrees, then lay the
nipple over 40 degrees toward the stake while elbow 3 turns back 40 degrees, the hand following the 152 mm arc the
linkage allows and holding the body upright all the way down. Pushing the head straight down tilts it 40 degrees,
and laying the nipple the other way is flush and plumb but 195 mm off the stake. This differs from folding a
two-link wall arm flat to its stops (a planar fold on parallel vertical hinges, gravity out of play): the three
swivels here are not parallel, gravity loads them, and the target is mid-travel, set by the grade boards and the
stake rather than by any stop.

## Parts (real size, real material)

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left); h is height above the
table top. The trench is shown in section, open toward the robot and at its y -15 end, so the swing joint is in view.

| Part | Real dimensions and material | Where |
|---|---|---|
| Sand floor | Sand, 0.3 cm deep | x 21.5 to 36.2, y -15 to +20 |
| Far bank | Soil, 5.8 x 35 cm, 23.8 cm tall | x 36.2 to 42.0 |
| Grade boards (2) | Pine 3.8 x 1.9 cm, 35 cm long; tops at finished grade, h 25.68 | Far board on the bank (x 36.2 to 40.0); near board on two 1.9 cm pine stakes (x 17.7 to 21.5, stakes at y -12 and +17) |
| Mark | 2.5 cm orange stripe painted across both boards, and a 1.9 x 1.9 cm pine grade stake with an orange top, 4 cm above grade | Stripe and stake at y -9.77; stake on the bank at x 41.0 |
| Lateral and tee | White 3/4 inch PVC pipe, 26.7 mm OD, along y; tee 33 mm OD, side outlet toward +x; freshly solvent welded, fixed | Pipe axis x 25.5, h 1.84; outlet to x 26.8 |
| Elbow 1 and nipple | Grey polyethylene 1/2 inch swing elbow (25 mm hubs) on the tee outlet, swivels about x; 21.3 mm nipple up to elbow 2, 152 mm centre to centre; 30 g | Swivel axis x 28.5, y 0, h 1.84 |
| Elbow 2 | Grey PE elbow, swivels about the nipple's axis; outlet out to +y; 12 g | On the nipple's top |
| Elbow 3 | Grey PE elbow, swivels about elbow 2's outlet, 32 mm from the nipple's axis; 12 g | Carries the head |
| Spray head | 2 inch pop-up body, black ABS, 46 mm OD, 102 mm tall on a 24 mm inlet; pale grey cap 56 mm OD, 10 mm tall, dark stem and nozzle slot; 90 g | Body base 20 mm and cap top 122 mm above elbow 3 |

Each swivel turns stiffly in steps, as a real swing joint's threads and O-rings hold where they are set: elbow 1
every 1 degree (40 N m/rad detent spring, about 0.35 N m to move one step), elbow 2 every 2 degrees (6 N m/rad, about
0.10 N m), elbow 3 every 1 degree (12 N m/rad, about 0.10 N m). Stops: elbows 1 and 3 at +-60 degrees, elbow 2 at
+-100 degrees. Under the head's weight elbow 1 sags at most 0.2 degrees and elbow 3 0.3 degrees, inside half a step, so each
holds its detent. One elbow-1 step moves the cap 1.7 mm up or down at the set pose.

Start (all swivels 0): nipple upright, elbow 2 along +y, head plumb at x 28.5, y 3.2, cap top h 29.24 (35.6 mm proud
of grade, 134 mm from the mark). Set pose: elbow 1 +40, elbow 2 -90, elbow 3 -40 degrees; cap centre at x 31.7,
y -9.77, top h 25.68, 17 mm clear of the far board. Along the demo's path the chain's parts stay at least 5.6 mm
apart (elbow 3 past the nipple's top) and 2.9 mm or more off the trench (elbow 1 above the sand).

## What the scripted demo does

The right arm does the job; the left arm has no plan and holds its ready pose. The grip leans the fingers 46 degrees
from vertical with the pads on the 46 mm housing, pad centre 35 mm below the cap top.

1. It comes in over the open -y end of the trench, turns the grip to yaw 90 degrees (pads squeezing along x, fingers
   leaned toward +y), opens to 64 mm, trims its height on its encoders and comes down at 20 mm/s over the cap to the
   housing, closes and checks the grip on the gripper encoder (pads stopped at least 40 mm apart), retrying once if
   they closed on air.
2. Swing: it carries the body 32 mm round the nipple's axis to +x, yawing the wrist from 90 to 0 degrees in step, at
   15 deg/s, until elbow 2 reads -90 within 1 degree. The nipple stays upright and the body plumb and at height.
3. Lowering: it steps elbow 1 from 0 to 40 degrees at 5 deg/s, the hand on the 152 mm arc (y = -15.2 sin a,
   h falling 15.2 (1 - cos a) cm), the leaned fingers holding the body upright so elbow 3 follows at -a. It then
   checks from the body's pose that the cap is within 1.5 mm of grade and under 1 degree of plumb, and if not, nudges
   elbow 1 one step and back.
4. It opens to 64 mm and waits 1 s while the detents take the load.
5. It rises at 20 mm/s until the pads are above the cap, moves clear toward the robot and parks at its ready pose.
   The last frames show the cap level with both boards at the orange stake, the nipple leaning toward it, both
   grippers open and both arms parked.

The demo reads the swing joint's three swivel angles from the simulator (and from them the body's pose) to aim, to
servo and to confirm, plus the fixed layout (object_jitter=0), its joint encoders and the gripper gap. Each control
step it compares the planned grasp point with the one measured from the swivels, folds the error into an offset on
the hand command (at most 12 mm) and waits while the body lags more than 6 mm. Tool speed stays under 0.09 m/s
(descent and rise 20 mm/s, swing about 8 mm/s, lowering about 13 mm/s), inside the declared 0.10 m/s limit. It takes
47 s (939 steps).

In a MuJoCo contact replay used offline (not Isaac), the demo left the swing joint at elbow 1 +40.2, elbow 2 -90.2 and
elbow 3 -39.0 degrees: cap 0.3 mm below grade, 1.2 degrees from plumb, 2.9 mm from the mark, and it did not move after
release. The arm touched nothing but the head. (That replay has no live release check, so release was taken from the
fingers' path; Isaac checks it directly.)

## Fidelity

Rigid bodies. A real swing joint holds by thread and O-ring friction; here each swivel is a hinge with detent springs
(steps of 1, 2 and 1 degrees, about 0.35, 0.10 and 0.10 N m to move one step), because a plain friction hold creeps
under gravity in this simulator. The lateral and tee are fixed. No backfill, water or pop-up stem (the stem and
nozzle slot are look only). The trench is shown in section and shortened to 35 cm so the arm can see and reach the
joint. The simulator scores the last step without opening the grippers: a head still held, or held up in the air,
is not released and scores 0.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Set the spray head over the orange stake mark, standing plumb with its top flush with the grade boards, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_spray_head_on_its_swing_joint_to_grade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/spray-dream

# Scripted demo (full score): the right arm sets the head; the left arm stays at its ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_spray_head_on_its_swing_joint_to_grade.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/spray-demo
```

- **Seed and layout:** `-E object_jitter=0` is required (the trench, lateral and swing joint are fixed in place).
- **Runtime per trial:** the demo's motion takes about 47 s (939 steps), and the run ends 8 s after full score holds;
  a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 160 s for the demo and about 340 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` (full score) has held for 8 s, otherwise `score()` at the end.

## Scoring

Measured after the robot lets go:

- **h:** cap top centre minus grade (a straightedge across both boards and a rule).
- **Tilt:** the body's axis from vertical (a torpedo level on the body).
- **c:** plan distance from the cap's centre to the mark, x 31.7, y -9.77 (tape from the stake).
- **1.0:** |h| 3 mm or less, tilt 3 degrees or less, c 10 mm or less.
- **0.5:** c 10 mm or less, tilt 3 degrees or less, |h| over 3 and up to 6 mm (one elbow-1 step off is 1.7 mm and still
  scores 1.0; two steps, 3.3 to 3.5 mm, score 0.5).
- **0.25:** c 20 mm or less, tilt 6 degrees or less, |h| 10 mm or less.
- **0** otherwise: the start (h +35.6 mm, c 134 mm), the head pushed straight down with elbow 2 along the trench
  (tilt 40 degrees), the nipple laid the other way (c 195 mm), elbow 2 swung to the front (c 64 mm), lowered halfway.
- **0 for the whole trial** if a finger pad ever touches the freshly glued lateral or tee (video), or unless at the end
  the head is let go (every finger pad 3 cm or more from the swing joint) and no swivel turns more than 0.5 degrees in
  the last 1 s.

In simulation h, tilt and c come from the three swivel angles; release from every finger pad point at least 3 cm
from the swing joint; the touch rule from the finger pad points every step (within 2 mm of the pipe's surface).
Doing nothing scores 0.

## Real-world reset

Turn the three swivels back by hand: nipple upright (a level on its side), elbow 2 along the trench toward the robot's
left, head plumb. Check the cap top reads 35.6 mm above a straightedge laid across the grade boards. Mark the stake
position on the bank so it goes back to the same spot.

## Files

- `set_a_spray_head_on_its_swing_joint_to_grade.py`: the task file (scene, `score()`, `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (jointed parts and geometry, the closed-loop demo
  controller, geometry helpers).
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
