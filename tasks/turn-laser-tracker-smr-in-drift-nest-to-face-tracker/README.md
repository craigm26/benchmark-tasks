# Re-aim a laser tracker SMR in its magnetic drift nest at the tracker's new station, never lifting it

Before a new robot arm is calibrated, its base frame and the cell's fixtures are located with a laser tracker. The
targets are spherically mounted retroreflectors (SMRs): steel balls with an open cube-corner mirror inside, left
seated in magnetic drift nests on the base plate. An open cube corner only returns the beam within about 25 to 30
degrees of its axis. So each time the tracker moves to a new station, every SMR has to be turned in its nest to face
it. It must stay on its 3-point land, because lifting it shifts the measured centre. The open optics must never be
touched, because a fingerprint on the mirrors spoils the return. In this task the SMR faces left and 60 degrees up, toward the
tracker's old station. The tracker now stands behind a beam post straight ahead, 15 degrees above the ball. The robot
must turn the ball about its own fixed centre in two axes, using two different contacts. First it pinches the ball
across the one line clear of the face (the tilt axis) and turns the wrist, which turns the face's compass direction.
Then it rolls the ball down with a closed fingertip on its back, past the face's rim, until the face points at the
pointer. Finally it lets go. A robot that can do this can handle the metrology targets of robot building and cell commissioning by itself.

## Parts (real size and material)

| Part | Real product | Size and material | In the scene |
|---|---|---|---|
| SMR | 1.5 in red-ring SMR (e.g. the 1.5 in red-ring SMR sold by Survey Pro, about $993; FARO lists its 1.5 in break-resistant SMR at 195 g) | 38.1 mm polished steel ball, open cube-corner face 22 mm across, red ring round it, 150 g | seated in the nest, centre 43.6 mm above the table; face toward +y, 60 deg up |
| Drift nest | Brunson 1.5THDN magnetic drift nest (Karaco, about $31) | black oxide steel cup, 44 mm across, rim 13 mm above its foot, SMR centre 28.6 mm (1.125 in) above the foot, magnet under a 3-point conical land | foot on the plate at x 24, y -6 cm |
| Base plate | painted steel plate | 220 x 300 x 15 mm, blue-grey | fixed, centred x 27, y 0 cm |
| Robot base | base of a UR3e-size arm (no arm fitted) | cast flange 128 mm across, 10 mm; base joint housing 90 mm across, 90 mm tall; black J1 cover 30 mm | fixed, centred x 31, y +8 cm on the plate |
| Beam post | steel rod on a weighted foot, with a pen laser pointer in a yoke | rod 12 mm, foot 70 mm across; pointer 16 mm across, red emitter 92 mm above the table | fixed, x 42, y -6 cm; aims back at the ball centre |

The nest's magnet is modelled as a 3,000 N/m spring holding the ball on its land: 9 N to lift it 3 mm. Turning it
in the seat takes 0.02 N m about either axis, which is 1.05 N at the ball's surface. The pinch uses the gripper's
squeeze; the roll needs about 2 N of fingertip press. Both are far under the 40 N per finger limit.

## What the scripted demo does and what it reads

The demo reads the SMR's yaw and tilt joint values (the nest Mech's joints) from the simulator to plan each turn,
plus its joint encoders and gripper opening; every position comes from the fixed layout (object_jitter=0). The left
arm stays at its ready pose.

1. **Yaw by a pinch on the tilt axis.** The right hand comes over the ball with its pads lined up along the tilt
   axis (perpendicular to the face's direction, so neither pad comes near the face), corrects its command on its
   encoders, and descends to 5 mm above the ball's centre (pad tips 6 mm above the cup rim). It closes and checks
   by its opening that the pads stopped on the 38 mm ball, adds a 2 mm downward bias that keeps the ball on its land, and turns the
   wrist at most 0.3 degrees a step until the ball reads its goal yaw (-90 degrees). On a miss it opens, rises and
   tries again (up to three strokes).
2. **Tilt by a fingertip roll.** It opens, rises, closes the fingers and comes down behind the ball. The closed
   fingertip touches it 35 degrees behind top dead centre, in the face's vertical plane and 65 degrees off the face axis.
   It presses in until it feels contact as arm lag, then moves along an arc about the tilt axis toward the face side.
   It keeps a small lead over the ball point it first touched, and lets that lead grow, up to 20 degrees, while the
   ball lags. It stops 5 degrees past the goal elevation (15 degrees), because the ball springs back a little when the
   finger leaves.
3. It eases the press off, backs 3 cm out along the surface normal, rises, opens and swings back to its ready pose,
   then waits 3 s.

About 47 s (944 steps at 20 Hz). In a MuJoCo contact replay (with the scorer's finger-pad reads filled from the
replay), the pinch turned the ball -90.5 degrees in one stroke and the roll left it at 43.7 degrees down (aim error
about 1.4 degrees), with no lift and no touch of the face. It scored 1.0.

## Fidelity (what the simulation approximates)

- The SMR, nest, plate and post are built at real size from procedural round geometry. The ball collides as a full
  38.1 mm sphere centred on both hinges, so neither carries a gravity torque. It is drawn cut flat at the face plane
  (15.55 mm from the centre) with a black 22 mm face and a red ring. There are no mirrors: the face is a black disc.
- The seat is a chain of joints about the ball's centre: a 5 mm lift slide, then a yaw hinge (vertical) and a tilt
  hinge (horizontal, perpendicular to the face's compass direction). The ball's centre cannot wander on the land, and
  rocking out over one land point is not modelled. The magnet is a 3,000 N/m spring to the land, and the seat
  friction is a joint hold of 0.02 N m on each hinge. The real nest's pull and turning torque are not published, so
  both values are estimates to be measured on a real nest with a force gauge. The score does not hinge on them.
- The three land balls are drawn but do not collide. The cup collides, with 1.5 mm round the ball.
- The pointer is the beam stand-in for the tracker, which in a real cell stands metres away; the aim error is
  measured to the pointer's emitter.
- The simulator scores the last step without opening the grippers, while a real trial opens them before measuring.
  So any finger pad point within 3 cm of the ball at the end scores 0, and the ball must be still for the last 1 s.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="The tracker is now behind the beam post. Turn the SMR in its nest to face the post's pointer without lifting it or touching its optics, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_laser_tracker_smr_in_drift_nest_to_face_tracker.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/smr-nest-dream

# Scripted demo (closed loop, runs demo_script through closedloop.py)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_laser_tracker_smr_in_drift_nest_to_face_tracker.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/smr-nest-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the SMR's start direction is fixed.
- **Runtime per trial:** the demo takes about 47 s of simulated time (944 steps); a Dream-1 trial runs up to 3,600
  steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 560 s for the demo and about 2750 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

Aim error A is the angle between the SMR's face axis and the line from the ball centre to the pointer's emitter,
averaged over the last 1 s.

- **0 for the whole run** if the ball is ever 3 mm or more off its land for 0.2 s without a break, or a finger pad
  point enters the open face 2 steps running. The face zone runs within 13 mm of the face axis, from 12 to 30 mm out
  from the ball centre.
- **Otherwise 0** unless every finger pad point of both grippers ends 3 cm or more from the ball for the last 1 s,
  and yaw and tilt each change by under 0.5 degrees over that second.
- **Then:** 1.0 if A is 10 degrees or less; 0.5 if A is 20 or less; 0.25 if A is 30 or less, or if the face's
  compass direction is within 10 degrees of the bearing (the yaw turn is done but not the elevation); else 0.

The start A is 77 degrees, so doing nothing scores 0. The yaw turn alone leaves 45 degrees (0.25). The elevation roll
alone leaves 86 degrees (0). A pinch straight across the face, a tug that unseats the ball, or a grip kept at the end
scores 0.

**Real trial:** switch the pointer on. The spot lands on the face's printed rings, 2.7, 5.7 and 9.0 mm from its centre
(10, 20 and 30 degrees). A side video against a backlit white card shows any lift: a 3 mm gap under the ball shows as
a light line under it. The side and top videos show a fingertip entering the black face's outline. A top photo
gives the face's compass direction for the 0.25 stage.

## Real-world build and reset

Bill of materials (approximate US prices):

| Item | Source | Price |
|---|---|---|
| Brunson 1.5THDN magnetic drift nest | Karaco (karaco.com) | about $31 |
| Printed replica SMR (fabricate.py), PETG, with a 1 in chrome steel bearing ball as its core | any FDM printer; bearing ball from McMaster-Carr or a bearing supplier | about $10 |
| 1.5 in red-ring SMR (optional, in place of the replica) | Survey Pro | about $993 |
| Pen laser pointer, class 2 (under 1 mW), red | office supplier | about $15 |
| Steel plate 220 x 300 x 15 mm, or 18 mm plywood painted blue-grey | steel stockist or home centre | about $25 |
| Robot base stand-in: 128 mm disc and 90 mm tube (or the real robot's base) | printed or plumbing parts | about $10 |
| Post: 12 mm steel rod, 70 mm weighted foot, printed yoke | home centre | about $8 |
| **Total (replica route)** | | **about $99** |

Assembly: screw the plate to the bench and set the nest at its mark (magnetic base; outline it with tape). Press the
bearing ball into the printed shell's bore and glue on the black printed face (aim rings up). Paint a red ring round
the face. Stand the post 180 mm straight ahead of the ball centre and set the pointer in its yoke so its spot lands on
the ball's centre with the face aimed at it. Stand the robot base stand-in on the plate.

Reset: turn the SMR back by hand (nitrile gloves, holding only the ball) until its face points left and 60 degrees up,
checked with the printed start gauge, and seat it on its land. Wipe the ball with a lint-free cloth if a pad marked it.
Nothing is consumed or wet. The pointer is class 2: the beam runs low across the bench, away from anyone's eyes, and
is switched on only to score.

Print files: `python3 fabricate.py` writes `smr_replica_shell.stl` (the 38.1 mm shell cut at the face plane, with a
1 in core bore), `aim_face.stl` (the 22 mm face with the 10, 20 and 30 degree rings) and `start_gauge.stl` (the 60
degree reset wedge) from the scene's dimensions.

## Files

- `turn_laser_tracker_smr_in_drift_nest_to_face_tracker.py`: the task file (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (geometry and joints; closed-loop demo).
- `fabricate.py`: STL writer for the printed replica SMR, its aim face and the start gauge.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
