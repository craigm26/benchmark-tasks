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
| SMR | 1.5 in SMR, e.g. FARO's standard 1.5 in SMR (PN 099-03915-xx: 440C stainless housing, 150 g, per FARO's SMR specifications page) | 38.1 mm polished steel ball, open cube-corner face 22 mm across, red ring round it, 150 g | seated in the nest, centre 43.6 mm above the table; face toward +y, 60 deg up |
| Drift nest | Brunson 1.5THDN magnetic drift nest (maker page: aluminium, black anodize, 1.600 in across, 0.5 in tall, SMR centre 1.125 in above its foot) | black anodized aluminium cup 40.6 mm across, rim 12.7 mm above its foot, a magnet under a 3-point land | foot on the plate at x 24, y -6 cm |
| Base plate | painted steel plate | 220 x 300 x 15 mm, blue-grey | fixed, centred x 27, y 0 cm |
| Robot base | base of a UR3e-size arm (no arm fitted) | cast flange 128 mm across, 10 mm; base joint housing 90 mm across, 90 mm tall; black J1 cover 30 mm | fixed, centred x 31, y +8 cm on the plate |
| Beam post | steel rod on a weighted foot, with a pen laser pointer in a yoke | rod 12 mm, foot 70 mm across; pointer 16 mm across, red emitter 92 mm above the table | fixed, x 42, y -6 cm; aims back at the ball centre |

The nest's magnet is modelled as a 3,000 N/m spring holding the ball on its land: 9 N to lift it 3 mm. Turning it
in the seat takes 0.02 N m about either axis, which is 1.05 N at the ball's surface. Brunson does not publish the
pull or the turning torque, so these are estimates. "Validate the stand-in" below says how to measure both on the
real nest, and "Fidelity" gives the range over which the result does not change. The pinch uses the gripper's
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
   finger leaves. Then it brings the fingertip back to the ball point it is touching now, so the lead does not carry
   the ball on. It eases the press off, backs 3 cm out along the surface normal and reads the tilt again. If the ball
   sprang back to more than 4 degrees short of the goal, it rolls again from 45 degrees behind the top (up to three
   strokes).
3. It rises, opens and swings back to its ready pose, then waits 3 s.

About 49 s (975 steps at 20 Hz). In a MuJoCo contact replay (with the scorer's finger-pad reads filled from the
replay), the pinch turned the ball -90.3 degrees in one stroke and the roll left it at 43.7 degrees down (aim error
1.3 degrees), with no lift and no touch of the face. It scored 1.0. The Isaac Sim run is the score of record.

## Fidelity (what the simulation approximates)

- The SMR, nest, plate and post are built at real size from procedural round geometry. The ball collides as a full
  38.1 mm sphere centred on both hinges, so neither carries a gravity torque. It is drawn cut flat at the face plane
  (15.55 mm from the centre) with a black 22 mm face and a red ring. There are no mirrors: the face is a black disc.
- The seat is a chain of joints about the ball's centre: a 5 mm lift slide, then a yaw hinge (vertical) and a tilt
  hinge (horizontal, perpendicular to the face's compass direction). The ball's centre cannot wander on the land, and
  rocking out over one land point is not modelled, and nor is a turn about the face axis (it does not change the
  aim). The magnet is a 3,000 N/m spring to the land, and the seat friction is a joint hold of 0.02 N m on each
  hinge. Brunson publishes neither the pull nor the turning torque, so both are estimates. "Validate the stand-in"
  says how to measure them on the real nest.
- Sensitivity (MuJoCo replays of the scripted demo; log in the build notes): every case scored 1.0 with no lift.
  The cases were the spring at 1,500, 3,000 and 6,000 N/m (4.5 to 18 N at 3 mm) and the seat hold at 0.01, 0.02,
  0.04 and 0.06 N m. Ball masses were 150 g (FARO standard SMR), 195 g (FARO heavy-duty SMR) and 221 g (the solid 440C
  stand-in). At 150 g the aim error ended between 0.8 and 1.7 degrees, and at 195 g it was 0.7 degrees. At 221 g the
  heavier ball rolled on about 9 degrees further, ending at 7.9 to 8.0 degrees: still inside the 10 degree band, but
  the one material difference the stand-in brings, as a policy will feel it.
- The three land balls are drawn but do not collide. The cup (40.6 mm across, rim 12.7 mm, as Brunson lists it)
  collides, with 1.2 to 1.5 mm round the ball.
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
- **Runtime per trial:** the demo takes about 49 s of simulated time (975 steps); a Dream-1 trial runs up to 3,600
  steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 580 s for the demo and about 2680 s for a full Dream-1 trial.
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

**Real trial** (on the steel-ball stand-in below; no tracker needed, the pointer and the painted rings are the
scoring aid):

- **Aim:** switch the pointer on. Its beam is aimed at the ball centre, so the spot lands where the ball's surface
  faces the pointer, and its distance from the face centre gives A directly. The rings painted on the ball sit 3.31,
  6.52 and 9.53 mm (plan radius) from the face centre: 10, 20 and 30 degrees. A top photo gives the face's compass
  direction for the 0.25 stage (the aim stem from fabricate.py, set on the face after the run, makes it plain).
- **Lift (calibrated reference):** two side cameras on small tripods, lenses level with the ball's top (table + 62.7
  mm), each with a lit white card behind the ball. One stands 40 cm to the ball's right, looking along +y. The other
  stands on the bench behind the arm bases, 40 cm behind the ball, looking along +x. Its lens is 5 cm left of the
  ball's line, so the post's rod appears about 15 mm to the side of the ball's top. Before the run, set the printed lift calibration cap on the seated ball. Its flat top is
  exactly 3.0 mm above the ball's top. In one frame from each camera, mark the cap's top edge as a horizontal line on
  the screen (or on a transparent overlay), then remove the cap. During the run the ball's top edge is its highest
  point whatever way it is turned, because the stand-in is a full sphere. If that edge reaches the line in either
  view for 0.2 s running (6 frames at 30 fps), the ball has lifted 3 mm or more and the run scores 0. A fingertip
  passing in front of the top in one view leaves the other view clear: the pinch pads sit on a horizontal line
  through the centre, and the rolling fingertip runs in the face's vertical plane. A 1080p frame 15 cm wide reads
  about 0.14 mm a pixel, so 3 mm is about 20 pixels. (On a real SMR the open face cuts the top of the ball when the
  face points within 30 degrees of straight up. Its top edge then reads up to 3.5 mm low, which is why the stand-in
  is a full sphere.)
- **Touch:** the side and top videos show a fingertip entering the black face's outline.

## Real-world build and reset

The trial uses the real nest and a same-steel stand-in for the SMR. A real 1.5 in SMR (for example the Metricop 1.5
in SMR, 440C housing, $1,282.99 on the maker's page) works the same way, but a gripper pad must not touch its optics.

Bill of materials (US prices from the seller pages where named):

| Item | Source | Price |
|---|---|---|
| Brunson 1.5THDN magnetic drift nest | Karaco (karaco.com; listed at $31.00 and $37.50) | $31 to $38 |
| SMR stand-in: 1-1/2 in (38.1 mm) 440C stainless steel ball, magnetic | Zoro Select 20523 (zoro.com, $43.89) or McMaster-Carr 9529K51 (Grade 100) | about $44 |
| Black and fine white paint pens, red paint pen | art or office supplier | about $10 |
| Pen laser pointer, class 2 (under 1 mW), red | office supplier | about $15 |
| Spring scale, 0 to 10 N in 0.1 N steps, and strong thread (for the validation) | hardware or school-science supplier | about $12 |
| Steel plate 220 x 300 x 15 mm, or 18 mm plywood painted blue-grey | steel stockist or home centre | about $25 |
| Robot base stand-in: 128 mm disc and 90 mm tube (or the real robot's base) | printed or plumbing parts | about $10 |
| Post: 12 mm steel rod, 70 mm weighted foot, printed yoke | home centre | about $8 |
| Printed guides, gauges and collar (fabricate.py), PLA or PETG; two M3 x 16 screws and nuts | any FDM printer | about $5 |
| Two phone cameras on small tripods (lift views), white card and a lamp | already on hand in most labs | $0 |
| **Total** | | **about $160 to $170** |

**Why the stand-in is a 440C ball.** FARO's standard 1.5 in SMR has a 440C stainless steel housing, 38.1 mm across,
and weighs 150 g. The 440C ball has the same diameter and the same steel. Its surface finish is a ground, polished
bearing finish like an SMR's. A real SMR is solid steel below its centre, and its optics sit in the upper half. The
nest's magnet sits under the land, so its field passes only through that solid lower half. The stand-in's lower half
is the same: same radius, same steel, same three land contacts. So the magnet should pull both balls equally. The one
known difference is weight: the solid ball is 221 g, 0.70 N heavier than 150 g. Turning friction is proportional to
the land's normal load (magnet pull plus weight). For any magnet pull of 5 N or more, the stand-in's turning torque
is therefore within 12 % of the real SMR's, and its pull-off force differs only by its 0.70 N of extra weight.

**Validate the stand-in (10 minutes, required before the first real trial).** Brunson publishes neither the pull nor
the turning torque, so measure both on the trial's nest, for the stand-in and, where one is available, for a real 1.5
in SMR:

1. Clamp the two printed collar halves round the ball's equator (two M3 screws), with the face pointing straight up so
   the collar is clear of it.
2. **Pull-off:** hook the spring scale to the collar's top eye and pull straight up, slowly. Record the peak reading
   as the ball leaves the land. Subtract the collar's weight (weigh it once). Repeat 5 times and take the median.
3. **Turning torque:** with the ball seated, tie the thread to the collar's lug, 50 mm from the centre. Pull it level
   and square to the lug until the ball starts to turn. The torque is the reading x 0.050 m. Take the median of 5.
4. **Accept** the stand-in if its median pull and torque are within 15 % of the real SMR's. Record all four values in
   a reset log kept with the rig. Without a real SMR, check that the pull is at least 5 N, which brings the 12 %
   bound above into play. If the pull is under 5 N, or more than twice the simulation's 9 N, the nest's magnet is weak
   or not as specified: replace the nest before trials.
5. **Compare with the simulation:** the sim's magnet gives 9 N at 3 mm, and its seat holds 0.02 N m. The sensitivity
   sweep under "Fidelity" shows the same result anywhere from half to double each of these.

Assembly: screw the plate to the bench and set the nest at its mark (it is held by its flat back with double-sided
tape; outline it). Set the face stencil on the ball and paint the 22 mm face black through it; once dry, trace the
10, 20 and 30 degree ring guides' hole edges with the white pen, and paint a 2 mm red ring round the face. Stand the
post 180 mm straight ahead of the ball centre and set the pointer in its yoke so its spot lands on the ball's centre
mark with the face aimed at it. Stand the robot base stand-in on the plate. Set up the two lift cameras and calibrate
them with the lift cap as above.

Reset and cleaning: set the start wedge on the plate pointing +y. Turn the ball back by hand (nitrile gloves, holding
the ball, not the face) until the aim stem, set on the face, lies along the wedge's 60 degree slope. Remove the stem
and seat the ball on its land. Wipe the ball with a lint-free cloth and isopropyl alcohol if a pad marked it; blow
dust and steel filings off the land with canned air (FARO's SMR care notes warn that magnetic tooling picks up steel
debris). Nothing is consumed or wet. The pointer is class 2: the beam runs low across the bench, away from anyone's
eyes, and is switched on only to score.

Print files: `python3 fabricate.py` writes, from the scene's dimensions, `face_stencil.stl`,
`ring_guide_10.stl`, `ring_guide_20.stl`, `ring_guide_30.stl`, `lift_cal_cap.stl` (the 3.0 mm lift reference),
`aim_stem.stl`, `start_wedge.stl` and `collar_half.stl` (print two). None of them carries the turning mechanism: the
mechanism is the bought nest and the bought steel ball.

## Files

- `turn_laser_tracker_smr_in_drift_nest_to_face_tracker.py`: the task file (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (geometry and joints; closed-loop demo).
- `fabricate.py`: STL writer for the painting guides, the lift calibration cap, the reset gauges and the measuring
  collar.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
