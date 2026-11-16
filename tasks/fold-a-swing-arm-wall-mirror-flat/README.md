# Fold an extended wall mirror flat

An extendable shaving mirror hangs on the tiled wall over a basin on two arm links, swung out over the counter. The
robot must fold it away: both links back flat against the wall, the round head parallel to the tile, and then let go.
People do this after every shave so the arm stops sticking out over the basin. The mirror is a passive two-link chain
the robot can steer only through its head, and the elbow has been left bent one way by chance: pushed straight back,
the shoulder and elbow both turn toward the wall and link 2 and the head jam against the tile, unfolded. The fold
that works carries the head round in front of the wall plate the way the elbow already bends, and the glass must be
squared to the wall first, because near the folded pose the tile stops a turned head 39 degrees from parallel.
The same skill folds monitor arms, wall TV arms and wall lamp arms.

**The trap, measured.** In the MuJoCo contact replay used to check this task offline (not yet in Isaac), a probe
gripped the head as the demo does and pushed it straight toward the tile at 10 mm/s. The shoulder turned from +60 to
+74 degrees and the elbow from +17 to +34, both toward the wall, while the head turned on its swivel to its -150
degree stop. After 70 mm of push the bezel's rim reached the tile (the touch rule fired, score 0) and the chain
jammed there unfolded: the hand fell 65 mm behind its command and the links did not fold.

## Parts (real size, real material)

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left); heights are above the
counter (the station's wood table top).

| Part | Real dimensions and material | Where |
|---|---|---|
| Tile wall | White glazed ceramic tile panel, 44 x 24 cm, 2 cm thick, grey grout lines every 10 cm; fixed | Face at x 32.5, centred on y 0, standing on the counter |
| Wall plate and bracket | Matt black brass, round plate 8 cm across, 1.5 cm thick, screwed to the tile; 22 mm shoulder knuckle | Plate centre y 0, 6.9 cm up; shoulder hinge axis at x 29.5, y 0 (3 cm in front of the tile) |
| Link 1 | 18 mm black brass tube, 11 cm axis to axis, 20 mm elbow knuckle, 90 g | Vertical shoulder hinge, link 6.9 cm up |
| Link 2 | 18 mm black brass tube, 10 cm axis to axis, 22 mm swivel boss 18 mm tall, 80 g | Vertical elbow hinge 2.6 cm higher than the shoulder's |
| Head | Round two-sided mirror: 15 cm across, 2.5 cm matt black brass bezel, silvered glass 13.6 cm across on both faces (0.6 mm proud), chrome trim ring, 14 mm neck; 0.45 kg | Vertical swivel hinge; head centre 3 cm in front of the swivel axis, 9.5 cm up; rim 2 cm above the counter |

Hinge friction: shoulder 0.25 N m, elbow 0.15 N m, swivel 0.10 N m. A real mirror's hinges are set with their screws
and checked with a spring scale pulled at right angles: link 1 starts to move at 2.3 N at the elbow knuckle (11 cm
out), link 2 at 1.5 N at the swivel (10 cm out), the head at 1.3 N on the bezel's side, 7.5 cm from the swivel axis.

Angles below are relative, link to link: 0 is a link pointing straight out at the robot, + is counter-clockwise from
above. Start: shoulder +60 (elbow at x 24.0, y -9.5), elbow +17, bent toward the wall (swivel axis at x 21.8,
y -19.3), swivel -127, so the glass is turned 50 degrees from facing the robot toward the robot's left (head centre
at x 19.8, y -17.0). Stops: shoulder +-90 (link along the wall), elbow +-178, swivel +-150. The folded pose the demo
makes is shoulder -90, elbow +178, swivel -88: elbow axis 3.0 cm and swivel axis 3.35 cm from the tile, head centre
at x 26.2, y +1.0, glass front 7.66 cm from the tile, bezel back 10.5 mm clear of link 1. The mirror image (+90, -178,
+88) scores the same. In the task file every shape is laid at the start pose, so each joint value is the angle minus
its start angle.

Why the order matters, measured offline from the colliders at the folded pose: the head can turn only 38.8 degrees
one way from parallel before it meets the tile, and 9.8 degrees the other way before it meets link 1. A head still
turned 50 degrees cannot reach the folded pose; it has to be squared first. Along the demo's path the head stays at
least 26 mm from the tile, and the bezel passes the elbow knuckle 4 mm clear while it is squared.

## What the scripted demo does

The right arm does the whole fold; the left arm has no plan and holds its ready pose.

1. It moves over the head, pads turned along the glass normal, opens to 45 mm, trims its height on its encoders and
   comes down to a top-down pinch across the 25 mm bezel, its tool point 2 cm below the top of the rim. It checks
   the grip on the gripper encoder (pads stopped at least 20 mm apart) and retries once if they closed on air.
2. Squaring: it turns the head +50 degrees about the swivel axis (tool point on a 3 cm arc round it), links held.
3. Swing: it carries the head across in front of the wall plate along a planned path, the shoulder turning from +60
   to -90 while the elbow folds from +17 to +140 the way it is already bent, glass kept parallel to the wall.
4. It checks the shoulder sits on its -90 stop, then makes the last fold as an arc about the measured elbow axis
   (elbow +140 to +178) at about 20 mm/s.
5. It opens to 45 mm, waits, slides the open pads 10.5 cm off the head along the glass plane, rises, draws back
   toward the robot and swings to its ready pose, and holds there. The last frame shows the black arm folded flat on the white tile, glass
   parallel, grippers open.

The demo reads the mirror's hinge angles from the simulator (and from them the head's pose) to correct its path, and
its own joint encoders and gripper gap. Each control step it compares the planned head centre with the measured one,
folds the error into an offset on the hand command (at most 15 mm) and holds the path while the head lags more than
8 mm. Tool speed stays under 0.10 m/s (squaring and swing slower), inside the declared 0.15 m/s limit.

In the MuJoCo contact replay used offline the demo scored 1.0: the mirror ended at shoulder -90, elbow +178 and
swivel -88.7 degrees, and the arm touched nothing but the head.

## Fidelity

Rigid bodies. The mirror is mounted low (rim 2 cm above the counter) so the arm reaches the whole fold; all three
hinges are vertical, so height does not change the fold. The head's tilt pivot is omitted (the kit chains three
joints); tilt turns about the horizontal diameter that scoring measures, so it cannot change the score. Hinge
friction is a Coulomb hold (0.25, 0.15, 0.10 N m) on vertical hinges, which gravity does not load. The glass is a
rigid disc. The simulator scores the last step without opening the grippers, and a mirror still held scores 0.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Fold the wall mirror away: both arm links back flat against the wall and the mirror parallel to it, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fold_a_swing_arm_wall_mirror_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/mirror-dream

# Scripted demo (full score): the right arm folds the mirror; the left arm stays at its ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fold_a_swing_arm_wall_mirror_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/mirror-demo
```

- **Seed and layout:** `-E object_jitter=0` is required (the mirror and tile are fixed to the wall).
- **Runtime per trial:** the demo's motion takes about 41 s (816 steps), and the run ends 8 s after success holds;
  a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 180 s for the demo and about 340 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` (full score) has held for 8 s, otherwise `score()` at the end.

## Scoring

Measured from the tile face after the robot has let go:

- **Folded (F):** the elbow and swivel hinge axes each within 4.5 cm of the tile, and the swivel axis within 3 cm of
  the wall plate's centre along the wall.
- **Parallel (P):** the glass front, 6 cm either side of its centre on the horizontal diameter, is 6.2 to 9.2 cm from
  the tile at both points, and the two are within 1.5 cm of each other (7 degrees). Either face may face the room.
- **1.0** for F and P; **0.5** for F only; **0** otherwise, including links jammed against the tile, links laid
  straight along it, or the swing made but the last fold not.
- **0 for the whole trial** if any part of the head ever comes within 3 mm of the tile (in the real trial a 3 mm
  feeler card on the tile and 20 fps video), or unless for the last 3 s the mirror is let go, every part of the robot
  is over 5 cm from the head and no hinge turns faster than 0.5 deg/s.
- Either mirror-image fold scores.

In simulation, F and P come from the mirror's hinge angles; the touch latch from the head's closest point to the tile
every step; release from every finger pad at least 3 cm from the mirror, and clearance from points every 2 cm along
both arms' links and on the finger pads. Doing nothing scores 0.

## Real-world reset

Lay a card template on the counter with the start angles marked. Unfold the mirror by hand, bend the elbow 17
degrees toward the wall (the same way every trial, so the bend direction never changes), and turn the head to the
template's 50 degree mark. Recheck the three hinge frictions with the spring scale every 10 trials.

## Files

- `fold_a_swing_arm_wall_mirror_flat.py`: the task file (scene, `score()`, `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (jointed parts and geometry, the closed-loop demo
  controller, geometry helpers).
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
