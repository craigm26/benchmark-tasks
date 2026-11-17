# Fold an extended wall mirror flat

An extendable shaving mirror hangs on the tiled wall over a basin on two arm links, swung out over the counter. The
robot must fold it away: both links back flat against the wall, the round head parallel to the tile, and then let go.
People do this after every shave so the arm stops sticking out over the basin. The mirror is a passive two-link chain
the robot can steer only through its head, and the elbow can be left bent either way after use; here it is bent toward the wall. Pushed straight back,
the elbow bends further toward the wall and the head jams against the tile, unfolded. The fold that works carries the
head round in front of the wall plate the way the elbow already bends, and the glass must be squared to the wall
first, because near the folded pose the tile stops a turned head 39 degrees from parallel.
The same skill folds monitor arms, wall TV arms and wall lamp arms.

**The trap, measured.** In the MuJoCo contact replay used to check this task offline, a probe
gripped the head as the demo does and pushed it straight toward the tile at 10 mm/s without squaring it. The
shoulder hardly moved (+60 to +65 degrees) while the elbow bent from +17 to +47 toward the wall and the head turned on
its swivel from -27 to -63 degrees. After 70 mm of push the bezel's rim reached the tile (the touch rule fired, score
0). Pushing on to 120 mm the chain stayed jammed there unfolded (elbow +53), and the gripper itself ran into the tile.

## Parts (real size, real material)

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left); heights are above the
counter (the station's wood table top). The mirror is a full-size 6 inch two-arm wall shaving mirror, the class of
the **Jerdon JP7508C** (Jerdon wall-mount vanity mirror, chrome, 6 inch round head on a two-link arm with a 360 degree
swivel, extending about 10 inches from the wall). Every part that carries the mechanism is printed from `fabricate.py`,
using the dimensions the scene uses, and the rest is standard hardware (see "Build the real mirror"). This printed
set is the reference build: the simulator models it exactly.

**What comes from the JP7508C and what is set by the reference build.**

- From the JP7508C, at full size: the layout (round wall plate, two arm links on vertical-axis pivots at the wall and
  the elbow, a round head on a swivel at the arm's tip), the 6 inch (150 mm) head, and the reach: at full stretch the
  head centre is about 24 cm from the shoulder hinge on the wall plate (11 + 10 cm links, 3 cm swivel to head centre;
  25.5 cm from the plate's face) against the JP7508C's 10 inch (25.4 cm) extension.
- Simplified: commercial mirrors of this kind also tilt the head on a horizontal pivot. That pivot is left out of both
  the print and the scene (all three hinges are vertical, as if the tilt were locked level), because the fold happens
  in the horizontal plane and a tilting head would only add a free wobble. The printed head has one mirror face; the
  trial never uses the JP7508C's magnifying reverse face.
- Set by the reference build: the two link lengths (11 and 10 cm), the three stop angles and the three friction
  values. The stops are printed or pinned into the parts, and the frictions are set and measured on the printed
  mirror with a spring scale, so every trial and the simulator behave the same. Commercial mirrors have no printed
  stops, so a store-bought JP7508C may let the elbow or swivel turn further and its pivots may be stiffer or looser;
  the values here are the reference build's. Where each one comes from is in "Hinge stops" and "Hinge friction"
  below.

| Part | Real dimensions and material | Where |
|---|---|---|
| Tile wall | 44 x 24 cm panel, 2 cm thick: 12 mm plywood faced with white glazed 10 x 10 cm ceramic wall tiles, grey grout; clamped to the counter | Face at x 32.5, centred on y 0, standing on the counter |
| Witness sheet | 3 mm white EVA foam, 44 x 24 cm, with an 82 mm hole for the wall plate | Taped flat on the tile face |
| Wall plate and bracket | Black PETG: round plate 8 cm across, 1.5 cm thick, bonded to the tile; two ears round link 1's eye (22 mm knuckle, 26 mm tall); two stop cheeks | Plate centre y 0, 6.9 cm up; shoulder hinge axis at x 29.5, y 0 (3 cm in front of the tile) |
| Link 1 | Black PETG, 18 mm round, 11 cm axis to axis, lower half of the 20 mm elbow knuckle, 50 g | Vertical shoulder hinge, link 6.9 cm up |
| Link 2 | Black PETG, 18 mm round, 10 cm axis to axis, upper half of the elbow knuckle, 22 mm swivel boss 18 mm tall, 45 g | Vertical elbow hinge 2.6 cm higher than the shoulder's |
| Head | Black PETG bezel 15 cm across and 2.5 cm thick, 136 mm silver acrylic mirror 3 mm thick (0.6 mm proud) on the face away from the swivel, chrome-look trim ring, 14 mm neck; 0.45 kg | Vertical swivel hinge; head centre 3 cm in front of the swivel axis, 9.5 cm up; rim 2 cm above the counter |

Hinge friction: shoulder 0.25 N m, elbow 0.15 N m, swivel 0.10 N m. These are the reference build's values, set and
measured on the printed mirror with the spring scale; they are also the simulator's Coulomb holds. Each is set so the
hinge holds its part wherever it is left, as a vanity mirror's pivots must, with the shoulder stiffest and the swivel
lightest so the head turns before the links move, and all three low enough for one gripper to drive through the head
(the elbow moves at 1.5 N, the head at 1.3 N). Each hinge is an M5 screw through a 5.3 mm hole
with a conical spring washer and a nylon-insert lock nut. Tighten the nut until a 5 N spring scale pulled at right
angles moves the part at the stated force: link 1 at 2.3 N at the elbow knuckle (11 cm out), link 2 at 1.5 N at the
swivel (10 cm out), the head at 1.3 N on the bezel's side, 7.5 cm from the swivel axis.

Hinge stops (all printed or pinned, and in the simulator as joint limits at the same angles). The angles are the
reference build's, fixed by the printed parts:

- **Shoulder +-90 degrees:** where link 1 lies along the wall, as a two-arm wall mirror's first link does when
  folded. Link 1's flank lands on one of the bracket's two stop cheeks (faces 9 mm in front of the shoulder axis, 12
  to 20 mm to each side).
- **Elbow +-178 degrees:** where link 2 lies back over link 1, as the two links of a double-swivel arm stack when
  folded, stopped 2 degrees short so the links never rub. Two 3 mm steel dowel pins, each 86 mm from the elbow axis,
  one standing up from link 1's top and one hanging from link 2's underside, meet there.
- **Swivel +-95 degrees:** the reference build's stop (commercial swivels turn further). The head's flat neck bar
  (8 mm wide) meets the back web of link 2's swivel boss; the bezel is then still 2 mm clear of link 2's tube, which
  it would meet at 97 degrees.

Angles below are relative, link to link: 0 is a link pointing straight out at the robot, + is counter-clockwise from
above. Start: shoulder +60 (elbow at x 24.0, y -9.5), elbow +17, bent toward the wall (swivel axis at x 21.8,
y -19.3), swivel -27. The glass is turned 50 degrees from facing the robot, toward the robot's right (head centre at
x 19.8, y -21.6). The folded pose the demo makes is shoulder -90, elbow +178, swivel -88: elbow axis 3.0 cm and
swivel axis 3.35 cm from the tile, head centre at x 26.2, y +1.0, glass front 7.66 cm from the tile, bezel back
10.5 mm clear of link 1. The mirror image (+90, -178, +88) scores the same. In the task file every shape is laid at
the start pose, so each joint value is the angle minus its start angle.

Why the order matters, measured offline from the colliders at the folded pose: the head can turn only 38.8 degrees
one way from parallel before it meets the tile, and 9.8 degrees the other way before it meets link 1. A head still
turned 50 degrees cannot reach the folded pose, so it has to be squared first. Along the demo's path the head stays
at least 26 mm from the tile and 22 mm from link 1, and the links and head stay at least 4 mm from the wall plate.

## What the scripted demo does

The right arm does the whole fold; the left arm has no plan and holds its ready pose.

1. It moves over the head, turns its pads along the glass normal, opens to 45 mm, trims its height on its encoders
   and comes down to a top-down pinch across the 25 mm bezel, its tool point 2 cm below the top of the rim. It checks
   the grip on the gripper encoder (pads stopped at least 20 mm apart) and retries once if they closed on air.
2. Squaring: it turns the head -50 degrees about the swivel axis (tool point on a 3 cm arc round it), links held.
3. Swing: it carries the head across in front of the wall plate along a planned path, the shoulder turning from +60
   to -90 while the elbow folds from +17 to +140 the way it is already bent. The glass stays parallel to the wall,
   except mid-swing, where it turns up to 14 degrees so the swivel stays within 88 degrees, clear of link 2.
4. It checks the shoulder sits on its -90 stop, then makes the last fold as an arc about the measured elbow axis
   (elbow +140 to +178) at about 20 mm/s.
5. It opens to 45 mm, waits, slides the open pads 10.5 cm off the head along the glass plane, rises, draws back
   toward the robot and swings to its ready pose, and holds there. The last frame shows the black arm folded flat on
   the white tile, glass parallel, grippers open.

The demo reads the mirror's hinge angles from the simulator, and computes the head's pose from them, to correct its
path. It also reads its own joint encoders and gripper gap. Each control step it compares the planned head centre with
the measured one, adds the error into an offset on the hand command (at most 15 mm) and holds the path while the head
lags more than 8 mm. Tool speed stays under 0.10 m/s (squaring and swing slower), inside the declared 0.15 m/s limit.

In the MuJoCo contact replay used offline the demo scored 1.0 (success at 36.4 s). The mirror ended at shoulder -90,
elbow +178 and swivel -88.6 degrees, the swivel never passed -88.2 on the way, and the arm touched nothing but the
head. The time and score in the Isaac video are given under "Scores" below once the Isaac run is done; they replace
the replay's figures.

## Fidelity

Rigid bodies. The mirror is mounted low (rim 2 cm above the counter) so the arm reaches the whole fold. All three
hinges are vertical, so height does not change the fold. The hinge stops are joint limits at the angles the printed
stops give (+-90, +-178, +-95). The cheeks are modelled; the pins and neck bar are drawn for looks only. Hinge
friction is a Coulomb hold (0.25, 0.15, 0.10 N m) on vertical hinges, which gravity does not load. The mirror disc is
rigid. The 3 mm foam witness sheet is not modelled: the simulator latches the same rule as a distance, any head point
within 3 mm of the tile. The simulator scores the last step without opening the grippers, and a mirror still held
scores 0. The mirror is printed PETG rather than plated metal; the scene's masses are the printed parts'.

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
- Wall-clock time on one L40S, including Isaac Sim start-up: about 420 s for the demo; the Dream-1 run ended after about 20 s with no score and no recording (see the Dream-1 run result below).

**Dream-1 run result:** no score and no video. Dream-1 refused the instruction: the model's server answered the request with an error before any action. The error was `PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Folding against a wall could slam or force joints past limits.`, on all 3 attempts on 2026-11-17. The scripted demo scores 1.0 and shows the task can be done.
- **Score:** `task_score` reports 1.0 once `success()` (full score) has held for 8 s, otherwise `score()` at the end.

## Scores

- Dream-1 run: no score (the model refused the instruction before acting, see no_model_video_reason).
- Scripted demo: 1 (about 420 s on one L40S including Isaac Sim start-up).

## Scoring

Measured from the tile face after the robot has let go, on a top photo (see "Reading the distances" below):

- **Folded (F):** the elbow and swivel hinge axes each within 4.5 cm of the tile, and the swivel axis within 3 cm of
  the wall plate's centre along the wall.
- **Parallel (P):** the glass front, 6 cm either side of its centre on the horizontal diameter, is 6.2 to 9.2 cm from
  the tile at both points, and the two are within 1.5 cm of each other (7 degrees).
- **1.0** for F and P; **0.5** for F only; **0** otherwise, including links jammed against the tile, links laid
  straight along it, or the swing made but the last fold not.
- **0 for the whole trial** if any part of the head ever comes within 3 mm of the tile, or unless for the last 3 s the
  mirror is let go, every part of the robot is over 5 cm from the head and no hinge turns faster than 0.5 deg/s.
- Either mirror-image fold scores.

**Reading the distances (real trial).** All the F and P distances are read from one photo taken straight down from
about 60 cm above the folded mirror, after the 3 s release, with a steel tape measure laid on the counter along the
tile face, its zero at the wall plate's centre line (y 0), and a 30 cm rule laid square to the tile, its end against
the tile face (for distances out from the tile). Before the trials, mark the top of the bezel's rim with two thin
white ticks on the glass front edge, 6 cm either side of the head's centre on the horizontal diameter.

- Elbow and swivel axes: the M5 screw heads on top of the elbow and swivel hinges mark the axes. Read each one's
  distance out from the tile on the rule (F needs 4.5 cm or less), and the swivel screw's position along the tape
  (F needs 3 cm or less either way from zero).
- Glass front: read each white tick's distance out from the tile on the rule. P needs both between 6.2 and 9.2 cm.
- Glass skew: subtract the two tick readings. P needs the difference to be 1.5 cm or less; over the ticks' 12 cm that
  is a glass turned at most 7 degrees from parallel.

The photo is kept with the trial. If a reading is within 2 mm of a limit, check it with the tape on the mirror itself
before moving anything.

**Contact witness (real trial).** The tile face carries a 3 mm white EVA foam sheet, and before each trial the bezel's
rim and back edge are rubbed with blue chalk. A head that comes within 3 mm of the tile presses the foam and leaves a
blue mark on it. Read the sheet only after the 3 s release, once the robot is clear: any blue mark scores 0. Replace the sheet when
it stays pressed thinner than 3 mm (check with calipers every 10 trials), so the 3 mm margin is the same every trial. Only the head is chalked, so a robot
finger brushing the foam leaves no mark. The 20 fps video is a backup for when a mark is unclear. The links and the
wall plate never reach the foam (link 1 lies 21 mm from the tile when folded, and the plate sits in the sheet's hole).

In simulation, F and P come from the mirror's hinge angles. The touch latch comes from the head's closest point to
the tile every step. Release comes from every finger pad being at least 3 cm from the mirror, and clearance from
points every 2 cm along both arms' links and on the finger pads. Doing nothing scores 0.

## Build the real mirror

Run `python3 fabricate.py` to write `stl/wall_bracket.stl`, `stl/link1.stl`, `stl/link2.stl` and `stl/head.stl`
(ASCII STL in mm; each part is a few overlapping closed shells, which PrusaSlicer, Bambu Studio and OrcaSlicer join on
slicing, as does Cura with "Union Overlapping Volumes"). The script prints each part's size and solid volume.

### Parts list

The printed set below is the reference build, the one the simulator models exactly. The store-bought route is a
**Jerdon JP7508C** (6 inch head, about 10 inch extension): mount its wall plate on the same panel at the same place,
set the same start angles from the card template, lock its head tilt level, and check its stops and hinge friction
with the spring scale against the values under "Parts"; note any difference with the trial. A difference is logged with the trial only; it never changes the scoring thresholds.

| Item | Qty | Source |
|---|---|---|
| Printed parts: wall bracket, link 1, link 2, head | 1 each | `fabricate.py`; black PETG (about 0.5 kg in all) |
| Silver acrylic mirror disc, 136 mm across, 3 mm thick | 1 | Simply Plastics silver mirror acrylic disc cut to 136 mm, or laser-cut from 3 mm silver mirror acrylic sheet (McMaster-Carr mirrored acrylic sheet). Acrylic, not glass, so a dropped head cannot shatter |
| Socket head cap screws ISO 4762: M5 x 35 (shoulder), M5 x 60 (elbow), M5 x 25 (swivel) | 1 each | Any fastener supplier |
| Nylon-insert lock nuts ISO 10511 M5 | 3 | Any fastener supplier |
| Conical spring washers DIN 6796 M5 | 3 | Any fastener supplier |
| Flat washers ISO 7089 M5 | 6 | Any fastener supplier |
| Steel dowel pins ISO 8734, 3 x 10 mm | 2 | Any fastener supplier |
| 3M VHB 5952 tape (wall plate to tile) and 3M VHB 4910 clear tape (mirror into bezel) | 1 roll each | 3M distributors |
| 12 mm plywood 440 x 240 mm, white glazed ceramic wall tiles 100 x 100 x 6 mm, tile adhesive, grey grout | 1 panel, 12 tiles | Hardware store |
| Steel L-brackets and two C-clamps (panel to counter) | 2 + 2 | Hardware store |
| 3 mm white EVA foam sheet, 440 x 240 mm; removable double-sided tape | 1 per few trials | Craft store |
| Blue blackboard chalk | 1 | Stationery store |
| 5 N spring scale, Pesola Light-Line 18005 | 1 | Pesola distributors |

### Print settings

- Wall bracket: plate back on the bed, supports under the ears, 100% infill.
- Links: on their side (tube axis flat), tree supports, 100% infill. Link 1 should weigh 50 g and link 2 45 g with
  their pins and screws.
- Head: standing on its rim, tree supports, 4 walls; set the infill (about 70%) so the printed head weighs 395 g.
  With the mirror disc and screw it weighs 450 g.
- Drill every hinge hole out to 5.3 mm after printing.

### Assembly

1. Drill a 2.9 mm hole 4 mm deep in the top of link 1, on its centreline 86 mm from the elbow axis (24 mm from the
   shoulder axis), and one in the underside of link 2, 86 mm from the elbow axis. Press a dowel pin into each so 6 mm
   stands out.
2. Bond the mirror disc into the head's front recess with VHB 4910, so it stands 0.6 mm proud. Rub the bezel's rim
   chrome-look with a silver paint pen if wanted.
3. Swivel: put the head's eye into the slot of link 2's boss. Fit the M5 x 25 screw from below with a flat washer, and
   on top put a flat washer, the spring washer and the lock nut.
4. Elbow: stand link 2's eye on link 1's knuckle and fit the M5 x 60 screw up through both, washers, spring washer and
   nut on top. Check the pins meet with link 2 lying back over link 1.
5. Shoulder: put link 1's eye between the bracket's ears and fit the M5 x 35 screw the same way. Check link 1 lands on
   a cheek when it lies along the wall, on either side.
6. Wall: glue the tiles to the plywood and grout them. Stand the panel on the counter, tile face 32.5 cm in front of
   the midpoint between the arm bases, and fix it with the L-brackets and C-clamps. Bond the wall plate to the tile
   with VHB 5952, its centre on the panel's centreline and 6.9 cm above the counter, the ears level. Leave it 24 h.
7. Tighten each lock nut until the spring scale gives the forces under "Parts", starting with the swivel.
8. Tape the foam sheet flat on the tile, the plate through its hole.

### Reset and cleaning between trials

- Wipe the foam sheet with a damp cloth, or replace it when a chalk mark will not come off. Re-chalk the bezel's rim
  and back edge. Wipe the mirror with a dry microfibre cloth (acrylic scratches; no paper towels).
- Lay a card template on the counter with the start angles marked. Unfold the mirror by hand, bend the elbow 17
  degrees toward the wall (the same way every trial, so the bend direction never changes), and turn the head to the
  template's 50 degree mark, glass toward the robot's right.
- Recheck the three hinge frictions with the spring scale every 10 trials, and retighten the nuts if they have eased.

## Files

- `fold_a_swing_arm_wall_mirror_flat.py`: the task file (scene, `score()`, `success()`, `demo_script()`).
- `fabricate.py`: writes print-ready STL files of the printed reference build, from the scene's dimensions.
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (jointed parts and geometry, the closed-loop demo
  controller, geometry helpers).
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py and fabricate.py in this folder; no third-party assets.
