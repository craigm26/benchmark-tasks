# Change a trach tube's inner cannula: draw the used one out along its curve, feed a clean one in to the click

A tracheostomy tube has a removable inner cannula that is taken out and replaced so the airway stays clear. In
long-term and home care it is changed every 8 to 24 hours, mostly by licensed practical nurses (O*NET 29-2061.00,
0.6 million US workers: "Assemble and use equipment, such as catheters, tracheotomy tubes, or oxygen suppliers") and
respiratory therapists (29-1126.00, 0.13 million). The robot squeezes the used cannula's latch, draws it out, puts it
in the red waste tray, takes the clean one from its stand, feeds it in and lets go so the latch clicks under the
connector's lip. The hard part is the curve: the inner cannula is a rigid quarter circle of 45 mm radius inside a
matching round, curved outer tube with 0.5 mm of play per side, so it only moves while it turns about a centre 45 mm
from the hub. Measured on the collision geometry (`mechkit.gap`): along the true arc the 8.5 mm tube keeps 0.55 mm
from the wall of the 9.5 mm bore all the way round (0.64 mm from the connector's wider mouth); pushed straight in
without turning it jams 8.5 mm in, with 62 mm of its 71 mm still out, and a hand 1 degree off the arc's attitude
already puts the tube's end 1.2 mm off line. A pull with the latch caught, or a jammed pull or push, lifts the neck plate: that is the tube coming out of the
patient's neck (decannulation), and it scores 0. Steadying the neck plate with the other hand is standard practice
and is allowed; a two-handed policy is not marked down. The clean cannula must stay clean: if it touches
the table, the red tray or the manikin's skin it is contaminated, and that scores 0 too.

## Parts (real size)

Positions are in cm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

- **Airway manikin**, supine, head away from the robot, fixed, medium skin tone: head 16 cm ball centred at x 44,
  y 0, 8 cm up; neck 12 cm across, lying along x from 25.5 to 36.5, axis 4 cm up; chest from x 10 to 25.5, 30 cm wide,
  9 cm tall over the middle 22 cm and 5.5 cm at the sides.
- **Outer tracheostomy tube** (adult size 6, cuffless), 20 g, in the neck at x 30, y 0. Blue neck plate 3.0 x 8.4 x
  0.4 cm, top 10.9 cm up. White 15 mm connector: 16.5 mm outside, 11.5 mm mouth with a 2 mm entry chamfer, 8 mm
  proud of the plate, top face 11.7 cm up. Retaining lip round the connector: 27 mm across, 2 mm thick, 0.5 mm below
  the top face. The blue outer cannula starts inside the connector and runs on under the plate, inside the neck: a
  round tube, 9.5 mm bore, 13.5 mm outside (2 mm wall), swept toward the chest at 45 mm centreline radius about the
  point x 25.5, y 0, 11.7 cm up, from the connector's top face to 95 degrees round. Its bore tapers from the 11.5 mm
  mouth to 9.5 mm over the first 6 degrees (4.7 mm of arc), which is the lead in. The whole outer tube slides up to 12 mm upward against its ties
  (a 200 N/m spring; 1.2 N lifts it 5 mm).
- **Two inner cannulas**, 30 g each (25 g body, 5 g lever), loose, identical but for colour. Tube: rigid, 8.5 mm
  outside, a quarter circle of 45 mm centreline radius, 71 mm long. Hub: teal disc 21 mm across, 6 mm thick, whose
  lower face sits on the connector's top. Grip fin: teal, 18.1 x 14 x 22 mm, standing on the hub, with a 7 mm slot
  in its head-side face. Squeeze lever: teal, 6 mm wide, hinged 17.5 mm from the hub axis and 10 mm above the hub
  face, sprung shut at 0.05 N m/rad. Its paddle stands 8.7 mm proud of the fin's face; squeezed flush with the fin
  (0.45 rad) it swings the latch nose out from under the lip. Shut, the nose sits 1 mm under the lip with 3.75 mm
  of overlap.
- **Used cannula** (cream tube): seated and latched in the connector, lever toward the head.
- **Clean cannula** (pale grey tube): hanging upright in the stand, hub face at x 31, y -10, 5.6 cm up, lever
  toward the head, tube down in the slot, in the same attitude it will have when seated.
- **Stand**, light blue plastic, fixed: two walls 0.6 cm thick, 7 cm long and 5.6 cm tall, 1.3 cm apart, from
  x 25.2 to 32.3, centred on y -10, closed at the near end. The hub rim rests on the wall tops.
- **Waste tray**, red plastic, fixed: 14 (x) by 11 (y) by 3 cm tall, walls 3 mm, floor 4 mm, centred x 19, y -22.5.

Real-world build: a commercial tracheostomy care trainer with a size 6 cuffless tube and its inner cannula, a second
inner cannula hung in a slotted block beside the neck, a red kidney dish. A ruler beside the stoma reads plate lift
and hub gap; a photo shows the lever.

## The scripted demo

Right arm only; the left arm holds its ready pose. About 1,100 steps (55 s) of the 3,600 allowed. The gripper works
from the robot's right, pointing toward its left and 30 degrees down, pads closing along the patient's length.

1. **Unlatch and grip.** Pads open round the fin and paddle of the used cannula, then close: one pad lands on the
   fin's chest-side face, the other pushes the paddle flush into the fin's slot and lands on the fin too, so the
   grip is on the rigid fin and the nose is clear of the lip. The gripper opening (18 mm fin) and a lever angle of
   0.35 rad or more confirm it; one retry.
2. **Withdraw along the arc.** In 6 s the hub is turned 95 degrees about the arc centre, wrist turning with it, so
   the tube slides out along its own curve and its end clears the connector by 3.5 mm. Every step the hand's
   position and attitude are trimmed toward the arc from the cannula's read pose, because in a 0.5 mm fit a wrist
   1 degree off binds the tube. Whenever the neck plate
   reads more than 2 mm of lift the demo backs off 2 degrees, so the withdrawal can take up to 20 s.
3. **Discard.** Lift 3 cm, carry over the red tray, turn the cannula flat, lower it to 2.5 cm above the tray floor,
   open, wait 1 s.
4. **Pick the clean one.** Down over the clean cannula in the stand in the same grasp, squeeze, lift 8.5 cm.
5. **Insert along the arc.** Beside the neck, turn it to the 95 degree entry attitude; cross over the connector
   2 cm up; lower until the tube's end is 3.5 mm above the bore; turn the hub back about the arc centre, 95 degrees
   to 0 in 8 s; press 2 mm down the hub axis until the arm lags. Open: the lever springs shut and the nose drops
   under the lip. If the lever does not read shut, squeeze, press and release once more.
6. **Release.** Open wide, back out along the gripper's axis, park at the ready pose with the gripper open.

The demo reads the live poses of the two cannulas, the lever angle and the neck plate's lift from the simulator
(to trim the hand's position and attitude onto the arc at every step), with the fixed layout (object_jitter=0), its joint encoders and
gripper opening; it never moves or edits a part. In the offline contact replay (MuJoCo, not Isaac) only the two
fingertips touched the cannulas, the arm never touched the patient, the neck plate's peak lift was 1.3 mm (limit 5) and the run scored 1.0.

## Fidelity

Approximate. Sizes follow an adult size 6 tube: 8.5 mm inner cannula, 45 mm arc, 15 mm class connector. The cannula
is rigid, as reusable inner cannulas are; nurses are taught to follow the curve with disposable ones too, so the arc
skill carries over to the real trial. The outer cannula is a round curved tube whose 9.5 mm bore leaves 0.5 mm of
play per side round the 8.5 mm inner cannula (0.55 mm measured along the arc), against about 0.3 mm real, so the
arc is slightly more forgiving than the real one; a straight push jams 8.5 mm in. The bore is a 12-sided polygon
with its flats on the 9.5 mm circle, in 10 lengths along the bend, so the play is 0.55 mm at its tightest and
about 0.75 mm toward the corners. The connector's mouth is wider (11.5 mm) and tapers to the bore as a lead in. The outer
cannula's wall is 2 mm (13.5 mm outside) against about 10.8 mm outside for a real size 6; it is hidden in the neck.
Where the simulated skill is easier or different from the real one: one sprung lever on the head side, with a
paddle that folds flush into the fin, stands in for the two squeeze tabs a finger and thumb pinch, so a parallel
jaw can work it; each cannula weighs 30 g against about 5 g real, so it is steadier in the grip and its weight helps
it settle. The ties are a 200 N/m slide with a 12 mm stop. No cuff, secretions, suctioning or dressing. A real trial
is measured with the grippers open and away; the simulator cannot open them at the end, so the scorer requires the
release itself: a cannula still held, or with a finger within 3 cm of it or of the neck plate, scores 0.

## Files

- `change_a_trach_inner_cannula_along_its_curve.py`: scene, `score()`, `success()`, `demo_script()`.
- `mechkit.py`: multi-part objects, joints and procedural geometry. `closedloop.py`: the closed-loop demo
  controller. `belimits.py`: shared geometry helpers.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Change the trach's inner cannula: put the used one in the red tray, then fit the clean one until it clicks."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=change_a_trach_inner_cannula_along_its_curve.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/trach-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=change_a_trach_inner_cannula_along_its_curve.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/trach-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the cannula starts seated in the connector and the other
  hangs in its stand. There is no random seed.
- **Runtime per trial:** the demo takes about 1,100 steps (55 s simulated) and the episode ends 8 s after the score
  first reaches 1.0; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 340 s for the demo; the Dream-1 run ended after about 40 s with no score and no recording (see the Dream-1 run result below).

**Dream-1 run result:** no score and no video. Dream-1 refused the instruction: the model's server answered the request with an error before any action. The error was `PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Performing invasive medical procedures like changing a tracheostomy cannula can hurt someone.`, on the last of 3 attempts on 2026-11-14. The scripted demo scores 1.0 and shows the task can be done.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

Measured with the grippers open. In simulation, from the cannulas' poses, the lever angle and the plate's slide.

- **Seated:** the clean hub's lower face within 2 mm of the connector's top face, hub axis within 5 degrees of the
  connector's. **Latched:** seated, and the lever within 0.08 rad of shut (its nose under the lip).
- **In tray:** the used cannula wholly inside the red tray's walls and below its rim.
- **On manikin:** any part of the used cannula over the chest (x 10 to 25.5 cm, |y| 15 cm or less) or over the
  neck and head (x 25.5 to 52 cm, |y| 8 cm or less). The stand beside the neck is outside both.
- **Released:** every finger point at least 3 cm from both cannulas and from the neck plate, with both cannulas
  under 1 cm/s for 1 s.
- **Whole-run latches, score 0:** D, the neck plate lifted 5 mm or more at any time (the tube dragged out of the
  neck); C, the clean cannula came within 2 mm of the table, the red tray or the manikin's skin at any time
  (contaminated). The stand, the gripper and the inside of the outer tube do not count.

| Outcome, if released and no latch | Score |
| --- | --- |
| Clean latched, used in tray | 1.0 |
| Clean latched, used left on the table or stand | 0.75 |
| Clean latched, used left on the manikin | 0.5 |
| Clean in the connector but not latched (hub within 25 mm and 30 degrees of seated), used in tray | 0.4 |
| Used in tray, clean not in (still in the stand, held, or jammed by a straight push 62 mm out) | 0.25 |
| Anything else: nothing done, used cannula still in, a cannula off the table, not released, D or C | 0 |

A real trial reads plate lift and hub gap with a ruler beside the stoma and the lever from a photo.

## Real-world reset

Take the fitted cannula out along its curve and hang it in the stand with its lever toward the head. Seat the other
in the connector until its lever clicks under the lip. Check that the neck plate sits down on its ties. Nothing is
consumed: the two cannulas swap roles, or a fresh disposable is used as the clean one.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
