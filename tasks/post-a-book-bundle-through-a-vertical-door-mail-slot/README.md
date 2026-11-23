# Post a boxed book through a vertical door mail slot

A section of a front door stands across the table with a vertical brass mail slot in it. The robot must post a
boxed book through it: lift the box, feed it edge-on through a letter opening only 16 mm wider than the box while a
spring flap pushes back, set it on the slot floor (the hand cannot follow it through), regrip it by its trailing end and
slide it on until its centre of mass passes the inside edge and it tips off and drops into
the hall, where the flap swings shut. Posting a parcel or thick envelope through a door slot is something most
people have done or watched; the hard part is physical: a square, centred approach into a narrow opening, pushing
against a flap, and judging how far to push before the box drops instead of jamming half in the slot.

## Parts (real size, real material)

| Part | Real product | Size and material | In the scene |
|---|---|---|---|
| Mail slot | Salsbury Industries 4085 vertical mail slot (spring-loaded flap, polished brass) | plate 3-1/2 x 11 in (89 x 279 mm) overall, rough opening 3 x 8-1/4 in (76 x 210 mm), flap 1-3/4 x 7-3/4 in (44 x 197 mm) | brass plate 8 mm thick on the door front (front face x 36.2 cm) with a 41 x 191 mm letter opening; 2 mm brass flap hinged on a vertical pin at the opening's left edge, spring closed, swinging into the hall (0 to 1.6 rad) |
| Door section | custom (see Real-world build) | 300 x 370 x 45 mm (1-3/4 in exterior door thickness), oak | fixed, front face x 37.0, back face x 41.5, y -21 to +9 cm; 76 x 210 mm cut centred y -6 cm, floor 12.5 cm above the table, flush with the plate's opening |
| Boxed book | Uline S-16530 white Indestructo mailer, 9 x 6 x 1 in, holding a paperback | 229 x 152 x 25 mm corrugated board, 150 g packed | free box standing on its long edge on the table, length along x, centre x 21.5, y -6 cm; white for contrast |
| Hall floor | the table behind the door | | where the posted box lands |

Positions are cm from the midpoint between the arm bases, x forward, y to the robot's left. The 25 mm box in the
41 mm opening leaves 8 mm a side; 152 mm of box height in a 191 mm opening leaves 39 mm, enough to tip only about 26
degrees while the box still spans the door, so it must be pushed well past the inside edge to slide off.

## What the scripted demo does and what it reads

The right arm does the task; the left arm holds its ready pose. With its fingers leaned 34 degrees toward the door
(pitch 0.6 rad), the right hand grips the standing box across its 25 mm thickness, 33 mm behind its centre (the
gravity moment about the pad line is 0.15 kg x 9.81 x 0.033 = 0.049 N m), lifts it 1 cm above the slot floor and
carries it straight along x through the plate opening at about 5 cm/s; the flap swings in. With the tool point at
x 34.5 cm (fingertips 5 mm short of the plate) it sets the box on the slot floor, its centre now just past the door's
front face. It eases the pads to 45 mm, so they still box the 25 mm box in, slides the hand back along it to 15 mm
from its trailing end and closes again, then slides the box on along the slot floor, held so it cannot turn in the
opening, until the tool point is back at x 34.5 cm: the box's centre is then 30 mm past the inside edge (x 41.5).
It opens while rising 25 mm; the box tips off the slot floor and drops into the hall, and the flap closes. The hand
backs off and returns to its ready pose with the gripper open. If a look after the release shows the box still on
the slot floor, the hand grips its trailing end from above and slides it again. No part of either arm goes past
x 35.8 cm in front of the door (sweep.py and the gate envelope check it). The demo reads the mailer's live pose
from the simulator to find its trailing end before each regrasp and once after the release to confirm the drop;
everything else comes from the fixed layout (object_jitter=0), its joint encoders and gripper opening.

## Fidelity

The door is a 30 x 37 cm leaf section fixed to the table, with the slot floor flush from the plate to the back face.
The flap is a rigid 2 mm brass slab on a hinge spring of 0.02 N m/rad (0.031 N m fully open, about 1 N at the box);
Salsbury publishes no spring force, so the real trial measures it with a spring scale. The draught brush is omitted.
The box is one rigid block (a packed mailer is stiff), friction 0.35. The simulator scores the last step without
opening the grippers: a box still held at the door, or resting in the slot, has a corner short of the door's back
face and scores at most 0.60, and without a release (both hands 3 cm clear for the last 1 s) at most 0.35.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Post the boxed book through the door's mail slot so it drops into the hall behind the door, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=post_a_book_bundle_through_a_vertical_door_mail_slot.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/mailslot-dream

# Scripted demo (closedloop demo_script, right arm)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=post_a_book_bundle_through_a_vertical_door_mail_slot.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/mailslot-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the slot and the box have one fixed layout.
- **Runtime per trial:** the demo takes about 37 s of simulated time (about 740 steps); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 130 s for the demo; the Dream-1 run ended after about 20 s with no score and no recording (see the Dream-1 run result below).

**Dream-1 run result:** no score and no video. Dream-1 refused the instruction: the model's server answered the request with an error before any action. The error was `PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Dropping an object onto the floor behind a door could break it`, on all 4 attempts on 2026-11-23. The scripted demo scores 1.0 and shows the task can be done.
- **Score:** `task_score` reports 1.0 once `success()` (score 1.0) has held for 8 s, otherwise `score()` at the end.

## Scoring

1. **Score 0** if the box's centre ever crosses the door's mid plane (x 39.25 cm) outside the plate's letter opening
   (over the door or round its end), or if it never enters the opening: the first box corners to pass the plate
   front (x 36.2) must all be inside the plate's 4.1 x 19.1 cm letter opening (2 mm allowed). A box pushed along the
   table into the door's foot never enters.
2. Otherwise, at the end of the run:
   - **0.15** the box entered the opening;
   - **0.35** it also swung the flap open at least 0.9 rad (52 degrees) at some step while the box reached past the
     door's front face (the flap line, x 37.0); a finger poking the flap open earns nothing;
   - **0.60** it also ends wholly behind the door's front face (every corner past x 37.0, the flap line);
   - **1.0** it ends wholly behind the door's back face (every corner past x 41.3), its lowest point at least 1 cm
     under the slot floor (it dropped off the inside edge), still, and the flap is back within 0.15 rad of closed.
3. Both grippers must be at least 3 cm from the box over the last 1 s, otherwise the score is at most 0.35.

Doing nothing scores 0. A box left jammed or resting in the slot, or held there, scores at most 0.60; the flap held
open by a box or a hand blocks 1.0. In a real trial: open the grippers, then check from a photo and a tape measure
that the box lies wholly behind the door below the slot floor and the flap has closed (flap angle with a protractor
for 0.9 rad, which a video frame also shows).

## Real-world build

- **Bill of materials:** Salsbury 4085 vertical mail slot, brass (listed at about $50); Uline S-16530 9 x 6 x 1 in
  white Indestructo mailer ($1.30 each at the 100 tier) with any paperback, packed to 150 g; a 300 x 370 mm piece of
  45 mm door blank or laminated plywood (about $25); two printed PETG door feet and two bar clamps (about $25).
  Total about $100.
- **Print and cut files:** `python3 fabricate.py out/` writes `door_section.stl` (the leaf with its 76 x 210 mm cut,
  floor 125 mm above the table, centred 150 mm from the left edge seen from the robot) and `door_foot.stl`.
- **Assembly:** cut the opening, screw the Salsbury plate to the front face centred on it with its letter opening's
  floor level with the cut floor (pack under the plate if needed), stand the section on the feet with its front face
  37 cm in front of the arm bases, and clamp the feet to the table.
- **Force:** the flap needs about 1 N to open; the box weighs 1.5 N. Nothing needs more than a few newtons per finger.

## Real-world reset

Pick the box up off the floor behind the door, check it has not been crushed, and stand it on its long edge on the
marked spot. Check the flap has closed. Nothing is consumed and nothing gets dirty.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
