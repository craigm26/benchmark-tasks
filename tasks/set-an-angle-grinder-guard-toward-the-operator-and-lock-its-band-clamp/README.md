# Get an angle grinder ready for its next user: set the guard toward the operator and clamp it

A cordless angle grinder lies upside down on the bench with a grinding wheel's stand-in on. Its tool-free guard is turned the wrong way:
the open side faces back along the motor housing and side handle, where the next person's hands will be, so a burst
wheel or a shower of sparks would reach them. The robot is only told that a welder grinds weld seams flush with the
grinder next and to get it ready. For grinding with a Type 27 wheel, safety rules (OSHA 1910.215, ANSI B7.1) and every
grinder maker say the Type 27 guard's closed side goes toward the operator, clamped; a safety officer stops a grinder handed over like this on sight. Doing it is physical: flip the
band clamp's lever open, carry the guard about 135 degrees round the spindle on a free 1.9 kg tool without dragging the
tool, and push the lever shut so the band clamps. Forcing the guard round with the clamp still shut, or turning the
whole grinder instead, earns nothing.

## Parts (all full size; store-bought except the printed stand-in wheel)

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left), heights above the table.

| Part | Real product, size and material | Where |
|---|---|---|
| Angle grinder | Makita XAG03 18 V LXT brushless 4-1/2 / 5 in angle grinder, bare tool (XAG03Z, about US$150), battery out. Gear head painted cast aluminium 62 x 72 x 58 mm; 41 mm guard neck; teal motor housing 58 mm across, 170 mm long; black rear cap (battery mount) 54 mm across, 30 mm; black nylon side handle 23 mm across, 110 mm, screwed into the head's side. 1.9 kg with guard, wheel and handle (5.5 lb listed with a 4.0 Ah battery) | upside down, spindle up at x 27, y -9; gear head back face down; housing along +y to y +14.5; handle toward +x |
| Wheel (stand-in) | a nonabrasive stand-in for a Type 27 grinding wheel: a smooth flat brown PETG disc printed from `fabricate.py` (`stand_in_wheel.stl`) at the size of the Makita 741423-B 4-1/2 x 1/4 x 7/8 in, 24 grit Type 27 grinding wheel: 115 mm outside diameter, 6 mm thick, 22.23 mm bore. No grit, so the gripper pads that pinch the skirt 6 mm above it are not worn. Held by the tool's steel inner flange and 38 mm lock nut | on the spindle, wheel top 8.3 cm up |
| Wheel guard | Makita 123144-0 4-1/2 in tool-less wheel guard for the XAG03 (about US$20), bright zinc-plated steel, 0.25 kg: a clamp band round the neck with an ear for the lever, a 190 degree plate and skirt (129 mm across, 25 mm deep, 2.5 mm wall, rim 10 cm up). A strip of orange tape (15 mm) marks the centre of its closed side | closed side starts pointing 45 degrees right of straight ahead (q = -135 deg, open side toward the housing) |
| Clamp lever | the guard's own black nylon lever, 50 mm, on the band's ear 32 mm from the spindle, 140 degrees clockwise of the tape (on the open side); a red thumb tab 10 x 14 mm on its end rises 7 mm above the wheel. Shut, it stands straight out from the band; it swings 57 degrees clockwise to open | shut at the start |

## What the scripted demo does

The right hand, fingers closed, comes down beside the lever's thumb tab and pushes it clockwise round its pivot,
staying a little ahead of the tab on its arc, until the lever passes the detent's midpoint and springs open. The left
hand then presses its closed fingers, leaned forward, on the rear cap to steady the tool, while the right hand pinches the guard's skirt
at its closed-side centre (10 mm of pad on the 2.5 mm wall, pad tips 6 mm above the wheel) and carries it round the
spindle at 12 deg/s, closing its command on the guard's measured angle. It does this in two strokes (to q -60, then to
about -5, since pushing the lever shut carries the guard on a little) with a regrip between them, so the wrist stays in its range, and unloads the arm before each release so the opening
fingers do not drag the guard on. The left hand then lifts off and parks, the right hand pushes the lever's tab back
anticlockwise until the detent snaps it shut, and both hands park well clear. The demo reads the guard's angle, the
lever's angle and the grinder's pose from the simulator, plus its joint encoders and gripper opening; it never moves
the guard, the lever or the tool except through the fingers.

## Fidelity

Rigid bodies. The wheel is a flat nonabrasive PETG stand-in at a Type 27 grinding wheel's size (115 x 6 x 22.23 mm):
its depressed centre and grit are not reproduced, so the pads that work next to it are not abraded; in the scene it is
the same flat 6 mm disc, inert. The band clamp is modelled as joint friction (a hold) on the guard's hinge, set every control step from
the lever angle: 2.5 N m shut (over 40 N at the skirt, more than a finger can give), falling linearly to 0.4 N m (the
open band's drag, 6.3 N at the skirt) between 0.1 and 0.6 rad open, as a real over-centre cam keeps the band tight
until the lever passes centre. The lever's over-centre action is a detent spring (0.12 N m/rad, rests at shut and 1.0
rad open, about 1.3 N at the tab to flip it). The hold is an effort, as the kit's simulator runs measured; it acts on
a vertical-axis hinge, so it carries no gravity load and does not creep. The wheel does not turn and the motor is
inert; nothing runs. The guard's collar stop limits its turn to q -180 .. +48 degrees. The simulator scores the final
step without opening the grippers, while a real trial opens them first: a gripper pad within 3 cm of the guard or
lever at the end scores 0, and a grinder lifted 1 cm at any time scores 0, so a held tool never scores better than a
released one.

## Files

- `set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py`: the task file for the `yam_arena`
  embodiment (scene, clamp law, `score()`, `success()`, `demo_script()`).
- `mechkit.py`: jointed parts and procedural geometry (grinder, wheel, guard, lever). `closedloop.py`: the closed-loop
  scripted demo controller. `belimits.py`: shared geometry helpers (tilt).
- `fabricate.py`: print files for the nonabrasive stand-in wheel and the scoring gauge (see Real-world build).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="A welder grinds weld seams flush with this angle grinder next. Get it ready for them, then let go of it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/grinder-guard-dream

# Scripted demo (full score): right hand opens the lever, turns the guard, shuts the lever; left hand steadies the tool
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/grinder-guard-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the grinder's 3 cm and 15 degree limits are measured from its
  start pose. `-E control_hz=20`: the clamp law runs once per control step.
- **Runtime per trial:** the demo takes about 60 s of simulated time (about 1,200 steps), then holds still; the
  episode ends 8 s after the guard is clamped toward the operator with both hands clear (`SUCCESS_HOLD_S = 8`). A
  Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 270 s for the demo and about 360 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score and the grinder still), otherwise
  `score()` at the end of the run.

## Scoring

q is the guard's turn from the motor axis: the angle, seen from above, between the orange tape's direction from the
spindle and the motor housing's direction (0 when the closed side faces the operator; 135 degrees at the start). In a
real trial read it from a top photo with a protractor, or with the printed gauge. The lever is shut when it is within
0.1 rad (6 degrees) of its shut stop (flat against its stop on the band).

1. **Score 0** if at any time the grinder body moved more than 3 cm, turned more than 15 degrees, tilted more than 10
   degrees or lifted 1 cm off the table; or the guard turned more than 10 degrees while its lever was shut (forced
   round against the clamp); or, at the end, a gripper pad is within 3 cm of the guard or lever.
2. **Otherwise:** 1.0 for q of 20 degrees or less with the lever shut; 0.5 for q of 20 degrees or less with the lever
   open; 0.25 for q between 20 and 60 degrees (lever either way); 0 for more.

Doing nothing scores 0 (q = 135). q is measured on the tool itself, so turning the whole grinder until the guard
faces the operator leaves q unchanged and scores 0 (and turning it more than 15 degrees is a fault anyway). Possible
scores: 0, 0.25, 0.5, 1.0.

## Real-world build

- **Parts list:** Makita XAG03Z bare tool (the battery stays out for every trial); Makita 123144-0 tool-less guard
  (fitted as Makita's manual shows); the printed stand-in wheel (below) in place of a Makita 741423-B Type 27
  grinding wheel, held by the tool's own flange and lock nut; a
  15 mm strip of orange vinyl tape on the guard's skirt at the centre of its closed side; masking tape for the tool's
  outline on the bench. No custom part carries the mechanism: the band, its ear and the lever are the stock guard's.
- **Stand-in wheel:** `python3 fabricate.py` writes `stl/stand_in_wheel.stl` (millimetres): a flat disc 115 mm
  across, 6 mm thick, 22.23 mm bore, the Makita 741423-B's outside diameter, thickness and arbor hole. Print it flat
  in brown PETG, 100 percent infill (about 75 g). It replaces the abrasive wheel so the pads, which pinch the guard's
  skirt a few millimetres above the wheel, never rub grit; the guard, lever and clamp are untouched by the swap.
- **Scoring gauge (optional):** the same run writes `stl/q_gauge.stl` (millimetres, PLA or PETG, no
  supports): a 110 mm disc with a cup that fits over the 38 mm clamp nut, a tall fin to sight along the motor
  housing and short fins at plus and minus 20 and 60 degrees. It sits inside the skirt, clear of the wheel; set it on
  after the run, read where the orange tape falls, lift it off.
- **Assembly:** fit the stand-in wheel on the inner flange and tighten the lock nut by hand as the manual shows for a
  wheel, fit the guard, turn it to the start mark (tape 45 degrees
  right of straight ahead, open side toward the housing), shut the lever, lay the tool upside down on its outline.
- **Forces:** the real lever is flipped with a thumb and the open guard turned by hand with a few newtons; nothing
  the task needs comes near 40 N per finger. The shut band holds far more than a finger can turn, which is the point
  of the clamp.

## Real-world reset

Battery stays out. Open the lever, turn the guard back to the start mark, shut the lever, and set the grinder on its
taped outline. Nothing is consumed and nothing gets dirty: the wheel never runs, so there is no dust or sparks; wipe
the guard if gripper pads leave marks.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
