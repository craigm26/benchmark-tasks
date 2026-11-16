# Put a resident's reading glasses on a care manikin

Nursing assistants and care aides put residents' glasses on every morning as part of dressing and grooming. The
taught method: hold the glasses by both temples near the hinges, bring them up level in front of the face, tip the
temples up and slide them back over the ears, set the two pads on the sides of the nose, and lower the temples so each
bent earpiece hooks down behind its ear root, never moving toward an eye. The robot must do exactly that on a
full-size manikin head, then let go. The hard part is physical: the earpieces must clear the ear roots and then drop
into the groove behind them, and both pads must land on the sloped nose flanks, all with a 25 g hinged frame that
must stay there once released. A frame that touches an eyelid at any time scores 0.

## Parts (real sizes)

Positions in cm from the midpoint between the arm bases, x forward, y left; heights above the table.

- **Manikin head**, fixed: a rigid vinyl nursing-skills head on a grey stand (foot 15 cm across and 1.5 cm tall, neck
  10 cm across), facing the robot, its vertical axis at x 34.5, y -7. Skull 19 cm front to back and 14 cm across at the
  ears, chin 12.5 cm up, crown 31 cm up. Not built from blocks: the skull is ten convex slices of an ellipsoid, the face
  is set back under a brow ridge (glabella at x 25.2, 21.2 cm up), and the nose is a ridge 5 mm wide running from x 25.8
  at 20.3 cm up to the tip at x 23.4, 15.8 cm up, with flanks 40 deg off the midline (where the pads sit). Closed eyelids
  (fronts at x 26.3, 3.2 cm either side of the midline, 19.3 cm up). Each ear has a root (x 32.4 to 34.6, 6.2 to 8.8 cm out
  from the axis, top 19.7 cm up, its back face a wall with a 1 mm round top edge, where the earpiece hooks) and a pinna (3.2 x 6.2 cm, 6 mm thick, inner face 8.7 cm out at
  the front, flaring 30 deg outward going back, top 21.2 cm up) with a 3.5 mm round rim over the top and back. Look-only
  brows, lashes, lips and dark hair.
- **Overbed tray**, fixed: light grey plastic, 17 x 17.5 x 3.8 cm, in front of the resident under the chin, x 12 to 29,
  y -15.75 to 1.75 (centred on the head axis). Part of the setup.
- **Reading glasses**, black acetate, 25 g: front 14 g, 14 cm across; grey-blue lenses 5 x 3.2 cm, 2 mm thick, in 5 mm
  rims; bridge 18 x 4 x 6 mm; two pads 10 x 6 x 2 mm on wire pad arms, 1.8 cm apart, each angled to its nose flank;
  hinges 13.6 cm apart. Temples (5.5 g each) on hinges with a 96 deg open stop and 0.004 N m friction: 9.8 cm x 4 x 8 mm
  from hinge to bend, then a 3.5 cm earpiece bent 75 deg down (13.3 cm in all, a common reading glasses length). Worn,
  each earpiece hangs down 1.5 mm behind the back wall of its ear root, so once the frame slides a little down the
  nose the earpieces hook on the roots and stop it (the vinyl skin has friction 1.0). They start open on the tray, standing on lens bottoms and earpiece tips, temples pointing away from the
  robot, hinges at x 14.7, centred on the head axis at y -7.

## What the demo does and reads

Both arms, one temple each (left arm the left temple, right arm the right), the way aides are taught; one hand on the
front would score the same if it left the same result. Each arm grips its temple behind the hinge (left 3.5 cm, right
2 cm), pads on the temple's side faces, approach leaned forward about the pad line (left arm 45 deg, right arm 15 deg:
unequal leans keep the two forearms apart over temples 14 cm apart). The glasses lie centred on the head axis, so each
arm takes the temple on its own side and the arms never cross. Then: spread 1 mm to load both open stops, lift 3.2 cm
(levelling the frame, which rests slightly tipped on the tray), raise straight up at 3 cm/s in front of the face, tip
the temples up 18 deg about the line between the nose pads, slide 10.6 cm straight back at 2 cm/s with the earpieces
passing over the ear roots, lower the temples level so each earpiece drops behind its root (to a seat 3 mm above the
worn pose, earpieces 3 mm clear of the roots on the way down), hold 1 s, open only to an 8 mm gap (each pad moves 2 mm
off the temple), back each gripper 1.2 cm off along its own temple, rise 4 cm, move 4 cm outward and park both arms at
READY; the frame drops the last 3 mm onto the nose and ear roots. About 53 s (1,061 steps). While it carries the frame,
the demo corrects both hands together from the frame's measured pose: position, pitch, roll and yaw (so the earpieces
pass level over both ears and come down behind them), and each hand's spread from its hinge angle (so both temples stay
on their open stops). The demo reads the live pose and hinge angles of the glasses from the simulator (to align each
grasp and to servo the frame onto its worn mark), plus the fixed head layout, its joint encoders, tracking error and
the gripper gap.

## Fidelity

Rigid frame and head (training heads are hard vinyl; real ears give a little). The temples are 4 x 8 mm, thicker than
real ones (about 3 x 6 mm); the 96 deg open stop's 6 deg splay stands in for real temples flexing round the head. The earpieces are rigid 75 deg bends that hook behind the ear roots; real ones are curved. The
hinges have 0.004 N m of friction; they are vertical axes, so gravity does not load that hold. Pads are rigid with
friction 0.9 for silicone. The eyelid rule is a 1 mm gap in simulation, a conservative proxy for the real trial's ink
contact. The simulator scores the final step without opening the grippers, so glasses still held score 0 (release is
checked from the finger positions).

## Files

- `put_reading_glasses_on_a_care_manikin.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (procedural geometry and joints, the closed-loop demo
  controller, geometry helpers).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="Put the reading glasses on the manikin: pads on the nose, both earpieces behind the ears, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=put_reading_glasses_on_a_care_manikin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/glasses-dream

# Scripted demo (both arms)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_reading_glasses_on_a_care_manikin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/glasses-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the scoring windows are measured from the head's landmarks.
- **Runtime per trial:** the demo takes about 53 s of simulated time (about 1,061 steps); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 250 s for the demo and about 370 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds (score 1.0), otherwise `score()` at the end of the run.

## Scoring

Whole run, score 0: any glasses collider ever within 1 mm of an eyelid (real trial: ink on the lids, any on the frame),
or the glasses touch the table (the tray is part of the setup; resting on it is not a touch).

Otherwise the glasses are scored once released (every finger pad point of both arms 3 cm clear of them), still (under
3 mm/s and 3 deg/s) for the last 1 s, and front up within 30 deg:

| Part | Credit | Condition (real trial: ruler, 3 mm feeler, front and side photos with a scale card) |
|---|---|---|
| Nose | 0.4 | both pads within 3 mm of the nose flanks; bridge within 6 mm of the midline and within 8 mm of its worn mark |
| Each ear | +0.2 | only with the nose: temple within 10 deg of its open stop; underside 0 to 4 mm over the root top; bend from 6 mm in front of to 15 mm behind the root's back edge (a hooked earpiece rests on its back corner); earpiece tip 5 mm or more below the root top and 7.2 to 8.6 cm out from the head axis |
| Fit | +0.2 | with the nose and both ears: roll and yaw within 4 deg; lenses 6 mm or more from the lids |

| Outcome | Score |
|---|---|
| untouched on the tray, still held, on the forehead, upside down | 0 |
| any eyelid touch, or dropped on the table | 0 |
| pads on the nose, both temples resting on top of the ears | 0.4 |
| one earpiece hooked | 0.6 |
| both hooked but rolled or yawed over 4 deg, or lenses within 6 mm of the lids (rare: a roll that large usually lifts one earpiece off its root, scoring 0.4) | 0.8 |
| worn, level, released | 1.0 |

Every window is measured from landmarks on the head, so any stock head can be used: the ear root top height and
lateral offset, the nose ridge line and flank angle, and the lid positions. In the real trial the earpiece tip's depth
below the root top and its distance out are read from a side photo with a scale card held behind the ear, and the pad
gaps with a 3 mm feeler.

## Real-world reset

Wipe and re-ink the eyelids. Set the glasses open on the tray at their marks, standing on lens bottoms and earpiece
tips, temples on their open stops.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
