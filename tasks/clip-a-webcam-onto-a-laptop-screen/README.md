# Clip a webcam onto a laptop screen

A webcam stands on the desk beside an open laptop. The robot must clip it onto the middle of the screen's top edge,
lens toward the keyboard: hook the clip's front lip over the edge so it hangs in front of the screen, then fold the
clip's hinged rear foot down until it lies against the back of the lid, and let go. Since 2020 this is an everyday
home-office step that most people have done themselves. It needs a two-sided fit over a thin edge that moves when
pushed: the lid is held open only by its friction hinge, so pressing the camera down off the lid's line, shoving the
foot hard into the lid or knocking the screen tips it; and a foot left open lets the camera hang tipped back on its
lip. One hand lowers the clip's throat over the 7 mm edge and keeps the camera seated; the other swings the foot about
90 degrees round its hinge and closes it onto the lid back by feel. Board tasks move a monitor (raise, pivot, restore
its height) or reseat a camera on a mount, and one clips an air freshener's fork along a car vent vane; this one closes
a hinged two-part clip round both faces of a laptop lid that moves, with the second hand folding the foot.

## Parts (real size, real material)

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left), heights above the
table top. Both objects are store-bought; nothing is printed.

| Part | Real product, dimensions and material | Where |
|---|---|---|
| Laptop | Acer Chromebook 314 CB314-3H (14 in; 326.4 x 225 x 20 mm closed, 1.45 kg), silver plastic shell, black keyboard deck and display. Base 13 mm thick; lid 7 mm thick at its top edge, 213 mm from the hinge axis to the top edge, about 0.45 kg | Turned so the screen faces away from the robot and the hinge is toward it; base x 29 to 51.5, centred on y 0, taped to the table along both side edges; hinge axis at x 29.5, 1 cm up; lid open 110 deg (leaning 20 deg toward the robot); top edge at x 22.2, 21.0 cm up |
| Webcam | Logitech C920e (part 960-001360), black ABS, 94 x 71 x 43.3 mm with its clip, 162 g. Pill-shaped body 94 mm wide, 24 mm deep, 29 mm tall (a 65 mm flat middle and two 14.5 mm radius ends), lens 16 mm across in a 22 mm silver ring on the front. Mounting clip under the body: head 19 x 40 x 8 mm that rests on the edge; front lip 40 mm wide, 4 mm thick with a rubber pad on its inner face, reaching 22 mm below the head under the lens side; rear foot 36 mm wide, 5 mm thick, 52 mm long, on a stiff hinge at the head's back edge, about 20 g | Standing on the table on the lip's bottom edge and the foot's end, foot swung 93 deg open from folded, lens toward +x, body level, body centre at x 17.5, y 19, 4.4 cm up |

**How the clip fits the lid.** The clip's throat (the lip pad's inner face to the folded foot's inner face) is 9 mm
across a 7 mm lid: 1 mm in front, 1 mm behind with the foot folded. The camera's centre of mass is behind the lid's
top edge, so once let go it hangs back on the edge until the lip's pad bears on the screen face: about 3 deg from
level with the foot folded (the folded foot's end on the lid back holds it there), more with the foot left open.
The clip's throat, lip and foot sizes, the camera's centre of
mass relative to the edge, the foot's hinge torque and the lid's push force are our estimates for the C920e and the
CB314-3H; before building, measure them on the bought units with calipers and a small spring scale (method in "Build
the real trial") and write the measured numbers here and in the task file.

**Forces.** The lid's friction hinge holds it against its own weight and the camera's (0.28 N m); about 2.4 N at the
top edge opens it one notch and about 5 N closes it one notch (the simulator's detent spring, below). The foot's hinge
clicks between 3 deg notches at about 2.6 N, 30 mm from its hinge. Holding the 162 g camera needs a few newtons of
grip. Every force the real action needs is far below 40 N per finger.

## What the scripted demo does

The demo reads the live pose of the webcam, its foot angle and the lid angle from the simulator, and nothing else.
The demo (`demo_script`, closed loop at 20 Hz) reads the webcam's pose, its foot angle and the lid angle from the
simulator, and plans from them and from the fixed layout (object_jitter=0). No tool point is commanded faster than
0.10 m/s.

1. Left arm: leaned 29 deg about the pad line toward its own base for height, pads across the body's 24 mm front and
   back, 25 mm left of the body centre (off the lens). It opens to 50 mm, comes down over the body, closes, checks
   from the gripper encoder that the pads stopped on the body (retries once), and lifts 8 cm.
2. Left arm: carries the camera level to 30 mm above its seat along the lid's line (20 deg off vertical), then lowers
   it down that line at 8 mm/s. Every step it compares the camera's read throat point with the lid's mid-plane and
   adds the difference to the hand command (arm sag and grip shift otherwise put the lip on top of the edge), so the
   lip slides down 1 mm in front of the screen; it stops when the throat is within 1 mm of the edge and keeps
   holding. Lowering along the lid pushes the lid toward its hinge, not round it.
3. Right arm: closed fingers, hand leaned forward. It comes in behind the lid, sets its fingertips on the open foot's
   outer face 30 mm from the foot hinge and 12 mm right of the camera's centre, and swings the foot round its hinge on
   that 30 mm arc, its pitch turning with the face (up to 66 deg), from the read foot angle to 5 deg past folded (the
   lid back stops the foot). It reads the foot angle and, if the foot is not on its flat notch (under 1.5 deg), pushes
   3 deg further, up to three times. It backs off 3 cm toward the robot and rises 5 cm. All through this step the
   left hand reads the throat and holds it 0.5 mm behind the lid's mid-plane (the push otherwise shoves the camera
   toward the screen, and the foot meets the lid back short of its flat notch).
4. Left arm: opens to 50 mm, rises 4 cm straight up (pad tips clear of the body) and moves back toward its own side.
5. Both arms move slowly toward their ready poses and swing there with the grippers open, well clear of the laptop.

## Fidelity

All parts are rigid bodies at full size. The lid's friction hinge is a detent spring: 60 N m/rad toward the nearest
of 30 notches 1.5 deg apart (90 to 135 deg open), so the lid stays where it is left and sags 0.27 deg under its own
and the camera's weight; 2.4 N at the top edge opens it a notch and 5 N closes it. The foot's stiff hinge is a detent
spring too (3 N m/rad, notches every 3 deg, -8 to 140 deg). The camera's tilt hinge (between body and clip) is fixed
with the body level for a lid at 110 deg. The clip's rubber pads are rigid with friction 1.0, and the USB cable is not
modelled. The simulator scores the last step without opening the grippers, while a real trial measures after the robot
lets go: a camera still in a hand counts as not released (0), and a camera held in the air is not resting on the edge,
so it is not seated (0).

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Clip the webcam onto the middle of the laptop screen's top edge, lens toward the keyboard, foot folded against the back, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clip_a_webcam_onto_a_laptop_screen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/webcam-dream

# Scripted demo (full score): the left arm seats the webcam on the edge and holds it; the right arm folds the foot
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clip_a_webcam_onto_a_laptop_screen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/webcam-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the demo plans from the fixed layout, and the lid's 2 deg
  limit is measured from its start.
- **Runtime per trial:** the demo takes about 67 s of simulated time (about 1,340 steps); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 260 s for the demo and about 350 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` (a full score, both hands parked 10 cm or more from the webcam) holds for 8 s, otherwise `score()` at the end of
  the run.

## Scoring

Read after the robot has let go, in this order:

1. **0** if either hand is within 3 cm of the webcam, or the webcam moves over 2 mm or turns over 2 deg during
   the last 1 s (two photos 1 s apart); if the lid was ever more than 2 deg off its start angle for 0.5 s or more (side photo against a protractor card
   behind the hinge; 1 deg moves the top edge 3.7 mm); or if the webcam, once lifted, rested on the table or the
   keyboard deck for 0.5 s or more (dropped).
2. **Seated** (else 0): the clip head on the lid's top edge (the throat's top within 3 mm of the edge, the lid between
   the lip and the foot); the lip on the screen side, its bottom 10 mm or more below the edge; the lens within 10 deg of
   square to the screen; the body's centre within 2 cm of the lid's centre line.
3. **Folded:** the foot within 6 deg of lying flat on the lid back (its end 6 mm or less from the lid back, ruler).
   **Tilt:** the body top's angle from level, from a digital angle gauge laid on the top.
4. **1.0** seated, folded and tilt 4 deg or less. **0.5** seated, folded, tilt over 4 and up to 10 deg. **0.25**
   seated with tilt up to 20 deg (this is a camera hooked on with its foot left open, which tips back further).
   **0** otherwise.

Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0. Perching the camera on the edge without the lip in front,
hanging it backwards, leaving it off the middle, dropping it, still holding it, or pushing the lid scores 0.

## Build the real trial

### Parts list

| Item | Source |
|---|---|
| Laptop | Acer Chromebook 314 CB314-3H (about 250 USD); any 14 in laptop whose lid is 5 to 8 mm thick at its top edge and opens to at least 135 deg also works, with its measured numbers |
| Webcam | Logitech C920e, part 960-001360 (C920 family, same clip) |
| Tape | 25 mm double-sided removable mounting tape (two strips under the laptop's side edges) and 6 mm masking tape for marks |
| Gauges | Digital angle gauge (for example a 0.1 deg magnetic bevel box), protractor card printed at 1:1, 150 mm steel ruler, 0 to 10 N spring scale |

### Measure before the first trial

1. Webcam: calipers on the clip's throat (lip inner face to the folded foot's inner face), the lip's depth below the
   head, the foot's length and width, and the head's depth. Balance the camera on a straightedge, foot folded and then
   open, to find its centre of mass relative to the throat.
2. Foot hinge: with the camera held, pull the foot's end 30 mm from its hinge with the spring scale at right angles;
   record the force at which it turns.
3. Lid: with the laptop taped down and the lid at 110 deg, push the middle of the top edge with the spring scale at
   right angles, toward the keyboard and away from it; record the force at which it moves.
4. Write the measured numbers in this README and the task file (the scene constants) before the trials.

### Assembly

1. Mark the laptop's base outline with masking tape so its hinge axis sits 29.5 cm in front of the arm bases' midpoint,
   centred between them, screen facing away from the robot. Tape the base down with the two strips of mounting tape.
2. Open the lid to 110 deg: the angle gauge on the lid back reads 70 deg from the table.
3. Mark the webcam's spot: the lip's bottom edge and the foot's end on the table with tape, body centre at x 17.5,
   y 19, lens toward the laptop.

### Reset and cleaning between trials

Lift the webcam off the lid, swing its foot to 93 deg open on a card gauge, and stand it on its tape marks. Reset the
lid to 110 deg with the angle gauge. Wipe the lid's top edge and the clip's pads with a dry microfibre cloth (no
liquids near the laptop). Keep the laptop closed and powered off during trials; its lid hinge is the only part that
moves.

## Files

- `clip_a_webcam_onto_a_laptop_screen.py`: the task file (scene, `score()`, `success()`, the scripted demo).
- `mechkit.py`: the kit that builds the laptop and webcam as jointed objects with real-shape parts.
- `closedloop.py`: the closed-loop demo controller. `belimits.py`: shared scoring helpers.
- `fabricate.py`: writes STL files of the two reset gauges (lid angle and foot angle) when run as a script.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
