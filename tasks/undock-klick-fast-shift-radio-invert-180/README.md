# Invert a locked shift radio 180 degrees in its wall dock, lift it out, and stand it upright

A shift radio hangs in an anti-drop wall dock (a Peter Jones Klick Fast Dock 05 with a Stick-On Stud on the radio's
back). The dock holds it against a pull forward and against a lift straight up: the front lips stop the stud's round
collar, and a lug inside stops the C-shaped collar's closed side. Only after the radio is turned a half turn about the
stud does the collar's gate meet the lug, and then the radio slides straight up and out. The robot has to make that half
turn, lift the radio clear, and stand it upright on a green pad. One wrist cannot make the whole turn from a side grip
(joint 6 stops at +-120 degrees), so the two hands pass the radio between them. Security guards (O*NET 33-9032.00),
police patrol officers (33-3051.00) and correctional officers (33-3012.00) dock and undock radios and body cameras on
anti-drop mounts like this at every shift change; the task tests the physical skill of freeing a captive part by turning
it, not the walk to the dock.

## Parts, sizes and materials (all at real size)

| Part | Product (model or part number) | Size | Material | In the scene |
|---|---|---|---|---|
| Dock | Peter Jones Klick Fast Dock 05, FG-42-000013 | 46 x 46 mm plate, 14.5 mm deep with its lips | black nylon | fixed, axis 14.5 cm above the table, centred y 0 |
| Dock detail | (the product's channel and lug) | 36 mm wide U channel open at the top, two 3 mm lips leaving a 20 mm slot, 6 x 6 x 4 mm lug on the backplate | nylon | fixed, lug amber |
| Stud | Peter Jones Klick Fast Stick-On Stud, FG-41-000076 | base plate 32.8 x 39.8 x 2 mm, neck r 7 mm, flange disc r 14.5 mm, C collar r 8.5 / 15 mm, 3.5 mm thick, gate 90 degrees open (12 mm wide at its inner radius) | nylon | on the radio's back |
| Radio | Retevis H-777 (fleet equivalent Motorola CLS1110) | body 102 x 56 x 34 mm, yellow 12 mm top cap, side button, two silver 11 mm knobs, 52 mm stubby antenna | ABS | loose, 132 g with stud, friction 0.45 |
| Console | wall console panel and base | panel 16 x 150 x 200 mm, base 80 x 160 x 15 mm | aluminium, friction 0.35 | fixed at x 32.8 cm |
| Pad | green handover mat | 90 x 70 x 6 mm | matte ABS or silicone | fixed at x 22, y -13.5 cm |

The stud is modelled 11.5 mm deep from base plate to collar back against 10.35 mm for the product, so the radio has
about 1 mm of axial play in the dock (the real dock has the same kind of play). Heights and positions are in cm from the
midpoint between the arm bases (x forward, y left): console x 32.8, dock axis 14.5 above the table top, radio centre
x 28.5, pad x 22, y -13.5.

## What the scripted demo does and what it reads

The right hand takes the upright radio by its 56 mm side walls, 30 mm above the stud axis, and turns it 90 degrees about
the stud with its tool point moving on a 30 mm circle about that axis. The left hand then takes the lower end across the
radio's top and bottom walls, the right hand lets go and clears, and the left hand turns it the other 90 degrees, which
puts the collar's gate over the lug. The left hand lifts it 45 mm straight up (the collar clears the dock top) and draws
it 55 mm back. It turns the radio back to horizontal; the right hand takes the other end, the left lets go and clears,
and the right hand turns the radio upright, carries it to the pad, lowers it until its base touches, opens slowly and
backs out. Both hands finish parked at their ready pose. The hands are pitched 20 degrees below level, which only
helps the right arm reach the pad; the pads still close along the radio's width. Fingers are commanded to a 47 mm gap on
the 56 mm radio (about 5 mm per finger, roughly 25 N), which centres the radio between the pads. Speeds stay under
0.15 m/s at the tool point. The demo reads the live pose of the radio from the simulator (once, to square it over the
pad, and every step of the last descent to stop at the pad) plus its joint encoders and gripper opening; every other
target comes from the fixed layout (object_jitter=0). The demo never moves an object except with the grippers.

## Fidelity

Approximate. Dock, stud and radio are rigid bodies at product size; the collar turns in the channel by contact, the lug
and lips block by contact, and nothing is placed by a formula. The product's flexing tongue and its 7-position detent are
modelled as a rigid lug and nylon friction (0.35 to 0.45); the maker publishes no rotation torque, so the turn takes
only the friction of a 132 g radio resting on the cradle (about 0.01 N m). The sim cannot tell a held radio from a
released one and scores the final step without opening the grippers: a radio held in the air is never on the pad (its
base must be on the pad top) and never released, so it scores below the released case. The radio is a rigid block with
box, cylinder and turned shapes, not a moulded case.

## Run a trial

```bash
INSTR="Invert the shift radio 180 deg in the wall dock, slide it straight up free, and stand it upright on the green pad."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=undock_klick_fast_shift_radio_invert_180.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/undock-dream

# Scripted demo (both arms)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=undock_klick_fast_shift_radio_invert_180.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/undock-demo
```

- **Seed and layout:** `-E object_jitter=0` is required (the dock and the pad are fixed, the radio starts locked in the dock).
- **Runtime per trial:** the demo takes about 87 s of simulated time (1,746 steps) and holds 8 s after full score
  (`SUCCESS_HOLD_S = 8.0`) so the video shows both hands parked; a Dream-1 trial runs up to 2,400 steps (2 minutes).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 290 s for the demo and about 490 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s, otherwise `score()` at the end.

## Scoring

Sustained means 10 control steps (0.5 s at 20 Hz). x_stud, z_stud and roll are the collar's centre and the radio's turn
about the stud axis (0 upright, 180 inverted).

- **Zero for the whole run:** the radio's lowest point goes 2 cm under the table top, or the stud is forced out forward
  through the lips (collar x within 2 cm in front of x 30.4 cm, in the dock's height band, while not turned to within
  30 degrees of 180).
- **Inverted:** roll within 20 degrees of 180 with the stud still in the dock (within 12 mm of its axis), sustained.
- **Freed:** inverted earlier, and the stud lifted more than 2.6 cm above the dock axis or the radio drawn forward to
  x < 28.5 cm.
- **On the pad:** body within 8 degrees of upright, its base within 3 mm of the pad top, its centre within 2 cm of the pad centre.
- **Released:** both grippers 3 cm clear and the radio under 5 mm/s and 3 degrees/s, for the last 2 s.
- **Score:** 0 if never inverted in the dock (a straight pull scores 0); 0.25 inverted in the dock; 0.50 freed and
  released and still for 2 s, anywhere but on the pad; 0.75 freed and on the pad without the 2 s of release; 1.00 freed,
  on the pad, released and still for the last 2 s.

In a real trial the turn is judged from video (the antenna points down within 20 degrees while the stud is still in the
dock), the pad target with a ruler (20 mm) and the upright tilt with a phone level (8 degrees).

## Real-world build and reset

Bill of materials (approximate retail, US dollars): Klick Fast Dock 05 FG-42-000013, 10 to 20; Stick-On Stud FG-41-000076,
4 to 11; Retevis H-777 radio, 25 to 35; 150 x 200 mm panel on a base with four M4 screws, plus a 90 x 70 x 6 mm green mat, 12 to 18;
total 51 to 84, nothing consumed. A cheaper printed route for the dock and stud: run `python3 fabricate.py` (standard library
only, writes `dock_05_model.stl` and `stud_model.stl` in millimetres from the same dimensions as the scene) and print in
PA12 nylon or PETG, 0.12 mm layers, 100 % infill. Assembly: screw the dock to the panel face with its open side up, stick
the stud to the radio's back, mark the pad outline on the table. Reset: hang the radio upright in the dock (about 10 s);
the pad stays. Forces: the lift is the radio's weight (1.3 N); each finger squeezes about 25 N, under the 40 N limit.
Employment counts for the three occupations come from the US Bureau of Labor Statistics occupational employment
statistics (about 1.2 million security guards, 0.7 million patrol officers, 0.4 million correctional officers).

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
