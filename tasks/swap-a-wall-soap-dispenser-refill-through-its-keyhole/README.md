# Swap a wall soap dispenser's refill: drop its pump through the keyhole, slide it into the yoke, test-push

The functional test comes first: after the swap, one push on the bar must stroke the refill's pump. In a push-bar
restroom soap dispenser the bar's lever ends in a fork (the yoke) under the bottle, and the fork lifts the pump collar
only when the collar sits over it. To get it there the pump goes down through a keyhole entrance in the ledge, hidden
under the bottle, and the bottle then slides 43 mm back until its neck stops at the end of a slot. A refill left at the
entrance, perched on its collar or not slid fully back dispenses nothing, so the hidden path is what the score
measures. Unlike swapping a roll or a cartridge, the part follows a two-axis path the hand cannot see and must engage
its actuator. Janitors, housekeeping cleaners and restroom attendants on restroom rounds (O*NET 37-2011.00 and
37-2012.00, task "Service, clean, or supply restrooms") swap a refill whenever a dispenser runs dry. Unlocking the cover
with its key is not tested: the cover starts hanging open on its stop.

## Parts (real size, as simulated)

Positions are in cm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.
The dispenser's centre line is y 0. The trial rig is the real unit, which hangs at 110 to 120 cm on a restroom wall,
screwed to a board at table height.

- **Splashback board**, plywood painted pale blue with white tile lines, 40 x 36 cm, 1.8 cm thick, front face at
  x 40.5, fixed.
- **Dispenser body**, white ABS, fixed to the board; the whole unit is 125 mm wide, 148 mm deep, 271 mm tall. Back
  plate 125 x 265 x 8 mm (x 39.7 to 40.5, 5 to 31.5 cm up). Two side walls 135 x 100 x 5 mm (x 26.2 to 39.7, 5 to
  15 cm up), 115 mm apart inside. The sides are open above the ledge.
- **Ledge**, dark grey ABS, 6 mm thick, top 15.0 cm up, between the side walls: six flat strips round a keyhole. The
  entrance is 44 x 44 mm (x 28.8 to 33.2); a 34 mm wide slot runs back from it to x 36.7.
- **Cover**, white ABS, 5 mm, 0.26 kg, hinged along its bottom edge at x 26.2, 9.0 cm up: face 125 x 226 mm with a dark
  sight window, top plate 125 x 143 mm with a rounded front edge, two side returns 25 mm deep. Open, it leans 60 degrees
  forward on its stop (gravity holds it there with 0.31 N m). A detent spring on the hinge (0.35 N m/rad) pulls it to
  whichever of open and shut is nearer, so it goes shut once pushed past 30 degrees and stays shut.
- **Push lever**, grey ABS, 80 g, pivot at x 29.0, 12.55 cm up, travel 0.12 rad, return spring 0.3 N m/rad: dark push
  bar 110 x 30 x 12 mm below the cover (front face x 25.2, 5 to 8 cm up); two 10 mm side arms; a 5 mm fork plate
  100 mm wide (x 33.4 to 38.0, top 12.0 cm up) with a 24 mm U slot open to the front and closed at x 36.6; two
  contact ridges 8 x 5 x 3 mm either side of the slot (x 34.9 to 35.7, top 12.3 cm up); a 26 x 90 x 22 mm
  counterweight behind the pivot, which puts the lever's centre of mass 3.2 cm behind the pivot so gravity rests it
  on its stop.
- **Two refills**, HDPE, 600 mL, 188 mm tall, pump down. Bottle 76 (x) x 64 (y) x 150 mm with 6 mm chamfers; neck
  28 mm across; pump 22 mm across (neck and pump 22 mm long together); pump collar 38 mm across and 4 mm thick, 20 g,
  with a 10 mm nozzle 12 mm long. The collar strokes up to 10 mm into the bottle against a 250 N/m spring.
  - **Spent refill**, grey, 0.12 kg, seated: neck axis at x 35.3, y 0 (neck against the slot end), bottle on the ledge,
    collar 1 mm above the fork ridges.
  - **New refill**, white with a blue band, 0.66 kg (600 mL of soap), standing pump down on a blue ABS cradle at
    x 22.0, y -19.0: two rails 100 x 27 x 50 mm, 46 mm apart, the pump hanging between them.
- **Drip tray**, white, 120 x 100 x 15 mm with 2 mm walls, on the table under the dispenser (x 27.0 to 39.0, y -5.0
  to 5.0), fixed. It sits under both nozzle positions (entrance and seat) and stays 3.5 cm below the push bar, so it
  catches any soap a real pump gives at the test push.
- **Waste bin**, dark grey plastic, fixed, inside 180 x 180 mm, 150 mm tall, 3 mm walls, centred at x 22.0, y 17.0.
- **Fits:** collar 38 mm in the 44 mm entrance (3 mm per side); neck 28 mm in the 34 mm slot (3 mm per side); pump
  22 mm and nozzle 10 mm in the fork's 24 mm U slot (1 and 7 mm per side). The collar is wider than the slot, so a seated refill cannot be lifted
  straight out. At full lever travel the ridges rise 8 mm and stroke the collar 7 mm with 1.8 N, touching only the collar, well under the full
  refill's 6.5 N weight, so the push does not lift the bottle. With the refill at the entrance the fork is 5 mm behind
  the collar and a push strokes nothing.

## The scripted demo

Both arms, about 1,600 steps (80 s) of the 2,400 allowed. Every bottle grasp is a top grasp with the hand leaned 57
degrees forward, pads closing across the bottle's 64 mm flat sides, tool point 2.5 cm below the bottle's top.

1. **Spent refill out (left).** Grasp the seated spent refill, then draw it 43 mm forward to the keyhole entrance,
   following its read pose.
2. **Bin it (left) while the right picks.** The left lifts the spent refill 8 cm (nozzle 4 cm over the side wall),
   carries it over the wall to the bin, lowers it until the nozzle is 2 cm over the bin floor, lets go and parks. The
   right grasps the new refill on its cradle and lifts it to carry height (nozzle 27 mm over the side walls).
3. **New refill in (right).** Carry to the entrance, centre on the read pose, lower at 3 cm/s and then 1 cm/s with the
   pose fed back until the bottle is on the ledge: the collar is then through the entrance.
4. **Slide home (right, still holding).** Slide 43 mm back along the slot at 1 cm/s until the neck meets the slot
   end (read pose within 2.5 mm of the seat, or stalled), let go, rise clear.
5. **Shut (left).** Closed fingertips on the open cover's outer face, 21 cm from the hinge, push along the closing
   arc, aimed 0.15 rad ahead of the read hinge angle (further ahead, up to 0.5 rad, while the hinge is not turning). Once the hinge reads 6 degrees past the midpoint the hand backs off and the detent spring takes it shut. One retry if
   it falls back open. The left parks.
6. **Test (right).** Closed fingertips push the bar in at 1 cm/s until the lever reads 0.105 rad, hold 1.5 s, let go;
   the right parks. Both arms end open at the ready pose, clear of everything.

The demo reads the live poses of both refills and the cover and lever joint angles from the simulator, plans from the
fixed layout (object_jitter=0), and uses its joint encoders and gripper opening to trim wrist sag and confirm each
grasp. It never moves an object or edits the scene. The keyhole drop, the fork and the pump stroke are under the
bottle and cannot be seen from the top camera: the end of the video shows the cover shut and the bar pushed, and the
score reports the stroke.

## Fidelity

Real size and rigid parts throughout. The 600 mL of soap is a solid fill: no slosh, no flow, no priming. The pump is
a collar sliding 10 mm on a 250 N/m spring, softer than a real pump (about 10 N), so that the arm's 3 to 4 N push
strokes it. The cover's catch is a detent spring on its hinge (0.35 N m/rad, flipping 30 degrees from shut); the key
and lock are not modelled. The ledge is six flat strips, so the slot end is square, not rounded. The simulator
scores without opening the grippers, so a refill still held, or with a finger within 3 cm, scores 0. The real rig
(see Building the real rig) is printed from fabricate.py at the same dimensions; its refills carry steel shot instead
of soap and its cover latches on a magnet.

An integrator's declared limit runs beside the score, not in it: the pinch line between the closing cover's top
plate and the back plate's top edge (x 37.5 to 41.0, y -7.5 to 7.5, 29 to 34 cm up) is a keep-out for the tool and
fingertip points, so the cover is pushed shut on its face. `ARM_KEEP_OUT` in the task file declares it.

## Files

- `swap_a_wall_soap_dispenser_refill_through_its_keyhole.py`: the task file for the `yam_arena` embodiment (scene,
  `score()`, `success()`, `demo_script()`).
- `fabricate.py`: writes a print-ready ASCII STL (mm) of every custom part, from the scene's dimensions
  (`python3 fabricate.py [out_dir]`, standard library only).
- `mechkit.py`: jointed parts and procedural geometry. `closedloop.py`: the closed-loop demo controller.
  `belimits.py`: scoring geometry helpers.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Swap the empty soap refill for the new one, put the empty in the bin, shut the cover and test it with one push."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_a_wall_soap_dispenser_refill_through_its_keyhole.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/soap-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swap_a_wall_soap_dispenser_refill_through_its_keyhole.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/soap-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the layout is fixed and no seed is used.
- **Runtime per trial:** the demo takes about 1,600 steps (80 s simulated) and the episode ends 8 s after full score
  is first held; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 380 s for the demo and about 580 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

Read from poses and joint values. S, N and L are judged at the end; P and D latch over the whole run.

- **S**: the spent refill's centre is inside the bin's inner 18 x 18 cm footprint and its lowest point is 1 cm or
  more below the rim.
- **N**: the new refill's neck axis is within 4 mm of the slot-end seat (x 35.3, y 0), its base within 2 mm of the
  ledge top, its tilt 5 degrees or less.
- **L**: the cover is within 2 degrees of shut.
- **P**: after N and L have held for 1 s, the lever turns 0.08 rad or more and the pump collar strokes 5 mm or more.
- **D**: the new refill drops 5 cm or more within 0.5 s while released, or tilts past 45 degrees.
- **End condition:** both refills at rest (under 1 cm/s for 1 s) and both grippers 3 cm clear of them; otherwise 0.

| Outcome | Score |
| --- | --- |
| S, N, L and P | 1.0 |
| S, N and L | 0.75 |
| S and N | 0.5 |
| S alone, or N without S | 0.25 |
| anything else, including nothing done | 0 |

D caps the score at 0.25. A refill left at the entrance, perched on its collar on top of the ledge, or still held
earns no N and no P. Pushing the bar with the cover open earns no P.

In a real trial: S by eye. N with a ruler, since the neck is hidden: the bottle's back face stands 6 mm from the back
plate (within 4 mm) and the bottle sits flat on the ledge. L by eye: the cover's top plate is level. P: one push on
the bar hides the pen line drawn round the pump plunger 5 mm below the neck (the collar has stroked 5 mm); watch it
from below the ledge. D from the video.

## Building the real rig

No commercial dispenser has this exact keyhole-and-fork layout (the push-bar units we found, such as the Impact
Foam-eeze 9326, take a refill differently), so the dispenser, its refills and the small parts are printed from
`fabricate.py`. It writes one ASCII STL per custom part, in mm, from the same dimensions as the scene:

```bash
python3 fabricate.py stl      # writes stl/*.stl and prints each part's size and volume
```

### Parts list

| Part | Qty | Source |
| --- | --- | --- |
| `ledge.stl`: keyhole plate, 135 x 115 x 6 mm | 1 | print, PETG or ABS, dark grey, 100% infill |
| `back_plate.stl`: 8 x 125 x 265 mm | 1 | print on a bed of 270 mm or more, or cut from 8 mm white ABS sheet |
| `side_wall.stl`: 135 x 5 x 100 mm with hinge boss and two 3.4 mm pin holes | 2 | print, white |
| `cover.stl`: face, top plate, side returns and two hinge knuckles | 1 | print, white, lying on its face; weigh 0.26 kg (set the infill) |
| `lever.stl`: push bar, arms, pivot lugs, U fork, ridges, counterweight | 1 | print, grey; weigh 80 g (about 30% infill) |
| `lever_rest_stop.stl`, `lever_travel_stop.stl`: 17 x 20 x 10 mm and 9 x 20 x 17 mm blocks | 1 each | print |
| `pin_spacer.stl`: 8 mm OD, 7.5 mm long | 2 | print |
| `refill_bottle.stl`: 76 x 64 mm shell, 1.2 mm walls, neck with a 22.5 mm bore | 2 | print PETG, shoulder face down; one grey, one white with a blue tape band at 45 to 95 mm |
| `refill_lid.stl`: chamfered base with a press-in plug | 2 | print PETG |
| `refill_plunger.stl`: nozzle, 38 mm collar, 22 mm plunger, retaining flange | 2 | print PETG, dark grey, 100% infill |
| `refill_spring_cup.stl`: spring cup | 2 | print PETG |
| `cradle_rail.stl`: 100 x 27 x 50 mm | 2 | print, blue |
| `drip_tray.stl`: 120 x 100 x 15 mm, 2 mm walls | 1 | print, white |
| `waste_bin.stl`: inside 180 x 180 x 150 mm, 3 mm walls | 1 | print, dark grey (or any bin with the same inside size) |
| Board, 400 x 360 x 18 mm plywood, painted pale blue with white tile lines | 1 | hardware store plywood |
| Two steel shelf brackets, 100 mm, and two C-clamps | 1 set | hardware store; hold the board upright at the table's back edge |
| 3 mm steel rod, two 125 mm lengths (hinge pin and pivot pin) | 2 | any 3 mm steel rod or drill rod |
| Pump spring: music-wire compression spring, 12.7 mm OD, 0.8 mm wire, free length 32 mm, about 0.25 N/mm (1.4 lbf/in) | 2 | McMaster-Carr or Lee Spring stock compression springs, cut to 32 mm |
| Lever return spring: 90 degree torsion spring for a 3 mm pin, about 0.005 N m per degree (0.05 in-lbf/deg) | 1 | McMaster-Carr 9271K torsion spring range |
| Cover catch: neodymium disc magnets 3/8 x 1/8 in | 2 | K&J Magnetics D62 |
| 2 mm nylon cord, 15 cm (cover check strap) | 1 | any |
| Steel BBs (4.5 mm), in a small zip bag per refill | about 0.6 kg | any airgun BB tin |
| M4 x 16 wood screws (back plate to board), epoxy or ABS cement | 4 | hardware store |

### Assembly

1. Print every part (two of each refill part make one spent and one new refill). Run a 3.2 mm drill through the
   square pin holes.
2. Glue the two side walls to the back plate's front face, 115 mm apart inside, bottom edges 50 mm above the board's
   bottom edge, hinge bosses at the front and low. Glue the ledge between the walls, flush with their tops, the
   keyhole's entrance towards the front.
3. Lever: slide the pivot pin through one wall, a spacer, the lever's two lugs (torsion spring between them, one leg
   on the lever arm, one on the ledge's underside), the other spacer and the other wall. Glue the rest stop to the back
   plate's face under the counterweight (top 93 mm up, where the lever rests with the fork's top 120 mm up) and the
   travel stop under the ledge's back strip (bottom 127 mm up), which ends the push at 0.12 rad with the ridges
   about 8 mm higher.
4. Cover: slide the hinge pin through a wall boss, both cover knuckles and the other boss. Tie the nylon cord from the
   left side return to the left wall so the open cover hangs 60 degrees forward. Glue one magnet under the top plate's
   rear edge and the other on the back plate's top edge so they meet when the cover is shut.
5. Screw the back plate to the board (dispenser centred, back plate bottom 50 mm above the table), stand the board on
   the table with its face 40.5 cm in front of the arm bases' midpoint, and clamp the shelf brackets to the table.
6. Refills: drop a plunger into each bottle from the open end (flange on the inner face, plunger out through the neck),
   stand a pump spring on the flange, glue the spring cup over it. Draw a pen line round the plunger 5 mm below the
   neck. Add BBs in a zip bag until the refill with its lid weighs 0.66 kg (new) or 0.12 kg (spent), then press the
   lid in. Check that a 1.5 N push on the collar strokes it about 5 mm.
7. Place the cradle rails 46 mm apart at x 22.0, y -19.0, the bin at x 22.0, y 17.0, and the drip tray under the
   dispenser, lined with a folded paper towel. Seat the spent refill, stand the new one on the cradle.

Differences from the simulation: the cover is held shut by the magnets instead of a detent spring, so it must be
pushed nearly shut before it latches, and the open stop is the check cord. The printed parts add what the simulation's
joints stand for: pin holes, hinge bosses and knuckles, pivot lugs, the two lever stops, the plunger's bore, flange and
spring cup. The cover's top plate has a square front edge instead of a 3 mm round, and the refills are PETG with steel
shot, not HDPE with soap.

## Real-world reset and keeping the rig clean

Lift the cover onto its 60 degree stop. Slide the new refill forward to the entrance, lift it out and stand it on the
cradle. Seat the spent refill at the slot end. Empty the bin. The printed refills carry steel shot, not soap, so the
test push dispenses nothing; the paper-towel-lined drip tray under the nozzles catches anything if the rig is used with
a wet pump. Wipe both nozzles, change the paper towel and wipe the tray at each reset.

## Licence

All geometry is procedural, generated by mechkit.py (simulation) and fabricate.py (print files) in this folder; no
third-party assets.
