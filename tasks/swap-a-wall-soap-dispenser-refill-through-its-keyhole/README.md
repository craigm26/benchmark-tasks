# Swap a wall soap dispenser's refill: drop its pump through the keyhole, slide it into the yoke, test-push

The functional test comes first: after the swap, one push on the bar must stroke the refill's pump. In a push-bar
restroom soap dispenser the bar's lever ends in a fork (the yoke) under the bottle, and the fork lifts the pump collar
only when the collar sits over it. To get it there the pump goes down through a keyhole entrance in the ledge, hidden
under the bottle, and the bottle then slides 43 mm back until its neck stops at the end of a slot. A refill left at the
entrance, perched on its collar or not slid fully back dispenses nothing, so the hidden path is what the score
measures. Unlike swapping a roll or a cartridge, the part follows a two-axis path the hand cannot see and must engage
its actuator. Janitors and cleaners (O*NET 37-2011.00, 2.2 million US workers) and maids and housekeeping cleaners
(37-2012.00, 0.9 million) do this on every restroom round ("Service, clean, or supply restrooms"). Unlocking the cover
with its key is not tested: the cover starts hanging open on its stop.

## Parts (real size)

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
scores without opening the grippers, so a refill still held, or with a finger within 3 cm, scores 0.

An integrator's declared limit runs beside the score, not in it: the pinch line between the closing cover's top
plate and the back plate's top edge (x 37.5 to 41.0, y -7.5 to 7.5, 29 to 34 cm up) is a keep-out for the tool and
fingertip points, so the cover is pushed shut on its face. `ARM_KEEP_OUT` in the task file declares it.

## Files

- `swap_a_wall_soap_dispenser_refill_through_its_keyhole.py`: the task file for the `yam_arena` embodiment (scene,
  `score()`, `success()`, `demo_script()`).
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
  Wall-clock time on one L40S, including Isaac Sim start-up: about 360 s for the demo and about 510 s for a full Dream-1 trial.
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
the bar visibly moves the collar 5 mm, or soap appears at the nozzle of a real refill. D from the video.

## Real-world reset

Open the cover with its key and let it hang on its 60 degree stop. Slide the new refill forward to the entrance, lift
it out and stand it on the cradle. Seat the spent refill (an empty bottle with its pump) at the slot end. Empty the
bin. With real soap, wipe the nozzle; a refill whose pump has been stroked can be reused for trials.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
