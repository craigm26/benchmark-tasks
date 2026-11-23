# Tilt-slide a quick-acting knob down a stud, square it on the washer, and turn it until its own thread clamps it snug

AI hardware reliability engineers and qualification technicians clamp edge compute boxes and GPU carrier modules onto
vibration shaker subplates with slotted aluminum crossbars, vertical M8 studs and quick-acting knobs. The maker's
instructions for the Carr Lane quick-acting four-prong knob (CLM-8-QAK, M8x1.25) are: "Tilt knob to slide in.
Straighten knob to engage threads, then tighten knob to clamp." Pushed straight down, the knob's half-threads jam on
the top thread crests. Tilted, its relieved bore slides freely down 55 mm of exposed stud. Squared on the washer over
the crossbar, its half-threads catch, and a short clockwise turn clamps the crossbar. How far it turns depends on
where the thread catches; the operator stops when it is snug.

## Parts, dimensions, and materials

All coordinates are in the station frame (table top `TOP = 0.7475 m`; `+x` forward, `+y` left, `+z` up):

- **T-slot subplate and edge compute module (`plate`, fixed):** 6061-T6 aluminum subplate, 160 x 200 x 12 mm, at
  `(0.260, 0.0, TOP)` carrying a dark-anodized extruded aluminum edge compute enclosure (90 x 103 x 35 mm, modeled
  after a Hammond 1455-series housing) with silver heatsink stripes and a green PCB edge indicator.
- **Crossbar and M8x1.25 stud (`crossbar`, fixed):** 6061-T6 aluminum crossbar, 25 x 180 x 12 mm (top at
  `TOP + 0.059 m`), spanning `y = -0.090..+0.090 m` on two 35 mm aluminum spacer sleeves at `y = +0.075` and
  `y = -0.075 m`. The left stud (`+0.075 m`) is pre-clamped by a hex nut; the active right M8x1.25 steel stud at
  `(0.260, -0.075 m)` rises 55 mm above the crossbar to `TOP + 0.114 m` (5.0 mm core diameter with 8.0 mm
  thread-crest collars at 15, 29, 36, 43 and 49.5 mm above the crossbar and a chamfered tip).
- **Washer (`washer`, free, 11 g):** a zinc-plated **DIN 6340 M8 washer** (8.4 mm bore, 23 mm OD, 4 mm thick) lying
  flat on the crossbar round the right stud (top at `TOP + 0.063 m`).
- **First thread crest (`thread`, fixed Mech with a kinematic screw):** the stud's first crest (8.0 mm OD, 5.4 mm bore,
  3 mm tall), inside the washer's bore with its top 0.61 mm above the washer. It rides a mechkit kinematic screw
  (M8x1.25, right hand, 1.25 mm lead). It stands for the turn of thread the knob's half-threads catch. Its seat, a
  quarter turn (0.31 mm) down, is where the knob face meets the washer. Running friction 0.0008 N m, plus 0.03 N m per
  turn past the seat (at most 0.01 N m). Modelled at 20 g with a 1e4 N/m thread so the MuJoCo replay stays stable.
- **Staging block (`stage`, fixed):** anodized blue aluminum block, 44 x 44 x 12 mm, at `(0.200, -0.150, TOP)` with
  an 18 mm OD x 10 mm pedestal (`STAGE_TOP = TOP + 0.022 m`) and a 4.2 mm diameter, 8 mm tall brass staging peg.
- **Quick-acting knob (`knob`, free, 41 g, friction 0.6):** **Carr Lane CLM-8-QAK** quick-acting four-prong knob,
  M8x1.25, sintered iron. Catalog dimensions: A 38 mm, B dia 17 mm, C 22 mm, D 12.5 mm, F dia 8.5 mm; a UK distributor
  gives the outside diameter as 1-1/2 in. The catalog does not label the letters on its product page, so the model reads
  them as 38 mm across the four prongs, a 17 mm hub, 22 mm overall height, and prongs over the top 9.5 mm above a 12.5 mm
  hub. The weight is 0.09 lb (41 g) per the All Industrial listing. The modelled solid is 5.36 cm^3 (about 40 g at
  7.4 g/cm^3). Inside it are a relieved 15.2 mm bore, two opposing half-thread lugs (top on `-x`, bottom on `+x`, inner
  faces 3.25 mm from the axis, the bottom lug's face 0.2 mm above the knob face) and side guides 9.6 mm apart across
  `y`. The prongs point along `+-x` and `+-y`, with a gold index mark on the `-x` prong. The knob starts upright on the
  staging peg at `(0.200, -0.150, TOP + 0.033 m)`.

## Scripted demo and state reads

The right arm approaches the staging peg and pinches the knob across its `+-x` prong ends, pads closing along `x`
(pad tips 2.5 mm above the prongs' underside). It lifts clear and tilts 13 degrees about `y`, across the pad line, so
the pinch holds the tilt rather than letting the knob swing about the pads. High above the M8 stud tip it reads the
knob's own tilt and centre, trims the hand pitch until the knob reads 13 degrees (within 1 degree) and centres the knob
over the stud. It measures where the knob sits in the hand and places the knob by that from then on. It slides the knob
straight down the 55 mm exposed stud to just above the washer under closed-loop sag tracking, holding the tilt. If the
knob's read bottom is still more than 4 mm above that target, it backs up to the stud tip, re-aligns and slides again
(at most three tries). It then pivots square, which brings the bottom lug down onto the first thread crest, and
trims the hand by the small rotation that brings the knob's read axis within 0.4 degrees of vertical, with its centre
within 0.2 mm of the stud axis (the lug overlaps the crest by only 0.75 mm). Turn strokes follow, each about the
knob's own centre, with the hand re-centring the knob on the stud from its read position as it turns. The first
stroke keeps the squaring grasp: it pushes 2 mm and turns the held knob 135 degrees clockwise, from the 90 degree
grasp to -45 degrees, which covers the thread's quarter-turn run and the 15 degrees past it. The hand then unloads
to the height the thread has drawn the knob down to. If the thread is not yet snug, each later stroke opens to about
70 mm, turns the open hand to +45 degrees over the knob's read centre, closes level on the next prong pair, pushes
1 mm and turns clockwise to -45 degrees. Wrist sag integration is frozen during strokes so the push cannot wind up.
The hand follows the thread's read travel down and stops once the thread's turn angle is 0.06 turn past its seat by
the screw law, after at most eight strokes (three in the offline dry run, which has no simulator readings). Then it
opens the gripper, lifts clear above the stud tip, returns to `READY` and settles for 2.5 s. The left arm holds
`READY` throughout.

**State reads:** the scripted demo reads simulator state. It reads the knob's live pose (centre and attitude) above
the stud tip, at the stud tip, after each slide, while squaring and every step of a turn, to trim the tilt, centre
the knob, detect a slide caught on a crest, square it and keep it centred on the stud. Between strokes it reads the thread crest's state
(travel and turn angle) to follow it down and to stop at snug. It also uses its joint encoders and the gripper opening. The scorer reads the
simulator state of the knob, washer and thread crest (poses, velocities, the screw joint values and collider gaps)
and the gripper finger positions.

## Fidelity note

M8x1.25 thread crests are modelled as discrete 8.0 mm cylindrical collars, and the knob's relieved half-threads as two
opposing internal lugs 6.5 mm apart. Held at 13 degrees, the knob clears the collars by 0.5 mm (at least 0.3 mm while
pivoting square on the washer). Held upright, its lugs land on the chamfered tip and stop 3.0 mm below the stud top.

The thread is a mechkit **kinematic screw**: a joint coupling at the real M8x1.25 lead (1.25 mm, right hand) on the
stud's first thread crest. It is not a contact-resolved helix. Only the squared knob's own bottom lug reaches that
crest. The knob's bore (15.2 mm) and face (15.2 to 17 mm) do not, and neither does a knob without lugs (check.py
measures 0.8 mm clearance). Turning the knob clockwise drives the crest down by friction at the lug. The knob follows
it 0.31 mm per quarter turn until its face meets the free washer, and every turn past that adds hold. A tilted, lifted
or off-axis knob is off the crest. In the real fixture the knob's half-threads advance on a fixed thread; the height
lost per turn is the same. Where the thread catches is fixed in the simulation, a quarter turn before face contact; in
a real trial it is wherever the threads meet. Tightness is scored as turns past face contact (15 degrees) in both. The
torque rise past the seat is a modelled friction, not a clamp force; the kit computes no clamp force. All bodies are
rigid. `released()` requires 30 mm clearance from both grippers, so a held knob never scores above 0.75.

## Run a trial

```bash
INSTR="Tilt and slide the quick acting knob down the stud, square it on the washer, and turn it until it is snug."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ext_ai_robot_vibration_shock_ip_testing_9.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/vib-knob-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ext_ai_robot_vibration_shock_ip_testing_9.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/vib-knob-demo
```

- **Seed and layout:** `-E object_jitter=0` is required so the staging peg, stud axis, washer and crossbar sit at the
  exact nominal coordinates.
- **Runtime per trial:** the scripted demo ends when the thread reads snug (success at 51.0 s in the MuJoCo contact
  replay, three strokes). With all eight strokes it would run about 86 s of simulated time (about 1,720 steps at 20 Hz),
  inside the 2,400-step limit.
  Wall-clock time on one L40S, including Isaac Sim start-up: about 300 s for the demo and about 700 s for a full Dream-1 trial.

## Scoring

The score is taken at the end of the run. Two failures latch to 0 for the rest of the run:

1. **Score 0.0 if the knob fell** off the table (knob center below `TOP - 0.05 m` for 3 steps, 0.15 s).
2. **Score 0.0 if the knob jammed**: upright (tilt <= 5 deg), with its axis within 15 mm of the stud, its bottom in
   the top 8 mm of the stud, collider contact with the stud's tip region (gap <= 0.3 mm), and not moving down
   (vertical speed above -5 mm/s), all for 10 steps (0.5 s). Hovering above or beside the stud, reorienting the knob,
   or tilting it anywhere is not penalized, and a brief upright touch followed by a tilt is allowed.

Otherwise:

3. **0.25** if the knob is on the stud (axis within 6 mm) and its bottom is at least 3 cm down the exposed stud
   (`bottom <= TOP + 0.084 m`).
4. **0.50** if the knob is squared on the washer: its face within 0.6 mm of the washer's measured top, axis within
   4 mm of the stud, tilt <= 3 deg, held for at least 1 s (20 steps).
5. **0.75** if the knob turned its own thread to snug. That needs all of the following:
   - **Engaged:** squared as in 4, axis within 1.5 mm of the stud, and its bottom lug on the first thread crest (lug
     face between 0.2 mm into and 0.4 mm above the crest's measured top).
   - **Drawn down:** the thread was driven clockwise through its quarter-turn run until the knob face met the washer
     (face within 0.25 mm of it).
   - **Past contact:** the thread then turned at least 15 deg (1/24 turn) past face contact.
   - **The knob turned it:** the knob itself turned clockwise about the stud, in the world frame, at least 0.8 times
     the thread turn credited.

   Only motion made while engaged is credited, and unscrewing subtracts. Thread turned while the knob was tilted,
   lifted or off axis does not count, and neither does thread turned while the knob itself did not turn. A knob pressed
   flat on the washer and twisted, with its lugs not on the crest, scores 0.5 (check.py's smooth-knob probe).
6. **1.00** if snug as in 5, both grippers are at least 3 cm clear of the knob, and the knob is still (< 2 mm/s) for the
   last 1.5 s (30 steps).

**Real-trial check (simple tools; same thresholds as the simulator):**

- **Jam (0):** on video, the knob sits upright on the top of the stud with its bottom in the top 8 mm, touching the
  threads, and does not move down for 0.5 s.
- **Squared (0.5):** a 0.6 mm feeler gauge does not enter between the knob face and the washer at four points 90 deg
  apart. A phone inclinometer on the knob's prongs reads under 3 degrees.
- **Snug (0.75 and 1.0):** a 0.25 mm feeler gauge does not enter between the knob face and the washer at four points.
  On the top-camera video, the knob's gold index mark turned at least 15 deg clockwise past the frame where the knob
  stopped descending onto the washer (face contact). Mark both angles on the frames with a protractor.
- **Clamped (0.75 and 1.0):** after the hand lets go, push one prong tangentially with a fingertip (about 2 N, a
  kitchen scale pressed against the prong reads the push). A snug knob does not turn, and the washer under it cannot
  be turned by hand either: it is clamped to the crossbar. A knob that turns, or a washer that spins, is not snug and
  scores at most 0.5. The simulator has no clamp force; it scores the same state by face contact and turn past it.

No fixed turn count is required. The real thread reaches snug wherever it catches. The simulated stand-in asks only
for its quarter-turn run-down plus 15 degrees past face contact.

## Real-world build, parts list, and reset

All required actions stay well below 40 N per finger. Gripping a 41 g knob across its prongs takes about 1 to 2 N of
normal force. The tilted slide has bore clearance (< 2 N friction). Snugging needs a 2 to 5 N press, with under 5 N
tangential pad force at the 38 mm prong ends.

### Bill of materials (low-cost reproducible build)

| Item | Commercial product / source | Qty | Est. price (USD) |
| --- | --- | --- | --- |
| Quick-acting four-prong knob (M8x1.25) | Carr Lane `CLM-8-QAK` (carrlane.com unit price, 2026-11-23) | 1 | $13.63 |
| Washer M8 (8.4 x 23 x 4 mm) | DIN 6340, Hoffmann `375340 M8` or McMaster-Carr DIN 6340 M8 | 1 | $2.00 |
| M8x1.25 threaded rod (zinc-plated steel, 1 m, cut to 102 mm & 70 mm) | Bolt Depot / McMaster-Carr | 1 | $5.50 |
| Aluminum crossbar (6061-T6 flat bar, 25 x 12 x 180 mm, 9 mm bore at y +-75 mm) | McMaster-Carr (or printed via `fabricate.py`) | 1 | $6.00 |
| Aluminum T-slot subplate (160 x 200 x 12 mm) + 35 mm spacer sleeves & M8 hex nut | McMaster-Carr / local shop (or printed base via `fabricate.py`) | 1 set | $28.00 |
| Edge compute enclosure (103 mm wide extruded aluminum, trimmed to 90 mm, ballasted to 450 g) | Hammond `1455L1201` / `1455K1201` series | 1 | $23.05 |
| Staging block + 4.2 mm brass dowel pin (or printed via `fabricate.py`) | Hardware store / PLA print | 1 | $4.00 |
| Feeler gauge set with 0.25 mm and 0.6 mm blades (scoring) | any automotive feeler gauge set | 1 | $5.00 |
| **Total** | | | **~$87.18** |

Carr Lane's order minimum is $75; distributors (All Industrial $13.63 on 2026-11-23, others $12 to $13) ship single
knobs. Run `python3 fabricate.py` to export millimeter-scale ASCII STLs (`knob_replica.stl`, `crossbar.stl`,
`staging_block.stl`, `subplate.stl`) for the printed fixture parts. The printed knob replica follows the model above
(four prongs 38 mm across, 17 mm hub, 22 mm tall, relieved bore, two lugs). It has no thread, so use the real Carr Lane
knob for trials; only its real half-threads clamp.

### Assembly and real-world reset

1. Bolt the subplate to the table at `(0.260, 0.0)` and mount both vertical M8 studs at `y = +0.075` (70 mm tall) and
   `y = -0.075` (102 mm tall) with 35 mm spacer sleeves. Place the ballasted enclosure centered on the subplate, set the
   crossbar over both studs, and snug the `+0.075 m` stud with an M8 hex nut.
2. Drop the DIN 6340 washer over the right stud so it lies flat on the crossbar.
3. Fasten the staging block at `(0.200, -0.150)` with its 10 mm pedestal and 4.2 mm upright brass pin.
4. **Between trials:** turn the knob counter-clockwise about a quarter turn until it is loose, tilt it and slide it up
   off the right M8 stud. Check that the washer lies flat on the crossbar and turns freely. Wipe dust from the stud,
   washer and crossbar face with a dry cloth. Seat the knob upright on the staging peg with its prongs along `x` and
   `y` and the gold index mark facing `-x` (toward the robot bases).

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
