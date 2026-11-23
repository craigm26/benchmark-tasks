# Lay an open book face down on a flatbed scanner, the right page square in the arrow corner

Copying a page of a book on a flatbed scanner or copier is something most people have done at home, school or a
library. A pocket hardcover stands open in a V in front of a flatbed scanner whose lid is raised. The robot must turn
the open book over without letting it shut, lay it face down on the glass and square the right-hand page into the
corner marked by the arrow, where the glass's 1.5 mm recess gives a hand an edge to square against. The hard part is
physical: two leaves on a free spine, held one per hand, opened flat in the air, turned face down without the spine
folding, landed tail first and then registered to within 4 mm of two edges. The nearest board task copies a single
rigid ID card on a printer scanner; this one differs in the physical ability: two free leaves on a spine held open,
one per hand, through an inversion, then landed square.

## The parts (real products, real size)

- **Flatbed scanner:** Canon CanoScan LiDE 400 (part 2996C010), black plastic, 250 x 367 x 42 mm, 1.7 kg, A4/Letter
  platen 216 x 297 mm. In the scene the glass is a blue grey plate 1.5 mm below a black 28 mm high bezel; a white 12 mm
  alignment arrow on the front bezel points at the glass corner x 23.2, y -14.85 cm (front right in the robot's view).
  The lid (250 x 367 x 14 mm, 0.35 kg, white pad inside) is hinged along the back edge and raised 100 degrees, leaning
  back on its stop, so it stays out of the way.
- **Book:** Everyman's Library Pocket Poets hardcover, Baudelaire, *Poems* (ISBN 9781857157017), 164 x 113 x 21 mm
  closed, 228 g, 256 pages. Each half is a red cloth board (164 x 113 x 2.5 mm) and a cream text block (158 x 110 x
  8 mm, 3 mm squares), 114 g. It stands on its tail opened at the middle to a V of 130 degrees, pages toward the robot,
  spine at x 20.0, y -3.6 cm: a new sewn hardcover opened at the middle and set down rests partly open like this, and a
  standing V is also where a parallel gripper can take each half by its head edge (a gripper cannot pinch a leaf of a
  book lying flat).
- **Table:** oak top, nothing else on it. Positions are in cm from the midpoint between the arm bases, x forward, y to
  the robot's left.

## What the scripted demo does and reads

Both arms work together, each on its own half, at 0.07 m/s or less (contacts 0.015 m/s), under the declared
0.12 m/s tool speed limit. (1) Each hand comes down straight over its half's head edge, 12 mm in from the fore-edge,
open 40 mm, and pinches the 10.5 mm leaf; if the encoder gap shows no leaf it reopens and retries 4 mm lower. (2) Both
lift 40 mm. (3) The hands swing apart on arcs about the spine, yawing out 25 degrees each, so the book opens flat in
the air (spine on its 180 degree stop). (4) Both wrists pitch together through 73 degrees about the head-edge line, so
the tail swings out over the scanner and the pages face down, the grip separation fixed so the spread stays open.
(5) The book is carried over the glass, the tail set on it, and the head lowered by pivoting about the tail to 10
degrees from flat. (6) The jaws open 6 mm each side, the head settles, and both hands draw straight back off the head
edge. (7) Closed fingertips push both tail edges toward the robot, driven 18 mm past the point where the head would
meet the bezel lip so the arm's lag under the book's sliding friction cannot leave it short; then the left hand pushes
the left fore-edge toward the right the same way until the right fore-edge stops on the lip. The demo then reads the
book again and repeats, 6 mm deeper, any push whose edge still lies more than 2.5 mm inside the glass edge (one retry
round at most). (8) Both arms swing back to their ready pose, grippers open, and hold still.
The demo reads the book's live pose and spine angle from the simulator after the release and after each push, to plan
how far to push and whether to push again; it never moves or edits the book. Everything else comes from the fixed
layout (object_jitter=0), its joint encoders and gripper opening.

## Fidelity

Approximate. Each half of the book is a rigid board and a rigid text block on one spine hinge, with a 0.02 N m/rad
spring toward its 130 degree set and light damping (assumed, not measured on the book); pages cannot fan, flop or
turn, so the sim book can never lose its spread, and the lost-page rule below only matters in a real trial, where the
preview scan checks it. The glass recess (1.5 mm) and the arrow corner's place are taken from the product's layout,
not measured on a unit. The lid is a damped hinge resting on its open stop and is not used. The sim scores the
last step without opening the grippers, while a real trial opens them before measuring: a book still held up off the
glass is not within 3 mm of it everywhere, so it scores as not face down, and a book whose lowest point is more than
1 cm above the bezel counts as not placed and earns no 0.2 credit either. Contact friction is the simulator's (book
0.6).

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.

## Files

- `flatbed_scan_open_book_face_down.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (jointed parts, the closed-loop demo controller,
  geometry helpers), unchanged.
- `fabricate.py`: writes `start_template.stl`, the printed start template, when run as a script.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Copy the right-hand page: lay the open book face down on the scanner glass, that page square in the arrow corner."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flatbed_scan_open_book_face_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/fsb-dream

# Scripted demo (both arms turn the book over, land it and square it into the arrow corner)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flatbed_scan_open_book_face_down.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/fsb-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the book's start V and the scanner are at fixed spots.
- **Runtime per trial:** the demo script runs about 1,505 steps (75 s simulated), then the 8 s success hold; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 320 s for the demo and about 470 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (score 1.0, the book still and both grippers 10 cm clear of it), otherwise
  `score()` at the end of the run.

## Scoring

The score is the book's final placement. What happens on the way is free unless it damages the book or the scanner or
loses the page: the spread may narrow or shut for a while in hand, the book may be set back on the table and
regripped, and it may be moved quickly while held.

1. **0 for the whole run (DROP, LOST PAGE)** if the book is dropped: its lowest point falls 10 cm or more within 0.2 s
   with neither hand on it (a fall that can dent a board corner or chip the platen glass; a 10 cm fall lasts 0.14 s).
   In a real trial, also 0 if the preview scan does not show the starting spread with page 129 at the arrow (the page
   was lost when the book shut or leaves turned).
2. **FACE:** both page faces within 3 degrees of the glass, and every page corner within 3 mm of the glass top and over
   the scanner. This also means the book is resting, not held up.
3. **1.0:** FACE, and the right page's head edge (both corners) and fore-edge (both corners) are each 0 to 4 mm inside
   the glass edges at the arrow (x 23.2 cm, y -14.85 cm).
4. **0.6:** FACE with only one of those two edges registered. **0.4:** FACE with neither.
5. **0.2:** not FACE, but at the end the book lies still on the scanner (its lowest point within 1 cm of the bezel
   top, moving under 2 cm/s, every page corner over the scanner) with both pages facing down within 30 degrees and the
   book open within 20 degrees of flat.
6. **0** otherwise: the book never turned over, face up, standing, on the table, off the scanner, or still held up in
   the air (lowest point more than 1 cm above the bezel).

The end result must hold for the last 1 s: the lowest of these bands over the final 20 control steps counts, so a
book that slides into or out of the corner at the very end is judged by where it stayed.

A real trial measures a drop from the video (fall height against the scanner's 4.2 cm body, hands visibly off the
book), FACE with a protractor on the boards and a ruler at the page corners, and registration and the page from a
75 dpi preview scan, where the page number and the page's head and fore-edge show against the glass edges. In the sim
DROP also needs every finger pad of both grippers at least 2 cm from the book's bounding box when the fall ends.

## Real-world build and reset

- **Parts list:** Canon CanoScan LiDE 400 (2996C010, about US$80 to 100 at retail); Everyman's Library Pocket Poets,
  Baudelaire *Poems* (ISBN 9781857157017, about US$16); a printed start template (`fabricate.py`, about 40 g of PLA).
  Total about US$120. No power or computer is needed for the trial; the preview scan is taken afterwards.
- **Assembly:** print the template, set the scanner's front-right corner against its L-stop and tape round the
  scanner's feet; open the book at the middle (pages 128 and 129), stand its two tail edges inside the template's V
  rails and mark them with tape; remove the template. Raise the lid to its stop.
- **Reset:** close the book, reopen it at the same pages, stand it on the marks in its V, raise the lid, and wipe the
  glass with a dry cloth if a finger touched it. Nothing is consumed.
- **Forces:** the leaf weighs 1.1 N; the pinch needs well under 40 N per finger, and the squaring pushes move a 228 g
  book on glass (about 1.5 N).
