# Vacuum spilled lentils from under a kitchen toe kick with a swivel-head brush tool

Dried lentils are spilled on a kitchen floor, some in the open and some under a base cabinet's toe kick, the 4 in
high, 3 in deep recess at its foot, where the top camera cannot see them. The robot takes a low-profile vacuum brush
tool by its hose cuff and must run the head over every lentil, then set the tool down. Vacuuming under kitchen
cabinets is a chore most people do. The hard part is physical: the brush head swivels freely through 360 deg on a
central pivot, so the wand cannot set its heading. Pushed straight at the kick board, a turned head wedges on its
corner and leaves the back corner dirty. Drawn along the board with light pressure, it turns until an edge lies flat
on the board, and it must then be run along the board flat, without tipping onto its back bristle row, all through
a 42 cm wand.

## The parts (real sizes and materials)

- **Brush tool:** Milwaukee 49-90-2027 AIR-TIP Low-Profile Pivoting Brush Tool. The maker lists 19.49 x 3.19 x
  2.52 in (495 mm long, 81 mm head, 64 mm cuff), 0.7 lb (0.32 kg), a 360 deg flat swivelling brush head, a
  button-released pivot, and a fit to 1-1/4, 1-7/8 and 2-1/2 in hoses (milwaukeetool.com, product 49-90-2027).
  Red plastic. In the scene the pivot is locked straight (the position for low reach) and the swivel is the only
  joint. Head 81 x 55 x 24 mm with a 4 mm black bristle skirt and a 66 x 44 mm mouth under it, on a flat duct
  50 x 22 mm; stepped cuff, 64 mm outside, bored for the three hose sizes. The head starts turned 70 deg on its
  swivel from square to the wand.
- **Cabinet:** the bottom 18 cm of a 9 in (229 mm) white base cabinet with the standard toe kick: recess 101.6 mm
  (4 in) high and 76 mm (3 in) deep, open at both ends, matte black 19 mm (3/4 in) plywood kick board.
- **Floor:** slate grey vinyl, 6 mm, 32 x 61 cm.
- **Debris:** ten brown dried lentils, about 5 mm across and 2.5 mm thick: three on open floor 1 cm in front of the
  door, three under the overhang 3.5 cm in, four in the back corner 11 mm from the kick board (the last 8 mm from
  the cabinet's far end).

## The scripted demo and what it reads

The demo reads the live pose of the brush head and its swivel angle from the simulator, plus its joint encoders and
gripper opening; the lentil positions are the fixed layout (object_jitter=0). The left arm stays at its ready pose.
The right arm takes the cuff from above with the pads across it and swings the wand to point straight at the
cabinet. It then pushes the head lightly into the kick board and strokes it 6.5 cm toward the cabinet's near end.
The board's friction on the head's leading corner turns it until its long edge lies flat on the board; the demo
checks the heading and retries up to three times. Then it runs the square head along the board to the far end
through the back corner. Throughout, the head's command leads the live head by 3 mm into the board (about 0.7 N)
and eases off if the head starts to tip. Last, it draws the head out and back along the open floor, leaves it
there, opens, rises 12 cm and parks. The wand turns a few degrees in the pinch under the board's drag, so the
grip carries a slowly integrated offset that brings the head itself, not the hand, to each stroke's end.

## Fidelity

Approximate. Suction is a rule: a lentil is taken when the flat mouth passes over it. Lentils are look-only; 5 mm
grains are too small to simulate under a brush. The bristles are a rigid 4 mm skirt with friction 0.4 (the floor and
kick board are 0.4 too), and the swivel has 0.002 N m of friction; the score does not depend on that value. The
hose and the suction's downforce are omitted (the maker publishes no figure), so the drag is the tool's own weight
on its skirt, about 1 N. The simulator scores the last step without opening the grippers, so a tool still held, or
more than 1 cm off the floor, cannot score.

## Files

- `vacuum_toe_kick_swivel_brush.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (procedural geometry and joints, the closed-loop demo
  runner, geometry helpers), unchanged.
- `fabricate.py`: writes STL files of the custom parts of a real trial (below).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Use the brush tool to vacuum up every spilled lentil, including those under the cabinet's toe kick, then set it down."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=vacuum_toe_kick_swivel_brush.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/vtk-dream

# Scripted demo (full score): the right arm steers the tool, the left arm stays at its ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=vacuum_toe_kick_swivel_brush.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/vtk-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the lentils sit at fixed marks and the tool at its start pose.
- **Runtime per trial:** the demo takes about 55 s of simulated time (about 1,100 steps, including the 8 s that
  `success()` must hold); a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 310 s for the demo and about 980 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score with both grippers 10 cm clear of
  the tool), otherwise `score()` at the end of the run.

## Scoring

A lentil counts once when its centre stays at least 3 mm inside the head's 66 x 44 mm mouth for 3 control steps
(0.15 s), while the head is flat (normal within 6 deg of vertical, bristle tips within 3 mm of the floor) and both
grippers are at least 3 cm from the head (so the head is driven through the wand, not carried in the hand).

1. **Score 0** if the head's centre leaves the floor panel at any step, or unless, over the last 2 s, the tool lies
   with its lowest point within 1 cm of the floor, moves under 5 mm/s and has both grippers at least 3 cm clear.
2. **Otherwise, of the 10 lentils** (4 corner, 3 under the overhang, 3 open floor): 1.0 all 10; 0.75 at least 9 with
   at least 3 corner; 0.5 at least 7 with at least 2 corner; 0.25 at least 4 with at least 2 under the cabinet
   (overhang or corner); otherwise 0.

Doing nothing scores 0. Sweeping only the open floor takes the three open-floor lentils and scores 0. A head that is
never squared on the board takes at most the open-floor and overhang lentils (0.25): pressed on the board while
turned 20 to 70 deg off square, its mouth stops short of the corner row. Lifting the cuff tips the head onto its back bristle
row and nothing is taken while it is tipped. A collected lentil disappears from the floor.

**Real trial:** at the end, open the grippers, tip out the vacuum's canister onto a tray and count the lentils, and
take a front photo under the toe kick to confirm which are left. The head must lie on the floor, not in the hand.

## Real-world build

All the force a real trial needs is the drag of the head on the floor and a light push into the board, about 1 N,
far below 40 N per finger.

| Item | Source | Price |
|---|---|---|
| Milwaukee 49-90-2027 AIR-TIP Low-Profile Pivoting Brush Tool | Do it Best 382163 | $29.99 |
| Milwaukee M18 2-Gallon Wet/Dry Vacuum 0880-20 (tool only, 1-1/4 in hose; uses an M18 battery) | Do it Best 341756 | $164.99 |
| 3/4 in x 2 x 4 ft PureBond maple plywood project panel (deck, door skirt, spacers) | Home Depot 311925836 | $32.94 |
| 1x4x8 premium whitewood board (kick board, cut to 229 mm) | Home Depot 100023465 | $8.98 |
| FloorPops Remy grey peel-and-stick tile FP2953, 10 sq ft | Home Depot 318609336 | $11.73 |
| Great Value lentils, 1 lb | Walmart 545884744 | $1.92 |
| White and matte black paint, wood screws, 3D-printed lentil template | shop stock | about $10 |
| **Total** | | **about $260** |

Alternative to the plywood fixture: a stock 9 in base cabinet, Hampton Bay KBF09-UF (9 x 24 x 34.5 in, Home Depot
302970284, $87.98). Its toe kick is 3 in deep and 4-1/2 in high; the extra half inch of height does not change the
task, since the head is 24 mm tall.

**Custom parts:** `python3 fabricate.py [out_dir]` writes ASCII STL files (mm) from the scene's dimensions: the kick
board (229 x 19 x 101.6 mm), the deck over the recess (229 x 102 x 19 mm), two spacer blocks (19 x 26 x 101.6 mm),
the door skirt (229 x 19 x 78.4 mm) and the lentil template (a 3 mm plate, 260 x 120 mm, with ten 8 mm holes at the
lentil centres). Importing it writes nothing.

**Assembly:** lay the vinyl tiles on a flat board as the floor. Screw the two spacer blocks behind the ends of the
kick board, stand it on the floor, and screw the deck on top so it overhangs the board's face by 76 mm. Stand the
door skirt on the deck flush with its front. Paint the kick board matte black and the deck and skirt white. Clamp the
fixture to the bench at the scene's position. Fit the brush tool to the vacuum's 1-1/4 in hose; the hose hangs off
the far side of the table so it does not lie on the work.

## Real-world reset

Tip the canister out onto a tray and count. Butt the printed template against the kick board's face and the
cabinet's near end, drop one lentil in each hole, and lift the template straight up. Lay the tool on its taped start
outline with the head turned 70 deg from square to the wand. Lentils are dry and do not mark the floor; replace any
that crack.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
