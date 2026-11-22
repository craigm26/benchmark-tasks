# Pull a flush shower drain grate with its lifting hook, lay it on the mat, and put the hook back

A walk-in shower has a flush square floor drain: a slotted stainless grate lies level with the tile, in a recess 3 mm
wider on every side, with no rim to take hold of. People lift it with the little stainless hook that comes in the box,
to clear the hair from the strainer underneath: the hook goes down one slot, turns a quarter turn out of sight under the
middle rib, and lifts the grate straight up and level; the grate is laid down, the hook turned back and drawn out. The
robot has to do all of it with a tool it cannot see into the grate with: take the hook from its holder, feed its foot
through a 13 mm slot, turn it under the rib, lift 0.24 kg level, carry it to a mat, set it down flat, free the hook
without dragging the grate, and put the hook away. Bathroom cleaners and householders do this to clear the strainer; plumbing guides show the hook-in-a-slot lift (e.g.
[Mr. Rooter](https://www.mrrooter.com/about/blog/how-to-remove-a-shower-drain-cover/)); no job headcount is claimed.
Related pending tasks: our pop-up stopper hair wand task and the flush plate nest task ("Push a flush plate out of its nest, then take it by the overhang").
This one differs in ability: a quarter turn under a rib and a level lift of a 0.24 kg flush plate, versus drawing hair out under a stopper head, versus shoving a plate out of a nest.

## What is on the table (real size, real materials)

All dimensions are the real parts'; positions are in cm from the midpoint between the arm bases (x forward, y left).

| Part | Size and material | Position |
| --- | --- | --- |
| Floor drain deck | 17.6 x 21.6 cm, 4 cm tile slab, blue-white ceramic tile with dark grout lines | x 16.2 to 33.8, y -23.8 to -2.2 |
| Drain body (Elegante Drain Collection 6 in. Square Slot Pattern, [KD01A113-6](https://www.homedepot.com/p/321473973)) | 304 stainless: 15.2 cm square frame, 14.2 cm square recess 1.5 cm deep, 1 cm ledge, 12.2 cm square dark well, 2 in. round hair strainer (look only) | centred x 25, y -10.6 |
| Grate (from the same kit) | 304 stainless, 13.6 x 13.6 x 1.5 cm, 0.24 kg; 7 mm skirt, five bars along x (middle 12 mm, two of 10, two of 9 mm), two 6 mm cross bars, 18 slots; the slot beside the middle rib on the robot's side is 36 x 13 mm; 11 box colliders; mu 0.6 | flush in the recess, 3 mm play per side |
| Lifting hook (from the same kit) | stainless, 88 mm long: 22 x 7 x 4 mm foot, 6 mm round shaft 74 mm long, T-handle 22 x 30 x 14 mm (orange silicone sleeve added for the camera), 55 g, mu 0.8 | stands foot down in the holder, leaning 8 degrees on the slot's near end |
| Hook holder | charcoal silicone, 6.0 x 4.4 cm, 3.4 cm tall, 30 x 13 mm slot 3 cm deep (4 mm floor), mu 1.0 | x 25, y -21, on the right tile strip |
| Service mat | charcoal silicone, 17.6 x 15.4 x 1 cm, mu 1.0 | x 25, y 6.5 |

## What the scripted demo does and what it reads

The demo reads the live poses of the hook and the grate from the simulator (its pose, to keep the hook's foot over the slot
and to know when the grate is down), and its joint encoders and gripper opening. It never moves an object, writes a
joint or claims a score. The right arm does everything; the left arm holds its ready pose.

1. Takes the hook from its holder with a straight-down pinch across the 30 mm crossbar of the handle (pads open 46 mm, closed
   on it, checked with the encoder), lifts it clear of the holder and stands it upright over the slot.
2. Lowers the foot through the slot beside the middle rib, to 7.5 mm under the ribs, then turns it 90 degrees with wrist
   joint 6 so the foot lies under the middle rib, and slides it 6 mm to the grate's middle (over its centre of mass).
3. Lifts 40 mm: 7.5 mm to take up the slack, then the grate comes up level, 17 mm above the tile.
4. Carries it 17 cm to the mat and lowers it until the grate reports it is down on the mat, then lowers the hook a further
   3 mm so the foot floats between the ribs and the mat, slides it back 6 mm, turns it back 90 degrees and draws it straight
   out of the slot.
5. Stands the hook in its holder (tipped to lean on the near end as it started), opens the gripper, lifts the hand 12 cm and
   swings back to the ready pose, and waits for 9 s with both arms parked clear.

## Fidelity: what the simulation approximates

Rigid parts, no water, soap or hair. The holder and the mat are bench conveniences, and scoring credit comes mainly from the lift
(0.40 for the blind lift alone; the put-away adds only the last 0.25). The hook is stood in a holder (a real one lies loose in a drawer) so a straight-down
pinch can take it: the arm cannot grip a lying hook and stand it up. The grate sits in a 142 mm recess with 3 mm play per
side (the real fit is looser or tighter by a millimetre or two); its ribs are 5 mm thick boxes (16 gauge sheet is about 1.5
mm thick) and the hook's foot is 4 mm thick. Clearances kept at real values: foot 7 mm in a 13 mm slot (3 mm a side), foot top
1.5 mm under the rib undersides before the lift, so the turn and the lift contact match a real 2 mm foot under 1.5 mm ribs; the thick stand-ins
only cost 2.5 mm of rib and 2 mm of foot height. The hook's dimensions are our estimate (the maker's page,
[Home Depot](https://www.homedepot.com/p/321473973), gives none). Contacts are the simulator's rigid contacts with the friction given above. A
grate still held in the air counts as not placed (the simulator scores the last step with the grippers closed, so the scorer
requires the fingers 3 cm clear of the grate and of the hook), and a grate lifted 1 cm clear without the hook under it is a
violation. Measurements a real trial can make: a ruler, a spirit level, a video.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Use the lifting hook to pull the flush stainless shower drain grate out, lay it flat on the mat, and put the hook back in its holder."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=extract_flush_shower_drain_grate_with_lifting_hook.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/drain-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=extract_flush_shower_drain_grate_with_lifting_hook.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/drain-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the grate starts flush in its recess and the hook standing in its holder.
- **Runtime per trial:** the demo takes about 91 s of simulated time (1,818 steps at 20 Hz, including the 9 s wait with both arms parked); a Dream-1 trial runs up to 3,000
  steps (150 s simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 230 s for the demo and about 600 s for a full Dream-1 trial.

## Scoring

`task_score` reports 1.0 when `success()` holds for 8 s, otherwise `score()` at the end of the run. Everything below is judged
once the fingers of both hands are 3 cm clear of the part and everything has been still (under 1 cm/s, 0.5 rad/s) for 2 s.

- **Score 0** (latched): the grate more than 35 degrees from upright for 0.15 s (prying, cocking or flipping it); the grate or
  the hook outside x 10 to 42 cm or |y| 28 cm, or the grate lying on the bare table; the grate lifted 1 cm clear of the tile
  with the hook's foot not under it (pinched or pushed out some other way).
- **1.0** the grate flat on the mat (within 5 degrees of level, its bottom within 3 mm of the mat's top, centre within 25 mm of
  the mat's centre, wholly on the mat) and the hook standing in its holder (foot in the slot within 12 mm of the floor, shaft
  within 40 degrees of upright).
- **0.75** the grate on the mat and the hook at least 5 cm from it (bounding boxes), not in the holder.
- **0.50** the grate on the mat and the hook still on or beside it (not yet clear).
- **0.40** the grate was lifted 1 cm clear of the tile, within 25 degrees of level, with the hook under it for 0.25 s, but is not
  on the mat.
- Otherwise 0. Doing nothing scores 0.

## Parts list, sources, assembly and real-world reset

| Item | Source | Price |
| --- | --- | --- |
| Elegante Drain Collection 6 in. Square Slot Pattern stainless shower drain with hook, KD01A113-6 | listed at The Home Depot | about $33 |
| Deck: 18 x 22 cm plywood or tile offcut 4 cm thick with a 15.2 cm square pocket, cream or blue-white tile adhesive-backed | hardware store | about $8 |
| Service mat: 10 mm silicone or EVA mat, cut to 17.6 x 15.4 cm | any hardware or craft store | about $7 |
| Hook holder printed in PETG (fabricate.py writes holder.stl, and hook.stl, grate.stl, frame.stl, deck_tile.stl and mat.stl) | home printer | about $2 |
| **Total** | | **about $50** |

A cheaper way to the same rig with no kit: any 6 in. square slotted shower drain with a lifting hook of 1.5 to 2 mm stainless
works; the holder is printed. The hook's orange handle sleeve is a slip-on silicone tool-handle grip, only for the
camera. Assembly: pocket the deck, set the drain's frame flush in it with the supplied adhesive or mortar, lay the grate in
its recess, glue the holder to the right strip, stand the hook in the holder, lay the mat beside the deck. Reset between
trials: drop the grate back flush, stand the hook in the holder leaning on its near end, wipe the foot and the grate's
underside dry. Keep it dry: no water, no soap, no hair. All forces are under 3 N (the grate weighs 2.4 N); a pinch of 20 N
across the 30 mm handle is enough, well under the 40 N per finger limit.

## Files

- `extract_flush_shower_drain_grate_with_lifting_hook.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`). `closedloop.py`, `mechkit.py` and `belimits.py` are the shared kit, unchanged.
- `fabricate.py`: writes print-ready STL files of the hook, grate, frame, deck, holder and mat, only when run as a script.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
