# Drift a sprung splice plate into line, bolt it, then swap the drift for the last bolt

A blue splice plate lies on a grey flange, held by a spring 2.9 cm out of line, so only a thin strip of each flange hole
shows through it and a bolt cannot start. The robot has to use the orange drift pin: its taper goes into the strip of
hole A, and as it goes down it cams the plate over until the holes line up. With the drift standing in hole A, a bolt goes
into hole B. Only then is the drift pulled, which lets the spring pull the plate back until it rests on the first bolt;
the drift goes back in its holder and the second bolt goes into hole A. Pull the drift before the first bolt and the plate
springs back, so neither bolt can start. This is the fit-up job of lining up connection holes with a drift pin, bolting
the free hole, then bolting the hole the drift held. Bolt tightening is not tested.

## Files

- `drift_the_splice_into_line_then_bolt_it.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, and the scripted demo `demo_script`).
- `task.json`: the task's details.
- `mechkit.py`: a small helper the task file imports for the sprung plate (a slide) and the multi-part drift, bolts and
  collars. It adds moving-part support to `yam_arena` and changes nothing else.
- `belimits.py`: a small helper the task file imports for tilt.
- `closedloop.py`: a small helper that lets the scripted demo read object positions and the arms' measured positions
  while it runs, so it can steer the parts into the holes. It changes nothing else.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Bolt the blue splice plate to the grey flange through both holes. Use the drift pin, then stand it back in its holder."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drift_the_splice_into_line_then_bolt_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/dsl-dream

# Scripted demo (full score): the right arm drops the drift into hole A and holds it there; the left arm bolts hole B;
# the right arm pulls the drift, stands it in its holder, then bolts hole A; both arms clear
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drift_the_splice_into_line_then_bolt_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1800 --store-frames --log-dir ~/yam-eval/dsl-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the strips of the holes that show are 4 to 10 mm wide and the
  drift's tip is 3 mm wide, so the layout is measured from where the task file puts everything.
- **Runtime per trial:** the demo finishes in about 70 s of simulated time (1,400 steps) and ends 8 s after full score
  holds; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 340 s for the demo and about 550 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (both holes bolted, the drift in its holder, all still),
  otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened). J is how far the plate has slid from its rest position; the spring sets
J = 0 when nothing holds the plate.

- **Drift in hole A:** the drift's tip is below the plate's underside and within 2 cm of the hole A axis, and the drift is
  tilted at most 15 degrees.
- **Bolt home:** the shank is through the flange (tip 5 mm or more below the flange's underside), the head's underside is
  within 4 mm of the plate top, the bolt is tilted at most 5 degrees, and its axis is within 1.2 cm of a hole axis.

| Score | Outcome |
|---|---|
| 1.0 | both holes have a bolt home, the first put there while the drift was in a hole, and the drift stands in its holder (head's underside within 1 cm of the rim, tilted at most 10 degrees, centred within 1.5 cm); every part still |
| 0.75 | both holes bolted in that order, drift not in its holder |
| 0.5 | one bolt home, put there while the drift was in a hole |
| 0.25 | at some time the drift stood in hole A with J of 1.4 cm or more for 1 s |
| 0 | otherwise; a bolt home with no drift line made; any part off the table |

The earlier steps are remembered over the run, so letting the plate spring back after the drift is pulled does not erase
them. Doing nothing scores 0: at rest the strips (1.0 cm of hole A, 0.4 cm of hole B) are narrower than a bolt tip
(1.3 cm), so a bolt alone cannot start. With the drift in hole A the strip of hole B is about 2.2 cm, enough for a 1.6 cm shank;
once that bolt holds the plate, the strip of hole A is about 2.1 cm. Every part has to be lifted out of its holder
to reach a hole, so pushing parts along the table scores 0. In simulation the grippers are not opened before the last
step, so a bolt whose head is more than 4 mm above the plate, or a drift whose head is more than 1 cm above its rim, counts
as not placed.

The arms' path is not scored by the simulation. In a real trial keep fingers off the plate edges and out of the holes
while the plate is under spring load; handle the parts by their heads and tabs.

## Real-world reset

Let the spring push the plate back to rest. Hang the drift and both bolts by their heads in their holders (collars
6.5 cm tall with a 3.2 cm opening). Nothing is consumed or bent. The real build is two steel bars for the flange and
plate, a die spring, a bought drift pin and two bolts.
