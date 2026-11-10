# Centre a square block in a four-jaw chuck by bringing in each pair of opposite jaws

A yellow 4 cm square block stands on the table in front of the left arm. A face-up four-jaw chuck stands between the
arms, its four jaws (each with an upright grip tab on its outer end) open 5.5 cm from the axis. The robot has to lift the block 5 cm onto the chuck face, then bring each
jaw in on its own so that both jaws of each opposite pair grip the block and it ends within 1.5 mm of the chuck axis on
both axes. Setters do this by hand on independent four-jaw chucks. A jaw closed alone pushes the block off centre, so each
pair has to be worked against the other. Jaw screw torque, dial indicators, finer than 1.5 mm and spindle rotation are not
tested. A fixed tool post with a black cutting insert stands behind the +x jaw.

## Files

- `centre_a_block_in_a_four_jaw_chuck.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  and the scripted demo `demo_script`).
- `task.json`: the task's details.
- `mechkit.py`: a small helper the task file imports for the chuck (a fixed body with four sliding jaws), the block and the
  tool post. It adds moving-part support to `yam_arena` and changes nothing else.
- `belimits.py`: a small helper the task file imports for the block's outline and tilt.
- `closedloop.py`: a small helper that lets the scripted demo read object positions and the jaws' travel while it runs, so
  it can carry each jaw to its stop position. It changes nothing else.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Centre the yellow block in the four-jaw chuck."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=centre_a_block_in_a_four_jaw_chuck.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/cab-dream

# Scripted demo (full score): the left arm sets the block on the chuck face 1 cm off the axis; then the right arm brings in
# the -x jaw, the left arm the +x jaw, the right arm the -y jaw and the left arm the +y jaw, which pushes the block true;
# both arms park clear
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=centre_a_block_in_a_four_jaw_chuck.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/cab-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the score tolerances are 1.5 to 2 mm, measured from where the task
  file puts the chuck.
- **Runtime per trial:** the demo takes about 85 s of simulated time and ends 8 s after full score holds; a Dream-1
  trial runs up to 3,600 steps (3 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 380 s for the demo and about 600 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (block seated, both pairs gripping, centred, still),
  otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened). Each jaw's inner face is read from its travel; the block's sides from its outline.

- **On the face:** the block is upright (tilt at most 2 degrees) with its bottom within 2 mm of the chuck face.
- **A pair grips:** each of its two jaws has its inner face within 1.5 mm of the block side facing it, and the block is
  turned at most 3 degrees off square to the jaws.
- **Centred on an axis:** the block's centre is within 1.5 mm of the chuck axis along that pair's direction.

| Score | Outcome |
|---|---|
| 1.0 | on the face, both pairs grip, centred on both axes, still for 1 s |
| 0.5 | on the face, one pair grips and the block is centred on that pair's axis |
| 0.25 | on the face, centre within 2 cm of the axis and between all four jaws |
| 0 | otherwise: block on the table, on a jaw, tipped, held up or off the table |

Doing nothing scores 0 (the block stands on the table). The block must be lifted 5 cm onto the face, so pushing it never
scores. The jaws can only reach 1.9 cm from the axis, so a 4 cm block can be gripped by a pair only when it is centred on
that axis to about 1 mm. In simulation the grippers are not opened before the last step, so a block whose bottom is more
than 2 mm above the face counts as not placed.

The arms' path is not scored by the simulation. In a real trial keep fingers off the cutting insert while working the +x
jaw; the insert stands well behind the +x jaw, so a hand working that jaw keeps clear of it.

## Real-world reset

Slide all four jaws back to their open stops (each outer end on its white rim mark) and stand the block at its start spot.
Nothing is consumed or bent. The real build is a 1:3 printed four-jaw chuck face up on a riser, with the jaws sliding on
plain rails without screws, and a block of steel bar stock.
