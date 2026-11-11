# Swap a dicer's blade grid under its pusher

A small food dicer stands on the table: a frame with two ledges, retaining lips over its rear half and a back stop,
holding a worn red blade grid, with a hinged pusher lying down on it, its nine fingers through the grid's nine
openings. A new blue grid lies on a stand to the left and a drain board lies to the right. The robot must raise the
pusher, slide the red grid out from under the lips and lay it on the drain board, set the blue grid on the ledges and
slide it under the lips to the back stop, then lower the pusher. The fingers pass the grid only when it is home (6 mm
a side in the openings): a grid left 1 cm short stops the pusher 0.2 rad up. A full score needs the blue grid home,
the pusher fully down and the red grid flat on the drain board.

## Files

- `swap_a_dicer_blade_grid_under_its_pusher.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`: multi-part objects and joints (the frame, the hinged pusher, the grids).
- `closedloop.py`: runs `demo_script()` inside the standard `yam_scripted` policy with live object poses.
- `belimits.py`: geometry helpers used by `score()` (outline, tilt, flat on a surface, held in the air).
- `task.json`: the task's details.

No third-party assets: every object is built from boxes in the task file.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Fit the new blue blade grid in the dicer in place of the worn red one."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_a_dicer_blade_grid_under_its_pusher.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/dicer-dream

# Scripted demo (full score): the right arm swings the pusher up by its handle, draws the red grid out by its tab
# and lays it on the drain board; the left arm sets the blue grid on the ledges and slides it home; the right arm
# swings the pusher down through it
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swap_a_dicer_blade_grid_under_its_pusher.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/dicer-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every object exactly where the task file puts it.
  The red grid starts inside the dicer with the pusher's fingers through it, so it must not be shifted.
- **Runtime per trial:** the demo takes about 113 s of simulated time (about 2,250 steps), including the 8 s the end
  state must hold; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 600 s for the demo and about 820 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score, both grids still), otherwise
  `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened), with a ruler and a top-down photo:

- **Home (H):** the blue grid rests on both ledges between the cheeks, its rear face within 3 mm of the back stop,
  tilted at most 3 degrees (5 mm end to end).
- **Down (D):** the pusher is within 0.05 rad of fully down (the middle of its block within 5 mm of its down height,
  8.7 cm above the table). In the top photo the nine fingers stand in the nine openings.
- **Up:** the pusher is 1.5 rad or more from down, leaning back past vertical.
- **Out (O):** the red grid is wholly outside the dicer and flat on the drain board: underside within 4 mm of the
  board's top, tilted at most 5 degrees (8 mm end to end), the middle of its frame over the board.

| Result | Score |
|---|---|
| H, D and O | 1.0 |
| H and O, pusher up | 0.5 |
| O, pusher up, blue grid not home | 0.25 |
| anything else | 0 |

Doing nothing scores 0. So does a pusher resting on the blades or let go part way, a red grid left on the table,
leaning on the board or off the table, and a pusher lowered into an empty frame. The blue grid has to be lifted 4 cm
onto the ledges and the red grid 1 cm onto the board, so pushing either along the table cannot score. The simulator
scores the final step without opening the grippers, so a grid held more than 1 cm above every support counts as not
placed.

## Real-world reset

Nothing is consumed. Raise the pusher, slide the red grid back home against the back stop and lower the pusher
through it. Put the blue grid back on its stand with its tab toward the arms. Use printed grids with blunt blades;
real blade grids are sharp, so handle them only by the front tab.
