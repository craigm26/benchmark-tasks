# Turn a shaft one full turn in V-blocks under a dial indicator

Inspectors, testers and sorters (O*NET 51-9061, about 600,000 US workers; "measure dimensions of products to verify
conformance to specifications, using measuring instruments") and machinists (51-4041) check a turned shaft's runout by
laying it in two V-blocks, resting a dial indicator on the journal and turning the shaft one full turn by hand. The
turn is the hard part: the shaft is loose, so it has to be turned in place by repeated regrasps while it stays down in
both Vs and against the end stop. A hand that swings off the shaft's axis carries the shaft up a 45 degree V flank; a
lift of h at both Vs moves the reading by h, and an axial push while it turns walks it off the stop. The scorer measures
the seat directly (0.10 mm at each V) and records the reading an indicator in max/min hold would show; full credit needs
that reading to match the journal's true 0.50 mm. Not tested: reading the dial, the pass or fail call, or setting up
the indicator (it starts resting on the journal, as on a set-up inspection bench).

## Parts (real sizes and materials)

| Part | Size | Material, mass | Where (cm from the midpoint between the arm bases; height above the table) |
| --- | --- | --- | --- |
| Bench stand | 260 x 260 x 54 mm | steel, fixed | centre x 28, y 5.5 |
| Surface plate | 300 x 300 x 50 mm | black granite, fixed | x 13 to 43, y -9.5 to 20.5, top at 10.4 |
| V-blocks (2) | 50 wide, 50 tall, 40 long; 90 degree V, 36 wide, 18 deep | hardened steel, fixed, friction 0.2 | x 28, y -5 and y 4, tops at 15.4 |
| Shaft | 20 mm diameter, 200 mm long | ground steel, 0.54 kg, friction 0.2, loose | along y from -12 to 8, axis at x 28, height 15.0 |
| Journal (part of the shaft) | 24 mm diameter, 40 mm long, axis 0.25 mm off the shaft's (0.50 mm TIR) | steel | y -2.5 to 1.5, 5 mm clear of each block |
| Turn marks | two red stripes 8 x 18 mm, 180 degrees apart, and a red bar on the near end face | paint | y 6.1 to 7.9 |
| End stop | 30 x 20 x 70 mm | steel, fixed | x 28, y 8.05 to 10.05, 0.5 mm from the shaft's far end |
| Dial indicator | 58 mm dial, 25 mm thick, on a 12 mm arm and 16 mm post; magnetic base 50 x 60 x 55 mm | steel, fixed | dial centre x 28, y -0.5, height 24.8; base x 38.5, y -0.5 |
| Plunger | 8 mm flat contact point on a 5 mm rod, 10 mm travel, gauge force 0.65 N | steel, 66 g, vertical slide | on top of the journal at x 28, y -0.5 |

The shaft rests in the Vs by its own weight (5.3 N): its axis sits 3.9 mm below the block tops. The round parts are
round (native cylinders), the V-blocks are extruded 90 degree Vs, and the dial is a cylinder with a drawn face.

## The scripted demo and what it reads

Right arm only; the left arm stays at its ready pose. The hand comes down over the 50 mm of shaft that overhangs the
near V-block, pads closing across the shaft. One open pass over the stroke learns the command trim that keeps the
encoder-measured tool point on the shaft's axis. Then five strokes: close slowly to a 14 mm command (the pads stop on
the 20 mm shaft, confirmed from the gripper encoder), swing the hand 82 degrees round the shaft's axis in 9 s while the
encoder-measured tool point is held on a circle about the axis and the hand leans 1.5 mm toward the end stop, open to
40 mm, swing back in 5 s. It stops once its own count passes 364 degrees, opens, draws back 12 cm along the hand's axis
and parks at the ready pose. About 95 s (1,900 steps) of the 3,600 step limit.

The demo reads the live pose of the shaft (to stay on its axis, to see the end gap and to count its own turn) and the
plunger joint (to confirm it is on the journal) from the simulator; it never moves or edits a body or joint.

Why the turn is the hard part, from the contact replays made while building the demo: a hand that swung about its own
tool point instead of the shaft's axis lifted the shaft 1.2 to 3.8 mm at a V-block (the 0.5 mm latch, score 0), and a
hand whose fingers brushed the near V-block walked the shaft 4.5 mm off the stop and counted 4 of about 345 degrees
turned. With the axis servo the peak lift was 0.30 mm, the counted turn 370 degrees and the recorded reading 0.56 mm
against the true 0.50. One hand is enough because the end stop takes the axial push; an inspector's second hand does
the same job as the 1.5 mm lean.

## Fidelity

Everything is full size with steel masses. The plunger is a real vertical slide that rides the journal; its 0.65 N
gauge force comes from its own 66 g weight instead of a 15 g plunger on a spring (so nothing starts overlapping), with
2 N s/m of damping and no hold. The dial's face, pointer and ticks are drawn and not geared: the reading is the plunger's
travel, which the scorer records as max minus min. Contacts are rigid, steel on steel friction 0.2. The indicator is
preset; no dial reading or pass or fail decision is tested. The simulator scores the last step without opening the
grippers, so a shaft still gripped, lifted or held off its seat scores 0, as it would once released.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Turn the shaft one full turn in its V-blocks under the dial indicator, keeping it down in both Vs and against the end stop, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=check_shaft_runout_in_v_blocks.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/runout-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=check_shaft_runout_in_v_blocks.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/runout-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the seat is measured from the shaft's own rest height.
- **Runtime per trial:** the demo takes about 95 s of simulated time (1,900 steps); a Dream-1 trial runs up to 3,600
  steps (3 minutes simulated).

Wall-clock time on one L40S, including Isaac Sim start-up: about 440 s for the demo and about 630 s for a full Dream-1 trial.

- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s, otherwise `score()` at the end of the run.

## Scoring

Measured every control step from the shaft's pose and the plunger's position.

- **Rest height:** the shaft axis height at each V-block, averaged over the first second (nothing can reach it sooner).
- **Counted turn:** net rotation about the shaft axis, either way, over steps where the axis is no more than 0.10 mm
  above rest at both V-blocks, the end gap is 2 mm or less and the plunger is on the journal (0.5 mm off both stops).
  Turning done lifted, off the stop or in the air does not count; back and forth counts net only.
- **Reading:** the plunger's highest minus lowest position over the whole run, regrasps included.
- **Score 0, latched for the run:** the axis 0.5 mm or more above rest at either V-block; the end gap over 5 mm; a
  finger point within 10 mm of the plunger or the dial.
- **Score 0 unless released:** at the end both grippers are 3 cm clear of the shaft and it has lain still for 1 s
  (0.05 mm, 0.5 degrees), seated in both Vs with the end gap 2 mm or less.
- **1.0:** counted turn 360 degrees or more and reading 0.35 to 0.65 mm (true 0.50). **0.5:** 180 degrees or more.
  **0.25:** 90 degrees or more. **0** otherwise. Doing nothing scores 0.

In a real trial: a 0.01 mm indicator with max/min hold on the journal gives the reading; two more on the plain shaft
over each V-block (on stands behind the plate, clear of the arm, which works only at the near overhang) give the seat
and the 0.5 mm latch; a camera on the end mark against a fixed mark gives the unwrapped turn; a feeler gauge gives the
end gap.

## Real-world reset

Seat the shaft in both Vs with the end-face bar pointing up and its far end 0.5 mm from the stop (feeler gauge). Check
the plunger rides the journal, then clear the indicators' max/min hold. Verify the journal's 0.50 mm TIR between
centres before the first trial.

## Files

- `check_shaft_runout_in_v_blocks.py`: scene, `score()`, `success()`, `demo_script()`.
- `mechkit.py`, `closedloop.py`, `belimits.py`: geometry and joints, the closed-loop demo controller, scoring helpers.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
