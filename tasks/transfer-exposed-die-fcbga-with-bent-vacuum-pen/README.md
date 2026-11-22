# Transfer an exposed-die FCBGA chip from a waffle pack to a test socket with a bent vacuum pen

Lidless flip-chip packages (a bare 14 x 14 mm silicon die on a 19 x 19 mm substrate, as used on AI accelerator and FPGA
parts) are moved in bring-up and test labs with a handheld bent-probe vacuum pen, not tweezers or grippers: the chip lies
0.8 mm below the waffle-pack rim with no edge to hold, metal touching the die chips it, and a gripper would crush the solder
balls under the substrate. A pen with a bellows button needs a set order: squeeze the button before the cup touches down
(this expels the air), bring the cup down flat and centred on the die, release the button to draw the vacuum, lift the chip
clear of the pocket walls, carry it, lower it to hover 4 mm above the test-socket floor, squeeze again so the vacuum breaks and
the chip drops flat into the nest, then put the pen back on its bench cradle. Here one arm holds the pen pitched 45 degrees
and the other squeezes its side button, which is what a person does with two fingers of one hand. The instruction is "Use the
bent vacuum pen to transfer the chip from the waffle pack into the test socket, then return the pen."

## The parts, at real size

| Part | Real product (maker, number) | Size and material in the scene |
|---|---|---|
| Vacuum pen | Virtual Industries PEN-VAC V8901-A-ESD, with bent probe V3950-B and cup V9038-ESD | Aluminium barrel 12.7 mm OD x 146 mm (12-sided collider, silver), 15 mm nose cone, steel probe 8 mm straight then 18 mm at 45 degrees (3.0 mm collider, the real 18 ga tube is 1.27 mm), black silicone cup 9.53 mm OD x 5 mm, face tilted so it is horizontal when the barrel is pitched 45 degrees. 40 g: base 36 g, button 4 g. Side button 14 x 10 x 4.5 mm, 3.5 mm travel, 714 N/m spring (2.5 N at full travel), 33 mm ahead of the barrel centre. |
| Waffle pack | Entegris H44-999-62C02, 4 in conductive 3 x 3 tray | Charcoal, 101.6 x 101.6 x 6.5 mm, 3.2 mm floor, 3.3 mm walls, nine 25.32 mm square pockets at 30.8 mm pitch (5.48 mm between pockets, 7.34 mm border). |
| Chip | AMD/Xilinx SBG484 lidless FCBGA | 19 x 19 x 1.75 mm emerald substrate plus a 14 x 14 x 0.75 mm silver bare die (2.5 mm in all), 5 g, lying in the centre pocket with the die top 0.8 mm below the rim (3.16 mm clearance per side). Solder balls are not modelled. |
| Test socket | Open-top BGA nest of the Enplas / Ironwood drop-in type (printed from `fabricate.py`) | Ivory, 56 x 56 x 10 mm, 25 x 25 mm floor (3.0 mm per side round the chip), 2 mm vertical wall then a 45 degree funnel to a 33 mm mouth. |
| Pen cradle | Printed bench cradle (`fabricate.py`) | Slate, 110 x 40 x 3 mm base, two 10 mm V saddles 100 mm apart (90 mm clear), barrel axis 41 mm above the mat, rear stop 4 mm behind the barrel end. |
| ESD mat | Any 4 mm static-dissipative bench mat cut to size | Blue-grey, 22 x 36 x 0.4 cm. |

Layout (m from the midpoint between the arm bases; x forward, y left): mat centre (0.270, -0.045); waffle pack and chip
(0.305, +0.045); test socket (0.305, -0.065); pen cradle and pen (0.245, -0.175), barrel axis 4.5 cm above the table top.

## What the scripted demo does and what it reads

The right arm takes the pen out of the cradle from the -y side, with the pads closing across the barrel 22 mm behind its
centre, lifts it, rolls it 45 degrees nose down and carries it over the die so the cup hovers 25 mm above it. The cup is
centred over the die using the pen's live pose. The left arm comes in from +y, straddles the barrel at the button and squeezes
it (the lower pad stays on the barrel's underside, the upper pad presses the button 3 mm), so the button is down before
touchdown. Both arms lower together 26 mm at 1.2 cm/s until the cup lands flat on the die; the left arm opens, the button
rebounds and the vacuum is drawn, and it backs away. The right arm re-centres the chip, lifts 28 mm (clear of the pocket
walls), carries it 110 mm to the nest at under 5 cm/s and lowers it to hover 4 mm above the floor. The left arm squeezes the
button again, the vacuum breaks and the chip drops 4 mm flat into the nest. The right arm lifts the pen clear while the button
is still held (so the cup cannot re-seal), the left arm opens and goes to its ready pose, and the right arm brings the pen back,
unrolls it to level, sets it on the saddles, opens and goes to its ready pose. It then waits 2.5 s with both grippers well clear.
The demo reads the live poses of the pen and the chip and the button's travel from the simulator (to centre the cup and to stop
the squeeze when the button reads 3 mm), plus its joint encoders and gripper openings. It never moves an object or writes a joint.

## Fidelity note

The vacuum is a scorer-side hold, not a fluid model. If the button was squeezed 2.5 mm while the cup was at least 5 mm above the
die, and then returns to within 0.6 mm of rest with the cup flat (6 degrees), within 2.5 mm of the die top and within 2.5 mm of
the die centre, the chip is written onto the cup at its offset before every physics step (240 Hz) and stays there until the button
is squeezed 2.5 mm again. The real V9038-ESD cup pulls about 6 in.Hg and lifts 150 g; the 5 g chip is far inside that. The
bellows is a spring button (2.5 N), the cup and probe are rigid, the probe collider is 3.0 mm (real 1.27 mm), the chip underside
is flat (no balls), and a held chip passes through geometry (a sideways drag below the rim is scored as a violation instead). The
simulator scores the final step without opening the grippers, so a chip still on the pen at the end counts as not placed, a chip
released more than 15 mm above a floor or away from the pack and nest is a violation, and a held part never scores better than a
released one. Pad force on the pen is the gripper's closing stiffness times about 6 mm (about 32 N per finger); the button
needs 2.5 N, so nothing the real action needs is near 40 N.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Use the bent vacuum pen to transfer the chip from the waffle pack into the test socket, then return the pen."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=transfer_exposed_die_fcbga_with_bent_vacuum_pen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/fcbga-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=transfer_exposed_die_fcbga_with_bent_vacuum_pen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/fcbga-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: every part stays where the task file puts it.
- **Runtime per trial:** the demo takes about 97 s of simulated time (about 1,950 steps at 20 Hz) plus the 8 s success hold; a
  Dream-1 trial runs up to 3,000 steps (2.5 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 400 s for the demo and about 800 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score, both grippers 10 cm clear), otherwise `score()`
  at the end of the run.

## Scoring

At the end of the run, in this order:

1. **Score 0 for the whole run** if: a gripper pad touches the chip for 0.1 s (2 steps); the chip falls under -0.55 m/s for 0.1 s;
   it rests on the table or the pack rim (underside under 6 mm above the table top outside every pocket and the nest mouth);
   it is released (button squeezed while sealed) away from a pocket or the nest, or more than 15 mm above the floor under it; it
   is carried sealed below the pocket rim with any part of it outside its pocket; or the pen falls more than 5 cm below the table top.
2. **Suction** forms only when the button goes from 2.5 mm squeezed to within 0.6 mm of rest with the cup at least 5 mm above the
   die when the squeeze began, the cup flat (6 degrees), within 2.5 mm of the die top and within 2.5 mm of the die centre. A
   button released in mid-air, squeezed only after the cup touched, or released with the cup tilted or off-centre draws nothing.
3. **Tiers**, with the chip released (vacuum off), still (under 5 mm/s) and both grippers at least 3 cm clear for the last 2 s:
   **0.25** the chip was lifted by suction at least 10 mm above the pocket rim (underside above TOP + 20.5 mm) and now rests in
   a pocket or in the nest; **0.60** the chip lies flat on the nest floor (centre within 3.5 mm, underside within 2 mm of the floor,
   tilt within 5 degrees); **1.00** also the pen is back on the cradle (centre within 15 mm, barrel within 10 degrees of level),
   still and both grippers 3 cm from it. Anything else is 0.

Doing nothing scores 0. Pushing the chip into the nest scores 0 (no lift by suction). A chip left on the pen scores 0.

## Real-world build and trial

**Parts list (USD, as listed in 2026-11; re-check before ordering):** Virtual Industries PEN-VAC V8901-A-ESD with V3950-B probe and
V9038-ESD cup, 83.60 (distributor price; list 113.20); spare V9038-ESD cup, 2.50; waffle pack: surplus Entegris H44-999-62C02, about
10, or the printed `waffle_pack.stl` in ESD-PETG, about 3; chip: a dead or dummy SBG484 about 10, or the printed `chip_dummy.stl`
(green PETG, glue a 14 x 14 x 0.75 mm silicon offcut or aluminium shim on top, steel shim to 5.0 g), about 2; test-socket nest
`test_socket.stl` (the commercial Enplas / Ironwood nests cost 150 to 400), about 4 of PETG; bench cradle `pen_cradle.stl`, about 2;
ESD mat, about 15. **Total about 112 USD** with the printed route (the pen is the only part over 20). Print at 0.2 mm layers, 20 %
infill; every size is the scene's (`python3 fabricate.py out_dir` writes ASCII STL in mm; nothing is written on import).

**Assembly:** lay the mat flat; put the waffle pack, nest and cradle on it at the layout above (mark their outlines with tape); lay the
pen level in the saddles, button up, nose toward +x, with the rear cap 4 mm from the stop; drop the chip die-up in the centre pocket.
Use a wrist strap and ground the mat.

**Real-trial scoring** (photo and ruler, one overhead video): the squeeze before touchdown is seen as the button down in the frame
before the cup meets the die; lift is seen as the chip clear of the pocket rim by 10 mm on a ruler; the nest result by a ruler and a
straight edge (centre 3.5 mm, flat); 'vacuum off' is the chip lying free in the nest with the pen lifted away from it.

**Reset and cleaning:** put the chip back in its pocket, die up; set the pen on the cradle with its button released; wipe the cup with
isopropyl alcohol and lint-free wipe between trials, and replace it (2.50) if it sticks or marks the die. Nothing is consumed.

## Files

- `transfer_exposed_die_fcbga_with_bent_vacuum_pen.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  the suction hold and the demo).
- `fabricate.py`: print files for the nest, cradle, pack and dummy chip.
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit the task imports.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
