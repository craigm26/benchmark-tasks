# Temporal temperature on a manikin: scanner flush across the forehead to the hairline, then behind the ear

The robot takes a temporal artery temperature on a full-size manikin head, in the scanner maker's own steps: probe
flat on the centre of the forehead, button held, slide slowly straight across to the hairline (not down the side of
the face), then, still holding the button, touch the probe flat on the neck just behind the ear lobe, release, and put
the scanner back in its cradle. Medical assistants (O*NET 31-9092.00, 0.8M US workers), nursing assistants
(31-1131.00, 1.4M) and licensed practical nurses (29-2061.00, 0.6M) take vital signs at every visit or shift, and a
temporal scanner is one of the usual thermometers. The ability tested is keeping a held tool's face square and in
light contact along a curved surface, from a start point to a stop line, avoiding no-go areas (hair, down the face),
then on a second contact site in the same press. Holding a probe still in an ear canal, or wiping area on a flat
panel, tests neither the path, the tool's angle nor the stop line. This is a contact sweep with a real-size scanner on
a sculpted head with face, ears and hairline, not a hover.

## Parts (real sizes and materials)

Positions are cm from the midpoint between the arm bases, x forward, y left, heights above the table. The manikin
faces the robot, so its left is the robot's right (-y).

- **Manikin bust** (fixed, clamped): vinyl adult head and neck on navy-gowned shoulders. Shoulders 34 x 15 x 4.5 cm
  tall, centred x 36, y 0. Neck 11 cm across at x 33.5. Head 16 cm wide, 19.5 cm deep (x 23 to 42.5, face toward
  the robot), 23.8 cm from chin to crown, chin 6 cm above the table: every horizontal section is a circle of the
  measured radius (25 mm at the chin, 67 at the jaw, 78 at the brows, 80 at mid-forehead, 64 near the crown) swept
  3.5 cm front to back. Nose 2.1 cm proud; ears 6.8 cm tall and about 3 cm wide, 11.8 to 18.6 cm up, back edge at
  x 34.6. Brows, closed lids and lips are painted on. Brown hair above the front hairline (24.5 cm up) and from the
  temples (7.0 cm of arc from the forehead midline) round the back, down to 18.5 cm.
- **Scan line**: 21.5 cm up (15.5 cm above the chin, midway between brows and front hairline), where the forehead is
  16 cm wide. A band of dry-wipe marker 2.5 cm tall is drawn on it from the midline to past the left temple hairline.
- **Spot behind the left ear lobe**: a small dot at x 36.8, y -6.4, 13.5 cm up, on the neck 2.2 cm behind the ear's
  back edge.
- **Temporal artery scanner**: off-white ABS, 15 cm long, 80 g. Handle 12.2 x 4 x 2.8 cm with rounded edges; head
  tapering from 3.2 to 2.5 cm across; grey probe cup 2.5 cm across, 6 mm deep, black lens 4 mm in. Dark grey scan
  button 2 x 1.4 cm on one side face, 4 mm proud, 2 N to press fully; red scan lamp and a dark display on the top face.
  A grip across the handle presses the button.
- **Cradle** (fixed): charcoal ABS base 15 x 6 x 2 cm at x 24, y -18, with two U saddles (3.4 cm inside, so 3 mm a
  side on the handle, 4 mm walls, floors 4.5 cm up) under the handle at x 17.5 to 20 and 27.2 to 29. The scanner
  lies in them display up, probe toward the head (+x), button facing -y, grip centre at x 25, y -18, 6.5 cm up.

## What the demo does and what it reads

The demo reads the scanner's live pose and its button and probe-cup travel from the simulator, and computes the
cup's gap and tilt from the fixed head geometry in the task file (`contact_origin`). The right arm alone works; the
left arm holds its ready pose.

1. Top-down grasp across the handle between the saddles; the closing pad pushes the scan button in (the lamp lights).
   It checks the pads stopped on the handle and the button reads 3 mm or more, and regrips once if not.
2. Lifts 6 cm at 50 mm/s and carries the scanner (peak 0.12 m/s) round the right of the face to 3 cm out from the
   forehead midline on the scan line, the grip leaned 29 degrees about the scanner's own axis for reach.
3. Approaches along the forehead normal until the cup is pressed 2 mm, then slides it 5.6 cm across to the left at
   15 mm/s (3.7 s), turning with the forehead so the probe stays square, and stops with the rim 1.5 mm short of the
   temple hairline, where it rests 3.5 s.
4. Lifts off 3 cm along the normal, goes round the ear on waypoints at least 3 cm from the head, and presses the cup
   flat on the spot behind the ear lobe for 2.5 s, button still held.
5. Carries the scanner back above the cradle, lowers it to 1 mm over the saddles, opens (the button springs out),
   rises 8 cm and parks the arm at its ready pose, gripper open.

Throughout, the hand command is corrected from the scanner's measured pose (arm sag and any shift in the grip), so it
is the scanner, not the hand, that follows the path. The demo takes about 73 s of simulated time (1,453 steps); the
scanner resting on the temple hairline is in the third of six evenly spaced video frames and the touch behind the ear
in the fourth.

## Fidelity

The head is rigid vinyl built from the smooth profile above; the collision facets lie within 0.4 mm of it, and the
scorer uses the smooth surface. The skin's give is modelled as a 4 mm sprung probe cup (400 N/m, 1.6 N fully in): the
cup must actually be pressed onto the skin to count as touching, so a hover scores nothing. The infrared reading is not
simulated: a valid scan is the scored path. The grip holds the button for as long as it holds the scanner, so the
button is let go at the cradle; the real scanner keeps the peak of a scan, so the reading is the same. The scan lamp
is switched on the scorer side; the hair, brows, lids and lips are look-only. The simulator scores the last step
without opening the grippers, while a real trial opens them before measuring: a scanner still held, or anywhere but
resting in its saddles, scores 0. No joint uses a friction hold.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the temperature: scan mid-forehead across to the left hairline, then behind the left ear lobe; cradle the scanner."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=take_a_temporal_temperature_on_a_manikin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/temporal-dream

# Scripted demo (full score): the right arm scans the forehead and behind the ear, then cradles the scanner
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=take_a_temporal_temperature_on_a_manikin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/temporal-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the cradle, the head and the spot are fixed, and the
  scanner must start in its saddles. `-E control_hz=20` is required: the scorer's speed limit uses 0.05 s steps.
- **Runtime per trial:** the demo finishes in about 73 s of simulated time (1,453 steps), and the episode ends about
  9 s after the release, once `success()` has held for 8 s; a Dream-1 trial runs up to 3,600 steps (3 minutes).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 320 s for the demo; the Dream-1 run ended after about 30 s with no score and no recording (see the Dream-1 run result below).

**Dream-1 run result:** no score and no video. Dream-1 refused the instruction: the model's server answered the request with an error before any action. The error was `PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Scanning forehead and ear involves touching or coming forcefully near a person.`, on the last of 3 attempts on 2026-11-16. The scripted demo scores 1.0 and shows the task can be done.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score and the scanner still), otherwise
  `score()` at the end of the run.

## Scoring

Checked every control step (20 Hz); in a real trial, from a 20 fps video and the marker band.

- **On skin:** the cup rim flat on the skin (cup pressed in 0.5 mm or more, rim within 2.5 mm of the skin) with the
  probe within 15 degrees of square to it.
- **A scan** is one continuous button press. Its first on-skin touch must be within 1 cm of the forehead midline on
  the scan line, or the scan is invalid. It is void if, on the forehead, the cup centre goes more than 1.5 cm off the
  scan line (down the face) or more than 6.6 cm of arc from the midline (the rim 9 mm onto the hair).
- **Bins:** ten 5 mm bins from the midline toward the left temple. A bin counts when the cup centre crosses it on
  skin, within 1 cm of the scan line, under 50 mm/s.
- **Best valid scan:** 0.6 x bins/10, and 0 under 5 bins; +0.2 if its farthest on-skin cup centre is 5.1 to 6.1 cm
  along the arc (the rim stops on the hairline, 7.0 cm along); +0.2 if, in the same press, the cup is on skin within
  1.2 cm of the spot behind the ear for 0.5 s in a row.
- **Score 0** unless at the end the scanner rests released and still (1 s) in its saddles: centre within 8 mm of its
  start, within 6 mm in height, turned under 10 degrees, both grippers 3 cm clear and the button out. **Score 0 for
  the whole run** if the scanner ever touches the table.

Possible scores: 0, 0.3 to 0.6 in steps of 0.06, plus 0.2 for each of the stop line and the ear. A forehead sweep
alone, stopped on the hairline, scores 0.8; a full procedure 1.0. A hovering, tilted, fast, off-centre, down-the-face
or onto-the-hair scan scores 0.

**Real-world measurement.** Before trials, press the cup 2 mm onto a test strip of the marker at 15 degrees of tilt;
the height it wipes is the threshold (a cup flat on the vinyl wipes most of its 25 mm rim). A bin counts when the band
inside it is wiped clean over at least that height. The 20 fps video gives the scan lamp (the press), the start
point, the speed across each bin and the 0.5 s touch behind the ear (the dot wiped); a photo shows the scanner in its
saddles.

## Real-world reset

Wipe the forehead and the dot clean and redraw the marker band and dot before each trial. Put the scanner in its
saddles, display up, probe toward the head. The manikin and cradle stay clamped; mark their outlines before the first
trial.

## Files

- `take_a_temporal_temperature_on_a_manikin.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`: multi-part objects, sprung joints, the indicator lamp and the procedural geometry builders.
- `closedloop.py`: the closed-loop demo controller `yam_scripted` runs.
- `belimits.py`: shared geometry helpers.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
