# Set a mixer bowl's ears on its support pins under the beater, then raise it and lock the lift

A steel-grey bowl stands on a dark pad in front of the right arm. At the far end of the table a bowl-lift mixer has a
dark column, a head that overhangs the work area and a white beater hanging from the head. In front of the column a grey
lift frame (two arms, open at the front) carries two orange pins, and a red handle at its left side. To the right of the
column a dark post carries a blue lock bolt. The job: take the
bowl by its green tab, bring it in low under the beater, drop its two ears over the two pins, then raise the frame by the
red handle so the bowl's orange rear lug rises into the 2 cm channel of the grey guide on the column face, and, while one
hand holds the frame up, slide the blue bolt in under the frame's red tab with the other, then let the frame down onto it. The head and
the beater leave no way to reach the pins from above, so the bowl has to come in low from the front. A bowl that sits on
only one pin rides up crooked and its lug stops on the guide mouth, so the lift only finishes if both pins took the ears.
The raised frame rests on the bolt's solid bar: there are no springs and nothing has to snap in. A frame that is let go
without the bolt falls back, so the lift takes two hands. Raising or locking the empty frame, or pushing the bowl along the
table, scores 0.

## Files

- `set_a_mixer_bowl_on_its_pins_and_raise_it.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, closed-loop `demo_script()`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: helper modules the task file imports (multi-part objects with a sliding
  frame and a sliding bolt, geometry checks, closed-loop demo controller). Keep them beside the task file.
- `task.json`: the task's details.

The helper modules are used unchanged. The score does not depend on which arm does which stage.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Put the steel bowl on the mixer, raise it into mixing position and lock the lift."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_mixer_bowl_on_its_pins_and_raise_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ./runs/mixer-bowl-dream

# Scripted demo (full score): the right arm brings the bowl in under the beater and seats the ears on the pins, then the
# left arm raises the frame by its handle and holds it, the right arm slides the bolt in, and the left lets the frame down
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_mixer_bowl_on_its_pins_and_raise_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ./runs/mixer-bowl-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every part exactly where the task file puts it, and the
  pins, the guide and the bolt are measured from fixed positions.
- **Runtime per trial:** a Dream-1 trial runs up to 3,600 steps (3 minutes simulated); the demo takes about 100 s of
  simulated time (about 2,000 steps) and ends 12 s after the frame is let down on the bolt (it must stay up, hands off, the
  whole time). Wall-clock time on one L40S, including Isaac Sim start-up: about 410 s for the demo and about 610 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 12 s (bowl seated on both pins, frame raised and let
  down on the bolt, bolt in, lug in the guide, bowl still), otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened):

| Score | Condition | Check on the mock-up |
|---|---|---|
| 1.0 | bowl seated, frame let down on the bolt (10.0 to 10.65 cm up), lock bolt 2.6 cm or more in, lug in the channel, bowl still, all held for 1 s | ruler at the ears, the handle and the bolt; side photo of the lug between the rails |
| 0.5 | the bowl was seated and still for 1 s at some point | ruler under the ears |
| 0.25 | one ear was on its pin and still for 1 s at some point | ruler under that ear |
| 0 | anything else | |

- **Seated:** both ear holes within 4.5 mm of their pins' axes, both ear undersides within 3 mm of the arm tops, bowl
  level within 3 degrees.
- **Lug in the guide:** the lug's centre within 4 mm of the channel's centre line and between the rails' heights.
- **Bolt in:** the bolt 2.6 cm or more along its 3.6 cm travel, so 2.0 cm or more of its bar is under the 3.0 cm tab.
- **One ear on its pin:** that ear's hole within 4.5 mm of its pin and its underside within 4 mm of the arm top.
- A bowl off a pin cannot rise: its lug stops on the guide mouth, so the score cannot pass 0.5.
- Raising or locking the empty frame scores 0. A raised frame with the bolt out stays at 0.5 even if a hand holds it up:
  it falls when let go. A frame still held above the bolt at its stop (more than 10.65 cm up) is not let down yet.
- **Still:** moving under 1 cm/s.
- Pushing the bolt in with the frame down blocks the tab from rising; the score stays at 0.5 or less. Pushing the bowl along the table cannot score: the ears must be lifted onto the pins.
- A bowl with no support within 1 cm (held up) at the end scores 0, so a held bowl never scores better than a released one.
- The 0.5 and 0.25 stages are kept over the whole run; the 1.0 stage is judged at the end.
- Doing nothing scores 0.

In simulation the pose of the bowl and the positions of the frame and the bolt are read from the objects, so a turned or tilted bowl is
judged by its real pose.

## Real-world reset

Hold the lift handle, raise the frame off the bolt, pull the bolt out, lower the frame to its stops, and put the bowl back on
its pad, tab toward the robot. Nothing is consumed. Use a plywood mock-up of the listed dimensions (a real mixer's lift
locks by its lever and has no bolt); keep hands
out of the bowl while the lift moves.
