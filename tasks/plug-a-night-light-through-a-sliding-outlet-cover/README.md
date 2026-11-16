# Plug a night light in through a sliding outlet cover

A universal self-closing outlet cover is the usual childproofing plate in homes with toddlers: a sprung door hides
the receptacle slots, and the plug itself has to open it. Following the cover's own instructions, the robot puts the
night light's blade tips 4 mm into the door's two slots, slides the door 16 mm across against its spring (up to
3.6 N), keeps it on its stop, and only then pushes the blades into the receptacle and lets go. A straight push stops on
the door face, and blades pushed into the shut door's slots stop on solid backing: both score 0. If the plug eases
back or turns while the door is being carried, a tip leaves its slot and the door snaps shut. Plug tasks on the board
are straight mates into open ports; here the held part is first used as the tool that opens the cover in its way.

## Parts (all full size, rigid)

- **Night light:** plug-in type, 45 g. White plastic housing 45 mm wide x 65 mm tall x 28 mm deep with 4 mm rounded
  corners; amber lens 41 x 60 x 2 mm on the front (the lens glows once both blades are in the receptacle, look only);
  grey photocell window. Two nickel blades (NEMA 1-15) on the back: 15 mm long x 1.5 mm thick x 6.3 mm tall, 12.7 mm
  apart, centred 47 mm above the housing's bottom. It stands on its bottom on the mat, blades toward the wall (+x),
  lens toward the robot, housing centre at x 0.240 m, y -0.055 m.
- **Mat:** black rubber mat 90 x 75 x 3 mm, lying flat on the table and fixed, centred under the light's housing
  (x 0.240 m, y -0.055 m). It only sets the white light off against the table in the top camera.
- **Cover plate:** white ABS universal sliding cover, 70 x 115 x 8 mm, rounded corners, front face at x 0.337 m,
  centred y 0, 81 mm above the table. Each outlet sits in a 60 x 30 x 5 mm recess with a dark grey rim; a screw head
  between them. The top recess holds the sliding door.
- **Door:** light grey ABS, 42 x 24 x 4 mm (front face x 0.338, back 0.342), 6 g, friction 0.4, sliding along y on a
  spring of 225 N/m: 0 N shut, 3.6 N at its open stop, 16 mm of travel, damping 2 N s/m, no friction hold. Two slots
  5.5 x 10.3 mm (2 mm round each blade) on 12.7 mm centres. Shut, the slots sit 18 mm left (+y) of the receptacle's
  centre over solid dark backing, and the door's solid face covers both receptacle slots (the door edge overlaps the
  right slot by 2.9 mm, the solid between its slots overlaps the left slot by 0.3 mm). At its open stop each door slot
  is 2 mm left of its receptacle slot, so a blade bearing on its slot's right-hand wall is centred on the receptacle.
- **Receptacle (top outlet):** two blind slots 4.5 mm (y) x 9.3 mm (z), 1.5 mm round each blade, with a 1.5 mm
  lead-in, at y -6.35 and +6.35 mm, 100 mm above the table; the backing face is at x 0.342 and the slots run to x 0.362.
  Seated, the light's back face is on the plate and the tips are at x 0.352 (d = 15 mm). The lower outlet is look only.
- **Wall:** pale blue painted board 240 wide x 200 tall x 22 mm, face at x 0.345, on a 100 x 240 x 15 mm foot behind
  it. In a real trial: a stock universal self-closing cover on an unwired duplex receptacle in a single-gang box set in
  a drywall offcut clamped upright, and a stock plug-in night light.

Measured in the code with `mechkit.gap` (see the offline checks): a straight push with the door shut stops at
d = 1.0 mm; blades in the shut door's slots stop on the backing at d = 5.0 mm; no plug position with the door shut gets
both tips past the wall face; at the open stop the blades clear the receptacle slots by 1.5 mm all the way in.

## The scripted demo

Right arm only; the left arm stays at its ready pose. About 50 s of motion, then it waits parked.

1. Grip: yaw 0, pitched 25 degrees so the wrist leans back from the wall; the pads close across the housing's two
   sides, 24 mm in front of its back face, after coming down the tilted approach axis at 20 mm/s.
2. Lift the blade line to the slot line and carry the light (0.06 m/s) until the tips are 5 mm short of the door, on
   the door's slots.
3. Press in at 5 mm/s until the tips stop on the backing, then hold them 3.5 mm into the 4 mm door. (Tips that stop
   on the door face mean a miss: back out 10 mm and try once more.)
4. Drag the door across at 4 mm/s, leading the tips 1:1 and correcting the arm each step from the light's read pose,
   until the door reads its open stop. If the light moves 6 mm without the door following, or the door slips back
   2 mm, back out and retry.
5. Push the blades in at 5 mm/s until the back face meets the plate, holding the tips on the receptacle openings'
   centre line (where a blade bearing on its door slot's wall is centred, so the door stays on its stop with no extra
   side load) and the blades square to the wall. The push is closed loop on the read tip position: near the backing it
   waits while the tips are more than 1 mm off line, and it keeps adding force while blade friction slows the tips. A
   stall (under 0.2 mm in 3 s) ends a try; the demo then reads the depth and, if the plug is not home, backs the tips
   out to 2.5 mm into the backing (still through the door, so it stays open), re-centres and pushes again, up to three
   tries.
6. Ease off the sideways and inward preload until the tips start to follow (0.3 mm), then open to 70 mm slowly (2 s)
   while holding the measured tool point still along x, so any push the arm still stores bleeds off as the pads slip
   instead of throwing the fingers at the plate. Back straight out 60 mm, move up and away from the wall and park at
   the ready pose. The door spring then sets the blades 1.5 mm across onto the receptacle slot walls.

The demo reads the live pose of the night light and the door's joint value from the simulator (to aim, to servo the
carry and the push, to confirm the door is at its stop and to confirm the seat), plus its joint encoders and gripper opening. It never
moves or edits a body or joint. Every tool-point command stays under 0.10 m/s.

## Fidelity

Approximate. The receptacle contacts are rigid blind slots: the plug is kept in by the door spring's side load and
friction, not by contact springs, and there is no electricity. The door slots (2 mm a side) and the receptacle slots
(1.5 mm a side) are looser than a real cover and receptacle, which makes yaw and height alignment easier than in a real
trial. The door is 4 mm thick with 16 mm of travel on a 225 N/m spring. All parts are rigid. The simulator scores the
last step without opening the grippers, so a light still held scores 0, as it would if it fell on release.

## Files

- `plug_a_night_light_through_a_sliding_outlet_cover.py`: the task file for the `yam_arena` embodiment (scene,
  `score()`, `success()`, `demo_script()`).
- `mechkit.py`: procedural geometry, the sprung door joint and scorer hooks. `closedloop.py`: the closed-loop demo
  controller. `belimits.py`: shared limit helpers.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Plug the night light into the top outlet through the sliding childproof cover, push it fully in, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=plug_a_night_light_through_a_sliding_outlet_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/nightlight-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=plug_a_night_light_through_a_sliding_outlet_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/nightlight-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the layout is fixed and there is no random seed.
- **Runtime per trial:** the demo plan is about 1,150 steps (58 s simulated, the last 10 s parked); the episode ends
  8 s after success. A Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 250 s for the demo and about 400 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (`SUCCESS_HOLD_S`), otherwise `score()` at the
  end of the run.

## Scoring

From the light's pose and the door's position. d is how far the shallower blade tip is past the plate's front face.
In the pockets means each blade tip is past the backing face and inside its own 4.5 x 9.3 mm receptacle slot.

- **Carried (C), latched over the run:** at some step the door was 10 of its 16 mm open (q, its distance from the open
  stop, 6 mm or less) with both blade tips at least 0.5 mm into the door and inside its slots.
- **End condition:** the light released (every finger pad point of both arms 30 mm or more from its bounding box) and
  still (under 3 mm/s for 1 s). Otherwise the score is 0.

| Outcome | Score |
| --- | --- |
| Both blades in their receptacle slots, d 13 mm or more, light square to the wall within 4 degrees | 1.0 |
| Both blades in, d 9 mm or more (or 13 mm or more but out of square) | 0.5 |
| C, and neither of the above (door carried, then lost, or blades not far enough in) | 0.25 |
| Straight push (stops on the door, d about 1 mm), blades only in the shut door's slots (d 5 mm), still held, light on the table without C | 0 |
| Light ever off the table (its lowest point 2 cm below the table top) | 0 for the run |

Holding the door open with a finger or the other hand, then plugging in, is scored by the same end state. In a real
trial: d is 15 mm minus the exposed blade length (ruler, side photo), the angle is read with a protractor from a top
photo, release and stillness are judged from the video, and C is read from the video against a mark on the recess rim
10 mm right of the shut door's right-hand edge (the door's edge reaching the mark with both tips in its slots).

## Real-world reset

Pull the light straight out (the door snaps shut) and stand it centred on its mat, blades toward the wall.
Nothing is consumed. The receptacle is unwired; it is never connected to the mains.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
