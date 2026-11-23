# Thread a hotel ball-top hanger into a free-swinging captive closet ring and drop its stem into the seat

Most hotel travelers have handled anti-theft ball-top hangers that have no open hook and hang only in a loose captive
security ring threaded onto the closet rod. Pushing the ball toward the ring with one hand simply skates the light 40 g
ring along the rail and swings it away like a pendulum. The robot must catch and steady the free-sliding, free-swinging
ring with one arm (keeping the front keyhole window plumb and unobstructed) while the other arm lifts the full-size
wooden suit hanger, guides its 16 mm chrome ball through the 19 x 18 mm upper window, drops its 6 mm stem into the
9 mm lower captive collar so the ball seats on the inner cradle ledges, and releases both parts so the hanger hangs
freely above the table.

## Parts (real sizes and commercial hardware)

Positions in cm from the midpoint between the arm bases, x forward, y left, z above the oak station table top.

- **Hotel ball-top suit hanger** (commercial reference: **National Hospitality Supply CPP88**, "Natural Heavyweight
  Solid Hardwood Hangers", men's suit, ball-top hook style, 17 in wide, 5/8 in thick, flat, fixed trouser bar, clear
  lacquer natural finish, chrome-plated metal parts; nathosp.com, $142.00 per case of 100): 43.1 cm span, 1.59 cm
  (5/8 in) thick flat natural hardwood body (`(0.88, 0.73, 0.48)`), 20 deg sloped shoulders (2.2 cm tall), 1.0 cm
  diameter wooden pant bar 12.8 cm below the ball center, 1.6 cm chrome steel ball on a 0.6 cm chrome steel stem
  (4.2 cm from ball center to wooden crown top), no hook. Starts leaning 12 deg backward in the valet tray with the ball
  center at x 24.5, y -4.0, z 14.2 cm. From the seller page: the 17 in width, 5/8 in thickness, flat profile, ball-top
  hook and fixed bar. Design values (the page gives none of them): the 140 g mass, the 16 mm ball, the 6 mm stem, the
  shoulder slope and the bar drop.
- **Captive closet security ring** (commercial reference: **National Hospitality Supply CPJRNG**, "J" style hanger
  ring, chrome-plated steel, "for use with ball-top hangers with solid bars", fits rods up to 1.5 in, slips over the end
  of the closet rod; nathosp.com, $39.95 per case of 100; modeled at 40 g split 15 g on `ring_slide` and 25 g on
  `ring_swing`): rides on chained `ring_slide` (y travel -31.2 to +31.2 cm, damping 0.25) and `ring_swing` (y-axis
  hinge +-54.4 deg, damping 0.002) joints on the closet rod starting at y +4.5 cm. Built from a 3.8 cm ID / 4.3 cm OD /
  1.0 cm wide chrome upper loop (3 mm radial clearance around the 3.2 cm rod) joined by a 0.3 cm top cap to a centered
  steel lower keyhole housing (`(0.46, 0.50, 0.56)`, 3.2 cm front-to-back, 4.5 cm wide, 2.75 cm tall below the cap,
  walls >= 2.5 mm): 1.9 cm wide x 1.8 cm tall upper front window above a 0.9 cm lower stem slot, solid rear wall,
  1.3 cm thick outer grasp cheeks, and two inner floor cradle ledges 4.95 cm below the rod axis. The whole ring is
  7.35 cm tall (loop top 2.15 cm above to ledge bottom 5.2 cm below the rod axis).
  **Which ring sizes are confirmed:** the seller page gives only the material (chrome-plated steel), the ball-top use
  and the 1.5 in rod fit, so the 38 mm loop ID is set from that fit. The 73.5 mm overall height, the 40 g mass, and the
  keyhole window, slot depth, ledge height and ledge-to-rod drop are design values sized to the hanger's 16 mm ball and
  6 mm stem (1.0 mm/side vertical and 1.5 mm/side lateral clearance). The 73.5 mm height is inside the 68 to 75 mm
  overall height of rings of this type. These design values are checked against the real ring with a caliper and
  scale at the one-time bench step in the parts list (step 6) before the first trial.
- **Closet bay and rod** (ordinary hardware: standard **1-1/4 in / 32 mm OD chrome steel closet rod**, e.g. Knape &
  Vogt 770 Series or Everbilt 1-1/4 in chrome rod, cut to 70 cm, mounted in a dark walnut tabletop frame): fixed dark
  walnut plywood (`(0.24, 0.19, 0.16)`) side walls 11.3 x 1.8 x 30 cm tall (x 35.4 to 46.7, inner faces y +-35.0 cm),
  back panel 1.2 x 73.6 x 30 cm (front face x 45.5 cm), and fixed 3.2 cm OD chrome rod at x 37.0, z 25.0 cm.
- **Valet tray**: fixed charcoal wood (`(0.22, 0.24, 0.28)`) tabletop tray, 5.65 x 44.0 x 0.8 cm base centered at
  x 22.6, y -4.0 cm, front lip 1.6 cm tall (back face x 21.0 cm), and two rear lean-rest pillars (1.8 x 2.4 x 5.4 cm)
  at y -18.5 and +10.5 cm leaving a 26.6 cm open center bay behind the right shoulder grasp.

## What the demo does and reads

Both arms run closed-loop at 20 Hz (`demo_script` in `hotel_ball_top_hanger_into_captive_closet_ring.py`, about 42 s /
840 steps including the 13.0 s final hold, well inside the 2,400-step limit). The left arm stays on the +y side, opens
to 64 mm, approaches the ring's +y cheek from front-left (yaw -70 deg, keeping the wrist and link 4 well in front of
the closet rod) at z 20.8 cm (on the cheek beside the upper keyhole window), closes gently onto the 32 mm cheek (`CLOSE_LEFT = -0.013`
m), and holds the ring steady and plumb near y +4.5 cm (8.9 cm clear of the left closet wall). Concurrently, the right
arm opens to 52 mm, approaches along +x pitched 12 deg down and rolled 20 deg to match the reclined hanger's right
wooden shoulder (5.5 cm right of the stem, zero gravity torque about the pad closing line), closes on the 22 mm wood
height, lifts 9.5 cm straight up clear above the valet tray lip and rear pillars, and then rotates
upright well above the tray. While the left hand holds the ring steady, the right hand aligns the chrome ball 3.2 cm in
front of the live keyhole window, advances smoothly along +x into the pocket center, pauses 0.5 s, and lowers 9.4 mm
straight down so the stem drops into the 9 mm lower slot until the ball is ~1 mm above the cradle ledges. The right
hand opens to 72 mm and backs straight out along -x so the hanger settles onto the cradle ledges, then the left hand
opens and backs straight out front-left clear of the rod, and both arms park at READY with open grippers well clear of
the closet and hanger.

The demo reads the live poses of the hanger and the `ring_swing` body from the simulator (to align the grasps and servo
the ball center cleanly through the keyhole window and down into the seat), plus the fixed layout (`object_jitter=0`),
its joint encoders, tracking error, and gripper opening.

## Fidelity

All parts are rigid wood and metal bodies with default contact offsets. The security ring rides on chained prismatic
(`ring_slide`) and revolute (`ring_swing`) joints centered on the rod axis with 3 mm radial clearance to the 32 mm rod,
preserving free lateral slide and free pendulum swing without simulating loose ring-on-rod tumbling. From the
commercial parts: the ring's 38 mm loop ID (fits rods up to 1.5 in), the hanger's 17 in width, 5/8 in thickness and
fixed bar. Design values: the ring's 73.5 mm overall height and 40 g mass, its keyhole window (19 mm wide x 18 mm
tall), inner pocket depth (22 mm), lower captive slot (9 mm) and ledge drop (49.5 mm below the rod axis), and the
hanger's 140 g mass, 16 mm ball and 6 mm stem. The keyhole gives 1.0 to 1.5 mm/side clearance on the ball and stem;
the housing is centered under the rod so the ring hangs plumb unloaded and loaded. Before the first real trial, the
bench step (parts list, step 6) measures the real ring and re-derives the simulated keyhole if any size differs by more
than 1 mm, and the real-trial seat check is calibrated on the ring used (see Scoring). The demo releases both parts,
opens both grippers and parks them at READY; the scorer also requires every finger pad >= 3 cm from both the hanger
and the ring, so a held hanger never scores.

## Files

- `hotel_ball_top_hanger_into_captive_closet_ring.py`: task scene, `score()`, `success()`, `ARM_KEEP_OUT`, and
  `demo_script()`.
- `fabricate.py`: optional stdlib-only script that writes ASCII STL files (in mm) of the valet tray and closet side
  stiles from the scene's exact dimensions when run directly.
- `mechkit.py`, `closedloop.py`, `belimits.py`: shared kit modules (procedural geometry and chained joints, closed-loop
  6-DoF demo controller, geometry helpers).
- `task.json`: task metadata and scoring specification.

## Run a trial

From this folder:

```bash
INSTR="Hang the wooden ball-top hanger in the chrome closet ring on the rail, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hotel_ball_top_hanger_into_captive_closet_ring.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/hanger-dream

# Scripted demo (bimanual, full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hotel_ball_top_hanger_into_captive_closet_ring.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/hanger-demo
```

- **Seed and layout:** `-E object_jitter=0` is required so the closet bay, valet tray, and starting tape marks stay at
  their specified coordinates.
- **Runtime per trial:** the scripted demo finishes in about 42 s of simulated time (about 840 steps, including the
  final stillness and success hold); a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 190 s for the demo and about 400 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` holds continuously for `SUCCESS_HOLD_S = 8.0` s, otherwise
  `score()` at the final step.

## Scoring

Scored after both parts are **Released** (every finger pad point of both arms >= 3.0 cm from both the hanger and the
ring) and **Still** (hanger and ring both moving < 1.5 cm/s linearly and < 0.15 rad/s angularly) continuously for
2.0 s at the end of the trial.

- **Latch D (dropped off table):** hanger lowest point drops more than 2.0 cm below the table top for >= 0.5 s -> score
  **0.0** for the entire trial.
- **Latch E (keyhole entry):** ball center stays inside the keyhole pocket (`|x_rel| <= 9 mm`, `|y_rel| <= 9 mm`,
  `z_rel` between -49 and -25 mm, in the swung ring's frame about the rod axis) for >= 0.3 s.
- **Seated:** all of (1) the ball center sits in the lower captive slot in the swung ring's frame: `|x_rel| <= 8 mm`
  (front-to-back), `|y_rel| <= 6 mm` (along the rod), and `z_rel` between -46.5 and -39.5 mm (it rests at -42.9 mm on
  the ledges); and (2) the hanger's lowest point is >= 3.5 cm above the table top. The ball can only be drawn forward
  out of the housing once its center rises above -35.9 mm, so the band top keeps 3.6 mm of captive margin. On a real
  ring the x/y part of (1) is the front photo showing the chrome stem between the two lower lips (the slot is 9 mm
  wide, so a stem there is within the 6 mm y limit) and the z part is the drop H or the pull test below.

### Seat check: simulation and real trial

The seat is scored by the same physical quantity in both, the **drop H** from the top of the closet rod to the top of
the hanger's wooden crown, measured straight down with a caliper depth rod or a steel rule while the hanger hangs
released. Each setting is calibrated on its own ring:

| | Simulation | Real trial |
|---|---|---|
| Calibration H0 (hanger seated on the ledges) | 100.9 mm, from the scene geometry (rod radius 16 mm + ball rest 42.9 mm + crown 42 mm below ball center); `check.py` prints it | Once per ring and hanger pair: seat the hanger by hand, let go, read H three times and log the mean as H0 |
| Seated | H from H0 - 3.4 to H0 + 3.6 mm (z_rel -39.5 to -46.5 mm) | Same band about the logged H0 if H0 - H_p >= 7 mm, else the pull test; plus the front photo shows the chrome stem between the two lower lips |
| Not seated, ball perched in the upper window | H about H0 - 9.4 mm (91.5 mm) | H more than 3.4 mm short of H0; the ball is visible in the window |

**When the drop band is valid.** At calibration also log the perched drop H_p: rest the ball in the upper window
(stem not in the slot), let go, read H three times. The H band above is valid only if H0 - H_p >= 7 mm on that ring
(simulation: 9.4 mm). If the step is under 7 mm, score Seated by a **pull test** instead of H: hold the ring still,
lift the hanger straight up by its crown by 4 mm (or half the logged step H0 - H_p if that is under 8 mm), then draw it
10 mm straight forward with under 2 N (pull through a 0 to 5 N spring scale, e.g. a Pesola Micro
20005 hooked on the crown; if it is not to hand, a light fingertip pull, no tug). Seated if the ball stays captive (the stem stops against the lower lips and the
hanger drops back onto the ledges when let go); not Seated if the ball comes out through the window. Log which method
was used for each trial.

So the score never depends on how far the commercial ring's seat sits below the rod, which varies with the ring
batch; it depends only on whether the stem has dropped down the slot onto the ledges. The band width is unchanged from
the first version of the task (7 mm).


| Score | Condition | Real-trial measurement |
|---|---|---|
| **1.0** | Seated, hanger tilt <= 18 deg, `|ring_swing|` <= 15 deg, both side walls clear >= 0.5 cm, Released, Still 2.0 s, no D | Caliper drop H within the band about H0 (or the pull test when H0 - H_p < 7 mm), front and side photos (tilt <= 18 deg, ring swing <= 15 deg, >= 5 mm gap to both walls), ruler >= 3.5 cm under lowest bar, trial video confirming release and 2 s stillness |
| **0.5** | Seated, Released, Still 2.0 s, no D, but hanger tilt 18 to 35 deg, `|ring_swing|` 15 to 30 deg, or side-wall clearance < 0.5 cm | Caliper drop H within the band about H0 (or the pull test when H0 - H_p < 7 mm), ruler >= 3.5 cm check, stem visible between the lower lips, front/side photo showing crooked hang (18 to 35 deg tilt or 15 to 30 deg swing) or stile contact |
| **0.25** | Latch E occurred, Released, Still 2.0 s, no D, not Seated within 0.5 limits (e.g. hanger dropped back onto tray/table or tilted > 35 deg) | Trial video shows the chrome ball visibly inside the keyhole window/pocket for >= 0.3 s before exiting or finishing unseated on the table/tray |
| **0.0** | Latch D, still held, still moving, draped on top of the rod, ball never entered keyhole pocket (no E), or nothing done | Trial video shows drop off table, gripper still touching/holding part, hanger draped over rod without entering keyhole, or untouched start |

Any lateral position along the rod with `|y_ring| <= 12.9 cm` clears both closet side walls by at least 5 mm (including
steadying the ring right at its starting y +4.5 cm without sliding it to center), so score does not depend on
irrelevant lateral placement.

## Real-world build and reset

### Parts list and assembly
1. **Hanger:** 1x National Hospitality Supply CPP88 17 in natural solid hardwood ball-top men's suit hanger with
   fixed trouser bar, 5/8 in thick (nathosp.com, $142.00 per case of 100; about $1.42 each).
2. **Captive ring:** 1x National Hospitality Supply CPJRNG "J" style chrome-plated steel hanger ring for ball-top
   hangers, fits rods up to 1.5 in (nathosp.com, $39.95 per case of 100). Its seated drop H0 and perched drop H_p are
   calibrated before the first trial (step 6).
3. **Closet rod:** 1x standard 1-1/4 in (32 mm OD) chrome steel closet rod (e.g. Knape & Vogt 770 Series or Everbilt
   1-1/4 in chrome rod, hardware store, ~$12), cut to 70 cm length.
4. **Closet frame and valet tray:** Cut two side walls (11.3 x 1.8 x 30 cm) and one back panel (1.2 x 73.6 x 30 cm)
   from 18 mm and 12 mm dark walnut-stained plywood (~$15), drill 32.5 mm blind rod sockets centered 1.6 cm from the
   front edge and 25.0 cm up, slide the CPJRNG ring onto the rod end before screwing the frame together, and clamp or
   screw the frame to the table with the rod at x 37.0 cm and back face at x 45.5 cm. Build the charcoal wooden valet
   tray from scrap wood or print the exact cradle with `python3 fabricate.py --out ./stl` (PLA/PETG, 15% infill) and
   tape it down centered at x 22.6, y -4.0 cm.
5. **Forces and cleanliness:** Lifting the 140 g hanger and steadying the 40 g ring require under 3 N per finger (far
   below the YAM's ~40 N per finger limit). All parts are dry rigid wood and plated steel, leaving no residue on shared
   rigs.
6. **One-time bench step: measure the ring and calibrate the seat check (once per ring and hanger pair).**
   (a) With a 150 mm digital caliper (any brand, ~$20) and a 0.1 g kitchen scale, measure the real ring's overall
   height, loop ID and mass, its keyhole window width and height, lower slot width, slot depth (window bottom to ledge
   top) and ledge-to-rod drop (rod axis to ledge top, with the ring on the rod), and the hanger's ball and stem
   diameters. Compare them with the simulated values (README Parts and `RING_*` constants: window 19 x 18 mm, slot
   9 mm, slot depth 7.0 mm, ledge drop 49.5 mm, ball 16 mm, stem 6 mm). If any differs by more than 1 mm, re-derive
   the simulated keyhole from the measurements (edit the `RING_*`, `WIN_HALF_Y`, `SLOT_HALF`, `BALL_R`, `STEM_R`
   constants and the `SEAT_Z` band about the new resting height) and rerun the offline checks before trials.
   (b) Seat the hanger by hand (ball through the window, stem down the slot onto the ledges), let go, and measure the
   drop H from the top of the rod to the top of the wooden crown three times with the caliper depth rod or a steel
   rule; log the mean as H0. (c) Rest the ball in the upper window, let go, and log the perched drop H_p the same way.
   If H0 - H_p >= 7 mm, mark the H0 -3.4 / +3.6 mm band on a strip of tape on the back panel beside the ring for quick
   reading; otherwise use the pull test (Scoring) for Seated.

**Bill of materials (approximate retail):** hangers $142 (case of 100) + rings $39.95 (case of 100) + chrome rod
$12 + plywood $15 + caliper $20 + kitchen scale $12 + screws, tape and printed tray filament $5 = **about $246**
(about $66 counting one hanger and one ring at their per-unit case cost).

### Real-world reset
Lift the hanger about 1 cm straight up to align the chrome ball with the upper keyhole window, draw it forward out of
the ring, place it leaning 12 deg back in the valet tray against the two rear pillars on taped alignment marks, and
slide the chrome ring back to the y +4.5 cm tape mark on the closet rod. Nothing is consumed between trials.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
