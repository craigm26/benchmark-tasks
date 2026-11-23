# Thread a hotel ball-top hanger into a free-swinging captive closet ring and drop its stem into the seat

Most hotel travelers have handled anti-theft ball-top hangers that have no open hook and hang only in a loose captive
security ring threaded onto the closet rod. Pushing the ball toward the ring with one hand simply skates the light 40 g
ring along the rail and swings it away like a pendulum. The robot must catch and steady the free-sliding, free-swinging
ring with one arm (keeping the front keyhole window plumb and unobstructed) while the other arm lifts the full-size
wooden suit hanger, guides its 16 mm chrome ball through the 22 x 22 mm upper window, drops its 6 mm stem into the
12 mm lower captive collar so the ball seats on the inner cradle ledges, and releases both parts so the hanger hangs
freely above the table.

## Parts (real sizes and commercial hardware)

Positions in cm from the midpoint between the arm bases, x forward, y left, z above the oak station table top.

- **Hotel ball-top suit hanger** (commercial reference: **Nahanco 60017BEC**, 17 in natural low-gloss maple flat suit
  hanger with chrome anti-theft ball top and stationary wooden pant bar; ~140 g): 43.1 cm span, 1.3 cm (1/2 in) thick
  flat natural maple body (`(0.88, 0.73, 0.48)`), 20 deg sloped shoulders (2.2 cm tall), 1.0 cm diameter wooden pant
  bar 12.8 cm below the ball center, 1.6 cm (5/8 in) chrome steel ball on a 0.6 cm chrome steel stem (4.2 cm from ball
  center to wooden crown top), no hook. Starts leaning 12 deg backward in the valet tray with the ball center at
  x 24.5, y -4.0, z 14.2 cm.
- **Captive closet security ring** (commercial reference: **Nahanco HHSRA100**, solid chrome-plated steel A-ring for
  1.5 in / 38 mm closet poles, ~41 g each; modeled at 40 g split 15 g on `ring_slide` and 25 g on `ring_swing`): rides
  on chained `ring_slide` (y travel -31.2 to +31.2 cm, damping 0.25) and `ring_swing` (y-axis hinge +-54.4 deg,
  damping 0.002) joints on the closet rod starting at y +4.5 cm. Built from a 3.8 cm ID / 4.6 cm OD / 1.0 cm wide chrome
  upper loop (3 mm radial clearance around the 3.2 cm rod) joined by a 0.5 cm top cap to a centered brushed-steel lower
  keyhole housing (`(0.46, 0.50, 0.56)`, 3.2 cm front-to-back, 4.8 cm wide, 4.4 cm tall, walls >= 5 mm): 2.2 x 2.2 cm
  upper front window above a 1.2 cm lower stem slot, solid rear wall, 1.3 cm thick outer grasp cheeks, and two inner
  floor cradle ledges 6.35 cm below the rod axis.
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
the closet rod) at z 19.4 cm (below the upper keyhole window), closes gently onto the 32 mm cheek (`CLOSE_LEFT = -0.013`
m), and holds the ring steady and plumb near y +4.5 cm (8.9 cm clear of the left closet wall). Concurrently, the right
arm opens to 52 mm, approaches along +x pitched 12 deg down and rolled 20 deg to match the reclined hanger's right
wooden shoulder (5.5 cm right of the stem, zero gravity torque about the pad closing line), closes on the 22 mm wood
height, lifts 9.5 cm straight up clear above the valet tray lip and rear pillars, and then rotates
upright well above the tray. While the left hand holds the ring steady, the right hand aligns the chrome ball 3.2 cm in
front of the live keyhole window, advances smoothly along +x into the pocket center, pauses 0.5 s, and lowers 2.2 cm
straight down so the stem drops into the 12 mm lower slot until the ball is ~1 mm above the cradle ledges. The right
hand opens to 72 mm and backs straight out along -x so the hanger settles onto the cradle ledges, then the left hand
opens and backs straight out front-left clear of the rod, and both arms park at READY with open grippers well clear of
the closet and hanger.

The demo reads the live poses of the hanger and the `ring_swing` body from the simulator (to align the grasps and servo
the ball center cleanly through the keyhole window and down into the seat), plus the fixed layout (`object_jitter=0`),
its joint encoders, tracking error, and gripper opening.

## Fidelity

All parts are rigid wood and metal bodies with default contact offsets. The security ring rides on chained prismatic
(`ring_slide`) and revolute (`ring_swing`) joints centered on the rod axis with 3 mm radial clearance to the 32 mm rod,
preserving free lateral slide and free pendulum swing without simulating loose ring-on-rod tumbling. The keyhole window
(22 x 22 mm), inner pocket depth (22 mm), and lower captive slot (12 mm) include 3 mm/side clearance around the 16 mm
ball and 6 mm stem for reliable simulator contact (making the modeled ring housing 90 mm overall vs ~68 to 75 mm on the
commercial Nahanco HHSRA100, and keeping the housing centered under the rod so the ring hangs within 1.6 deg of plumb
unloaded and < 0.2 deg loaded). The simulator scores the final step without opening the grippers; the scorer checks
finger-pad clearance (>= 3 cm from both hanger and ring) so a held hanger never scores better than a released one.

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
- Wall-clock time on one L40S, including Isaac Sim start-up: about 160 s for the demo and about 380 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 once `success()` holds continuously for `SUCCESS_HOLD_S = 8.0` s, otherwise
  `score()` at the final step.

## Scoring

Scored after both parts are **Released** (every finger pad point of both arms >= 3.0 cm from both the hanger and the
ring) and **Still** (hanger and ring both moving < 1.5 cm/s linearly and < 0.15 rad/s angularly) continuously for
2.0 s at the end of the trial.

- **Latch D (dropped off table):** hanger lowest point drops more than 2.0 cm below the table top for >= 0.5 s -> score
  **0.0** for the entire trial.
- **Latch E (keyhole entry):** ball center stays inside the 2.2 x 2.2 cm keyhole pocket (`|x_rel| <= 9 mm`,
  `|y_rel| <= 9 mm`, `z_rel` between -63 and -24 mm below the rod axis) for >= 0.3 s.
- **Seated:** ball center inside the lower captive collar below the window (`|x_rel| <= 8 mm`, `|y_rel| <= 6 mm`,
  `z_rel` between -62 and -54.5 mm below the rod axis) AND hanger lowest point >= 3.5 cm above the table top.

| Score | Condition | Real-trial measurement |
|---|---|---|
| **1.0** | Seated, hanger tilt <= 18 deg, `|ring_swing|` <= 15 deg, both side walls clear >= 0.5 cm, Released, Still 2.0 s, no D | Front and side photos (tilt <= 18 deg, ring swing <= 15 deg, >= 5 mm gap to both walls), ruler >= 3.5 cm under lowest bar, trial video confirming release and 2 s stillness |
| **0.5** | Seated, Released, Still 2.0 s, no D, but hanger tilt 18 to 35 deg, `|ring_swing|` 15 to 30 deg, or side-wall clearance < 0.5 cm | Same ruler >= 3.5 cm check, ball visibly captive in lower collar, front/side photo showing crooked hang (18 to 35 deg tilt or 15 to 30 deg swing) or stile contact |
| **0.25** | Latch E occurred, Released, Still 2.0 s, no D, not Seated within 0.5 limits (e.g. hanger dropped back onto tray/table or tilted > 35 deg) | Trial video shows the chrome ball visibly inside the keyhole window/pocket for >= 0.3 s before exiting or finishing unseated on the table/tray |
| **0.0** | Latch D, still held, still moving, draped on top of the rod, ball never entered keyhole pocket (no E), or nothing done | Trial video shows drop off table, gripper still touching/holding part, hanger draped over rod without entering keyhole, or untouched start |

Any lateral position along the rod with `|y_ring| <= 12.9 cm` clears both closet side walls by at least 5 mm (including
steadying the ring right at its starting y +4.5 cm without sliding it to center), so score does not depend on
irrelevant lateral placement.

## Real-world build and reset

### Parts list and assembly
1. **Hanger:** 1x Nahanco 60017BEC 17 in natural wooden anti-theft ball-top hotel suit hanger (hospitality supply /
   Nahanco, ~$5 single or ~$1.50/unit in bulk packs).
2. **Captive ring:** 1x Nahanco HHSRA100 solid chrome-plated steel A-style security ring for 1.5 in rod (Nahanco,
   ~$1/unit in bulk or ~$5 single).
3. **Closet rod:** 1x standard 1-1/4 in (32 mm OD) chrome steel closet rod (e.g. Knape & Vogt 770 Series or Everbilt
   1-1/4 in chrome rod, hardware store, ~$12), cut to 70 cm length.
4. **Closet frame and valet tray:** Cut two side walls (11.3 x 1.8 x 30 cm) and one back panel (1.2 x 73.6 x 30 cm)
   from 18 mm and 12 mm dark walnut-stained plywood (~$15), drill 32.5 mm blind rod sockets centered 1.6 cm from the
   front edge and 25.0 cm up, thread the HHSRA100 ring onto the rod before screwing the frame together, and clamp or
   screw the frame to the table with the rod at x 37.0 cm and back face at x 45.5 cm. Build the charcoal wooden valet
   tray from scrap wood or print the exact cradle with `python3 fabricate.py --out ./stl` (PLA/PETG, 15% infill) and
   tape it down centered at x 22.6, y -4.0 cm.
5. **Forces and cleanliness:** Lifting the 140 g hanger and steadying the 40 g ring require under 3 N per finger (far
   below the YAM's ~40 N per finger limit). All parts are dry rigid wood and plated steel, leaving no residue on shared
   rigs.

### Real-world reset
Lift the hanger about 2.2 cm straight up to align the chrome ball with the upper keyhole window, draw it forward out of
the ring, place it leaning 12 deg back in the valet tray against the two rear pillars on taped alignment marks, and
slide the chrome ring back to the y +4.5 cm tape mark on the closet rod. Nothing is consumed between trials.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
