# Dock a hand shower in its wall holder

A hand shower on its hose lies on a ledge. The robot picks it up, stands it head up, brings the hanging hose in through
the front slot of the wall holder's cup, lowers the tapered handle until it seats on the cup's bottom ring, and lets go.
Anyone who has rinsed with a hand shower has done this: the cup closes round the handle, so the handset seats only if
the hose went in sideways through the slot first. The hose is anchored at the wall outlet and pulls on the handset the
whole way.

## Parts (all full size)

- **Hand shower:** white ABS, 250 mm long, 0.10 kg. Chrome hose nut 20 mm diameter x 10 mm. Round handle tapering from
  22 mm to 28 mm over 10 to 31 mm from the nut's tip, then 28 mm out to 160 mm. Red depth band 9 to 15 mm from the tip.
  Round head 90 mm diameter x 28 mm thick, centred 205 mm from the tip, with a chrome rim and a grey nozzle disc.
- **Holder:** matte black ABS cup: conical bore 26 mm at the bottom ring and 34 mm at the rim, 32 mm deep, 4 mm wall,
  leaning 15 degrees toward the user. Front slot 18 mm wide at the ring (3 mm each side of the hose) and 24 mm at the
  rim, narrower than the 28 mm handle. Bottom-ring centre at x 0.395, y 0.025, 0.151 m above the table. Arm 62 x 24 x
  16 mm back to a 50 mm rosette on the wall.
- **Hose:** black, 12 mm diameter, 0.50 m, 0.085 kg (0.17 kg/m), fixed at the outlet spout (x 0.458, y 0.165, 0.173 m
  up).
- **Outlet elbow:** black 20 mm bar from a 44 mm wall rosette, turned down to the spout (look only; the hose is
  anchored there).
- **Wall panel:** pale blue-grey tile, 440 x 360 x 12 mm, face at x 0.470. **Ledge:** grey stone block 158 x 90 x
  100 mm, fixed; the handset lies face down on it, handle overhanging 102 mm toward the wall.

The handle's 26 mm section meets the bottom ring 24 mm from the nut's tip (the seat point). Measured in the code with
`mechkit.gap`: the 12 mm hose passes the slot with 3.4 mm each side, the 28 mm handle and the 20 mm nut cannot pass
it, and a seated handle can lean at most 4 degrees.

## The scripted demo

Right arm only; the left arm stays parked. About 34 s of motion, then it waits (909 steps with the wait).

1. Level gripper pointing left, one finger above the handle and one below, slides in from the side where the handle
   overhangs the ledge, and closes on the handle 80 mm from the nut's tip.
2. Lifts 75 mm, then rolls the wrist 75 degrees so the head rises and the handset lies along the cup's axis.
3. Moves to 40 mm in front of the slot, nut tip 15 mm above the rim, and waits until the hose hangs within 3 mm of the
   slot's centre plane.
4. Moves 40 mm straight in, so the hose passes through the slot, then lowers the handle along the cup axis at 15 mm/s
   until it stops on the ring.
5. Opens the gripper, backs the pads off sideways, parks at the ready pose and stays still.

The demo reads the live pose of the handset and the hose's segment points from the simulator each step, to grasp the
handle, to keep the hose centred on the slot and to know when the handle has seated; it also uses its joint encoders
(`trim_to_measured`, `holding`, `tracking_error`). It never moves or edits a body or joint.

## Fidelity

Approximate. The handset, cup, slot and hose diameter are full size. The hose is 0.5 m, where real hoses are 1.5 to
2 m with the spare length lying in the tub; it has the real weight per metre, so the handle is pulled by a shorter
hanging length. The last 10 cm of hose, the part that has to enter the slot, is full size. The hose is a chain of 20
rigid 25 mm segments with bend springs (0.015 N m/rad) and no twist; there is no swivel at the nut; the cup and handle
are rigid, where real holders are slightly springy plastic. The holder is 15 cm above the table instead of at head
height, which keeps the same motion: the wrist roll, the coaxial approach and the drop into the cup. No water. The
simulator scores the last step without opening the grippers, so a handset still held scores 0, as it would if it fell
on release. The real build (below) prints the holder and handset from the scene's dimensions; its hose is 12.7 mm
neoprene with a steel rope inside to match the 0.17 kg/m.

## Files

- `dock_a_hand_shower_in_its_slotted_wall_holder.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `fabricate.py`: writes print-ready ASCII STL files (mm) of the holder, the handset, the outlet elbow and the ledge
  from the scene's dimensions (`python3 fabricate.py [out_dir]`; standard library only, importing it writes nothing).
- `mechkit.py`, `ropekit.py`: procedural geometry, the hose and scorer hooks. `closedloop.py`: the closed-loop demo
  controller. `belimits.py`: shared limit helpers.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Put the hand shower back in its wall holder: hose in through the cup's front slot, handle down into the cup, let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=dock_a_hand_shower_in_its_slotted_wall_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/shower-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=dock_a_hand_shower_in_its_slotted_wall_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/shower-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the layout is fixed and there is no random seed.
- **Runtime per trial:** the demo plan is 909 steps (about 45 s simulated, the last 11 s waiting); a Dream-1 trial runs
  up to 2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 190 s for the demo and about 510 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (`SUCCESS_HOLD_S`), otherwise `score()` at the
  end of the run.

## Scoring

From the handset's pose against the fixed cup. Seat point S is on the handle axis 24 mm from the nut's tip.

- **Seated:** S within -4 to +6 mm of the bottom ring's centre along the cup axis and within 4 mm of the axis, and the
  handle axis within 10 degrees of the cup's.
- **Released:** every finger pad point of both arms at least 3 cm from the handset and from the last 15 cm of hose
  (6 segments).
- **Still:** the nut's tip under 1 cm/s and the handset turning under 5 deg/s for 2 s.

| Outcome | Score |
| --- | --- |
| Seated, released and still | 1.0 |
| Released and still, not seated, nut end inside the bore 10 mm or more below the rim plane (or through the bottom ring) | 0.4 |
| Still held (even if seated), hung on a hose pinched over the rim, beside the cup, left on the ledge | 0 |
| Handset ever dropped: falling over 0.6 m/s, or any part under 3 cm above the table, with no finger within 3 cm | 0 for the run |
| Hose torn from the outlet (measured length over 1.02 x 0.50 m) | 0 for the run |

The hose rule is a guard: the simulated chain cannot stretch, so it only trips if the simulation breaks. The drop rule
does not trip while the handset is held. In a real trial: the top of the red band shows 3 to 13 mm below the cup's
bottom edge when seated (ruler), the axis angle is read with a protractor from a side photo, and release and stillness
are judged from the video.

## Real-world build

The holder cup and the handset are printed from `fabricate.py`, so the slot, taper and seat match the scene exactly;
the hose is a stock tube. Everything costs under about US$60.

### Parts

| Part | Source | Notes |
| --- | --- | --- |
| Holder (cup, arm, rosette) | `holder.stl` from `fabricate.py` | Black PETG, 4 walls, 40% infill, printed rim up as in the scene, supports under the arm and rosette. 18 mm slot at the bottom ring, 24 mm at the rim. |
| Hand shower | `handset.stl` | White PETG, spray face down on the bed, supports under the handle and nut. 272 cm3 solid: set walls and infill so it weighs 100 +/- 10 g (2 walls and about 10% gyroid is a starting point; weigh it). The 20 mm nut has a 13 mm socket 15 mm deep with a barb for the hose. |
| Outlet elbow | `outlet_elbow.stl` | Black PETG, rosette back on the bed, supports under the down bar. The spout end has the same hose socket and barb. |
| Ledge | `ledge.stl` | Grey PLA, open-bottom box 158 x 90 x 100 mm with 3 mm walls, printed top face down. A wood block of the same size painted grey works too. |
| Hose | McMaster-Carr 5034K23, black neoprene tube 1/4 in ID x 1/2 in OD (6.35 x 12.7 mm), 1 ft | Cut 530 mm: 500 mm free plus 15 mm in each socket. |
| Hose ballast | 1/8 in 7x19 galvanised steel wire rope (any hardware store), 490 mm | Threaded inside the tube between the two barbs, ends wrapped in tape, to bring the hose to the scene's 0.17 kg/m. |
| Wall panel | 12 mm plywood or MDF, 440 x 360 mm | Gloss pale blue-grey paint, or peel-and-stick tile sheet; two steel corner braces behind it and two bench clamps. |
| Mounting | 3M VHB 5952 tape, 25 mm wide | One 50 mm disc on the holder's rosette and one 44 mm disc on the elbow's. |
| Paint | Red and silver paint markers | Red band 9 to 15 mm from the nut's tip; silver on the nut and the head's rim (look only). |

### Assembly

1. Run `python3 fabricate.py` and print the four STL files. Clear the support from inside the cup and the sockets;
   check the slot with a 12.7 mm drill shank (it must pass) and the nut with the handset itself (20 mm, it must not).
2. Paint the red band 9 to 15 mm from the nut's tip (a ruler and masking tape give a clean edge) and the silver
   trim. Weigh the handset: 100 +/- 10 g.
3. Hose: cut 530 mm of the neoprene tube. Push one end over the handset's barb until it bottoms in the socket (15 mm
   deep; warm the tube end in hot water if it is stiff). Thread the 490 mm wire rope in from the open end, then push
   that end onto the elbow's barb the same way. A drop of cyanoacrylate on each barb makes the joints permanent. Weigh
   the tube and rope before fitting them: 85 +/- 5 g (trim the rope if heavy).
4. Stand the panel on the table, tile face toward the robot, 47 cm in front of the midpoint between the arm bases
   (x 0.470), centred on y = 0, and clamp its braces to the table.
5. Stick the holder's rosette to the panel with VHB: rosette centre 2.5 cm left of the panel's centre line as seen by
   the robot and 17.2 cm above the table, slot facing the robot. The cup's lean is built in.
6. Stick the elbow's rosette: centre 16.5 cm left of the panel's centre line and 19.7 cm above the table, the spout
   pointing down. Let the VHB cure for an hour before the first trial.
7. Place the ledge at x 13.0 to 28.8 cm, y -9.0 to 0 (its long side along x), and fix it with double-sided tape. Put
   two tape marks on its top: a cross at the head's centre (x 18.5, y -4.5 cm) and a line along y -4.5 cm for the
   handle. Laid on them, the nut's tip is at x 39 cm, 10.2 cm past the ledge's wall-side edge.

The handset seats when its 26 mm handle section meets the 26 mm bottom ring; the top of the red band then shows 3 to
13 mm below the cup's bottom edge. The real hose is 12.7 mm, 0.7 mm more than the scene's 12 mm, so it passes the
18 mm slot with 2.6 mm each side instead of 3.

### Reset and cleaning

Lift the handset out of the cup and lay it face down on the ledge at its marks, nut toward the wall, handle
overhanging; lay the hose down the wall, along its foot and up to the nut as in the setup photo. Nothing is consumed
and no water is used. Wipe the handle and the cup's bore with a dry cloth if they get dusty; check every 50 trials
that the VHB pads and hose joints are tight.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder (print files by fabricate.py); no
third-party assets.
