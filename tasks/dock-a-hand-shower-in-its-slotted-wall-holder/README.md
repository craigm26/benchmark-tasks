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
on release.

## Files

- `dock_a_hand_shower_in_its_slotted_wall_holder.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
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
  up to 2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 200 s for the demo and about 490 s for a full Dream-1 trial.
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

## Real-world reset

Lift the handset out of the cup and lay it face down on the ledge at its marks, with the hose along the foot of the
wall. Nothing is consumed. Use a wall offcut or board clamped to the bench for the panel; no water connection is
needed.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder; no third-party assets.
