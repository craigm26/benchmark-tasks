# Route a plugged-in cord out the bottom notch of a while-in-use cover and close the hood over it

A tool cord is plugged into an outdoor receptacle under an open while-in-use cover, and it runs straight out of the
front of the cover. Shutting the hood now would land its front wall on the cord and leave the receptacle open to rain.
The robot has to take the cord down through the exit notch in the hood's bottom wall and keep it there while the hood
swings shut over the plug and clicks onto its rim. NEC 406.9(B)(1) requires a cover that stays weatherproof with a plug
in, and closing it over a cord led out through its exit is daily temporary-power work for construction laborers (O*NET
47-2061.00, about 1.0M US workers), electricians (47-2111.00, 0.8M) and carpenters (47-2031.00, 0.7M). The hard part is
physical: a springy 11 mm cord must be held in an 18 mm notch it cannot see from above while a hinged hood closes past
the hand, and a cord left under the rim holds the hood open.

## Parts (all full size)

| Part | Real product | Size | Material |
|---|---|---|---|
| While-in-use cover | TayMac MM410C (1-gang, 16-in-1, 2-3/4 in deep, clear) | base frame 102 x 140 x 10 mm; hood 70 mm deep, 0.09 kg, hinged on its top edge | clear polycarbonate |
| Weatherproof box | Hubbell-Bell 5320-0 (1-gang, three 1/2 in outlets) | 114 x 70 x 51 mm | grey die-cast aluminium |
| Receptacle | any 15 A 125 V duplex, NEMA 5-15R | outlets 38 mm apart | ivory nylon |
| Plug and cord | 12/3 SJTW extension cord, 25 ft, moulded NEMA 5-15P plug | jacket 11 mm, about 0.2 kg/m; plug body 28 x 28 x 36 mm, 40 mm with strain relief | yellow vinyl jacket, black plug |
| Backer, foot, gusset | custom (fabricate.py) | 300 x 300 x 19 mm, 300 x 150 x 19 mm, 100 mm gusset | 3/4 in plywood |

Layout (cm from the midpoint between the arm bases, x forward, y left, heights above the table): backer front face at
x 41.6; box front at x 36.5, centred y 0, 17 up; cover base x 35.5 to 36.5, 10 to 24 up; hood hinged at 24 up, open
about 75 deg upward; exit notch in the hood's bottom wall at y +2.6 to +4.4, 5.2 cm deep from the base; plug seated
in the lower outlet, 15.1 up. The cord leaves the plug toward the robot, past the hood's front plane, drops to the
table at x 26, y +9, runs left to y +17 and back toward the robot to x 12, y +21.

## What the scripted demo does

The left hand takes the cord where it reaches the table, leaned 26 deg so its wrist stays clear of the open hood, and
carries it back and down to x 31.5, y +13.5 just above the table. That pulls the run from the plug down behind the
hood's front plane and through the notch's line; the hand then steers its hold until the cord crosses the cover's bottom
plane at the notch centre, and keeps holding. The right hand, fingers closed and leaned 46 deg, presses the hood's front
face and swings it down along its arc to 25 deg open; the detent spring and the hood's weight carry it onto its rim,
the notch closing round the cord. If it has not latched, the right hand presses it again. Then the left hand opens, rises
and both arms swing home. The demo reads the cord's live segment positions and the hood's hinge angle from the
simulator.

## Fidelity

The cord is 20 rigid 2 cm segments joined by bend springs (0.012 N m/rad) toward its laid shape, so it springs back out
of the notch if let go before the hood is shut. The plug is welded into the outlet: pulling the cord cannot unseat it
in the simulation, so plug seating is checked only in the real trial. The hood is a rigid hinge with a 0.5 N m/rad
detent spring holding it open and snapping it shut past half way, which stands in for the latch click; there is no
gasket. The simulator scores the final step without opening the grippers; a hand still on the cover or cord cannot
earn more than the 0.5 tier, which needs the hood within 10 deg of shut whether held or not.

## Run a trial

```bash
INSTR="Route the plugged-in cord down through the exit notch in the bottom of the open in-use cover, then close the hood until it latches."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ext_jobsite_temporary_power_spider_boxes_25.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/inuse-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ext_jobsite_temporary_power_spider_boxes_25.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/inuse-demo
```

- **Seed and layout:** `-E object_jitter=0` keeps the fixture where the task file puts it; the cord is never jittered.
- **Runtime per trial:** the demo takes about 30 s of simulated time (about 600 steps); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 200 s for the demo and about 520 s for a full Dream-1 trial.

## Scoring

At the end of the run, grippers opened. The cord is **routed** when, followed from the plug, its centreline stays inside
the shut hood's volume until it leaves it downward through the bottom exit notch (crossing the cover's bottom plane
inside the 18 mm notch) and never re-enters.

- **1.0:** routed, the hood shut on its base rim with the latch clicked (within 0.02 rad, about 1 deg, of shut) and both
  hands at least 3 cm from the cover.
- **0.5:** routed and the hood within 10 deg of shut.
- **0.25:** routed, the hood further open, both hands at least 3 cm from the cord and the cover.
- **0:** the cord leaves through the hood's front or a side, crosses the bottom outside the notch, lies under the rim
  (holding the hood open), or is pulled apart; or the plug is not fully seated (any blade showing; real trial). Doing
  nothing scores 0 (hood open, cord out the front).

In a real trial, judge routing by eye and photo from below the cover (the hood is clear) and the latch by the click and
a flush rim (no gap a 1 mm feeler enters).

## Real-world build and reset

Bill of materials (typical US retail, November 2026): TayMac MM410C cover about $15; Hubbell-Bell 5320-0 box about $12;
15 A duplex receptacle about $3; 25 ft 12/3 SJTW extension cord about $35; 3/4 in plywood offcut, four 1-1/4 in wood
screws, two 1/2 in close-up plugs for the box's unused outlets about $10. Total about $75.

Assembly: cut the backer, foot and gusset (fabricate.py writes their STL templates), screw the backer upright to the
foot with the gusset behind it, and clamp the foot to the table so the backer's front face is at x 41.6. Screw the box
to the backer through its mounting lugs with its bottom 11 cm above the table and plug its outlets. Fit the receptacle
(not wired) and the MM410C base with its gasket, hinge at the top. Break out the hood's bottom cord exit on the left
side so it forms an 18 mm U notch. Plug the cord into the lower outlet; its far end is not plugged into anything, so
nothing is live. Every force the action needs is under 5 N (the hood weighs 0.09 kg; its latch closes by hand).

Reset: open the hood to its open stop, re-seat the plug, and lay the cord along tape marks on the table (straight out of
the plug, down to x 26, y +9, left to y +17, toward the robot to x 12, y +21). Wipe the hood if handled; nothing is
consumed.

## Files

- `ext_jobsite_temporary_power_spider_boxes_25.py`: scene, `score()`, `success()` and `demo_script()`.
- `mechkit.py`, `ropekit.py`, `closedloop.py`, `belimits.py`: the shared kit (geometry, cord, closed-loop demo).
- `fabricate.py`: STL templates for the plywood backer, foot and gusset.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder; no third-party assets.
