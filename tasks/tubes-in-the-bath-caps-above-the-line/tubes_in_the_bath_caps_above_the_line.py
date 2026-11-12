"""On the station table: a steel-grey water bath (fixed, 16 x 20 cm outside, walls 7 cm tall) stands 17 to 33 cm in
front of the arms, centred between them. A small flat blue ledge on each side wall marks the water line, 6 cm above the
table. Inside the bath stands a dark grey tube rack (fixed): a grid 5 cm above the table with four 5 cm square openings,
centred at x 21 and 29 cm, y +4 and -4 cm, and a thin steel post in the middle, top 10.5 cm above the table.

The simulation has no water. Instead a blue disc floats in each opening at the water line and pushes up on whatever
stands on it exactly as the water would on a 3 cm tube: 0.069 N for every centimetre the tube's underside is below the
line. Four loose orange sample tubes (3 cm across, 8 cm tall, 20 g: part filled, so they float) stand on the bench in a
row at x 12, y +15, +6, -6 and -15. Stood in an opening and let go, a tube floats with its underside 3.2 cm above the
table, 2.2 cm above its seat in the rack (1 cm above the table).

A loose white hold-down plate (12 x 12 x 0.8 cm, 180 g, with a square grip collar 4 cm across and 3 cm tall around a
3 cm hole in its middle) lies on the bench at x 25, y +21. Lowered over the post onto the four caps, its weight pushes
the tubes down onto their seats and keeps them there.

Requires -E object_jitter=0: the demo's grasps and the setup are for the listed positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=tubes_in_the_bath_caps_above_the_line.py \\
           -E object_jitter=0 --instruction "Load the four sample tubes into the water bath rack and fit the
           hold-down plate over them." --scorer task_score ...
"""

import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, bottom, inside, overlaps, tilt_deg, top  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl  # noqa: E402

from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
# success() only sees the objects, and the plate counts as fitted the moment it rests on the caps, so a 1 s hold would
# end the episode with the hand still around its collar. Holding success for 6 s keeps the run going while the gripper
# opens, rises and parks, so the release is seen.
SUCCESS_HOLD_S = 6.0

STEEL, RACK_GREY, BLUE = (0.60, 0.62, 0.65), (0.24, 0.25, 0.28), (0.05, 0.25, 0.90)
ORANGE = (0.95, 0.40, 0.05)  # deep orange: stands out from the oak table
WHITE = (0.93, 0.93, 0.90)

# --- the bath ---------------------------------------------------------------------------------------------------------
# A 0.5 cm floor and 1 cm walls around an interior of x 0.18..0.32, |y| <= 0.09. Wall tops at TOP + 0.07.
BATH_X, BATH_Y = 0.25, 0.0
FLOOR_H, WALL_T, WALL_TOP = 0.005, 0.01, TOP + 0.07
FLOOR_Z = TOP + FLOOR_H
WALL_H = WALL_TOP - FLOOR_Z
WALL_Z = FLOOR_Z + WALL_H / 2
INTERIOR = Rect((0.18, 0.32), (-0.09, 0.09))
BATH = [
    Box("bath_floor", pos=(BATH_X, BATH_Y, TOP + FLOOR_H / 2), size=(0.16, 0.20, FLOOR_H), color=STEEL),
    Box("bath_wall_near", pos=(0.175, 0.0, WALL_Z), size=(WALL_T, 0.20, WALL_H), color=STEEL),
    Box("bath_wall_far", pos=(0.325, 0.0, WALL_Z), size=(WALL_T, 0.20, WALL_H), color=STEEL),
    Box("bath_wall_left", pos=(0.25, 0.095, WALL_Z), size=(0.14, WALL_T, WALL_H), color=STEEL),
    Box("bath_wall_right", pos=(0.25, -0.095, WALL_Z), size=(0.14, WALL_T, WALL_H), color=STEEL),
]

# The water line: a blue ledge on the inside of each side wall, its centre at the line.
WATER = 0.06  # above the table
WATER_LINE = TOP + WATER
LINES = [
    Box(f"water_line_{side}", pos=(0.25, sign * 0.084, WATER_LINE), size=(0.12, 0.012, 0.004), color=BLUE)
    for side, sign in (("left", 1), ("right", -1))
]

# --- the tubes --------------------------------------------------------------------------------------------------------
# 3 cm across and 8 cm tall, capped, 20 g with their sample: lighter than the 35 g of water they displace when stood
# on a seat 5 cm under the line, so they float.
R, TUBE_H, TUBE_MASS = 0.015, 0.08, 0.020
BENCH_X = 0.12


def _tube(name, y):
    return Cylinder(name, pos=(BENCH_X, y, TOP + TUBE_H / 2), radius=R, height=TUBE_H, mass=TUBE_MASS, color=ORANGE)


TUBE_A = _tube("tube_a", 0.15)
TUBE_B = _tube("tube_b", 0.06)
TUBE_C = _tube("tube_c", -0.06)
TUBE_D = _tube("tube_d", -0.15)
TUBES = (TUBE_A, TUBE_B, TUBE_C, TUBE_D)

# --- the rack and the water's push ------------------------------------------------------------------------------------
# Heights in this section are above the table; the rack's own frame has its origin on the table top under the post.
SEAT = 0.010  # where a tube's underside rests when it is pushed fully down
RACK_TOP, GRID_T = 0.050, 0.005  # the grid's top face and thickness
OPEN_HALF = 0.025  # openings 5 cm square: 1 cm of room around a tube
OPENINGS = {"near_left": (-0.04, 0.04), "far_left": (0.04, 0.04), "near_right": (-0.04, -0.04), "far_right": (0.04, -0.04)}
POST_R, POST_TOP = 0.005, 0.105

# Buoyancy on an upright cylinder is linear in how deep its underside is: rho g A per metre. A sprung disc under the
# tube gives the same force: stiffness rho g A, rest position at the water line. The disc's own weight would sag it, so
# its rest is set that much higher and it settles with its top face at the line. Damping stands for the water's drag.
BUOY_K = 1000.0 * 9.81 * np.pi * R * R  # 6.93 N/m
DISC_R, DISC_T, DISC_MASS, DISC_DAMP = 0.022, 0.004, 0.005, 2.5
DISC_SAG = DISC_MASS * 9.81 / BUOY_K
DISC_REST = WATER + DISC_SAG  # top face at joint value 0
DISC_LOW = SEAT - DISC_REST  # top face at the seat: the joint's low stop is the seat
FLOAT_AT = WATER - TUBE_MASS * 9.81 / BUOY_K  # underside of a free-floating tube: 3.17 cm above the table

_g = RACK_TOP - GRID_T / 2
RACK = Mech(
    "rack",
    pos=(BATH_X, BATH_Y, TOP),
    mass=None,
    shapes=[
        # the grid, filling the bath's interior: three strips along y, then the cross pieces that close the four openings
        box((-0.0675, 0, _g), (0.005, 0.17, GRID_T), RACK_GREY),
        box((0.0, 0, _g), (0.03, 0.17, GRID_T), RACK_GREY),
        box((0.0675, 0, _g), (0.005, 0.17, GRID_T), RACK_GREY),
        *[box((sx * 0.04, y, _g), (0.05, w, GRID_T), RACK_GREY)
          for sx in (-1, 1) for y, w in ((-0.075, 0.02), (0.0, 0.03), (0.075, 0.02))],
        # what it stands on: an end plate under each end and a column under the post
        box((0, 0.075, (0.006 + RACK_TOP - GRID_T) / 2), (0.14, 0.02, RACK_TOP - GRID_T - 0.006), RACK_GREY),
        box((0, -0.075, (0.006 + RACK_TOP - GRID_T) / 2), (0.14, 0.02, RACK_TOP - GRID_T - 0.006), RACK_GREY),
        box((0, 0, (0.006 + RACK_TOP - GRID_T) / 2), (0.026, 0.026, RACK_TOP - GRID_T - 0.006), RACK_GREY),
        cyl((0, 0, (RACK_TOP + POST_TOP) / 2), POST_R, POST_TOP - RACK_TOP, STEEL),
    ],
    joints=[
        Joint(
            f"water_{name}",
            "slide",
            axis="z",
            at=(ox, oy, DISC_REST - DISC_T / 2),
            low=DISC_LOW,
            high=0.0,
            spring=BUOY_K,
            damping=DISC_DAMP,
            mass=DISC_MASS,
            shapes=[cyl((0, 0, 0), DISC_R, DISC_T, BLUE)],
        )
        for name, (ox, oy) in OPENINGS.items()
    ],
)
OPENING_RECT = {
    n: Rect((BATH_X + ox - OPEN_HALF, BATH_X + ox + OPEN_HALF), (BATH_Y + oy - OPEN_HALF, BATH_Y + oy + OPEN_HALF))
    for n, (ox, oy) in OPENINGS.items()
}

# --- the hold-down plate ----------------------------------------------------------------------------------------------
# Its own frame: origin at the middle of its underside. A 12 cm square, 8 mm thick, with a 3 cm square hole (1 cm of
# room around the post) and a square collar around the hole to hold it by.
PLATE_HALF, PLATE_T, HOLE_HALF = 0.06, 0.008, 0.015
COLLAR_HALF, COLLAR_H = 0.02, 0.03
PLATE_MASS = 0.18
PLATE_XY = (0.25, 0.21)
_pz, _cz = PLATE_T / 2, PLATE_T + COLLAR_H / 2
_side = (PLATE_HALF + HOLE_HALF) / 2
_wall = (COLLAR_HALF + HOLE_HALF) / 2
PLATE = Mech(
    "hold_down",
    pos=(*PLATE_XY, TOP + 0.0005),
    mass=PLATE_MASS,
    shapes=[
        box((_side, 0, _pz), (PLATE_HALF - HOLE_HALF, 2 * PLATE_HALF, PLATE_T), WHITE),
        box((-_side, 0, _pz), (PLATE_HALF - HOLE_HALF, 2 * PLATE_HALF, PLATE_T), WHITE),
        box((0, _side, _pz), (2 * HOLE_HALF, PLATE_HALF - HOLE_HALF, PLATE_T), WHITE),
        box((0, -_side, _pz), (2 * HOLE_HALF, PLATE_HALF - HOLE_HALF, PLATE_T), WHITE),
        box((0, _wall, _cz), (2 * COLLAR_HALF, COLLAR_HALF - HOLE_HALF, COLLAR_H), WHITE),
        box((0, -_wall, _cz), (2 * COLLAR_HALF, COLLAR_HALF - HOLE_HALF, COLLAR_H), WHITE),
        box((_wall, 0, _cz), (COLLAR_HALF - HOLE_HALF, 2 * HOLE_HALF, COLLAR_H), WHITE),
        box((-_wall, 0, _cz), (COLLAR_HALF - HOLE_HALF, 2 * HOLE_HALF, COLLAR_H), WHITE),
    ],
)

OBJECTS = [*BATH, *LINES, RACK, *TUBES, PLATE]

# --- scoring ----------------------------------------------------------------------------------------------------------
MAX_TILT = 10.0  # degrees: a standing tube
LYING = 45.0  # degrees: past this a tube in the bath is lying in the water
HELD_UP = 0.01  # underside more than this above the water's push under it: still in the hand
DOWN_TOL = 0.004  # a held-down tube's top is within this of its seated height
PLATE_MAX_TILT = 10.0
OFF_TABLE = TOP - 0.05  # anything whose lowest point is below this has left the table


def _opening(s, spec):
    """The opening the tube's whole outline is inside, seen from above, or None."""
    for name, rect in OPENING_RECT.items():
        if inside(s, spec, rect):
            return name
    return None


def _disc_top(rack, name) -> float:
    return TOP + DISC_REST + float(rack.joints.get(f"water_{name}", 0.0))


def _in_rack(s, spec, rack) -> bool:
    """Standing in an opening: whole outline inside it, upright, underside below the grid's top, resting on the water's
    push (not held more than 1 cm above it) and still."""
    name = _opening(s, spec)
    return bool(
        name is not None
        and tilt_deg(s) <= MAX_TILT
        and bottom(s, spec) < TOP + RACK_TOP
        and bottom(s, spec) - _disc_top(rack, name) <= HELD_UP
        and s.resting()
    )


def _plate_on(p, rack) -> bool:
    """The plate is on the post: the post is inside the hole seen from above, the underside's middle is at least 3 mm
    below the post's top, and the plate is within 10 degrees of level and still."""
    post = p.to_local(np.array([rack.origin[0], rack.origin[1], p.origin[2]]))
    return bool(
        abs(post[0]) <= HOLE_HALF
        and abs(post[1]) <= HOLE_HALF
        and p.origin[2] <= TOP + POST_TOP - 0.003
        and tilt_deg(p) <= PLATE_MAX_TILT
        and p.resting()
    )


def _held_down(s, spec) -> bool:
    """The tube's top is no higher than its seated height plus 4 mm."""
    return top(s, spec) <= TOP + SEAT + TUBE_H + DOWN_TOL


def _lying_in_bath(s, spec) -> bool:
    return tilt_deg(s) > LYING and overlaps(s, spec, INTERIOR)


def score(objects):
    """0 if any tube lies in the bath (its cap is in the water), or a tube or the plate has left the table. Otherwise
    0.125 for each tube standing in a rack opening, and 0.125 more for each of those held down under the plate on the
    post."""
    rack, plate = objects["rack"], objects["hold_down"]
    tubes = [(objects[t.name], t) for t in TUBES]
    if any(_lying_in_bath(s, t) or bottom(s, t) < OFF_TABLE for s, t in tubes) or float(plate.low[2]) < OFF_TABLE:
        return 0.0
    fitted = _plate_on(plate, rack)
    total = 0.0
    for s, t in tubes:
        if _in_rack(s, t, rack):
            total += 0.125 + (0.125 if fitted and _held_down(s, t) else 0.0)
    return total


def success(objects):
    """All four tubes in the rack, held down under the plate, everything still."""
    return score(objects) == 1.0


# Checked offline with the URDF; the scorer itself only sees the objects. No arm part may enter the water, or pass below
# the wall tops over the walls or the ring around them. The ring starts at x 0.145, 2.5 cm in front of the near wall, so
# the bench grasps stay 5 mm outside it (a finger reaches x 0.14), and runs 4 cm beyond the other walls. These regions
# stop at the water and the walls rather than at TOP + 0.25, and the ring leaves the interior open, because the job is to
# reach into the bath from above: the hand has to be over it and between the walls, just never in the water. The water
# region runs up to the risen surface: with all four tubes pushed down the water rises from the line to about 6.6 cm.
RING_X = (0.145, 0.37)
RING_Y = 0.14
ARM_KEEP_OUT = [
    {"name": "the water", "x": (INTERIOR.x0, INTERIOR.x1), "y": (INTERIOR.y0, INTERIOR.y1), "z_top": WATER_LINE + 0.006},
    {"name": "the near wall and 2.5 cm before it", "x": (RING_X[0], INTERIOR.x0), "y": (-RING_Y, RING_Y), "z_top": WALL_TOP},
    {"name": "the far wall and 4 cm past it", "x": (INTERIOR.x1, RING_X[1]), "y": (-RING_Y, RING_Y), "z_top": WALL_TOP},
    {"name": "the left wall and 4 cm past it", "x": (INTERIOR.x0, INTERIOR.x1), "y": (INTERIOR.y1, RING_Y), "z_top": WALL_TOP},
    {"name": "the right wall and 4 cm past it", "x": (INTERIOR.x0, INTERIOR.x1), "y": (-RING_Y, INTERIOR.y0), "z_top": WALL_TOP},
]

# --- the demo ---------------------------------------------------------------------------------------------------------
# It reads no simulator object state: only the fixed layout above and the arms' own joint encoders. The joint drives
# trail their commands by 0.06 s (shoulder) to 0.15 s (wrist) and joint friction holds part of that error, so a hand
# moving straight down runs several millimetres to the side of its line, and a pad that strikes a tube before the grip
# closes knocks it over. Light tubes also tip rather than slide when pushed high up. Hence: before every descent, grasp
# and release the hand waits half a second and is then trimmed until the encoders put the tool point within 1.5 mm of
# where it should be; on the last stretch down onto the rack the encoders hold the hand on its line step by step;
# fingers open to 8 cm at the bench (2.5 cm each side of a tube) and only 5 cm over the tubes in the bath; slow
# descents, set-downs and first centimetres of retreat; a pause to settle before each close and each release; a
# two-stage close, quick to 2.5 mm from the surface and then slow; and one arm at a time, the other parked, because the
# hands' link4s ride ahead of them and come within 2 cm of each other when both hands work over the bench.
# Grip along y. A tube is gripped 1.5 cm below its top and let go at its floating height, so nothing pops up or drops:
# the pad tips stay 2.4 cm above the water line. Each arm takes its inner tube to the far opening first and its outer
# tube to the near one, so a carried tube never passes over a floating one or the post.
PICK_OPEN = -0.04
GRIP_OPEN = -0.025
PLATE_OPEN = -0.035  # 1.5 cm each side of the collar when the plate is let go: nothing else is that high
TUBE_SQUEEZE = 0.003  # finger target this far inside a tube's surface: about 15 N a side. Harder, and the pads slowly
# squeeze a round tube out sideways along themselves (7 mm at 30 N in a contact replay, 2 mm at 15 N).
PLATE_SQUEEZE = 0.006  # about 30 N a side on the collar's flat faces
NEAR_SURFACE = 0.0025  # where the quick part of the close stops
TUBE_GRIP_BELOW_TOP = 0.015
TRAVEL = TOP + 0.155  # a carried tube's underside clears the walls by 2 cm
TUBE_RELEASE = TOP + FLOAT_AT + TUBE_H - TUBE_GRIP_BELOW_TOP  # TCP height with the tube at its floating height
TUBE_ABOVE = TOP + WATER + 0.01 + TUBE_H - TUBE_GRIP_BELOW_TOP  # TCP height with the tube's underside 1 cm above the water line
PADS_CLEAR_TUBE = TOP + FLOAT_AT + TUBE_H + 0.01 + 0.013  # TCP height at which the pad tips are 1 cm above a floating tube
COLLAR_GRIP_BELOW_TOP = 0.012
PLATE_GRIP = TOP + 0.0005 + PLATE_T + COLLAR_H - COLLAR_GRIP_BELOW_TOP  # pad tips 5 mm above the plate's top face
PLATE_TRAVEL = TOP + 0.165  # the carried plate's underside is 2.7 cm above the floating caps and 3.4 cm above the post
PLATE_RELEASE_GAP = 0.004  # let go with the underside this far above the seated caps
PLATE_RELEASE = TOP + SEAT + TUBE_H + PLATE_RELEASE_GAP + PLATE_T + COLLAR_H - COLLAR_GRIP_BELOW_TOP
PLATE_ABOVE = PLATE_RELEASE + (FLOAT_AT - SEAT) + 0.01  # TCP height with the plate's underside about 1.4 cm above the floating caps
PADS_CLEAR_PLATE = PLATE_RELEASE + COLLAR_GRIP_BELOW_TOP + 0.01 + 0.013
PARK = (0.12, 0.25, TOP + 0.16)
PEAK_SPEED = {"descend": 0.10, "lift": 0.10, "carry": 0.15, "approach": 0.10, "lower": 0.03, "retreat": 0.05}  # m/s, TCP; 0.20 elsewhere
MAX_SPEED = 0.20  # m/s: the fastest any TCP goes
SETTLE_S = 0.4  # s: the hand stops before the fingers move
AT_LEAST = {"pre-close": 0.6, "close": 1.2, "release": 1.0}  # s, whatever the distance
READY_TCP = {"left": (0.323, 0.305, 1.008), "right": (0.323, -0.305, 1.008)}
TRIM_AFTER = ("to the object", "descend", "carry", "approach", "lower")  # where the encoders trim the hand onto its target
FINGERS_ONLY = ("settle", "pre-close", "close", "release")  # the hand stays where the trim left it
KEEP_TRIM = ("descend", "lift", "approach", "lower", "retreat")  # straight up or down from a trimmed spot: keep its correction
TRIM_WAIT_S, TRIM_TOL, TRIM_S = 0.5, 0.0015, 1.5  # let the lag die away first, then trim what is left
GUIDED = ("approach", "lower")  # straight down onto the rack: the encoders hold the hand on its line all the way
GUIDE_GAIN, GUIDE_STEP, GUIDE_MAX = 0.2, 0.001, 0.02  # per control step: share of the sideways error, most per step, most in all


def _spot(name):
    ox, oy = OPENINGS[name]
    return (BATH_X + ox, BATH_Y + oy)


def _timed(waypoints, start):
    """Pace each move by its length: a smoothstep peaks at 1.5 times its average speed."""
    out, prev = [], np.asarray(start, float)
    for w in waypoints:
        pos = np.asarray(w.pos, float)
        need = 1.5 * float(np.linalg.norm(pos - prev)) / PEAK_SPEED.get(w.label, MAX_SPEED)
        out.append(w if w.label == "settle" else replace(w, duration=max(need, AT_LEAST.get(w.label, 0.5))))
        prev = pos
    return out


def _move(grasp, place, half_width, squeeze, travel, above, pads_clear, start, let_go=GRIP_OPEN):
    """One object from ``grasp`` to ``place`` (TCP positions): fingers wide at the bench, a settle before each close and
    release, a stop at ``above`` (just clear of first contact) before a slow last stretch down, and a slow start to the
    retreat while the pads are beside what they let go."""
    hold, near = -(half_width - squeeze), -(half_width + NEAR_SURFACE)
    plan = []
    for w in pick_place_waypoints(grasp, place, travel_z=travel):
        if w.label in ("to the object", "descend"):
            w = replace(w, finger=PICK_OPEN)
        elif w.label in ("release", "retreat"):
            w = replace(w, finger=let_go)
        else:
            w = replace(w, finger=hold)
        if w.label == "retreat":
            plan += [replace(w, pos=(*place[:2], pads_clear)), replace(w, label="clear the bath")]
            continue
        if w.label == "lower":
            plan.append(replace(w, pos=(*place[:2], above), label="approach"))
        if w.label in ("close", "release"):
            plan.append(replace(plan[-1], duration=SETTLE_S, label="settle"))
        if w.label == "close":
            plan.append(replace(w, finger=near, label="pre-close"))
        plan.append(w)
    return _timed(plan, start)


def _tube_move(tube, opening, start):
    grasp = (*tube.pos[:2], tube.pos[2] + tube.height / 2 - TUBE_GRIP_BELOW_TOP)
    return _move(grasp, (*_spot(opening), TUBE_RELEASE), tube.radius, TUBE_SQUEEZE, TRAVEL, TUBE_ABOVE, PADS_CLEAR_TUBE,
                 start)


def _park_at(side):
    return (PARK[0], (1 if side == "left" else -1) * PARK[1], PARK[2])


def _park(side, finger, start):
    return _timed([Waypoint(0, _park_at(side), finger, "park")], start)


def _tubes(side, start):
    """One arm's two tubes: the inner one to the far opening, then the outer one to the near opening, then park."""
    inner, outer = (TUBE_B, TUBE_A) if side == "left" else (TUBE_C, TUBE_D)
    plan = _tube_move(inner, f"far_{side}", start)
    plan += _tube_move(outer, f"near_{side}", plan[-1].pos)
    return plan + _park(side, GRIP_OPEN, plan[-1].pos)


def _plate(start):
    """The left arm: the plate from the bench onto the post, then park."""
    plan = _move((*PLATE_XY, PLATE_GRIP), (BATH_X, BATH_Y, PLATE_RELEASE), COLLAR_HALF, PLATE_SQUEEZE,
                 PLATE_TRAVEL, PLATE_ABOVE, PADS_CLEAR_PLATE, start, let_go=PLATE_OPEN)
    return plan + _park("left", PLATE_OPEN, plan[-1].pos)


def plans():
    """The demo's waypoints, in the order they are played: (arm, waypoints)."""
    return [
        ("left", _tubes("left", READY_TCP["left"])),
        ("right", _park("right", PICK_OPEN, READY_TCP["right"])),
        ("right", _tubes("right", _park_at("right"))),
        ("left", _plate(_park_at("left"))),
    ]


def _guided(arm, w):
    """Straight down to ``w.pos``. Joint friction pulls the hand a few millimetres off its line whenever it moves down, so
    each step the commanded line is shifted by a share of the sideways error the encoders report."""
    start, goal = arm.pos.copy(), np.asarray(w.pos, float)
    steps = max(1, round(w.duration / arm.dt))
    xy = start[:2].copy()
    for i in range(1, steps + 1):
        u = i / steps
        xy = xy + np.clip(GUIDE_GAIN * (goal[:2] - arm.measured_pos[:2]), -GUIDE_STEP, GUIDE_STEP)
        xy = goal[:2] + np.clip(xy - goal[:2], -GUIDE_MAX, GUIDE_MAX)
        z = start[2] + u * u * (3 - 2 * u) * (goal[2] - start[2])
        yield from arm.move((xy[0], xy[1], z), arm.dt, w.finger)


def _play(arm, waypoints):
    """Play waypoints on one arm, trimming the hand onto its target with the arm's own encoders where it matters."""
    keep = np.zeros(3)
    for w in waypoints:
        if w.label in FINGERS_ONLY:
            yield from arm.grip(w.finger, w.duration)
            continue
        if w.label not in KEEP_TRIM:
            keep = np.zeros(3)
        if w.label in GUIDED:
            yield from _guided(arm, w)
        else:
            yield from arm.move(np.asarray(w.pos, float) + keep, w.duration, w.finger)
        if w.label in TRIM_AFTER:
            yield from arm.hold(TRIM_WAIT_S)
            yield from arm.trim_to_measured(w.pos, tol=TRIM_TOL, max_duration=TRIM_S)
            keep = arm.pos - np.asarray(w.pos, float)
            keep[2] = 0.0  # the sideways part carries over to the next straight move; height is trimmed afresh


demo = placeholder_demo


def demo_script(ctx):
    """One arm at a time, the other parked beside the bench. The left arm stands its two tubes in the left openings
    and parks while the right arm moves to its park; the right arm stands its two in the right openings and parks; all
    four float. Then the left arm takes the hold-down plate by its collar, lowers it over the post onto the caps, lets
    go and parks."""
    arms = {"left": ctx.left, "right": ctx.right}
    (_, left_tubes), (_, right_park), (_, right_tubes), (_, plate) = plans()
    yield from ctx.both(_play(arms["left"], left_tubes), _play(arms["right"], right_park))
    yield from _play(arms["right"], right_tubes)
    yield from _play(arms["left"], plate)
