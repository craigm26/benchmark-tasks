"""Post a boxed paperback through a vertical door mail slot.

A section of an exterior door (45 mm leaf) stands across the table in front of the right arm. A vertical brass mail
slot (Salsbury 4085 size: 89 x 279 mm plate, 76 x 210 mm cut through the door) is set in it, with a spring flap behind
the plate that swings into the hall. A white mailer box (Uline S-16530 size, 229 x 152 x 25 mm, 0.15 kg with a
paperback inside) stands on its long edge on the table in front of the door. The robot must lift it, feed it edge-on
through the 41 mm plate opening against the flap, let go once it rests on the slot floor, and push the rest of it in
with its fingertips until it tips off the inside edge and drops into the hall behind the door, where the flap closes.

Requires -E object_jitter=0 (the slot and the mailer have one fixed layout).

Run it:  inspect-robots run --embodiment yam_arena -E task=post_a_book_bundle_through_a_vertical_door_mail_slot.py \\
           -E object_jitter=0 --instruction "Post the boxed book through the door's mail slot so it drops into the
           hall behind the door, then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, on_reset, rod  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the demo has parked by then
SCORE_OVER = "end"

OAK_DOOR = (0.36, 0.20, 0.10)
BRASS = (0.83, 0.66, 0.22)
BRASS_DARK = (0.62, 0.47, 0.13)
MAILER_WHITE = (0.95, 0.95, 0.92)

# ---------------------------------------------------------------------------------------------------------------------
# The door section and the slot. x forward from the robot, y left, z up; all numbers in metres.
SLOT_Y = -0.060  # slot centre line
SLOT_FLOOR = TOP + 0.125  # floor of the plate opening and of the cut through the door (flush)
DOOR_FRONT, DOOR_BACK = 0.370, 0.415  # 45 mm leaf (1-3/4 in exterior door)
DOOR_MID = (DOOR_FRONT + DOOR_BACK) / 2
DOOR_Y = (-0.210, 0.090)  # 300 mm leaf section
DOOR_TOP = TOP + 0.370
CUT_W, CUT_H = 0.076, 0.210  # Salsbury 4085 rough opening, 3 x 8-1/4 in
PLATE_T = 0.008
PLATE_FRONT = DOOR_FRONT - PLATE_T  # 0.362
PLATE_W, PLATE_H = 0.089, 0.279  # 3-1/2 x 11 in overall
OPEN_W, OPEN_H = 0.041, 0.191  # the plate's letter opening (inside the 44 x 197 mm flap)
PLATE_BOTTOM = SLOT_FLOOR - (PLATE_H - OPEN_H) / 2  # plate centred on its opening
FLAP_W, FLAP_H, FLAP_T = 0.044, 0.197, 0.002  # catalog flap 1-3/4 x 7-3/4 in, 2 mm brass
FLAP_SPRING = 0.020  # N m/rad: 0.031 N m fully open, about 1 N at the mailer face
FLAP_OPEN_Q, FLAP_CLOSED_Q = 0.9, 0.15


def _box_from(x0, x1, y0, y1, z0, z1, color):
    """A box between world-frame limits, in the door Mech's frame (origin at x 0, y 0, table top)."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2 - TOP), (x1 - x0, y1 - y0, z1 - z0), color)


_cy0, _cy1 = SLOT_Y - CUT_W / 2, SLOT_Y + CUT_W / 2
_oy0, _oy1 = SLOT_Y - OPEN_W / 2, SLOT_Y + OPEN_W / 2
_py0, _py1 = SLOT_Y - PLATE_W / 2, SLOT_Y + PLATE_W / 2
_pz0, _pz1 = PLATE_BOTTOM, PLATE_BOTTOM + PLATE_H
_oz1 = SLOT_FLOOR + OPEN_H

DOOR_SHAPES = [
    # oak leaf around the 76 x 210 mm cut
    _box_from(DOOR_FRONT, DOOR_BACK, DOOR_Y[0], _cy0, TOP, DOOR_TOP, OAK_DOOR),
    _box_from(DOOR_FRONT, DOOR_BACK, _cy1, DOOR_Y[1], TOP, DOOR_TOP, OAK_DOOR),
    _box_from(DOOR_FRONT, DOOR_BACK, _cy0, _cy1, TOP, SLOT_FLOOR, OAK_DOOR),
    _box_from(DOOR_FRONT, DOOR_BACK, _cy0, _cy1, SLOT_FLOOR + CUT_H, DOOR_TOP, OAK_DOOR),
    # brass plate: four rails round the 41 x 191 mm opening
    _box_from(PLATE_FRONT, DOOR_FRONT, _py0, _oy0, _pz0, _pz1, BRASS),
    _box_from(PLATE_FRONT, DOOR_FRONT, _oy1, _py1, _pz0, _pz1, BRASS),
    _box_from(PLATE_FRONT, DOOR_FRONT, _oy0, _oy1, _pz0, SLOT_FLOOR, BRASS),
    _box_from(PLATE_FRONT, DOOR_FRONT, _oy0, _oy1, _oz1, _pz1, BRASS),
]
FLAP_HINGE = (DOOR_FRONT + 0.0005 + FLAP_T / 2, SLOT_Y + FLAP_W / 2, SLOT_FLOOR + 0.003 + FLAP_H / 2)
FLAP_JOINT = Joint(
    "flap", "hinge", axis="z", at=(FLAP_HINGE[0], FLAP_HINGE[1], FLAP_HINGE[2] - TOP), low=0.0, high=1.6,
    spring=FLAP_SPRING, damping=0.004, mass=0.15, friction=0.3,
    shapes=[box((0.0, -FLAP_W / 2, 0.0), (FLAP_T, FLAP_W - 0.002, FLAP_H), BRASS_DARK),
            rod((0.0, 0.0, -FLAP_H / 2), (0.0, 0.0, FLAP_H / 2), 0.0015, BRASS_DARK)],
)
DOOR = Mech("door", pos=(0.0, 0.0, TOP), mass=None, shapes=DOOR_SHAPES, joints=[FLAP_JOINT], friction=0.3)

# ---------------------------------------------------------------------------------------------------------------------
# The mailer: Uline S-16530 white 9 x 6 x 1 in mailer with a paperback inside, 0.15 kg, standing on its long edge.
MAILER_L, MAILER_T, MAILER_H = 0.229, 0.025, 0.152
MAILER_MASS = 0.15
MAILER_START = (0.215, SLOT_Y, TOP + MAILER_H / 2)
MAILER = Mech("mailer", pos=MAILER_START, mass=MAILER_MASS, friction=0.35,
              shapes=[box((0.0, 0.0, 0.0), (MAILER_L, MAILER_T, MAILER_H), MAILER_WHITE)])

OBJECTS = [DOOR, MAILER]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
RELEASE_CLEAR = 0.03
RELEASE_STEPS = 20  # 1 s at 20 Hz
DROPPED_BELOW = SLOT_FLOOR - 0.010  # the mailer's lowest point is under the slot floor: it has left the lip
BEHIND_X = DOOR_BACK - 0.002  # every corner past the door's back face (2 mm for the box corners' rounding)
PAST_FLAP_X = DOOR_FRONT  # every corner behind the door front: past the plate and the flap line
_ST: dict = {}
FORCE_RELEASED: list = [None]  # offline tests only: overrides the release check (None: live check, True offline)


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, prev=None, entered=False, flap_peak=0.0, through_wrong=False, rel_steps=0, steps=0)


_reset()


def mailer_corners(m) -> np.ndarray:
    return belimits.corners(m, MAILER)


def in_window(p) -> bool:
    """A point inside the plate's letter opening, seen along x (|y - slot| within 20.5 mm, z on the floor to the top)."""
    return bool(abs(float(p[1]) - SLOT_Y) <= OPEN_W / 2 and SLOT_FLOOR - 0.002 <= float(p[2]) <= _oz1)


def in_opening(p, tol: float = 0.002) -> bool:
    """A corner inside the plate's letter opening (41 x 191 mm, 2 mm allowed for contact) seen along x."""
    return bool(abs(float(p[1]) - SLOT_Y) <= OPEN_W / 2 + tol and SLOT_FLOOR - tol <= float(p[2]) <= _oz1 + tol)


def _released(objects) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    if mechkit._ENV.get("env") is None:  # offline replays cannot see the fingers; the simulator run checks it
        return True
    return bool(mechkit.released(objects["mailer"], clearance=RELEASE_CLEAR))


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["steps"] += 1
    m, door = objects["mailer"], objects["door"]
    c = np.asarray(m.pos, float)
    flap = float(door.joints.get("flap", 0.0))
    pts = mailer_corners(m)
    lead = pts[pts[:, 0] > PLATE_FRONT]
    if not _ST["entered"] and len(lead) and all(in_opening(p) for p in lead):
        _ST["entered"] = True  # first corners past the plate front all inside the letter opening
    if _ST["entered"] and float(pts[:, 0].max()) >= DOOR_FRONT:
        _ST["flap_peak"] = max(_ST["flap_peak"], flap)  # flap travel counts only while the box is through the plate
    prev = _ST["prev"]
    if prev is not None and (prev[0] - DOOR_MID) * (c[0] - DOOR_MID) <= 0.0 and prev[0] != c[0]:
        s = (DOOR_MID - prev[0]) / (c[0] - prev[0])
        if not in_window(prev + s * (c - prev)):
            _ST["through_wrong"] = True  # over or round the door, not through the slot
    _ST["prev"] = c
    _ST["rel_steps"] = _ST["rel_steps"] + 1 if _released(objects) else 0


def flags(objects) -> dict:
    _update(objects)
    m, door = objects["mailer"], objects["door"]
    pts = mailer_corners(m)
    flap = float(door.joints.get("flap", 0.0))
    past_flap = bool(pts[:, 0].min() >= PAST_FLAP_X)
    posted = bool(pts[:, 0].min() >= BEHIND_X and pts[:, 2].min() <= DROPPED_BELOW and flap <= FLAP_CLOSED_Q
                  and m.resting() and door.resting(max_spin=0.5))
    return {"A": _ST["entered"], "B": _ST["flap_peak"] >= FLAP_OPEN_Q, "C": past_flap, "D": posted,
            "wrong": _ST["through_wrong"], "released": _ST["rel_steps"] >= RELEASE_STEPS}


def score(objects):
    """0 if the mailer's centre ever crossed the door's mid plane outside the slot opening (over or round the door),
    or if no corner ever passed the plate front inside the letter opening (the first corners past x 0.362 must all be
    inside the 41 x 191 mm opening). Otherwise: 0.15 it entered the opening; 0.35 and the flap swung open 0.9 rad
    while the mailer was past the flap line (a finger poking the flap earns nothing); 0.60 and it ends
    wholly behind the door's front face (past the flap); 1.00 it ends wholly behind the door's back face, below the
    slot floor, still, with the flap back within 0.15 rad of closed. Without a release (both hands 3 cm clear for the
    last 1 s) at most 0.35."""
    f = flags(objects)
    if f["wrong"] or not f["A"]:
        return 0.0
    if f["D"] and f["B"]:
        s = 1.0
    elif f["C"] and f["B"]:
        s = 0.60
    elif f["B"]:
        s = 0.35
    else:
        s = 0.15
    return s if f["released"] else min(s, 0.35)


def success(objects):
    return bool(score(objects) >= 1.0)


# ---------------------------------------------------------------------------------------------------------------------
# Where no part of either arm may go (checked offline by sweep.py; the gate envelope holds the same line for the tool
# points): the door leaf and plate, 4 mm in front of the plate, and the hall behind it, up to above the door top.
ARM_KEEP_OUT = [
    {"name": "door, mail-slot plate and the hall behind", "x": (PLATE_FRONT - 0.004, 0.75), "y": DOOR_Y,
     "z_top": DOOR_TOP + 0.03},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (right arm only; the left arm holds READY).
PITCH = 0.6  # fingers leaned 34 deg toward the door: everything but the fingertips stays behind the tool point
GRIP_BACK = 0.033  # pads 33 mm behind the mailer's centre: 0.15 kg x 9.81 x 0.033 = 0.049 N m about the pad line
GRIP_X = MAILER_START[0] - GRIP_BACK
GRIP_Z = TOP + MAILER_H - 0.016  # tool point 16 mm under the mailer's top edge
LIFT = (SLOT_FLOOR - TOP) + 0.010  # mailer bottom 10 mm above the slot floor
HAND_STOP_X = 0.345  # tool point limit; leaned fingertips reach 12 mm further, 5 mm short of the plate
TAIL_GRIP = 0.015  # second grip: pads 15 mm in from the trailing end, so the hand can slide it 66 mm further in
LOOSE_GAP = 0.045  # pads eased to 45 mm round the 25 mm box while the hand slides back along it (10 mm a side)
TAIL_PLAN = HAND_STOP_X + GRIP_BACK - MAILER_L / 2  # where the trailing end rests after the set-down (0.264)
FINAL_TAIL = HAND_STOP_X - TAIL_GRIP  # trailing end after the held slide (0.330): centre 0.4445, 30 mm past the edge
SLIDE_S = 2.5  # 66 mm held slide, about 3 cm/s
DEMO_LOOKS: dict = {}

demo = placeholder_demo


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / 0.094


def _tail(state):
    """Centre of the mailer's trailing end face (the four corners nearest the robot)."""
    pts = belimits.corners(state, MAILER)
    near = pts[np.argsort(pts[:, 0])[:4]]
    return near.mean(axis=0)


def _posted(state) -> bool:
    pts = belimits.corners(state, MAILER)
    return bool(pts[:, 0].min() >= BEHIND_X and pts[:, 2].min() <= DROPPED_BELOW)


def _slide_in_and_let_go(ctx, R, y):
    """Hold the mailer by its trailing end and slide it along the slot floor until the tool point is at HAND_STOP_X
    (its centre 30 mm past the inside edge), then open while rising 25 mm so the tail is free to lift as it tips."""
    yield from R.move((HAND_STOP_X, y, R.pos[2]), SLIDE_S, pitch=PITCH)
    yield from R.hold(0.4)
    yield from R.move((R.pos[0] - 0.004, R.pos[1], R.pos[2] + 0.025), 0.8, FINGER_OPEN, pitch=PITCH)
    yield from R.move((R.pos[0] - 0.03, R.pos[1], R.pos[2] + 0.02), 1.0, pitch=PITCH)
    yield from R.move((R.pos[0] - 0.05, R.pos[1], R.pos[2]), 1.0, pitch=PITCH)


def demo_script(ctx):
    """Grip the standing mailer across its 25 mm thickness, 33 mm behind its centre; lift it 10 mm above the slot
    floor; carry it straight along x through the plate opening (the flap swings in) until the hand is 17 mm short of
    the plate and set it on the slot floor. Ease the pads to 45 mm (they still box the mailer in), slide the hand back
    to 15 mm from its trailing end, close again and slide the mailer on along the slot floor, held, until its centre
    is 30 mm past the inside edge; open while rising, and it tips off the edge into the hall. If a later look shows it
    still on the slot floor, grip its trailing end from above and slide it again. Reads the mailer's pose to find its
    trailing end before each regrasp and once at the end to confirm the drop."""
    R = ctx.right
    y = SLOT_Y
    above = (GRIP_X, y, GRIP_Z + 0.08)
    yield from R.move(above, 2.0, FINGER_OPEN, pitch=PITCH)
    yield from R.trim_to_measured(above)
    for attempt in range(2):
        yield from R.move((GRIP_X, y, GRIP_Z - 0.004 * attempt), 1.2, FINGER_OPEN, pitch=PITCH)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.020):
            break
        yield from R.grip(FINGER_OPEN, 0.5)
    else:
        yield from R.move(above, 1.0, FINGER_OPEN, pitch=PITCH)
        yield from R.home()
        return
    gz = R.pos[2]
    lifted = (GRIP_X, y, gz + LIFT)
    yield from R.move(lifted, 1.5, pitch=PITCH)
    yield from R.trim_to_measured(lifted)
    dy = R.pos[1] - lifted[1]  # trim offset carried into the later moves
    # carry through the opening at about 5 cm/s, y held on the slot line
    carry_z = R.pos[2]
    yield from R.move((HAND_STOP_X, y + dy, carry_z), 3.5, pitch=PITCH)
    yield from R.hold(0.5)
    # set it down on the slot floor; ease the pads, slide back to its trailing end and grip it again
    yield from R.move((R.pos[0], R.pos[1], R.pos[2] - 0.011), 0.8, pitch=PITCH)
    yield from R.hold(0.3)
    grip_z = R.pos[2]
    for attempt in range(2):
        yield from R.grip(_finger(LOOSE_GAP), 0.6)
        yield from R.hold(0.3)
        tail = _tail(ctx.obj("mailer"))
        tx = min(max(float(tail[0]), TAIL_PLAN - 0.02), TAIL_PLAN + 0.02)  # offline the read is the start pose
        ty = min(max(float(tail[1]), SLOT_Y - 0.010), SLOT_Y + 0.010)
        yield from R.move((tx + TAIL_GRIP, ty + dy, grip_z), 1.5, pitch=PITCH)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.020):
            break
    yield from _slide_in_and_let_go(ctx, R, y + dy)
    yield from ctx.wait(1.0)
    # look once: still on the slot floor? then grip its trailing end from above and slide it again
    m = ctx.obj("mailer")
    if not _posted(m):
        tail = _tail(m)
        tx = min(max(float(tail[0]), TAIL_PLAN - 0.02), FINAL_TAIL)
        ty = min(max(float(tail[1]), SLOT_Y - 0.010), SLOT_Y + 0.010)
        hover = (tx + TAIL_GRIP, ty + dy, grip_z + 0.045)
        yield from R.move(hover, 1.5, FINGER_OPEN, pitch=PITCH)
        yield from R.move((hover[0], hover[1], grip_z), 1.2, FINGER_OPEN, pitch=PITCH)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.020):
            yield from _slide_in_and_let_go(ctx, R, y + dy)
        else:
            yield from R.grip(FINGER_OPEN, 0.5)
            yield from R.move((R.pos[0] - 0.04, R.pos[1], R.pos[2] + 0.04), 1.0, pitch=PITCH)
    yield from R.move((R.pos[0], R.pos[1], max(R.pos[2], GRIP_Z + 0.12)), 1.0, FINGER_OPEN, pitch=PITCH)
    yield from R.home()
    yield from ctx.wait(1.5)
