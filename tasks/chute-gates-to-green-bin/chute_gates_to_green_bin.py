"""On the station table: a short plywood chute that slopes down toward the arms, 22 cm wide, with a closed hopper at its
high end holding three yellow wooden balls behind an orange slide gate, and two grey diverter gates (each a tilted bar with
an upright handle outside the chute wall) across the chute below it. At the low end the chute feeds three bins side by
side: grey, grey and green. The hopper's roof has a 2 cm slot that shows the balls but is too narrow for one to pass. The balls are loose; the chute, gates and bins are a fixed fixture with sliding parts.

The two grey gates start as follows: the left one (handle on the left side) sits across the chute and sends the balls to
the left-hand grey bin; the right one (handle on the right side) is pulled out to the right and sends nothing. The
orange slide gate is spring-loaded shut and opens only while its handle is held pulled out to the left.

Task: Set the two grey chute gates so the balls run into the green bin, then pull the orange slide gate open and hold it
until all three have left the hopper.

Requires -E object_jitter=0 and -E control_hz=20.

Run it:  inspect-robots run --embodiment yam_arena -E task=chute_gates_to_green_bin.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Set the two grey chute gates so the balls run into the green bin, then pull the orange slide
           gate open and hold it until all three have left the hopper." --scorer task_score --max-steps 2400 ...
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from belimits import Rect, ResetOnJumpTracker, overlaps  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
# success() is true from the moment the last ball settles with the slide gate shut, while the hand may still be at the
# handle. Holding it this long lets the demo open the gripper and lift clear on camera.
SUCCESS_HOLD_S = 12.0  # the episode may end only after the demo has let go and moved clear

# -- the chute: a ramp tilted 8 degrees about y, rising toward +x (away from the arms) --------------------------------------
THETA = math.radians(8.0)
CT, ST = math.cos(THETA), math.sin(THETA)
X0, Z0 = 0.13, 0.042  # the ramp's top surface at its low end (x, height above the table)
RAMP_X = (0.13, 0.50)
RAMP_Y = 0.115  # half width
WALL_Y = (0.10, 0.11)  # side walls: inner face at |y| = 0.10
WALL_H = 0.05  # walls, dividers and bars stand 5 cm over the ramp
R_BALL = 0.0125

DARK, WHITE = (0.16, 0.16, 0.19), (0.93, 0.93, 0.96)
GREY, HAND_GREY = (0.62, 0.64, 0.68), (0.74, 0.76, 0.80)
ORANGE, BLUE = (0.95, 0.48, 0.05), (0.18, 0.30, 0.55)
GREY_BIN, GREEN_BIN, YELLOW = (0.50, 0.51, 0.54), (0.08, 0.70, 0.22), (0.97, 0.86, 0.08)


def rz(x: float) -> float:
    """Height of the ramp's top surface above the table at surface position x."""
    return Z0 + math.tan(THETA) * (x - X0)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def qy(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


def qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def point(x, y, w):
    """Mech-frame (x, y, z) of the point at surface position x, lateral y and height w above the ramp's surface."""
    u = (x - X0) / CT
    return (X0 + u * CT - w * ST, y, Z0 + u * ST + w * CT)


def rbox(x0, x1, y0, y1, w0, w1, color):
    """A box lying on the ramp: surface x range, y range and height range above the ramp surface."""
    cx, cy, cw = (x0 + x1) / 2, (y0 + y1) / 2, (w0 + w1) / 2
    size = ((x1 - x0) / CT, y1 - y0, w1 - w0)
    return box(point(cx, cy, cw), size, color, qy(-THETA))


def rbar(p, q, thick, w0, w1, color, shift=(0.0, 0.0)):
    """A bar on the ramp from surface point p=(x, y) to q=(x, y), turned across the chute."""
    (px, py), (qx, qy_) = p, q
    cx, cy = (px + qx) / 2 + shift[0], (py + qy_) / 2 + shift[1]
    length = math.hypot(qx - px, qy_ - py)
    psi = math.atan2(qy_ - py, qx - px)
    return box(point(cx, cy, (w0 + w1) / 2), (length, thick, w1 - w0), color, qmul(qy(-THETA), qz(psi)))


def lbox(x0, x1, y0, y1, z0, z1, color):
    """A level box in the Mech frame (heights above the table), for handles, rods and bin walls."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def rel(shapes, at):
    """The same shapes in a joint's own frame, whose origin sits at ``at``."""
    return tuple(type(s)(s.kind, tuple(np.subtract(s.at, at)), s.size, s.color, s.tilt) for s in shapes)


# fixed parts ---------------------------------------------------------------------------------------------------------------
BAR_T, BAR_W0, BAR_W1 = 0.012, 0.012, WALL_H  # gate bars: 1.2 cm thick, 1.2 cm over the ramp (they pass over the low rails), 5 cm up
RAIL_H = 0.010  # a low rail stands in the wall gaps, 2 mm under the bars
MOUTH = 0.395  # the hopper's mouth (x); its interior is 8 cm deep, 5 cm wide, 3.5 cm tall
HOPPER_X = (MOUTH, 0.485)
FIXED = [
    rbox(RAMP_X[0], RAMP_X[1], -RAMP_Y, RAMP_Y, -0.01, 0.0, DARK),  # the ramp, 1 cm thick
    lbox(0.125, 0.135, -RAMP_Y, RAMP_Y, 0.0, 0.037, DARK),  # its low-end face, to the table
]
for lx in (0.15, 0.49):  # four legs under the ramp
    for ly in (-0.10, 0.10):
        FIXED.append(lbox(lx - 0.01, lx + 0.01, ly - 0.01, ly + 0.01, 0.0, rz(lx) - 0.0105 - 0.001, DARK))
# side walls, open where the gate bars and handles pass through them
FIXED += [
    rbox(0.13, 0.268, WALL_Y[0], WALL_Y[1], 0.0, WALL_H, WHITE),  # +y wall
    rbox(0.41, RAMP_X[1], WALL_Y[0], WALL_Y[1], 0.0, WALL_H, WHITE),
    rbox(0.13, 0.185, -WALL_Y[1], -WALL_Y[0], 0.0, WALL_H, WHITE),  # -y wall
    rbox(0.295, RAMP_X[1], -WALL_Y[1], -WALL_Y[0], 0.0, WALL_H, WHITE),
    rbox(0.268, 0.41, WALL_Y[0], WALL_Y[1], 0.0, RAIL_H, WHITE),  # low rails under the bars and handles
    rbox(0.185, 0.295, -WALL_Y[1], -WALL_Y[0], 0.0, RAIL_H, WHITE),
    rbox(0.13, 0.18, 0.03, 0.04, 0.0, WALL_H, WHITE),  # lane dividers
    rbox(0.13, 0.18, -0.04, -0.03, 0.0, WALL_H, WHITE),
]
# the hopper: a closed roofed box on the ramp, open only at its mouth
FIXED += [
    rbox(HOPPER_X[0], HOPPER_X[1], 0.025, 0.035, 0.0, 0.045, BLUE),
    rbox(HOPPER_X[0], HOPPER_X[1], -0.035, -0.025, 0.0, 0.045, BLUE),
    rbox(HOPPER_X[0], HOPPER_X[1], 0.01, 0.035, 0.035, 0.045, BLUE),  # roof, with a 2 cm slot along it: it shows the
    rbox(HOPPER_X[0], HOPPER_X[1], -0.035, -0.01, 0.035, 0.045, BLUE),  # balls from above but is too narrow for one to pass
    rbox(HOPPER_X[1] - 0.01, HOPPER_X[1], -0.035, 0.035, 0.0, 0.045, BLUE),  # back
]
# the three bins on the table, one per lane; walls 7 cm tall
BIN_X = (0.05, 0.13)
BIN_H = 0.07
FIXED += [
    lbox(0.04, 0.05, -0.11, 0.11, 0.0, BIN_H, WHITE),  # back wall
    lbox(BIN_X[0], BIN_X[1], 0.03, 0.04, 0.0, BIN_H, WHITE),
    lbox(BIN_X[0], BIN_X[1], -0.04, -0.03, 0.0, BIN_H, WHITE),
    lbox(BIN_X[0], BIN_X[1], 0.10, 0.11, 0.0, BIN_H, WHITE),
    lbox(BIN_X[0], BIN_X[1], -0.11, -0.10, 0.0, BIN_H, WHITE),
    lbox(BIN_X[0], BIN_X[1], 0.04, 0.10, 0.0, 0.002, GREY_BIN),  # floors
    lbox(BIN_X[0], BIN_X[1], -0.03, 0.03, 0.0, 0.002, GREY_BIN),
    lbox(BIN_X[0], BIN_X[1], -0.10, -0.04, 0.0, 0.002, GREEN_BIN),
]

GATE_FRICTION = 0.05  # slick plastic bars: a ball slides along them instead of sticking
# gate_left: a bar from (0.365, -0.03) to (0.28, +0.08) across the chute, a rod above the lane to a handle outside the
# +y wall. At rest it sends the balls to the +y (grey) bin; pulled +y 8.5 cm or more it clears the stream.
GL_AT = (0.31, 0.0, 0.10)
GL_BAR = ((0.365, -0.03), (0.28, 0.07))
GATE_LEFT = Joint(
    "gate_left", "slide", axis="y", at=GL_AT, low=0.0, high=0.09, spring=0.0, damping=10.0, mass=0.05,
    friction=GATE_FRICTION,
    shapes=rel([
        rbar(*GL_BAR, BAR_T, BAR_W0, BAR_W1, GREY),
        lbox(0.292, 0.304, 0.06, 0.15, 0.095, 0.104, GREY),  # rod, above the balls
        lbox(0.28, 0.32, 0.13, 0.16, 0.095, 0.145, HAND_GREY),  # handle
    ], GL_AT),
)
# gate_right: the mirror bar, resting pulled out to the -y side; pushed +y 8.5 cm or more it sends the balls to the -y
# (green) bin. Working position is the rest position shifted 9 cm +y.
GR_AT = (0.24, -0.10, 0.10)
GR_BAR = ((0.28, -0.05), (0.195, -0.15))
GATE_RIGHT = Joint(
    "gate_right", "slide", axis="y", at=GR_AT, low=0.0, high=0.09, spring=0.0, damping=10.0, mass=0.05,
    friction=GATE_FRICTION,
    shapes=rel([
        rbar(*GR_BAR, BAR_T, BAR_W0, BAR_W1, GREY),
        lbox(0.18, 0.22, -0.165, -0.135, 0.095, 0.145, HAND_GREY),  # handle on the bar's outer end
    ], GR_AT),
)
# slide_gate: a plate across the hopper mouth, a rod above the lane to a handle outside the +y wall; spring 60 N/m shut.
SG_AT = (0.378, 0.0, 0.10)
SLIDE_GATE = Joint(
    "slide_gate", "slide", axis="y", at=SG_AT, low=0.0, high=0.065, spring=60.0, damping=5.0, mass=0.05,
    friction=0.3,
    shapes=rel([
        rbox(MOUTH - 0.016, MOUTH - 0.001, -0.03, 0.04, BAR_W0, BAR_W1, ORANGE),
        lbox(MOUTH - 0.028, MOUTH - 0.018, 0.0, 0.145, 0.095, 0.104, ORANGE),
        lbox(MOUTH - 0.04, MOUTH, 0.13, 0.16, 0.095, 0.145, ORANGE),  # handle
    ], SG_AT),
)

CHUTE = Mech("chute", pos=(0.0, 0.0, TOP), mass=None, friction=0.4, shapes=FIXED,
             joints=(GATE_LEFT, GATE_RIGHT, SLIDE_GATE))


def _ball(i, xs):
    cx, _, cz = point(xs, 0.0, R_BALL)
    return Sphere(f"ball_{i}", pos=(cx, 0.0, TOP + cz + 0.001), radius=R_BALL, mass=0.03, color=YELLOW)


BALLS = [_ball(i, x) for i, x in enumerate((0.4065, 0.4315, 0.4565))]
BALL_NAMES = [b.name for b in BALLS]
OBJECTS = [CHUTE, *BALLS]

# -- scoring ------------------------------------------------------------------------------------------------------------------
SET_AT = 0.085  # a chute gate counts as set at this travel or more
SHUT = 0.005  # the slide gate counts as shut under this
LIFT_Z = TOP + 0.07  # a ball centre above this over the bins: lifted out
BINS = Rect((0.04, 0.13), (-0.11, 0.11))
GREEN = Rect((0.05, 0.14), (-0.10, -0.04))
GREY_BINS = Rect((0.04, 0.14), (-0.03, 0.11))  # both grey bins: any ball over them, at any height, is lost
IN_BIN_Z = TOP + 0.04  # a settled ball's centre lies below this
AT_REST = dict(max_speed=0.03, max_spin=5.0)

_tracker = ResetOnJumpTracker({b.name: b.pos for b in BALLS}, home_tol=0.01)
on_reset(lambda env: _tracker.clear())


def _latch(objects) -> dict:
    """Ever-true flags over the whole run: a ball over the grey bins (lost), any ball lifted out over the bins (void)."""
    values = {"void": 0.0}
    for b in BALLS:
        s = objects[b.name]
        values[b.name] = 1.0 if overlaps(s, b, GREY_BINS) else 0.0
        if overlaps(s, b, BINS) and s.pos[2] > LIFT_Z:
            values["void"] = 1.0
    return _tracker.update(objects, values)


def _on_floor(s) -> bool:
    """A ball's underside within 1 cm of the green bin's floor."""
    return s.pos[2] - R_BALL - (TOP + 0.002) <= 0.01


def _in_green(objects, b, flags) -> bool:
    """Settled in the green bin: not lost, resting, centre in the bin, and on the floor or nested on balls lying on it
    (a ball whose underside is more than 1 cm above every support counts as not placed)."""
    s = objects[b.name]
    if flags.get(b.name, 0.0) > 0 or not s.resting(**AT_REST):
        return False
    if not (GREEN.x0 <= s.pos[0] <= GREEN.x1 and GREEN.y0 <= s.pos[1] <= GREEN.y1 and TOP < s.pos[2] < IN_BIN_Z):
        return False
    if _on_floor(s):
        return True
    return any(  # touching a ball that lies on the floor, and at least 1.5 cm above its centre
        o.name != b.name and _on_floor(objects[o.name]) and GREEN.x0 <= objects[o.name].pos[0] <= GREEN.x1
        and np.linalg.norm(objects[o.name].pos - s.pos) <= 2 * R_BALL + 0.003 and s.pos[2] - objects[o.name].pos[2] >= 0.015
        for o in BALLS
    )


def _count(objects, flags) -> int:
    return sum(_in_green(objects, b, flags) for b in BALLS)


def score(objects):
    """0 if any ball was lifted out over the bins. Otherwise by the balls now settled in the green bin (a ball that ever
    went over a grey bin is lost): 3 with the slide gate shut, 1.0; 3 with it open, or 2, 0.75; 1, 0.5; none but both
    chute gates set, no ball lost, 0.25. Else 0."""
    flags = _latch(objects)
    if flags.get("void", 0.0) > 0:
        return 0.0
    j = objects["chute"].joints
    n = _count(objects, flags)
    if n == 3:
        return 1.0 if j["slide_gate"] < SHUT else 0.75
    if n:
        return 0.25 * (n + 1)
    lost = any(flags.get(b.name, 0.0) > 0 for b in BALLS)
    return 0.25 if (j["gate_left"] >= SET_AT and j["gate_right"] >= SET_AT and not lost) else 0.0


def success(objects):
    """Full score, every ball and the gates still."""
    return score(objects) == 1.0 and all(objects[b.name].resting(**AT_REST) for b in BALLS)


# -- the robot itself keeps out of the grey bins too (checked offline with the URDF) ----------------------------------------------
ARM_KEEP_OUT = [
    {"name": "grey bins and 4 cm around them", "x": (0.0, 0.17), "y": (-0.07, 0.15), "z_top": TOP + 0.25},
]

# -- the demo: both arms set their gate, then the left arm pulls the slide gate and holds it ----------------------------------------
HOVER = TOP + 0.17  # TCP height while travelling
GRIP = TOP + 0.125  # TCP height at the handles (the pad tips then reach 1.3 cm lower)


def _pull(x, y_from, y_to, secs, yaw=0.0):
    """Descend on a handle at (x, y_from), grip, slide it to y_to, let go and rise."""
    return [
        Waypoint(3.0, (x, y_from, HOVER), FINGER_OPEN, "over the handle", yaw),
        Waypoint(2.0, (x, y_from, GRIP), FINGER_OPEN, "descend", yaw),
        Waypoint(0.8, (x, y_from, GRIP), FINGER_CLOSED, "close", yaw),
        Waypoint(secs, (x, y_to, GRIP), FINGER_CLOSED, "slide the gate", yaw),
        Waypoint(0.8, (x, y_to, GRIP), FINGER_OPEN, "let go", yaw),
        Waypoint(1.5, (x, y_to, HOVER), FINGER_OPEN, "rise", yaw),
    ]


def demo():
    """Left: set gate_left (pull it +y to its stop), then take the slide gate's handle, pull it its full 6.5 cm, hold it
    for 5 s while the balls run, ease it back, let go. Right: push gate_right +y to its stop, then stay clear. The hands
    are commanded 5 mm past each stop so the gates seat against it."""
    left = _pull(0.30, 0.145, 0.145 + 0.095, 5.0)
    left += [Waypoint(2.0, (0.375, 0.145, HOVER), FINGER_OPEN, "to the slide handle", 0.0)]
    left += [
        Waypoint(2.0, (0.375, 0.145, GRIP), FINGER_OPEN, "descend", 0.0),
        Waypoint(0.8, (0.375, 0.145, GRIP), FINGER_CLOSED, "close", 0.0),
        Waypoint(2.5, (0.375, 0.145 + 0.095, GRIP), FINGER_CLOSED, "pull the slide gate", 0.0),
        Waypoint(5.0, (0.375, 0.145 + 0.095, GRIP), FINGER_CLOSED, "hold it open", 0.0),
        Waypoint(3.0, (0.375, 0.145, GRIP), FINGER_CLOSED, "ease it back", 0.0),
        Waypoint(0.8, (0.375, 0.145, GRIP), FINGER_OPEN, "let go", 0.0),
        Waypoint(1.5, (0.375, 0.145, HOVER), FINGER_OPEN, "rise", 0.0),
    ]
    right = _pull(0.21, -0.15, -0.15 + 0.095, 5.0)
    # Park both hands high and back so the last frame shows open grippers well clear of every handle.
    left += [Waypoint(3.0, (0.26, 0.22, TOP + 0.20), FINGER_OPEN, "park clear", 0.0)]
    right += [Waypoint(3.0, (0.26, -0.22, TOP + 0.20), FINGER_OPEN, "park clear", 0.0)]
    return {"left": left, "right": right}
