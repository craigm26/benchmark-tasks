"""On the station table, between the arms: a grey steel flange plate on two dark legs, with two slotted holes (3.0 cm along x,
3.6 cm along y) at y -8 cm (hole A) and +8 cm (hole B). A blue splice plate lies on it, held by a spring (150 N/m) that has
slid it 2.9 cm out of line, so at rest only a 1.0 cm strip of hole A and a 0.4 cm strip of hole B show through the plate (its
holes are also 0.6 cm closer together than the flange's, as after fit-up). An orange drift pin (a tapered tip, a shank and a
head) and two yellow bolts (a head, an upright grip tab, a chamfered shank) each hang by their heads in a dark holder: the
drift and one bolt in front of the right arm, the other bolt in front of the left arm. The plate and the three parts are
loose; the flange and the holders are fixed.

Requires -E object_jitter=0.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import closedloop  # noqa: E402, F401
from belimits import tilt_deg  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips things: keep the episode (and the video) running until the release is seen

GREY, DARK = (0.62, 0.62, 0.66), (0.17, 0.17, 0.20)
BLUE, ORANGE, YELLOW = (0.08, 0.28, 0.95), (0.98, 0.45, 0.02), (0.98, 0.85, 0.05)

# -- the flange and the splice plate -------------------------------------------------------------------------------------------
# Mech frame: x forward and y left from FLANGE_XY on the table, z up from the table top.
FX, FY = 0.27, 0.0
HOLE_Y = 0.08  # flange hole centres at y -HOLE_Y (hole A) and +HOLE_Y (hole B)
HX, HY = 0.030, 0.036  # slotted hole, x by y
FL_Z0, FL_Z1 = 0.060, 0.080  # flange plate
PL_Z0, PL_Z1 = 0.0803, 0.0953  # splice plate (0.3 mm above the flange)
PLATE_TOP = TOP + PL_Z1
PL_HOLE_Y = HOLE_Y - 0.003  # the plate's hole centres, in the plate's own frame: 0.6 cm closer together than the flange's
PL_HALF_Y = HOLE_Y + 0.03
FL_HALF = HOLE_Y + 0.045
REST = 0.029  # the plate rests this far (in -y) from where its holes sit over the flange holes' centres, less 0.3 cm of pitch misfit
J_MAX = 0.034
J_LINE = 0.014  # the line is made when the plate is slid this far from rest, with the drift in hole A
J_BOLT_B_MIN = 0.012  # a bolt cannot start in hole B below this slide

_hy2 = HY / 2
FLANGE = Mech(
    "flange",
    pos=(FX, FY, TOP),
    mass=None,
    friction=0.6,
    shapes=[
        box((0.0, -(FL_HALF - 0.0075), 0.030), (0.12, 0.015, 0.060), DARK),  # legs
        box((0.0, FL_HALF - 0.0075, 0.030), (0.12, 0.015, 0.060), DARK),
        box((-0.0375, 0.0, (FL_Z0 + FL_Z1) / 2), (0.045, 2 * FL_HALF, FL_Z1 - FL_Z0), GREY),  # front and back strips
        box((0.0375, 0.0, (FL_Z0 + FL_Z1) / 2), (0.045, 2 * FL_HALF, FL_Z1 - FL_Z0), GREY),
        box((0.0, -(HOLE_Y + _hy2 + FL_HALF) / 2, (FL_Z0 + FL_Z1) / 2), (HX, FL_HALF - HOLE_Y - _hy2, FL_Z1 - FL_Z0), GREY),  # outside the holes
        box((0.0, 0.0, (FL_Z0 + FL_Z1) / 2), (HX, 2 * (HOLE_Y - _hy2), FL_Z1 - FL_Z0), GREY),  # between them
        box((0.0, (HOLE_Y + _hy2 + FL_HALF) / 2, (FL_Z0 + FL_Z1) / 2), (HX, FL_HALF - HOLE_Y - _hy2, FL_Z1 - FL_Z0), GREY),
    ],
    joints=[
        Joint(
            "plate",
            "slide",
            axis="y",
            at=(0.0, -REST, 0.0),
            low=-0.003,
            high=J_MAX,
            spring=150.0,
            damping=5.0,
            mass=0.12,
            friction=0.3,
            shapes=[
                box((-0.0325, 0.0, (PL_Z0 + PL_Z1) / 2), (0.035, 2 * PL_HALF_Y, PL_Z1 - PL_Z0), BLUE),
                box((0.0325, 0.0, (PL_Z0 + PL_Z1) / 2), (0.035, 2 * PL_HALF_Y, PL_Z1 - PL_Z0), BLUE),
                box((0.0, -(PL_HALF_Y + PL_HOLE_Y + _hy2) / 2, (PL_Z0 + PL_Z1) / 2),
                    (HX, PL_HALF_Y - PL_HOLE_Y - _hy2, PL_Z1 - PL_Z0), BLUE),
                box((0.0, 0.0, (PL_Z0 + PL_Z1) / 2), (HX, 2 * (PL_HOLE_Y - _hy2), PL_Z1 - PL_Z0), BLUE),
                box((0.0, (PL_HALF_Y + PL_HOLE_Y + _hy2) / 2, (PL_Z0 + PL_Z1) / 2),
                    (HX, PL_HALF_Y - PL_HOLE_Y - _hy2, PL_Z1 - PL_Z0), BLUE),
            ],
        )
    ],
)


def overlap_a(j):
    """Width (m) of the strip of hole A that shows through the plate at slide j."""
    return (-PL_HOLE_Y + _hy2 - REST + j) - (-HOLE_Y - _hy2)


def overlap_b(j):
    return (PL_HOLE_Y + _hy2 - REST + j) - (HOLE_Y - _hy2)


# -- the holders -------------------------------------------------------------------------------------------------------------
RIM = 0.065  # holder height: a part hangs by its head on the rim with its tip 5 mm above the table
HOLDERS = {"drift": (0.19, -0.165), "bolt_a": (0.19, 0.17), "bolt_b": (0.19, -0.245)}


def _holder(name, x, y):
    return Mech(
        name + "_holder",
        pos=(x, y, TOP),
        mass=None,
        friction=0.6,
        shapes=[
            box((-0.021, 0.0, RIM / 2), (0.010, 0.052, RIM), DARK),
            box((0.021, 0.0, RIM / 2), (0.010, 0.052, RIM), DARK),
            box((0.0, -0.021, RIM / 2), (0.032, 0.010, RIM), DARK),
            box((0.0, 0.021, RIM / 2), (0.032, 0.010, RIM), DARK),
        ],
    )


# -- the drift and the bolts: local origin at the centre of the head's underside, z up -----------------------------------------
def _wedge(sign, y_tip, y_top, z_tip, z_top, width, color, t=0.006):
    """A slab tilted about x: its outer face runs from (y_tip, z_tip) to (y_top, z_top) on the sign side."""
    dy, dz = (y_top - y_tip), (z_top - z_tip)
    length = math.hypot(dy, dz)
    ang = math.atan2(dy, dz)  # from vertical
    nrm = (math.cos(ang), -math.sin(ang))  # outward normal (y, z)
    cy = (y_tip + y_top) / 2 - nrm[0] * t / 2
    cz = (z_tip + z_top) / 2 - nrm[1] * t / 2
    a = -sign * ang / 2
    return box((0.0, sign * cy, cz), (width, t, length), color, tilt=(math.cos(a), math.sin(a), 0.0, 0.0))


DR_HEAD = (0.040, 0.029, 0.040)  # x, y, z
DR_SHANK = (0.020, 0.029, 0.020)  # z 0..-2 cm
DR_TAPER = 0.040  # z -2..-6 cm, from 2.9 wide to 0.3 wide in y
DR_TIP_Y = 0.0015  # half width at the tip
DR_LEN = 0.060

DRIFT = Mech(
    "drift",
    pos=(HOLDERS["drift"][0], HOLDERS["drift"][1], TOP + RIM + 0.0004),
    mass=0.10,
    friction=1.0,
    shapes=[
        box((0, 0, DR_HEAD[2] / 2), DR_HEAD, ORANGE),
        box((0, 0, -DR_SHANK[2] / 2), DR_SHANK, ORANGE),
        box((0, 0, -(DR_SHANK[2] + DR_TAPER / 2)), (0.020, 0.002, DR_TAPER), ORANGE),
        _wedge(1, DR_TIP_Y, DR_SHANK[1] / 2, -DR_LEN, -DR_SHANK[2], 0.020, ORANGE, t=0.0025),
        _wedge(-1, DR_TIP_Y, DR_SHANK[1] / 2, -DR_LEN, -DR_SHANK[2], 0.020, ORANGE, t=0.0025),
    ],
)

BT_HEAD = 0.010
BT_TAB = (0.040, 0.018, 0.030)
BT_S = 0.016  # shank, square
BT_LEN = 0.060
BT_CH = 0.010  # chamfer length
BT_TIP_Y = 0.0065  # half width at the tip (1.3 cm)


def _bolt(name):
    x, y = HOLDERS[name]
    return Mech(
        name,
        pos=(x, y, TOP + RIM + 0.0004),
        mass=0.08,
        friction=1.0,
        shapes=[
            box((0, 0, BT_HEAD / 2), (0.040, 0.040, BT_HEAD), YELLOW),
            box((0, 0, BT_HEAD + BT_TAB[2] / 2), BT_TAB, YELLOW),
            box((0, 0, -(BT_LEN - BT_CH) / 2), (BT_S, BT_S, BT_LEN - BT_CH), YELLOW),
            box((0, 0, -(BT_LEN - BT_CH / 2)), (BT_S, 2 * BT_TIP_Y, BT_CH * 0.8), YELLOW),
            _wedge(1, BT_TIP_Y, BT_S / 2, -BT_LEN, -(BT_LEN - BT_CH), BT_S, YELLOW, t=0.004),
            _wedge(-1, BT_TIP_Y, BT_S / 2, -BT_LEN, -(BT_LEN - BT_CH), BT_S, YELLOW, t=0.004),
        ],
    )


BOLT_A, BOLT_B = _bolt("bolt_a"), _bolt("bolt_b")
OBJECTS = [FLANGE, _holder("drift", *HOLDERS["drift"]), _holder("bolt_a", *HOLDERS["bolt_a"]),
           _holder("bolt_b", *HOLDERS["bolt_b"]), DRIFT, BOLT_A, BOLT_B]

# -- scoring -----------------------------------------------------------------------------------------------------------------
HOLD_STEPS = 20  # 1 s at the 20 Hz control rate
HEAD_TOL = 0.004  # a bolt head's underside within this of the plate top
SEAT_TILT = 5.0
TIP_BELOW = TOP + FL_Z0 - 0.005  # a bolt's tip below the flange underside by 5 mm or more
HOLE_TOL = 0.012  # bolt axis within 1.2 cm of a hole axis (a shank through the flange hole is always within 1.0 cm in y)
DRIFT_TIP_BELOW = TOP + PL_Z0  # a drift in a hole has its tip under the plate
DRIFT_TILT = 15.0
STOW_XY, STOW_Z, STOW_TILT = 0.015, 0.010, 10.0
OFF_TABLE = 0.02

_S = {"line": 0, "line_ok": False, "held": False}


def _clear():
    _S.update({"line": 0, "line_ok": False, "held": False})


@on_reset
def _reset(_env=None):
    _clear()


def _pt(s, local):
    return s.origin + s.rotation() @ np.asarray(local, float)


def _tip_z(s, length):
    return float(_pt(s, (0, 0, -length))[2])


def _bolt_hole(s):
    """'A', 'B' or None: the hole this bolt is home in (shank through the flange, head flat on the plate)."""
    if tilt_deg(s) > SEAT_TILT:
        return None
    if abs(float(s.origin[2]) - PLATE_TOP) > HEAD_TOL or _tip_z(s, BT_LEN) > TIP_BELOW:
        return None
    for h, hy in (("A", -HOLE_Y), ("B", HOLE_Y)):
        if abs(s.origin[0] - FX) <= HOLE_TOL and abs(s.origin[1] - (FY + hy)) <= HOLE_TOL:
            return h
    return None


def _drift_in(s, hole_y):
    """The drift's tip is through the plate and flange at that hole, upright to 15 degrees."""
    ax = _pt(s, (0, 0, -DR_LEN))
    return bool(tilt_deg(s) <= DRIFT_TILT and _tip_z(s, DR_LEN) < DRIFT_TIP_BELOW and abs(ax[0] - FX) <= 0.02
                and abs(ax[1] - (FY + hole_y)) <= 0.02)


def _stowed(s):
    hx, hy = HOLDERS["drift"]
    return bool(tilt_deg(s) <= STOW_TILT and abs(s.origin[0] - hx) <= STOW_XY and abs(s.origin[1] - hy) <= STOW_XY
                and abs(float(s.origin[2]) - (TOP + RIM)) <= STOW_Z)


def _off_table(s):
    return bool(s.low[2] < TOP - OFF_TABLE)


def _levels(objects):
    f, d, ba, bb = objects["flange"], objects["drift"], objects["bolt_a"], objects["bolt_b"]
    j = float(f.joints["plate"])
    in_a = _drift_in(d, -HOLE_Y)
    made = in_a and j >= J_LINE
    _S["line"] = _S["line"] + 1 if made else 0
    if _S["line"] >= HOLD_STEPS:
        _S["line_ok"] = True
    holes = {h for h in (_bolt_hole(ba), _bolt_hole(bb)) if h}
    if holes and (in_a or _drift_in(d, HOLE_Y)):
        _S["held"] = True
    return j, holes


def score(objects):
    """Flags are kept over the run. 0.25: the drift stood in hole A with the plate slid 1.4 cm or more for 1 s. 0.5: also a bolt is
    home (shank through the flange, head flat on the plate within 4 mm and 5 degrees, axis within 1.2 cm of a hole) and was put there while the
    drift was still in a hole. 0.75: both holes bolted. 1.0: also the drift stands in its holder (head on the rim) and all are still. Bolts or the
    drift off the table: 0. A bolt home with no drift line made scores 0."""
    d, ba, bb = objects["drift"], objects["bolt_a"], objects["bolt_b"]
    j, holes = _levels(objects)
    if _off_table(d) or _off_table(ba) or _off_table(bb):
        return 0.0
    if not _S["line_ok"]:
        return 0.0
    if not (_S["held"] and holes):
        return 0.25
    if len(holes) < 2:
        return 0.5
    if not (_stowed(d) and d.resting() and ba.resting() and bb.resting()):
        return 0.75
    return 1.0


def success(objects):
    return score(objects) == 1.0


# No area is forbidden to the parts. The robot's own rule is a pinch zone over the splice: no arm part passes below 10.5 cm
# over the plates, so the fingers stay out of the hole line (the gate enforces it; the scorer cannot see arms).
ARM_KEEP_OUT = [
    {"name": "splice pinch zone", "x": (FX - 0.07, FX + 0.07), "y": (-(FL_HALF + 0.015), FL_HALF + 0.015), "z_top": TOP + 0.105},
]

# -- the scripted demo -------------------------------------------------------------------------------------------------------
DRIFT_OPEN = -0.026  # fingers opened 5.2 cm round the 2.9 cm drift head
GRIP_OPEN = -0.022  # fingers opened 4.4 cm: room round a 1.8 cm tab, and 8 cm between the two grippers
TRAVEL = TOP + 0.20
DRIFT_GRIP_Z = TOP + RIM + 0.020  # head mid-height on the rim
BOLT_GRIP_Z = TOP + RIM + BT_HEAD + 0.015
DRIFT_SEATED_Z = PLATE_TOP + 0.020
BOLT_SEATED_Z = PLATE_TOP + BT_HEAD + 0.015 + 0.0
DRIFT_LIFT_Z = TOP + 0.185
TIP_AIM_Y = -HOLE_Y - _hy2 + overlap_a(0.0) / 2  # centre of the strip of hole A that shows at rest
_PRESS = 0.002


def _j(ctx):
    return float(ctx.obj("flange").joints["plate"])


def demo():
    from closedloop import placeholder_demo
    return placeholder_demo()


def _aim(arm, goal_xy, d, z0, z1, dur, finger, stop=None, gain=0.3, clamp=0.03):
    """Move from height z0 to z1 over dur seconds while closed-loop steering the TCP's measured xy so the held part's origin
    (TCP xy + d) sits at goal_xy. goal_xy may be a function (re-read every step). stop() ends the move early."""
    steps = max(1, round(dur / arm.dt))
    rot = arm.rot.copy()
    c = getattr(arm, "_c", np.zeros(2))
    for i in range(1, steps + 1):
        g = np.asarray(goal_xy() if callable(goal_xy) else goal_xy, float)
        want = g - d
        c = np.clip(c + gain * (want - arm.measured_pos[:2]), -clamp, clamp)
        z = z0 + (z1 - z0) * i / steps
        arm._set_cartesian(np.array([want[0] + c[0], want[1] + c[1], z]), rot, finger)
        yield
        if stop is not None and stop():
            break
    arm._c = c


def _grab(ctx, arm, name, grip_z, travel_z, open_f, shut_f):
    """Hover over the part, steer onto its origin, lower beside it, close, lift straight up. Returns the part's xy offset from
    the TCP while it hangs in the grip."""
    arm._c = np.zeros(2)
    o = np.asarray(ctx.obj(name).origin[:2], float)
    yield from arm.move((float(o[0]), float(o[1]), travel_z), 2.5, open_f)
    yield from _aim(arm, o, np.zeros(2), travel_z, travel_z, 1.0, open_f)
    yield from _aim(arm, o, np.zeros(2), travel_z, grip_z, 1.5, open_f)
    yield from arm.grip(shut_f, 0.8)
    yield from arm.move((float(arm.pos[0]), float(arm.pos[1]), travel_z), 1.5, shut_f)
    yield from arm.hold(0.4)
    return np.asarray(ctx.obj(name).origin[:2], float) - arm.measured_pos[:2]


def demo_script(ctx):
    right, left = ctx.right, ctx.left
    C = FINGER_CLOSED
    hx, hy = HOLDERS["drift"]
    hover = TOP + 0.185

    # 1. right: take the drift, carry it over the strip of hole A that shows, lower it so the taper cams the plate
    d = yield from _grab(ctx, right, "drift", DRIFT_GRIP_Z, DRIFT_LIFT_Z, DRIFT_OPEN, C)
    yield from right.move((FX - d[0], TIP_AIM_Y - d[1], DRIFT_LIFT_Z), 2.5, C)
    right._c = np.zeros(2)
    aim = lambda: (FX, TIP_AIM_Y)  # noqa: E731
    yield from _aim(right, aim, d, DRIFT_LIFT_Z, DRIFT_LIFT_Z, 1.2, C)
    # tip 1 cm over the plate -> 4 cm of taper (5 s) -> shank down to the head on the plate; the flange wall centres it
    z_cam_end = DRIFT_LIFT_Z - 0.050
    yield from _aim(right, aim, d, DRIFT_LIFT_Z, z_cam_end, 5.0, C)
    yield from _aim(right, (FX, -HOLE_Y - 0.002), d, z_cam_end, DRIFT_SEATED_Z - _PRESS, 2.5, C)
    yield from right.hold(0.8)
    drift_c = right.pos.copy()

    # 2. left: bolt into hole B while the drift holds the line
    d_l = yield from _grab(ctx, left, "bolt_a", BOLT_GRIP_Z, hover, GRIP_OPEN, C)
    bx = lambda: (FX, HOLE_Y - _hy2 + min(max(overlap_b(_j(ctx)), 0.012), 0.03) / 2)  # noqa: E731
    yield from left.move((FX - d_l[0], float(bx()[1]) - d_l[1], hover), 2.5, C)
    left._c = np.zeros(2)
    yield from _aim(left, bx, d_l, hover, hover, 1.2, C)
    z_pre = BOLT_SEATED_Z + 0.030
    yield from _aim(left, bx, d_l, hover, z_pre, 1.8, C)
    seated = lambda: float(ctx.obj("bolt_a").origin[2]) <= PLATE_TOP + 0.0015  # noqa: E731
    yield from _aim(left, bx, d_l, z_pre, BOLT_SEATED_Z - 2 * _PRESS, 4.0, C, stop=seated)
    yield from left.hold(0.6)
    yield from left.grip(FINGER_OPEN, 0.6)
    yield from left.move((left.pos[0], left.pos[1], hover), 1.2, FINGER_OPEN)
    yield from left.move((0.20, 0.20, TOP + 0.20), 2.0, FINGER_OPEN)

    # 3. right: pull the drift straight up, stand it in its holder
    yield from right.move((drift_c[0], drift_c[1], drift_c[2] + 0.07), 3.0, C)
    right._c = np.zeros(2)
    hold_xy = lambda: (hx, hy)  # noqa: E731
    yield from right.move((hx - d[0], hy - d[1], DRIFT_LIFT_Z), 3.0, C)
    yield from _aim(right, hold_xy, d, DRIFT_LIFT_Z, DRIFT_LIFT_Z, 1.2, C)
    yield from _aim(right, hold_xy, d, DRIFT_LIFT_Z, DRIFT_GRIP_Z + 0.004, 2.5, C)
    yield from right.hold(0.3)
    yield from right.grip(DRIFT_OPEN, 0.6)
    yield from right.move((right.pos[0], right.pos[1], hover), 1.2, DRIFT_OPEN)

    # 4. right: the second bolt into hole A, aimed at the strip that shows with the plate held by the first bolt
    d_r = yield from _grab(ctx, right, "bolt_b", BOLT_GRIP_Z, hover, GRIP_OPEN, C)
    ax = lambda: (FX, -HOLE_Y - _hy2 + min(max(overlap_a(_j(ctx)), 0.012), 0.03) / 2)  # noqa: E731
    yield from right.move((FX - d_r[0], float(ax()[1]) - d_r[1], hover), 2.5, C)
    right._c = np.zeros(2)
    yield from _aim(right, ax, d_r, hover, hover, 1.2, C)
    yield from _aim(right, ax, d_r, hover, z_pre, 1.8, C)
    seated_b = lambda: float(ctx.obj("bolt_b").origin[2]) <= PLATE_TOP + 0.0015  # noqa: E731
    yield from _aim(right, ax, d_r, z_pre, BOLT_SEATED_Z - 2 * _PRESS, 4.0, C, stop=seated_b)
    yield from right.hold(0.6)
    yield from right.grip(FINGER_OPEN, 0.6)
    yield from right.move((right.pos[0], right.pos[1], hover), 1.2, FINGER_OPEN)
    yield from right.move((0.20, -0.20, TOP + 0.20), 2.0, FINGER_OPEN)
    yield from ctx.wait(1.0)
