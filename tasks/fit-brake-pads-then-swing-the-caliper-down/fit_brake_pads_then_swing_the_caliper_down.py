"""On the station table, between the arms and a little ahead of them: a fixed brake (a dark grey caliper bracket with a
light grey rotor standing on edge in its middle, and a red hinged caliper) and two loose brake pads in blue-grey holders.

Each pad is a silver steel backing plate (0.6 x 9.0 x 5.0 cm) whose two ends are its ears, with a dark brown friction
lining (1.4 x 7.0 x 4.0 cm) on one face and an orange grip tab (1.8 x 4.0 x 3.0 cm) on top. The bracket has two
abutment blocks, 8 cm apart (inner faces), each cut with two vertical ear slots (1.2 cm wide, 1 cm deep); the inner
pair of slots is on one side of the rotor and the outer pair on the other, 3.1 cm from its middle. A dark pad stop
stands behind each pad position, where the piston and the caliper finger would be, so a pad turned lining-outward
cannot get its ears down. The red caliper is hinged on a post behind the left abutment (+y) and starts swung up past
upright; its red bridge covers both pads when closed, with a red post on top of it to hold. The pads start upright in
two blue holders on the right-hand side of the table, with their linings facing +y; each must be turned a quarter
turn, one each way, to face the rotor. The right arm reaches both holders; the left arm reaches the caliper.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=fit_brake_pads_then_swing_the_caliper_down.py -E object_jitter=0 \\
           --instruction "Fit both new brake pads in the bracket with their linings facing the rotor, then swing the
           red caliper down over them." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, _sat_overlap, tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, Shape, box  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips things: let the release be seen before the episode ends

# -- colours ----------------------------------------------------------------------------------------------------------
BRACKET = (0.30, 0.31, 0.34)
STOP = (0.18, 0.18, 0.21)
ROTOR = (0.62, 0.68, 0.76)
HOLDER = (0.22, 0.32, 0.55)
STEEL = (0.86, 0.87, 0.89)
LINING = (0.30, 0.14, 0.06)
TAB = (0.97, 0.52, 0.05)
BRIDGE = (0.62, 0.04, 0.06)
POST = (0.95, 0.10, 0.08)

# -- the brake, in the station frame ----------------------------------------------------------------------------------
BX, BY = 0.27, -0.04  # the middle of the rotor face, x, and the middle of the bracket, y
ROTOR_X = (BX - 0.01, BX + 0.01)  # x 0.26..0.28
ROTOR_Y = (BY - 0.08, BY + 0.08)  # y -0.12..0.04
ROTOR_H = 0.08
SLOT_DX = 0.031  # slot centre from the rotor's middle
SLOT_W = 0.012
FLOOR = 0.035  # slot floor height above the table
ABUT_TOP = 0.075
STOP_TOP = 0.07
HINGE_Y, HINGE_Z = BY + 0.075, 0.12  # caliper hinge axis (along x)
OPEN = 1.75  # rad the caliper swings from its start to closed

PLATE_T, PLATE_L, PLATE_H = 0.006, 0.09, 0.05
LINING_T, LINING_L, LINING_H = 0.014, 0.07, 0.04
TAB_T, TAB_L, TAB_H = 0.018, 0.04, 0.03
PAD_MASS = 0.06

SLOT_CENTRES = (BX - SLOT_DX, BX + SLOT_DX)  # inner (lining toward +x), outer (lining toward -x)
FACING_X = (+1.0, -1.0)  # which way the lining must face at each pair of slots


def _rx(angle: float):
    """Quaternion (w, x, y, z) for a turn about x."""
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)


def _open_pose(y: float, z: float) -> tuple[float, float]:
    """A point at (y, z) from the hinge when the caliper is closed, as it sits when the caliper has swung up OPEN rad."""
    c, s = math.cos(OPEN), math.sin(OPEN)
    return y * c + z * s, -y * s + z * c


def _brake() -> Mech:
    shapes = [
        box((0, 0, 0.015), (0.10, 0.14, 0.03), BRACKET),  # carrier base
        box((0, 0, ROTOR_H / 2), (0.02, 0.16, ROTOR_H), ROTOR),  # rotor on edge
        box((0, 0.0775, 0.0725), (0.08, 0.015, 0.085), BRACKET),  # hinge post
    ]
    h = ABUT_TOP - 0.03
    for sign in (-1, 1):  # the two abutments, each a back block and a slotted front strip
        shapes.append(box((0, sign * 0.06, 0.03 + h / 2), (0.10, 0.02, h), BRACKET))
        for cx, w in ((-0.0435, 0.013), (0.0, 0.05), (0.0435, 0.013)):
            shapes.append(box((cx, sign * 0.045, 0.03 + h / 2), (w, 0.01, h), BRACKET))
        for cx in (-SLOT_DX, SLOT_DX):
            shapes.append(box((cx, sign * 0.045, 0.03 + (FLOOR - 0.03) / 2), (SLOT_W, 0.01, FLOOR - 0.03), BRACKET))
    for cx in (-0.0435, 0.0435):  # pad stops behind each pad position
        shapes.append(box((cx, 0.0, 0.03 + (STOP_TOP - 0.03) / 2), (0.013, 0.08, STOP_TOP - 0.03), STOP))
    ay, az = _open_pose(-0.07, 0.01)
    py, pz = _open_pose(-0.07, 0.035)
    tilt = _rx(-OPEN)
    caliper = Joint(
        "caliper", "hinge", axis="x", at=(0.0, HINGE_Y - BY, HINGE_Z), low=0.0, high=OPEN, spring=0.0, damping=3.0, mass=0.15,
        shapes=(
            Shape("box", (0.0, ay, az), (0.09, 0.14, 0.02), BRIDGE, tilt),
            Shape("box", (0.0, py, pz), (0.02, 0.02, 0.03), POST, tilt),
        ),
    )
    return Mech("brake", pos=(BX, BY, TOP), mass=None, shapes=shapes, joints=[caliper])


def _holder(name: str, hx: float, hy: float) -> Mech:
    """A tray with two cheeks that stands a pad upright, its lining toward +y: the pad's plate is at (hx, hy)."""
    return Mech(
        name, pos=(hx, hy, TOP), mass=None,
        shapes=[
            box((0, 0.007, 0.002), (0.10, 0.042, 0.004), HOLDER),
            box((0, 0.024, 0.015), (0.10, 0.008, 0.03), HOLDER),
            box((0, -0.010, 0.015), (0.10, 0.008, 0.03), HOLDER),
        ],
    )


def _pad(name: str, hx: float, hy: float) -> Mech:
    """A pad whose origin is the middle of its plate's bottom edge; local +x is the lining side, +y the long way."""
    return Mech(
        name, pos=(hx, hy, TOP + 0.0045), mass=PAD_MASS, yaw=math.pi / 2,
        shapes=[
            box((0, 0, PLATE_H / 2), (PLATE_T, PLATE_L, PLATE_H), STEEL),
            box((PLATE_T / 2 + LINING_T / 2, 0, 0.005 + LINING_H / 2), (LINING_T, LINING_L, LINING_H), LINING),
            box((0, 0, PLATE_H + TAB_H / 2), (TAB_T, TAB_L, TAB_H), TAB),
        ],
    )


BRAKE = _brake()
HOLDER_A, HOLDER_B = _holder("holder_a", 0.205, -0.23), _holder("holder_b", 0.205, -0.145)
PAD_A, PAD_B = _pad("pad_a", 0.205, -0.23), _pad("pad_b", 0.205, -0.145)
PADS = ("pad_a", "pad_b")
OBJECTS = [BRAKE, HOLDER_A, HOLDER_B, PAD_A, PAD_B]
SPECS = {o.name: o for o in OBJECTS}

# -- scoring ----------------------------------------------------------------------------------------------------------
SEAT_Z_TOL = 0.003  # a seated pad's bottom within this of the slot floor
SEAT_X_TOL = 0.004  # plate middle from the slot middle
SEAT_Y_TOL = 0.003  # pad middle from the bracket's middle (its ears must be in both slots)
SEAT_TILT_DEG = 10.0  # a pad leans up to about 8 degrees in its 1.2 cm slots
SEAT_YAW_DEG = 6.0  # pad's long axis from y, and lining direction from the required one
CLOSED_TOL = 0.05  # rad: closed when within this of the stop (a tape: the post within 4 mm of resting)
AIRBORNE_M = 0.01
_ROTOR = Rect((ROTOR_X[0] + 0.002, ROTOR_X[1] - 0.002), ROTOR_Y)  # 2 mm of give: a lining resting against the rotor face is not over it
_COS_YAW = math.cos(math.radians(SEAT_YAW_DEG))


def _rot(s) -> np.ndarray:
    return s.rotation()


def _plate_bottom(s) -> float:
    """World z of the lowest point of the pad's backing plate (its two bottom ears)."""
    r, o = _rot(s), np.asarray(s.origin, float)
    pts = np.array([[a, b, 0.0] for a in (-PLATE_T / 2, PLATE_T / 2) for b in (-PLATE_L / 2, PLATE_L / 2)])
    return float((o + pts @ r.T)[:, 2].min())


def _outline(s) -> np.ndarray:
    """The pad seen from above: every corner of its plate, lining and tab, projected onto the table (x, y)."""
    r, o = _rot(s), np.asarray(s.origin, float)
    pts = []
    for cx, cy, cz, sx, sy, sz in (
        (0.0, 0.0, PLATE_H / 2, PLATE_T, PLATE_L, PLATE_H),
        (PLATE_T / 2 + LINING_T / 2, 0.0, 0.005 + LINING_H / 2, LINING_T, LINING_L, LINING_H),
        (0.0, 0.0, PLATE_H + TAB_H / 2, TAB_T, TAB_L, TAB_H),
    ):
        for a in (-1, 1):
            for b in (-1, 1):
                for c in (-1, 1):
                    pts.append([cx + a * sx / 2, cy + b * sy / 2, cz + c * sz / 2])
    return (o + np.array(pts) @ r.T)[:, :2]


def over_rotor(s) -> bool:
    """Any part of the pad is over the rotor, at any height."""
    return bool(_sat_overlap(_outline(s), _ROTOR.poly()))


def airborne(s) -> bool:
    """The pad's bottom is more than 1 cm above every support under it (held, or in flight)."""
    b = _plate_bottom(s)
    supports = [TOP, TOP + 0.004, TOP + 0.03, TOP + FLOOR, TOP + STOP_TOP, TOP + ABUT_TOP, TOP + ROTOR_H]
    return all(b - z > AIRBORNE_M for z in supports)


def seat_of(s):
    """0 if the pad is seated in the inner slots, 1 in the outer slots, else None. Seated: its plate's ears in both
    slots of a pair (bottom within 3 mm of the slot floor, plate middle within 4 mm of the slot middle, pad middle within
    3 mm of the bracket's middle), upright within 10 degrees, with its lining turned to face the rotor."""
    r, o = _rot(s), np.asarray(s.origin, float)
    if tilt_deg(s) > SEAT_TILT_DEG or airborne(s) or over_rotor(s):
        return None
    if abs(_plate_bottom(s) - (TOP + FLOOR)) > SEAT_Z_TOL or abs(o[1] - BY) > SEAT_Y_TOL:
        return None
    if abs(r[1, 1]) < _COS_YAW:  # long axis not along y
        return None
    lining = math.hypot(r[0, 0], r[1, 0])  # the lining's direction seen from above, so a lean in the slots does not count
    for k, cx in enumerate(SLOT_CENTRES):
        if abs(o[0] - cx) <= SEAT_X_TOL and lining > 0.5 and r[0, 0] * FACING_X[k] / lining >= _COS_YAW:
            return k
    return None


def caliper_closed(objects) -> bool:
    return objects["brake"].joints.get("caliper", 0.0) >= OPEN - CLOSED_TOL


def score(objects):
    """0 if a pad is over the rotor. Otherwise 0.25 for one pad seated facing the rotor, 0.5 for both pads seated, 1.0
    for both seated and the caliper closed (success() also needs everything still). A pad held in the air, turned
    lining-outward, or standing on the bracket is not seated."""
    pads = [objects[n] for n in PADS]
    if any(over_rotor(p) for p in pads):
        return 0.0
    seats = [seat_of(p) for p in pads]
    n = sum(k is not None for k in seats)
    if n == 2 and seats[0] == seats[1]:
        n = 1
    if n < 2:
        return 0.25 * n
    return 1.0 if caliper_closed(objects) else 0.5


def success(objects):
    """Both pads seated, the caliper closed, nothing moving and nothing held up; holds 8 s."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in PADS) and not any(airborne(objects[n]) for n in PADS)


# The robot keeps off the rotor too: no arm point goes below 13 cm above the table over the rotor (the pads sit 4 mm from it, so no wider margin).
ARM_KEEP_OUT = [
    {"name": "rotor face", "x": ROTOR_X, "y": (ROTOR_Y[0] - 0.01, ROTOR_Y[1] + 0.01), "z_top": TOP + 0.13},
]

# -- the scripted demo (closed loop) ----------------------------------------------------------------------------------
demo = placeholder_demo

TCP_UP = 0.067  # grip height above the pad origin (the tab's upper two thirds)
CARRY_Z = TOP + 0.16  # TCP height while carrying a pad
GAP = -0.03  # finger joint when opened to let go of the 4 cm tab (60 mm between the pads)
GRIP_YAW = {"pad_a": -math.pi / 2, "pad_b": math.pi / 2}  # gripper yaw at the pick; it ends at 0, a quarter turn the way that turns the lining to the rotor
POST_OFFSET = (-0.07, 0.04)  # where the gripper holds the post: from the hinge, along the closed bridge and above it


def _origin(ctx, name) -> np.ndarray:
    s = ctx.obj(name)
    o = getattr(s, "origin", None)
    return np.array(o if o is not None else SPECS[name].pos, float)


def _lining_yaw(ctx, name) -> float:
    r = ctx.obj(name).rotation()
    return math.atan2(r[1, 0], r[0, 0])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _post_pos(j: float) -> np.ndarray:
    """Where the held point of the post is when the caliper has closed j rad."""
    ang = -OPEN + j
    y_c, z_c = POST_OFFSET
    return np.array([BX, HINGE_Y + y_c * math.cos(ang) - z_c * math.sin(ang),
                     TOP + HINGE_Z + y_c * math.sin(ang) + z_c * math.cos(ang)])


def _seat_pad(ctx, arm, name, slot_x, home=False):
    """Pick the pad by the long side of its tab, carry it over its slots, turning it a quarter turn, then lower it on live
    pad feedback until it sits on the floor."""
    target_yaw = 0.0 if slot_x < BX else math.pi  # lining direction wanted
    g_yaw = 0.0
    g0 = GRIP_YAW[name]
    org = _origin(ctx, name)
    grasp = np.array([org[0], org[1], org[2] + TCP_UP])
    yield from arm.move((grasp[0], grasp[1], CARRY_Z), 2.5, FINGER_OPEN, yaw=g0)
    yield from arm.move(grasp + np.array([0.0, 0.0, 0.03]), 1.5, FINGER_OPEN, yaw=g0)
    yield from arm.move(grasp, 1.0, FINGER_OPEN, yaw=g0)
    yield from arm.trim_to_measured(grasp)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)
    yield from arm.move((grasp[0], grasp[1], CARRY_Z), 1.8, FINGER_CLOSED, yaw=g0)
    yield from arm.move((slot_x, BY, CARRY_Z), 3.2, FINGER_CLOSED, yaw=g_yaw)
    ox = oy = dyaw = 0.0
    z = CARRY_Z
    z_goal = TOP + 0.035 + TCP_UP - 0.003
    settle = 0
    for k in range(900):
        o = _origin(ctx, name)
        in_hand = bool(np.hypot(*(o[:2] - arm.pos[:2])) < 0.04)
        if in_hand:  # correct the pad's place from where it is
            ox = float(np.clip(ox + 0.3 * (slot_x - o[0]), -0.02, 0.02))
            oy = float(np.clip(oy + 0.3 * (BY - o[1]), -0.02, 0.02))
            dyaw = float(np.clip(dyaw + 0.3 * _wrap(target_yaw - _lining_yaw(ctx, name)), -0.25, 0.25))
        if k >= 20:  # align first, then go down
            speed = 0.02 if z > z_goal + 0.045 else 0.01
            z = max(z_goal, z - speed * ctx.dt)
        arm._set_cartesian(np.array([slot_x + ox, BY + oy, z]), grip_rot(yaw=g_yaw + dyaw), FINGER_CLOSED)
        yield
        if z <= z_goal + 1e-9 or (in_hand and o[2] - (TOP + FLOOR) < 0.0015):
            settle += 1
            if settle > 10:
                break
    yield from arm.hold(0.3)
    yield from arm.grip(GAP, 0.8)
    yield from arm.hold(0.4)
    here = arm.pos.copy()
    yield from arm.move((here[0], here[1], CARRY_Z), 1.8, GAP, rot=arm.rot.copy())
    if home:
        yield from arm.home(4.0)


def _swing_caliper(ctx, arm):
    """Take the red post with the fingers closing along x, then bring it round its arc until the caliper is closed."""
    pitch, yaw = 0.4, -math.pi / 2
    start = _post_pos(0.0)
    yield from arm.move(start + np.array([0.0, 0.0, 0.10]), 3.0, -0.02, yaw=yaw, pitch=pitch)
    yield from arm.move(start, 1.5, -0.02, yaw=yaw, pitch=pitch)
    yield from arm.trim_to_measured(start)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)
    live = hasattr(ctx.obj("brake"), "joints") and ctx.obj("brake").joints.get("caliper") is not None
    steps = round(8.0 / ctx.dt)
    j = 0.0
    for k in range(1, steps + 1):
        s = k / steps
        plan = (OPEN + 0.03) * (s * s * (3 - 2 * s))
        meas = ctx.obj("brake").joints.get("caliper", 0.0)
        if live and meas > 0.0:
            j = min(plan, meas + 0.15)  # never run far ahead of the caliper
            j = max(j, meas)
        else:
            j = plan
        arm._set_cartesian(_post_pos(min(j, OPEN + 0.03)), grip_rot(yaw=yaw, pitch=pitch), FINGER_CLOSED)
        yield
    for _ in range(int(8.0 / ctx.dt)):  # servo the last bit until the stop is reached
        if not live or ctx.obj("brake").joints.get("caliper", 0.0) >= OPEN - 0.02:
            break
        arm._set_cartesian(_post_pos(OPEN + 0.03), grip_rot(yaw=yaw, pitch=pitch), FINGER_CLOSED)
        yield
    yield from arm.hold(0.4)
    yield from arm.grip(FINGER_OPEN, 0.8)
    here = arm.pos.copy()
    yield from arm.move(here + np.array([0.0, 0.0, 0.06]), 2.0, FINGER_OPEN, yaw=yaw, pitch=pitch)
    yield from arm.home(4.0)


def demo_script(ctx):
    """The right arm stands pad_a in the outer slots and then pad_b in the inner slots (each turned a quarter turn the way
    that puts its lining toward the rotor), then the left arm swings the caliper down by its red post. Everything is set
    from the side: no arm part goes below 13 cm over the rotor."""
    yield from _seat_pad(ctx, ctx.right, "pad_a", SLOT_CENTRES[1])  # the outer pair first: the inner pad is carried in after
    yield from _seat_pad(ctx, ctx.right, "pad_b", SLOT_CENTRES[0], home=True)
    yield from _swing_caliper(ctx, ctx.left)
    yield from ctx.wait(8.5)  # both arms parked and open: the end state holds for SUCCESS_HOLD_S
