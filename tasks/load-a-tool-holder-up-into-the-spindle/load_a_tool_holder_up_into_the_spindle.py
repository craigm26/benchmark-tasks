"""On the station table: a small mill's spindle head hangs on a grey column at the back, its square bore opening
downward, with two black drive keys under its face and an orange release tab on its left side. A blue tool holder with
a yellow flange (two key slots in its edge) stands in a grey cup in front of the right arm, turned a quarter turn from
the keys. The holder goes up into the bore with its slots on the keys while the release is held out, and must then
hang on the latch when the release is let go. The holder is loose; the mill and the cup are fixed (the release tab
slides on a spring).

Requires -E object_jitter=0: the holder's start pose decides the scored positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_a_tool_holder_up_into_the_spindle.py -E object_jitter=0 \\
           --instruction "Load the blue tool holder into the mill spindle." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
from belimits import PeakTracker  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: let the release show before the episode ends

GREY, DARK, BLACK = (0.50, 0.52, 0.57), (0.28, 0.28, 0.31), (0.06, 0.06, 0.07)
ORANGE, BLUE, YELLOW, LIGHT = (0.98, 0.45, 0.02), (0.10, 0.27, 0.88), (0.96, 0.82, 0.05), (0.88, 0.88, 0.90)

# -- the mill ---------------------------------------------------------------------------------------------------------------
# Fixed. The Mech sits at the table origin, so shape positions are station x, y and z above the table top.
BX, BY = 0.28, -0.02  # the bore's axis
FACE, CEIL = 0.22, 0.258  # the spindle face (underside of the head) and the bore's ceiling, above the table top
HEAD_X, HEAD_Y = (0.25, 0.31), (-0.05, 0.01)
BORE_HW = 0.017  # 3.4 cm square bore
WALL_X = (BX - BORE_HW, BX + BORE_HW)
WALL_Y = (BY - BORE_HW, BY + BORE_HW)  # -0.037 .. -0.003
KEY_WIDE = (0.252, 0.262, 0.016)  # x0, x1, width (y): on the -x side
KEY_NARROW = (0.298, 0.308, 0.010)  # on the +x side
KEY_DROP = 0.008
TONGUE_Z = (0.238, 0.248)
TONGUE_HW = 0.005  # 1.0 cm wide (x)
TONGUE_REACH = 0.018  # the tongue's tip is 1.8 cm into the bore at rest, just past its axis, so a holder hanging on it stays level
TONGUE_TIP_Y = WALL_Y[1] - TONGUE_REACH
WIN_HW, WIN_Z = 0.009, (0.234, 0.252)  # the window in the +y wall the tongue slides through (1.8 x 1.8 cm)
PULL = 0.020  # the tongue clears the bore at 1.8 cm; the demo pulls 2.0 cm
PULL_STOP = 0.0225
TAB_Y = (0.016, 0.034)
TAB_Z = (0.24, 0.27)
TAB_HW = 0.009

_J = (BX, TONGUE_TIP_Y, 0.243)  # the latch joint's origin: the tongue's tip, mid height


def _mill_shapes():
    s = [
        box((0.35, BY, 0.145), (0.04, 0.06, 0.29), GREY),  # column x 0.33..0.37
        box((0.32, BY, 0.255), (0.02, 0.06, 0.07), GREY),  # arm x 0.31..0.33, z 0.22..0.29
        box((BX, BY, (CEIL + 0.29) / 2), (0.06, 0.06, 0.29 - CEIL), GREY),  # the head above the bore
        box(((HEAD_X[0] + WALL_X[0]) / 2, BY, (FACE + CEIL) / 2), (WALL_X[0] - HEAD_X[0], 0.06, CEIL - FACE), GREY),
        box(((WALL_X[1] + HEAD_X[1]) / 2, BY, (FACE + CEIL) / 2), (HEAD_X[1] - WALL_X[1], 0.06, CEIL - FACE), GREY),
        box((BX, (HEAD_Y[0] + WALL_Y[0]) / 2, (FACE + CEIL) / 2), (WALL_X[1] - WALL_X[0], WALL_Y[0] - HEAD_Y[0], CEIL - FACE), GREY),
    ]
    yw = (WALL_Y[1] + HEAD_Y[1]) / 2
    ys = HEAD_Y[1] - WALL_Y[1]
    xw = WALL_X[1] - WALL_X[0]
    s.append(box((BX, yw, (FACE + WIN_Z[0]) / 2), (xw, ys, WIN_Z[0] - FACE), GREY))  # +y wall round the window
    s.append(box((BX, yw, (WIN_Z[1] + CEIL) / 2), (xw, ys, CEIL - WIN_Z[1]), GREY))
    side = (xw - 2 * WIN_HW) / 2
    for sx in (-1, 1):
        s.append(box((BX + sx * (WIN_HW + side / 2), yw, (WIN_Z[0] + WIN_Z[1]) / 2), (side, ys, WIN_Z[1] - WIN_Z[0]), GREY))
    for x0, x1, w in (KEY_WIDE, KEY_NARROW):  # the two drive keys hang under the face
        s.append(box(((x0 + x1) / 2, BY, FACE - KEY_DROP / 2), (x1 - x0, w, KEY_DROP), BLACK))
    return s


_ROD_LEN = TAB_Y[0] - TONGUE_TIP_Y
LATCH = Joint(
    "latch",
    "slide",
    axis="y",
    at=_J,
    low=-0.001,
    high=PULL_STOP,
    spring=50.0,
    damping=3.0,
    mass=0.02,
    shapes=[
        box((0.0, _ROD_LEN / 2, 0.0), (2 * TONGUE_HW, _ROD_LEN, TONGUE_Z[1] - TONGUE_Z[0]), DARK),  # tongue and rod
        box((0.0, _ROD_LEN + (TAB_Y[1] - TAB_Y[0]) / 2, (TAB_Z[0] + TAB_Z[1]) / 2 - _J[2]), (2 * TAB_HW, TAB_Y[1] - TAB_Y[0], TAB_Z[1] - TAB_Z[0]), ORANGE),
    ],
)
MILL = Mech("mill", pos=(0.0, 0.0, TOP), mass=None, friction=0.6, shapes=_mill_shapes(), joints=[LATCH])

# -- the holder -------------------------------------------------------------------------------------------------------------
# Frame origin: the middle of the blue body's foot; z up, +y is the face with the notch. Body z 0..0.07, flange 0.07..0.08,
# shank 0.08..0.118. Keyed, its frame is the world's.
BODY_H, FL_Z, SH_TOP = 0.07, 0.08, 0.118
NOTCH_Z, NOTCH_HX, NOTCH_Y0 = (0.096, 0.110), 0.007, -0.004  # notch 1.4 (x) x 1.8 (y) x 1.4 (z), 0.8 cm under the shank top


def _holder_shapes():
    s = [box((0, 0, BODY_H / 2), (0.024, 0.024, BODY_H), BLUE)]
    # flange 5.6 x 5.6 x 1.0 cm, yellow; slot 2.2 wide on the -x edge, 1.4 wide on the +x edge, 1.2 deep
    s.append(box((0, 0, 0.075), (0.032, 0.056, 0.01), YELLOW))
    for sy in (-1, 1):
        s.append(box((-0.022, sy * (0.011 + 0.017 / 2), 0.075), (0.012, 0.017, 0.01), YELLOW))
        s.append(box((0.022, sy * (0.007 + 0.021 / 2), 0.075), (0.012, 0.021, 0.01), YELLOW))
    # shank 2.8 x 2.8 x 3.8 cm, light grey, with the notch in its +y face (1.8 cm deep, to just past the axis)
    s.append(box((0, (-0.014 + NOTCH_Y0) / 2, (FL_Z + SH_TOP) / 2), (0.028, NOTCH_Y0 + 0.014, SH_TOP - FL_Z), LIGHT))
    yn, ly = (NOTCH_Y0 + 0.014) / 2, 0.014 - NOTCH_Y0
    s.append(box((0, yn, (FL_Z + NOTCH_Z[0]) / 2), (0.028, ly, NOTCH_Z[0] - FL_Z), LIGHT))
    s.append(box((0, yn, (NOTCH_Z[1] + SH_TOP) / 2), (0.028, ly, SH_TOP - NOTCH_Z[1]), LIGHT))
    for sx in (-1, 1):
        s.append(box((sx * (NOTCH_HX + (0.014 - NOTCH_HX) / 2), yn, (NOTCH_Z[0] + NOTCH_Z[1]) / 2), (0.014 - NOTCH_HX, ly, NOTCH_Z[1] - NOTCH_Z[0]), LIGHT))
    return s


CUP_XY = (0.24, -0.13)
CUP_TOP = 0.006
HOLDER_START = (CUP_XY[0], CUP_XY[1], TOP + 0.0022)
START_YAW = -math.pi / 2  # a quarter turn from the keys
HOLDER = Mech("holder", pos=HOLDER_START, yaw=START_YAW, mass=0.08, friction=1.2, shapes=_holder_shapes())


def _cup_shapes():
    cx, cy = CUP_XY
    w = 0.005  # 4 cm outer, 3 cm pocket
    s = [box((cx, cy, 0.001), (0.04, 0.04, 0.002), DARK)]
    for sx in (-1, 1):
        s.append(box((cx + sx * (0.015 + w / 2), cy, CUP_TOP / 2 + 0.001), (w, 0.04, CUP_TOP - 0.002), DARK))
        s.append(box((cx, cy + sx * (0.015 + w / 2), CUP_TOP / 2 + 0.001), (0.03, w, CUP_TOP - 0.002), DARK))
    return s


CUP = Mech("cup", pos=(0.0, 0.0, TOP), mass=None, friction=0.6, shapes=_cup_shapes())
OBJECTS = [MILL, CUP, HOLDER]

# -- scoring ----------------------------------------------------------------------------------------------------------------
FACE_Z = TOP + FACE
FLANGE_TOL = 0.003  # flange top within 3 mm of the spindle face
YAW_TOL = 4.0  # degrees from keyed
TILT_TOL = 3.0  # degrees from upright
BORE_TOL = 0.005  # holder axis within 5 mm of the bore's axis
DEEP_Z = TOP + FACE + 0.010  # shank top at least 1 cm up inside the bore
REST_TOL = 0.002  # latch within 2 mm of rest
HANG_MIN = 0.001  # hanging on the tongue: the flange top 1 to 3 mm under the face (a holder held up against the face sits nearer)


def _tilt(h) -> float:
    return float(np.degrees(np.arccos(np.clip(h.rotation()[2, 2], -1.0, 1.0))))


def _yaw_err(h) -> float:
    r = h.rotation()
    return abs(float(np.degrees(np.arctan2(r[1, 0], r[0, 0]))))


def _on_axis(h) -> bool:
    o = np.asarray(h.origin, float)
    return bool(abs(o[0] - BX) <= BORE_TOL and abs(o[1] - BY) <= BORE_TOL)


def _flange_gap(h) -> float:
    """Metres the flange's top face is below the spindle face (negative: above)."""
    return float(FACE_Z - h.to_world((0.0, 0.0, FL_Z))[2])


def _deep(h) -> bool:
    return bool(h.to_world((0.0, 0.0, SH_TOP))[2] >= DEEP_Z and _on_axis(h) and _tilt(h) <= 10.0)


def _seated(h) -> bool:
    return bool(
        abs(_flange_gap(h)) <= FLANGE_TOL and _on_axis(h) and _yaw_err(h) <= YAW_TOL and _tilt(h) <= TILT_TOL
    )


def _latched(h, mill) -> bool:
    j = mill.joints["latch"]
    if abs(j) > REST_TOL:
        return False
    p = h.to_local((BX, TONGUE_TIP_Y + j + 0.002, 0.5 * (TONGUE_Z[0] + TONGUE_Z[1]) + TOP))  # 2 mm behind the tip
    return bool(
        abs(p[0]) + TONGUE_HW <= NOTCH_HX + 0.0005  # the whole tongue width inside the notch
        and NOTCH_Y0 - 0.0005 <= p[1] <= 0.014 + 0.0005
        and NOTCH_Z[0] - 0.0005 <= p[2] <= NOTCH_Z[1] + 0.0005
    )


def _hanging(h, mill) -> bool:
    return bool(_seated(h) and _latched(h, mill) and HANG_MIN <= _flange_gap(h) <= FLANGE_TOL and h.resting())


def _bbox_centre(spec):
    from mechkit import _local_corners

    c = _local_corners(spec, {})
    return tuple(np.asarray(spec.pos, float) + (c.max(0) + c.min(0)) / 2)


def _new_tracker():
    return PeakTracker({"holder": _bbox_centre(HOLDER)}, home_tol=0.01)


TRACK = _new_tracker()


@on_reset
def _reset_track(env=None):
    global TRACK
    TRACK = _new_tracker()


def score(objects):
    """Kept over the whole run (flags stay set) and judged at the end.
    1.0: the holder hangs from the latch: flange top 1 to 3 mm under the spindle face, yaw within 4 degrees of keyed, tilt
    within 3, the release at rest (within 2 mm) with the tongue in the notch, and the holder still.
    0.5: the holder was seated at any time (flange top within 3 mm of the face, yaw within 4, tilt within 3).
    0.25: the shank was at least 1 cm up inside the bore at any time.
    Else 0. A holder held up against the face has no 1 mm drop, so it never scores more than it would if released."""
    h, mill = objects["holder"], objects["mill"]
    peaks = TRACK.update(objects, {"deep": float(_deep(h)), "seated": float(_seated(h))})
    if _hanging(h, mill):
        return 1.0
    if peaks["seated"] >= 1.0:
        return 0.5
    return 0.25 if peaks["deep"] >= 1.0 else 0.0


def success(objects):
    """The holder hangs from the latch, still."""
    return _hanging(objects["holder"], objects["mill"])


# The robot keeps clear of the mill's column and arm (the scorer sees only the objects), the fingers counted at full opening;
# the head itself is reached under (right hand, below TOP+0.20) and beside (left hand, outside the head's side) by design.
ARM_KEEP_OUT = [
    {"name": "mill column and arm", "x": (0.33, 0.41), "y": (-0.09, 0.05), "z_top": TOP + 0.31},
]

# -- the scripted demo -----------------------------------------------------------------------------------------------------
demo = placeholder_demo

FO = FINGER_OPEN * 0.6  # fingers open 5.6 cm: clears the 2.4 cm body, the 1.8 cm tab and the column
FC = 0.0
GRIP_UP = 0.025  # the right TCP grips the body 2.5 cm above its foot
CARRY_Z = TOP + 0.082  # holder origin height while carried: shank top 2 cm under the face, flange under the keys
TAB_TCP_Y = 0.028  # left TCP on the tab (pad tips stay 5 mm off the head's face)
TAB_TCP_Z = TOP + 0.255


def _rot_about(axis, ang):
    return axis_angle_matrix(np.asarray(axis, float), float(ang))


def demo_script(ctx):
    """Right arm takes the holder from the cup by its blue body (fingers 20 degrees below level, closing along y, then x), turns it a quarter turn
    while carrying it to under the bore, and pushes it up on its measured pose. The left arm takes the release tab, pulls it
    2.0 cm and holds it while the holder goes up, then eases it back so the tongue lands in the notch. Both then open and park.
    Every move runs well under 30 cm/s."""
    left, right = ctx.left, ctx.right
    SHANK = (0.0, 0.0, 0.11)  # a point mid-way up the shank: the part that has to meet the bore
    pitch = math.radians(70)  # right hand: fingers 20 degrees below level (a level hand cannot reach the cup)
    DT = ctx.dt

    def S(sec):
        return max(1, round(sec / DT))

    def holder():
        return ctx.obj("holder")

    def latch():
        return float(ctx.obj("mill").joints.get("latch", 0.0))

    # ---- 1. right arm: take the holder by its body, hand level pointing +x
    h0 = np.asarray(holder().origin, float)
    grip = h0 + np.array([0.0, 0.0, GRIP_UP])
    above = grip - 0.10 * grip_rot(yaw=0.0, pitch=pitch)[:, 2]  # 10 cm back along the fingers' line, clear of the flange
    yield from right.move(above, duration=6.0, finger=FO, yaw=0.0, pitch=pitch)
    yield from right.trim_to_measured(above, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = right.pos - above
    yield from right.move(grip + c, duration=4.0, finger=FO, yaw=0.0, pitch=pitch)
    yield from right.trim_to_measured(grip, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from right.grip(FC, duration=0.9)
    yield from right.hold(0.3)
    hand0 = right.rot.copy()
    off_hand = hand0.T @ (np.asarray(holder().origin, float) - right.pos)  # holder origin in the hand's frame
    hand_yaw = [0.0]
    dtilt = [0.0]

    def hand_rot(yaw):
        base = grip_rot(yaw=yaw + hand_yaw[0], pitch=pitch)
        return _rot_about(base[:, 0], dtilt[0]) @ base

    def track(want_fn, steps, yaw_fn, gain=0.25, cap=0.0006, max_comp=0.035, mask=(1, 1, 1), stop=None, level=True, trim_yaw=False, pt=(0.0, 0.0, 0.0)):
        """Closed loop on the holder: every step the hand goes to where the holder's wanted origin ``want_fn(i)`` puts the
        TCP, plus an offset that grows with the live error, so sag and slip are trimmed out."""
        comp = np.zeros(3)
        mask = np.asarray(mask, float)
        pt_l = np.asarray(pt, float)
        for i in range(steps):
            if stop is not None and stop():
                break
            h = holder()
            want = np.asarray(want_fn(i), float)
            err = (want + pt_l - h.to_world(pt_l)) * mask  # error of the point ``pt`` of the holder (default: its foot)
            if float(np.linalg.norm(err)) < 0.04:  # a bigger error is a stale or missing reading: leave the trim alone
                comp = np.clip(comp + np.clip(gain * err, -cap, cap), -max_comp, max_comp)
            yaw = yaw_fn(i)
            if trim_yaw:
                r = h.rotation()
                psi = math.atan2(r[1, 0], r[0, 0])
                if abs(psi) < 0.3:  # a bigger yaw error is a stale reading: leave the trim alone
                    hand_yaw[0] = float(np.clip(hand_yaw[0] - 0.1 * psi, -0.12, 0.12))
            if level:
                a = grip_rot(yaw=yaw + hand_yaw[0], pitch=pitch)[:, 0]
                u = h.rotation()[:, 2]
                th = math.atan2(float(u @ np.cross(a, [0.0, 0.0, 1.0])), float(u[2]))
                dtilt[0] = float(np.clip(dtilt[0] - 0.3 * th, -0.2, 0.2))
            rot = hand_rot(yaw)
            nominal = want - rot @ off_hand
            right._set_cartesian(nominal + comp, rot, right.finger)
            yield

    # ---- 2. lift high (the hand turns freely up there), turn a quarter turn, then lower in front of the bore
    HI = TOP + 0.175  # holder origin height for the turn: well above the cup, 10 cm in front of the head's side
    up = np.array([h0[0], h0[1], HI])
    turn_at = np.array([0.26, BY - 0.10, HI])
    n = S(3.0)
    yield from track(lambda i: h0 + (up - h0) * ((i + 1) / n), n, lambda i: 0.0)
    n = S(8.0)
    yield from track(
        lambda i: up + (turn_at - up) * ((i + 1) / n),
        n,
        lambda i: (math.pi / 2) * ((i + 1) / n),
        mask=(1, 1, 0),
    )
    mid = np.array([BX, BY - 0.09, CARRY_Z])
    n = S(5.0)
    yield from track(lambda i: turn_at + (mid - turn_at) * ((i + 1) / n), n, lambda i: math.pi / 2, mask=(1, 1, 0))
    yield from track(lambda i: mid, S(2.0), lambda i: math.pi / 2, mask=(1, 1, 1))
    final = np.array([BX, BY, CARRY_Z])
    n = S(6.0)  # slide in under the bore at 1.5 cm/s
    yield from track(lambda i: mid + (final - mid) * ((i + 1) / n), n, lambda i: math.pi / 2, mask=(1, 1, 1), pt=SHANK)

    # ---- 3. left arm: take the release tab, pointing -y, and pull it out 2.0 cm
    t_grip = np.array([BX, TAB_TCP_Y + latch(), TAB_TCP_Z])
    t_pre = t_grip + np.array([0.0, 0.09, 0.0])
    yield from left.move(t_pre, duration=5.0, finger=FO, yaw=-math.pi / 2, pitch=math.pi / 2)
    yield from left.trim_to_measured(t_pre, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = left.pos - t_pre
    yield from left.move(t_grip + c, duration=4.0, finger=FO, yaw=-math.pi / 2, pitch=math.pi / 2)
    yield from left.trim_to_measured(t_grip, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from left.grip(FC, duration=0.9)
    yield from left.hold(0.3)
    anchor = left.pos.copy()
    hand_l = left.rot.copy()
    y0 = float(anchor[1])
    for i in range(S(4.0)):  # pull at 0.8 cm/s until the release has moved 2.0 cm (the stop is at 2.25)
        if latch() >= PULL - 0.0003:
            break
        anchor = anchor + np.array([0.0, 0.008 * DT, 0.0])
        anchor[1] = min(anchor[1], y0 + PULL_STOP + 0.004)
        left._set_cartesian(anchor, hand_l, FC)
        yield
    yield from left.hold(0.5)

    # ---- 4. right arm: steer the holder onto the bore's axis, then push it straight up on its measured pose
    def under(i):
        return np.array([BX, BY, CARRY_Z])

    yield from track(under, S(4.0), lambda i: math.pi / 2, gain=0.3, trim_yaw=True, pt=SHANK)
    z_seat = FACE - FL_Z  # holder origin height above the table when the flange top meets the face: 0.14
    z_target = TOP + z_seat

    def seated_now():
        return _flange_gap(holder()) <= 0.001

    fast_end = TOP + z_seat - 0.045  # flange top 4.5 cm under the face: the shank top still 7 mm short of the bore
    n1 = S(2.0)
    yield from track(
        lambda i: np.array([BX, BY, CARRY_Z + (fast_end - CARRY_Z) * ((i + 1) / n1)]), n1, lambda i: math.pi / 2, gain=0.3, trim_yaw=True, pt=SHANK
    )
    yield from track(lambda i: np.array([BX, BY, fast_end]), S(3.0), lambda i: math.pi / 2, gain=0.3, trim_yaw=True, pt=SHANK)
    zs = [fast_end]

    def climb(i):
        """Up at 0.5 cm/s, but only while the shank is on the bore's axis (within 1.5 mm) and the hand is not being held back."""
        h = holder()
        on_axis = np.hypot(*(np.asarray(h.to_world(SHANK))[:2] - np.array([BX, BY]))) < 0.0015
        lag = zs[0] - float(h.origin[2])
        if on_axis and lag < 0.004:
            zs[0] += 0.005 * DT
        zs[0] = min(zs[0], float(h.origin[2]) + 0.006)
        return np.array([BX, BY, zs[0]])

    yield from track(climb, S(24.0), lambda i: math.pi / 2, gain=0.3, trim_yaw=True, stop=seated_now, pt=SHANK)
    yield from right.hold(0.6)

    # ---- 5. left arm: ease the release back; the tongue lands in the notch
    anchor = left.pos.copy()
    y_floor = y0 - 0.001
    for i in range(S(4.0)):
        if latch() <= 0.0004:
            break
        anchor = anchor + np.array([0.0, -0.013 * DT, 0.0])
        anchor[1] = max(anchor[1], y_floor)
        left._set_cartesian(anchor, hand_l, FC)
        yield
    yield from left.hold(0.6)

    # ---- 6. right arm lets go (the holder drops 2 mm onto the tongue and hangs); then both back off and park together, inside
    # the 8 s the run holds after success, so the video shows both hands open and clear
    yield from right.grip(FO, duration=1.0)
    yield from right.hold(0.4)

    def right_tail():
        yield from right.move(right.pos + np.array([0.0, -0.09, 0.0]), duration=2.0, finger=FO, rot=right.rot.copy())
        yield from right.home(duration=3.0)

    def left_tail():
        yield from left.hold(0.3)
        yield from left.grip(FO, duration=0.8)
        yield from left.move(left.pos + np.array([0.0, 0.05, 0.0]), duration=2.0, finger=FO, rot=left.rot.copy())
        yield from left.home(duration=3.0)

    yield from ctx.both(left_tail(), right_tail())
    yield from ctx.wait(1.0)
