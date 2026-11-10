"""On the station table: a bowl-lift mixer stands at the far end between the arms. A dark column carries a head that
overhangs the work area, with a white beater hanging from it. In front of the column a grey lift frame (two arms joined at
the back, open at the front) carries two orange pins and, on its left side, a red lift handle. A grey guide with a 2 cm
channel is fixed on the column face. To the right of the column a dark post carries a blue lock bolt that slides under a
tab on the raised frame. A steel-grey bowl with two dark ears (each with a hole), an orange lug on its back wall
and a green grip tab on its front stands on a dark pad in front of the right arm. The ears go over the pins; the lift
handle then raises the frame so the lug rises into the guide, and the bolt is slid in under the frame's tab so the frame
rests on it. The bowl, the lift frame and the bolt are loose; the rest is fixed.

Requires -E object_jitter=0: the pins', the guide's, the bolt's and the tab's positions decide the scored poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=set_a_mixer_bowl_on_its_pins_and_raise_it.py -E object_jitter=0 \\
           --instruction "Put the steel bowl on the mixer, raise it into mixing position and lock the lift." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
from belimits import ResetOnJumpTracker, tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402
from yam_arena import ik  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 12.0  # the demo grips: let the release show, and the frame prove it rests on the bolt, before the episode ends

DARK, CHAR = (0.17, 0.17, 0.20), (0.26, 0.26, 0.30)
GREY, LIGHT = (0.50, 0.52, 0.56), (0.80, 0.82, 0.86)
WHITE, ORANGE, RED, GREEN = (0.97, 0.97, 0.95), (0.98, 0.45, 0.02), (0.88, 0.10, 0.10), (0.10, 0.72, 0.25)
BLUE = (0.08, 0.30, 0.92)

# -- the mixer (fixed) and its lift frame -----------------------------------------------------------------------------------
# The Mech sits at the table origin: shape positions are station x, y and z above the table top.
PIN_X, PIN_Y = 0.27, 0.06  # the pins' axes (the right one at -PIN_Y)
ARM_TOP = 0.045  # the lift frame's arm tops, above the table top, with the lift at 0
PIN_TOP = 0.060
LIFT_MAX = 0.115  # the lift's travel; on the lock bolt the frame rests at REST_J
RAISED_J = 0.100  # raised: handle 10.0 cm or more up
REST_J = 0.104  # the frame's height when its tab rests on the lock bolt (bolt top 10.9 cm, tab underside 0.5 cm at rest)
LET_DOWN_J = 0.1065  # the frame counts as let down on the bolt at or below this; the demo raises it to its stop (0.115) first
GUIDE_Z0, GUIDE_Z1 = 0.15, 0.22  # the guide rails, above the table top
CHANNEL_HALF = 0.010  # the channel is 2.0 cm wide
HANDLE = (0.27, 0.14, 0.0575)  # centre of the red lift handle at lift 0 (x, y, z above the table top)


# The lock: a separate fixed object, so the frame's tab rests on another object's solid bar (never on a catch that has to
# engage after the hand lets go, and never on a part of the frame's own object). The bolt slides along y only: gravity
# does not move it, and its slide carries the frame's weight.
BOLT_X = 0.345
BOLT_TRAVEL = 0.036  # the bolt's travel toward the column (+y)
BOLT_IN = 0.026  # locked: the bolt at least this far in (2.0 cm or more of bar under the 3.0 cm tab)
BOLT_TOP = 0.109  # the bar's top, above the table top
TAB_Y0, TAB_Y1 = -0.086, -0.056  # the frame's lock tab (y); its underside is 0.5 cm above the table with the lift at 0
KNOB = (BOLT_X, -0.176, 0.1315)  # centre of the blue knob with the bolt out (x, y, z above the table top)

MIXER = Mech(
    "mixer",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.8,
    shapes=[
        box((0.35, 0.0, 0.14), (0.04, 0.10, 0.28), DARK),  # column x 0.33..0.37
        box((0.305, 0.0, 0.26), (0.13, 0.10, 0.04), DARK),  # head x 0.24..0.37, z 0.24..0.28
        box((0.27, 0.0, 0.1925), (0.03, 0.01, 0.095), WHITE),  # beater 3.0 x 1.0 x 9.5 cm, bottom at z 0.145 (7 mm over the bowl floor at the stop)
        box((0.36, 0.0, 0.01), (0.06, 0.10, 0.02), DARK),  # foot, no wider than the column: the lock tab passes beside it
        box((0.325, 0.02, 0.185), (0.01, 0.02, 0.07), GREY),  # guide rails x 0.32..0.33, channel 2.0 cm between them
        box((0.325, -0.02, 0.185), (0.01, 0.02, 0.07), GREY),
    ],
    joints=[
        Joint(
            "lift",
            "slide",
            axis="z",
            at=(0.0, 0.0, 0.0),
            low=0.0,
            high=LIFT_MAX,
            spring=0.0,
            damping=10.0,
            mass=0.06,
            friction=0.5,
            shapes=[
                box((0.279, 0.06, 0.025), (0.078, 0.02, 0.04), GREY),  # arms x 0.24..0.318, tops at z 0.045
                box((0.279, -0.06, 0.025), (0.078, 0.02, 0.04), GREY),
                box((0.323, 0.0, 0.02), (0.01, 0.14, 0.03), GREY),  # joined at the back
                box((PIN_X, PIN_Y, 0.0525), (0.008, 0.008, 0.015), ORANGE),  # pins 0.8 x 0.8 x 1.5 cm
                box((PIN_X, -PIN_Y, 0.0525), (0.008, 0.008, 0.015), ORANGE),
                box((0.27, 0.095, 0.0375), (0.02, 0.05, 0.015), RED),  # bracket y 0.07..0.12, out to the handle
                box(HANDLE, (0.02, 0.04, 0.055), RED),  # lift handle 2.0 x 4.0 x 5.5 cm
                # lock tab behind the right arm, beside the column: x 0.318..0.360, y -0.086..-0.056, z 0.005..0.020
                box((0.339, (TAB_Y0 + TAB_Y1) / 2, 0.0125), (0.042, TAB_Y1 - TAB_Y0, 0.015), RED),
            ],
        ),
    ],
)

# -- the bowl ---------------------------------------------------------------------------------------------------------------
# Frame origin: the middle of the bowl's outer floor, on its underside. x runs toward the mixer, z up.
# 9 x 9 x 8 cm, walls 0.8 cm. Ears 4.0 (x) x 3.0 (y) x 1.0 cm with a 1.4 cm square hole at y +-0.06, undersides 3.0 cm up.
# Lug on the back wall, 4 to 7 cm up. Grip tab on the front wall at rim height, 7.0 (x) x 2.4 (y) x 3.0 (z).
EAR_Y, EAR_Z0, EAR_Z1 = 0.06, 0.030, 0.040
LUG_LOCAL = (0.051, 0.0, 0.055)  # centre of the lug
TAB_X = -0.085  # where the gripper's TCP sits on the tab (x), 4 cm out from the front wall
TAB_Z = 0.065
WALL, RIM = 0.008, 0.08


def _ear(s):
    y = s * EAR_Y
    zc, h = (EAR_Z0 + EAR_Z1) / 2, EAR_Z1 - EAR_Z0
    return [
        box((0.0, s * 0.071, zc), (0.04, 0.008, h), CHAR),
        box((0.0, s * 0.049, zc), (0.04, 0.008, h), CHAR),
        box((-0.0135, y, zc), (0.013, 0.014, h), CHAR),
        box((0.0135, y, zc), (0.013, 0.014, h), CHAR),
    ]


BOWL_START = (0.25, -0.17, TOP + 0.01)
BOWL = Mech(
    "bowl",
    pos=BOWL_START,
    mass=0.14,
    friction=1.2,
    shapes=[
        box((0.0, 0.0, 0.004), (0.09, 0.09, 0.008), LIGHT),  # floor
        box((-0.041, 0.0, 0.008 + (RIM - 0.008) / 2), (WALL, 0.09, RIM - 0.008), LIGHT),  # front wall
        box((0.041, 0.0, 0.008 + (RIM - 0.008) / 2), (WALL, 0.09, RIM - 0.008), LIGHT),  # back wall
        box((0.0, 0.041, 0.008 + (RIM - 0.008) / 2), (0.074, WALL, RIM - 0.008), LIGHT),
        box((0.0, -0.041, 0.008 + (RIM - 0.008) / 2), (0.074, WALL, RIM - 0.008), LIGHT),
        *_ear(1),
        *_ear(-1),
        box(LUG_LOCAL, (0.012, 0.008, 0.03), ORANGE),  # lug 1.2 x 0.8 x 3.0 cm: 6 mm a side in the 2.0 cm channel, 3 mm with the bowl at the edge of its pin slack
        box((-0.080, 0.0, 0.065), (0.07, 0.024, 0.03), GREEN),  # grip tab: x -0.115..-0.045
    ],
)
PAD = Box("start_pad", pos=(BOWL_START[0], BOWL_START[1], TOP + 0.005), size=(0.11, 0.11, 0.01), color=CHAR)

LOCK = Mech(
    "lock",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.8,
    shapes=[
        box((BOLT_X, -0.135, 0.0445), (0.024, 0.03, 0.089), DARK),  # post x 0.333..0.357, y -0.15..-0.12, top at z 0.089
    ],
    joints=[
        Joint(
            "bolt",
            "slide",
            axis="y",
            at=(BOLT_X, -0.144, BOLT_TOP - 0.0075),
            low=0.0,
            high=BOLT_TRAVEL,
            spring=0.0,
            damping=8.0,
            mass=0.08,
            friction=0.8,
            shapes=[
                box((0.0, 0.0, 0.0), (0.02, 0.104, 0.015), BLUE),  # bar 2.0 x 10.4 x 1.5 cm: y -0.196..-0.092 when out
                box((0.0, KNOB[1] + 0.144, KNOB[2] - (BOLT_TOP - 0.0075)), (0.02, 0.04, 0.045), BLUE),  # knob 2.0 x 4.0 x 4.5 cm
            ],
        ),
    ],
)

OBJECTS = [PAD, MIXER, LOCK, BOWL]

# -- scoring ----------------------------------------------------------------------------------------------------------------
HOLE_TOL = 0.0045  # a hole's centre within this of its pin (the hole is 3 mm a side looser than the pin)
SEAT_Z = 0.003  # ear undersides within 3 mm of the arm tops
ONE_Z = 0.004
SEAT_TILT = 3.0  # degrees
GUIDE_Y = 0.004  # lug centre within this of the channel's centre line
HOLD_STEPS = 20  # control steps (1 s at 20 Hz)
FLOAT = 0.01  # a part whose underside is more than 1 cm above every support counts as not placed


def _bbox_centre(spec):
    from mechkit import _local_corners

    c = _local_corners(spec, {})
    return tuple(np.asarray(spec.pos, float) + (c.max(0) + c.min(0)) / 2)


def _lift(objects) -> float:
    return float(objects["mixer"].joints.get("lift", 0.0))


def _bolt(objects) -> float:
    return float(objects["lock"].joints.get("bolt", 0.0))


def _ears(bowl, j):
    """For each ear: the distance of its hole's centre from the pin it is over (the nearer pairing of the two ears with the
    two pins, so a bowl turned end for end is judged the same), and its underside's height above the arm top."""
    arm_top = TOP + ARM_TOP + j
    pts = [bowl.to_world((0.0, s * EAR_Y, EAR_Z0)) for s in (-1, 1)]
    pins = [(PIN_X, -PIN_Y), (PIN_X, PIN_Y)]
    d = [[float(np.hypot(p[0] - q[0], p[1] - q[1])) for q in pins] for p in pts]
    same, swap = d[0][0] + d[1][1], d[0][1] + d[1][0]
    dist = [d[0][0], d[1][1]] if same <= swap else [d[0][1], d[1][0]]
    return [(dist[i], float(pts[i][2] - arm_top)) for i in range(2)]


def _airborne(bowl, j) -> bool:
    """Held up: no support within 1 cm. Supports: the table and pad under the floor, the arm tops and pin tips under the
    ears, and, for a bowl left lying somewhere else, the arm tops under a floor corner, the lock's bolt and knob, and the
    head's top."""
    o = bowl.to_world((0.0, 0.0, 0.0))
    fz = float(o[2] - TOP)

    def near(h):
        return -0.005 <= fz - h <= FLOAT

    if fz - 0.01 <= FLOAT:  # on the table or the pad
        return False
    if not all(dz - (PIN_TOP - ARM_TOP) > FLOAT for _, dz in _ears(bowl, j)):  # ears on or just over the arms and pins
        return False
    corners = [bowl.to_world((sx * 0.045, sy * 0.045, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]
    if near(ARM_TOP + j) and any(0.24 <= c[0] <= 0.318 and 0.05 <= abs(c[1]) <= 0.07 for c in corners):
        return False  # lying across the arm tops
    if (near(BOLT_TOP) or near(KNOB[2] + 0.0225)) and abs(o[0] - BOLT_X) <= 0.06 and -0.25 <= o[1] <= -0.01:
        return False  # left on the lock
    if near(0.28) and 0.20 <= o[0] <= 0.41 and abs(o[1]) <= 0.09:
        return False  # left on the head
    return True


def _seated(bowl, j) -> bool:
    return bool(
        all(d <= HOLE_TOL and abs(dz) <= SEAT_Z for d, dz in _ears(bowl, j)) and tilt_deg(bowl) <= SEAT_TILT
    )


def _one_ear(bowl, j) -> bool:
    return any(d <= HOLE_TOL and abs(dz) <= ONE_Z for d, dz in _ears(bowl, j))


def _lug_in_guide(bowl) -> bool:
    p = bowl.to_world(LUG_LOCAL)
    return bool(
        abs(p[1]) <= GUIDE_Y
        and 0.31 <= p[0] <= 0.332
        and TOP + GUIDE_Z0 + 0.013 <= p[2] <= TOP + GUIDE_Z1 - 0.013
        and tilt_deg(bowl) <= 5.0
    )


def _final(objects) -> bool:
    bowl, j = objects["bowl"], _lift(objects)
    return bool(
        _seated(bowl, j)
        and RAISED_J <= j <= LET_DOWN_J
        and _bolt(objects) >= BOLT_IN
        and _lug_in_guide(bowl)
        and bowl.resting()
    )


TRACK = ResetOnJumpTracker({"bowl": _bbox_centre(BOWL)}, home_tol=0.01)
RUN = {"seat": 0, "one": 0, "done": 0}


@on_reset
def _reset_track(env=None):
    global TRACK
    TRACK = ResetOnJumpTracker({"bowl": _bbox_centre(BOWL)}, home_tol=0.01)
    RUN.update(seat=0, one=0, done=0)


def score(objects):
    """Kept over the whole run: 0.25 once one ear has sat on its pin (hole over the pin, underside within 4 mm of the arm
    top) for 1 s; 0.5 once both ears have sat on both pins for 1 s (underside within 3 mm, bowl level within 3 degrees,
    still). 1.0 if it ends seated with the frame let down on the bolt (10.0 to 10.65 cm up), the lock bolt 2.6 cm or more in,
    the lug inside the guide, and still, all for 1 s.
    0 if it ends with no support within 1 cm (held up)."""
    bowl, j = objects["bowl"], _lift(objects)
    still = bowl.resting()
    seated, one = _seated(bowl, j), _one_ear(bowl, j)
    RUN["seat"] = RUN["seat"] + 1 if seated and still else 0
    RUN["one"] = RUN["one"] + 1 if one and still else 0
    RUN["done"] = RUN["done"] + 1 if _final(objects) else 0
    peaks = TRACK.update(objects, {"seat": float(RUN["seat"] >= HOLD_STEPS), "one": float(RUN["one"] >= HOLD_STEPS)})
    if _airborne(bowl, j):
        return 0.0
    if RUN["done"] >= HOLD_STEPS:
        return 1.0
    if peaks["seat"] >= 1.0:
        return 0.5
    return 0.25 if peaks["one"] >= 1.0 else 0.0


def success(objects):
    """The same state score() gives 1.0 for (a frame still held above the bolt at its stop does not count in either):
    this only ends the episode, SUCCESS_HOLD_S later."""
    return _final(objects)


# The robot keeps off the mixer too (the scorer sees only the objects): the column and head with 4 cm round them, and the
# bowl's inside while the bowl sits on the pins.
ARM_KEEP_OUT = [
    {"name": "mixer column and head, 4 cm margin", "x": (0.29, 0.41), "y": (-0.09, 0.09), "z_top": TOP + 0.30},
    {"name": "inside of the seated bowl", "x": (0.235, 0.305), "y": (-0.035, 0.035), "z_top": TOP + 0.25},
]

# -- the scripted demo ------------------------------------------------------------------------------------------------------
demo = placeholder_demo

DT = 0.05
O, C = FINGER_OPEN, FINGER_CLOSED
CARRY_ORIGIN_Z = TOP + 0.06  # bowl floor 1.5 cm over the arm tops; rim at 0.14, just under the beater's tip
UNDER_ORIGIN_Z = TOP + 0.035  # rim at 0.115, 2.5 cm under the beater; ears 5 mm over the pin tips
GRIP_DZ = TAB_Z - 0.003  # the TCP rides just under the tab's middle
PITCH = 0.3  # the right hand leans back 17 degrees so its forearm clears the head
STAGE_X = 0.20  # bowl centre in front of the arm ends and the beater


HAND = {"base": None, "dp": 0.0}  # the right hand's gripped orientation, and the pitch trim that keeps the bowl level
LEVEL_GAIN = 0.15


def _hand_rot(ctx, rot0, level):
    """The hand's orientation: ``rot0`` turned about y by a trim that grows against the bowl's measured pitch, because a
    pinch on the tab lets the bowl creep about the pinch axis. Offline (ctx.step_count 0) there is no trim."""
    if level is None or ctx.step_count == 0:
        return rot0
    HAND["dp"] = float(np.clip(HAND["dp"] - LEVEL_GAIN * level(), -0.35, 0.35))
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -HAND["dp"]) @ rot0


def _chase(ctx, arm, get_err, steps, feed=(0.0, 0.0, 0.0), gain=0.2, step_cap=0.0006, max_comp=0.01, stop=None, lead=None, floor=None, level=None):
    """Closed-loop correction: every step the planned TCP moves on by ``feed`` (m/s) and an offset grows by ``gain`` times
    the live error ``get_err()`` (desired minus measured part position, m), so a constant slip or sag is trimmed out."""
    anchor, comp = arm.pos.copy(), np.zeros(3)
    rot0 = HAND["base"] if HAND["base"] is not None else arm.rot.copy()
    feed = np.asarray(feed, float)
    for _ in range(steps):
        if stop is not None and stop():
            break
        # offline (closedloop.dry_run) the objects never move: ctx.step_count stays 0, so plan the nominal path only
        err = np.asarray(get_err(), float) if ctx.step_count > 0 else np.zeros(3)
        comp = np.clip(comp + np.clip(gain * err, -step_cap, step_cap), -max_comp, max_comp)
        anchor = anchor + feed * DT
        if lead is not None:
            anchor = np.minimum(anchor, lead)
        if floor is not None:
            anchor = np.maximum(anchor, floor)
        arm._set_cartesian(anchor + comp, _hand_rot(ctx, rot0, level), arm.finger)
        yield


def _line(ctx, arm, target, duration, level=None):
    """Smooth straight move of the TCP to ``target`` while the hand trims the bowl level."""
    p0, n = arm.pos.copy(), max(1, round(duration / DT))
    rot0 = HAND["base"] if HAND["base"] is not None else arm.rot.copy()
    for i in range(1, n + 1):
        s = ik.smoothstep(i / n)
        arm._set_cartesian(p0 + s * (np.asarray(target, float) - p0), _hand_rot(ctx, rot0, level), arm.finger)
        yield


def demo_script(ctx):
    """Right arm: take the bowl by its tab, carry it to the front of the mixer, bring it in under the beater, steer the
    holes over the pins and lower it until the ears sit. Left arm: take the red handle, raise the frame to its stop and
    hold it. Right arm: slide the blue bolt in under the frame's tab. Left arm: let the frame down onto the bolt and
    let go. Every move runs under 10 cm/s."""
    left, right = ctx.left, ctx.right
    HAND["base"], HAND["dp"] = None, 0.0

    def bowl():
        return np.asarray(ctx.obj("bowl").origin, float)

    def bowl_pitch():
        """Radians the bowl's x axis is above the horizontal."""
        return float(np.arcsin(np.clip(ctx.obj("bowl").rotation()[2, 0], -1.0, 1.0)))

    def mixer_j():
        return float(ctx.obj("mixer").joints.get("lift", 0.0))

    # 1. right arm: take the bowl by its tab
    b0 = bowl()
    tab = np.array([b0[0] + TAB_X, b0[1], b0[2] + GRIP_DZ])
    hover = np.array([tab[0], tab[1], TOP + 0.15])
    yield from right.move(hover, duration=5.0, finger=O, pitch=PITCH)
    yield from right.trim_to_measured(hover, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = right.pos - hover
    yield from right.move(tab + c, duration=3.5, finger=O, pitch=PITCH)
    yield from right.trim_to_measured(tab, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from right.grip(C, duration=0.9)
    yield from right.hold(0.3)
    HAND["base"], HAND["dp"] = right.rot.copy(), 0.0
    # 2. lift to the carry height, then in front of the mixer: first straight back (-x), then across (y)
    off = right.pos - bowl()  # the TCP relative to the bowl's origin, as gripped
    yield from _line(ctx, right, right.pos + np.array([0.0, 0.0, CARRY_ORIGIN_Z - bowl()[2]]), 2.5, bowl_pitch)
    yield from right.hold(0.3)
    stage1 = np.array([STAGE_X, b0[1], CARRY_ORIGIN_Z]) + off
    yield from _line(ctx, right, stage1, 3.0, bowl_pitch)
    stage2 = np.array([STAGE_X, 0.0, CARRY_ORIGIN_Z]) + off
    yield from _line(ctx, right, stage2, 4.5, bowl_pitch)
    yield from right.hold(0.3)
    # 3. down to rim 0.115 (under the beater's tip), then in along +x to the pins
    under = np.array([STAGE_X, 0.0, UNDER_ORIGIN_Z]) + off
    yield from _line(ctx, right, under, 2.5, bowl_pitch)
    yield from right.hold(0.3)
    want = np.array([PIN_X, 0.0, UNDER_ORIGIN_Z])
    yield from _chase(ctx, right, lambda: np.array([0.0, (want - bowl())[1], (want - bowl())[2]]), 30, gain=0.25, level=bowl_pitch)
    x_end = PIN_X + off[0] + 0.004
    yield from _chase(
        ctx,
        right,
        lambda: np.array([0.0, (want - bowl())[1], (want - bowl())[2]]),
        int(14.0 / DT),
        feed=(0.02, 0.0, 0.0),
        stop=lambda: bowl()[0] >= PIN_X - 0.0008,
        lead=np.array([x_end, 1.0, 1.0]),
        level=bowl_pitch,
    )
    # 4. steer the holes onto the pins, then lower until the ears sit
    for _ in range(3):
        yield from _chase(ctx, right, lambda: np.array([(want - bowl())[0], (want - bowl())[1], 0.0]), 20, gain=0.3, max_comp=0.012, level=bowl_pitch)
    seat_z = TOP + ARM_TOP - EAR_Z0 + 0.0005
    yield from _chase(
        ctx,
        right,
        lambda: np.array([(want - bowl())[0], (want - bowl())[1], 0.0]),
        int(8.0 / DT),
        feed=(0.0, 0.0, -0.01),
        stop=lambda: bowl()[2] <= seat_z,
        floor=np.array([-9.0, -9.0, right.pos[2] - 0.03]),
        max_comp=0.012,
        level=bowl_pitch,
    )
    yield from right.hold(0.8)
    yield from right.grip(O, duration=1.0)
    yield from right.hold(0.5)
    HAND["base"] = None
    yield from right.move(right.pos + np.array([-0.05, 0.0, 0.0]), duration=2.5, finger=O, pitch=PITCH)
    yield from right.move(np.array([right.pos[0], right.pos[1], TOP + 0.17]), duration=2.5, finger=O, pitch=PITCH)
    yield from right.move(np.array([0.16, -0.20, TOP + 0.17]), duration=3.5, finger=O, pitch=PITCH)  # out to the side first
    def knob():
        lk = ctx.obj("lock")
        return np.asarray(lk.origin, float) + np.array([KNOB[0], KNOB[1] + float(lk.joints.get("bolt", 0.0)), 0.0])

    def bolt_j():
        return float(ctx.obj("lock").joints.get("bolt", 0.0))

    # the right hand waits 7.5 cm outboard of the bolt's knob, already at grip height, while the left raises the frame:
    # no finger passes the knob before the frame is up, and the wrist stays well inside its limits (a straight-down hand
    # at this reach runs out of wrist travel 19 cm above the table, so it never hovers over the knob)
    ryaw = np.pi / 2
    GRIP_Z = TOP + BOLT_TOP + 0.027  # the open fingers straddle the bar, which is as wide as the knob
    k0 = knob()
    side_r = np.array([k0[0], k0[1] - 0.075, GRIP_Z])
    yield from right.move(np.array([0.20, side_r[1], TOP + 0.17]), duration=2.0, finger=O, yaw=ryaw)
    yield from right.move(side_r, duration=4.0, finger=O, yaw=ryaw)
    yield from right.trim_to_measured(side_r, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c_side = right.pos - side_r

    # 5. left arm: take the lift handle, raise the frame to its stop and hold it there
    mx = np.asarray(ctx.obj("mixer").origin, float)
    grip = np.array([mx[0] + HANDLE[0], mx[1] + HANDLE[1], TOP + 0.07])
    yaw = -np.pi / 2
    up = np.array([grip[0], grip[1], TOP + 0.17])
    yield from left.move(up, duration=5.0, finger=O, yaw=yaw)
    yield from left.trim_to_measured(up, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = left.pos - up
    yield from left.move(grip + c, duration=3.5, finger=O, yaw=yaw)
    yield from left.trim_to_measured(grip, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from left.grip(C, duration=0.9)
    yield from left.hold(0.3)
    z0 = left.pos[2]
    yield from _chase(
        ctx,
        left,
        lambda: np.zeros(3),
        int(14.0 / DT),
        feed=(0.0, 0.0, 0.012),
        stop=lambda: mixer_j() >= LIFT_MAX - 0.0007,
        lead=np.array([1.0, 1.0, z0 + LIFT_MAX + 0.006]),
    )
    yield from left.hold(0.4)

    # 6. right arm: slide in level onto the knob, grip it and push the bolt under the frame's tab to its stop (the left
    # still holds the frame). If the bolt is not in afterwards, take the knob where it now is and push once more.
    for attempt in range(2):
        if attempt == 1 and (ctx.step_count == 0 or bolt_j() >= BOLT_IN + 0.004):
            break
        k = knob()
        kg = np.array([k[0], k[1], GRIP_Z])
        if attempt == 1:
            pre = kg + np.array([0.0, -0.06, 0.0])
            yield from right.move(pre + c_side, duration=1.5, finger=O, yaw=ryaw)
            yield from right.trim_to_measured(pre, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
            c_side = right.pos - pre
        yield from right.move(kg + c_side, duration=3.0, finger=O, yaw=ryaw)
        yield from right.trim_to_measured(kg, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
        yield from right.grip(C, duration=0.9)
        yield from right.hold(0.3)
        y0 = right.pos[1]
        yield from _chase(
            ctx,
            right,
            lambda: np.zeros(3),
            int(8.0 / DT),
            feed=(0.0, 0.01, 0.0),
            stop=lambda: bolt_j() >= BOLT_TRAVEL - 0.0007,
            lead=np.array([1.0, y0 + (BOLT_TRAVEL - bolt_j()) + 0.004, 9.0]),
        )
        yield from right.hold(0.4)
        yield from right.grip(O, duration=0.8)
        yield from right.hold(0.3)
        # back off level, away from the column: the open fingers leave the knob without rising to the wrist's limit
        yield from right.move(right.pos + np.array([0.0, -0.06, 0.0]), duration=2.0, finger=O, yaw=ryaw)

    # 7. left arm: let the frame down onto the bolt (until it rests or stops sinking), let go, and both arms park
    z1 = left.pos[2]
    sink = {"last": None, "n": 0}

    def let_down():
        if ctx.step_count == 0:
            return False
        j = mixer_j()
        sink["n"] = sink["n"] + 1 if sink["last"] is not None and abs(j - sink["last"]) < 0.0002 and j < LIFT_MAX - 0.003 else 0
        sink["last"] = j
        return j <= REST_J + 0.0015 or sink["n"] >= 6

    yield from _chase(
        ctx,
        left,
        lambda: np.zeros(3),
        int(4.0 / DT),
        feed=(0.0, 0.0, -0.006),
        stop=let_down,
        floor=np.array([-9.0, -9.0, z1 - (LIFT_MAX - REST_J) - 0.003]),
    )
    yield from left.hold(0.5)
    yield from left.grip(O, duration=0.8)
    yield from left.move(np.array([left.pos[0], left.pos[1], left.pos[2] + 0.03]), duration=1.0, finger=O, yaw=-np.pi / 2)
    yield from ctx.both(left.home(duration=4.5), right.home(duration=4.5))
    yield from ctx.wait(1.0)
