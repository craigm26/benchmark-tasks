"""On the station table: an arbor press (fixed dark grey column and head, a black ram with a blue handle on its near
side, ram axis at x 27, y -3) and, under the ram, a grey press plate that turns on a post at x 27, y +3. The plate has
three slots cut in from its edges (1.4, 3.0 and 5.4 cm wide) and an orange grip tab on one corner; the 1.4 cm slot is
under the ram at the start. In front of the right arm a red pulley hub stands on a light grey stand, with a yellow shaft
pressed through its bore and standing 4 cm proud of the hub. The plate turns, the ram slides down against a light
return spring, the pulley is loose and its shaft slides down through the hub when pressed; the column, the post and the
stand are fixed.

Requires -E object_jitter=0: the shaft must end within 3 mm of the ram axis, measured from the press's marked position.

Run it:  inspect-robots run --embodiment yam_arena -E task=press_a_shaft_out_on_the_arbor_plate.py -E object_jitter=0 \\
           --instruction "Press the shaft out of the red pulley hub on the arbor press." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips things: keep the episode (and the video) running until the release is seen

GREY, DARK_GREY, LIGHT_GREY = (0.55, 0.56, 0.60), (0.33, 0.34, 0.38), (0.72, 0.72, 0.74)
BLACK, BLUE, ORANGE = (0.05, 0.05, 0.05), (0.10, 0.30, 0.90), (0.98, 0.45, 0.02)
RED, YELLOW = (0.85, 0.10, 0.10), (0.98, 0.85, 0.05)

# -- the press -----------------------------------------------------------------------------------------------------------------
AX, AY = 0.27, -0.03  # ram axis
RAM_Z0, RAM_Z1 = 0.20, 0.45  # ram bar at rest, above the table; it runs up through the head
RAM_W = 0.024
HANDLE = (0.078, 0.018, 0.03)  # x, y, z; on the ram's -x face, x 0.18..0.258
HANDLE_Z = 0.245
RAM_LOW = -0.114  # the face stops 1 mm above a seated hub
RAM_SPRING, RAM_DAMP, RAM_MASS = 25.0, 10.0, 0.02  # a return spring; it sags mass*g/spring = 8 mm at rest

PRESS = Mech(
    "press",
    pos=(AX, AY, TOP),
    mass=None,
    friction=0.5,
    shapes=[
        box((0.0, -0.12, 0.185), (0.06, 0.04, 0.37), DARK_GREY),  # column, x 24..30, y -17..-13, 37 tall
        box((0.0, -0.0625, 0.35), (0.06, 0.155, 0.04), DARK_GREY),  # head arm, x 24..30, y -17..-1.5, z 33..37
    ],
    joints=[
        Joint("ram", "slide", axis="z", at=(0.0, 0.0, (RAM_Z0 + RAM_Z1) / 2), low=RAM_LOW, high=0.0, spring=RAM_SPRING,
              damping=RAM_DAMP, mass=RAM_MASS, friction=0.5,
              shapes=[box((0.0, 0.0, 0.0), (RAM_W, RAM_W, RAM_Z1 - RAM_Z0), BLACK),
                      box((-RAM_W / 2 - HANDLE[0] / 2, 0.0, HANDLE_Z - (RAM_Z0 + RAM_Z1) / 2), HANDLE, BLUE)]),
    ],
)

# -- the press plate -----------------------------------------------------------------------------------------------------------
PX, PY = 0.27, 0.03  # pivot post
POST_H, PLATE_T, PLATE_HALF = 0.04, 0.015, 0.07
PLATE_TOP = TOP + POST_H + PLATE_T
PLATE_MID = POST_H + PLATE_T / 2
PLATE_J = math.pi / 2  # the turn that brings the 3.0 cm slot under the ram
J_TOL = 0.05
SLOT_NARROW, SLOT_MID, SLOT_WIDE = 0.014, 0.030, 0.054
SLOT_END = 0.03  # each slot runs in to this far from the pivot
TAB = 0.018  # square grip tab, 3 cm tall
TAB_C = 0.057  # tab centre from the pivot on x and y (8 cm out on the diagonal)


def _slab(x0, x1, y0, y1):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, 0.0), (x1 - x0, y1 - y0, PLATE_T), GREY)


h, n, m, w = PLATE_HALF, SLOT_NARROW / 2, SLOT_MID / 2, SLOT_WIDE / 2
PLATE_SHAPES = [
    _slab(-h, -n, -h, -SLOT_END), _slab(n, h, -h, -SLOT_END),  # -y edge: the 1.4 cm slot
    _slab(-h, h, -SLOT_END, -m),
    _slab(-SLOT_END, h, -m, m),  # -x edge: the 3.0 cm slot
    _slab(-h, h, m, SLOT_END),
    _slab(-h, -w, SLOT_END, h), _slab(w, h, SLOT_END, h),  # +y edge: the 5.4 cm slot
    box((TAB_C, TAB_C, PLATE_T / 2 + 0.015), (TAB, TAB, 0.03), ORANGE),
]
PLATE = Mech(
    "plate",
    pos=(PX, PY, TOP),
    mass=None,
    friction=0.8,
    shapes=[box((0.0, 0.0, POST_H / 2), (0.02, 0.02, POST_H), DARK_GREY)],
    joints=[Joint("plate", "hinge", axis="z", at=(0.0, 0.0, PLATE_MID), low=0.0, high=4.75, spring=0.0, damping=0.4,
                  mass=0.10, friction=0.8, shapes=PLATE_SHAPES)],
)

# -- the pulley ----------------------------------------------------------------------------------------------------------------
HUB, HUB_H, BORE = 0.044, 0.030, 0.024
SHAFT, SHAFT_LEN, PROUD = 0.018, 0.070, 0.040
STAND_H, STAND_W = 0.05, 0.06
STAND_XY = (0.17, -0.19)
SHAFT_J_LOW = -0.045
SHAFT_DAMP, SHAFT_MASS, HUB_MASS = 1600.0, 0.008, 0.12  # the press fit is a heavy damper (it does not hold by itself)
SHAFT_AT_Z = -HUB_H / 2 + SHAFT_LEN / 2  # shaft centre above the hub centre, shaft bottom flush with the hub bottom
_b, _t = BORE / 2, HUB / 2
HUB_SHAPES = [
    box((-(_b + _t) / 2, 0.0, 0.0), (_t - _b, HUB, HUB_H), RED),
    box(((_b + _t) / 2, 0.0, 0.0), (_t - _b, HUB, HUB_H), RED),
    box((0.0, -(_b + _t) / 2, 0.0), (BORE, _t - _b, HUB_H), RED),
    box((0.0, (_b + _t) / 2, 0.0), (BORE, _t - _b, HUB_H), RED),
]
STAND = Mech("stand", pos=(*STAND_XY, TOP + STAND_H / 2), mass=None, friction=0.8,
             shapes=[box((0.0, 0.0, 0.0), (STAND_W, STAND_W, STAND_H), LIGHT_GREY)])
PULLEY = Mech(
    "pulley",
    pos=(*STAND_XY, TOP + STAND_H + HUB_H / 2 + 0.0005),
    mass=HUB_MASS,
    friction=1.0,
    shapes=HUB_SHAPES,
    joints=[Joint("shaft", "slide", axis="z", at=(0.0, 0.0, SHAFT_AT_Z), low=SHAFT_J_LOW, high=0.0, spring=0.0,
                  damping=SHAFT_DAMP, mass=SHAFT_MASS, friction=1.0,
                  shapes=[box((0.0, 0.0, 0.0), (SHAFT, SHAFT, SHAFT_LEN), YELLOW)])],
)

OBJECTS = [PRESS, PLATE, STAND, PULLEY]

# -- scoring -----------------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.002  # hub bottom within this of the plate top
TILT_MAX = 3.0  # degrees
CENTRE_TOL = 0.003  # shaft centre from the ram axis
PRESSED_J = -0.035  # shaft top within 5 mm of the hub top
RAM_CONTACT = 0.004  # while the shaft goes down the ram face is within this of its top
FREE_DROP = 0.012  # total descent allowed without the ram on the shaft (gravity creep of the damped slide)
RAM_CLEAR = 0.03  # ram face this far above the hub top at the end
DROPPED_Z = 0.02  # a hub whose centre-line bottom is lower than this above the table has been dropped
_PENDING = {"prev_j": 0.0, "free": 0.0, "bad": False}


def _reset(*_a):
    _PENDING.update(prev_j=0.0, free=0.0, bad=False)


mechkit.on_reset(_reset)


def _ram_face(objects) -> float:
    return float(objects["press"].origin[2]) + RAM_Z0 + float(objects["press"].joints["ram"])


def _shaft_top(objects) -> float:
    p = objects["pulley"]
    return float(p.origin[2]) + SHAFT_AT_Z + float(p.joints["shaft"]) + SHAFT_LEN / 2


def _hub_top(objects) -> float:
    return float(objects["pulley"].origin[2]) + HUB_H / 2


def _track(objects) -> None:
    """Latch over the whole run: the shaft must be driven down by the ram (a hand, a finger or lifting the hub off it does
    not count). Descent while the ram face is not on the shaft top adds up; more than FREE_DROP of it is a violation."""
    j = min(0.0, float(objects["pulley"].joints["shaft"]))  # above its rest stop counts as at rest
    drop = max(0.0, _PENDING["prev_j"] - j)
    p = objects["pulley"]
    under_ram = math.hypot(p.origin[0] - AX, p.origin[1] - AY) <= RAM_W / 2
    if drop > 0.0 and not (under_ram and abs(_ram_face(objects) - _shaft_top(objects)) <= RAM_CONTACT):
        _PENDING["free"] += drop
        if _PENDING["free"] > FREE_DROP:
            _PENDING["bad"] = True
    _PENDING["prev_j"] = j


def _plate_turned(objects) -> bool:
    return abs(float(objects["plate"].joints["plate"]) - PLATE_J) <= J_TOL


def _seated(objects) -> bool:
    p = objects["pulley"]
    if tilt_deg(p) > TILT_MAX:
        return False
    if abs(float(p.origin[2]) - HUB_H / 2 - PLATE_TOP) > SEAT_TOL:
        return False
    return bool(math.hypot(p.origin[0] - AX, p.origin[1] - AY) <= CENTRE_TOL)


def _dropped(objects) -> bool:
    """The hub is on the table or off it: its lowest point is under 2 cm above the table top (stand top 5 cm, plate top
    5.5 cm). A pulley knocked off its stand, pushed off the table or dropped through the 5.4 cm slot ends here."""
    return float(objects["pulley"].origin[2]) - HUB_H / 2 < TOP + DROPPED_Z


def _pressed(objects) -> bool:
    return bool(not _PENDING["bad"] and float(objects["pulley"].joints["shaft"]) <= PRESSED_J)


def _ram_clear(objects) -> bool:
    return _ram_face(objects) >= _hub_top(objects) + RAM_CLEAR


def score(objects):
    """0 if the hub is on the table or off it (lowest point under 2 cm above the table), or the 3.0 cm slot is not under the
    ram (plate angle more than 0.05 rad from a quarter turn). Otherwise 0.25 for the plate alone; 0.5 when also the hub
    lies flat on the plate (tilt at most 3 degrees, bottom within 2 mm of the plate top) with the shaft centre within 3 mm
    of the ram axis; 1.0 when also the shaft top is within 5 mm of the hub top, driven there by the ram, and the ram face is
    back at least 3 cm above the hub. A hub held up, on the stand or on another slot is not seated; more than 1.2 cm of
    shaft travel without the ram face on the shaft top (a finger, a hand, lifting the hub off it) voids the press for the
    rest of the run."""
    _track(objects)
    if _dropped(objects) or not _plate_turned(objects):
        return 0.0
    if not _seated(objects):
        return 0.25
    if _pressed(objects) and _ram_clear(objects):
        return 1.0
    return 0.5


def success(objects):
    """Full score, with the pulley and the plate still."""
    return score(objects) == 1.0 and objects["pulley"].resting() and objects["plate"].resting()


def distance(objects):
    """From the shaft centre to the ram axis (m)."""
    p = objects["pulley"]
    return float(math.hypot(p.origin[0] - AX, p.origin[1] - AY))


# The scripted demo keeps every part of both arms clear of the press column (4 cm margin, below 25 cm above the table),
# checked offline with the URDF. The ram's stroke above the work (its 3 x 3 cm column from 15 cm above the table up) is
# a keep-out the envelope gives the gate: the pulley goes in below the ram face and the ram is worked by its handle.
# The scorer itself sees only the objects.
ARM_KEEP_OUT = [
    {"name": "press column and 4 cm around it", "x": (AX - 0.03 - 0.04, AX + 0.03 + 0.04), "y": (-0.17 - 0.04, -0.13 + 0.04),
     "z_top": TOP + 0.25},
]

# -- the scripted demo -------------------------------------------------------------------------------------------------------
HOVER = TOP + 0.14
TAB_GRIP_Z = TOP + POST_H + PLATE_T + 0.017  # tool point at the tab's middle
TURN_S = 4.0  # seconds for the quarter turn
TAB_YAW = -math.pi / 2  # the hand starts a quarter turn back, so its wrist ends the turn mid-range, not on its stop
GRASP_PITCH = 1.3  # shaft grasp: fingers nearly horizontal, from -x, closing along y
SHAFT_GRIP_ABOVE_HUB = 0.025  # tool point this far above the hub top while carrying
CARRY_Z = TOP + 0.14
CARRY_CAP = TOP + 0.142  # under the ram the hand is never commanded above this (the gate holds it at 15 cm)
HAND_Z = TOP + HANDLE_Z
PRESS_SPEED = 0.0025  # m/s once the ram is on the shaft
PRESS_LEAD = 0.045  # m
PRESS_DONE_J = -0.0375  # stop pressing once the shaft top is within 2.5 mm of the hub top


demo = placeholder_demo  # the motion is in demo_script(ctx), which closedloop runs inside the yam_scripted policy


def _live(ctx) -> bool:
    return ctx.step_count > 0


def _turn_plate(ctx, arm):
    """Take the orange tab from straight above, swing the plate round its pivot until the 3.0 cm slot is under the ram
    (the hand turns with the tab, so the tab does not twist in the fingers), take the push off, let go."""
    r0 = np.array([TAB_C, TAB_C])

    def tab_at(theta):
        c, s = math.cos(theta), math.sin(theta)
        return np.array([PX + c * r0[0] - s * r0[1], PY + s * r0[0] + c * r0[1]])

    plate_j = lambda fallback: float(ctx.obj("plate").joints["plate"]) if _live(ctx) else fallback  # noqa: E731
    j0 = plate_j(0.0)
    t0 = tab_at(j0)
    yield from arm.move((t0[0], t0[1], HOVER), 2.5, FINGER_OPEN, yaw=j0 + TAB_YAW)
    yield from arm.move((t0[0], t0[1], TAB_GRIP_Z), 1.5, FINGER_OPEN, yaw=j0 + TAB_YAW)
    yield from arm.trim_to_measured((t0[0], t0[1], TAB_GRIP_Z), tol=0.0015)
    yield from arm.hold(0.3)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    off = arm.pos[:2] - t0  # what the command needed to put the hand on the tab (sag trim)

    def put(theta):
        xy = tab_at(theta) + off
        arm._set_cartesian(np.array([xy[0], xy[1], TAB_GRIP_Z]), grip_rot(yaw=theta + TAB_YAW), FINGER_CLOSED)

    steps = round(TURN_S / arm.dt)
    th = j0
    for i in range(1, steps + 1):  # along the tab's arc
        th = j0 + (PLATE_J - j0) * i / steps
        put(th)
        yield
    settled = 0
    for _ in range(160):  # close the last of the angle on the plate's own reading
        err = PLATE_J - plate_j(PLATE_J)
        settled = settled + 1 if abs(err) <= 0.012 else 0
        if settled >= 8:
            break
        th = float(np.clip(th + 0.15 * err, PLATE_J - 0.35, PLATE_J + 0.35))
        put(th)
        yield
    for _ in range(16):  # take the push off before opening: bring the command back to where the plate is
        th += 0.25 * (plate_j(PLATE_J) - th)
        put(th)
        yield
    yield from arm.grip(FINGER_OPEN, 1.2)
    yield from arm.hold(0.3)
    yield from arm.move((arm.pos[0], arm.pos[1], HOVER + 0.04), 1.5, FINGER_OPEN, rot=arm.rot.copy())
    yield from arm.home(3.0)


def _seat_pulley(ctx, arm):
    """Take the shaft, carry the pulley in below the ram face and set the hub on the plate with the shaft on the ram axis."""
    live = _live(ctx)
    p = ctx.pos("pulley")
    gx, gy = p[0], p[1]
    z_grip = TOP + STAND_H + HUB_H + SHAFT_GRIP_ABOVE_HUB
    kw = dict(pitch=GRASP_PITCH)
    yield from arm.move((gx - 0.03, gy, z_grip + 0.03), 2.5, FINGER_OPEN, **kw)
    yield from arm.move((gx, gy, z_grip), 1.8, FINGER_OPEN, **kw)
    yield from arm.trim_to_measured((gx, gy, z_grip), tol=0.0015)
    yield from arm.hold(0.3)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)
    yield from arm.move((gx, gy, CARRY_Z), 1.5, FINGER_CLOSED, **kw)
    yield from arm.move((gx, AY - 0.02, CARRY_Z), 2.5, FINGER_CLOSED, **kw)
    d = np.zeros(2)
    if live:
        d = ctx.pos("pulley")[:2] - arm.measured_pos[:2]
    # the hub hangs lower than the hand says (wrist sag, slip in the fingers): carry it 2 cm above the plate on its own
    # reading, but never command the hand above CARRY_CAP under the ram
    sag = 0.0
    if live:
        sag = (arm.pos[2] - SHAFT_GRIP_ABOVE_HUB - HUB_H) - (float(ctx.obj("pulley").origin[2]) - HUB_H / 2)
    carry = min(CARRY_CAP, PLATE_TOP + 0.02 + HUB_H + SHAFT_GRIP_ABOVE_HUB + float(np.clip(sag, 0.0, 0.03)))
    yield from arm.move((gx, AY - 0.02, carry), 1.0, FINGER_CLOSED, **kw)
    yield from arm.move((AX - d[0], AY - d[1], carry), 2.5, FINGER_CLOSED, **kw)
    rot = arm.rot.copy()
    c = np.zeros(2)

    def aim(z0, z1, dur, stop=None):
        nonlocal c
        steps = max(1, round(dur / arm.dt))
        for i in range(1, steps + 1):
            if live:
                cur = ctx.pos("pulley")[:2]
                c = np.clip(c + 0.25 * (np.array([AX, AY]) - cur), -0.03, 0.03)
            want = np.array([AX, AY]) - d + c
            z = z0 + (z1 - z0) * i / steps
            arm._set_cartesian(np.array([want[0], want[1], z]), rot, FINGER_CLOSED)
            yield
            if stop is not None and stop():
                break

    seat_tcp_z = PLATE_TOP + HUB_H + SHAFT_GRIP_ABOVE_HUB
    yield from aim(carry, carry, 1.2)
    hub_down = lambda: live and (float(ctx.obj("pulley").origin[2]) - HUB_H / 2 <= PLATE_TOP + 0.0015)  # noqa: E731
    yield from aim(carry, seat_tcp_z + 0.01, 1.5, stop=hub_down)
    if not hub_down():
        yield from aim(seat_tcp_z + 0.01, seat_tcp_z - 0.004, 2.5, stop=hub_down)
    yield from arm.hold(0.6)
    yield from arm.grip(FINGER_OPEN, 0.9)
    yield from arm.hold(0.4)
    here = arm.pos.copy()
    yield from arm.move((AX - 0.10, here[1], here[2]), 2.0, FINGER_OPEN, **kw)
    yield from arm.move((AX - 0.10, here[1], CARRY_Z + 0.02), 1.5, FINGER_OPEN, **kw)
    yield from arm.move((STAND_XY[0], STAND_XY[1] - 0.08, CARRY_Z + 0.02), 3.0, FINGER_OPEN, **kw)  # round the press column
    yield from arm.home(3.5)


def _press_ram(ctx, arm):
    """Take the ram handle by its free end, drive the ram down on the shaft top until the shaft is out, lift the ram, let go."""
    live = _live(ctx)
    gx = AX - RAM_W / 2 - HANDLE[0] + 0.02  # 2 cm in from the handle end
    ram0 = float(ctx.obj("press").joints["ram"]) if live else -RAM_MASS * 9.81 / RAM_SPRING
    z0 = HAND_Z + ram0
    # the open fingers are 9.4 cm apart: come round behind the handle's free end, then slide in along it
    yield from arm.move((gx - 0.06, AY + 0.11, z0 + 0.01), 3.0, FINGER_OPEN, pitch=GRASP_PITCH)
    yield from arm.move((gx - 0.06, AY, z0 + 0.01), 1.5, FINGER_OPEN, pitch=GRASP_PITCH)
    yield from arm.move((gx, AY, z0), 1.8, FINGER_OPEN, pitch=GRASP_PITCH)
    yield from arm.trim_to_measured((gx, AY, z0), tol=0.0015)
    yield from arm.hold(0.3)
    yield from arm.grip(FINGER_CLOSED, 0.9)
    yield from arm.hold(0.3)
    rot = arm.rot.copy()
    rj = ram0
    base = arm.pos[2] - ram0  # tool height at ram J = 0
    floor = RAM_LOW - PRESS_LEAD  # the hand may lead the ram by this much: that is how hard it pushes
    for _ in range(1200):
        if live:
            st = ctx.states()
            shaft, meas = float(st["pulley"].joints["shaft"]), float(st["press"].joints["ram"])
            v = 0.03 if _ram_face(st) - _shaft_top(st) > 0.015 else PRESS_SPEED  # quick down to the shaft, then slow
            if shaft <= PRESS_DONE_J or meas <= RAM_LOW + 0.001:
                break
        else:
            meas, v = rj, 0.02
            if rj <= RAM_LOW:
                break
        if rj - meas > -PRESS_LEAD:
            rj = max(floor, rj - v * arm.dt)
        arm._set_cartesian(np.array([arm.pos[0], arm.pos[1], base + rj]), rot, FINGER_CLOSED)
        yield
    yield from arm.hold(0.4)
    top = min(-0.01, ram0)
    while rj < top:  # lift the ram back to the top of its stroke
        rj = min(top, rj + 0.08 * arm.dt)
        arm._set_cartesian(np.array([arm.pos[0], arm.pos[1], base + rj]), rot, FINGER_CLOSED)
        yield
    yield from arm.hold(0.2)
    yield from arm.grip(FINGER_OPEN, 0.6)
    yield from arm.move((arm.pos[0] - 0.06, arm.pos[1], arm.pos[2]), 0.9, FINGER_OPEN, pitch=GRASP_PITCH)
    yield from arm.home(2.5)


def demo_script(ctx):
    """The left arm turns the plate 90 degrees by its orange tab and goes home. The right arm takes the pulley by its shaft,
    carries it in below the ram face and sets the hub on the plate with the shaft on the ram axis, then goes home. The left
    arm takes the ram handle, drives the ram down until the shaft is out, lifts it and lets go."""
    yield from _turn_plate(ctx, ctx.left)
    yield from _seat_pulley(ctx, ctx.right)
    yield from _press_ram(ctx, ctx.left)
    yield from ctx.wait(1.5)
