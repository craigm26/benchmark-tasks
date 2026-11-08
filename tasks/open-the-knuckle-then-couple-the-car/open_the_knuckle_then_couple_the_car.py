"""On the station table: a short red car on a dark track in front of the left arm, and a grey coupler head at the far end
of the track (the head sits in front of the right arm). The car has a dark necked pin on its nose, a blue push handle on
its side and a red striker finger on its right. The head's mouth is shut by a dark sliding knuckle jaw. A yellow spring
lock stands in a notch in the jaw and holds it shut; a red lever bar with a grip block reaches out from the jaw.

The jaw only moves sideways, and only after the yellow lock is lifted clear of its notch. With the jaw open the car's
pin can go into the head's throat, and the striker finger then rides up a ramp on the jaw and pushes it shut behind
the pin; the lock drops back into its notch. With the jaw shut the pin only butts against it.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_the_knuckle_then_couple_the_car.py -E object_jitter=0 \\
           --instruction "Couple the red car to the grey coupler head and leave them locked together." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import belimits  # noqa: F401  (shared geometry helpers, kept beside this file)
from closedloop import grip_rot, placeholder_demo
from mechkit import Joint, Mech, box, on_reset
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
SUCCESS_HOLD_S = 8.0  # the demo grips and releases: keep the run going so the video shows the release and the arms clear
TOP = TABLE_TOP[SCENE]

DARK, GREY, BLACK = (0.16, 0.16, 0.20), (0.55, 0.57, 0.62), (0.05, 0.05, 0.06)
RED, BLUE, YELLOW = (0.88, 0.08, 0.08), (0.10, 0.30, 0.90), (0.98, 0.85, 0.05)

OX = 0.27  # the Mech's origin is (OX, 0, TOP); parts below are written in world x, y and height above the table


def bx(x0, x1, y0, y1, w0, w1, color, tilt=(1.0, 0.0, 0.0, 0.0)):
    """A box from world x, y and height w (m above the table top), as a mechkit shape in the Mech frame."""
    return box(((x0 + x1) / 2 - OX, (y0 + y1) / 2, (w0 + w1) / 2), (x1 - x0, y1 - y0, w1 - w0), color, tilt)


# -- the head (fixed) -----------------------------------------------------------------------------------------------
HEAD_Y0, HEAD_FACE = -0.14, -0.06  # the head's far end and its front face (towards the car)
THROAT_X = (0.256, 0.286)  # 3.0 cm wide, 3.5 cm deep (y -0.095 to -0.06)
THROAT_BACK = -0.095
BASE_SHAPES = [
    bx(0.24, 0.30, -0.20, 0.20, 0.0, 0.02, DARK),  # track bed
    bx(0.235, THROAT_X[0], HEAD_Y0, HEAD_FACE, 0.02, 0.08, GREY),  # head: left wall
    bx(THROAT_X[1], 0.305, HEAD_Y0, HEAD_FACE, 0.02, 0.08, GREY),  # right wall
    bx(THROAT_X[0], THROAT_X[1], HEAD_Y0, THROAT_BACK, 0.02, 0.08, GREY),  # back of the throat
    bx(0.285, 0.385, -0.058, -0.026, 0.0, 0.03, GREY),  # the slideway under the jaw
]

# -- the car (slides along y, J <= 0 towards the head) --------------------------------------------------------------------
PIN_TIP0 = 0.070  # y of the pin's tip at rest
NECK_X = (0.267, 0.275)  # necked pin: 0.8 cm neck, 2.4 cm head
PIN_HEAD_X = (0.259, 0.283)
BODY_Y = (0.150, 0.220)
STRIKER_X = (0.3195, 0.3295)
STRIKER_E = 0.04675  # the striker's front face is this far ahead of the pin tip
STRIKER_Y0 = PIN_TIP0 + STRIKER_E
HANDLE_X, HANDLE_Y, HANDLE_W = (0.21, 0.24), (0.17, 0.20), (0.04, 0.09)
J_COUPLED = -0.1435  # pin tip at y -0.0735, behind the shut jaw
CAR = Joint(
    "car",
    "slide",
    axis="y",
    low=-0.15,
    high=0.0,
    damping=20.0,
    mass=0.3,
    friction=0.15,
    shapes=[
        bx(0.24, 0.30, BODY_Y[0], BODY_Y[1], 0.02, 0.08, RED),  # the car
        bx(NECK_X[0], NECK_X[1], PIN_TIP0 + 0.010, BODY_Y[0], 0.03, 0.07, BLACK),  # pin neck
        bx(PIN_HEAD_X[0], PIN_HEAD_X[1], PIN_TIP0, PIN_TIP0 + 0.010, 0.03, 0.07, BLACK),  # pin head
        bx(0.30, STRIKER_X[1], 0.205, BODY_Y[1], 0.03, 0.05, RED),  # bridge to the striker
        bx(STRIKER_X[0], STRIKER_X[1], STRIKER_Y0, BODY_Y[1], 0.03, 0.05, RED),  # striker finger
        bx(HANDLE_X[0], HANDLE_X[1], HANDLE_Y[0], HANDLE_Y[1], HANDLE_W[0], HANDLE_W[1], BLUE),  # push handle
    ],
)

# -- the knuckle jaw (slides along x, K = how far open, 0 shut) ---------------------------------------------------------
D_OPEN = 0.025
JAW_Y = (-0.054, -0.030)
JAW_X = (0.2785, 0.335)  # shut: its tip stands 4.5 mm over the pin head's right edge
LOCK_X, LOCK_Y = 0.3175, -0.042  # the lock's foot, over the jaw
NOTCH_X, NOTCH_Y = (0.3085, 0.3265), (-0.051, -0.033)  # 1.8 cm square, 3 mm clearance round the 1.2 cm foot
SLAB_TOP, JAW_TOP = 0.062, 0.070  # notch floor and jaw top (the notch is 8 mm deep)
RAMP_M = 0.5  # the ramp's surface falls 1 cm in y per 2 cm in x
PSI = math.atan(RAMP_M)
RAMP_REF = (0.314, -0.024)  # a point on the ramp's surface (x, y) with the jaw shut
RAMP_LEN, RAMP_T = 0.046, 0.010
_n = np.array([math.sin(PSI), math.cos(PSI)])  # surface normal (up-and-towards +x)
_mid = np.array([0.3115, RAMP_REF[1] - RAMP_M * (0.3115 - RAMP_REF[0])])
_ctr = _mid - _n * RAMP_T / 2
LEVER_X, BLOCK_Y = (0.335, 0.345), (0.060, 0.090)
BLOCK_X = (0.331, 0.349)
BLOCK_W = (0.03, 0.06)
KNUCKLE = Joint(
    "knuckle",
    "slide",
    axis="x",
    low=0.0,
    high=D_OPEN,
    damping=1.0,
    mass=0.05,
    friction=0.15,
    shapes=[
        bx(JAW_X[0], JAW_X[1], JAW_Y[0], JAW_Y[1], 0.03, SLAB_TOP, BLACK),  # jaw slab
        bx(JAW_X[0], NOTCH_X[0], JAW_Y[0], JAW_Y[1], SLAB_TOP, JAW_TOP, BLACK),  # jaw top, left of the notch
        bx(NOTCH_X[1], JAW_X[1], JAW_Y[0], JAW_Y[1], SLAB_TOP, JAW_TOP, BLACK),  # right of the notch
        bx(NOTCH_X[0], NOTCH_X[1], NOTCH_Y[1], JAW_Y[1], SLAB_TOP, JAW_TOP, BLACK),  # front lip
        bx(NOTCH_X[0], NOTCH_X[1], JAW_Y[0], NOTCH_Y[0], SLAB_TOP, JAW_TOP, BLACK),  # back lip
        box(
            (_ctr[0] - OX, _ctr[1], 0.04),
            (RAMP_LEN, RAMP_T, 0.02),
            BLACK,
            (math.cos(-PSI / 2), 0.0, 0.0, math.sin(-PSI / 2)),
        ),  # the ramp the striker rides up
        bx(LEVER_X[0], LEVER_X[1], JAW_Y[0], BLOCK_Y[1], 0.03, 0.05, RED),  # lever bar
        bx(BLOCK_X[0], BLOCK_X[1], BLOCK_Y[0], BLOCK_Y[1], BLOCK_W[0], BLOCK_W[1], RED),  # lever grip block
    ],
)

# -- the lock (slides along z, L = how far lifted, 0 dropped) -------------------------------------------------------------
LOCK_MAX = 0.03
TAB_W = (0.100, 0.130)
LOCK = Joint(
    "lock",
    "slide",
    axis="z",
    low=0.0,
    high=LOCK_MAX,
    spring=100.0,
    damping=3.0,
    mass=0.03,
    friction=0.6,
    shapes=[
        bx(LOCK_X - 0.006, LOCK_X + 0.006, LOCK_Y - 0.006, LOCK_Y + 0.006, SLAB_TOP, TAB_W[0], YELLOW),  # foot and post
        bx(LOCK_X - 0.015, LOCK_X + 0.015, LOCK_Y - 0.009, LOCK_Y + 0.009, TAB_W[0], TAB_W[1], YELLOW),  # grip tab
    ],
)

TRACK = Mech(
    "track",
    pos=(OX, 0.0, TOP),
    mass=None,
    friction=0.5,
    shapes=BASE_SHAPES,
    joints=[CAR, KNUCKLE, LOCK],
)
OBJECTS = [TRACK]

# -- scoring ---------------------------------------------------------------------------------------------------------------
K_SHUT = 0.003  # m: jaw within 3 mm of shut
K_OPENED = 0.020  # m: jaw pulled open 2 cm or more
L_DROPPED = 0.003  # m: lock within 3 mm of its notch floor
L_LIFTED = 0.012  # m: lock lifted 1.2 cm or more (it clears the notch at 0.8 cm)
J_REST_NEAR = -0.02  # m: the car within 2 cm of where it started
J_COUPLED_TOL = 0.010
J_BUTT = (-0.108, -0.092)  # the pin head against the shut jaw (-0.100)
STILL = 0.0003  # m per control step (6 mm/s)
HOLD_STEPS = 20  # 1 s at 20 Hz
LIFT_STEPS = 10  # 0.5 s

_last = None
_prev = None
_lift_run = 0
_open_run = 0
_strike_run = 0
_still_run = 0
_lifted = False
_opened = False
_struck = False


def _clear() -> None:
    global _prev, _lift_run, _open_run, _strike_run, _still_run, _lifted, _opened, _struck
    _prev = None
    _lift_run = _open_run = _strike_run = _still_run = 0
    _lifted = _opened = _struck = False


on_reset(lambda env: _clear())


def _kLJ(objects):
    j = objects["track"].joints
    return float(j["knuckle"]), float(j["lock"]), float(j["car"])


def _coupled(k: float, lift: float, car: float) -> bool:
    """Jaw shut, lock dropped, pin 1 cm or less from its seat in the throat."""
    return bool(k <= K_SHUT and lift <= L_DROPPED and abs(car - J_COUPLED) <= J_COUPLED_TOL)


def _update(objects) -> None:
    """Called first by score() and success(); both get the same dict each control step, so it runs once a step."""
    global _last, _prev, _lift_run, _open_run, _strike_run, _still_run, _lifted, _opened, _struck
    if objects is _last:
        return
    _last = objects
    k, lift, car = _kLJ(objects)
    if _prev is not None and _prev[2] < -0.05 and car > -0.01:  # the car teleported home: a new trial
        _clear()
    moved = _prev is None or max(abs(k - _prev[0]), abs(lift - _prev[1]), abs(car - _prev[2])) > STILL
    _prev = (k, lift, car)
    _lift_run = _lift_run + 1 if lift >= L_LIFTED else 0
    _lifted = _lifted or _lift_run >= LIFT_STEPS
    _open_run = _open_run + 1 if (k >= K_OPENED and car >= J_REST_NEAR) else 0
    _opened = _opened or _open_run >= 3
    _strike_run = _strike_run + 1 if (k <= K_SHUT and J_BUTT[0] <= car <= J_BUTT[1]) else 0
    _struck = _struck or _strike_run >= 2
    _still_run = 0 if moved else _still_run + 1


def score(objects):
    """1.0: pin seated in the throat (car within 1 cm of -14.35 cm), jaw shut (within 3 mm), lock dropped (within 3 mm),
    no earlier butt of the pin against the shut jaw, and everything still for 1 s. 0.75: the same without the 1 s, or
    after a butt. Otherwise 0.5 if the jaw was ever opened 2 cm or more while the car was within 2 cm of its start,
    0.25 if the lock was ever lifted 1.2 cm for 0.5 s, else 0."""
    _update(objects)
    k, lift, car = _kLJ(objects)
    if _coupled(k, lift, car):
        return 1.0 if (not _struck and _still_run >= HOLD_STEPS) else 0.75
    if _opened:
        return 0.5
    if _lifted:
        return 0.25
    return 0.0


def success(objects):
    _update(objects)
    return score(objects) == 1.0


# The car-and-head gap: no arm part may be there during the scripted demo (checked offline with the URDF; the scorer
# sees only the track's joints). The lock tab, the lever block and the car handle all lie outside it.
GAP_X, GAP_Y = (0.24, 0.30), (-0.055, 0.075)
ARM_KEEP_OUT = [
    {"name": "gap between the car and the coupler head", "x": GAP_X, "y": GAP_Y, "z_top": TOP + 0.12},
]

# -- the scripted demo (closed loop) -----------------------------------------------------------------------------------------
demo = placeholder_demo

TAB_TCP = (LOCK_X, LOCK_Y, TOP + 0.113)
BLOCK_TCP = ((BLOCK_X[0] + BLOCK_X[1]) / 2, (BLOCK_Y[0] + BLOCK_Y[1]) / 2, TOP + 0.045)
HANDLE_TCP = ((HANDLE_X[0] + HANDLE_X[1]) / 2, (HANDLE_Y[0] + HANDLE_Y[1]) / 2, TOP + 0.065)
LIFT_BY = 0.022
PUSH_V = 0.03  # m/s


def _mock(ctx) -> bool:
    """dry_run hands the script one frozen dict; the simulator builds a new one every call."""
    return ctx.states() is ctx.states()


def _joints(ctx):
    j = ctx.obj("track").joints
    return float(j["knuckle"]), float(j["lock"]), float(j["car"])


def demo_script(ctx):
    mock = _mock(ctx)
    left, right = ctx.left, ctx.right
    rot_y = grip_rot(yaw=0.0)  # fingers close along y
    rot_x = grip_rot(yaw=-math.pi / 2)  # fingers close along x
    tx, ty, tz = TAB_TCP
    # RIGHT: take the lock by its yellow tab and lift it out of the notch.
    yield from right.move((tx, ty, tz + 0.05), 3.0, FINGER_OPEN, rot=rot_y)
    yield from right.move((tx, ty, tz), 1.2, FINGER_OPEN, rot=rot_y)
    yield from right.grip(FINGER_CLOSED, 0.9)
    yield from right.move((tx, ty, tz + LIFT_BY), 1.8, FINGER_CLOSED, rot=rot_y)
    if not mock:
        for _ in range(60):
            if _joints(ctx)[1] >= LIFT_BY - 0.003:
                break
            yield from right.move(right.pos + (0.0, 0.0, 0.002), 0.1, FINGER_CLOSED, rot=rot_y)
    yield from ctx.wait(0.5)
    # LEFT: take the lever block and pull the jaw open to its stop.
    bx_, by_, bz_ = BLOCK_TCP
    yield from left.move((bx_, by_, bz_ + 0.10), 3.0, FINGER_OPEN, rot=rot_x)
    yield from left.move((bx_, by_, bz_), 1.5, FINGER_OPEN, rot=rot_x)
    yield from left.grip(FINGER_CLOSED, 0.9)
    pull_to = bx_ + D_OPEN + 0.003
    yield from left.move((pull_to, by_, bz_), 3.5, FINGER_CLOSED, rot=rot_x)
    if not mock:
        for _ in range(100):
            if _joints(ctx)[0] >= D_OPEN - 0.002:
                break
            yield from left.move(left.pos + (0.001, 0.0, 0.0), 0.05, FINGER_CLOSED, rot=rot_x)
    yield from ctx.wait(0.5)
    # LEFT lets go and clears the lever; RIGHT lets the lock down onto the open jaw and leaves.
    yield from left.grip(FINGER_OPEN, 0.8)
    p = left.pos.copy()
    yield from left.move((p[0], p[1], p[2] + 0.05), 1.2, FINGER_OPEN, rot=rot_x)
    yield from right.grip(FINGER_OPEN, 0.9)
    p = right.pos.copy()
    yield from right.move((p[0], p[1] - 0.04, p[2] + 0.04), 1.8, FINGER_OPEN, rot=rot_y)
    yield from ctx.both(left.home(3.0), right.home(3.0))
    yield from ctx.wait(0.5)
    # LEFT: take the car by its blue handle and push it at 3 cm/s until the jaw has shut behind the pin and the lock has dropped.
    hx, hy, hz = HANDLE_TCP
    yield from left.move((hx, hy, hz + 0.10), 3.0, FINGER_OPEN, rot=rot_y)
    yield from left.move((hx, hy, hz), 1.5, FINGER_OPEN, rot=rot_y)
    yield from left.grip(FINGER_CLOSED, 0.9)
    steps = round((abs(J_COUPLED) + 0.004) / PUSH_V / ctx.dt)
    y_cmd = hy
    for _ in range(steps + 200):
        if not mock:
            k, lift, car = _joints(ctx)
            if k <= K_SHUT and lift <= L_DROPPED and car <= J_COUPLED + 0.002:
                break
            if hy + car - y_cmd > 0.03:  # the handle is lagging the command by over 3 cm: wait for it
                yield
                continue
        elif hy - y_cmd >= abs(J_COUPLED) + 0.004:
            break
        y_cmd -= PUSH_V * ctx.dt
        left._set_cartesian(np.array([hx, y_cmd, hz]), rot_y, FINGER_CLOSED)
        yield
    yield from ctx.wait(1.0)
    yield from left.grip(FINGER_OPEN, 0.8)
    p = left.pos.copy()
    yield from left.move((p[0] - 0.02, p[1] + 0.06, p[2] + 0.10), 1.8, FINGER_OPEN, rot=rot_y)
    yield from left.home(3.0)
    yield from ctx.wait(1.0)
