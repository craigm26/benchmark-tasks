"""On the station table, in front of the left arm: a white dewar, 15 x 23 cm and 7.8 cm tall, whose top deck (the
shoulder) has a square neck 8 cm across in the middle. A grey index ring 1 cm wide runs round the neck, with two 3 cm
notches in it, one on the +y side (framed in green on the deck) and one on the -y side. Under the shoulder, on the table
inside the dewar, are two black floor slots, one below each notch. Beside the dewar, on the table, stands an orange
canister (4.5 x 4.5 x 5 cm) with a purple handle off one side of its top; the handle ends in a hook with a short lip.
The canister is loose; the dewar is fixed. To index the canister, it goes down through the neck, slides sideways under
the shoulder into the slot below the green notch, and the lip of its hook drops into that notch.

Requires -E object_jitter=0: the slots and the hook have millimetre clearances.

Run it:  inspect-robots run --embodiment yam_arena -E task=index_a_cryo_canister_under_the_dewar_shoulder.py
           -E object_jitter=0 --instruction "Return the orange canister to the dewar at the green-marked position."
           --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import tilt_deg
from closedloop import grip_rot, placeholder_demo  # noqa: F401  (grip_rot used by the demo)
from mechkit import Mech, box
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo's release and park stay in the video before the episode ends

WHITE, GREY, BLACK = (0.93, 0.93, 0.91), (0.38, 0.38, 0.42), (0.05, 0.05, 0.05)
GREEN, ORANGE, PURPLE = (0.05, 0.70, 0.20), (1.0, 0.42, 0.0), (0.45, 0.15, 0.80)

# -- the dewar (fixed). Origin: the neck's centre on the table. Local x forward, y left, z up. -------------------------
NX, NY = 0.235, 0.07  # the neck's centre
NECK = 0.04  # the neck's half-width, both ways (8 cm across)
HALF_X, HALF_Y = 0.075, 0.115  # the dewar's outer half-sizes (15 x 23 cm)
WALL = 0.01
DECK_BOT, DECK_TOP, RING_TOP = 0.070, 0.078, 0.086  # heights above the table (m)
NOTCH = 0.015  # the notches' half-width (3.0 cm)
SLOT_IN_X = 0.0255  # the slots' inside half-width in x (5.1 cm across)
SLOT_Y0, SLOT_Y1 = 0.043, 0.094  # the slots' inside extent in y (5.1 cm), either side of the neck's centre
SLOT_MID = (SLOT_Y0 + SLOT_Y1) / 2  # 0.0685
SLOT_WALL_H = 0.006


def _dewar_shapes():
    s = []
    zc, zh = DECK_TOP / 2, DECK_TOP
    s.append(box((-(HALF_X - WALL / 2), 0, zc), (WALL, 2 * HALF_Y, zh), WHITE))  # outer walls
    s.append(box((HALF_X - WALL / 2, 0, zc), (WALL, 2 * HALF_Y, zh), WHITE))
    s.append(box((0, -(HALF_Y - WALL / 2), zc), (2 * HALF_X - 2 * WALL, WALL, zh), WHITE))
    s.append(box((0, HALF_Y - WALL / 2, zc), (2 * HALF_X - 2 * WALL, WALL, zh), WHITE))
    dz, dh = (DECK_BOT + DECK_TOP) / 2, DECK_TOP - DECK_BOT  # the deck, round the neck
    side = HALF_X - NECK
    s.append(box((-(NECK + side / 2), 0, dz), (side, 2 * HALF_Y, dh), WHITE))
    s.append(box((NECK + side / 2, 0, dz), (side, 2 * HALF_Y, dh), WHITE))
    run = HALF_Y - NECK
    s.append(box((0, -(NECK + run / 2), dz), (2 * NECK, run, dh), WHITE))
    s.append(box((-(NECK - 0.005), NECK + run / 2, dz), (0.01, run, dh), WHITE))  # +y: the green mark in the middle
    s.append(box((NECK - 0.005, NECK + run / 2, dz), (0.01, run, dh), WHITE))
    s.append(box((0, 0.0575, dz), (0.06, 0.035, dh), GREEN))
    s.append(box((0, 0.095, dz), (0.06, 0.04, dh), WHITE))
    rz, rh = (DECK_TOP + RING_TOP) / 2, RING_TOP - DECK_TOP  # the index ring, with a notch at +y and at -y
    for sx in (-1, 1):
        s.append(box((sx * 0.045, 0, rz), (0.01, 0.10, rh), GREY))
    piece = NECK - NOTCH
    for sy in (-1, 1):
        for sx in (-1, 1):
            s.append(box((sx * (NOTCH + piece / 2), sy * 0.045, rz), (piece, 0.01, rh), GREY))
    for sy in (-1, 1):  # the floor slots: four black walls each, open to the table
        w = SLOT_IN_X + WALL / 2
        length = SLOT_Y1 - SLOT_Y0 + 2 * WALL
        for sx in (-1, 1):
            s.append(box((sx * w, sy * SLOT_MID, SLOT_WALL_H / 2), (WALL, length, SLOT_WALL_H), BLACK))
        for y in (SLOT_Y0 - WALL / 2, SLOT_Y1 + WALL / 2):
            s.append(box((0, sy * y, SLOT_WALL_H / 2), (2 * SLOT_IN_X, WALL, SLOT_WALL_H), BLACK))
    return s


DEWAR = Mech("dewar", pos=(NX, NY, TOP), mass=None, shapes=_dewar_shapes())

# -- the canister (free). Origin: the centre of the body's underside. ----------------------------------------------------
BODY = 0.045  # body 4.5 x 4.5 cm
BODY_H = 0.05
BRACKET_T = 0.005
HANDLE_Y = -0.0435  # the handle's centre, toward -y of the body's centre
HANDLE_X, HANDLE_T, HANDLE_TOP = 0.018, 0.010, 0.140  # 1.8 cm across x, 1.0 cm thick, rising to 14 cm
ARM_BOT = 0.130  # the hook arm: its underside
LIP_BOT = 0.080  # the lip's lower end
LIP_Y = HANDLE_Y + 0.023  # centre of the lip: the hook arm reaches 3.3 cm out from the handle's -y face
CAN_START = (NX, 0.238)
CANISTER = Mech(
    "canister",
    pos=(*CAN_START, TOP + 0.001),
    mass=0.10,
    shapes=[
        box((0, 0, BODY_H / 2), (BODY, BODY, BODY_H), ORANGE),
        box((0, -0.0305, BODY_H + BRACKET_T / 2), (HANDLE_X, 0.036, BRACKET_T), PURPLE),
        box((0, HANDLE_Y, BODY_H + BRACKET_T + (HANDLE_TOP - BODY_H - BRACKET_T) / 2),
            (HANDLE_X, HANDLE_T, HANDLE_TOP - BODY_H - BRACKET_T), PURPLE),
        box((0, (HANDLE_Y + LIP_Y) / 2, (ARM_BOT + HANDLE_TOP) / 2),
            (HANDLE_X, 0.033, HANDLE_TOP - ARM_BOT), PURPLE),
        box((0, LIP_Y, (LIP_BOT + ARM_BOT) / 2), (HANDLE_X, HANDLE_T, ARM_BOT - LIP_BOT), PURPLE),
    ],
)
OBJECTS = [DEWAR, CANISTER]

# -- scoring --------------------------------------------------------------------------------------------------------------
SEAT_XY = (NX, NY + SLOT_MID)  # the body's centre when seated in the green slot
BODY_TOL = 0.003  # seated: body's underside within this of the table
CENTRE_TOL = 0.006  # seated: body's centre within this of the slot's centre
SEAT_TILT, UPRIGHT_TILT, INSIDE_TILT = 4.0, 15.0, 45.0
LIP_X_TOL = 0.006  # seated: the lip's centre within this of the notch's centre in x
UNDER_SHOULDER_Y = 0.05  # 0.5: the body's centre at least this far toward the green side of the neck's centre

_BODY_CORNERS = [(sx * BODY / 2, sy * BODY / 2, z) for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, BODY_H)]


def _body_points(s):
    return np.array([s.to_world(p) for p in _BODY_CORNERS])


def _facts(s):
    pts = _body_points(s)
    centre = pts.mean(axis=0)
    lip = s.to_world((0.0, LIP_Y, (LIP_BOT + ARM_BOT) / 2))
    lip_bottom = s.to_world((0.0, LIP_Y, LIP_BOT))
    return {
        "bottom": float(pts[:, 2].min()) - TOP,
        "top": float(pts[:, 2].max()) - TOP,
        "cx": float(centre[0]) - NX,
        "cy": float(centre[1]) - NY,
        "tilt": tilt_deg(s),
        "lip_x": float(lip[0]) - NX,
        "lip_y": float(lip[1]) - NY,
        "lip_bottom": float(lip_bottom[2]) - TOP,
    }


def _inside(f):
    """The body is within the dewar's walls and its top is below the deck."""
    return abs(f["cx"]) < HALF_X - WALL - BODY / 2 and abs(f["cy"]) < HALF_Y - WALL - BODY / 2 and f["top"] < DECK_BOT - 0.003


def _held(s, f):
    """More than 1 cm above every support under it (the table, a slot wall): in the air, as when still gripped."""
    return f["bottom"] > SLOT_WALL_H + 0.01


def _seated(s, f):
    return bool(
        abs(f["bottom"]) <= BODY_TOL
        and math.hypot(f["cx"], f["cy"] - SLOT_MID) <= CENTRE_TOL
        and f["tilt"] <= SEAT_TILT
        and f["lip_bottom"] < RING_TOP - 0.002
        and abs(f["lip_x"]) <= LIP_X_TOL
        and NECK <= f["lip_y"] <= NECK + 0.016
    )


def score(objects):
    """1.0 seated in the green slot with the lip in the green notch; 0.5 upright under the green-side shoulder (on a slot
    wall, or the hook on the ring); 0.25 inside the dewar; else 0. Held in the air counts as nothing placed."""
    s = objects["canister"]
    f = _facts(s)
    if _held(s, f) or not _inside(f):
        return 0.0
    if _seated(s, f):
        return 1.0
    if f["tilt"] <= UPRIGHT_TILT and f["cy"] >= UNDER_SHOULDER_Y:
        return 0.5
    if f["tilt"] <= INSIDE_TILT:
        return 0.25
    return 0.0


def success(objects):
    """Seated and still."""
    return score(objects) == 1.0 and objects["canister"].resting()


# The robot keeps out of the neck too, with 4 cm round it, up to the ring's top: a hand that follows the canister in
# would be in the nitrogen vapour. (Checked offline with the URDF; the scorer sees only the canister.)
ARM_KEEP_OUT = [
    {"name": "dewar neck below the ring, and 4 cm round it", "x": (NX - NECK - 0.04, NX + NECK + 0.04),
     "y": (NY - NECK - 0.04, NY + NECK + 0.04), "z_top": TOP + RING_TOP},
]

# -- the demo (left arm only; the right arm holds the ready pose) -----------------------------------------------------------
demo = placeholder_demo

GRIP_Z = 0.108  # the tool point's height above the body's underside when it holds the handle
CARRY = 0.090  # body underside this high: 4 mm over the ring top
LOW = 0.010  # body underside this high while it slides: 4 mm over the slot walls
YAW = -math.pi / 2  # fingers close along x on the handle's 1.8 cm side


def demo_script(ctx):
    arm = ctx.left

    def live():
        return ctx.step_count > 0

    def origin():
        return np.array(ctx.obj("canister").origin, dtype=float)

    s0 = ctx.obj("canister")
    o0 = np.array(s0.origin, dtype=float)
    h0 = s0.to_world((0.0, HANDLE_Y, 0.0))
    grasp = np.array([h0[0], h0[1], TOP + GRIP_Z])
    off = np.array([0.0, HANDLE_Y, GRIP_Z])

    def go(goal, dur):
        yield from arm.move(np.asarray(goal, float) + off, dur, yaw=YAW)

    def refine(goal, tol=0.0015, rounds=6, z=True):
        for _ in range(rounds):
            yield from arm.hold(0.4)
            if not live():
                return
            e = np.asarray(goal, float) - origin()
            if not z:
                e[2] = 0.0
            if np.linalg.norm(e) < tol:
                return
            yield from arm.move(arm.pos + 0.9 * e, 0.5, yaw=YAW)

    yield from arm.move(grasp + (0, 0, 0.09), 3.0, FINGER_OPEN, yaw=YAW)
    yield from arm.move(grasp, 2.0, FINGER_OPEN, yaw=YAW)
    yield from arm.trim_to_measured(grasp)
    yield from arm.hold(0.3)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.5)
    if live():
        o0 = origin()  # the canister may have settled; carry on from where it is
        off = arm.pos - o0
    base = np.array([o0[0], o0[1], TOP])
    over = np.array([NX, NY + 0.013, TOP + CARRY])  # centred in the neck, 6 mm over the ring
    yield from go((base[0], base[1], TOP + CARRY), 2.2)
    yield from go(over, 3.5)
    yield from refine(over, tol=0.0012)
    down = np.array([NX, NY + 0.013, TOP + LOW])
    yield from go(down, (CARRY - LOW) / 0.02)
    yield from refine(down, tol=0.0015)
    slid = np.array([NX, NY + SLOT_MID, TOP + LOW])
    yield from go(slid, 5.5)
    yield from refine(slid, tol=0.0015, z=False)
    seat = np.array([NX, NY + SLOT_MID, TOP - 0.001])
    yield from go(seat, LOW / 0.01 + 0.5)
    yield from arm.hold(0.6)
    yield from arm.grip(FINGER_OPEN, 0.8)
    yield from arm.hold(0.8)
    yield from arm.move(arm.pos + (0, 0, 0.10), 2.0, FINGER_OPEN, yaw=YAW)
    yield from arm.home(3.0)
    yield from ctx.wait(2.0)
