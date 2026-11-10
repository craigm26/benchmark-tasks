"""On the station table: a small-engine intake. A dark grey engine block stands at the back with a black gasket face, two
steel studs pointing toward the arms, and a grey throttle link on its top (a bar in a guide, with an orange pin on a leg
that hangs down beside the studs, pointing toward the right arm; the link slides 5 cm toward the block). A rust-brown muffler
stands beside the block. A carburettor (yellow flange with two holes, blue body, yellow thin throttle lever plate with a
square hole, green grip tab) stands on the table in front of the right arm, flange toward the block.

Requires -E object_jitter=0: the start pose decides the scored positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=hook_the_throttle_link_then_hang_the_carburettor.py -E object_jitter=0 \\
           --instruction "Refit the carburettor to the engine and hook up its throttle link." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
from belimits import _rot, tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips the carburettor: let it open the gripper and park before the episode ends

DARK, BLACK, STEEL = (0.20, 0.20, 0.23), (0.04, 0.04, 0.05), (0.88, 0.88, 0.92)
GREY, ORANGE, RUST = (0.66, 0.66, 0.69), (0.98, 0.45, 0.02), (0.55, 0.26, 0.09)
YELLOW, BLUE, GREEN = (0.97, 0.82, 0.05), (0.08, 0.25, 0.90), (0.10, 0.75, 0.20)
PAD_GREY = (0.42, 0.42, 0.45)

# -- the engine (fixed): block, gasket, studs, intake pad, and the sliding throttle link --------------------------------
# Shape positions are station x, y and z above the table top.
GASKET_X = 0.328  # the gasket face (the seated flange's back face touches it)
STUD_Y = (-0.015, 0.055)
STUD_Z = 0.09
PAD_TOP = 0.069  # the intake pad the seated carburettor rests on, 1 mm under the seated flange bottom
PIN_X0, PIN_TIP_Y, PIN_BASE_Y, PIN_Z = 0.248, 0.036, 0.068, 0.122  # pin at link rest; it moves +x with the link
LINK_STROKE = 0.055


def _e(x0, x1, y0, y1, z0, z1, color):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


ENGINE = Mech(
    "engine",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.6,
    shapes=[
        _e(0.33, 0.38, -0.08, 0.10, 0.0, 0.16, DARK),  # engine block
        _e(0.328, 0.330, -0.03, 0.07, 0.06, 0.12, BLACK),  # gasket face
        _e(0.285, 0.328, STUD_Y[0] - 0.004, STUD_Y[0] + 0.004, STUD_Z - 0.004, STUD_Z + 0.004, STEEL),  # stud
        _e(0.285, 0.328, STUD_Y[1] - 0.004, STUD_Y[1] + 0.004, STUD_Z - 0.004, STUD_Z + 0.004, STEEL),  # stud
        _e(0.283, 0.328, -0.03, 0.07, 0.0, PAD_TOP, PAD_GREY),  # intake pad
        _e(0.31, 0.38, 0.060, 0.066, 0.155, 0.178, GREY),  # link guide, -y wall
        _e(0.31, 0.38, 0.080, 0.086, 0.155, 0.178, GREY),  # link guide, +y wall
        _e(0.31, 0.38, 0.066, 0.080, 0.150, 0.163, GREY),  # link guide floor
    ],
    joints=[
        Joint(
            "link", "slide", axis="x", at=(0, 0, 0), low=0.0, high=LINK_STROKE, spring=0.0, damping=5.0, mass=0.02,
            shapes=[
                _e(0.243, 0.323, 0.068, 0.078, 0.165, 0.175, GREY),  # bar x 0.243..0.323 at rest
                _e(0.243, 0.253, 0.068, 0.078, 0.120, 0.165, GREY),  # leg down from the -x end
                _e(0.243, 0.253, PIN_TIP_Y, PIN_BASE_Y, PIN_Z - 0.005, PIN_Z + 0.005, ORANGE),  # pin toward -y
            ],
        ),
    ],
)

# The muffler: fixed, beside the block, 1 cm clear of it on the right-arm side.
MUFFLER = Box("muffler", pos=(0.34, -0.135, TOP + 0.05), size=(0.08, 0.09, 0.10), color=RUST)

# -- the carburettor (free) ----------------------------------------------------------------------------------------------
# The carburettor's own frame has its origin under the seated lever hole (x), at the middle of the flange (y), at the bottom
# (z); the shapes are written in seated station coordinates and shifted by P0.
P0 = (0.298, 0.02, 0.07)  # seated: origin x, y and bottom height above the table top
HOLE = 0.007  # flange hole half size (1.4 cm), stud half size 0.4 cm: 3 mm a side
LHOLE = 0.008  # lever plate hole half size (1.6 cm), pin half size 0.5 cm: 3 mm a side


def _c(x0, x1, y0, y1, z0, z1, color):
    return box(((x0 + x1) / 2 - P0[0], (y0 + y1) / 2 - P0[1], (z0 + z1) / 2 - P0[2]), (x1 - x0, y1 - y0, z1 - z0), color)


_hy = [STUD_Y[0], STUD_Y[1]]
CARB_SHAPES = [
    # flange x 0.318..0.328, y -0.03..0.07, z 0.07..0.11 with two 1.4 cm holes on the stud lines
    _c(0.318, 0.328, -0.03, 0.07, STUD_Z + HOLE, 0.11, YELLOW),
    _c(0.318, 0.328, -0.03, 0.07, 0.07, STUD_Z - HOLE, YELLOW),
    _c(0.318, 0.328, -0.03, _hy[0] - HOLE, STUD_Z - HOLE, STUD_Z + HOLE, YELLOW),
    _c(0.318, 0.328, _hy[0] + HOLE, _hy[1] - HOLE, STUD_Z - HOLE, STUD_Z + HOLE, YELLOW),
    _c(0.318, 0.328, _hy[1] + HOLE, 0.07, STUD_Z - HOLE, STUD_Z + HOLE, YELLOW),
    _c(0.250, 0.318, 0.0, 0.04, 0.07, 0.11, BLUE),  # body and intake horn
    _c(0.253, 0.271, 0.005, 0.035, 0.11, 0.145, GREEN),  # grip tab: 1.8 cm in x, 3.5 cm tall
    # throttle lever plate x 0.283..0.313, y 0.040..0.046, z 0.10..0.135 with a 1.6 cm square hole centred (0.298, 0.122)
    _c(0.283, 0.290, 0.040, 0.046, 0.10, 0.135, YELLOW),
    _c(0.306, 0.313, 0.040, 0.046, 0.10, 0.135, YELLOW),
    _c(0.290, 0.306, 0.040, 0.046, PIN_Z + LHOLE, 0.135, YELLOW),
    _c(0.290, 0.306, 0.040, 0.046, 0.10, PIN_Z - LHOLE, YELLOW),
]
CARB_ORIGIN0 = (0.229, -0.15, TOP + 0.001)  # start: bounding box centred at (0.22, -0.15), flange toward the block
CARB = Mech("carburettor", pos=CARB_ORIGIN0, mass=0.09, friction=1.0, shapes=CARB_SHAPES)

OBJECTS = [ENGINE, MUFFLER, CARB]

# -- scoring -------------------------------------------------------------------------------------------------------------
SEAT_XY = 0.004  # seated: flange hole line within 4 mm of the stud line (y and z)
SEAT_GAP = 0.003  # flange within 3 mm of the gasket
SEAT_TILT = 3.0  # degrees
ON_STUDS_GAP = 0.03  # 0.5: flange within 3 cm of the gasket, so the studs are through the flange
HOOK_XZ = 0.005  # pin axis within 5 mm of the lever hole centre
HOOK_TILT = 20.0
AIR_STEPS = 20  # 1 s at 20 Hz
OFF_TABLE = 0.01  # the carburettor's bottom more than 1 cm above the table


class _Latch:
    """Flags kept over the whole run; cleared when a trial resets (carburettor teleported back to its start)."""

    def __init__(self):
        self.clear()

    def clear(self):
        self.run = 0  # consecutive steps hooked and off the table
        self.air = False
        self.onstud = False
        self.prev = None


LATCH = _Latch()


@on_reset
def _reset_latch(env=None):
    LATCH.clear()


def _origin_error(c):
    o = np.asarray(c.origin, float)
    return o[0] - P0[0], o[1] - P0[1], o[2] - (TOP + P0[2])


def hooked(c, link) -> bool:
    """The pin passes through the lever hole and its tip is past the plate's inner face."""
    tip = c.to_local((PIN_X0 + link, PIN_TIP_Y, TOP + PIN_Z))
    base = c.to_local((PIN_X0 + link, PIN_BASE_Y, TOP + PIN_Z))
    hole = np.array([0.0, 0.0, PIN_Z - P0[2]])  # lever hole centre in the carburettor's frame
    inner, outer = 0.040 - P0[1], 0.046 - P0[1]
    return bool(
        abs(tip[0] - hole[0]) <= HOOK_XZ
        and abs(tip[2] - hole[2]) <= HOOK_XZ
        and tip[1] < inner - 0.002
        and base[1] > outer + 0.002
        and tilt_deg(c) <= HOOK_TILT
    )


def gap_to_gasket(c) -> float:
    return -_origin_error(c)[0]


def seated(c) -> bool:
    """Flange within 3 mm of the gasket, both studs through the holes (hole lines within 4 mm), square and upright."""
    dx, dy, dz = _origin_error(c)
    return bool(
        -SEAT_GAP - 1e-6 <= dx <= 0.001
        and abs(dy) <= SEAT_XY
        and abs(dz) <= SEAT_XY
        and tilt_deg(c) <= SEAT_TILT
        and _rot(c.quat_xyzw)[0, 0] >= np.cos(np.radians(SEAT_TILT))
    )


def on_studs(c) -> bool:
    dx, dy, dz = _origin_error(c)
    return bool(-ON_STUDS_GAP - 1e-6 <= dx <= 0.001 and abs(dy) <= SEAT_XY and abs(dz) <= SEAT_XY)


def _update(objects):
    c, link = objects["carburettor"], objects["engine"].joints["link"]
    home = np.asarray(CARB_ORIGIN0, float)
    d = float(np.linalg.norm(np.asarray(c.origin, float) - home))
    if LATCH.prev is not None and LATCH.prev > 0.03 and d <= 0.005 and np.linalg.norm(c.lin_vel) < 0.01:
        LATCH.clear()  # a teleport back to the start is a trial reset
    LATCH.prev = d
    h = hooked(c, link)
    off_table = c.low[2] > TOP + OFF_TABLE
    LATCH.run = LATCH.run + 1 if (h and off_table) else 0
    if LATCH.run >= AIR_STEPS:
        LATCH.air = True
    if h and on_studs(c):
        LATCH.onstud = True
    return c, h


def score(objects):
    """Flags are kept over the whole run; the end state decides the top score.
    1.0: pin through the lever hole (H) and the carburettor seated (S: flange within 3 mm of the gasket, both studs through,
    tilted at most 3 degrees), which also needs its bottom within 4 mm of the seat height, so a carburettor held up does not count.
    0.5: H reached with the flange within 3 cm of the gasket. 0.25: H held for 1 s with the carburettor off the table.
    Seated but never hooked scores 0: on the studs the carburettor cannot move sideways, so the pin cannot enter."""
    c, h = _update(objects)
    if h and seated(c):
        return 1.0
    if LATCH.onstud:
        return 0.5
    if LATCH.air:
        return 0.25
    return 0.0


def success(objects):
    """Full score, with the carburettor and the link still."""
    c = objects["carburettor"]
    return score(objects) == 1.0 and c.resting()


# The robot keeps off the hot muffler too: no arm part within 4 cm of it below 25 cm above the table.
ARM_KEEP_OUT = [
    {"name": "muffler and 4 cm around it", "x": (0.26, 0.42), "y": (-0.22, -0.05), "z_top": TOP + 0.25},
]

# -- the scripted demo (right arm, closed loop) -------------------------------------------------------------------------
demo = placeholder_demo
TAB_LOCAL = np.array([0.262 - P0[0], 0.0, 0.061])  # grasp point (fingertip centre) in the carburettor's frame
HOOK_ORIGIN = np.array([P0[0] - 0.05, P0[1] - 0.03, TOP + 0.071])  # 5 cm short, 3 cm toward -y, bottom 1 mm over the pad
SEAT_ORIGIN = np.array([P0[0], P0[1], TOP + 0.0705])
YAW = np.pi / 2  # fingers close along x on the 1.8 cm tab
SLIDE_MPS = 0.01


def _fix(ctx, arm, goal, clip=0.012, duration=0.8):
    """Nudge the grip so the carburettor's origin is at goal (bounded)."""
    err = np.clip(goal - np.asarray(ctx.obj("carburettor").origin, float), -clip, clip)
    yield from arm.move(arm.pos + err, duration, FINGER_CLOSED, rot=arm.rot.copy())


def _creep(ctx, arm, axis, limit_steps, done):
    """Advance the held carburettor along axis (0 = x, 1 = y) at 1 cm/s until done(), or limit_steps."""
    rot = arm.rot.copy()
    for _ in range(limit_steps):
        if done():
            return
        nxt = arm.pos.copy()
        nxt[axis] += SLIDE_MPS * ctx.dt
        arm._set_cartesian(nxt, rot, FINGER_CLOSED)
        yield


def demo_script(ctx):
    """Right arm: grip the green tab, lift the carburettor, carry it in front of the link with the lever hole on the pin's
    line, move it 3 cm toward +y so the pin goes through the hole, then 5 cm toward the block along the studs (the link slides
    along), let go, and park clear."""
    arm = ctx.right
    O, C = FINGER_OPEN, FINGER_CLOSED
    o = np.asarray(ctx.obj("carburettor").origin, float)
    grasp = o + TAB_LOCAL
    yield from arm.move(grasp + np.array([0, 0, 0.14]), 3.2, O, yaw=YAW)
    yield from arm.move(grasp, 1.8, O, yaw=YAW)
    yield from arm.trim_to_measured(grasp, tol=0.0015, max_comp=0.03, step_mps=0.03, max_duration=0.8)
    yield from arm.grip(C, 0.8)
    yield from arm.hold(0.4)
    off = np.asarray(ctx.obj("carburettor").origin, float) - arm.measured_pos  # carburettor origin minus fingertip centre

    # lift, carry over to the hook position, lower
    yield from arm.move(arm.pos + np.array([0, 0, 0.10]), 1.6, C, rot=arm.rot.copy())
    hook_hi = HOOK_ORIGIN + np.array([0, 0, 0.03])
    yield from arm.move(hook_hi - off + (arm.pos - arm.measured_pos), 3.2, C, rot=arm.rot.copy())
    tcp = HOOK_ORIGIN - off
    yield from arm.move(tcp + (arm.pos - arm.measured_pos), 1.6, C, rot=arm.rot.copy())
    yield from arm.trim_to_measured(tcp, tol=0.0015, max_comp=0.03, step_mps=0.03, max_duration=0.8)
    live = float(np.linalg.norm(np.asarray(ctx.obj("carburettor").origin, float) - o)) > 0.02  # False in the offline replay
    for _ in range(2 if live else 0):
        yield from _fix(ctx, arm, HOOK_ORIGIN)
    yield from arm.hold(0.4)

    # hook: 3 cm toward +y until the pin is through the lever hole
    def hooked_now():
        s = ctx.states()
        return hooked(s["carburettor"], s["engine"].joints["link"]) and _origin_error(s["carburettor"])[1] >= -0.0005

    yield from _creep(ctx, arm, 1, 70, hooked_now)
    yield from arm.hold(0.6)

    # hang: 5 cm toward the block along the studs, the link sliding with the lever
    def at_gasket():
        return _origin_error(ctx.obj("carburettor"))[0] >= -0.0008

    yield from _creep(ctx, arm, 0, 108, at_gasket)
    yield from arm.hold(0.8)

    # let go and park well clear
    yield from arm.grip(O, 0.8)
    yield from arm.hold(0.6)
    yield from arm.move(arm.pos + np.array([0, 0, 0.035]), 1.4, O, rot=arm.rot.copy())  # open fingers clear of the tab top
    yield from arm.move(np.array([0.20, -0.02, TOP + 0.19]), 2.0, O, rot=arm.rot.copy())
    yield from arm.move(np.array([0.16, -0.20, TOP + 0.20]), 2.0, O, rot=arm.rot.copy())
    yield from arm.home(3.0)
    yield from ctx.wait(0.5)
