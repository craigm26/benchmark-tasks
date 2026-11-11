"""On the station table, a little ahead of the arms: a small food dicer. A fixed grey frame (two cheeks with a ledge on
each inner face, retaining lips over the rear half and a back stop) holds a worn red blade grid with a yellow grip tab at
its front, slid fully home. A white hinged pusher lies down on the grid, its nine grey fingers standing through the grid's
nine openings; it is hinged on the right-hand (-y) side and has an orange handle on its left end. A new blue blade grid
with a green grip tab lies on a grey stand to the left of the dicer. A dark drain board for the worn grid lies on the
table to the right.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_a_dicer_blade_grid_under_its_pusher.py -E object_jitter=0 \\
           --instruction "Fit the new blue blade grid in the dicer in place of the worn red one." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, inside, is_airborne, is_flat_on, overlaps  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips things: let the last release and the parked arms be seen before the episode ends

GREY = (0.46, 0.48, 0.52)
DARK = (0.24, 0.25, 0.28)
WHITE = (0.93, 0.93, 0.90)
STEEL = (0.62, 0.64, 0.68)
ORANGE = (0.97, 0.52, 0.05)
RED = (0.82, 0.08, 0.08)
YELLOW = (0.97, 0.85, 0.05)
BLUE = (0.10, 0.28, 0.88)
GREEN = (0.10, 0.75, 0.25)

# -- the dicer (station frame, heights above the table) ---------------------------------------------------------------
D0 = (0.32, 0.0, TOP)  # the dicer's origin: the middle of the grid bay, on the table
BAY_X = (0.27, 0.365)  # the bay runs from the open front to the back stop
BAY_HALF = 0.049  # cheek inner faces at y +-4.9 cm: 4 mm a side round a 9 cm grid
CHEEK_OUT = 0.059
CHEEK_TOP = 0.084
LEDGE_IN = 0.037
LEDGE_Z = (0.05, 0.06)  # the grid slides on the ledge tops, 6 cm above the table
LIP_X = (0.32, 0.365)  # retaining lips over the rear half of the bay
LIP_Z = (0.074, 0.084)  # 4 mm over a seated grid, whose top is 7 cm up
STOP_X = (0.365, 0.375)
HINGE_Y, HINGE_Z = -0.105, 0.065  # the pusher's hinge axis runs along x, level with a seated grid
BLOCK_Z = (0.087, 0.099)  # the pusher block when down: 3 mm over the lips
HANDLE_X, HANDLE_Y, HANDLE_Z = (0.305, 0.335), (0.027, 0.045), (0.099, 0.134)
FINGER_W = 0.012
FINGER_XS, FINGER_YS = (0.293, 0.320, 0.347), (-0.027, 0.0, 0.027)
FINGER_BOTTOM = 0.055
PUSHER_UP = 1.9  # rad: the back stop, past vertical, where the pusher rests under its own weight
UP_MIN = 1.5  # rad: counts as raised
DOWN_MAX = 0.05  # rad: counts as lowered (the block within about 5 mm of its down height)


def _fixed(x0, x1, y0, y1, z0, z1, color):
    """A fixed part of the dicer, from its world x and y extents and its heights above the table."""
    return box(((x0 + x1) / 2 - D0[0], (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _moving(x0, x1, y0, y1, z0, z1, color):
    """A part of the pusher as it lies when down, in the hinge's frame."""
    return box(((x0 + x1) / 2 - D0[0], (y0 + y1) / 2 - HINGE_Y, (z0 + z1) / 2 - HINGE_Z), (x1 - x0, y1 - y0, z1 - z0), color)


def _dicer() -> Mech:
    shapes = []
    for s in (-1, 1):
        a, b = sorted((s * BAY_HALF, s * CHEEK_OUT))
        shapes.append(_fixed(BAY_X[0], STOP_X[1], a, b, 0.0, CHEEK_TOP, GREY))  # cheek
        a, b = sorted((s * LEDGE_IN, s * BAY_HALF))
        shapes.append(_fixed(*BAY_X, a, b, *LEDGE_Z, GREY))  # ledge
        shapes.append(_fixed(*LIP_X, a, b, *LIP_Z, GREY))  # retaining lip
    shapes.append(_fixed(*STOP_X, -BAY_HALF, BAY_HALF, LEDGE_Z[0], CHEEK_TOP, GREY))  # back stop
    shapes.append(_fixed(0.275, 0.365, HINGE_Y - 0.01, -CHEEK_OUT, 0.0, 0.05, GREY))  # hinge pedestal
    parts = [
        _moving(0.275, 0.365, -0.045, 0.045, *BLOCK_Z, WHITE),  # push block
        _moving(*HANDLE_X, *HANDLE_Y, *HANDLE_Z, ORANGE),  # handle
    ]
    for x0, x1 in ((0.275, 0.295), (0.345, 0.365)):  # two arms from the block to the hinge
        parts.append(_moving(x0, x1, HINGE_Y - 0.01, -0.045, *BLOCK_Z, GREY))
        parts.append(_moving(x0, x1, HINGE_Y - 0.01, HINGE_Y + 0.01, 0.055, BLOCK_Z[0], GREY))
    for fx in FINGER_XS:
        for fy in FINGER_YS:
            parts.append(_moving(fx - FINGER_W / 2, fx + FINGER_W / 2, fy - FINGER_W / 2, fy + FINGER_W / 2,
                                 FINGER_BOTTOM, BLOCK_Z[0], STEEL))
    pusher = Joint("pusher", "hinge", axis="x", at=(0.0, HINGE_Y, HINGE_Z), low=0.0, high=PUSHER_UP, spring=0.0,
                   damping=2.0, mass=0.15, shapes=tuple(parts))
    return Mech("dicer", pos=D0, mass=None, shapes=shapes, joints=[pusher])


# -- the blade grids --------------------------------------------------------------------------------------------------
G = 0.09  # frame edge
GT = 0.01  # frame thickness
BORDER = 0.006
BLADE = 0.003
PLATE_X = (-0.071, -0.045)  # front handle plate, in the grid frame (origin: the middle of the frame's underside)
PLATE_Y = 0.05
TAB_CX, TAB_X, TAB_Y, TAB_H = -0.059, 0.018, 0.03, 0.03  # upright grip tab on the plate
GRID_MASS = 0.08
GRIP_UP = 0.028  # where the gripper holds the tab, above the grid's underside


def _grid(name: str, pos, frame_color, tab_color) -> Mech:
    h, inner = G / 2, G - 2 * BORDER
    shapes = [
        box((-(h - BORDER / 2), 0, GT / 2), (BORDER, G, GT), frame_color),
        box((h - BORDER / 2, 0, GT / 2), (BORDER, G, GT), frame_color),
        box((0, -(h - BORDER / 2), GT / 2), (inner, BORDER, GT), frame_color),
        box((0, h - BORDER / 2, GT / 2), (inner, BORDER, GT), frame_color),
    ]
    off = ((inner - 2 * BLADE) / 3 + BLADE) / 2  # two blades each way leave nine 2.4 cm openings
    for k in (-off, off):
        shapes.append(box((k, 0, GT / 2), (BLADE, inner, GT), frame_color))
        shapes.append(box((0, k, GT / 2), (inner, BLADE, GT), frame_color))
    shapes.append(box((sum(PLATE_X) / 2, 0, GT / 2), (PLATE_X[1] - PLATE_X[0], PLATE_Y, GT), frame_color))
    shapes.append(box((TAB_CX, 0, GT + TAB_H / 2), (TAB_X, TAB_Y, TAB_H), tab_color))
    return Mech(name, pos=tuple(pos), mass=GRID_MASS, shapes=shapes)


STAND_X, STAND_Y, STAND_H = (0.17, 0.29), (0.12, 0.22), 0.02
BOARD_X, BOARD_Y, BOARD_H = (0.15, 0.27), (-0.24, -0.12), 0.01
REST = 0.0002  # objects start a hair above what they rest on
RED_HOME = (BAY_X[1] - G / 2, 0.0, TOP + LEDGE_Z[1] + REST)
BLUE_START = (0.245, 0.17, TOP + STAND_H + REST)

DICER = _dicer()
WORN = _grid("worn_grid", RED_HOME, RED, YELLOW)
NEW = _grid("new_grid", BLUE_START, BLUE, GREEN)
STAND = Box("grid_stand", pos=(sum(STAND_X) / 2, sum(STAND_Y) / 2, TOP + STAND_H / 2),
            size=(STAND_X[1] - STAND_X[0], STAND_Y[1] - STAND_Y[0], STAND_H), color=GREY)
BOARD = Box("drain_board", pos=(sum(BOARD_X) / 2, sum(BOARD_Y) / 2, TOP + BOARD_H / 2),
            size=(BOARD_X[1] - BOARD_X[0], BOARD_Y[1] - BOARD_Y[0], BOARD_H), color=DARK)

OBJECTS = [DICER, WORN, NEW, STAND, BOARD]
SPECS = {o.name: o for o in OBJECTS}

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_GAP = 0.003  # m: the new grid's rear face within this of the back stop
HOME_Z_TOL = 0.003  # m: its underside within this of the ledge tops
HOME_TILT = 3.0  # degrees
BOARD_Z_TOL = 0.004
BOARD_TILT = 5.0
FRAME = SimpleNamespace(size=(G, G, GT))  # a grid's 9 x 9 x 1 cm frame, without its plate and tab
BAY = Rect((BAY_X[0] - 0.006, STOP_X[0] + 0.001), (-BAY_HALF - 0.001, BAY_HALF + 0.001))
DICER_OUTLINE = Rect((BAY_X[0], STOP_X[1]), (-CHEEK_OUT, CHEEK_OUT))
BOARD_RECT = Rect(BOARD_X, BOARD_Y)
SUPPORTS = (TOP, TOP + BOARD_H, TOP + STAND_H, TOP + LEDGE_Z[1], TOP + CHEEK_TOP)


def _origin(s, name=None) -> np.ndarray:
    o = getattr(s, "origin", None)
    return np.asarray(o if o is not None else s.pos, float)


def _frame(s):
    """The grid's 9 x 9 x 1 cm frame as a box state the belimits helpers read (its real pose, turned and tipped)."""
    centre = _origin(s) + s.rotation() @ np.array([0.0, 0.0, GT / 2])
    return SimpleNamespace(pos=centre, quat_xyzw=np.asarray(s.quat_xyzw, float))


def _corners(s) -> np.ndarray:
    pts = np.array([[a * G / 2, b * G / 2, c * GT] for a in (-1, 1) for b in (-1, 1) for c in (0, 1)])
    return _origin(s) + pts @ s.rotation().T


def held_up(s) -> bool:
    """The grid's underside is more than 1 cm above every support: still in a hand, or falling."""
    return is_airborne(_frame(s), FRAME, SUPPORTS)


def stop_gap(s) -> float:
    """Metres between the grid's rear face and the back stop."""
    return float(STOP_X[0] - _corners(s)[:, 0].max())


def home(s) -> bool:
    """On both ledges between the cheeks, rear face within 3 mm of the back stop, tilted at most 3 degrees."""
    f = _frame(s)
    return bool(not held_up(s) and is_flat_on(f, FRAME, TOP + LEDGE_Z[1], HOME_Z_TOL, HOME_TILT)
                and inside(f, FRAME, BAY) and -0.001 <= stop_gap(s) <= HOME_GAP)


def out(s) -> bool:
    """Wholly out of the dicer and lying flat on the drain board: underside within 4 mm of the board's top, tilted at
    most 5 degrees, the middle of the frame over the board."""
    f = _frame(s)
    x, y = f.pos[0], f.pos[1]
    return bool(not held_up(s) and not overlaps(f, FRAME, DICER_OUTLINE)
                and is_flat_on(f, FRAME, TOP + BOARD_H, BOARD_Z_TOL, BOARD_TILT)
                and BOARD_RECT.x0 <= x <= BOARD_RECT.x1 and BOARD_RECT.y0 <= y <= BOARD_RECT.y1)


def pusher_angle(objects) -> float:
    return float(objects["dicer"].joints.get("pusher", 0.0))


def score(objects):
    """Home (H): the blue grid on the ledges against the back stop. Down (D): the pusher within 0.05 rad of down. Out (O):
    the red grid flat on the drain board. 1.0 for H, D and O. With the pusher up (1.5 rad or more): 0.5 for H and O, 0.25
    for O alone. Otherwise 0. A grid held in the air is neither home nor out."""
    h, o, a = home(objects["new_grid"]), out(objects["worn_grid"]), pusher_angle(objects)
    if h and o and a <= DOWN_MAX:
        return 1.0
    if o and a >= UP_MIN:
        return 0.5 if h else 0.25
    return 0.0


def success(objects):
    """The full score with both grids still; it has to hold for 8 s."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in ("new_grid", "worn_grid"))


# The robot keeps out of the bay too: no part of either arm goes below 8.5 cm inside the cutter frame, so the grids are
# worked by their front tabs and the pusher by its handle (checked offline with the URDF; the scorer sees only objects).
KEEP_OUT_TOP = 0.085
ARM_KEEP_OUT = [
    {"name": "inside the cutter frame", "x": (BAY_X[0], STOP_X[0] + 0.005), "y": (-BAY_HALF, BAY_HALF),
     "z_top": TOP + KEEP_OUT_TOP},
]

# -- the scripted demo (closed loop) ----------------------------------------------------------------------------------
demo = placeholder_demo

TAB_GAP = -0.023  # finger joint that leaves 4.6 cm between the pads, round the 3 cm tab and clear of the cheeks
HANDLE_GAP = -0.025  # 5 cm between the pads, round the 3 cm handle
HANDLE_GRIP = (0.036, 0.118)  # where the handle is held when the pusher is down: y, and height above the table
LEAN0, LEAN = 0.3, 0.3  # rad: the gripper starts leaned toward the hinge and leans on by 0.3 of the pusher's turn; the
# handle pivots between the pads for the rest (the arm cannot follow the whole turn)
CARRY_Z = TOP + 0.15
TAB_OFF = np.array([TAB_CX, 0.0, GRIP_UP])
MOUTH_X = 0.312 - G / 2  # grid middle when its rear edge is just in front of the lips
RIGHT_LANE = (0.19, -0.24)  # where the right arm crosses in front of the raised pusher


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def handle_pos(theta: float) -> np.ndarray:
    """Where the gripper holds the handle when the pusher is at theta."""
    rel = np.array([0.0, HANDLE_GRIP[0] - HINGE_Y, HANDLE_GRIP[1] - HINGE_Z])
    return np.array([D0[0], HINGE_Y, TOP + HINGE_Z]) + _rx(theta) @ rel


def handle_rot(theta: float) -> np.ndarray:
    """Fingers closing along x, across the handle's 3 cm; the approach leans toward the hinge as the pusher swings."""
    return _rx(LEAN0 + LEAN * theta) @ grip_rot(yaw=math.pi / 2)


class _Run:
    """What the demo knows as it goes. In the simulator it reads the objects; in an offline replay (where nothing ever
    moves) it carries the poses the plan expects, so the later stages are checked on the path they really take."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.live = False
        self.angle = 0.0
        self.origin = {"worn_grid": np.array(RED_HOME, float), "new_grid": np.array(BLUE_START, float)}

    def watch(self, arm, gen):
        """Run a move and note whether the arm lags its commands, which only a simulated arm does."""
        for _ in gen:
            yield
            if float(np.abs(arm.measured_q - arm.q).max()) > 1e-4:
                self.live = True

    def pusher(self) -> float:
        return float(self.ctx.obj("dicer").joints.get("pusher", 0.0)) if self.live else self.angle

    def grid(self, name) -> np.ndarray:
        return _origin(self.ctx.obj(name)) if self.live else self.origin[name].copy()

    def carried(self, name, arm):
        """Offline: the held grid goes where the hand goes."""
        if not self.live:
            self.origin[name] = arm.pos - TAB_OFF


def _settle(run, arm, target, rot, finger, seconds=1.2, gain=0.2):
    """Bring the hand's measured position onto `target`: nulls the arm's sag before a grip or a release."""
    target = np.asarray(target, float)
    comp = arm.pos - target
    for _ in range(round(seconds / run.ctx.dt)):
        if run.live:
            comp = np.clip(comp + gain * (target - arm.measured_pos), -0.03, 0.03)
        arm._set_cartesian(target + comp, rot, finger)
        yield


def _reach(run, arm, target, rot, finger, hover, seconds=2.0):
    """Go to `target + hover`, null the sag there, come down on the same correction and null it again."""
    target, hover = np.asarray(target, float), np.asarray(hover, float)
    yield from run.watch(arm, arm.move(target + hover, seconds, finger, rot=rot))
    yield from _settle(run, arm, target + hover, rot, finger, 0.8)
    bias = arm.pos - (target + hover)
    yield from arm.move(target + bias, 1.5, finger, rot=rot)
    yield from _settle(run, arm, target, rot, finger, 1.0)


def _swing(run, arm, goal, rate):
    """Turn the pusher by its handle to `goal` rad at `rate` rad/s. The commanded point never runs more than 0.3 rad ahead
    of the pusher, and the move ends only when the pusher is within 0.03 rad of the goal."""
    ctx, sign = run.ctx, (1.0 if goal > run.pusher() else -1.0)
    plan = run.pusher()
    over = goal + sign * 0.06  # aim a little past the stop so the arm's give does not leave it short
    for _ in range(round(40.0 / ctx.dt)):
        meas = run.pusher()
        if abs(meas - goal) <= 0.03:
            break
        plan = plan + sign * rate * ctx.dt
        plan = min(plan, over, meas + 0.3) if sign > 0 else max(plan, over, meas - 0.3)
        arm._set_cartesian(handle_pos(plan), handle_rot(plan), FINGER_CLOSED)
        if not run.live:
            run.angle = float(np.clip(plan, 0.0, PUSHER_UP))
        yield
    yield from arm.hold(0.4)


def _raise_pusher(run, arm):
    rot = handle_rot(0.0)
    yield from _reach(run, arm, handle_pos(0.0), rot, HANDLE_GAP, rot @ np.array([0.0, 0.0, -0.05]), 3.0)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)
    yield from _swing(run, arm, PUSHER_UP, 0.25)
    yield from arm.grip(HANDLE_GAP, 0.8)
    rot = arm.rot.copy()
    back = rot @ np.array([0.0, 0.0, -1.0]) * 0.05  # straight back along the gripper's own axis
    yield from arm.move(arm.pos + back, 1.2, HANDLE_GAP, rot=rot)
    yield from arm.move((RIGHT_LANE[0], RIGHT_LANE[1], CARRY_Z), 2.5, HANDLE_GAP, yaw=0.0)


def _take_tab(run, arm, name):
    tab = run.grid(name) + TAB_OFF
    yield from _reach(run, arm, tab, grip_rot(), TAB_GAP, (0.0, 0.0, 0.05), 2.5)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)


def _steer(run, arm, name, step, want_y=0.0, z_hold=None):
    """One control step of a slow guided move: `step` (x, y, z) plus a small correction that keeps the grid on `want_y`
    and the hand at the height `z_hold`."""
    dy = dz = 0.0
    if run.live:
        dy = float(np.clip(0.2 * (want_y - run.grid(name)[1]), -0.0005, 0.0005))
        if z_hold is not None:
            dz = float(np.clip(0.2 * (z_hold - arm.measured_pos[2]), -0.0005, 0.0005))
    arm._set_cartesian(arm.pos + np.asarray(step, float) + np.array([0.0, dy, dz]), arm.rot, FINGER_CLOSED)
    run.carried(name, arm)


def _remove_worn(run, arm):
    """Draw the worn grid out by its tab until its rear edge is clear of the lips, lift it and lay it on the board."""
    ctx, name = run.ctx, "worn_grid"
    yield from arm.move((RIGHT_LANE[0], 0.0, CARRY_Z), 2.5, TAB_GAP, yaw=0.0)
    yield from _take_tab(run, arm, name)
    z_hold = float(arm.measured_pos[2]) if run.live else None
    for _ in range(round(8.0 / ctx.dt)):
        if run.grid(name)[0] + G / 2 <= LIP_X[0] - 0.008:
            break
        _steer(run, arm, name, (-0.02 * ctx.dt, 0.0, 0.0), 0.0, z_hold)
        yield
    yield from arm.move((arm.pos[0], arm.pos[1], CARRY_Z), 1.5, FINGER_CLOSED, yaw=0.0)
    run.carried(name, arm)
    drop_xy = np.array([sum(BOARD_X) / 2, sum(BOARD_Y) / 2])
    yield from arm.move((drop_xy[0] + TAB_OFF[0], 0.0, CARRY_Z), 1.5, FINGER_CLOSED, yaw=0.0)
    yield from arm.move((drop_xy[0] + TAB_OFF[0], drop_xy[1], CARRY_Z), 2.5, FINGER_CLOSED, yaw=0.0)
    run.carried(name, arm)
    for _ in range(round(1.5 / ctx.dt) if run.live else 0):  # put the grid itself over the board's middle
        e = np.clip(0.2 * (drop_xy - run.grid(name)[:2]), -0.001, 0.001)
        arm._set_cartesian(arm.pos + np.array([e[0], e[1], 0.0]), arm.rot, FINGER_CLOSED)
        yield
    floor = TOP + BOARD_H + GRIP_UP
    for _ in range(round(12.0 / ctx.dt)):  # lower until the grid is on the board
        low = run.grid(name)[2] - (TOP + BOARD_H)
        if low <= 0.0015 or (not run.live and arm.pos[2] <= floor + 0.001) or arm.pos[2] <= floor - 0.012:
            break
        arm._set_cartesian(arm.pos + np.array([0.0, 0.0, -0.02 * ctx.dt]), arm.rot, FINGER_CLOSED)
        run.carried(name, arm)
        yield
    yield from arm.hold(0.4)
    yield from arm.grip(FINGER_OPEN, 0.8)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.12]), 1.5, FINGER_OPEN, yaw=0.0)
    yield from arm.home(3.0)


def _seat_new(run, arm):
    """Carry the new grid to the mouth of the bay, lower it onto the ledges and slide it under the lips to the stop."""
    ctx, name = run.ctx, "new_grid"
    yield from _take_tab(run, arm, name)
    yield from arm.move((arm.pos[0], arm.pos[1], CARRY_Z), 1.5, FINGER_CLOSED, yaw=0.0)
    over = TOP + LEDGE_Z[1] + 0.04  # the grid's underside 4 cm over the ledges
    mouth = np.array([MOUTH_X, 0.0])
    yield from arm.move((mouth[0] + TAB_OFF[0], mouth[1], over + GRIP_UP), 3.0, FINGER_CLOSED, yaw=0.0)
    run.carried(name, arm)
    for _ in range(round(2.0 / ctx.dt) if run.live else 0):  # centre the grid itself between the cheeks
        e = np.clip(0.2 * (mouth - run.grid(name)[:2]), -0.001, 0.001)
        arm._set_cartesian(arm.pos + np.array([e[0], e[1], 0.0]), arm.rot, FINGER_CLOSED)
        yield
    floor = TOP + LEDGE_Z[1] + GRIP_UP
    for _ in range(round(12.0 / ctx.dt)):  # lower until it is on both ledges
        low = run.grid(name)[2] - (TOP + LEDGE_Z[1])
        if low <= 0.001 or (not run.live and arm.pos[2] <= floor + 0.0005) or arm.pos[2] <= floor - 0.012:
            break
        _steer(run, arm, name, (0.0, 0.0, -0.015 * ctx.dt), 0.0)
        yield
    yield from arm.hold(0.4)
    z_hold = float(arm.measured_pos[2]) if run.live else None
    for _ in range(round(14.0 / ctx.dt)):  # slide it home at 1 cm/s
        if run.grid(name)[0] + G / 2 >= STOP_X[0] - 0.001:
            break
        _steer(run, arm, name, (0.01 * ctx.dt, 0.0, 0.0), 0.0, z_hold)
        yield
    yield from arm.hold(0.5)
    yield from arm.grip(TAB_GAP, 0.8)
    yield from arm.move(arm.pos + np.array([-0.05, 0.0, 0.0]), 1.2, TAB_GAP, yaw=0.0)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), 1.2, FINGER_OPEN, yaw=0.0)
    yield from arm.home(3.0)


def _lower_pusher(run, arm):
    up = run.pusher()
    rot = handle_rot(up)
    back = rot @ np.array([0.0, 0.0, -1.0]) * 0.05
    yield from _reach(run, arm, handle_pos(up), rot, HANDLE_GAP, back, 3.0)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.hold(0.3)
    yield from _swing(run, arm, 0.0, 0.2)
    yield from arm.grip(HANDLE_GAP, 0.8)
    rot = arm.rot.copy()
    yield from arm.move(arm.pos + rot @ np.array([0.0, 0.0, -0.05]), 1.2, HANDLE_GAP, rot=rot)
    yield from arm.home(3.5)


def demo_script(ctx):
    """Right arm: swing the pusher up by its handle to the back stop. Right arm: draw the red grid out by its yellow tab,
    lift it and lay it on the drain board. Left arm: carry the blue grid to the mouth of the bay by its green tab, lower
    it onto the ledges and slide it under the lips to the stop. Right arm: swing the pusher down through the grid.
    The grids are held only by their front tabs and the pusher only by its handle."""
    run = _Run(ctx)
    yield from _raise_pusher(run, ctx.right)
    yield from _remove_worn(run, ctx.right)
    yield from _seat_new(run, ctx.left)
    yield from _lower_pusher(run, ctx.right)
    yield from ctx.wait(8.5)  # both arms parked and open while the end state holds for SUCCESS_HOLD_S
