"""On the station table: a half-scale bypass closet door track at the back. A dark grey head track (a flat plate on two end
posts, with a front lip and a rear lip) runs over a thin orange floor guide rib; a silver panel, the other door, stands
fixed behind. The left end post, on the left arm's side, is the jamb. A white door 12 cm long and 14 cm tall stands loose
on the table in front of the left arm, with a dark shoe (two cheeks) at each end of its bottom and a blue handle on its
front face, a little toward its right-hand end.

Requires -E object_jitter=0: the start pose decides the scored positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=hang_a_bypass_closet_door_on_its_track.py -E object_jitter=0 \\
           --instruction "Hang the white closet door on its track and slide it shut." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
from belimits import PeakTracker, tilt_deg  # noqa: E402,F401
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 16.0  # the demo grips the door: let it open the gripper and park clear before the episode ends

WHITE, DARK, SILVER = (0.96, 0.96, 0.94), (0.30, 0.30, 0.34), (0.80, 0.82, 0.86)
ORANGE, BLUE, SLATE, SHOE = (0.98, 0.50, 0.05), (0.08, 0.25, 0.90), (0.38, 0.50, 0.62), (0.08, 0.08, 0.10)

# -- the closet (fixed). Shape positions are station x, y and z above the table top. ----------------------------------------
CHANNEL = (0.29, 0.33)  # clear width between the lips: 4.0 cm
FRONT_LIP_Z, REAR_LIP_Z, CEILING_Z = 0.135, 0.135, 0.183  # lip bottoms and the head plate's underside
RIB_X, RIB_H = (0.306, 0.314), 0.015
JAMB_Y = 0.21  # the left end post's inner face


def _e(x0, x1, y0, y1, z0, z1, color):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


CLOSET = Mech(
    "closet",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.6,
    shapes=[
        _e(0.28, 0.36, -0.23, 0.23, CEILING_Z, CEILING_Z + 0.015, DARK),  # head plate
        _e(0.28, 0.36, -0.23, -0.21, 0.0, CEILING_Z, DARK),  # right end post
        _e(0.28, 0.36, 0.21, 0.23, 0.0, CEILING_Z, DARK),  # left end post: the jamb
        _e(0.28, 0.29, -0.21, 0.21, FRONT_LIP_Z, CEILING_Z, DARK),  # front lip
        _e(0.33, 0.34, -0.21, 0.21, REAR_LIP_Z, CEILING_Z, DARK),  # rear lip
        _e(RIB_X[0], RIB_X[1], -0.21, 0.21, 0.0, RIB_H, ORANGE),  # floor guide rib
        _e(0.345, 0.355, -0.205, 0.0, 0.005, 0.17, SILVER),  # the other door
    ],
)

# -- the door (free). Its frame: origin at the middle of the panel's bottom face, x across, y along, z up. -------------------
PANEL_T, PANEL_L, PANEL_H = 0.018, 0.12, 0.14
CHEEK_T, CHEEK_H, SHOE_Y = 0.006, 0.012, 0.05  # shoe cheeks 0.6 x 2.0 x 1.2 cm; shoe centres at y +-5 cm
HANDLE = (0.025, 0.018, 0.03)  # out of the -x face, 2.5 x 1.8 x 3.0 cm
HANDLE_Y = -0.035  # 3.5 cm toward the -y end, so the hand stays well short of the jamb when the door is shut
HANDLE_LOCAL = (-PANEL_T / 2 - HANDLE[0] / 2, HANDLE_Y, 0.07)


def _d(at, size, color):
    return box(at, size, color)


DOOR_SHAPES = [_d((0, 0, PANEL_H / 2), (PANEL_T, PANEL_L, PANEL_H), WHITE), _d(HANDLE_LOCAL, HANDLE, BLUE)]
for _ys in (-SHOE_Y, SHOE_Y):
    for _sx in (-1, 1):
        DOOR_SHAPES.append(_d((_sx * (PANEL_T / 2 + CHEEK_T / 2), _ys, -CHEEK_H / 2), (CHEEK_T, 0.02, CHEEK_H), SHOE))

DOOR_Y0 = 0.06  # start and hung y of the door's middle
DOOR_X0 = 0.17
DOOR_ORIGIN0 = (DOOR_X0, DOOR_Y0, TOP + CHEEK_H)  # standing on its shoes
DOOR = Mech("door", pos=DOOR_ORIGIN0, mass=0.10, friction=1.0, shapes=DOOR_SHAPES)

OBJECTS = [CLOSET, DOOR]

# -- scoring -------------------------------------------------------------------------------------------------------------
LIP_TOUCH = 0.0015  # a door leaning on a lip may sit this far into it (contact softness)
LIP_DEPTH = 0.015  # hung: the door top at least 1.5 cm above the rear lip's bottom
SHOE_OVERLAP = 0.008  # cheeks overlap the rib 8 mm or more in height
HUNG_TILT = 10.0  # degrees
SHUT_TOL = 0.003  # the door's +y edge within 3 mm of the jamb
CHANNEL_DEPTH = 0.010  # sticky 0.25: the door top at least 1 cm above the rear lip's bottom
AIR_STEPS = 20  # 1 s at 20 Hz
ON_TABLE = 0.0015  # a door point this close to the table top is on the table
AIRBORNE_Z = 0.03  # the door's lowest point more than 3 cm above the table (the rib is 1.5 cm)

_CHEEK_LEVELS = np.arange(-CHEEK_H, 1e-9, 0.001)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _world(d, local):
    return np.asarray(d.origin, float) + (_rot(d.quat_xyzw) @ np.asarray(local, float).T).T


def _top_corners(d):
    loc = [(sx * PANEL_T / 2, sy * PANEL_L / 2, PANEL_H) for sx in (-1, 1) for sy in (-1, 1)]
    return _world(d, loc)


def _lowest(d):
    """Height above the table top of the door's lowest point (a shoe cheek's bottom edge)."""
    loc = [(sx * (PANEL_T / 2 + CHEEK_T), sy * 0.06, -CHEEK_H) for sx in (-1, 1) for sy in (-1, 1)]
    return float(_world(d, loc)[:, 2].min() - TOP)


def top_in_channel(d, depth) -> bool:
    """All four top corners at least ``depth`` above the rear lip's bottom and between the lips (and the end posts)."""
    c = _top_corners(d)
    return bool(
        np.all(c[:, 2] - TOP >= REAR_LIP_Z + depth)
        and np.all(c[:, 0] >= CHANNEL[0] - LIP_TOUCH) and np.all(c[:, 0] <= CHANNEL[1] + LIP_TOUCH)
        and np.all(np.abs(c[:, 1]) < JAMB_Y + 0.002)
    )


def shoes_straddle(d) -> bool:
    """At each end, the rib lies between the shoe's two cheeks for 8 mm or more of their height."""
    rot = _rot(d.quat_xyzw)
    if rot[0, 0] < np.cos(np.radians(25.0)):  # turned about the vertical or flipped
        return False
    engaged = np.ones(len(_CHEEK_LEVELS), bool)
    for ys in (-SHOE_Y, SHOE_Y):
        for dy in (-0.008, 0.008):
            a = _world(d, [(-PANEL_T / 2, ys + dy, z) for z in _CHEEK_LEVELS])  # the -x cheek's inner face
            b = _world(d, [(PANEL_T / 2, ys + dy, z) for z in _CHEEK_LEVELS])  # the +x cheek's inner face
            for p in (a, b):
                engaged &= (p[:, 2] - TOP >= 0.0) & (p[:, 2] - TOP <= RIB_H) & (np.abs(p[:, 1]) < JAMB_Y)
            engaged &= (a[:, 0] <= RIB_X[0]) & (b[:, 0] >= RIB_X[1])
    return bool(engaged.sum() * 0.001 >= SHOE_OVERLAP + 0.001 - 1e-9)


def hung(d) -> bool:
    return bool(shoes_straddle(d) and top_in_channel(d, LIP_DEPTH) and tilt_deg(d) <= HUNG_TILT)


def shut(d) -> bool:
    edge = float(_world(d, [(sx * PANEL_T / 2, PANEL_L / 2, z) for sx in (-1, 1) for z in (0.0, PANEL_H)])[:, 1].max())
    return bool(JAMB_Y - SHUT_TOL <= edge <= JAMB_Y + 0.002)


class _Latch:
    """Flags kept over the whole run; cleared when a trial resets (the door teleported back to its start)."""

    def __init__(self):
        self.clear()

    def clear(self):
        self.run = 0
        self.captured = False
        self.prev = None


LATCH = _Latch()


@on_reset
def _reset_latch(env=None):
    LATCH.clear()


def _update(objects):
    d = objects["door"]
    home = np.asarray(DOOR_ORIGIN0, float)
    dist = float(np.linalg.norm(np.asarray(d.origin, float) - home))
    if LATCH.prev is not None and LATCH.prev > 0.03 and dist <= 0.005 and np.linalg.norm(d.lin_vel) < 0.01:
        LATCH.clear()  # a teleport back to the start is a trial reset
    LATCH.prev = dist
    lifted = top_in_channel(d, CHANNEL_DEPTH) and _lowest(d) > ON_TABLE
    LATCH.run = LATCH.run + 1 if lifted else 0
    if LATCH.run >= AIR_STEPS:
        LATCH.captured = True
    return d


def score(objects):
    """Flags are kept over the whole run; the end state decides the rest.
    1.0: hung (both shoes straddle the rib for 8 mm or more, the top 1.5 cm or more above the rear lip's bottom and between
    the lips, tilted at most 10 degrees) and shut (the +y edge within 3 mm of the jamb). 0.5: hung, not shut.
    0.25: the top was in the channel 1 cm or more with no part of the door on the table for 1 s, at any time. Else 0.
    A door whose lowest point is more than 3 cm above the table (still held) scores 0."""
    d = _update(objects)
    if _lowest(d) > AIRBORNE_Z:
        return 0.0
    if hung(d):
        return 1.0 if shut(d) else 0.5
    return 0.25 if LATCH.captured else 0.0


def success(objects):
    d = objects["door"]
    return score(objects) == 1.0 and d.resting()


# The hand never goes where a finger could be pinched between the door's edge and the jamb: the last 4 cm before the jamb,
# below the head track and the carry height.
ARM_KEEP_OUT = [
    {"name": "jamb pinch zone", "x": (0.26, 0.36), "y": (0.17, 0.21), "z_top": TOP + 0.25},
]

# -- the scripted demo (left arm, closed loop) ---------------------------------------------------------------------------
demo = placeholder_demo
GRIP_LOCAL = np.array([HANDLE_LOCAL[0] - 0.006, HANDLE_Y, 0.07])  # fingertip centre on the handle; pad tips 5 mm short of the face
GRIP_DOWN = 0.35  # rad: the hand points 20 degrees down onto the handle, which the arm reaches more easily
# The door's top-centre (tilt in degrees, z above the table, x) while the top is lifted into the channel and the door is
# turned upright about it: each pose clears the lips, the head plate and the rib by 4 mm or more.
SCHEDULE = [(46, .130, .316), (40, .136, .314), (34, .142, .314), (28, .148, .314), (22, .154, .314), (16, .159, .314),
            (13, .166, .314), (10, .172, .314), (5, .173, .311), (0, .173, .310)]


def Ry(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _door_roll(state) -> float:
    """Rotation about the x axis (the door's +y end up is positive)."""
    return float(np.arcsin(np.clip(_rot(state.quat_xyzw)[2, 1], -1.0, 1.0)))


def _door_yaw(state) -> float:
    r = _rot(state.quat_xyzw)
    return float(np.arctan2(r[1, 0], r[0, 0]))


def _door_tilt(state) -> float:
    r = _rot(state.quat_xyzw)
    return float(np.arctan2(r[0, 2], r[2, 2]))  # rotation about +y, top toward +x positive


def pose_for_top(deg, ztop, xtop):
    """Door pose (x, z above the table, tilt rad) of the panel's bottom middle for a given top-centre."""
    t = np.radians(deg)
    return xtop - PANEL_H * np.sin(t), ztop - PANEL_H * np.cos(t), t


def demo_script(ctx):
    """Left arm: grip the handle, lift the door, tilt it 46 degrees top toward the track, carry it forward so its top
    passes under the front lip, lift the top up into the channel to 1.5 cm past the drop while turning the door upright
    about its top (so the foot swings in over the rib), lower it onto the rib, run it along to the jamb by the handle,
    let go, and park clear. Door poses are planned as (x, z, tilt) of the panel's bottom middle; the hand follows them
    through the grip offset and is corrected from the door's live pose."""
    arm = ctx.left
    O, C = FINGER_OPEN, FINGER_CLOSED
    o0 = np.asarray(ctx.obj("door").origin, float)
    y0 = o0[1]
    rel = grip_rot(yaw=0.0, pitch=np.pi / 2 - GRIP_DOWN)

    def tcp_for(x, z, th, y=None, yaw=0.0, roll=0.0):
        o = np.array([x, y0 if y is None else y, z])
        r = Rz(yaw) @ Ry(th) @ Rx(roll)
        return o + r @ GRIP_LOCAL, r @ rel

    grasp, _ = tcp_for(o0[0], o0[2], 0.0)
    yield from arm.move(np.array([0.18, 0.25, TOP + 0.30]), 2.5, O, rot=rel)  # up and back, clear of the jamb's corner
    yield from arm.move(grasp + np.array([-0.02, 0.0, 0.14]), 2.5, O, rot=rel)  # well above the door's top
    yield from arm.move(grasp + np.array([-0.02, 0.0, 0.0]), 2.5, O, rot=rel)  # down in front of the handle
    yield from arm.move(grasp, 1.2, O, rot=rel)
    yield from arm.trim_to_measured(grasp, tol=0.0015, max_comp=0.03, step_mps=0.03, max_duration=0.8)
    yield from arm.grip(C, 0.8)
    yield from arm.hold(0.4)

    corr = np.zeros(3)
    corr_th = 0.0
    corr_yaw = 0.0
    corr_roll = 0.0
    plan = [o0[0], o0[2], 0.0, y0]  # the planned door pose now: x, z, tilt, y
    state = {"live": ctx.states() is not ctx.states()}  # an offline replay hands back the same start poses every time

    def go(x, z, th, dur, y=None):
        """Move the planned door pose to (x, z, tilt[, y]) over dur seconds. The hand is placed through the grip offset;
        in a live run a small feedback from the door's measured pose is added every step."""
        nonlocal corr, corr_th, corr_yaw, corr_roll
        a = np.array(plan, float)
        b = np.array([x, z, th, plan[3] if y is None else y], float)
        n = max(1, round(dur / ctx.dt))
        for i in range(1, n + 1):
            p = a + closedloop.ik.smoothstep(i / n) * (b - a)
            tcp, rot = tcp_for(p[0], p[1], p[2] + corr_th, p[3], corr_yaw, corr_roll)
            arm._set_cartesian(tcp + corr, rot, C)
            if state["live"]:
                st = ctx.obj("door")
                err = np.array([p[0], p[3], p[1]]) - np.asarray(st.origin, float)
                corr = np.clip(corr + np.array([0.25, 0.15, 0.12]) * np.clip(err, -0.01, 0.01), -0.03, 0.03)
                corr_th = float(np.clip(corr_th + 0.12 * np.clip(p[2] - _door_tilt(st), -0.1, 0.1), -0.25, 0.25))
                corr_yaw = float(np.clip(corr_yaw + 0.15 * np.clip(-_door_yaw(st), -0.1, 0.1), -0.3, 0.3))
                corr_roll = float(np.clip(corr_roll + 0.15 * np.clip(-_door_roll(st), -0.1, 0.1), -0.3, 0.3))
            yield
        plan[:] = b

    def settle(dur=0.8):
        yield from go(plan[0], plan[1], plan[2], dur)

    x0, z0 = o0[0], o0[2]
    entry = np.radians(SCHEDULE[0][0])
    # lift 8 mm
    yield from go(x0, z0 + 0.008, 0.0, 1.2)
    # tilt on the spot (a little back, so the top stays short of the lip), then carry forward
    yield from go(x0 - 0.006, TOP + 0.022, entry, 3.5)
    yield from settle()
    ex, ez, et = pose_for_top(SCHEDULE[0][0], TOP + SCHEDULE[0][1], SCHEDULE[0][2])
    yield from go(ex, TOP + 0.022, entry, 3.0)  # the top passes under the front lip
    yield from settle()
    yield from go(ex, ez, et, 1.5)  # up into the channel mouth
    yield from settle()
    # lift the top into the channel while turning the door upright about it: the foot swings in over the rib
    for deg, zt, xt in SCHEDULE[1:]:
        px, pz, pt = pose_for_top(deg, TOP + zt, xt)
        yield from go(px, pz, pt, 1.6)
    yield from settle()
    # lower onto the rib
    rest_z = TOP + 0.0145
    yield from go(px, rest_z, 0.0, 4.0)
    yield from settle()
    yield from arm.hold(0.6)

    # run it shut by the handle at 2 cm/s, on into the jamb, and hold it there for a second
    shut_y = JAMB_Y - PANEL_L / 2
    cur_y = y0
    while cur_y < shut_y + 0.012:
        cur_y += 0.01
        yield from go(px, rest_z, 0.0, 0.5, cur_y)
    yield from arm.hold(1.0)

    # let go with the fingers opened only 1.6 cm each (the handle is 1.8 cm wide, and the jamb is close), back off, then open
    # fully and park clear above the pinch zone
    half = FINGER_OPEN * 0.35
    yield from arm.grip(half, 0.8)
    yield from arm.hold(0.6)
    yield from arm.move(arm.pos + np.array([-0.05, 0.0, 0.0]), 1.5, half, rot=arm.rot.copy())
    yield from arm.grip(O, 0.6)
    yield from arm.move(np.array([arm.pos[0], arm.pos[1], TOP + 0.30]), 3.0, O, rot=arm.rot.copy())  # above the pinch zone
    yield from arm.move(np.array([0.25, 0.25, TOP + 0.30]), 2.0, O, rot=arm.rot.copy())
    yield from arm.home(3.0)
    yield from ctx.wait(0.5)
