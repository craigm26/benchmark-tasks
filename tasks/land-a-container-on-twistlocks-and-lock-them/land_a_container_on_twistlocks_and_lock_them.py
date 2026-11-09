"""On the station table: a dark blue deck plate lies between the arms with two grey twistlock cones on it, 11 cm apart
along y. Each cone has a yellow head and, well out beside the deck's side, an orange square handle tab on a long arm that turns
the head a quarter turn. A red container (a white lifting lug on its top) stands on the table in front of the right arm,
its long side along x. It has a hole in its floor for each cone. The container goes down over both cones onto the deck,
and then both locks are turned shut with their handles. The container and the handles are loose; the deck and the cones
are fixed.

Requires -E object_jitter=0: the container's start pose decides the scored positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=land_a_container_on_twistlocks_and_lock_them.py -E object_jitter=0 \\
           --instruction "Land the red container on the deck twistlocks and lock it down." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import closedloop  # noqa: E402, F401
from belimits import PeakTracker, tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, on_reset  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: let the release show before the episode ends

BLUE, WHITE, GREY = (0.08, 0.14, 0.42), (0.96, 0.96, 0.94), (0.52, 0.53, 0.56)
YELLOW, ORANGE, RED = (0.98, 0.84, 0.04), (0.98, 0.42, 0.02), (0.82, 0.05, 0.06)

# -- the deck and its two twistlocks -----------------------------------------------------------------------------------------
# The deck Mech sits at the table origin, so shape x and y are station coordinates and z is height above the table top.
CX, CY = 0.25, 0.0  # the container's landed centre (the deck is centred on it)
LOCK_Y = 0.055  # cones at y = +-0.055 (11 cm apart)
DECK_X, DECK_HY = (0.16, 0.34), 0.145
FLANGE_TOP = 0.02  # the container lands on the flanges
HEAD_Z0, HEAD_Z1 = 0.032, 0.040
HEAD_X, HEAD_Y = 0.010, 0.022  # at rest (unlocked) the head's long side is along y, through the hole
ARM_LEN, ARM_R, ARM_MID = 0.092, 0.099, 0.054  # arm length; handle centre 9.9 cm from the post, at 45 degrees off the y axis (the hand is 6.8 cm wide, so the handle sits well out from the container's ends)
HANDLE_W, HANDLE_Z0, HANDLE_H = 0.020, 0.011, 0.031  # orange square tab, 2.0 x 2.0 cm and 3.1 cm tall
LOCKED = 1.57  # rad: a quarter turn
LOCK_TOL = 0.15  # rad: tape equivalent, the handle within about 1.5 cm of its locked mark
JOINT_DAMPING = 0.2  # N m s/rad: light enough for a hand to turn it, heavy enough to stay put


def _handle_xy(side: int, angle: float):
    """Handle centre when the lock on side ``side`` (+1 left, -1 right) is turned ``angle`` (rad, positive quarter turn)."""
    a0 = math.radians(135.0) if side > 0 else math.radians(225.0)
    a = a0 - side * angle
    return CX + ARM_R * math.cos(a), side * LOCK_Y + ARM_R * math.sin(a)


def _lock(side: int) -> Joint:
    a0 = math.radians(135.0) if side > 0 else math.radians(225.0)
    half = a0 / 2
    arm_tilt = (math.cos(half), 0.0, 0.0, math.sin(half))  # arm box turned to point at the handle
    hx, hy = ARM_R * math.cos(a0), ARM_R * math.sin(a0)  # handle centre relative to the post
    return Joint(
        "lock_left" if side > 0 else "lock_right",
        "hinge",
        axis="z",
        at=(CX, side * LOCK_Y, 0.0),
        low=-1.6 if side > 0 else 0.0,  # the left lock turns clockwise from above, the right one anticlockwise
        high=0.0 if side > 0 else 1.6,
        spring=0.0,
        damping=JOINT_DAMPING,
        mass=0.03,
        friction=1.2,
        shapes=[
            box((0, 0, (HEAD_Z0 + HEAD_Z1) / 2), (HEAD_X, HEAD_Y, HEAD_Z1 - HEAD_Z0), YELLOW),
            box((hx / ARM_R * ARM_MID, hy / ARM_R * ARM_MID, 0.01325), (ARM_LEN, 0.008, 0.0045), GREY, tilt=arm_tilt),  # arm, clear of the post and under the flange plate
            box((hx, hy, HANDLE_Z0 + HANDLE_H / 2), (HANDLE_W, HANDLE_W, HANDLE_H), ORANGE),
        ],
    )


def _deck_shapes():
    s = [box(((DECK_X[0] + DECK_X[1]) / 2, 0, 0.005), (DECK_X[1] - DECK_X[0], 2 * DECK_HY, 0.01), BLUE)]
    for sy in (-1, 1):
        s.append(box((CX, sy * LOCK_Y, 0.01825), (0.030, 0.030, 0.0035), GREY))  # flange plate, 3.0 x 3.0 x 0.35 cm, top at 0.02; the lock arm swings under it
        s.append(box((CX, sy * LOCK_Y, 0.021), (0.010, 0.010, 0.022), GREY))  # post, z 0.01..0.032
    # white corner marks, half under the landed container, and one under each handle's locked position
    for sx in (-1, 1):
        for sy in (-1, 1):
            s.append(box((CX + sx * 0.035, sy * 0.075, 0.01025), (0.010, 0.010, 0.0005), WHITE))
    for side in (-1, 1):
        hx, hy = _handle_xy(side, LOCKED)
        s.append(box((hx, hy, 0.01025), (0.028, 0.028, 0.0005), WHITE))
    return s


DECK = Mech("deck", pos=(0.0, 0.0, TOP), mass=None, friction=0.6, shapes=_deck_shapes(), joints=(_lock(1), _lock(-1)))

# -- the container ---------------------------------------------------------------------------------------------------------
# Frame origin: the middle of the underside. x is the short side (7 cm), y the long side (15 cm), z up. A 0.8 cm floor with a
# 1.6 (x) x 2.8 (y) hole centred 5.5 cm each side of the middle, 0.6 cm walls, a closed top, and a white lug on the top.
CW, CL, CH = 0.07, 0.15, 0.08
FLOOR, WALL = 0.008, 0.006
HOLE_X, HOLE_Y = 0.016, 0.028
LUG_X, LUG_Y, LUG_H = 0.018, 0.040, 0.030
LUG_MID = CH + LUG_H / 2


def _container_shapes():
    s = []
    for sy in (-1, 1):  # end walls, full height, and the strips of floor between the end wall and the hole
        s.append(box((0, sy * (CL - WALL) / 2, CH / 2), (CW, WALL, CH), RED))
    s.append(box((0, 0, FLOOR / 2), (CW, 2 * (LOCK_Y - HOLE_Y / 2), FLOOR), RED))  # floor between the holes
    for sy in (-1, 1):
        for sx in (-1, 1):  # floor either side of each hole
            side = (CW - HOLE_X) / 2
            s.append(box((sx * (HOLE_X + side) / 2, sy * LOCK_Y, FLOOR / 2), (side, HOLE_Y, FLOOR), RED))
    for sx in (-1, 1):
        s.append(box((sx * (CW - WALL) / 2, 0, CH / 2), (WALL, CL - 2 * WALL, CH), RED))  # side walls
    s.append(box((0, 0, CH - WALL / 2), (CW, CL, WALL), RED))  # closed top
    s.append(box((0, 0, LUG_MID), (LUG_X, LUG_Y, LUG_H), WHITE))  # lifting lug
    return s


START = (0.24, -0.195)
START_YAW = -math.pi / 2  # long side along x
CONTAINER = Mech(
    "container", pos=(START[0], START[1], TOP + 0.0005), yaw=START_YAW, mass=0.10, friction=1.2, shapes=_container_shapes()
)
OBJECTS = [DECK, CONTAINER]

# -- scoring ---------------------------------------------------------------------------------------------------------------
LANDED_Z = TOP + FLANGE_TOP
LAND_Z_TOL = 0.003  # bottom within 3 mm of the flange tops
LAND_XY_TOL = 0.006
LAND_YAW_TOL = math.radians(3.0)
LAND_TILT = 2.0  # degrees
LIFT = 0.03  # the container's bottom has been at least 3 cm above the table
SETDOWN_TILT = 15.0
HOLD_CLEAR = 0.01  # a part whose underside is more than 1 cm above every support counts as not placed
SUPPORTS = (TOP, TOP + FLANGE_TOP, TOP + HEAD_Z1)  # table, flange tops, head tops


def _yaw(s) -> float:
    r = s.rotation()
    return float(math.atan2(r[1, 0], r[0, 0]))


def _bottom(s) -> float:
    return float(s.low[2])


def _airborne(s) -> bool:
    """Underside more than 1 cm above every support: still held, or in flight."""
    return all(_bottom(s) - z > HOLD_CLEAR for z in SUPPORTS)


def _landed(s) -> bool:
    """On both flange tops: bottom within 3 mm of them, level, centred within 6 mm, square within 3 degrees. The cones
    can only be inside the holes then: a head in the way holds the floor 2 cm higher."""
    o = np.asarray(s.origin, float)
    yaw = (_yaw(s) + math.pi / 2) % math.pi - math.pi / 2  # the container looks the same turned half a turn
    return bool(
        abs(_bottom(s) - LANDED_Z) <= LAND_Z_TOL
        and tilt_deg(s) <= LAND_TILT
        and abs(o[0] - CX) <= LAND_XY_TOL
        and abs(o[1] - CY) <= LAND_XY_TOL
        and abs(yaw) <= LAND_YAW_TOL
    )


def _set_down(s) -> bool:
    """Resting upright with both cones under its footprint, not held (e.g. sitting on the heads)."""
    if _airborne(s) or tilt_deg(s) > SETDOWN_TILT:
        return False
    for side in (-1, 1):
        lx, ly, _ = s.to_local((CX, side * LOCK_Y, TOP))
        if abs(lx) > CW / 2 or abs(ly) > CL / 2:
            return False
    return True


def _locks(deck) -> int:
    return sum(abs(abs(deck.joints[n]) - LOCKED) <= LOCK_TOL for n in ("lock_left", "lock_right"))


def _bbox_centre(spec):
    from mechkit import _local_corners

    c = _local_corners(spec, {})
    half = spec.yaw / 2
    rot = np.array([[math.cos(spec.yaw), -math.sin(spec.yaw), 0], [math.sin(spec.yaw), math.cos(spec.yaw), 0], [0, 0, 1]])
    c = c @ rot.T
    return tuple(np.asarray(spec.pos, float) + (c.max(0) + c.min(0)) / 2)


def _new_tracker():
    return PeakTracker({"container": _bbox_centre(CONTAINER)}, home_tol=0.01)


TRACK = _new_tracker()


@on_reset
def _reset_track(env=None):
    global TRACK
    TRACK = _new_tracker()


def _lifted(objects) -> bool:
    c = objects["container"]
    peaks = TRACK.update(objects, {"lift": float(_bottom(c) - TOP >= LIFT)})
    return peaks["lift"] >= 1.0


def score(objects):
    """Kept over the run: the container's bottom has been at least 3 cm above the table (pushing cannot score).
    1.0 container landed (bottom within 3 mm of both flange tops, level within 2 degrees, centred within 6 mm, square
    within 3 degrees), both locks turned to within 0.15 rad of a quarter turn, container still. 0.5 landed with one or no
    lock turned. 0.25 container set down upright with both cones under its footprint but not landed (on the heads, or
    off by more than 3 mm). A container whose underside is more than 1 cm above every support counts as not placed. Else 0."""
    c, deck = objects["container"], objects["deck"]
    if not _lifted(objects) or _airborne(c):
        return 0.0
    if _landed(c):
        return 1.0 if _locks(deck) == 2 and c.resting() else 0.5
    return 0.25 if _set_down(c) else 0.0


def success(objects):
    """Landed, both locks turned, container still, and it has been lifted."""
    c, deck = objects["container"], objects["deck"]
    return bool(_lifted(objects) and not _airborne(c) and _landed(c) and _locks(deck) == 2 and c.resting())


# The robot keeps its hands out from over the landing footprint below the container's own top (the scorer sees only the
# objects). The handles lie 1.5 cm outside this footprint and are worked from outside it.
FOOT_X, FOOT_Y = (CX - CW / 2, CX + CW / 2), (CY - CL / 2, CY + CL / 2)
ARM_KEEP_OUT = [{"name": "landing footprint", "x": FOOT_X, "y": FOOT_Y, "z_top": TOP + 0.09}]

# -- the scripted demo -----------------------------------------------------------------------------------------------------
demo = placeholder_demo

DT = 0.05
O, C = FINGER_OPEN, FINGER_CLOSED
OPEN_TAB = -0.020  # fingers open just 2 cm each side of the 2 cm tab: the hand's pads stay well clear of the container's end
LUG_TCP_DZ = 0.100  # TCP height above the container's underside when gripping the lug (pads span the lug)
HANDLE_TCP_Z = TOP + 0.036  # pads span 2.3 to 4.9 cm up the 3.1 cm handle's height range
HAND_YAW = math.pi / 2  # fingers close along x on the handles, away from the container's ends
CARRY_BOTTOM = 0.07  # carry the underside 7 cm up: 3 cm above the heads
SEAT_START = 0.05  # slow descent starts with the underside 1 cm above the heads' tops


def _wrap(a):
    return (a + math.pi / 2) % math.pi - math.pi / 2


def _chase(arm, get_err, steps, feed=(0.0, 0.0, 0.0), rot_fn=None, finger=None, gain=0.25, step_cap=0.0006, max_comp=0.012, stop=None, floor=None):
    """Closed-loop correction: the planned TCP moves on by ``feed`` (m/s) each step and an offset grows by ``gain`` times
    the live error ``get_err()`` (desired minus measured, m), so a constant slip or sag is trimmed out."""
    anchor, comp = arm.pos.copy(), np.zeros(3)
    rot0 = arm.rot.copy()
    feed = np.asarray(feed, float)
    for _ in range(steps):
        if stop is not None and stop():
            break
        comp = np.clip(comp + np.clip(gain * np.asarray(get_err(), float), -step_cap, step_cap), -max_comp, max_comp)
        anchor = anchor + feed * DT
        if floor is not None:
            anchor = np.maximum(anchor, floor)
        arm._set_cartesian(anchor + comp, rot0 if rot_fn is None else rot_fn(), arm.finger if finger is None else finger)
        yield


def _work_lock(ctx, arm, side):
    """Take the lock's square handle tab between the fingers (they close along y at first, away from nothing: the container's
    end is 5 cm off), turn it a quarter turn along its arc with the hand turning with it so the pads stay flat on the same two
    faces, following the handle's measured angle, then let go. The hand ends with its fingers closing along x."""
    name = "lock_left" if side > 0 else "lock_right"
    h0 = _handle_xy(side, 0.0)
    above = np.array([h0[0], h0[1], TOP + 0.13])
    yield from arm.move(above, duration=4.0, finger=OPEN_TAB, yaw=0.0)
    yield from arm.trim_to_measured(above, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = arm.pos - above
    grasp = np.array([h0[0], h0[1], HANDLE_TCP_Z])
    yield from arm.move(grasp + c, duration=3.0, finger=OPEN_TAB, yaw=0.0)
    yield from arm.trim_to_measured(grasp, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from arm.grip(C, duration=0.9)
    yield from arm.hold(0.4)
    off = arm.pos - grasp  # where the hand sits on the handle (sag included)
    live = ctx.step_count > 0  # offline dry runs never move the objects: follow the schedule there
    cmd, rate, lead, goal = 0.0, 0.20, 0.04, 1.60  # commanded handle angle (rad), its rate, how far it may lead the handle
    sign = -1.0 if side > 0 else 1.0  # the left lock turns clockwise, the right one anticlockwise
    for _ in range(int(20.0 / DT)):
        turned = abs(ctx.obj("deck").joints[name]) if live else cmd
        if turned >= LOCKED - 0.02:
            break
        cmd = min(cmd + rate * DT, goal, max(cmd, turned + lead))  # never back off
        x, y = _handle_xy(side, cmd)
        arm._set_cartesian(np.array([x, y, HANDLE_TCP_Z]) + off, grip_rot(yaw=sign * cmd), C)
        yield
    yield from arm.hold(0.6)
    yield from arm.grip(O, duration=1.0)
    yield from arm.hold(0.5)
    out = arm.pos + np.array([0.0, side * 0.03, 0.0])  # step away from the container's end, then straight up
    yield from arm.move(out, duration=1.5, finger=O)
    yield from arm.move(np.array([out[0], out[1], TOP + 0.13]), duration=2.0, finger=O)


def demo_script(ctx):
    """Right arm carries the container by its lug, lands it on both cones (steering on the live container pose) and lets go.
    Then the left arm turns the +y lock and the right arm the -y lock, each by its square handle tab from outside the container."""
    left, right = ctx.left, ctx.right

    def origin():
        return np.asarray(ctx.obj("container").origin, float)

    def live():
        return ctx.step_count > 0  # offline dry runs never move the objects: no steering there

    def yaw_err():
        return _wrap(_yaw(ctx.obj("container"))) if live() else 0.0

    # 1. take the lug
    o0 = origin()
    above = np.array([o0[0], o0[1], o0[2] + LUG_TCP_DZ + 0.09])
    yield from right.move(above, duration=4.0, finger=O, yaw=0.0)
    yield from right.trim_to_measured(above, tol=0.0015, max_comp=0.03, step_mps=0.035, max_duration=0.7)
    c = right.pos - above
    grasp = np.array([o0[0], o0[1], o0[2] + LUG_TCP_DZ])
    yield from right.move(grasp + c, duration=3.0, finger=O, yaw=0.0)
    yield from right.trim_to_measured(grasp, tol=0.0015, max_comp=0.03, step_mps=0.025, max_duration=0.8)
    yield from right.grip(C, duration=0.9)
    yield from right.hold(0.4)
    # 2. lift, carry over the cones turning the container a quarter turn, and bring its underside to 7 cm
    lift = right.pos + np.array([0.0, 0.0, CARRY_BOTTOM])
    yield from right.move(lift, duration=2.5, finger=C, yaw=0.0)
    yield from right.hold(0.3)
    carry = np.array([CX, CY, TOP + CARRY_BOTTOM + LUG_TCP_DZ]) 
    yield from right.move(carry, duration=8.0, finger=C, yaw=HAND_YAW)
    yield from right.hold(0.5)
    # 3. steer the container's own centre and heading onto the cones
    state = {"yaw": HAND_YAW}

    def rot_fn():
        state["yaw"] = float(np.clip(state["yaw"] - 0.2 * yaw_err(), HAND_YAW - 0.15, HAND_YAW + 0.15))
        return grip_rot(yaw=state["yaw"])

    def err_xy():
        o = origin()
        return np.array([CX - o[0], CY - o[1], 0.0]) if live() else np.zeros(3)

    yield from _chase(right, err_xy, 100, rot_fn=rot_fn, gain=0.3, max_comp=0.012)
    # 4. lower: quickly to 1 cm over the heads, then 1 cm/s while steering, until the floor is on the flanges
    yield from _chase(right, err_xy, int(2.5 / DT), rot_fn=rot_fn, feed=(0.0, 0.0, -(CARRY_BOTTOM - SEAT_START) / 2.5), gain=0.3, max_comp=0.012,
                      stop=lambda: origin()[2] <= TOP + SEAT_START + 0.001)
    yield from _chase(right, err_xy, 60, rot_fn=rot_fn, gain=0.3, max_comp=0.012)
    z_floor = TOP + FLANGE_TOP - 0.004 + LUG_TCP_DZ
    yield from _chase(
        right, err_xy, int(9.0 / DT), rot_fn=rot_fn, feed=(0.0, 0.0, -0.01), gain=0.3, max_comp=0.012,
        stop=lambda: origin()[2] <= LANDED_Z + 0.0008,
        floor=np.array([-9.0, -9.0, z_floor - 0.0]),
    )
    yield from right.hold(0.8)
    yield from right.grip(O, duration=1.0)
    yield from right.hold(0.5)
    yield from right.move(np.array([right.pos[0], right.pos[1], TOP + 0.17]), duration=2.5, finger=O)
    # 5. left arm turns the +y lock while the right arm goes home; then the right arm turns the -y lock
    def left_job():
        yield from _work_lock(ctx, left, +1)
        yield from left.home(duration=5.0)

    def right_job():
        yield from right.home(duration=5.0)

    yield from ctx.both(left_job(), right_job())

    def right_job2():
        yield from _work_lock(ctx, right, -1)
        yield from right.home(duration=5.0)

    yield from right_job2()
    yield from ctx.wait(1.0)
