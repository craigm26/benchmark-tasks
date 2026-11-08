"""On the station table: a dark-grey coupler body bolted to a grey tank end wall, in front of the arms a little left of
centre, with a red cam arm hinged on each side of it (both swung out sideways). A blue hose adapter (a 4 x 4 x 9 cm
riser with a silver collar and a square silver shank pointing at the coupler) stands on a white pad in front of the right
arm. The adapter goes into the coupler's square bore until its collar meets the mouth; then each cam arm can swing
back along the body, its lobe passing through a side slot into a groove in the shank. Closed arms lock the adapter in.
A shank more than about 5 mm short stops the lobes at the shank's shoulder; arms closed first block the bore.

Requires -E object_jitter=0: the start is recognised by the adapter being on its pad.

Run it:  inspect-robots run --embodiment yam_arena -E task=couple_a_camlock_hose_adapter.py -E object_jitter=0 \\
           --instruction "Couple the blue hose adapter to the tank's coupler and close both red cam arms." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import tilt_deg
from closedloop import grip_rot, placeholder_demo
from mechkit import Joint, Mech, box, on_reset
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SUCCESS_HOLD_S = 8.0  # the demo grips and releases: keep the run going so the video shows the release and the arms clear
TOP = TABLE_TOP[SCENE]

DARK, GREY, BLACK = (0.20, 0.20, 0.24), (0.55, 0.57, 0.62), (0.04, 0.04, 0.05)
SILVER, BLUE, RED_C, WHITE = (0.80, 0.82, 0.86), (0.10, 0.30, 0.90), (0.88, 0.08, 0.08), (0.94, 0.94, 0.92)

# -- the coupler: fixed. Mech origin at world (0.30, -0.02); local x forward, y left, z up from the table top ----------------
CX, CY = 0.30, -0.02
MOUTH_X = 0.27  # world x of the coupler's mouth
AXIS_Z = TOP + 0.075  # bore axis height
BORE = 0.038  # square bore, 3.8 cm: the 3.0 cm shank has 4 mm per side
HINGE_LX, HINGE_DY = 0.015, 0.045  # hinge x (local) and its distance to the bore axis, either side
BAR_L, LOBE_L = 0.07, 0.034
WALL_Y, WALL_T = 0.025, 0.012  # side walls: centre offset from the bore axis and thickness (inner face 1.9 cm out)
SLOT_X = (-0.023, 0.025)  # side slots through the walls (local x), 2 cm tall
CLOSED = math.pi / 2
CLOSED_TOL = 0.1  # rad: 7 mm at the grip block


def _wall_pieces(s: float):
    """One side wall of the body (y = s * WALL_Y) with its slot cut out."""
    y = s * WALL_Y
    return [
        box((0.0065, y, 0.052), (0.073, WALL_T, 0.026), DARK),  # below the slot
        box((0.0065, y, 0.098), (0.073, WALL_T, 0.026), DARK),  # above the slot
        box((-0.0265, y, 0.075), (0.007, WALL_T, 0.020), DARK),  # in front of the slot
        box((0.034, y, 0.075), (0.018, WALL_T, 0.020), DARK),  # behind the slot
    ]


def _cam_arm(s: float) -> Joint:
    """Cam arm on side s (+1 left of the bore, -1 right). At rest the bar points out sideways with the grip block at its tip
    and the lobe points to -x; swung 90 degrees toward -x the bar lies along the body and the lobe points at the bore."""
    return Joint(
        "arm_l" if s > 0 else "arm_r",
        "hinge",
        axis="z",
        at=(HINGE_LX, s * HINGE_DY, 0.075),
        low=0.0 if s > 0 else -1.65,
        high=1.65 if s > 0 else 0.0,
        damping=0.5,
        mass=0.03,
        shapes=[
            box((0.0, s * BAR_L / 2, 0.0), (0.014, BAR_L, 0.014), RED_C),
            box((0.0, s * BAR_L, 0.0), (0.020, 0.020, 0.030), RED_C),  # grip block
            box((-LOBE_L / 2, 0.0, 0.0), (LOBE_L, 0.014, 0.014), RED_C),  # cam lobe
        ],
    )


COUPLER = Mech(
    "coupler",
    pos=(CX, CY, TOP),
    mass=None,
    friction=0.5,
    shapes=[
        box((0.06, 0.0, 0.07), (0.02, 0.08, 0.14), GREY),  # tank end wall, x 0.35..0.37
        box((0.01, 0.0, 0.1025), (0.08, 0.066, 0.017), DARK),  # body top
        box((0.01, 0.0, 0.0475), (0.08, 0.066, 0.017), DARK),  # body bottom
        box((0.0465, 0.0, 0.075), (0.007, 0.066, 0.038), DARK),  # bore end
        *_wall_pieces(1.0),
        *_wall_pieces(-1.0),
    ],
    joints=[_cam_arm(1.0), _cam_arm(-1.0)],
)

# -- the adapter: free. Mech origin at the riser's base centre; shank points +x --------------------------------------------
ADAPTER_START = (0.18, -0.20, TOP + 0.003)
PAD = Box("pad", pos=(0.18, -0.20, TOP + 0.0015), size=(0.08, 0.08, 0.003), color=WHITE)
RISER_H, SHANK_Z = 0.09, 0.02
COLLAR_X = 0.03  # local x of the collar's front face
TIP_X = 0.10  # local x of the shank's tip
SHOULDER_X = 0.087  # local x where the narrow neck meets the full-width head
ADAPTER = Mech(
    "adapter",
    pos=ADAPTER_START,
    mass=0.10,
    shapes=[
        box((0.0, 0.0, 0.045), (0.04, 0.04, RISER_H), BLUE),  # riser: the part to grip
        box((0.025, 0.0, SHANK_Z), (0.01, 0.05, 0.04), SILVER),  # collar
        box((0.057, 0.0, SHANK_Z), (SHOULDER_X - COLLAR_X, 0.012, 0.03), SILVER),  # neck, narrowed to 1.2 cm in y
        box((0.0905, 0.0, SHANK_Z), (0.007, 0.03, 0.03), SILVER),  # head, full width
        box((0.097, 0.0, SHANK_Z), (TIP_X - 0.094, 0.022, 0.022), SILVER),  # nose: 2.2 cm square lead-in on the tip
        *[box((x, 0.0, SHANK_Z), (0.02, 0.03, 0.03), BLACK) for x in (-0.03, -0.05, -0.07)],  # hose tail behind the riser
    ],
)
OBJECTS = [PAD, COUPLER, ADAPTER]

# -- scoring ---------------------------------------------------------------------------------------------------------------
HOME_G = 0.005  # m: collar front within this of the mouth (the 3 mm lobe clearance plus a few degrees of riser lean)
ALIGN = 0.010  # m: shank tip within this of the bore axis (it has 3 mm per side)
DEEP = 0.03  # m: shank tip this far past the mouth
DEEP_ALIGN = 0.012
MAX_TILT = 15.0  # deg: riser leaning at most 2.4 cm over its 9 cm
STILL = 0.01  # m/s
HOLD_STEPS = 20  # 1 s at 20 Hz
SUPPORT_TOL = 0.01  # a bottom this far above the pad or table counts as not placed

_last = None
_deep_run = 0
_full_run = 0
_deep = False


def _clear() -> None:
    global _deep_run, _full_run, _deep
    _deep_run, _full_run, _deep = 0, 0, False


on_reset(lambda env: _clear())


def _geometry(a):
    """Collar-front gap G (positive while the collar is short of the mouth), the tip depth, and the shank tip's offset from the bore axis in y and z, from the adapter's real pose."""
    front = a.to_world((COLLAR_X, 0.0, SHANK_Z))
    tip = a.to_world((TIP_X, 0.0, SHANK_Z))
    return float(MOUTH_X - front[0]), float(tip[0] - MOUTH_X), float(tip[1] - CY), float(tip[2] - AXIS_Z)


def _engaged(a, align: float, depth: float) -> bool:
    _, depth_now, dy, dz = _geometry(a)
    return bool(depth_now >= depth and abs(dy) <= align and abs(dz) <= align)


def _home(a) -> bool:
    g, _, _, _ = _geometry(a)
    return bool(
        -HOME_G <= g <= HOME_G
        and _engaged(a, ALIGN, 0.06)
        and tilt_deg(a) <= MAX_TILT
        and float(np.linalg.norm(a.lin_vel)) <= STILL
    )


def _arms_closed(objects) -> int:
    j = objects["coupler"].joints
    return int(j["arm_l"] >= CLOSED - CLOSED_TOL) + int(j["arm_r"] <= -(CLOSED - CLOSED_TOL))


def _at_start(objects) -> bool:
    a = objects["adapter"]
    return bool(
        np.linalg.norm(np.asarray(a.origin, float) - np.asarray(ADAPTER_START, float)) <= 0.004
        and float(np.linalg.norm(a.lin_vel)) <= STILL
    )


def _supported(a) -> bool:
    """On its pad or the table, or in the coupler's bore. Held up in the air outside the bore counts as not placed."""
    b = float(a.low[2])
    on_table = b - TOP <= SUPPORT_TOL + 0.003
    return bool(on_table or _engaged(a, DEEP_ALIGN, DEEP))


def _update(objects) -> None:
    """Called first by score() and success(); both get the same dict each control step, so it runs once a step."""
    global _last, _deep_run, _full_run, _deep
    if objects is _last:
        return
    _last = objects
    if _at_start(objects):
        _clear()
    a = objects["adapter"]
    _deep_run = _deep_run + 1 if _engaged(a, DEEP_ALIGN, DEEP) else 0
    if _deep_run >= HOLD_STEPS:
        _deep = True
    _full_run = _full_run + 1 if (_home(a) and _arms_closed(objects) == 2) else 0


def score(objects):
    """0 if the adapter is held in the air outside the bore or has left the table. Home (collar within 5 mm of the mouth,
    shank tip on the bore axis, riser within 15 degrees of upright, still): 0.5, plus 0.25 per closed cam arm, with both
    closed counting 1.0 once it has held for 1 s. Otherwise 0.25 once the shank tip has been 3 cm or more into the bore for
    1 s, else 0."""
    _update(objects)
    a = objects["adapter"]
    if not _supported(a):
        return 0.0
    if _home(a):
        n = _arms_closed(objects)
        if n == 2:
            return 1.0 if _full_run >= HOLD_STEPS else 0.75
        return 0.5 + 0.25 * n
    return 0.25 if _deep else 0.0


def success(objects):
    _update(objects)
    return score(objects) == 1.0


# No area is forbidden to the robot in this task; the tank end wall is a hazard prop, so no arm part may touch the wall or
# come within 4 cm of it below its top edge (checked offline with the URDF; the scorer sees only objects).
ARM_KEEP_OUT = [
    {"name": "tank end wall and 4 cm around it", "x": (0.31, 0.41), "y": (-0.10, 0.06), "z_top": TOP + 0.14},
]

# -- the scripted demo (closed loop) -----------------------------------------------------------------------------------------
demo = placeholder_demo

GRASP_UP = 0.072  # TCP height above the riser base when gripping its upper half
INSERT_V = 0.012  # m/s while the shank is going in
PUSH_Z = TOP + 0.088  # closed-finger pusher: its tips (1.3 cm lower) sit on the grip block
PUSH_OFF = 0.022  # pusher centre to the block face (finger half-width plus the block half)
BLOCK_R = BAR_L  # hinge to the grip block's centre


def _mock(ctx) -> bool:
    """dry_run hands the script one frozen dict; the simulator builds a new one every call."""
    return ctx.states() is ctx.states()


def _set(arm, pos, rot, finger) -> None:
    arm._set_cartesian(np.asarray(pos, float), rot, finger)


_ANG = {"yaw": 0.0, "pitch": 0.0}


def _level(ctx, mock, k=0.12):
    """Gripper rotation that keeps the held shank level and square: nudges the gripper's pitch and yaw against the shank's
    measured slope and heading, so a grasp that slips or a sagging wrist does not tilt the tip into the mouth."""
    if not mock:
        ax = ctx.obj("adapter").rotation()[:, 0]
        _ANG["pitch"] = float(np.clip(_ANG["pitch"] - k * ax[2], -0.3, 0.3))
        _ANG["yaw"] = float(np.clip(_ANG["yaw"] - k * ax[1], -0.3, 0.3))
    return grip_rot(yaw=_ANG["yaw"], pitch=_ANG["pitch"])


def _tip(ctx):
    return np.asarray(ctx.obj("adapter").to_world((TIP_X, 0.0, SHANK_Z)), float)


def _carry(ctx, arm, goal, seconds, mock, finger, nominal, gain=0.35):
    """Move the held adapter's origin to ``goal``, feeding back its measured position so wrist sag does not matter.
    ``nominal`` is where the plan has the origin now (used instead of the measurement in the offline replay)."""
    goal = np.asarray(goal, float)
    start = np.asarray(nominal if mock else ctx.obj("adapter").origin, float)
    n = max(1, round(seconds / ctx.dt))
    prev = start.copy()
    for i in range(1, n + 1):
        s = i / n
        s = s * s * (3 - 2 * s)
        want = start + s * (goal - start)
        rot = _level(ctx, mock)
        if mock:
            _set(arm, want + (0.0, 0.0, GRASP_UP), rot, finger)
        else:
            err = want - np.asarray(ctx.obj("adapter").origin, float)
            _set(arm, arm.pos + (want - prev) + gain * err, rot, finger)
        prev = want
        yield


def _align(ctx, arm, mock):
    """Hold the shank tip on the bore axis, level and square, before it goes in (at most 8 s)."""
    if mock:
        return
    for i in range(round(8.0 / ctx.dt)):
        t = _tip(ctx)
        err = np.array([0.0, CY - t[1], AXIS_Z - t[2]])
        ax = ctx.obj("adapter").rotation()[:, 0]
        _set(arm, arm.pos + 0.4 * err, _level(ctx, mock), FINGER_CLOSED)
        if i > 20 and abs(err[1]) < 0.0015 and abs(err[2]) < 0.0015 and abs(ax[2]) < 0.012 and abs(ax[1]) < 0.012:
            break
        yield


def _insert(ctx, arm, mock, nominal):
    """Push the shank in at 1.2 cm/s, steering the measured tip onto the bore axis and holding the shank level; the push
    pauses whenever the tip is more than 3 mm off the axis. Stops when the collar meets the mouth."""
    home_x = MOUTH_X - COLLAR_X  # riser centre x with the collar at the mouth
    o = np.asarray(nominal if mock else ctx.obj("adapter").origin, float)
    steps = round(((home_x + 0.004) - o[0]) / INSERT_V / ctx.dt)
    x_cmd = o[0]
    for _ in range(steps + 80):
        rot = _level(ctx, mock)
        if mock:
            x_cmd = min(x_cmd + INSERT_V * ctx.dt, home_x + 0.004)
            _set(arm, (x_cmd, CY, AXIS_Z - SHANK_Z + GRASP_UP), rot, FINGER_CLOSED)
        else:
            a = ctx.obj("adapter")
            g, _, _, _ = _geometry(a)
            if g <= 0.0005:
                break
            t = _tip(ctx)
            err = np.array([0.0, CY - t[1], AXIS_Z - t[2]])
            fwd = INSERT_V * ctx.dt if (abs(err[1]) < 0.003 and abs(err[2]) < 0.003) else 0.0
            _set(arm, arm.pos + 0.4 * err + (fwd, 0.0, 0.0), rot, FINGER_CLOSED)
        yield


def _arc_point(s: float, psi: float):
    """Pusher centre at swing angle psi (0 = open, pi/2 = closed) for the arm on side s."""
    hx, hy = CX + HINGE_LX, CY + s * HINGE_DY
    u = np.array([-math.sin(psi), s * math.cos(psi)])
    t = np.array([-math.cos(psi), -s * math.sin(psi)])
    p = np.array([hx, hy]) + BLOCK_R * u - PUSH_OFF * t
    return p


def _swing(ctx, arm, s, mock, seconds=4.5):
    """Close one cam arm: the closed fingers push the grip block's outer face along its arc, never more than 0.15 rad ahead
    of where the arm actually is."""
    name = "arm_l" if s > 0 else "arm_r"
    rot = grip_rot(yaw=0.0)
    p0 = _arc_point(s, 0.0)
    yield from arm.move((p0[0], p0[1], PUSH_Z + 0.08), 2.5, FINGER_CLOSED, rot=rot)
    yield from arm.move((p0[0], p0[1], PUSH_Z), 1.5, FINGER_CLOSED, rot=rot)
    n = round(seconds / ctx.dt)
    psi_cmd = 0.0
    for i in range(1, n + 1):
        s_t = i / n
        ramp = CLOSED * (s_t * s_t * (3 - 2 * s_t))
        if mock:
            psi_cmd = ramp
        else:
            meas = abs(float(ctx.obj("coupler").joints[name]))
            psi_cmd = min(ramp, meas + 0.15)
        p = _arc_point(s, psi_cmd)
        _set(arm, (p[0], p[1], PUSH_Z), rot, FINGER_CLOSED)
        yield
    if not mock:  # keep leading the arm by up to 0.15 rad until it is shut (at most 10 s more)
        for _ in range(round(10.0 / ctx.dt)):
            meas = abs(float(ctx.obj("coupler").joints[name]))
            if meas >= CLOSED - 0.03:
                break
            psi_cmd = min(psi_cmd + 0.01, meas + 0.15, CLOSED + 0.15)
            p = _arc_point(s, psi_cmd)
            _set(arm, (p[0], p[1], PUSH_Z), rot, FINGER_CLOSED)
            yield
    yield from ctx.wait(0.6)
    top = arm.pos.copy()
    yield from arm.move((top[0], top[1], top[2] + 0.08), 1.5, FINGER_CLOSED, rot=rot)
    yield from arm.home(3.0)


def demo_script(ctx):
    mock = _mock(ctx)
    _ANG.update(yaw=0.0, pitch=0.0)
    right, left = ctx.right, ctx.left
    rot0 = grip_rot(yaw=0.0)
    # RIGHT: take the adapter by its riser, lift, bring it to the mouth, and push the shank in.
    o = np.asarray(ctx.obj("adapter").origin, float)
    yield from right.move((o[0], o[1], o[2] + GRASP_UP + 0.10), 2.5, FINGER_OPEN, rot=rot0)
    yield from right.move((o[0], o[1], o[2] + GRASP_UP), 1.5, FINGER_OPEN, rot=rot0)
    yield from right.grip(FINGER_CLOSED, 0.9)
    pre = (MOUTH_X - 0.02 - TIP_X, CY, AXIS_Z - SHANK_Z)  # shank tip 2 cm short of the mouth, at the bore height
    legs = [  # (origin goal, seconds): lift, back off, across, down to the bore height
        ((o[0], o[1], TOP + 0.075), 1.5),
        ((pre[0], o[1], TOP + 0.075), 2.5),
        ((pre[0], pre[1], TOP + 0.075), 2.5),
        (pre, 1.5),
    ]
    cur = o
    for goal, secs in legs:
        yield from _carry(ctx, right, goal, secs, mock, FINGER_CLOSED, cur)
        cur = np.asarray(goal, float)
    yield from ctx.wait(0.5)
    yield from _align(ctx, right, mock)
    yield from _insert(ctx, right, mock, cur)
    yield from ctx.wait(0.8)
    # RIGHT lets go, rises and leaves
    yield from right.grip(FINGER_OPEN, 0.9)
    top = right.pos.copy()
    yield from right.move((top[0], top[1], top[2] + 0.07), 1.5, FINGER_OPEN, rot=rot0)
    yield from right.move((top[0] - 0.05, top[1] - 0.12, top[2] + 0.07), 2.0, FINGER_OPEN, rot=rot0)
    yield from right.home(3.0)
    yield from ctx.wait(1.0)
    # LEFT closes arm_l, then RIGHT closes arm_r
    yield from _swing(ctx, left, +1.0, mock)
    yield from _swing(ctx, right, -1.0, mock)
    yield from ctx.wait(1.5)
