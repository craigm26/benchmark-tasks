"""On the station table, between the arms: a grey riser carrying a dark, face-up four-jaw chuck (14 x 14 cm face, axis at
x 27, y 0). Four jaws (two orange on the x axis, two white on the y axis) each slide on their own along a radius, 5.5 cm
from the axis when open, 3.6 cm of travel. A yellow 4 x 4 x 8 cm block stands on the table in front of the left arm. Behind
the chuck, on the +x side, a fixed tool post carries a black cutting insert. The block and the jaws are loose; the riser, the
tool post and the white rim marks are fixed.

Requires -E object_jitter=0.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import closedloop  # noqa: E402, F401
from belimits import corners, tilt_deg  # noqa: E402
from mechkit import Joint, Mech, box  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips things: keep the episode (and the video) running until the release is seen

GREY, FACE_COL, DARK = (0.58, 0.58, 0.62), (0.13, 0.13, 0.16), (0.05, 0.05, 0.05)
ORANGE, WHITE, YELLOW = (0.98, 0.45, 0.02), (0.96, 0.96, 0.96), (0.98, 0.85, 0.05)

# -- the chuck ----------------------------------------------------------------------------------------------------------------
AX, AY = 0.27, 0.0  # chuck axis
RISER_H, FACE_T = 0.04, 0.01
FACE_Z = TOP + RISER_H + FACE_T  # the chuck face
FACE_HALF = 0.07
OPEN = 0.055  # a jaw's inner face from the axis when open
TRAVEL = 0.036  # jaw travel; at full close its face is 1.9 cm from the axis
J_SET = 0.0345  # the jaw travel that puts a face 2.05 cm from the axis (0.5 mm either side of the 4 cm block)
JAW_W = 0.03  # a jaw is 3 cm wide
BODY_H, TAB_H = 0.015, 0.03  # a 1.5 cm tall body that slides on the face, and a 3 cm tall upright grip tab on its outer end
TAB_T = 0.02  # the tab is 2 cm thick (radial)
TAB_C = 0.0355  # the tab's centre from the jaw's inner face: 9.05 cm from the axis when open, 5.6 cm when closed
BODY_L = TAB_C + TAB_T / 2  # radial length of the body: inner face to the tab's outer face (4.55 cm)
JAW_Z0 = RISER_H + FACE_T + 0.002  # jaws ride 2 mm above the face

JAW_SPEC = {  # name: (axis index, side of the axis, colour)
    "jaw_px": (0, +1, ORANGE),
    "jaw_nx": (0, -1, ORANGE),
    "jaw_py": (1, +1, WHITE),
    "jaw_ny": (1, -1, WHITE),
}


def _jaw(name):
    axis, side, colour = JAW_SPEC[name]
    at = [0.0, 0.0, JAW_Z0]
    at[axis] = side * OPEN  # the joint sits at the jaw's inner face
    body, tab = [JAW_W, JAW_W, BODY_H], [JAW_W, JAW_W, TAB_H]
    body[axis], tab[axis] = BODY_L, TAB_T
    c_body, c_tab = [0.0, 0.0, BODY_H / 2], [0.0, 0.0, BODY_H + TAB_H / 2]
    c_body[axis], c_tab[axis] = side * BODY_L / 2, side * TAB_C
    low, high = (-TRAVEL, 0.0) if side > 0 else (0.0, TRAVEL)
    return Joint(name, "slide", axis="xy"[axis], at=tuple(at), low=low, high=high, spring=0.0, damping=300.0, mass=0.05,
                 friction=0.5, shapes=[box(c_body, tuple(body), colour), box(c_tab, tuple(tab), colour)])


CHUCK = Mech(
    "chuck",
    pos=(AX, AY, TOP),
    mass=None,
    friction=0.3,
    shapes=[
        box((0.0, 0.0, RISER_H / 2), (2 * FACE_HALF, 2 * FACE_HALF, RISER_H), GREY),
        box((0.0, 0.0, RISER_H + FACE_T / 2), (2 * FACE_HALF, 2 * FACE_HALF, FACE_T), FACE_COL),
    ],
    joints=[_jaw(n) for n in ("jaw_px", "jaw_nx", "jaw_py", "jaw_ny")],
)

# White rim marks on the table, at each jaw's outer end of travel (a ruler from the jaw's outer end to its mark reads the travel).
MARK_T = 0.001


def _mark(name, axis, side):
    c = [AX, AY]
    c[axis] += side * (OPEN + BODY_L + 0.005)
    size = [JAW_W, JAW_W, MARK_T]
    size[axis] = 0.01
    return Box(name, pos=(c[0], c[1], TOP + MARK_T / 2), size=tuple(size), color=WHITE)


MARKS = [_mark("mark_px", 0, +1), _mark("mark_nx", 0, -1), _mark("mark_py", 1, +1), _mark("mark_ny", 1, -1)]

# -- the block -----------------------------------------------------------------------------------------------------------------
B = 0.04  # block square side
BH = 0.08  # block height
BLOCK_START = (0.21, 0.17)
BLOCK = Mech("block", pos=(*BLOCK_START, TOP + BH / 2 + 0.0005), mass=0.08, friction=1.0,
             shapes=[box((0.0, 0.0, 0.0), (B, B, BH), YELLOW)])

# -- the tool post (a hazard for the robot, not a scored object) ----------------------------------------------------------------
POST_X0, POST_X1, POST_HALF_Y, POST_H = 0.46, 0.495, 0.03, 0.10
POST = Mech(
    "tool_post",
    pos=(POST_X0, 0.0, TOP),
    mass=None,
    friction=0.6,
    shapes=[
        box(((POST_X1 - POST_X0) / 2, 0.0, POST_H / 2), (POST_X1 - POST_X0, 2 * POST_HALF_Y, POST_H), (0.40, 0.42, 0.50)),
        box((-0.0035, 0.0, 0.075), (0.007, 0.02, 0.01), DARK),  # the cutting insert, on the face toward the +x jaw
    ],
)

OBJECTS = [CHUCK, BLOCK, POST, *MARKS]

# -- scoring -----------------------------------------------------------------------------------------------------------------
TILT_MAX = 2.0  # degrees
SEAT_TOL = 0.002  # block bottom within this of the face
GRIP_TOL = 0.0015  # a jaw face within this of the block side facing it
CENTRE_TOL = 0.0015
YAW_TOL = 3.0  # degrees off square to the jaws
NEAR = 0.02  # 0.25: block centre this close to the axis
SLACK = 0.001  # the block "between the jaws": no jaw face is more than this inside its extent


def _yaw_off_square(s):
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    q = math.pi / 2
    return math.degrees(abs((yaw + q / 2) % q - q / 2))


def _read(objects):
    j = objects["chuck"].joints
    faces = (AX - OPEN + j["jaw_nx"], AX + OPEN + j["jaw_px"], AY - OPEN + j["jaw_ny"], AY + OPEN + j["jaw_py"])
    p = corners(objects["block"], BLOCK)
    ext = (p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max())
    return faces, ext


def _on_face(s):
    return bool(tilt_deg(s) <= TILT_MAX and abs(float(corners(s, BLOCK)[:, 2].min()) - FACE_Z) <= SEAT_TOL
                and abs(s.pos[0] - AX) < FACE_HALF and abs(s.pos[1] - AY) < FACE_HALF)


def _levels(objects):
    """(on face, x pair grips, y pair grips, centred on x, centred on y, within 2 cm and between the jaws)."""
    b = objects["block"]
    faces, ext = _read(objects)
    cx, cy = (ext[0] + ext[1]) / 2, (ext[2] + ext[3]) / 2
    square = _yaw_off_square(b) <= YAW_TOL
    gx = square and abs(faces[0] - ext[0]) <= GRIP_TOL and abs(faces[1] - ext[1]) <= GRIP_TOL
    gy = square and abs(faces[2] - ext[2]) <= GRIP_TOL and abs(faces[3] - ext[3]) <= GRIP_TOL
    near = math.hypot(cx - AX, cy - AY) <= NEAR
    between = (faces[0] <= ext[0] + SLACK and ext[1] <= faces[1] + SLACK and faces[2] <= ext[2] + SLACK
               and ext[3] <= faces[3] + SLACK)
    return (_on_face(b), gx, gy, abs(cx - AX) <= CENTRE_TOL, abs(cy - AY) <= CENTRE_TOL, near and between)


def score(objects):
    """0 unless the block stands upright on the chuck face (tilt <= 2 degrees, bottom within 2 mm of it). Then 0.25 with its
    centre within 2 cm of the axis and between all four jaws; 0.5 when one pair of opposite jaws grips it (each face within
    1.5 mm of the side facing it) and its centre is within 1.5 mm of the axis along that pair's direction; 1.0 when both
    pairs grip it and it is within 1.5 mm of the axis on both. A block held up, resting on the table or on a jaw, tipped or
    off the table scores 0, so pushing it never scores."""
    on, gx, gy, cx, cy, between = _levels(objects)
    if not on:
        return 0.0
    if gx and gy and cx and cy:
        return 1.0
    if (gx and cx) or (gy and cy):
        return 0.5
    return 0.25 if between else 0.0


def success(objects):
    return score(objects) == 1.0 and objects["block"].resting()


def distance(objects):
    """From the block's centre to the chuck axis (m)."""
    s = objects["block"]
    return float(math.hypot(s.pos[0] - AX, s.pos[1] - AY))


# The robot keeps clear of the tool post too: no part of either arm may come within 4 cm of it below 25 cm above the table
# (checked offline with the URDF; the scorer itself only sees the objects).
ARM_KEEP_OUT = [
    {"name": "tool post and cutting insert, 4 cm margin", "x": (POST_X0 - 0.04, POST_X1 + 0.04),
     "y": (-POST_HALF_Y - 0.04, POST_HALF_Y + 0.04), "z_top": TOP + 0.25},
]

# -- the scripted demo -------------------------------------------------------------------------------------------------------
HOVER = TOP + 0.16
GRIP_OPEN = -0.022  # fingers opened 4.4 cm round a 3 cm jaw
JAW_GRIP_Z = TOP + JAW_Z0 + BODY_H + TAB_H / 2 - 0.0005  # mid-height on the tab
BLOCK_GRIP_Z = TOP + 0.040  # low on the block, so it clears the jaw tabs (top 11.2 cm) when carried at 16 cm
ARM = {"jaw_px": "left", "jaw_nx": "right", "jaw_py": "left", "jaw_ny": "right"}
AIM = (AX, AY + 0.01)  # the block is set down 1 cm off centre on purpose: the jaws have to push it true


def demo():
    from closedloop import placeholder_demo
    return placeholder_demo()


def _aim(arm, goal_xy, d, z0, z1, dur, finger, stop=None, gain=0.3, clamp=0.03):
    """Move from height z0 to z1 over dur seconds while steering the TCP's measured xy so that the held part sits at goal_xy
    (the part's xy is the TCP's plus d). stop() ends the move early."""
    steps = max(1, round(dur / arm.dt))
    rot = arm.rot.copy()
    c = getattr(arm, "_c", np.zeros(2))
    for i in range(1, steps + 1):
        g = np.asarray(goal_xy() if callable(goal_xy) else goal_xy, float)
        want = g - d
        c = np.clip(c + gain * (want - arm.measured_pos[:2]), -clamp, clamp)
        z = z0 + (z1 - z0) * i / steps
        arm._set_cartesian(np.array([want[0] + c[0], want[1] + c[1], z]), rot, finger)
        yield
        if stop is not None and stop():
            break
    arm._c = c


def _move_block(ctx, arm):
    arm._c = np.zeros(2)
    b = ctx.pos("block")
    yield from arm.move((b[0], b[1], HOVER), 2.5, FINGER_OPEN)
    yield from arm.move((b[0], b[1], BLOCK_GRIP_Z), 1.5, FINGER_OPEN)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from arm.move((b[0], b[1], HOVER), 1.5, FINGER_CLOSED)
    yield from arm.hold(0.4)
    d = ctx.pos("block")[:2] - arm.measured_pos[:2]
    yield from arm.move((AIM[0] - d[0], AIM[1] - d[1], HOVER), 3.0, FINGER_CLOSED)
    arm._c = np.zeros(2)
    yield from _aim(arm, AIM, d, HOVER, HOVER, 1.2, FINGER_CLOSED)
    down = lambda: float(ctx.obj("block").low[2]) <= FACE_Z + 0.0015  # noqa: E731
    z_face = FACE_Z + (BLOCK_GRIP_Z - TOP)  # TCP height with the block's bottom on the face
    yield from _aim(arm, AIM, d, HOVER, z_face + 0.03, 1.5, FINGER_CLOSED)
    yield from _aim(arm, AIM, d, z_face + 0.03, z_face - 0.004, 3.0, FINGER_CLOSED, stop=down)
    yield from arm.hold(0.5)
    yield from arm.grip(FINGER_OPEN, 0.8)
    yield from arm.move((arm.pos[0], arm.pos[1], HOVER), 1.5, FINGER_OPEN)
    yield from arm.home(3.0)


def _jaw_coord(ctx, name):
    axis, side, _ = JAW_SPEC[name]
    return (AX, AY)[axis] + side * (OPEN + TAB_C) + float(ctx.obj("chuck").joints[name])  # the tab's centre


def _work_jaw(ctx, name):
    """Take a jaw by its sides, carry it in until its measured travel reaches J_SET, let go and go home."""
    axis, side, _ = JAW_SPEC[name]
    arm = getattr(ctx, ARM[name])
    yaw = 0.0 if axis == 0 else (math.pi / 2 if arm is ctx.right else -math.pi / 2)  # fingers close across the jaw's 3 cm side
    hover = TOP + 0.118 if name == "jaw_px" else HOVER  # the left arm cannot reach the +x jaw from higher
    c0 = _jaw_coord(ctx, name)

    def at(c, z):
        xy = [AX, AY]
        xy[axis] = c
        return (xy[0], xy[1], z)

    yield from arm.move(at(c0, hover), 2.5, GRIP_OPEN, yaw=yaw)
    yield from arm.move(at(c0, JAW_GRIP_Z), 1.5, GRIP_OPEN, yaw=yaw)
    yield from arm.hold(0.3)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    target = (AX, AY)[axis] + side * (OPEN - J_SET + TAB_C)  # tab centre coordinate at J_SET
    cmd = float(arm.pos[axis])
    cmd_lim = (cmd - 0.041, cmd) if side > 0 else (cmd, cmd + 0.041)  # never past the jaw's full travel
    ref = _jaw_coord(ctx, name)
    settled = 0
    for _ in range(260):
        v = 0.02 if abs(ref - target) > 0.012 else 0.008
        step = min(v * arm.dt, abs(ref - target))
        ref -= side * step
        now = _jaw_coord(ctx, name)
        cmd = min(max(cmd - side * step + 0.5 * (ref - now), cmd_lim[0]), cmd_lim[1])
        p = list(arm.pos)
        p[axis] = cmd
        arm._set_cartesian(np.array(p), arm.rot.copy(), FINGER_CLOSED)
        yield
        settled = settled + 1 if abs(now - target) <= 0.0003 else 0
        if settled >= 5:
            break
    yield from arm.grip(FINGER_OPEN, 0.5)
    yield from arm.move((arm.pos[0], arm.pos[1], hover), 1.5, FINGER_OPEN, yaw=yaw)
    yield from arm.home(3.0)


def demo_script(ctx):
    """The left arm lifts the block onto the chuck face 1 cm off the axis and goes home. Then each jaw in turn is taken by its
    sides and carried in to 2.05 cm from the axis, the x pair first (right arm the -x jaw, left arm the +x jaw), then the y
    pair (right arm the -y jaw, left arm the +y jaw), which pushes the block the last centimetre true."""
    yield from _move_block(ctx, ctx.left)
    for name in ("jaw_nx", "jaw_px", "jaw_ny", "jaw_py"):
        yield from _work_jaw(ctx, name)
    yield from ctx.wait(1.0)
