"""A full-size hand shower on its hose, and the slotted wall holder it docks in.

On the far side of the station table stands a tiled wall panel. On it: a matte black holder cup (a conical bore, 26 mm
at the bottom ring and 34 mm at the rim, 32 mm deep, leaning 15 degrees toward the user, with a front slot 18 mm wide)
and a black outlet elbow. A 12 mm black hose, 0.50 m long, is anchored at the elbow, runs down the wall, along the
table and up to a white hand shower that lies face down on a stone ledge, its handle overhanging toward the wall.
The robot picks the handset up by its handle, stands it head up, brings the hanging hose in through the cup's front
slot and lowers the tapered handle until it seats on the cup's bottom ring, then lets go.

Requires -E object_jitter=0. Run it:

  inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=dock_a_hand_shower_in_its_slotted_wall_holder.py \\
      -E object_jitter=0 -E control_hz=20 --instruction "Put the hand shower back in its wall holder: hose in through the
      cup's front slot, handle down into the cup, let go." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import belimits  # noqa: F401  shared limits kit (the scorer below needs only its Rect)
import mechkit
import ropekit
from closedloop import placeholder_demo
from mechkit import Mech, box, cyl, lathe, on_reset, on_step, ring_profile, rod
from ropekit import EndBody, Rope, fillet
from yam_arena import ik
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the episode runs on 8 s after success, so the video shows the release and the parked arm

TILE, GROUT, STONE = (0.62, 0.72, 0.80), (0.45, 0.50, 0.55), (0.52, 0.50, 0.47)
BLACK, WHITE, CHROME, RED, NOZZLE = (0.05, 0.05, 0.06), (0.96, 0.96, 0.94), (0.78, 0.80, 0.82), (0.85, 0.10, 0.10), (0.30, 0.32, 0.35)


def _look(shape):
    """A look-only copy of a shape (no collider)."""
    return dataclasses.replace(shape, collide=False)


# -- the wall panel, with the outlet elbow (look only) ------------------------------------------------------------------
WALL_X = 0.470  # the tile face
WALL_Y, WALL_H, WALL_T = (-0.22, 0.22), 0.36, 0.012
SPOUT = (0.458, 0.165, TOP + 0.1727)  # where the hose leaves the elbow, pointing down
_wall_shapes = [box((WALL_T / 2, 0.0, WALL_H / 2), (WALL_T, WALL_Y[1] - WALL_Y[0], WALL_H), TILE)]
_wall_shapes += [_look(box((-0.0005, y, WALL_H / 2), (0.001, 0.002, WALL_H), GROUT)) for y in (-0.11, 0.0, 0.11)]
_wall_shapes += [_look(box((-0.0005, 0.0, z), (0.001, WALL_Y[1] - WALL_Y[0], 0.002), GROUT)) for z in (0.09, 0.18, 0.27)]
_ez = SPOUT[2] - TOP + 0.024
_wall_shapes += [
    _look(cyl((-0.003, SPOUT[1], _ez), 0.022, 0.006, BLACK, axis="x")),  # outlet rosette on the tile
    _look(rod((0.0, SPOUT[1], _ez), (SPOUT[0] - WALL_X, SPOUT[1], _ez), 0.010, BLACK)),  # elbow, out of the wall
    _look(rod((SPOUT[0] - WALL_X, SPOUT[1], _ez + 0.008), (SPOUT[0] - WALL_X, SPOUT[1], SPOUT[2] - TOP), 0.010, BLACK)),
]
WALL = Mech("wall", pos=(WALL_X, 0.0, TOP), mass=None, shapes=_wall_shapes)

# -- the holder: a slotted conical cup on an arm ------------------------------------------------------------------------
TILT = math.radians(15.0)
U = np.array([-math.sin(TILT), 0.0, math.cos(TILT)])  # cup axis, bottom ring to rim (the rim leans toward the user)
F = np.array([-math.cos(TILT), 0.0, -math.sin(TILT)])  # out of the slot (toward the user, a little down)
C0 = np.array([0.395, 0.025, TOP + 0.151])  # centre of the bottom ring, where the handle seats
CUP_H, R_SEAT, R_RIM, CUP_WALL = 0.032, 0.013, 0.017, 0.004
SLOT_A = 2.374  # the wall spans -2.374..2.374 rad from the back: the front slot is 18.0 mm wide at the bottom ring
SLOT_Y = float(C0[1])
_TILT_Q = (math.cos(-TILT / 2), 0.0, math.sin(-TILT / 2), 0.0)  # (w, x, y, z): 15 deg about y, top toward -x
CUP = ring_profile((0.0, 0.0, 0.0), [(R_SEAT, R_SEAT + CUP_WALL, 0.0), (R_RIM, R_RIM + CUP_WALL, CUP_H)], BLACK,
                   tilt=_TILT_Q, a0=-SLOT_A, a1=SLOT_A)
ARM_X0 = 0.4078 - float(C0[0])  # the arm starts in the cup's back wall and runs to the tile
HOLDER_ARM = box(((ARM_X0 + WALL_X - C0[0]) / 2, 0.0, 0.021), (WALL_X - C0[0] - ARM_X0, 0.024, 0.016), BLACK)
ROSETTE = cyl((WALL_X - C0[0] - 0.003, 0.0, 0.021), 0.025, 0.006, BLACK, axis="x")
HOLDER = Mech("holder", pos=tuple(float(v) for v in C0), mass=None, shapes=[CUP, HOLDER_ARM, ROSETTE])

# -- the ledge the handset starts on ---------------------------------------------------------------------------------------
HY = -0.045  # the handset's axis as laid
LEDGE_X, LEDGE_Y, LEDGE_H = (0.130, 0.288), (HY - 0.045, HY + 0.045), 0.100
LEDGE = Box("ledge", pos=((LEDGE_X[0] + LEDGE_X[1]) / 2, HY, TOP + LEDGE_H / 2),
            size=(LEDGE_X[1] - LEDGE_X[0], LEDGE_Y[1] - LEDGE_Y[0], LEDGE_H), color=STONE)

# -- the handset: the hose's end body. Its frame: origin at the hose-side tip of the nut, +x along the handle toward the
#    head, z up as laid (the spray face is -z) ---------------------------------------------------------------------------
NUT_R, NUT_L = 0.010, 0.010
HANDLE_R0, HANDLE_R, TAPER_END, HANDLE_END = 0.011, 0.014, 0.031, 0.160
SEAT = NUT_L + (TAPER_END - NUT_L) * (R_SEAT - HANDLE_R0) / (HANDLE_R - HANDLE_R0)  # 0.024: where the handle is 26 mm
HEAD_AT, HEAD_R, HEAD_T = 0.205, 0.045, 0.028
HANDSET_LEN = HEAD_AT + HEAD_R
HANDSET_MASS = 0.10
HANDSET_SHAPES = (
    cyl((NUT_L / 2, 0.0, 0.0), NUT_R, NUT_L, CHROME, axis="x", sides=24),  # hose nut
    lathe((0.0, 0.0, 0.0), [(HANDLE_R0, NUT_L), (HANDLE_R, TAPER_END), (HANDLE_R, HANDLE_END + 0.004)], WHITE, axis="x"),
    cyl((HEAD_AT, 0.0, 0.0), HEAD_R, HEAD_T, WHITE),  # round head, spray face down as laid
    _look(cyl((0.012, 0.0, 0.0), 0.0118, 0.006, RED, axis="x")),  # red band 9 to 15 mm from the tip: the depth mark
    _look(cyl((HEAD_AT, 0.0, 0.0), HEAD_R + 0.0006, 0.010, CHROME)),  # chrome rim
    _look(cyl((HEAD_AT, 0.0, -HEAD_T / 2 - 0.0004), 0.036, 0.001, NOZZLE)),  # nozzle disc on the spray face
)
HANDSET = EndBody(HANDSET_SHAPES, mass=HANDSET_MASS, friction=1.0)

# -- the hose: anchored at the spout, down the wall, along the table, up and in to the handset's nut ---------------------
HOSE_R, HOSE_LEN = 0.006, 0.50
TIP0 = (0.390, HY, TOP + LEDGE_H + HANDLE_R + 0.0005)  # the nut's tip as laid
_ZT = TOP + HOSE_R + 0.0005
HOSE_PATH = fillet([SPOUT, (SPOUT[0], SPOUT[1], _ZT), (SPOUT[0], HY, _ZT), (SPOUT[0], HY, TIP0[2]), TIP0], 0.038)
HOSE = Rope("hose", path=HOSE_PATH, length=HOSE_LEN, segment=0.025, radius=HOSE_R, rope_mass=0.085, bend_stiffness=0.015, max_bend=1.0,
            anchor_start=True, end_body=HANDSET, color=BLACK)
N_NEAR = 6  # the hose segments nearest the handset that count as "holding the handset" for the release rule

OBJECTS = [WALL, HOLDER, LEDGE, HOSE]

# The declared limit of this task's safety envelope: no part of either arm goes near the wall outlet elbow or the top
# 10 cm of hose below it (the spout is at TOP + 0.173; box grown by 4 cm in x and y).
OUTLET_BOX = {"x": (0.420, 0.482), "y": (SPOUT[1] - 0.035, SPOUT[1] + 0.035), "z": (TOP + 0.040, TOP + 0.240)}
ARM_KEEP_OUT = [{"name": "outlet elbow and the top 10 cm of hose", "x": (OUTLET_BOX["x"][0] - 0.02, OUTLET_BOX["x"][1]),
                 "y": (OUTLET_BOX["y"][0] - 0.04, OUTLET_BOX["y"][1] + 0.04), "z_top": TOP + 0.25}]

# -- scoring -----------------------------------------------------------------------------------------------------------------
SEAT_LOW, SEAT_HIGH, SEAT_OFF, SEAT_ANG = -0.004, 0.006, 0.004, 10.0  # seat point: along the cup axis, off it, axis angle
JAM_BELOW_RIM, JAM_ANG = 0.010, 45.0  # the 0.4 rung: nut end at least 10 mm below the rim plane, inside the bore
RELEASE = 0.030  # every finger point at least this far from the handset and the hose's last 6 segments
STILL_V, STILL_W, STILL_S = 0.010, 5.0, 2.0  # m/s, deg/s, s
DROP_V, DROP_Z = 0.6, TOP + 0.030  # the handset let fall: this fast, or this low, with no finger within 30 mm
STRETCH = 1.02  # a guard only: a chain of rigid segments cannot stretch unless the simulation broke
STEP_S = 1.0 / 20.0  # the runs use 20 Hz control


def _fresh():
    return {"last": None, "prev": None, "still": 0.0, "fast": 0, "steps": 0, "dropped": False, "torn": False,
            "clear": False, "fingers": None, "t": None, "dt": STEP_S}


_S = _fresh()


@on_reset
def _reset_scorer(env=None):
    _S.clear()
    _S.update(_fresh())


@on_step
def _see_fingers(env=None):
    """Scorer side: where the finger pads of both arms are (the demo never reads this)."""
    try:
        _S["fingers"] = [np.asarray(p, dtype=float) for p in mechkit.finger_points(env)]
        t = mechkit.elapsed()
        if _S["t"] is not None and 0.005 < t - _S["t"] < 0.5:
            _S["dt"] = t - _S["t"]
        _S["t"] = t
    except Exception:  # noqa: BLE001  no live simulator: the fingers are unknown and nothing counts as released
        _S["fingers"] = None


def handset_pose(s):
    """(tip T, axis a from the tip toward the head, spray-face normal n = the end frame's z) of the handset."""
    t = np.asarray(s.end_point("end"), dtype=float)
    a = np.asarray(s.end_point("end", (1.0, 0.0, 0.0)), dtype=float) - t
    n = np.asarray(s.end_point("end", (0.0, 0.0, 1.0)), dtype=float) - t
    return t, a / np.linalg.norm(a), n / np.linalg.norm(n)


def cup_frame(p):
    """(height along the cup axis above the bottom ring, distance off the axis) of a world point."""
    d = np.asarray(p, dtype=float) - C0
    h = float(d @ U)
    return h, float(np.linalg.norm(d - h * U))


def _angle(a, b) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def seated(s) -> bool:
    """The handle's 26 mm section is on the cup's bottom ring: the seat point S = T + 24 mm along the axis lies within
    -4..+6 mm of the ring centre along the cup axis and 4 mm off it, and the axis is within 10 deg of the cup's."""
    t, a, _ = handset_pose(s)
    h, r = cup_frame(t + SEAT * a)
    return SEAT_LOW <= h <= SEAT_HIGH and r <= SEAT_OFF and _angle(a, U) <= SEAT_ANG


def in_bore(s) -> bool:
    """The nut end is inside the cup's bore at least 10 mm below the rim plane, or has passed down through the bottom
    ring with the handle still in it, head up within 45 deg of the cup axis."""
    t, a, _ = handset_pose(s)
    if _angle(a, U) > JAM_ANG:
        return False
    h, r = cup_frame(t)
    if 0.0 <= h <= CUP_H - JAM_BELOW_RIM:
        return r <= R_SEAT + (R_RIM - R_SEAT) * h / CUP_H
    if h < 0.0:  # the nut is below the ring: the handle must cross the ring plane inside the ring
        k = -h / float(a @ U)
        return k <= HANDLE_END and cup_frame(t + k * a)[1] <= R_SEAT
    return False


def handset_lowest(s) -> float:
    """Lowest point of the handset (m, world z), from its pose."""
    t, a, n = handset_pose(s)
    side = math.sqrt(max(0.0, 1.0 - a[2] ** 2))
    low = min(t[2] - NUT_R * side, (t + HANDLE_END * a)[2] - HANDLE_R * side)
    head = (t + HEAD_AT * a)[2] - HEAD_R * math.sqrt(max(0.0, 1.0 - n[2] ** 2)) - HEAD_T / 2 * abs(n[2])
    return float(min(low, head))


def finger_clearance(fingers, s) -> float:
    """Least distance (m) from any finger point to the handset (two boxes round the handle and the head, in its own
    frame) or to the hose's last 6 segments."""
    if fingers is None:
        return 0.0
    t, a, n = handset_pose(s)
    rot = np.column_stack([a, np.cross(n, a), n])
    local = (np.asarray(fingers, dtype=float).reshape(-1, 3) - t) @ rot
    best = math.inf
    for lo, hi in (((0.0, -HANDLE_R, -HANDLE_R), (HANDLE_END, HANDLE_R, HANDLE_R)),
                   ((HANDLE_END, -HEAD_R, -HEAD_T / 2), (HANDSET_LEN, HEAD_R, HEAD_T / 2))):
        out = np.maximum(np.maximum(np.asarray(lo) - local, local - np.asarray(hi)), 0.0)
        best = min(best, float(np.linalg.norm(out, axis=1).min()))
    near = np.asarray(s.points, dtype=float)[-N_NEAR:]
    d = np.linalg.norm(np.asarray(fingers, dtype=float).reshape(-1, 1, 3) - near[None, :, :], axis=2)
    return float(min(best, d.min() - s.seg_len / 2 - s.radius))


def _track(objects) -> None:
    """Update the whole-run latches and the stillness clock once per control step (success() and score() are both
    called on the same dict each step)."""
    if objects is _S["last"]:
        return
    _S["last"] = objects
    s = objects["hose"]
    t, a, n = handset_pose(s)
    dt = _S["dt"]
    _S["steps"] += 1
    gap = finger_clearance(_S["fingers"], s)
    _S["clear"] = _S["fingers"] is not None and gap >= RELEASE
    let_go = gap >= RELEASE or _S["fingers"] is None
    if _S["prev"] is not None:
        tp, ap, np_ = _S["prev"]
        v = float(np.linalg.norm(t - tp)) / dt
        w = max(_angle(a, ap), _angle(n, np_)) / dt
        _S["still"] = _S["still"] + dt if (v < STILL_V and w < STILL_W) else 0.0
        _S["fast"] = _S["fast"] + 1 if (v > DROP_V and let_go) else 0
        if _S["fast"] >= 2 and _S["steps"] > 10:  # two steps running, and not the settling just after a reset
            _S["dropped"] = True
    _S["prev"] = (t, a, n)
    if let_go and handset_lowest(s) < DROP_Z:
        _S["dropped"] = True
    if s.measured_length() > STRETCH * s.length:
        _S["torn"] = True


def score(objects):
    """0 if the handset was ever dropped (let fall at over 0.6 m/s, or left with any part under 3 cm above the table,
    with no finger within 3 cm) or the hose was torn. Otherwise, once the handset is released (every finger point 3 cm
    clear) and has been still for 2 s: 1.0 seated on the cup's bottom ring, 0.4 jammed in the bore unseated, else 0. A
    handset still in the gripper scores 0 wherever it is."""
    _track(objects)
    if _S["dropped"] or _S["torn"]:
        return 0.0
    if not _S["clear"] or _S["still"] < STILL_S:
        return 0.0
    s = objects["hose"]
    if seated(s):
        return 1.0
    return 0.4 if in_bore(s) else 0.0


def success(objects):
    return score(objects) == 1.0


# -- the scripted demo (right arm only; the left arm holds READY) --------------------------------------------------------
GRASP = 0.080  # the pads close on the handle this far from the tip (fingers 64 to 96 mm, clear of the rim at 56 mm)
TCP_SHORT = 0.005  # the tool point stops this far short of the handle's axis, to keep the gripper body off the ledge
GRIP_OPEN, GRIP_SHUT = -0.030, -0.010  # one finger's travel: pads 60 mm apart, then commanded to 20 mm (handle 28 mm)
R_LEVEL = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])  # gripper pointing +y, pads closing along z
ROLL = math.pi / 2 - TILT  # 75 deg: the handset goes from lying toward the wall to head up along the cup axis
LIFT = 0.075
ABOVE_RIM, FRONT = 0.015, 0.040  # pre-dock: nut tip 15 mm above the rim plane, 40 mm out in front of the slot

demo = placeholder_demo


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _axis(theta):
    return np.array([-math.cos(theta), 0.0, math.sin(theta)])


def _rotvec(v):
    v = np.asarray(v, dtype=float)
    ang = float(np.linalg.norm(v))
    if ang < 1e-9:
        return np.eye(3)
    k = v / ang
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * kx + (1.0 - math.cos(ang)) * (kx @ kx)


def _clamp(v, limit):
    m = float(np.linalg.norm(v))
    return v if m <= limit else v * (limit / m)


def _smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def _live(ctx):
    """The handset's tip and axis and the hose's segment centres from the simulator, or None offline (a dry run has
    no rope state, so the demo then follows its nominal path)."""
    try:
        s = ctx.obj("hose")
        t, a, _ = handset_pose(s)
        return t, a, np.asarray(s.points, dtype=float)
    except Exception:  # noqa: BLE001
        return None


class _Carry:
    """Carries the gripped handset along a desired path of its seat point S and its axis angle, correcting the arm's
    command each step from the handset's read pose (wrist sag, slip in the pads and the hose's pull all show there)."""

    def __init__(self, ctx, arm, s0, theta0):
        self.ctx, self.arm = ctx, arm
        self.rv = np.zeros(3)
        self.dp = np.zeros(3)
        self.dp = np.asarray(arm.pos, dtype=float) - self._tcp(s0, theta0)

    def _tcp(self, s_des, theta):
        a = _rotvec(self.rv) @ _axis(theta)
        return np.asarray(s_des, dtype=float) + (GRASP - SEAT) * a - np.array([0.0, TCP_SHORT, 0.0]) + self.dp

    def step(self, s_des, theta, gain=0.12, free_axis=None):
        live = _live(self.ctx)
        if live is not None:
            t, a, _ = live
            e = np.asarray(s_des, dtype=float) - (t + SEAT * a)
            if free_axis is not None:  # do not push along this direction (the seat itself stops the handle there)
                e = e - float(e @ free_axis) * free_axis
            self.dp = _clamp(self.dp + gain * _clamp(e, 0.02), 0.06)
            self.rv = _clamp(self.rv + gain * _clamp(np.cross(a, _axis(theta)), 0.2), 0.25)
        yield from self.arm.move(self._tcp(s_des, theta), self.arm.dt, rot=_rotvec(self.rv) @ _ry(theta) @ R_LEVEL)

    def run(self, s_fn, theta_fn, duration, **kw):
        steps = max(1, round(duration / self.arm.dt))
        for i in range(1, steps + 1):
            x = _smooth(i / steps)
            yield from self.step(s_fn(x), theta_fn(x), **kw)


def _hose_bias(ctx, bias):
    """Sideways offset (m) for the handset that centres the hose, where it hangs at cup height in front of the cup,
    on the slot's centre plane. Returns (bias, worst hose offset from the plane) or (bias, None) offline."""
    live = _live(ctx)
    if live is None:
        return bias, None
    pts = live[2][-5:-1]  # the last segment is rigid with the handset; the four before it are the ones that must enter
    use = [p for p in pts if -0.005 <= cup_frame(p)[0] <= CUP_H + 0.005] or list(pts[-2:])
    off = np.array([SLOT_Y - p[1] for p in use])
    bias = float(np.clip(bias + 0.15 * float(off.mean()), -0.005, 0.005))
    return bias, float(np.abs(off).max())


def _swing(arm, q1, duration):
    """Joint-space move to joint angles q1 (as closedloop's home() does toward READY)."""
    q0, q1 = arm.q.copy(), np.asarray(q1, dtype=float)
    steps = max(1, round(duration / arm.dt))
    for i in range(1, steps + 1):
        arm.q = q0 + ik.smoothstep(i / steps) * (q1 - q0)
        arm.pos, arm.rot = ik.tcp_pose(arm.q, arm.base_pos)
        yield


def demo_script(ctx):
    """Right arm: grip the handle where it overhangs the ledge, lift, turn the handset head up, line the hanging hose
    up with the cup's slot, bring it in through the slot, lower the handle onto the ring, let go and park."""
    R = ctx.right
    tip0, axis0 = np.array(TIP0), _axis(0.0)
    live = _live(ctx)
    if live is not None:
        tip0, axis0 = live[0], live[1]
    grasp = tip0 + GRASP * axis0 - np.array([0.0, TCP_SHORT, 0.0])
    before = grasp + np.array([0.0, -0.055, 0.0])

    # 1) pick: level gripper pointing +y, one finger above the handle and one below, in from the side
    above = before + np.array([0.0, 0.0, 0.065])
    q_above, _, _ = ik.solve(above, ik.matrix_to_quat_xyzw(R_LEVEL), R.q.copy(), base_pos=R.base_pos, iters=200)
    yield from _swing(R, q_above, 3.0)  # joint-space swing from READY: the wrist turns level on the way
    yield from R.move(above, 0.4, GRIP_OPEN, rot=R_LEVEL)
    yield from R.move(before, 1.2, GRIP_OPEN, rot=R_LEVEL)
    yield from R.trim_to_measured(before)
    yield from R.move(grasp, 1.5, GRIP_OPEN, rot=R_LEVEL)
    yield from R.trim_to_measured(grasp, max_comp=0.012)
    for attempt in range(2):
        yield from R.grip(GRIP_SHUT, 1.0)
        yield from R.hold(0.5)
        if R.holding(0.022):
            break
        yield from R.grip(GRIP_OPEN, 0.6)
        yield from R.trim_to_measured(grasp, max_comp=0.012)
    else:
        yield from R.home()
        return

    # 2) lift and turn: 75 mm up, then roll the wrist 75 deg so the head rises along the cup's axis
    live = _live(ctx)
    s0 = (live[0] + SEAT * live[1]) if live is not None else tip0 + SEAT * axis0
    carry = _Carry(ctx, R, s0, 0.0)
    p_grasp = s0 + (GRASP - SEAT) * _axis(0.0)
    p_up = p_grasp + np.array([0.0, 0.0, LIFT])
    yield from carry.run(lambda x: s0 + x * np.array([0.0, 0.0, LIFT]), lambda x: 0.0, 2.0)
    yield from carry.run(lambda x: p_up - (GRASP - SEAT) * _axis(ROLL * x), lambda x: ROLL * x, 3.0)

    # 3) line up in front of the slot: coaxial with the cup, 40 mm out along the slot direction, tip above the rim
    s_turn = p_up - (GRASP - SEAT) * U
    s_on = C0 + (CUP_H + ABOVE_RIM + SEAT) * U  # on the cup axis, nut tip 15 mm above the rim plane
    s_pre = s_on + FRONT * F
    yield from carry.run(lambda x: s_turn + x * (s_pre - s_turn), lambda x: ROLL, 3.0)
    bias, slow = 0.0, False
    for i in range(30):  # up to 1.5 s: wait until the hose hangs within 3 mm of the slot's centre plane
        bias, worst = _hose_bias(ctx, bias)
        yield from carry.step(s_pre + np.array([0.0, bias, 0.0]), ROLL)
        if worst is None or (i >= 9 and worst <= 0.003):
            break
    else:
        slow = True

    # 4) slot: 40 mm straight in, the hose passing through the slot (20 mm/s, or 10 mm/s if it was still swinging)
    steps = round((4.0 if slow else 2.0) / R.dt)
    for i in range(1, steps + 1):
        bias, _ = _hose_bias(ctx, bias)
        yield from carry.step(s_pre + (i / steps) * (s_on - s_pre) + np.array([0.0, bias, 0.0]), ROLL)

    # 5) seat: lower along the cup axis at 15 mm/s until the handle stops on the ring
    depth_max, speed = CUP_H + ABOVE_RIM + SEAT + 0.002, 0.015
    depth, stuck, lag0 = 0.0, 0, R.tracking_error(-U)
    while depth < depth_max:
        depth = min(depth_max, depth + speed * R.dt)
        b = bias * max(0.0, 1.0 - depth / 0.030)  # the sideways offset is gone by the time the nut is in the rim
        yield from carry.step(s_on - depth * U + np.array([0.0, b, 0.0]), ROLL, free_axis=U)
        live = _live(ctx)
        if live is not None:
            h = cup_frame(live[0] + SEAT * live[1])[0]
            behind = h - (CUP_H + ABOVE_RIM + SEAT - depth)
            pressed = (R.tracking_error(-U) - lag0) > 0.006
            stuck = stuck + 1 if (h <= 0.002 or ((behind > 0.006 or pressed) and h < 0.006)) else 0
            if stuck >= 2:
                break
    yield from R.hold(0.5)

    # 6) release: open, slide the pads off the handle sideways, park at READY and stay still
    yield from R.grip(GRIP_OPEN, 1.0)
    yield from R.hold(0.3)
    yield from R.move(np.asarray(R.pos, dtype=float) + np.array([0.0, -0.080, 0.0]), 1.5, GRIP_OPEN, rot=R.rot.copy())
    yield from R.move(above, 2.0, GRIP_OPEN, rot=R_LEVEL)  # back toward the robot, clear of the wall and the head
    yield from R.home()
    yield from ctx.wait(11.0)
