"""On the station table: an irrigation trench in section, open toward the robot. A sand floor, a soil bank on the far
side and pine grade boards on both edges whose tops mark finished grade; an orange stripe and stake on the boards mark
where the spray head belongs. A white 3/4 inch PVC lateral runs across the trench floor with a tee whose side outlet
points away from the robot, and a half inch swing joint stands on it: elbow 1 on the tee outlet (swivel about x), a
152 mm nipple, elbow 2 (swivel about the nipple) and elbow 3 (swivel about elbow 2's outlet) carrying a black pop-up
spray body with a pale grey cap. Each swivel is a hinge with fine detent springs, so the joint stays where it is left.

The task: set the spray head over the stake mark, plumb, its cap top flush with the grade boards, then let go. It
starts plumb but 35.6 mm proud, elbow 2 pointing along the trench. The only pose that is plumb, flush and over the
stake swings elbow 2 back 90 degrees and then lays the nipple over 40 degrees toward the stake while elbow 3 turns
back 40 degrees, the hand following the 152 mm arc the linkage allows.

Requires -E object_jitter=0 (the trench and the lateral are fixed; the swing joint is jointed to the tee).

Run it:  inspect-robots run --embodiment yam_arena -E task=set_a_spray_head_on_its_swing_joint_to_grade.py \\
           -E object_jitter=0 -E control_hz=20 --instruction "Set the spray head over the orange stake mark,
           standing plumb with its top flush with the grade boards, then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401  (shared kit; the swing joint's geometry is its own)
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, ball, box, cyl, lathe, on_reset, on_step, rod  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the demo has parked by then
CONTROL_DT = 0.05  # s per control step (-E control_hz=20)

SOIL = (0.40, 0.29, 0.19)
SAND = (0.72, 0.62, 0.46)
PINE = (0.88, 0.78, 0.58)
ORANGE = (0.98, 0.45, 0.05)
PVC = (0.93, 0.93, 0.91)
PE = (0.55, 0.55, 0.57)  # swing joint elbows and nipple, grey polyethylene
ABS = (0.08, 0.08, 0.09)  # spray body, black ABS
CAP = (0.72, 0.72, 0.70)
STEM = (0.25, 0.25, 0.27)

# ---------------------------------------------------------------------------------------------------------------------
# Geometry, station frame (m): x forward from the arms, y left, z up; h = height above the table top.
AX = (0.285, 0.0, 0.0184)  # elbow 1's swivel axis (along x) at the tee outlet: x, y, h
L_NIP = 0.152  # elbow 1 to elbow 2, centre to centre, along the nipple
ARM2 = 0.032  # elbow 2's outlet, nipple axis to elbow 3's swivel axis
CAP_TOP = 0.122  # elbow 3's axis to the cap top, along the body
BODY_BASE = 0.020  # elbow 3's axis to the body's inlet boss base
BODY_R, CAP_R, CAP_H = 0.023, 0.028, 0.010
HOME = (40.0, -90.0, -40.0)  # elbow 1, 2, 3 (deg): plumb, flush and over the stake
GRADE_H = round(AX[2] + L_NIP * math.cos(math.radians(HOME[0])) + CAP_TOP, 4)  # 0.2568: grade-board tops
MARK = (round(AX[0] + ARM2, 4), round(-L_NIP * math.sin(math.radians(HOME[0])), 4))  # (0.317, -0.0977)
STOPS = {"elbow1": (-60.0, 60.0), "elbow2": (-100.0, 100.0), "elbow3": (-60.0, 60.0)}
DETENT_DEG = {"elbow1": 1.0, "elbow2": 2.0, "elbow3": 1.0}
SPRING = {"elbow1": 40.0, "elbow2": 6.0, "elbow3": 12.0}  # N m/rad toward the nearest detent
DAMPING = {"elbow1": 0.3, "elbow2": 0.05, "elbow3": 0.05}
MASS = {"elbow1": 0.030, "elbow2": 0.012, "elbow3": 0.102}

# The trench (fixed): y -0.15 .. +0.20, open at the -y end and toward the robot.
TR_Y = (-0.15, 0.20)
TR_LEN, TR_CY = TR_Y[1] - TR_Y[0], (TR_Y[0] + TR_Y[1]) / 2
FLOOR_X = (0.215, 0.362)
BANK_X = (0.362, 0.420)
BANK_H = round(GRADE_H - 0.019, 4)  # 0.2378: the board sits on it
FRONT_BOARD_X = (0.177, 0.215)
BACK_BOARD_X = (0.362, 0.400)
FRONT_STAKES_Y = (-0.12, 0.17)
STAKE_X = 0.410  # the orange-topped grade stake, on the far bank behind the board, at the mark's y


def _bx(x, y, h, color, collide=True):
    """A box from x, y, h ranges (station frame), in the trench Mech's frame (origin at the table top, x 0, y 0)."""
    b = box(((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (h[0] + h[1]) / 2), (x[1] - x[0], y[1] - y[0], h[1] - h[0]), color)
    return b if collide else mechkit.dataclasses.replace(b, collide=False)


_TRENCH = [
    _bx(FLOOR_X, TR_Y, (0.0, 0.003), SAND),  # 3 mm of sand; the lateral sits 2 mm above it
    _bx(BANK_X, TR_Y, (0.0, BANK_H), SOIL),
    _bx(BACK_BOARD_X, TR_Y, (BANK_H, GRADE_H), PINE),
    _bx(FRONT_BOARD_X, TR_Y, (BANK_H, GRADE_H), PINE),
    *[_bx((0.1865, 0.2055), (y - 0.0095, y + 0.0095), (0.0, BANK_H), PINE) for y in FRONT_STAKES_Y],
    _bx((STAKE_X - 0.0095, STAKE_X + 0.0095), (MARK[1] - 0.0095, MARK[1] + 0.0095), (BANK_H, GRADE_H + 0.030), PINE),
    _bx((STAKE_X - 0.0095, STAKE_X + 0.0095), (MARK[1] - 0.0095, MARK[1] + 0.0095), (GRADE_H + 0.030, GRADE_H + 0.040),
        ORANGE),
    # the mark: a 25 mm orange stripe painted across both boards at the stake's y (look only)
    _bx(FRONT_BOARD_X, (MARK[1] - 0.0125, MARK[1] + 0.0125), (GRADE_H, GRADE_H + 0.0005), ORANGE, collide=False),
    _bx(BACK_BOARD_X, (MARK[1] - 0.0125, MARK[1] + 0.0125), (GRADE_H, GRADE_H + 0.0005), ORANGE, collide=False),
]
TRENCH = Mech("trench", pos=(0.0, 0.0, float(TOP)), mass=None, friction=0.8, shapes=_TRENCH)

# The lateral, tee and swing joint, in the swing joint's frame: origin on elbow 1's axis at the tee outlet.
LAT_R, TEE_R, HUB_R, NIP_R = 0.01335, 0.0165, 0.0125, 0.01065
LAT_X = 0.255 - AX[0]  # -0.030
_BASE = [
    rod((LAT_X, TR_Y[0] + 0.005, 0.0), (LAT_X, TR_Y[1] - 0.005, 0.0), LAT_R, PVC, sides=24),  # 26.7 mm lateral
    cyl((LAT_X, 0.0, 0.0), TEE_R, 0.050, PVC, axis="y", sides=24),  # tee run hub, 33 mm
    rod((LAT_X, 0.0, 0.0), (0.268 - AX[0], 0.0, 0.0), TEE_R, PVC, sides=24),  # tee side outlet to x 0.268
]
_ELBOW1 = [  # turns about x at the tee outlet; carries the nipple
    rod((0.268 - AX[0], 0.0, 0.0), (0.0, 0.0, 0.0), HUB_R, PE, sides=16),
    ball((0.0, 0.0, 0.0), HUB_R, PE, sides=16, rows=6),
    rod((0.0, 0.0, 0.0), (0.0, 0.0, 0.017), HUB_R, PE, sides=16),
    rod((0.0, 0.0, 0.017), (0.0, 0.0, L_NIP - 0.017), NIP_R, PE, sides=16),  # 1/2 inch nipple, 21.3 mm
]
_ELBOW2 = [  # turns about the nipple's axis at its top; outlet out along +y
    rod((0.0, 0.0, -0.017), (0.0, 0.0, 0.0), HUB_R, PE, sides=16),
    ball((0.0, 0.0, 0.0), HUB_R, PE, sides=16, rows=6),
    rod((0.0, 0.0, 0.0), (0.0, 0.015, 0.0), HUB_R, PE, sides=16),
]
_BODY_PROFILE = [(0.012, BODY_BASE - 0.004), (0.012, BODY_BASE + 0.004), (BODY_R, BODY_BASE + 0.008),
                 (BODY_R, CAP_TOP - CAP_H - 0.001), (0.020, CAP_TOP - CAP_H)]
_ELBOW3 = [  # turns about elbow 2's outlet axis; carries the spray body
    rod((0.0, -0.017, 0.0), (0.0, 0.0, 0.0), HUB_R, PE, sides=16),
    ball((0.0, 0.0, 0.0), HUB_R, PE, sides=16, rows=6),
    rod((0.0, 0.0, 0.0), (0.0, 0.0, BODY_BASE - 0.004), HUB_R, PE, sides=16),
    lathe((0.0, 0.0, 0.0), _BODY_PROFILE, ABS, sides=24),  # 46 mm body on a 24 mm inlet boss
    cyl((0.0, 0.0, CAP_TOP - CAP_H / 2), CAP_R, CAP_H, CAP, sides=24),  # 56 mm cap, 10 mm tall
    mechkit.dataclasses.replace(cyl((0.0, 0.0, CAP_TOP + 0.001), 0.010, 0.002, STEM), collide=False),  # stem
    mechkit.dataclasses.replace(box((0.0, 0.0, CAP_TOP + 0.0025), (0.016, 0.003, 0.001), ABS), collide=False),  # slot
]


def _rad(name: str, deg: float) -> float:
    return float(math.radians(deg))


def _detents(name: str) -> tuple[float, ...]:
    lo, hi = STOPS[name]
    step = DETENT_DEG[name]
    n0, n1 = int(round(lo / step)), int(round(hi / step))
    return tuple(float(math.radians(k * step)) for k in range(n0, n1 + 1))


def _joint(name, axis, at, shapes, parent=None):
    return Joint(name, "hinge", axis=axis, at=at, low=_rad(name, STOPS[name][0]), high=_rad(name, STOPS[name][1]),
                 spring=SPRING[name], detents=_detents(name), damping=DAMPING[name], mass=MASS[name], parent=parent,
                 shapes=shapes)


SWING = Mech(
    "swing_joint", pos=(float(AX[0]), float(AX[1]), float(TOP + AX[2])), mass=None, friction=0.6, shapes=_BASE,
    joints=[
        _joint("elbow1", "x", (0.0, 0.0, 0.0), _ELBOW1),
        _joint("elbow2", "z", (0.0, 0.0, L_NIP), _ELBOW2, parent="elbow1"),
        _joint("elbow3", "y", (0.0, ARM2, 0.0), _ELBOW3, parent="elbow2"),
    ],
)

OBJECTS = [TRENCH, SWING]
NAMES = ("elbow1", "elbow2", "elbow3")

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
H_FULL, H_HALF, H_PART = 0.003, 0.006, 0.010  # |cap top - grade|
TILT_FULL, TILT_PART = 3.0, 6.0  # deg, body axis from vertical
C_FULL, C_PART = 0.010, 0.020  # cap centre to the mark, in plan
STILL_DEG = 0.5  # each swivel turns less than this over the last 1 s
STILL_STEPS = 20
RELEASE = 0.030  # every finger pad point at least this far from the swing joint
TOUCH = 0.002  # a finger pad point within this of the lateral or the tee: touched the fresh solvent weld
_ST: dict = {}


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, touched=False, hist=[], released=False, step=0)


_reset()


def body_pose(state) -> dict:
    """The spray body's pose from the swivel values: cap top centre (world, relative to the env origin), body axis
    (unit), tilt from vertical (deg), height of the cap top above grade (m) and plan distance to the mark (m)."""
    r, o = mechkit.body_poses(SWING, state)["elbow3"]
    axis = np.asarray(r, float) @ np.array([0.0, 0.0, 1.0])
    top = np.asarray(o, float) + CAP_TOP * axis
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2])))))
    return {"top": top, "axis": axis, "tilt": tilt, "h": float(top[2] - (TOP + GRADE_H)),
            "c": float(math.hypot(top[0] - MARK[0], top[1] - MARK[1])), "elbow3": np.asarray(o, float)}


def _dist_to_rod(p, a, b, r) -> float:
    a, b, p = (np.asarray(v, float) for v in (a, b, p))
    d = b - a
    t = min(1.0, max(0.0, float((p - a) @ d / (d @ d))))
    return float(np.linalg.norm(p - (a + t * d))) - r


_O = np.array([AX[0], AX[1], TOP + AX[2]])
_WELDS = [  # the lateral, the tee hub and its side outlet, in the station frame (all fixed)
    (_O + [LAT_X, TR_Y[0] + 0.005, 0.0], _O + [LAT_X, TR_Y[1] - 0.005, 0.0], LAT_R),
    (_O + [LAT_X, -0.025, 0.0], _O + [LAT_X, 0.025, 0.0], TEE_R),
    (_O + [LAT_X, 0.0, 0.0], _O + [0.268 - AX[0], 0.0, 0.0], TEE_R),
]


def weld_clearance(points) -> float:
    """Least distance (m) from any of ``points`` to the surface of the lateral or the tee."""
    return min((_dist_to_rod(p, a, b, r) for p in points for a, b, r in _WELDS), default=math.inf)


@on_step
def _touch_watch(env):
    """Latch a finger pad touching the freshly glued lateral or tee (any pad point within 2 mm of its surface)."""
    try:
        pts = mechkit.finger_points(env)
    except Exception:  # noqa: BLE001  no live env: nothing to latch
        return
    if weld_clearance(pts) <= TOUCH:
        _ST["touched"] = True


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    s = objects["swing_joint"]
    _ST["hist"].append([math.degrees(float(s.joints.get(n, 0.0))) for n in NAMES])
    del _ST["hist"][:-(STILL_STEPS + 1)]
    _ST["released"] = bool(mechkit.released(s, clearance=RELEASE))
    _ST["pose"] = body_pose(s)


def still() -> bool:
    h = _ST["hist"]
    if len(h) < STILL_STEPS + 1:
        return False
    return all(max(c) - min(c) < STILL_DEG for c in zip(*h))


def grade(p) -> float:
    """1.0, 0.5, 0.25 or 0 from the cap's height above grade, the body's tilt and the cap's distance to the mark."""
    h, tilt, c = abs(p["h"]), p["tilt"], p["c"]
    if c <= C_FULL and tilt <= TILT_FULL and h <= H_FULL:
        return 1.0
    if c <= C_FULL and tilt <= TILT_FULL and h <= H_HALF:
        return 0.5
    if c <= C_PART and tilt <= TILT_PART and h <= H_PART:
        return 0.25
    return 0.0


def score(objects):
    """0 if a finger pad ever touched the lateral or tee, or unless at the end the swing joint is let go (every pad
    point 30 mm or more from it) and every swivel turned under 0.5 deg over the last 1 s. Otherwise 1.0 for the cap
    top within 3 mm of grade, the body within 3 deg of plumb and the cap centre within 10 mm of the mark in plan; 0.5
    for the same with the cap 3 to 6 mm off grade; 0.25 within 20 mm, 6 deg and 10 mm; 0 otherwise."""
    _update(objects)
    if _ST["touched"] or not _ST["released"] or not still():
        return 0.0
    return grade(_ST["pose"])


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps its hands off the freshly solvent-welded lateral and tee: no arm part within 4 cm of them.
ARM_KEEP_OUT = [
    {"name": "lateral and tee, 4 cm around them", "x": (0.255 - LAT_R - 0.040, 0.268 + 0.040),
     "y": (TR_Y[0] - 0.040 + 0.005, TR_Y[1] + 0.015), "z_top": float(TOP + AX[2] + TEE_R + 0.040)},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (right arm only; the left arm holds READY). It reads the swing joint's three swivel angles from the
# simulator, and from them the spray body's pose, to aim, to servo and to confirm.
demo = placeholder_demo
PITCH = 0.8  # rad: fingers leaned 46 deg from vertical, toward +y at the grasp, toward +x after the swing
PAD_BELOW_CAP = 0.035  # pad centre (the TCP) on the body axis, this far below the cap top
OPEN64 = -0.032  # one finger joint for a 64 mm pad gap
HOVER_UP = 0.045  # m above the grasp height
SWING_DEG_S, LOWER_DEG_S = 15.0, 5.0
APPROACH_VIA = (0.300, -0.140, TOP + 0.330)
PARK_VIA = (0.290, -0.200, TOP + 0.340)


def chain_point(q1, q2, q3=None) -> tuple[np.ndarray, float]:
    """Planned grasp point (pad centre, station frame) and grip yaw (rad) with elbow 1 at q1 and elbow 2 at q2 (deg),
    elbow 3 at -q1 (body plumb; the formula is exact when q2 is 0 or -90, the two the demo works at)."""
    a, b = math.radians(q1), math.radians(q2)
    top = np.array([AX[0], AX[1], TOP + AX[2]])
    nip = top + L_NIP * np.array([0.0, -math.sin(a), math.cos(a)])
    e3 = nip + ARM2 * np.array([-math.sin(b), math.cos(b) * math.cos(a), math.cos(b) * math.sin(a)])
    return e3 + np.array([0.0, 0.0, CAP_TOP - PAD_BELOW_CAP]), math.pi / 2 + b


def _rot(yaw):
    return grip_rot(yaw=yaw, pitch=PITCH)


def _joints(ctx):
    j = ctx.obj("swing_joint").joints
    return tuple(math.degrees(float(j.get(n, 0.0))) for n in NAMES)


class _Track:
    """Moves the hand so the spray body (not the hand) follows the plan. Live, the grasp point measured from the swivel
    angles is compared with the plan each step; the error is folded into an offset on the hand command (at most 12 mm,
    cancelling arm sag and slip in the grip) and the plan waits while the body lags more than 6 mm. Offline (dry runs,
    where nothing moves) the hand follows the plan directly."""

    def __init__(self, ctx):
        self.ctx, self.R = ctx, ctx.right
        self.off = np.zeros(3)
        self.lag_mm = 0.0
        self.paused = 0

    def measured(self):
        p = body_pose(self.ctx.obj("swing_joint"))
        return p["top"] - PAD_BELOW_CAP * p["axis"]

    def step(self, point, yaw, gain=0.15):
        point = np.asarray(point, float)
        if not self.R.offline:
            err = point - self.measured()
            self.lag_mm = float(np.linalg.norm(err)) * 1000.0
            self.off = self.off + gain * err
            n = float(np.linalg.norm(self.off))
            if n > 0.012:
                self.off *= 0.012 / n
        yield from self.R.move(tuple(float(v) for v in point + self.off), CONTROL_DT, rot=_rot(yaw))

    def follow(self, pose_fn, duration, max_pause=3.0):
        n = max(1, round(duration / CONTROL_DT))
        i, paused = 0, 0
        while i < n:
            if self.lag_mm > 6.0 and paused < max_pause / CONTROL_DT:
                paused += 1
            else:
                i += 1
            yield from self.step(*pose_fn(i / n))
        self.paused += paused


def demo_script(ctx):
    R = ctx.right
    q0 = (0.0, 0.0, 0.0) if R.offline else _joints(ctx)
    grasp, yaw0 = chain_point(q0[0], q0[1])
    hover = grasp + np.array([0.0, 0.0, HOVER_UP])
    # 1) in over the open -y end of the trench, then over the body at yaw 90 (pads squeezing along x), fingers leaned toward +y, then down at 20 mm/s and close
    yield from R.move(APPROACH_VIA, 5.0, OPEN64, rot=_rot(yaw0 - math.pi / 4))  # half turned: wrist off its stop
    yield from R.move(tuple(hover), 3.0, rot=_rot(yaw0))
    yield from R.trim_to_measured(tuple(hover))
    for _attempt in range(2):
        yield from R.move(tuple(grasp), HOVER_UP / 0.020, rot=_rot(yaw0))
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.040):  # pads stopped on the 46 mm housing
            break
        yield from R.grip(OPEN64, 0.6)
        yield from R.move(tuple(hover), 1.5, rot=_rot(yaw0))
    T = _Track(ctx)
    # 2) swing elbow 2 back: the body orbits 32 mm round the nipple's axis to +x, the wrist yawing 90 -> 0 deg
    yield from T.follow(lambda s: chain_point(0.0, -90.0 * s), 90.0 / SWING_DEG_S)
    if not R.offline:  # elbow 2 must read -90 within 1 deg before the lowering
        for _k in range(2):
            q2 = _joints(ctx)[1]
            if abs(q2 + 90.0) <= 1.0:
                break
            yield from T.follow(lambda s, t=-90.0 + 1.5 * (-90.0 - q2): chain_point(0.0, t), 1.0)
    # 3) lower on the arc: elbow 1 0 -> 40 deg with the body held plumb (elbow 3 follows at -elbow 1)
    yield from T.follow(lambda s: chain_point(HOME[0] * s, HOME[1]), HOME[0] / LOWER_DEG_S)
    yield from R.hold(0.5)
    if not R.offline:  # confirm from the body pose; nudge one detent toward home and recheck once
        p = body_pose(ctx.obj("swing_joint"))
        if abs(p["h"]) > 0.0015 or p["tilt"] > 1.0:
            q1 = _joints(ctx)[0]
            target = HOME[0] + (HOME[0] - q1)
            yield from T.follow(lambda s: chain_point(q1 + (target - q1) * s, HOME[1]), 1.0)
            yield from T.follow(lambda s: chain_point(target + (HOME[0] - target) * s, HOME[1]), 1.0)
            yield from R.hold(0.5)
    # 4) let go: open to 64 mm and wait while the detents take the load
    yield from R.grip(OPEN64, 1.0)
    yield from R.hold(1.0)
    # 5) rise at 20 mm/s, move clear toward the robot and park at READY
    up = np.asarray(R.pos, float).copy()
    up[2] = TOP + 0.330
    yield from R.move(tuple(up), (up[2] - R.pos[2]) / 0.020, rot=R.rot.copy())
    yield from R.move(PARK_VIA, 3.0, rot=_rot(0.0))
    yield from R.home(5.0)
    yield from R.hold(5.0)
