"""On the station table (the vanity counter): a white glazed tile panel stands across the far side of the counter, its
face at x 0.325, and an extendable shaving mirror is screwed to it on a round black wall plate. The mirror hangs on two
black arm links (110 and 100 mm, 18 mm tube) joined by vertical hinges at the wall, the elbow and the head, and its
round two-sided head (150 mm across, 25 mm thick bezel, silver glass on both faces) is swung out over the counter to
the robot's right, the elbow bent 17 degrees toward the wall side and the glass turned 50 degrees from facing the robot.

The task: fold the mirror away flat, both links back against the wall and the glass parallel to it, then let go.
Pushed straight back, the elbow jams link 2 and the head against the tile; the fold that works carries the head round
in front of the wall plate the way the elbow already bends, and the head must be squared to the wall first, because
near the folded pose the tile stops a turned head about 39 degrees from parallel.

Requires -E object_jitter=0 (the mirror and the tile are fixed to the wall; nothing on the counter is loose).

Run it:  inspect-robots run --embodiment yam_arena -E task=fold_a_swing_arm_wall_mirror_flat.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "Fold the wall mirror away: both arm links back flat against the wall and
           the mirror parallel to it, then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401  (shared kit; geometry here is the mirror's own)
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, on_reset, rod, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the demo has parked by then
CONTROL_DT = 0.05  # s per control step (-E control_hz=20)

TILE = (0.93, 0.93, 0.91)
GROUT = (0.60, 0.60, 0.58)
BLACK = (0.12, 0.12, 0.13)  # matt black brass
GLASS = (0.86, 0.89, 0.93)  # silvered glass
CHROME = (0.78, 0.79, 0.80)

# ---------------------------------------------------------------------------------------------------------------------
# Geometry, station frame (m): x forward from the arms, y left, z up. Angles in degrees, + counter-clockwise from above.
TILE_X = 0.325  # the tile face
TILE_H = 0.24  # tile panel height above the counter
SHOULDER = (0.295, 0.0)  # shoulder axis, 30 mm in front of the tile
Z_SHOULDER = TOP + 0.069  # link 1 height
Z_ELBOW = TOP + 0.095  # link 2 and head centre height (elbow hinge 26 mm above the shoulder's)
L1, L2 = 0.110, 0.100  # link lengths, axis to axis
NECK = 0.030  # swivel pivot to head centre, along the glass normal
HEAD_R, BEZEL_T = 0.075, 0.025
GLASS_R, GLASS_T = 0.068, 0.0262  # glass 0.6 mm proud of each bezel face
HALF_T = GLASS_T / 2
# Relative hinge angles (0 = link straight out at the robot, i.e. along -x; each measured from the link before).
START = (60.0, 17.0, -127.0)  # shoulder, elbow, swivel at the start
FOLDED = (-90.0, 178.0, -88.0)  # the fold the demo makes (the mirror image, +90 / -178 / +88, scores the same)
STOPS = {"shoulder": (-90.0, 90.0), "elbow": (-178.0, 178.0), "swivel": (-150.0, 150.0)}
HOLDS = {"shoulder": 0.25, "elbow": 0.15, "swivel": 0.10}  # N m, Coulomb friction in each vertical hinge


def _u(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a)])


def chain(a1: float, a2: float, a3: float) -> dict:
    """Planar pose of the mirror at relative angles (deg): elbow and pivot (x, y), the glass normal's direction (deg,
    pointing out of the face toward the robot at the folded pose) and the head centre."""
    d1, d2 = 180.0 + a1, 180.0 + a1 + a2
    elbow = np.array(SHOULDER) + L1 * _u(d1)
    pivot = elbow + L2 * _u(d2)
    normal = d2 + a3
    return {"elbow": elbow, "pivot": pivot, "normal": normal, "centre": pivot + NECK * _u(normal)}


def chain_ik(pivot, branch: float = 1.0) -> tuple[float, float]:
    """Shoulder and elbow angles (deg) that put the swivel pivot at ``pivot``, elbow bent the ``branch`` way."""
    v = np.asarray(pivot, dtype=float) - np.array(SHOULDER)
    d = float(np.hypot(*v))
    c = np.clip((d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1.0, 1.0)
    a2 = branch * math.degrees(math.acos(c))
    r2 = math.radians(a2)
    d1 = math.degrees(math.atan2(v[1], v[0]) - math.atan2(L2 * math.sin(r2), L1 + L2 * math.cos(r2)))
    a1 = (d1 - 180.0 + 180.0) % 360.0 - 180.0
    return a1, a2


S0 = chain(*START)


def _zq(deg: float) -> tuple[float, float, float, float]:
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _p3(xy, z=0.0) -> tuple[float, float, float]:
    return (float(xy[0]), float(xy[1]), float(z))


# Every shape is laid at the start pose, so each kit joint value is (relative angle - start angle), in radians.
_E_REL = S0["elbow"] - np.array(SHOULDER)  # elbow axis from the shoulder axis
_P_REL = S0["pivot"] - S0["elbow"]  # swivel pivot from the elbow axis
_N0 = S0["normal"]  # 130 deg at the start
_LINK1 = [
    rod((0.0, 0.0, 0.0), _p3(_E_REL), 0.009, BLACK, sides=16),  # 18 mm tube
    cyl(_p3(_E_REL, 0.013), 0.010, 0.052, BLACK, sides=16),  # elbow knuckle, 20 mm, up to the elbow hinge
]
_LINK2 = [
    rod((0.0, 0.0, 0.0), _p3(_P_REL), 0.009, BLACK, sides=16),
    cyl(_p3(_P_REL), 0.011, 0.018, BLACK, sides=16),  # swivel boss, 18 mm tall: clears the base knuckle by 4 mm when folded
]
_NU = _u(_N0)
_HEAD = [
    rod((0.0, 0.0, 0.0), _p3(_NU * (NECK - BEZEL_T / 2)), 0.007, BLACK, sides=12),  # neck, pivot to the bezel's back
    cyl(_p3(_NU * NECK), HEAD_R, BEZEL_T, BLACK, axis="x", tilt=_zq(_N0), sides=32),  # bezel, matt black brass
    cyl(_p3(_NU * NECK), GLASS_R, GLASS_T, GLASS, axis="x", tilt=_zq(_N0)),  # glass, both faces
    tube(_p3(_NU * NECK), GLASS_R, 0.0705, 0.0258, CHROME, axis="x", tilt=_zq(_N0)),  # chrome trim ring
]
_HEAD[2] = mechkit.dataclasses.replace(_HEAD[2], collide=False)
_HEAD[3] = mechkit.dataclasses.replace(_HEAD[3], collide=False)
_BASE = [
    cyl((TILE_X - 0.0075 - SHOULDER[0], 0.0, 0.0), 0.040, 0.015, BLACK, axis="x", sides=24),  # wall plate, 80 mm, on the tile
    box((0.012, 0.0, 0.0), (0.016, 0.020, 0.026), BLACK),  # bracket, plate to knuckle
    cyl((0.0, 0.0, 0.0), 0.011, 0.026, BLACK, sides=16),  # shoulder knuckle
]


def _kit(name: str, deg: float) -> float:
    return math.radians(deg - START[("shoulder", "elbow", "swivel").index(name)])


MIRROR = Mech(
    "mirror", pos=(float(SHOULDER[0]), float(SHOULDER[1]), float(Z_SHOULDER)), mass=None, friction=0.8, shapes=_BASE,
    joints=[
        Joint("shoulder", "hinge", axis="z", at=(0.0, 0.0, 0.0), low=_kit("shoulder", STOPS["shoulder"][0]),
              high=_kit("shoulder", STOPS["shoulder"][1]), mass=0.09, hold=HOLDS["shoulder"], damping=0.02,
              shapes=_LINK1),
        Joint("elbow", "hinge", axis="z", at=_p3(_E_REL, Z_ELBOW - Z_SHOULDER), low=_kit("elbow", STOPS["elbow"][0]),
              high=_kit("elbow", STOPS["elbow"][1]), mass=0.08, hold=HOLDS["elbow"], damping=0.02, parent="shoulder",
              shapes=_LINK2),
        Joint("swivel", "hinge", axis="z", at=_p3(_P_REL), low=_kit("swivel", STOPS["swivel"][0]),
              high=_kit("swivel", STOPS["swivel"][1]), mass=0.45, hold=HOLDS["swivel"], damping=0.02, parent="elbow",
              shapes=_HEAD),
    ],
)

# The wall: a white glazed tile panel 440 x 240 x 20 mm standing on the counter, grout lines every 100 mm (look only).
_GROUT = [box((-0.0101, y, 0.0), (0.0004, 0.002, TILE_H), GROUT) for y in (-0.2, -0.1, 0.1, 0.2)]
_GROUT += [box((-0.0101, 0.0, z - TILE_H / 2), (0.0004, 0.44, 0.002), GROUT) for z in (0.10, 0.20)]
WALL = Mech("tile_wall", pos=(float(TILE_X + 0.010), 0.0, float(TOP + TILE_H / 2)), mass=None, friction=0.6,
            shapes=[box((0.0, 0.0, 0.0), (0.020, 0.44, TILE_H), TILE),
                    *[mechkit.dataclasses.replace(g, collide=False) for g in _GROUT]])

OBJECTS = [WALL, MIRROR]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
FOLD_DIST = 0.045  # elbow axis and swivel pivot within 45 mm of the tile
PIVOT_ALONG = 0.030  # swivel pivot within 30 mm of the plate centre (y 0) along the wall
GLASS_BAND = (0.062, 0.092)  # glass front, 60 mm either side of centre, this far from the tile
GLASS_SKEW = 0.015  # the two points differ by 15 mm or less (7 deg)
SIDE = 0.060
TOUCH = 0.003  # any head point within 3 mm of the tile: the glass struck the wall
CLEAR = 0.050  # every robot point over 50 mm from the head
STILL_DEG = 0.25  # each hinge turns less than this in 0.5 s (0.5 deg/s)
END_STEPS = 60  # the end conditions hold for the last 3 s
_ST: dict = {}


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, touched=False, hist=[], end=0, step=0)


_reset()


def head_pose(s) -> dict:
    """World pose of the mirror's parts from its joint values: elbow axis and swivel pivot (x, y), head centre, the
    glass normal (unit, horizontal) and the head's rotation."""
    b = mechkit.body_poses(MIRROR, s)
    re, oe = b["elbow"]
    rs, os_ = b["swivel"]
    n = rs @ np.array([_NU[0], _NU[1], 0.0])
    return {"elbow": np.asarray(oe, float), "pivot": np.asarray(os_, float), "normal": n,
            "centre": np.asarray(os_, float) + NECK * n, "rot": rs}


def head_front_x(h) -> float:
    """Largest x reached by any point of the head (bezel with its glass, and the neck)."""
    n, c, p = h["normal"], h["centre"], h["pivot"]
    disc = c[0] + HALF_T * abs(n[0]) + HEAD_R * math.sqrt(max(0.0, 1.0 - n[0] * n[0] - n[2] * n[2]))
    neck_end = p + (NECK - BEZEL_T / 2) * n
    neck = max(p[0], neck_end[0]) + 0.007
    return max(disc, neck)


def glass_points(h) -> tuple[float, float]:
    """Distances from the tile (m) of the glass front 60 mm either side of centre on the horizontal diameter. The front
    is the face toward the robot (away from the tile), either side of the two-sided head."""
    n = h["normal"].copy()
    if n[0] > 0:
        n = -n
    side = np.array([-n[1], n[0], 0.0])
    side /= max(1e-9, float(np.linalg.norm(side)))
    front = h["centre"] + HALF_T * n
    return tuple(float(TILE_X - (front + k * SIDE * side)[0]) for k in (-1.0, 1.0))


def folded(h) -> bool:
    return bool(TILE_X - h["elbow"][0] <= FOLD_DIST and TILE_X - h["pivot"][0] <= FOLD_DIST
                and abs(h["pivot"][1] - SHOULDER[1]) <= PIVOT_ALONG)


def parallel(h) -> bool:
    a, b = glass_points(h)
    return bool(GLASS_BAND[0] <= a <= GLASS_BAND[1] and GLASS_BAND[0] <= b <= GLASS_BAND[1] and abs(a - b) <= GLASS_SKEW)


def _seg_points(a, b, step=0.02):
    n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
    return [a + (b - a) * i / n for i in range(n + 1)]


def _robot_points():
    """Sample points on both arms (every 20 mm between link origins, plus the finger pads), relative to the env origin.
    None offline (no simulator: the check is made in Isaac), "error" if the simulator could not be read."""
    env = mechkit._ENV.get("env")
    if env is None:
        return None
    try:
        base = env.unwrapped
        origin = mechkit._to_numpy(base.scene.env_origins)[0]
        pts = []
        for arm in ("left_robot", "right_robot"):
            robot = base.scene[arm]
            poses = mechkit._to_numpy(robot.data.body_link_pose_w)[0][:, :3] - origin
            for a, b in zip(poses[:-1], poses[1:]):
                pts += _seg_points(a, b)
        pts += [np.asarray(p, float) for p in mechkit.finger_points(env)]
        return pts
    except Exception:  # noqa: BLE001  an unread robot never counts as clear
        return "error"


def head_clearance(h, pts) -> float:
    """Least distance (m) from any of ``pts`` to the head's bezel (a disc of radius 75 mm, 26.2 mm thick)."""
    best = math.inf
    for p in pts:
        d = np.asarray(p, float) - h["centre"]
        a = float(d @ h["normal"])
        r = float(np.linalg.norm(d - a * h["normal"]))
        da, dr = abs(a) - HALF_T, r - HEAD_R
        dist = math.hypot(max(da, 0.0), max(dr, 0.0)) if (da > 0 or dr > 0) else max(da, dr)
        best = min(best, dist)
    return best


def _hands_clear(h) -> bool:
    pts = _robot_points()
    if pts is None:  # offline replays cannot see the robot: release and clearance are checked in the simulator
        return True
    if isinstance(pts, str):
        return False
    return bool(mechkit.released(MIRROR_STATE[0], clearance=0.03)) and head_clearance(h, pts) > CLEAR


MIRROR_STATE: list = [None]


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    s = objects["mirror"]
    MIRROR_STATE[0] = s
    h = head_pose(s)
    if TILE_X - head_front_x(h) <= TOUCH:
        _ST["touched"] = True
    q = [math.degrees(float(s.joints.get(n, 0.0))) for n in ("shoulder", "elbow", "swivel")]
    _ST["hist"].append(q)
    del _ST["hist"][:-11]
    still = len(_ST["hist"]) < 11 or all(abs(a - b) < STILL_DEG for a, b in zip(_ST["hist"][-1], _ST["hist"][0]))
    _ST["end"] = _ST["end"] + 1 if still and _hands_clear(h) else 0
    _ST["h"] = h


def score(objects):
    """1.0 folded (F) and parallel (P); 0.5 folded only; 0 otherwise. 0 if any point of the head ever came within 3 mm
    of the tile, or unless for the last 3 s the mirror was released, every robot point stayed over 50 mm from the head
    and every hinge turned under 0.5 deg/s."""
    _update(objects)
    if _ST["touched"] or _ST["end"] < END_STEPS:
        return 0.0
    h = _ST["h"]
    if not folded(h):
        return 0.0
    return 1.0 if parallel(h) else 0.5


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps its own parts off the tile: no arm part may enter the tile panel (1 cm margin in front of its face).
ARM_KEEP_OUT = [
    {"name": "tile wall, 1 cm in front of its face", "x": (TILE_X - 0.010, TILE_X + 0.020), "y": (-0.23, 0.23),
     "z_top": TOP + TILE_H},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (right arm only; the left arm holds READY). It reads the mirror's hinge angles from the simulator, and
# from them the head's pose, to correct its path.
demo = placeholder_demo
TCP_Z = TOP + 0.150  # grasp: TCP 20 mm below the top of the bezel's rim, pads across the 25 mm bezel
HOVER_Z = TOP + 0.190
OPEN45 = -0.0225  # one finger joint for a 45 mm pad gap
SQUARE_S, SWING_S, FOLD_S = 3.0, 11.0, 3.5  # seconds for each leg
SWING_PATH = [tuple(S0["pivot"]), (0.212, -0.150), (0.205, -0.100), (0.203, -0.050), (0.210, -0.005),
              tuple(chain(FOLDED[0], 140.0, 0.0)["pivot"])]  # swivel pivot path; the swing ends at elbow +140
FOLD_A2 = (140.0, FOLDED[1])  # last fold: elbow 140 -> 178 about the elbow, shoulder on its -90 stop
SLIDE_TO = (0.262, -0.095, TCP_Z)  # open fingers slide off along the glass plane before rising
RISE_TO = (0.240, -0.120, TOP + 0.185)
CLEAR_TO = (0.250, -0.220, TOP + 0.180)


def _catmull(pts, s: float) -> np.ndarray:
    """Centripetal-free uniform Catmull-Rom through ``pts`` at s in 0..1."""
    p = [np.asarray(x, float) for x in pts]
    p = [2 * p[0] - p[1], *p, 2 * p[-1] - p[-2]]
    n = len(pts) - 1
    t = min(max(s, 0.0), 1.0) * n
    i = min(int(t), n - 1)
    u = t - i
    a, b, c, d = p[i], p[i + 1], p[i + 2], p[i + 3]
    return 0.5 * ((2 * b) + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u * u + (-a + 3 * b - 3 * c + d) * u ** 3)


def swing_pose(s: float) -> tuple[np.ndarray, float]:
    """Head centre and glass normal (deg) along the swing, glass kept facing the robot."""
    pv = _catmull(SWING_PATH, s)
    return pv + NECK * _u(180.0), 180.0


def square_pose(s: float) -> tuple[np.ndarray, float]:
    """Head centre and normal while the glass turns square (+50 deg about the swivel), the links held."""
    n = _N0 + (180.0 - _N0) * s
    return S0["pivot"] + NECK * _u(n), n


def fold_pose(s: float, elbow=None) -> tuple[np.ndarray, float]:
    """Last fold: an arc about the elbow axis (measured, when given), elbow 140 -> 178, glass facing the robot."""
    e = np.array(chain(FOLDED[0], 0.0, 0.0)["elbow"]) if elbow is None else np.asarray(elbow, float)[:2]
    a2 = FOLD_A2[0] + (FOLD_A2[1] - FOLD_A2[0]) * s
    pv = e + L2 * _u(180.0 + FOLDED[0] + a2)
    return pv + NECK * _u(180.0), 180.0


def grip_for(normal_deg: float):
    from closedloop import grip_rot
    return grip_rot(yaw=math.radians(normal_deg - 90.0))  # pads close along the glass normal


def _smooth(i: int, n: int) -> float:
    x = i / n
    return x * x * (3 - 2 * x)


class _Track:
    """Moves the hand so the head (not the hand) follows the planned path. In a live run the head centre measured from
    the mirror's hinge angles is compared with the plan; the error is integrated into an offset on the hand command
    (cancelling arm sag and slip in the grip), and the path waits while the head lags more than 8 mm. Offline (dry
    runs, where the mirror never moves) the hand follows the plan directly."""

    def __init__(self, ctx):
        self.ctx, self.R = ctx, ctx.right
        self.off = np.zeros(2)
        self.lag_mm = 0.0
        self.paused = 0

    def measured(self):
        return head_pose(self.ctx.obj("mirror"))

    def step(self, centre, normal_deg, gain=0.15):
        centre = np.asarray(centre, float)[:2]
        if not self.R.offline:
            h = self.measured()
            err = centre - h["centre"][:2]
            self.lag_mm = float(np.linalg.norm(err)) * 1000.0
            self.off = self.off + gain * err
            n = float(np.linalg.norm(self.off))
            if n > 0.015:
                self.off *= 0.015 / n
        cmd = centre + self.off
        yield from self.R.move((float(cmd[0]), float(cmd[1]), TCP_Z), CONTROL_DT, rot=grip_for(normal_deg))

    def follow(self, pose_fn, duration, max_pause=2.0):
        n = max(1, round(duration / CONTROL_DT))
        i, paused = 0, 0
        while i < n:
            if self.lag_mm > 8.0 and paused < max_pause / CONTROL_DT:  # the head lags: hold the path and let it catch up
                paused += 1
                c, nd = pose_fn(_smooth(i, n))
            else:
                i += 1
                c, nd = pose_fn(_smooth(i, n))
            yield from self.step(c, nd)
        self.paused += paused


def _joints_deg(ctx) -> tuple[float, float, float]:
    j = ctx.obj("mirror").joints
    return tuple(math.degrees(float(j.get(n, 0.0))) + START[k] for k, n in enumerate(("shoulder", "elbow", "swivel")))


def demo_script(ctx):
    R = ctx.right
    c0 = S0["centre"]
    hover = (float(c0[0]), float(c0[1]), HOVER_Z)
    grasp = (float(c0[0]), float(c0[1]), TCP_Z)
    rot0 = grip_for(_N0)
    # 1) over the head, pads across the bezel, then down and close
    yield from R.move(hover, 3.0, OPEN45, rot=rot0)
    yield from R.trim_to_measured(hover)
    for attempt in range(2):
        yield from R.move(grasp, 2.0, OPEN45, rot=rot0)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.020):  # pads stopped on the 26.2 mm bezel and glass
            break
        yield from R.grip(OPEN45, 0.6)
        yield from R.move(hover, 1.5, rot=rot0)
    T = _Track(ctx)
    # 2) square the glass: turn the head +50 deg about the swivel, links held
    yield from T.follow(square_pose, SQUARE_S)
    # 3) swing across in front of the wall plate, elbow folding the way it is bent, glass kept parallel
    yield from T.follow(swing_pose, SWING_S)
    # the shoulder must sit on its -90 stop before the last fold (live only; offline the mirror does not move)
    if not R.offline:
        for k in range(3):
            if _joints_deg(ctx)[0] <= FOLDED[0] + 2.0:
                break
            a1 = FOLDED[0] - 1.5 * (k + 1)
            target = chain(a1, 140.0, 0.0)["pivot"] + NECK * _u(180.0)
            yield from T.follow(lambda s, t=target, c=swing_pose(1.0)[0]: (c + (t - c) * s, 180.0), 0.8)
    # 4) last fold about the measured elbow axis
    elbow = None if R.offline else head_pose(ctx.obj("mirror"))["elbow"]
    yield from T.follow(lambda s: fold_pose(s, elbow), FOLD_S)
    yield from R.hold(0.5)
    # 5) let go, slide the open pads off along the glass plane, rise and park
    yield from R.grip(OPEN45, 1.0)
    yield from R.hold(1.0)
    rot = grip_for(180.0)
    yield from R.move(SLIDE_TO, 2.5, rot=rot)
    yield from R.move(RISE_TO, 1.5, rot=rot)
    yield from R.move(CLEAR_TO, 2.0, rot=grip_for(90.0))  # back toward the robot, fingers along y, before READY
    yield from R.home(4.0)
    yield from R.hold(4.0)

