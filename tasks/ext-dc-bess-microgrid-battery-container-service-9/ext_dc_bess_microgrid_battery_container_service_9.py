"""Reset a tripped data hall emergency power off (EPO) station. A yellow Schneider Harmony XALK178E station (one red
40 mm XB4BS8445-type turn-to-release mushroom head) is screwed to a grey wall panel standing on the table. The head was
pressed and is latched in, 4.3 mm deeper than its reset position. Reset it the way the arrows on the head show: grip
the head and turn it clockwise (seen from the front) past 20 deg, when the latch lets go and the head's spring lifts it
4.3 mm out; then let go so it stays out. Pressing it back in re-trips it, and pulling a latched head does not reset it.

Requires -E object_jitter=0: the station is fixed and the demo plans from its fixed position.

Run it:  inspect-robots run --embodiment yam_arena -E task=ext_dc_bess_microgrid_battery_container_service_9.py \\
           -E object_jitter=0 --instruction "The EPO station has tripped. Reset it so it is ready for power to be
           restored." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
import belimits  # noqa: E402,F401
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, arc, box, cyl, hull, lathe, on_reset, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

RED = (0.80, 0.06, 0.05)
YELLOW = (0.96, 0.74, 0.05)      # RAL 1021 cover
LIGHT_GREY = (0.78, 0.79, 0.78)  # RAL 7035 base
CHROME = (0.80, 0.81, 0.83)
WALL_GREY = (0.45, 0.47, 0.50)
DARK = (0.18, 0.18, 0.20)
WHITE = (0.95, 0.95, 0.95)

# ---------------------------------------------------------------- the station (fixed) ------------------------------
# World frame: x forward (away from the robot), y left, z up, m. The head's axis runs along x at y 0, z ZB; the
# station's front (cover) face is the plane x = FACE. Mech frame: origin (FACE, 0, TOP), axes as the world's.
ZB = TOP + 0.140
FACE = 0.315
BOX_W, BOX_D, BOX_R = 0.068, 0.054, 0.006    # 68 x 68 mm face, 54 mm deep, 6 mm corner radius
COVER_D = 0.034                               # yellow cover; the last 20 mm is the light grey base
WALL_T, WALL_W, WALL_H = 0.012, 0.300, 0.220
BEZEL_RI, BEZEL_RO, BEZEL_L = 0.0125, 0.015, 0.008   # chrome XB4 bezel ring, 8 mm proud of the cover
NECK_R = 0.0105                                       # plunger neck, 21 mm: 2 mm per side in the bezel bore
SKIRT_R, SKIRT_L = 0.020, 0.012                       # 40 mm mushroom: 12 mm cylindrical skirt
DOME_L, DOME_R = 0.008, 0.014                         # domed front, 40 -> 28 mm over 8 mm
SKIRT_BASE = 0.0132                                   # skirt base proud of the cover face, latched (17.5 mm reset)
TRAVEL = 0.0043                                       # XB4 total travel 4.3 mm (datasheet)
HEAD_K = 800.0                                        # N/m return spring (modelled)
HEAD_REST_FREE = -0.012                               # rest once unlatched: 6.2 N at the reset stop, 9.6 N latched
RELEASE_AT = 0.35                                     # rad (20 deg) of turn at which the latch lets go
KNOB_HIGH = 0.70                                      # rad (40 deg) turn stop
KNOB_K = 0.15                                         # N m/rad turn-back spring: 0.05 N m at the release angle

Zs = ZB - TOP
L = lambda x: x - FACE  # noqa: E731  world x to Mech x


def _rounded_box(x0, x1, color):
    """A 68 x 68 mm square with 6 mm round corners, from Mech x0 to x1 (a convex hull of the two outlines)."""
    pts = []
    h = BOX_W / 2 - BOX_R
    for cy, cz, a0 in ((h, h, 0.0), (-h, h, 90.0), (-h, -h, 180.0), (h, -h, 270.0)):
        for k in range(4):
            a = math.radians(a0 + 30.0 * k)
            for x in (x0, x1):
                pts.append((x, cy + BOX_R * math.cos(a), Zs + cz + BOX_R * math.sin(a)))
    return hull((0, 0, 0), pts, color)


def _station_shapes():
    wall_x = BOX_D + WALL_T / 2
    return [
        _rounded_box(0.0, COVER_D, YELLOW),                                           # yellow polycarbonate cover
        _rounded_box(COVER_D, BOX_D, LIGHT_GREY),                                     # light grey base
        tube((-BEZEL_L / 2, 0, Zs), BEZEL_RI, BEZEL_RO, BEZEL_L, CHROME, axis="x"),   # chrome bezel ring
        box((wall_x, 0, WALL_H / 2), (WALL_T, WALL_W, WALL_H), WALL_GREY),            # wall panel
        box((BOX_D + WALL_T + 0.050, 0.110, 0.006), (0.100, 0.040, 0.012), DARK),     # two feet behind the wall
        box((BOX_D + WALL_T + 0.050, -0.110, 0.006), (0.100, 0.040, 0.012), DARK),
    ]


def _neck_shapes():
    """The plunger neck, on the slide: from 0.5 mm in front of the cover face to the skirt base (latched)."""
    return [cyl((-(0.0005 + SKIRT_BASE) / 2, 0, 0), NECK_R, SKIRT_BASE - 0.0005, DARK, axis="x")]


def _arrow(a0, a1, r=0.0095):
    """A white arrow on the head's flat front: an arc from a0 to a1 (rad about +x, so clockwise seen from the front)
    and a head at a1."""
    x = -(SKIRT_BASE + SKIRT_L + DOME_L) - 0.0004
    shaft = arc((x, 0, 0), r, a0, a1, WHITE, section=(0.0018, 0.0008), axis="x")
    c, s = math.cos(a1), math.sin(a1)
    tang = np.array([-s, c])          # direction of increasing angle in (y, z)
    rad = np.array([c, s])
    tip = (r * rad) + 0.004 * tang
    pts = []
    for dx in (-0.0004, 0.0004):
        for p in (r * rad + 0.0028 * rad, r * rad - 0.0028 * rad, tip):
            pts.append((x + dx, float(p[0]), float(p[1])))
    return [shaft, hull((0, 0, 0), pts, WHITE)]


def _head_shapes():
    """The red mushroom (on the turning joint): skirt and dome, plus the two white arrows."""
    x_skirt = -(SKIRT_BASE + SKIRT_L / 2)
    x_front = -(SKIRT_BASE + SKIRT_L + DOME_L)
    prof = [(DOME_R, 0.0), (0.0165, 0.0012), (0.0183, 0.0030), (0.0195, 0.0055), (SKIRT_R, DOME_L)]
    return [
        cyl((x_skirt, 0, 0), SKIRT_R, SKIRT_L, RED, axis="x"),
        lathe((x_front, 0, 0), prof, RED, axis="x"),
        *_arrow(math.radians(20), math.radians(130)),
        *_arrow(math.radians(200), math.radians(310)),
    ]


STATION = Mech(
    "stn", pos=(FACE, 0.0, TOP), mass=None, friction=0.6,
    shapes=_station_shapes(),
    joints=[
        Joint("head", "slide", axis="x", at=(0.0, 0.0, Zs), low=-TRAVEL, high=0.0, spring=HEAD_K, damping=4.0,
              mass=0.005, shapes=_neck_shapes()),
        Joint("knob", "hinge", axis="x", at=(0.0, 0.0, 0.0), low=0.0, high=KNOB_HIGH, spring=KNOB_K, damping=0.01,
              armature=5e-4, mass=0.025, parent="head", friction=0.9, shapes=_head_shapes()),
    ],
)
OBJECTS = [STATION]

# The head's region in the Mech frame, for the release check: the fingers must be 3 cm from it.
HEAD_LO = (-(SKIRT_BASE + SKIRT_L + DOME_L) - TRAVEL, -SKIRT_R, Zs - SKIRT_R)
HEAD_HI = (-SKIRT_BASE + 0.001, SKIRT_R, Zs + SKIRT_R)

_L: dict = {}


@on_reset
def _latch_reset(env):
    """Start latched: the head's spring has no preload (the latch carries it in the real station)."""
    _L.clear()
    if env is not None:
        try:
            mechkit.set_rest(env, "stn", "head", 0.0)
        except Exception:  # noqa: BLE001 - a fresh scene already rests at 0
            pass


def _latch_update(env, head: float, knob: float) -> None:
    """The modelled latch: turning the head 20 deg frees it (the spring's rest moves 12 mm out, so it pushes the head
    to its 4.3 mm stop); pressing a freed head fully home latches it again."""
    if not _L.get("free") and knob >= RELEASE_AT:
        mechkit.set_rest(env, "stn", "head", HEAD_REST_FREE)
        _L["free"] = True
    elif _L.get("free") and head >= -0.0003 and knob < RELEASE_AT:
        mechkit.set_rest(env, "stn", "head", 0.0)
        _L["free"] = False
        _L["relatched"] = True


@mechkit.on_step
def _latch(env):
    if env is None:
        return
    j = mechkit._mech_state(env, STATION).joints
    _latch_update(env, float(j["head"]), float(j["knob"]))


# The station and wall: no arm part may enter them (the fingers work in front of the cover).
ARM_KEEP_OUT = [
    {"name": "station box, wall panel and feet", "x": (FACE - 0.003, FACE + 0.170), "y": (-0.150, 0.150),
     "z_top": TOP + 0.25},
]

# ---------------------------------------------------------------- scoring -------------------------------------------
TURNED = RELEASE_AT          # rad: turned far enough to free the latch
UP = -(TRAVEL - 0.0005)      # head at least 3.8 mm out (within 0.5 mm of its reset stop)
PRESSED = -(TRAVEL - 0.0015)  # after it was up, pushed back in by more than 1.5 mm
PULLED = -0.0020             # a latched head pulled out by more than 2 mm
HOLD_UP_STEPS = 60           # 3 s at 20 Hz
FLAG_STEPS = 10              # 0.5 s


def _sustained(name: str, cond: bool, steps: int) -> bool:
    """True once cond has held on `steps` distinct control steps in a row (score() runs twice per step)."""
    k = mechkit.step_count()
    last, run = _L.get(name + "_k"), _L.get(name + "_n", 0)
    if not cond:
        _L[name + "_n"] = 0
    elif last != k:
        _L[name + "_n"] = run + 1
    _L[name + "_k"] = k
    return _L.get(name + "_n", 0) >= steps


def _station_moved(stn) -> bool:
    """The station's origin more than 2 mm from where it was screwed down (it is a fixed body, so this is a guard)."""
    o = np.asarray(getattr(stn, "origin", None) if getattr(stn, "origin", None) is not None else stn.pos, dtype=float)
    if not np.any(o):
        return False
    return bool(np.linalg.norm(o[:2] - np.asarray(STATION.pos)[:2]) > 0.002)


def score(objects):
    """0.25: head turned >= 20 deg (latch freed). 0.60: and the head >= 3.8 mm out, held for the last 3 s. 1.00: and
    both grippers' pads >= 3 cm from the head. 0 if, after the head came up, it was pushed back in by more than 1.5 mm
    for 0.5 s (re-tripped), if a latched head was pulled out by more than 2 mm for 0.5 s, or if the station moved."""
    stn = objects["stn"]
    head, knob = float(stn.joints["head"]), float(stn.joints["knob"])
    if knob >= TURNED:
        _L["turned"] = True
    if head <= UP:
        _L["was_up"] = True
    if _L.get("was_up") and _sustained("press", head > PRESSED, FLAG_STEPS):
        _L["pressed"] = True
    if not _L.get("turned") and _sustained("pull", head < PULLED, FLAG_STEPS):
        _L["pulled"] = True
    up_held = _sustained("up", head <= UP, HOLD_UP_STEPS)
    if _L.get("pressed") or _L.get("pulled") or _station_moved(stn):
        return 0.0
    if not _L.get("turned"):
        return 0.0
    if not up_held:
        return 0.25
    if mechkit.released(stn, clearance=0.03, lo=HEAD_LO, hi=HEAD_HI):
        return 1.0
    return 0.60


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- scripted demo -------------------------------------
demo = placeholder_demo
PI2 = math.pi / 2
GRIP_X = FACE - SKIRT_BASE - 0.008       # tool point 8 mm in front of the skirt base: fingertips 8 mm off the cover
TURN = 0.55                              # wrist roll, rad (32 deg): past the 20 deg release with slip to spare
V = 0.06


def _mv(arm, p, finger=None, v=V, **kw):
    d = float(np.linalg.norm(np.asarray(p, dtype=float) - arm.pos))
    yield from arm.move(p, max(0.4, d / v), finger, **kw)


def _roll_to(arm, p, r0, r1, seconds):
    n = max(1, round(seconds / arm.dt))
    for i in range(1, n + 1):
        yield from arm.move(p, arm.dt, pitch=PI2, roll=r0 + (r1 - r0) * i / n)


def _head(ctx) -> float:
    return float(ctx.obj("stn").joints["head"])


def _grasp(R, x):
    pre = (x - 0.050, 0.0, ZB)
    yield from _mv(R, pre, pitch=PI2)
    yield from R.trim_to_measured(pre)
    yield from _mv(R, (x, 0.0, ZB), v=0.02, pitch=PI2)
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from R.hold(0.3)


def demo_script(ctx):
    """Right arm: level grip pointing at the wall, over the head's skirt; turn the wrist 32 deg clockwise (seen from
    the front) with no push; when the head comes up, open the hand; back straight off and go home. The left arm stays
    at READY."""
    R = ctx.right
    yield from _mv(R, (0.20, -0.12, ZB + 0.06), pitch=PI2, v=0.05)
    yield from _grasp(R, GRIP_X)
    if not R.holding(0.030):                 # missed the skirt: open, back off, try once more
        yield from R.grip(FINGER_OPEN, 0.5)
        yield from _mv(R, (GRIP_X - 0.03, 0.0, ZB), pitch=PI2)
        yield from _grasp(R, GRIP_X)
    grip = R.pos.copy()
    roll = TURN
    yield from _roll_to(R, grip, 0.0, roll, 2.5)
    for _ in range(10):                      # wait up to 0.5 s for the latch to let go
        if _head(ctx) <= -0.001:
            break
        yield from R.hold(0.05)
    if _head(ctx) > -0.001:                  # not freed yet (slip in the grip): turn on to near the stop
        yield from _roll_to(R, grip, roll, 0.68, 1.0)
        roll = 0.68
    yield from R.grip(FINGER_OPEN, 0.6)      # let the spring lift the head; the hand does not follow it
    yield from R.hold(0.8)
    yield from _mv(R, (GRIP_X - 0.080, 0.0, ZB), pitch=PI2, roll=roll, v=0.04)
    yield from _roll_to(R, R.pos.copy(), roll, 0.0, 1.0)
    yield from _mv(R, (0.20, -0.15, ZB + 0.08), pitch=PI2, v=0.05)
    yield from R.home(4.0)
    yield from R.hold(1.0)
