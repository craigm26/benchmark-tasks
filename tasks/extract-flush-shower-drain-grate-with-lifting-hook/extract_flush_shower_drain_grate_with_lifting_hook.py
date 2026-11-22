"""On the station table: a tiled floor-drain deck on the right, a 6 inch square stainless shower drain (Elegante
KD01A113-6) set flush in it, with its slotted grate lying level with the tile and no rim to pinch. On the deck's right
strip a charcoal silicone holder stands the drain's lifting hook upright (an 88 mm stainless hook with a T-handle). To the left
of the deck lies a charcoal silicone service mat. The grate and the hook are loose.

Requires -E object_jitter=0: the grate starts flush in its recess and the hook starts standing in its holder.

Run it:  inspect-robots run --embodiment yam_arena -E task=extract_flush_shower_drain_grate_with_lifting_hook.py \\
           -E object_jitter=0 --instruction "Use the lifting hook to pull the flush stainless shower drain grate
           out, lay it flat on the mat, and put the hook back in its holder." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402,F401
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, mass_props, on_reset, released, tube  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to open, withdraw and park after the first 1.0

TILE, GROUT, WELL = (0.86, 0.90, 0.92), (0.30, 0.31, 0.33), (0.10, 0.11, 0.13)
FRAME, GRATE_C, HOOK_C, GRIP_C = (0.50, 0.52, 0.55), (0.88, 0.90, 0.93), (0.90, 0.92, 0.95), (0.95, 0.50, 0.08)
CHARCOAL = (0.20, 0.22, 0.25)

# ---- the floor drain (fixed): 176 x 216 x 40 mm tile deck, 152 mm square 304 stainless frame, 142 mm recess ---------------
DRAIN_XY = (0.250, -0.106)  # centre of the drain
DECK_X, DECK_Y, DECK_H = (-0.088, 0.088), (-0.132, 0.084), 0.040  # deck extents from the drain centre
FRAME_OUT, FRAME_IN, WELL_IN = 0.076, 0.071, 0.061  # half sizes: frame, recess, well
LEDGE_TOP, WELL_FLOOR = 0.025, 0.004  # the grate sits on a 10 mm ledge 15 mm below the tile top


def _slab(x0, x1, y0, y1, z0, z1, color, collide=True):
    s = box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)
    return s if collide else dataclasses.replace(s, collide=False)


_W, _O = FRAME_OUT, FRAME_IN
_cx, _cy = DRAIN_XY
FLOOR_SHAPES = [
    # tile around the frame
    _slab(DECK_X[0], -_W, DECK_Y[0], DECK_Y[1], 0, DECK_H, TILE),
    _slab(_W, DECK_X[1], DECK_Y[0], DECK_Y[1], 0, DECK_H, TILE),
    _slab(-_W, _W, _W, DECK_Y[1], 0, DECK_H, TILE),
    _slab(-_W, _W, DECK_Y[0], -_W, 0, DECK_H, TILE),
    # frame ledge (10 mm wide ring of stainless 25 mm tall) and the 5 mm frame wall above it
    _slab(-_W, -WELL_IN, -_W, _W, 0, LEDGE_TOP, FRAME), _slab(WELL_IN, _W, -_W, _W, 0, LEDGE_TOP, FRAME),
    _slab(-WELL_IN, WELL_IN, -_W, -WELL_IN, 0, LEDGE_TOP, FRAME), _slab(-WELL_IN, WELL_IN, WELL_IN, _W, 0, LEDGE_TOP, FRAME),
    _slab(-_W, -_O, -_W, _W, LEDGE_TOP, DECK_H, FRAME), _slab(_O, _W, -_W, _W, LEDGE_TOP, DECK_H, FRAME),
    _slab(-_O, _O, -_W, -_O, LEDGE_TOP, DECK_H, FRAME), _slab(-_O, _O, _O, _W, LEDGE_TOP, DECK_H, FRAME),
    _slab(-WELL_IN, WELL_IN, -WELL_IN, WELL_IN, 0, WELL_FLOOR, WELL),  # the dark well
    # the 2 in hair strainer basket (look only)
    dataclasses.replace(tube((0, 0, WELL_FLOOR + 0.006), 0.024, 0.028, 0.012, FRAME), collide=False),
    # grout lines (look only)
    _slab(DECK_X[0], DECK_X[1], -0.0245, -0.0215, DECK_H, DECK_H + 0.0004, GROUT, False),
    _slab(-0.0020, 0.0020, DECK_Y[0], DECK_Y[1], DECK_H, DECK_H + 0.0004, GROUT, False),
]
FLOOR = Mech("floor_drain", pos=(*DRAIN_XY, TOP), shapes=FLOOR_SHAPES, friction=1.0)

# ---- the service mat (fixed): 176 x 154 x 10 mm charcoal silicone --------------------------------------------------------
MAT_XY, MAT_SIZE = (0.250, 0.065), (0.176, 0.154, 0.010)
MAT = Mech("service_mat", pos=(*MAT_XY, TOP), friction=1.0, shapes=[box((0, 0, MAT_SIZE[2] / 2), MAT_SIZE, CHARCOAL)])
MAT_TOP = TOP + MAT_SIZE[2]

# ---- the grate (free, 0.24 kg): 136 x 136 x 15 mm, 11 box colliders; origin at the middle of its underside ----------------
G_HALF, G_H, G_RIB_T, G_SKIRT = 0.068, 0.015, 0.005, 0.007
G_RIBS = ((0.0, 0.012), (-0.024, 0.010), (0.024, 0.010), (-0.0465, 0.009), (0.0465, 0.009))  # (y centre, width)
G_BARS = (-0.021, 0.021)  # cross-bar x centres, 6 mm wide
GRATE_SHAPES = [
    box((-(G_HALF - G_SKIRT / 2), 0, G_H / 2), (G_SKIRT, 2 * G_HALF, G_H), GRATE_C),
    box((G_HALF - G_SKIRT / 2, 0, G_H / 2), (G_SKIRT, 2 * G_HALF, G_H), GRATE_C),
    box((0, -(G_HALF - G_SKIRT / 2), G_H / 2), (2 * (G_HALF - G_SKIRT), G_SKIRT, G_H), GRATE_C),
    box((0, G_HALF - G_SKIRT / 2, G_H / 2), (2 * (G_HALF - G_SKIRT), G_SKIRT, G_H), GRATE_C),
    *[box((0, y, G_H - G_RIB_T / 2), (2 * (G_HALF - G_SKIRT), w, G_RIB_T), GRATE_C) for y, w in G_RIBS],
    *[box((x, 0, G_H - G_RIB_T / 2), (0.006, 2 * (G_HALF - G_SKIRT), G_RIB_T), GRATE_C) for x in G_BARS],
]
GRATE_POS = (*DRAIN_XY, TOP + LEDGE_TOP)
GRATE = Mech("drain_grate", pos=GRATE_POS, mass=0.24, friction=0.6, shapes=GRATE_SHAPES)
SLOT_Y = 0.0125  # the central slot beside the middle rib, from the drain centre: 36 x 13 mm, y 6..19 mm
SLOT_LEN, SLOT_W = 0.036, 0.013
RIB_UNDER = TOP + LEDGE_TOP + G_H - G_RIB_T  # the ribs' underside while the grate sits in its recess

# ---- the hook holder (fixed): a 60 x 44 x 34 mm charcoal silicone block with a 30 x 13 mm slot 30 mm deep ---------------------
HOLDER_XY = (0.250, -0.210)  # centre of the slot
H_BLOCK, H_FLOOR_T, H_SLOT = (0.060, 0.044, 0.034), 0.004, (0.030, 0.013)
SLOT_FLOOR = TOP + DECK_H + H_FLOOR_T
_hx, _hy = H_SLOT[0] / 2, H_SLOT[1] / 2
HOLDER = Mech("hook_holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP + DECK_H), friction=1.0, shapes=[
    box((0, 0, H_FLOOR_T / 2), (H_BLOCK[0], H_BLOCK[1], H_FLOOR_T), CHARCOAL),
    box((0, -(_hy + (H_BLOCK[1] / 2 - _hy) / 2), H_BLOCK[2] / 2), (H_BLOCK[0], H_BLOCK[1] / 2 - _hy, H_BLOCK[2]), CHARCOAL),
    box((0, _hy + (H_BLOCK[1] / 2 - _hy) / 2, H_BLOCK[2] / 2), (H_BLOCK[0], H_BLOCK[1] / 2 - _hy, H_BLOCK[2]), CHARCOAL),
    box((-(_hx + (H_BLOCK[0] / 2 - _hx) / 2), 0, H_BLOCK[2] / 2), (H_BLOCK[0] / 2 - _hx, H_SLOT[1], H_BLOCK[2]), CHARCOAL),
    box((_hx + (H_BLOCK[0] / 2 - _hx) / 2, 0, H_BLOCK[2] / 2), (H_BLOCK[0] / 2 - _hx, H_SLOT[1], H_BLOCK[2]), CHARCOAL),
])
RIM_Z = TOP + DECK_H + H_BLOCK[2]  # top of the holder's walls

# ---- the lifting hook (free, 55 g): foot, round shaft and T-handle; hook frame u (foot), v (crossbar), w (shaft) ----------
FOOT_U, FOOT_V, FOOT_W = (-0.003, 0.019), 0.007, 0.004  # foot: 22 long, 7 wide, 4 thick, joined to the shaft's end
SHAFT_R, SHAFT_L = 0.003, 0.074
HANDLE = (0.022, 0.030, 0.014)  # handle: u 22, v 30 (crossbar), w 14, from w 74 to 88
HOOK_LEN = SHAFT_L + HANDLE[2]
HANDLE_C = SHAFT_L + HANDLE[2] / 2  # the handle's centre along the shaft: where the pads close
HOOK_MASS = 0.055


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat(r):
    """(w, x, y, z) of a rotation matrix."""
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return (s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    w = (r[k, j] - r[j, k]) / s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return (w, *q)


def _hook_boxes():
    """(centre, size, colour) of the hook's two boxes in the hook frame."""
    return [((0.5 * (FOOT_U[0] + FOOT_U[1]), 0.0, FOOT_W / 2), (FOOT_U[1] - FOOT_U[0], FOOT_V, FOOT_W), HOOK_C),
            ((0.0, 0.0, HANDLE_C), HANDLE, GRIP_C)]


FOOT_PTS = [np.array([u, v, w]) for u in FOOT_U for v in (-FOOT_V / 2, FOOT_V / 2) for w in (0.0, FOOT_W)]
FOOT_MARGIN = 0.011  # the foot's lowest corner nearest the robot starts 11 mm in from the slot's end wall (slot half 15)


def _rest(theta):
    """Hook leaning toward the robot by theta about y, its foot's lowest corner on the slot floor, 11 mm in from the
    slot's near end. Returns (rotation, origin, signed gap from the shaft to the near wall's top edge)."""
    r = _ry(-theta)
    low = min(float((r @ p)[2]) for p in FOOT_PTS)
    minx = min(float((r @ p)[0]) for p in FOOT_PTS)
    o = np.array([HOLDER_XY[0] - FOOT_MARGIN - minx, HOLDER_XY[1], SLOT_FLOOR + 0.0002 - low])
    axis = r[:, 2]
    n = np.array([axis[2], -axis[0]])
    n /= np.linalg.norm(n)
    q = np.array([HOLDER_XY[0] - _hx, RIM_Z])
    dist = abs(float(n @ (q - np.array([o[0], o[2]]))))
    return r, o, dist - SHAFT_R


def _solve_lean():
    lo, hi = 0.0, math.radians(25)
    for _ in range(60):
        mid = (lo + hi) / 2
        if _rest(mid)[2] > 0.0003:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


HOOK_LEAN = _solve_lean()  # about 8 degrees: the shaft rests on the near wall's top edge, the foot on the floor
R_REST, _o, _ = _rest(HOOK_LEAN)
HOOK_POS = tuple(float(v) for v in _o)
_Q = _quat(R_REST)
HOOK_SHAPES = [box(R_REST @ np.array(c), s, col, tilt=_Q) for c, s, col in _hook_boxes()]
HOOK_SHAPES.append(cyl(R_REST @ np.array([0, 0, SHAFT_L / 2]), SHAFT_R, SHAFT_L, HOOK_C, axis="z", tilt=_Q, sides=12))
HOOK = Mech("lifting_hook", pos=HOOK_POS, mass=HOOK_MASS, friction=0.8, shapes=HOOK_SHAPES)

OBJECTS = [FLOOR, MAT, HOLDER, GRATE, HOOK]

# ---- scoring -------------------------------------------------------------------------------------------------------------
TILT_FAIL_DEG, TILT_FAIL_STEPS = 35.0, 3  # a grate turned more than 35 degrees from upright for 0.15 s: V1
AREA_X, AREA_Y = (0.10, 0.42), 0.28  # the work area: grate or hook outside it for 0.15 s: V2
LIFT_CLEAR = 0.010  # lifted clear: the grate's underside 10 mm above the tile top
LIFT_TILT_DEG, LIFT_STEPS = 25.0, 5  # level within 25 degrees for 0.25 s
FLAT_TILT_DEG, MAT_TOL, MAT_CENTRE = 5.0, 0.003, 0.025  # on the mat: within 5 degrees, 3 mm, 25 mm of its centre
HOOK_CLEAR = 0.050  # the hook at least 50 mm clear of the grate
HOLDER_TOL, HOLDER_TILT_DEG = 0.012, 40.0
RELEASE_CLEAR = 0.03
STILL_SPEED, STILL_SPIN, REST_STEPS = 0.01, 0.5, 40  # under 1 cm/s and 0.5 rad/s for 2 s
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it for a held part)
SCORE_LOG = True

_L = {"V1": 0, "V2": 0, "V3": False, "lift": 0, "L": False, "still": 0, "seen": None, "step": 0, "said": None, "lines": 0,
      "tripped": set()}


@on_reset
def _clear(_env=None):
    _L.update(V1=0, V2=0, V3=False, lift=0, L=False, still=0, seen=None, step=0, said=None, lines=0, tripped=set())


def _say(text):
    if SCORE_LOG and _L["lines"] < 80:
        _L["lines"] += 1
        print(f"SCORE step {_L['step']}: {text}", flush=True)


def tilt_of(state):
    """Degrees between the part's own up axis and the vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(state.rotation()[2, 2])))))


def hook_frame(state):
    """The hook's frame (columns u, v, w) in the world, and its origin (the foot-axis point)."""
    return state.rotation() @ R_REST, np.asarray(state.origin, dtype=float)


def hook_axis(state):
    """Unit vector along the hook's shaft from the foot toward the handle."""
    return hook_frame(state)[0][:, 2]


def in_grate_footprint(g, point, margin=0.0):
    local = g.to_local(point)
    return abs(local[0]) <= G_HALF + margin and abs(local[1]) <= G_HALF + margin


def hooked(g, h):
    """The hook is on the grate: its foot-axis point is inside the grate's outline, below the grate's top face."""
    foot = hook_frame(h)[1]
    local = g.to_local(foot)
    return bool(abs(local[0]) <= G_HALF and abs(local[1]) <= G_HALF and -0.02 <= local[2] <= G_H + 0.005)


def _outside_area(s):
    return bool(s.low[0] < AREA_X[0] or s.high[0] > AREA_X[1] or s.low[1] < -AREA_Y or s.high[1] > AREA_Y)


def _gap_xy(a, b):
    """Plan-view distance between the bounding boxes of two states (0 when they overlap)."""
    dx = max(a.low[0] - b.high[0], b.low[0] - a.high[0], 0.0)
    dy = max(a.low[1] - b.high[1], b.low[1] - a.high[1], 0.0)
    return float(math.hypot(dx, dy))


def _on_mat(g):
    centre = np.asarray(g.origin, dtype=float)
    inside = (MAT_XY[0] - MAT_SIZE[0] / 2 <= g.low[0] and g.high[0] <= MAT_XY[0] + MAT_SIZE[0] / 2
              and MAT_XY[1] - MAT_SIZE[1] / 2 <= g.low[1] and g.high[1] <= MAT_XY[1] + MAT_SIZE[1] / 2)
    return bool(tilt_of(g) <= FLAT_TILT_DEG and abs(centre[2] - MAT_TOP) <= MAT_TOL and inside
                and math.hypot(centre[0] - MAT_XY[0], centre[1] - MAT_XY[1]) <= MAT_CENTRE)


def _in_holder(h):
    """The hook stands in its holder: foot-axis point inside the slot, within 12 mm of the slot floor, shaft within 40
    degrees of upright."""
    r, o = hook_frame(h)
    return bool(abs(o[0] - HOLDER_XY[0]) <= _hx and abs(o[1] - HOLDER_XY[1]) <= 0.005 and o[2] <= SLOT_FLOOR + HOLDER_TOL
                and r[2, 2] >= math.cos(math.radians(HOLDER_TILT_DEG)))


def _fingers_clear(state):
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(state, RELEASE_CLEAR)


def _trip(name, why):
    if name not in _L["tripped"]:
        _L["tripped"].add(name)
        _say(f"{name} latched: {why}")


def _update(objects):
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    _L["step"] += 1
    g, h = objects["drain_grate"], objects["lifting_hook"]
    tg = tilt_of(g)
    _L["V1"] = _L["V1"] + 1 if tg > TILT_FAIL_DEG else 0
    if _L["V1"] >= TILT_FAIL_STEPS:
        _trip("V1", f"grate {tg:.0f} deg from upright")
    off = _outside_area(g) or _outside_area(h) or float(g.origin[2]) < TOP + 0.005
    _L["V2"] = _L["V2"] + 1 if off else 0
    if _L["V2"] >= 3:
        _trip("V2", "the grate or the hook left the work area or the grate lies on the bare table")
    if float(g.low[2]) >= TOP + DECK_H + LIFT_CLEAR and not _L["L"]:
        if not hooked(g, h):
            _L["V3"] = True
            _trip("V3", "the grate was lifted clear without the hook under it")
        _L["lift"] = _L["lift"] + 1 if tg <= LIFT_TILT_DEG else 0
        if _L["lift"] >= LIFT_STEPS:
            _L["L"] = True
            _say("L latched: grate lifted clear and level with the hook under it")
    elif not _L["L"]:
        _L["lift"] = 0
    calm = (np.linalg.norm(g.lin_vel) < STILL_SPEED and np.linalg.norm(g.ang_vel) < STILL_SPIN
            and np.linalg.norm(h.lin_vel) < STILL_SPEED and np.linalg.norm(h.ang_vel) < STILL_SPIN)
    _L["still"] = _L["still"] + 1 if calm else 0


def score(objects):
    """0 if the grate turned more than 35 degrees from upright for 0.15 s (V1), the grate or the hook left the work area or
    the grate lay on the bare table (V2), or the grate was lifted clear without the hook under it (V3). Otherwise, once
    everything is still for 2 s with the fingers clear: 1.0 grate flat on the mat and the hook standing in its holder;
    0.75 grate on the mat and the hook 50 mm or more clear of it, not in the holder; 0.50 grate on the mat, hook not
    yet clear (these three need the hook lift L); 0.4 L but the grate is not on the mat; else 0."""
    _update(objects)
    g, h = objects["drain_grate"], objects["lifting_hook"]
    still = _L["still"] >= REST_STEPS
    clear_g, clear_h = _fingers_clear(g), _fingers_clear(h)
    on_mat = _on_mat(g)
    held = _in_holder(h)
    h_clear = _gap_xy(g, h) >= HOOK_CLEAR
    if _L["V1"] >= TILT_FAIL_STEPS or _L["V2"] >= 3 or _L["V3"] or "V1" in _L["tripped"] or "V2" in _L["tripped"]:
        value = 0.0
    elif not (still and clear_g):
        value = 0.0
    elif on_mat and _L["L"]:
        if h_clear and clear_h:
            value = 1.0 if held else 0.75
        else:
            value = 0.5
    elif _L["L"]:
        value = 0.4
    else:
        value = 0.0
    gates = (value, tuple(sorted(_L["tripped"])), still, clear_g, clear_h, on_mat, held, h_clear, _L["L"])
    if gates != _L["said"]:
        _L["said"] = gates
        _say(f"score {value:g}: tripped {sorted(_L['tripped'])}, still {still} (counter {_L['still']}), fingers clear "
             f"{clear_g}/{clear_h}, on mat {on_mat}, hook in holder {held}, hook clear {h_clear}, L {_L['L']}")
    return value


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps out of the drain housing ----------------------------------------------------------------------------
# No arm part may be inside the recessed drain housing (frame, recess and well, 4.4 cm above the table) at any time: the
# hook's foot goes in through the grate's slot while the hand stays above. The envelope's keep-out box is the same box.
HOUSING_X, HOUSING_Y, HOUSING_TOP = (0.174, 0.326), (-0.182, -0.030), TOP + 0.044
ARM_KEEP_OUT = [{"name": "recessed drain housing", "x": HOUSING_X, "y": HOUSING_Y, "z_top": HOUSING_TOP}]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
# The right arm takes the hook out of its holder with a straight-down pinch on the T-handle, stands it vertical, feeds the
# foot down through the grate's slot beside the middle rib, turns it a quarter turn under the rib, slides it under the
# grate's centre, lifts the grate level, carries it to the mat, sets it down, lowers the hook 3 mm so the foot floats in
# the cavity under the rib, slides and turns it back, draws it out through the slot, and stands it in its holder.
# The left arm has no plan: it holds its ready pose.
demo = placeholder_demo
DOWN = grip_rot()
HAND = np.array([0.0, 0.0, HANDLE_C])  # the pads close on the handle's centre, on the shaft's axis
OPEN_GAP, CLOSE_CMD, HOLD_GAP, WIDE_GAP = 0.046, 0.022, 0.034, 0.070  # handle crossbar is 30 mm across
SLOT_X0 = DRAIN_XY[0]
INS_DX = -0.006  # the foot goes in 6 mm toward the robot from the slot's centre so its 19 mm reach clears the slot's end
INS_Y = DRAIN_XY[1] - SLOT_Y  # the slot beside the middle rib on the robot's side (grate y -19..-6 mm)
INS_Z = TOP + LEDGE_TOP - 0.0015  # foot's underside 1.5 mm under the grate's underside, i.e. 7.5 mm below the ribs' underside
FOOT_UNDER = G_H - G_RIB_T + 0.0015  # ... see INS_Z: the foot top is FOOT_W above INS_Z
LIFT_UP = 0.040  # lift from the insertion depth: 7.5 mm to take up the slack, the rest raises the grate
MAT_GRATE = np.array([MAT_XY[0], MAT_XY[1]])
SETTLED_SPEED, SETTLED_STEPS, SETTLE_WAIT_S = 0.005, 30, 8.0


def _finger(gap):
    """Finger joint target for a pad gap in metres (94 mm is fully open)."""
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def _rotvec(r):
    """Small-angle rotation vector of a rotation matrix."""
    return 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def demo_script(ctx):
    from yam_arena import ik

    R = ctx.right
    live = not R.offline
    st = {"o": np.array(HOOK_POS), "R": R_REST.copy(), "fix": np.zeros(3), "rot": np.zeros(3)}

    def hook_now():
        if live:
            return hook_frame(ctx.obj("lifting_hook"))[::-1]
        return np.array(HOOK_POS), R_REST

    def grate_origin():
        return np.asarray(ctx.obj("drain_grate").origin, dtype=float) if live else np.array(GRATE_POS)

    def hand(o, rh, fix=0.0, rot=None):
        tcp = np.asarray(o, dtype=float) + rh @ HAND + fix
        rh_hand = rh @ DOWN
        if rot is not None and float(np.linalg.norm(rot)) > 1e-9:
            rh_hand = axis_angle_matrix(rot / (np.linalg.norm(rot) + 1e-12), float(np.linalg.norm(rot))) @ rh_hand
        return tcp, rh_hand

    def carry(o1, rh1, secs, gain=0.3, track=True):
        """Move the held hook so its foot-axis point goes to o1 and its frame to rh1, correcting the hand every step from the
        hook's pose as the simulator reports it (the arm sags under load and the pads give a little)."""
        o0, r0 = st["o"].copy(), st["R"].copy()
        o1 = np.asarray(o1, dtype=float)
        n = max(1, round(secs * 20))
        for i in range(1, n + 1):
            s = ik.smoothstep(i / n)
            want_o, want_r = o0 + s * (o1 - o0), ik.slerp_matrix(r0, rh1, s)
            if live and track:
                om, rm = hook_now()
                st["fix"] = np.clip(st["fix"] + gain * np.clip(want_o - om, -0.004, 0.004), -0.03, 0.03)
                st["rot"] = np.clip(st["rot"] + 0.2 * np.clip(_rotvec(want_r @ rm.T), -0.02, 0.02), -0.25, 0.25)
            tcp, rot = hand(want_o, want_r, st["fix"], st["rot"])
            yield from R.move(tcp, 0.05, rot=rot)
        st["o"], st["R"] = o1, np.asarray(rh1, dtype=float)

    def secs_for(a, b, speed=0.04, floor=1.0):
        return max(floor, float(np.linalg.norm(np.asarray(b) - np.asarray(a))) / speed)

    # 1) Take the hook: over the handle, down on it with the pads open 46 mm, close on its 30 mm crossbar.
    o_h, r_h = hook_now()
    grab = o_h + r_h @ HAND
    rot_pick = r_h @ DOWN
    yield from R.move(grab + np.array([0.0, 0.0, 0.07]), 4.0, _finger(OPEN_GAP), rot=rot_pick)
    yield from R.trim_to_measured(grab + np.array([0.0, 0.0, 0.07]))
    off = R.pos - (grab + np.array([0.0, 0.0, 0.07]))
    yield from R.move(grab + off, 2.5, rot=rot_pick)
    yield from R.grip(_finger(CLOSE_CMD), 0.8)
    yield from R.hold(0.3)
    if not R.holding(HOLD_GAP):
        yield from R.grip(_finger(OPEN_GAP), 0.6)
        yield from R.move(grab + off + np.array([0.0, 0.0, 0.004]), 0.6, rot=rot_pick)
        yield from R.grip(_finger(CLOSE_CMD), 0.8)
        yield from R.hold(0.3)
    o_h, r_h = hook_now()
    st["o"], st["R"] = o_h.copy(), r_h.copy()
    st["fix"] = (R.pos - (o_h + r_h @ HAND)) if live else np.zeros(3)

    # 2) Lift it clear of the holder, stand it upright over the slot, let the feedback settle.
    above = np.array([SLOT_X0 + INS_DX, INS_Y, TOP + 0.095])
    clear = st["o"] + np.array([0.0, 0.0, 0.065])
    yield from carry(clear, r_h, 3.0)
    yield from carry(above, np.eye(3), secs_for(clear, above, 0.03, 4.0))
    yield from carry(above, np.eye(3), 1.5)

    # 3) Foot down through the slot, a quarter turn under the middle rib, 6 mm along to the grate's middle.
    ins = np.array([SLOT_X0 + INS_DX, INS_Y, INS_Z])
    yield from carry(ins, np.eye(3), 3.0)
    yield from carry(ins, np.eye(3), 0.8)
    yield from carry(ins, _rz(math.pi / 2), 2.5)
    slid = ins + np.array([-INS_DX, 0.0, 0.0])
    yield from carry(slid, _rz(math.pi / 2), 1.2)
    yield from carry(slid, _rz(math.pi / 2), 0.6)

    # 4) Lift: the slack, then the grate, level.
    lifted = slid + np.array([0.0, 0.0, LIFT_UP])
    yield from carry(lifted, _rz(math.pi / 2), 3.5)
    yield from carry(lifted, _rz(math.pi / 2), 1.0)

    # 5) Carry to the mat and let the grate down on it.
    over = np.array([MAT_GRATE[0], MAT_GRATE[1] - SLOT_Y, lifted[2]])
    yield from carry(over, _rz(math.pi / 2), secs_for(lifted, over, 0.03, 4.0))
    yield from carry(over, _rz(math.pi / 2), 1.0)
    seat_z = TOP + MAT_SIZE[2] - FOOT_UNDER + G_H - G_RIB_T - FOOT_W - 0.0015  # nominal: foot top just under the ribs on the mat
    seat = np.array([over[0], over[1], seat_z + 0.0015])
    top_out = over[2] - seat[2]
    yield from carry(over + np.array([0.0, 0.0, -top_out + 0.012]), _rz(math.pi / 2), secs_for([0, 0, 0], [0, 0, top_out - 0.012], 0.025, 2.0))
    if live:  # slow the last 12 mm until the grate itself reports it is down
        z = st["o"][2]
        for _ in range(80):
            if float(ctx.obj("drain_grate").origin[2]) <= MAT_TOP + 0.0006:
                break
            z -= 0.0004
            yield from carry([st["o"][0], st["o"][1], z], _rz(math.pi / 2), 0.05)
        down = np.array([st["o"][0], st["o"][1], st["o"][2] - 0.0030])
    else:
        down = np.array([over[0], over[1], seat[2] - 0.0030])
    yield from carry(down, _rz(math.pi / 2), 1.5)  # the foot floats 3 mm under the ribs and 3 mm above the mat
    yield from carry(down, _rz(math.pi / 2), 0.8)

    # 6) Slide back 6 mm, turn the foot back along the slot, draw the hook straight out.
    back = down + np.array([INS_DX, 0.0, 0.0])
    yield from carry(back, _rz(math.pi / 2), 1.2)
    yield from carry(back, np.eye(3), 2.5)
    yield from carry(back, np.eye(3), 0.6)
    out = np.array([back[0], back[1], TOP + 0.062])
    yield from carry(out, np.eye(3), secs_for(back, out, 0.03, 4.0))

    # 7) Back to the holder: over the slot, upright, down into it, tipped to lean on its wall, let go.
    mid = np.array([HOOK_POS[0], -0.14, TOP + 0.062])
    hx = np.array([HOOK_POS[0], HOLDER_XY[1], TOP + 0.095])
    yield from carry(mid, np.eye(3), secs_for(out, mid, 0.04, 5.0))
    yield from carry(hx, np.eye(3), secs_for(mid, hx, 0.04, 3.0))
    yield from carry(hx, np.eye(3), 1.0)
    pre = np.array([HOOK_POS[0], HOOK_POS[1], HOOK_POS[2] + 0.030])
    yield from carry(pre, np.eye(3), 3.0)
    yield from carry(np.array(HOOK_POS) + np.array([0.0, 0.0, 0.004]), R_REST, 2.0, track=False)
    yield from R.hold(0.5)
    yield from R.grip(_finger(WIDE_GAP), 1.5)
    yield from R.hold(0.8)
    up = R.pos + np.array([0.0, 0.0, 0.12])
    yield from R.move(up, 2.5, _finger(WIDE_GAP), rot=R.rot.copy())
    yield from R.home(6.0)
    calm = 0
    for _ in range(round(SETTLE_WAIT_S * 20)):
        if live:
            speeds = [float(np.linalg.norm(ctx.obj(n).lin_vel)) for n in ("drain_grate", "lifting_hook")]
            calm = calm + 1 if max(speeds) < SETTLED_SPEED else 0
            if calm >= SETTLED_STEPS:
                break
        yield from ctx.wait(0.05)
    yield from ctx.wait(SUCCESS_HOLD_S + 1.0)
