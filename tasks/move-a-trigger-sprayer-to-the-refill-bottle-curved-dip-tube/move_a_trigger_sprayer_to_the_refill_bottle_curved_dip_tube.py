"""Move a trigger sprayer to the refill bottle: steer its curved dip tube down the neck, land the collar square.

On a charcoal drip tray on the station table stand two 16 oz (473 mL) PET spray bottles: an empty one (pale grey,
right, nearer the right arm) with an unscrewed trigger sprayer sitting on its neck, and a full one (pale blue, centre).
The sprayer's 184 mm LDPE dip tube keeps the curve it was made with: straight for 100 mm under the collar, then bent
toward +x on a 170 mm radius, so its tip sits 20 mm off the sprayer's axis. The robot draws the sprayer out of the
empty bottle, carries it over the full one, aims the off-axis tip into the 20 mm neck, leads the head back over the
neck as the curve passes through, and leaves the 28 mm collar sitting square on the neck finish, released.

Requires -E object_jitter=0 (the layout is fixed; ropes are never jittered).

Run it:  inspect-robots run --embodiment yam_arena -E task=move_a_trigger_sprayer_to_the_refill_bottle_curved_dip_tube.py
           -E object_jitter=0 --instruction "Move the sprayer onto the full bottle: steer its curved dip tube all the
           way down the neck, then sit the collar on the neck." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
import ropekit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, hull, ring_profile, sweep, tube  # noqa: E402,F401
from ropekit import EndBody, Plastic, Rope  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0

CHARCOAL = (0.18, 0.18, 0.19)
PALE_BLUE = (0.72, 0.86, 0.96)
PALE_GREY = (0.86, 0.87, 0.88)
WHITE = (0.95, 0.95, 0.93)
RED = (0.85, 0.10, 0.10)
ORANGE = (1.00, 0.45, 0.02)

# -- drip tray (fixed): 20 x 24 cm, 4 mm floor, 5 mm x 12 mm rims -------------------------------------------------------
TRAY_XY = (0.255, -0.095)
TRAY_W, TRAY_D, TRAY_FLOOR, RIM_T, RIM_H = 0.20, 0.24, 0.004, 0.005, 0.012
TT = TOP + TRAY_FLOOR  # tray floor top, where the bottles stand
TRAY = Mech(
    "drip_tray", pos=(*TRAY_XY, TOP), mass=None,
    shapes=[
        box((0, 0, TRAY_FLOOR / 2), (TRAY_W, TRAY_D, TRAY_FLOOR), CHARCOAL),
        box((TRAY_W / 2 - RIM_T / 2, 0, TRAY_FLOOR + RIM_H / 2), (RIM_T, TRAY_D, RIM_H), CHARCOAL),
        box((-TRAY_W / 2 + RIM_T / 2, 0, TRAY_FLOOR + RIM_H / 2), (RIM_T, TRAY_D, RIM_H), CHARCOAL),
        box((0, TRAY_D / 2 - RIM_T / 2, TRAY_FLOOR + RIM_H / 2), (TRAY_W - 2 * RIM_T, RIM_T, RIM_H), CHARCOAL),
        box((0, -TRAY_D / 2 + RIM_T / 2, TRAY_FLOOR + RIM_H / 2), (TRAY_W - 2 * RIM_T, RIM_T, RIM_H), CHARCOAL),
    ],
)

# -- the two bottles: 16 oz Boston-round-shouldered PET, 70 mm body, 28/400 neck finish --------------------------------
SIDES = 16
LAND = 0.190  # neck finish top (the land) above the bottle's base
NECK_R_IN, NECK_R_OUT = 0.0100, 0.0125  # 20 mm bore, 25 mm finish root
BEAD_R, BEAD_Z = 0.0165, (0.177, 0.180)  # support bead (neck ring) under the thread
BODY_R_IN, BODY_R_OUT = 0.033, 0.035
FLOOR = 0.003
BOTTLE_ROWS = [
    (BODY_R_IN, BODY_R_OUT, FLOOR),
    (BODY_R_IN, BODY_R_OUT, 0.135),
    (0.0105, NECK_R_OUT, 0.165),  # shoulder
    (0.0105, NECK_R_OUT, 0.176),
    (0.0105, BEAD_R, BEAD_Z[0]),
    (0.0105, BEAD_R, BEAD_Z[1]),
    (NECK_R_IN, NECK_R_OUT, 0.181),
    (NECK_R_IN, NECK_R_OUT, LAND - 0.001),
    (NECK_R_IN + 0.001, NECK_R_OUT, LAND),  # 1 mm entry chamfer on the bore
]


def _thread(color):
    """Look-only 28/400 thread: one turn of a 1.6 mm bead at r 14 mm rising 4.2 mm (6 TPI), z 182 to 186 mm."""
    pts = [(0.0135 * math.cos(a), 0.0135 * math.sin(a), 0.182 + 0.0042 * a / (2 * math.pi))
           for a in np.linspace(0.0, 2 * math.pi, 29)]
    s = sweep((0, 0, 0), pts, 0.0008, color, sides=6)
    import dataclasses
    return dataclasses.replace(s, collide=False)


def bottle(name, xy, mass, color):
    return Mech(name, pos=(*xy, TT), mass=mass, friction=0.6,
                shapes=[ring_profile((0, 0, 0), BOTTLE_ROWS, color, sides=SIDES, floor=FLOOR), _thread(WHITE)])


FULL_XY = (0.270, -0.035)
EMPTY_XY = (0.240, -0.165)
FULL = bottle("full_bottle", FULL_XY, 0.50, PALE_BLUE)  # 473 mL of water plus a 28 g bottle, rounded
EMPTY = bottle("empty_bottle", EMPTY_XY, 0.035, PALE_GREY)


def bottle_r_in(z):
    """Inner radius of a bottle at height z above its base (m), from BOTTLE_ROWS (inf above the land)."""
    if z >= LAND:
        return math.inf
    rows = BOTTLE_ROWS
    for (r0, _, z0), (r1, _, z1) in zip(rows, rows[1:]):
        if z0 <= z <= z1:
            return r0 + (r1 - r0) * (z - z0) / (z1 - z0)
    return rows[0][0] if z < rows[0][2] else rows[-1][0]


# -- the trigger sprayer: rigid start body of the dip tube ---------------------------------------------------------------
# Sprayer frame: origin on the seal face (the closure's inside top, which lands on the neck finish), z up, x toward the
# nozzle. The rope's start frame has x up, y = world -y, z = world +x for a tube hanging straight down, so a sprayer
# point (sx, sy, sz) is (sz, -sy, sx) in it.


def P(sx, sy, sz):
    return (sz, -sy, sx)


def _hullS(points, color):
    return hull((0, 0, 0), [P(*p) for p in points], color)


def _rect(x0, x1, w, z, c=0.004):
    """Chamfered rectangle (sprayer frame) at height z, x0..x1, width w across y."""
    h = w / 2
    return [(x0 + c, -h, z), (x1 - c, -h, z), (x1, -h + c, z), (x1, h - c, z), (x1 - c, h, z), (x0 + c, h, z),
            (x0, h - c, z), (x0, -h + c, z)]


COLLAR_R_IN, COLLAR_R_OUT, COLLAR_DEPTH, COLLAR_LEAD = 0.0145, 0.0165, 0.0085, 0.0012
STEM_W = 0.030  # across the pads (y)
SPRAYER_SHAPES = (
    # closure: skirt with an entry chamfer at its open bottom, and the solid top whose underside is the seal face
    ring_profile((0, 0, 0), [(COLLAR_R_IN + COLLAR_LEAD, COLLAR_R_OUT, -COLLAR_DEPTH),
                             (COLLAR_R_IN, COLLAR_R_OUT, -COLLAR_DEPTH + COLLAR_LEAD),
                             (COLLAR_R_IN, COLLAR_R_OUT, 0.0)], WHITE, axis="x", sides=16),
    cyl(P(0, 0, 0.002), COLLAR_R_OUT, 0.004, WHITE, axis="x", sides=16),
    _hullS(_rect(-0.013, 0.013, STEM_W, 0.004) + _rect(-0.012, 0.012, STEM_W - 0.002, 0.050), RED),  # stem
    _hullS(_rect(-0.022, 0.030, 0.034, 0.050) + _rect(-0.020, 0.035, 0.030, 0.080), RED),  # shroud
    _hullS(_rect(0.020, 0.032, 0.018, 0.020, 0.003) + _rect(0.020, 0.032, 0.018, 0.050, 0.003), RED),  # trigger
    cyl(P(0.045, 0, 0.066), 0.008, 0.020, WHITE, axis="z", sides=12),  # nozzle, lying along sprayer x
)
SPRAYER_MASS = 0.040
SPRAYER = EndBody(SPRAYER_SHAPES, mass=SPRAYER_MASS, friction=0.8)

# -- the dip tube: 6 mm OD LDPE, 184 mm, factory curve ------------------------------------------------------------------
TUBE_LEN, TUBE_STRAIGHT, TUBE_BEND_R, TUBE_R = 0.184, 0.100, 0.170, 0.003
TUBE_ARC = (TUBE_LEN - TUBE_STRAIGHT) / TUBE_BEND_R  # 0.494 rad, 28.3 deg
TIP_OFF = TUBE_BEND_R * (1 - math.cos(TUBE_ARC))  # 20.4 mm off the axis
TIP_DEPTH = TUBE_STRAIGHT + TUBE_BEND_R * math.sin(TUBE_ARC)  # 180.6 mm below the seal face
SEAT_GAP = 0.0003  # the seal face starts 0.3 mm over the empty bottle's land


def tube_path(seal):
    x, y, z = seal
    pts = [(x, y, z), (x, y, z - TUBE_STRAIGHT)]
    for a in np.linspace(0.0, TUBE_ARC + 0.08, 24)[1:]:
        pts.append((x + TUBE_BEND_R * (1 - math.cos(a)), y, z - TUBE_STRAIGHT - TUBE_BEND_R * math.sin(a)))
    return pts


def tube_offset(s):
    """(x, z) of the laid tube's axis at arclength s (m) from the seal face, relative to the seal centre."""
    if s <= TUBE_STRAIGHT:
        return 0.0, -s
    a = (s - TUBE_STRAIGHT) / TUBE_BEND_R
    return TUBE_BEND_R * (1 - math.cos(a)), -TUBE_STRAIGHT - TUBE_BEND_R * math.sin(a)


SEAL0 = (EMPTY_XY[0], EMPTY_XY[1], TT + LAND + SEAT_GAP)
LDPE = Plastic(yield_angle=0.27, stiffness=0.64, damping=0.64 * 0.002, segment=TUBE_LEN / 9,
               label="LDPE 6 x 4 mm tube, E 0.25 GPa")
DIP_TUBE = Rope("dip_tube", path=tube_path(SEAL0), length=TUBE_LEN, segment=TUBE_LEN / 9, radius=TUBE_R,
                rope_mass=0.0027, collision_sides=8, plastic=LDPE, start_body=SPRAYER, color=ORANGE)

OBJECTS = [TRAY, FULL, EMPTY, DIP_TUBE]

# Sprayer plus tube centre of mass, sprayer frame (x toward the nozzle). The pads are centred on it in x.
_v, _com_end, _ = mechkit.mass_props(SPRAYER_SHAPES, SPRAYER_MASS)
_tube_x = float(np.mean([tube_offset(s)[0] for s in np.linspace(0, TUBE_LEN, 50)]))
COM_X = (SPRAYER_MASS * float(_com_end[2]) + 0.0027 * _tube_x) / (SPRAYER_MASS + 0.0027)
GRIP_Z = 0.032  # tool point above the seal face, on the stem (stem z 4 to 50 mm)

# Nothing in the scene is off limits to the arms: the task's declared limit is the tool-point speed (0.12 m/s),
# enforced by the gate envelope; sweep.py has no region to check.
ARM_KEEP_OUT = []
SPEED_LIMIT = 0.12  # m/s, tool points

# -- scoring --------------------------------------------------------------------------------------------------------------
DEPTH_FULL = 0.160  # tip at least 160 mm below the land
DEPTH_PART = 0.060
SEAT_LOW, SEAT_HIGH = -0.001, 0.006  # seal face from the land (sim rests at 0; a real collar on the thread start 3-5 mm)
SEAT_AXIS = 0.0025
SEAT_TILT = 5.0
KINK_DEG = 10.0
WALL_TOL = 0.002
TIP_LIM, MOVE_LIM, LIFT_LIM = 15.0, 0.030, 0.010
LATCH_STEPS = 10  # 0.5 s at 20 Hz
STILL = 0.01  # m/s
RELEASE = 0.030


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _origin(s, spec):
    o = getattr(s, "origin", None)
    return np.asarray(o if o is not None else spec.pos, dtype=float)


def _bottle_fault(s, spec):
    """Tilted over 15 deg, moved over 30 mm, or lifted over 10 mm from where it stands."""
    o = _origin(s, spec)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, _rot(s.quat_xyzw)[2, 2]))))
    moved = math.hypot(o[0] - spec.pos[0], o[1] - spec.pos[1])
    return tilt > TIP_LIM or moved > MOVE_LIM or o[2] - spec.pos[2] > LIFT_LIM


_LATCH = {"last": None, "runs": {"full_bottle": 0, "empty_bottle": 0}, "zero": False}


@mechkit.on_reset
def _reset(env=None):
    _LATCH.update(last=None, runs={"full_bottle": 0, "empty_bottle": 0}, zero=False)


def _update_latches(objects):
    if objects is _LATCH["last"]:  # success() and score() share one dict per step
        return
    _LATCH["last"] = objects
    for spec in (FULL, EMPTY):
        if spec.name in objects and _bottle_fault(objects[spec.name], spec):
            _LATCH["runs"][spec.name] += 1
            if _LATCH["runs"][spec.name] >= LATCH_STEPS:
                _LATCH["zero"] = True
        else:
            _LATCH["runs"][spec.name] = 0


def _kinked(t):
    sb = getattr(t, "set_bends", None)
    if sb is None or len(sb) == 0:
        return False
    return float(np.degrees(np.linalg.norm(np.asarray(sb, dtype=float), axis=-1)).max()) > KINK_DEG


def _tube_points(t):
    """Segment ends along the tube's measured axis, seal face to tip."""
    pts = np.asarray(t.points, dtype=float)
    return np.vstack([t.tip("start"), (pts[:-1] + pts[1:]) / 2, pts, t.tip("end")])


def evaluate(objects):
    """The scored facts at this step, for score() and for check.py."""
    f, t = objects["full_bottle"], objects["dip_tube"]
    R, o = _rot(f.quat_xyzw), _origin(f, FULL)
    land = o + R @ np.array([0.0, 0.0, LAND])

    def local(p):
        return R.T @ (np.asarray(p, dtype=float) - o)

    tip_depth = float(land[2] - t.tip("end")[2])
    inside = True
    for p in _tube_points(t):
        q = local(p)
        if q[2] < FLOOR - WALL_TOL or math.hypot(q[0], q[1]) > bottle_r_in(q[2]) - TUBE_R + WALL_TOL:
            inside = False
            break
    seal = local(t.tip("start"))
    axis_off = math.hypot(seal[0], seal[1])
    up = R.T @ np.asarray(t.end_axis("start"), dtype=float)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2]) / max(1e-9, float(np.linalg.norm(up)))))))
    seat_h = float(seal[2] - LAND)
    seated = SEAT_LOW <= seat_h <= SEAT_HIGH and axis_off <= SEAT_AXIS and tilt <= SEAT_TILT
    resting = -0.002 <= seat_h <= 0.040 and axis_off <= 0.020
    still = float(np.linalg.norm(np.asarray(t.lin_vel, dtype=float))) <= STILL
    released = bool(mechkit.released(t, clearance=RELEASE))
    return dict(tip_depth=tip_depth, inside=inside, seat_h=seat_h, axis_off=axis_off, tilt=tilt, seated=seated,
                resting=resting, still=still, released=released, kinked=_kinked(t))


def score(objects):
    _update_latches(objects)
    if _LATCH["zero"]:
        return 0.0
    e = evaluate(objects)
    if e["kinked"] or not (e["released"] and e["still"]):
        return 0.0
    if e["tip_depth"] >= DEPTH_FULL and e["inside"]:
        return 1.0 if e["seated"] else 0.5
    if e["tip_depth"] >= DEPTH_PART and e["resting"]:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# -- scripted demo (closedloop) ------------------------------------------------------------------------------------------
demo = placeholder_demo
PITCH = math.pi / 2  # gripper level, pointing +x, pads closing along y on the stem's sides
CARRY_UP = 0.200  # seal face above the land while carrying: tip 19 mm over both lands
FULL_SEAL = (FULL_XY[0], FULL_XY[1], TT + LAND)


def _tcp(seal, off):
    return np.array([seal[0] + COM_X, seal[1], seal[2] + GRIP_Z]) + off


def demo_script(ctx):
    """Right arm only (the left holds READY). Reads the dip tube's measured tip and the full bottle's pose from the
    simulator; falls back to the fixed layout where they are not available (offline)."""
    R = ctx.right
    off = np.zeros(3)
    grasp = _tcp(SEAL0, off)
    # 1. approach the stem from -x, level, fingers open
    yield from R.move((0.19, -0.25, grasp[2] + 0.03), 2.5, FINGER_OPEN, pitch=PITCH)
    yield from R.move((grasp[0] - 0.07, grasp[1], grasp[2]), 2.0, FINGER_OPEN, pitch=PITCH)
    yield from R.trim_to_measured((grasp[0] - 0.07, grasp[1], grasp[2]))
    off = R.pos - np.array((grasp[0] - 0.07, grasp[1], grasp[2]))
    for attempt in range(2):
        yield from R.move(_tcp(SEAL0, off), 1.5, FINGER_OPEN, pitch=PITCH)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.3)
        if R.holding(0.016):
            break
        yield from R.grip(FINGER_OPEN, 0.6)
        yield from R.move(_tcp(SEAL0, off) - np.array([0.05, 0, 0]), 1.0, pitch=PITCH)
    # 2. draw it straight up: the curve drags through the empty bottle's neck
    up = (SEAL0[0], SEAL0[1], TT + LAND + CARRY_UP)
    yield from R.move(_tcp(up, off), 8.5, pitch=PITCH)
    yield from R.hold(1.0)
    # 3. carry: head off axis so the curved tip hangs over the full bottle's neck
    f_xy = np.array(FULL_SEAL[:2])
    try:
        fo = ctx.obj("full_bottle")
        f_xy = np.asarray(getattr(fo, "origin", FULL_SEAL), dtype=float)[:2]
    except Exception:  # noqa: BLE001
        pass
    head = np.array([f_xy[0] - TIP_OFF, f_xy[1], TT + LAND + CARRY_UP])
    yield from R.move(_tcp(head, off), 2.6, pitch=PITCH)
    yield from R.hold(1.5)
    yield from R.trim_to_measured(_tcp(head, off))
    off = R.pos - _tcp(head, np.zeros(3))
    # 4. measured tip over the neck centre (bounded correction)
    try:
        tip = np.asarray(ctx.obj("dip_tube").tip("end"), dtype=float)
    except Exception:  # noqa: BLE001  offline: the fixed layout
        tip = np.array([head[0] + TIP_OFF, head[1], head[2] - TIP_DEPTH])
    corr = np.clip(f_xy - tip[:2], -0.015, 0.015)
    head[:2] += corr
    yield from R.move(_tcp(head, off), 0.8, pitch=PITCH)
    # 5. feed: keep the tube point at the land on the neck axis, the head leading back over the neck
    land_z = TT + LAND
    h = head[2] - land_z
    h_entry = TIP_DEPTH + 0.003
    yield from R.move(_tcp((head[0], head[1], land_z + h_entry), off), 1.0, pitch=PITCH)
    h = h_entry
    dt = R.dt
    while h > TUBE_STRAIGHT:
        h = max(TUBE_STRAIGHT, h - 0.02 * dt)
        phi = math.asin(min(1.0, max(0.0, (h - TUBE_STRAIGHT) / TUBE_BEND_R)))
        phi = min(phi, TUBE_ARC)
        x = f_xy[0] + corr[0] * (h - TUBE_STRAIGHT) / (h_entry - TUBE_STRAIGHT) - TUBE_BEND_R * (1 - math.cos(phi))
        yield from R.move(_tcp((x, head[1], land_z + h), off), dt, pitch=PITCH)
    yield from R.move(_tcp((f_xy[0], f_xy[1], land_z + 0.015), off), 3.0, pitch=PITCH)
    yield from R.hold(0.5)
    yield from R.trim_to_measured(_tcp((f_xy[0], f_xy[1], land_z + 0.015), np.zeros(3)))
    off = R.pos - _tcp((f_xy[0], f_xy[1], land_z + 0.015), np.zeros(3))
    # 6. collar over the finish, then a short press until the seal face stops on the land
    yield from R.move(_tcp((f_xy[0], f_xy[1], land_z + 0.003), off), 1.0, pitch=PITCH)
    yield from R.press_until(_tcp((f_xy[0], f_xy[1], land_z + 0.003), off), max_depth=0.006, err_mm=3.0,
                             min_depth=0.002, speed=0.006)
    # 7. let go, back out along -x, rise, park
    yield from R.grip(FINGER_OPEN, 0.8)
    yield from R.hold(0.3)
    p = R.pos.copy()
    yield from R.move(p - np.array([0.06, 0.0, 0.0]), 1.5, pitch=PITCH)
    yield from R.move(p - np.array([0.06, 0.0, -0.10]), 1.5, pitch=PITCH)
    yield from R.home(8.0)
    yield from R.hold(2.0)
