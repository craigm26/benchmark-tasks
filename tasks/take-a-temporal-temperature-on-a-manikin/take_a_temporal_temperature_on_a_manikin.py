"""On the station table: a full-size adult manikin bust (vinyl head 160 mm wide, 195 mm deep, 238 mm chin to crown,
on a neck and navy-gowned shoulders) facing the robot, centred at x 0.33, y 0. On the table to the right of it, a
charcoal cradle holds an 80 g off-white temporal artery scanner lying on its side, probe end toward the bust. Gripping
the scanner's handle squeezes its scan button (a red lamp on top lights while it is held in). The probe cup at the front
end rides on a light spring, so it gives 1 to 4 mm when pressed flat on the skin.

The task: take the temperature as the scanner's directions say. Press the probe flat on the middle of the forehead,
slide it slowly straight across to the left temple hairline without lifting it and without going down the face or onto
the hair, then, without letting go of the button, touch the probe flat on the neck just behind the left ear lobe, and
put the scanner back in its cradle. The manikin's left is the robot's right (-y).

Requires -E object_jitter=0 (the cradle, the head and the behind-the-ear spot are fixed).

Run it:  inspect-robots run --embodiment yam_arena -E task=take_a_temporal_temperature_on_a_manikin.py \\
           -E object_jitter=0 -E control_hz=20 --instruction "Take the temperature: scan mid-forehead across to the
           left hairline, then behind the left ear lobe; cradle the scanner." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, extrude, hull, lamp, lathe, on_reset, on_step, set_lamp, sweep, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the demo parks before then
CONTROL_DT = 0.05  # s per control step: the runs use -E control_hz=20

SKIN = (0.94, 0.80, 0.70)
SKIN_EAR = (0.90, 0.74, 0.64)
HAIR = (0.25, 0.17, 0.11)
BROW = (0.20, 0.13, 0.08)
LIPS = (0.60, 0.25, 0.22)
GOWN = (0.18, 0.25, 0.42)
WHITE = (0.93, 0.93, 0.95)
CUP_GREY = (0.30, 0.31, 0.33)
DARK = (0.12, 0.12, 0.13)
DISPLAY = (0.06, 0.10, 0.25)
CHARCOAL = (0.20, 0.21, 0.23)
RED = (0.95, 0.08, 0.06)

# ---------------------------------------------------------------------------------------------------------------------
# The head. Every horizontal section is a "stadium": a circle of radius r(z) swept from the front axis A1 to the back
# axis A2, 35 mm apart (one smooth skull 160 mm wide and 195 mm deep). r(z) comes from these rows, (r, height above the
# chin) in mm, measured off an adult head: chin, jaw, cheeks, brow, mid-forehead (widest), crown.
A1 = (0.310, 0.0)  # front axis (x, y)
A2 = (0.345, 0.0)  # back axis
CHIN_Z = 0.060  # chin height above the table top (m)
ROWS_MM = [(25, 0), (50, 12), (62, 35), (67, 60), (69, 90), (74, 110), (78, 128), (80, 150), (79, 170), (74, 195),
           (64, 215), (45, 230), (0, 238)]
_RZ = np.array([[r / 1000.0, CHIN_Z + z / 1000.0] for r, z in ROWS_MM])  # (r m, z m above TOP)

SCAN_Z = CHIN_Z + 0.155  # the scan line, midway between the brows (+128) and the front hairline (+185), above TOP
HAIRLINE_FRONT_Z = CHIN_Z + 0.185
HAIR_BAND_Z = CHIN_Z + 0.125  # the hair comes down to here from the temples round the back (top of the ear)
TEMPLE_DEG = 50.0  # the temple hairline: azimuth from the forehead midline, at the scan line


def skin_r(z_above_top: float) -> float:
    """Skin radius (m) of the head section at this height above the table top; 0 above the crown or below the chin."""
    z = float(z_above_top)
    if z < _RZ[0, 1] or z > _RZ[-1, 1]:
        return 0.0
    return float(np.interp(z, _RZ[:, 1], _RZ[:, 0]))


def skin_dr(z_above_top: float) -> float:
    """dr/dz of the skin profile (slope of the wall; 0 is a vertical wall)."""
    z = float(z_above_top)
    i = int(np.clip(np.searchsorted(_RZ[:, 1], z) - 1, 0, len(_RZ) - 2))
    (r0, z0), (r1, z1) = _RZ[i], _RZ[i + 1]
    return float((r1 - r0) / (z1 - z0))


R_LINE = skin_r(SCAN_Z)  # 79.75 mm: radius of the forehead at the scan line
HAIRLINE_ARC = R_LINE * math.radians(TEMPLE_DEG)  # 69.6 mm of arc from the midline to the temple hairline


def _radial(p) -> tuple[float, np.ndarray]:
    """Distance in plan from the head's axis segment A1..A2, and the outward unit vector there."""
    x = float(np.clip(p[0], A1[0], A2[0]))
    d = np.array([p[0] - x, p[1] - A1[1]])
    rho = float(np.hypot(*d))
    u = d / rho if rho > 1e-9 else np.array([-1.0, 0.0])
    return rho, u


def skin_sd(p) -> float:
    """Signed distance (m, approx.) from a world point to the skin: + outside, - inside. Uses the smooth profile."""
    z = float(p[2]) - TOP
    rho, _ = _radial(p)
    r = skin_r(z)
    if r <= 0.0:
        return 1.0
    return rho - r


def skin_normal(p) -> np.ndarray:
    """Outward unit normal of the skin nearest a world point (from the smooth profile)."""
    _, u = _radial(p)
    n = np.array([u[0], u[1], -skin_dr(float(p[2]) - TOP)])
    return n / np.linalg.norm(n)


def forehead_point(phi: float, z_above_top: float = SCAN_Z) -> np.ndarray:
    """World point on the forehead at azimuth phi from the midline (positive toward the manikin's left, -y)."""
    r = skin_r(z_above_top)
    return np.array([A1[0] - r * math.cos(phi), A1[1] - r * math.sin(phi), TOP + z_above_top])


def arc_coord(p) -> tuple[float, float] | None:
    """(arc length from the forehead midline along the scan line toward the left temple, height above the scan line),
    for a point in front of the front axis; None behind it (sides and back of the head)."""
    if p[0] > A1[0]:
        return None
    phi = math.atan2(-(p[1] - A1[1]), A1[0] - p[0])
    return R_LINE * phi, float(p[2]) - TOP - SCAN_Z


# Behind the left ear lobe: on the back of the skull, 20 degrees behind the side, at lobe height.
T_Z = CHIN_Z + 0.075
_T_BETA = math.radians(20.0)
_T_R = skin_r(T_Z)
T_POINT = np.array([A2[0] + _T_R * math.sin(_T_BETA), A2[1] - _T_R * math.cos(_T_BETA), TOP + T_Z])
T_NORMAL = skin_normal(T_POINT)


def _stadium_ring(r: float, z: float, n: int = 16, front=(90.0, 270.0), back=(-90.0, 90.0)) -> list:
    """Points of a stadium section (head frame: Mech origin at (A1x, 0, TOP)), half circles of n points about A1 (front)
    and A2 (back), restricted to the given angle ranges (degrees, standard atan2 angles)."""
    if r <= 0.0:
        return [(0.0, 0.0, z), (A2[0] - A1[0], 0.0, z)]
    pts = []
    for (cx, (a0, a1)) in ((0.0, front), (A2[0] - A1[0], back)):
        if a0 is None:
            continue
        for a in np.linspace(math.radians(a0), math.radians(a1), n):
            pts.append((cx + r * math.cos(a), r * math.sin(a), z))
    return pts


def _row_r(z_rel_chin_mm: float, grow: float = 0.0) -> float:
    return skin_r(CHIN_Z + z_rel_chin_mm / 1000.0) + grow


def _head_shapes() -> list:
    shapes = []
    # skull: one convex stadium frustum per pair of profile rows (vertices on the smooth surface)
    for (r0, z0), (r1, z1) in zip(_RZ[:-1], _RZ[1:]):
        shapes.append(hull((0, 0, 0), _stadium_ring(r0, z0) + _stadium_ring(r1, z1), SKIN))
    # hair, look-only, 1.5 mm proud: a cap above the front hairline, and a band from the temples round the back
    cap_z = [HAIRLINE_FRONT_Z, CHIN_Z + 0.195, CHIN_Z + 0.215, CHIN_Z + 0.230, CHIN_Z + 0.2375]
    for za, zb in zip(cap_z[:-1], cap_z[1:]):
        pts = _stadium_ring(skin_r(za) + 0.0015, za) + _stadium_ring(skin_r(zb) + 0.0015, zb)
        shapes.append(dataclass_replace(hull((0, 0, 0), pts, HAIR), collide=False))
    band_z = [HAIR_BAND_Z, CHIN_Z + 0.150, CHIN_Z + 0.170, HAIRLINE_FRONT_Z]
    t = TEMPLE_DEG
    for za, zb in zip(band_z[:-1], band_z[1:]):
        for side in (-1, 1):  # each side: from the temple round to the back midline
            front = (180.0 + t, 270.0) if side < 0 else (90.0, 180.0 - t)
            back = (-90.0, 0.0) if side < 0 else (0.0, 90.0)
            pts = (_stadium_ring(skin_r(za) + 0.0015, za, 8, front, back)
                   + _stadium_ring(skin_r(zb) + 0.0015, zb, 8, front, back))
            pts.append((A2[0] - A1[0], 0.0, za))
            pts.append((A2[0] - A1[0], 0.0, zb))
            shapes.append(dataclass_replace(hull((0, 0, 0), pts, HAIR), collide=False))
    # nose (collides): root at the brows, tip 21 mm proud
    def face(zc_mm, phi_deg, out_mm):
        r = _row_r(zc_mm) + out_mm / 1000.0
        a = math.radians(phi_deg)
        return (-r * math.cos(a), -r * math.sin(a), CHIN_Z + zc_mm / 1000.0)

    nose = [face(118, 0, 0), face(118, 4, -3), face(118, -4, -3), face(95, 0, 11), face(95, 7, 2), face(95, -7, 2),
            face(72, 0, 21), face(64, 0, 15), face(62, 11, 5), face(62, -11, 5), face(70, 11, 4), face(70, -11, 4),
            face(62, 0, -4), face(118, 0, -5), face(85, 0, -5)]
    shapes.append(hull((0, 0, 0), nose, SKIN))
    # look-only face: brows, closed lids, lips
    for side in (-1, 1):
        brow = [face(126, side * a, 1.5) for a in (8, 16, 24, 32, 38)]
        lid = [face(110, side * a, 1.0) for a in (11, 18, 25, 31)]
        shapes.append(dataclass_replace(sweep((0, 0, 0), brow, 0.0028, BROW), collide=False))
        shapes.append(dataclass_replace(sweep((0, 0, 0), lid, 0.0016, BROW), collide=False))
    lips = [face(36, a, 1.5) for a in (-17, -9, 0, 9, 17)]
    shapes.append(dataclass_replace(sweep((0, 0, 0), lips, 0.0035, LIPS), collide=False))
    shapes.extend(_ear_shapes())
    return shapes


EAR_XC = 0.326  # world x of the ear's centre line (the helix rim reaches 20 mm behind it)
EAR_ZC = CHIN_Z + 0.092  # centre height above TOP (ear from about +60 to +125 above the chin)


def _ear_shapes() -> list:
    out = []
    for side in (-1, 1):
        rim = []
        for a in np.linspace(math.radians(-112), math.radians(112), 15):
            x = EAR_XC + 0.016 * math.cos(a)
            z = EAR_ZC + 0.030 * math.sin(a)
            y = side * (skin_r(z) + 0.008)
            rim.append((x - A1[0], y, z))
        out.append(sweep((0, 0, 0), rim, 0.004, SKIN_EAR))
        concha = []
        for dz in (-0.029, 0.0, 0.029):
            z = EAR_ZC + dz
            for dx in (-0.006, 0.014):
                concha.append((EAR_XC + dx - A1[0], side * (skin_r(z) - 0.002), z))
                concha.append((EAR_XC + dx - A1[0], side * (skin_r(z) + 0.006), z))
        out.append(hull((0, 0, 0), concha, SKIN_EAR))
    return out


def dataclass_replace(shape, **kw):
    import dataclasses

    return dataclasses.replace(shape, **kw)


def _bust_shapes() -> list:
    sh = []
    cx = 0.360 - A1[0]
    sh.append(hull((0, 0, 0), [(cx + dx, dy, 0.0) for dx in (-0.075, 0.075) for dy in (-0.17, 0.17)]
                   + [(cx + dx, dy, 0.034) for dx in (-0.062, 0.062) for dy in (-0.155, 0.155)]
                   + [(cx + dx, dy, 0.045) for dx in (-0.05, 0.05) for dy in (-0.07, 0.07)], GOWN))
    sh.append(lathe((0.335 - A1[0], 0.0, 0.025), [(0.055, 0.0), (0.052, CHIN_Z + 0.07 - 0.025)], SKIN))
    return sh


MANIKIN = Mech("manikin", pos=(A1[0], A1[1], TOP), mass=None, shapes=[*_bust_shapes(), *_head_shapes()], friction=0.6)

# ---------------------------------------------------------------------------------------------------------------------
# The scanner, frame at the grasp centre on the handle axis, +x toward the probe, +z up (display face), button on -y.
# Handle 40 mm tall x 28 mm thick (rounded), x -82..40 mm; head tapering r 16 to 12.5 mm, x 40..62; probe cup on a
# 4 mm sprung slide, rim at x 68 mm. 150 mm long, 80 g in all.
HANDLE_W, HANDLE_H = 0.028, 0.040
RIM_X = 0.068
CUP_TRAVEL = 0.004
BUTTON_TRAVEL = 0.004


def _rounded_rect(w, h, r, n=4):
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0.0), (-w / 2 + r, h / 2 - r, 90.0), (-w / 2 + r, -h / 2 + r, 180.0),
                       (w / 2 - r, -h / 2 + r, 270.0)):
        for a in np.linspace(math.radians(a0), math.radians(a0 + 90), n):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


_HANDLE = extrude((-0.021, 0.0, 0.0), _rounded_rect(HANDLE_W, HANDLE_H, 0.006), 0.122, WHITE, axis="x")
_HEAD = lathe((0.040, 0.0, 0.0), [(0.016, 0.0), (0.0125, 0.022)], WHITE, axis="x")
CUP = Joint("probe", "slide", axis="x", at=(RIM_X - 0.003, 0.0, 0.0), low=-CUP_TRAVEL, high=0.0, spring=400.0,
            mass=0.012, shapes=[tube((0.0, 0.0, 0.0), 0.0095, 0.0125, 0.006, CUP_GREY, axis="x"),
                                dataclass_replace(cyl((-0.001, 0.0, 0.0), 0.0094, 0.001, DARK, axis="x"), collide=False)])
BUTTON = Joint("button", "slide", axis="y", at=(0.0, -0.014, 0.0), low=0.0, high=BUTTON_TRAVEL, spring=500.0, mass=0.010,
               shapes=[box((0.0, -0.001, 0.0), (0.020, 0.006, 0.014), DARK)])
SCANNER_START = (0.250, -0.180, TOP + 0.0651)  # resting in the cradle: handle bottom 0.1 mm over the saddle floors
SCANNER = Mech("scanner", pos=SCANNER_START, mass=0.058, friction=0.9,
               shapes=[_HANDLE, _HEAD,
                       dataclass_replace(box((-0.055, 0.0, 0.0204), (0.040, 0.018, 0.001), DISPLAY), collide=False),
                       *lamp("scan", (0.033, 0.0, 0.0212), 0.004, {"red": RED})],
               joints=[CUP, BUTTON])

# The cradle: a charcoal base 150 x 60 x 20 mm and two U saddles (34 mm inside, 3 mm a side on the 28 mm handle,
# walls 4 mm thick and 22 mm tall, floors 45 mm above the table), under the handle at x -75..-50 and 22..40 mm.
_U = [(-0.021, 0.020), (0.021, 0.020), (0.021, 0.067), (0.017, 0.067), (0.017, 0.045), (-0.017, 0.045),
      (-0.017, 0.067), (-0.021, 0.067)]
CRADLE = Mech("cradle", pos=(SCANNER_START[0], SCANNER_START[1], TOP), mass=None, friction=0.8,
              shapes=[box((-0.010, 0.0, 0.010), (0.150, 0.060, 0.020), CHARCOAL),
                      extrude((-0.0625, 0.0, 0.0), _U, 0.025, CHARCOAL, axis="x"),
                      extrude((0.031, 0.0, 0.0), _U, 0.018, CHARCOAL, axis="x")])

OBJECTS = [MANIKIN, CRADLE, SCANNER]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
PRESSED = 0.003  # button in at least 3 mm
CUP_PUSHED = 0.0005  # probe cup pushed in at least 0.5 mm: it is touching something
GAP_MAX, GAP_MIN = 0.0025, -0.004  # rim within 2.5 mm of the smooth skin (facets sit up to 0.4 mm inside it)
TILT_MAX = 15.0  # deg between the probe axis and the skin normal
START_TOL = 0.010  # first touch within 10 mm of the forehead midline on the scan line
LINE_TOL = 0.010  # a bin counts only within 10 mm of the scan line
VOID_OFF_LINE = 0.015  # on the forehead, the cup centre 15 mm off the line voids the scan (down the face)
VOID_ON_HAIR = 0.066  # cup centre past 66 mm of arc: rim 9 mm onto the hair, voids the scan
BIN, NBINS = 0.005, 10
MAX_SPEED = 0.050  # m/s
HAIRLINE_STOP = (0.051, 0.061)  # farthest on-skin cup centre: rim edge on the hairline
EAR_TOL, EAR_STEPS = 0.012, 10  # within 12 mm of the spot for 0.5 s
IN_CRADLE_XY, IN_CRADLE_Z, IN_CRADLE_DEG = 0.008, 0.006, 10.0
STILL_STEPS = 20  # 1 s
_RING = [(math.cos(a), math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]

_ST: dict = {}


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, prev_c=None, scan=None, best=0.0, fail=False, still=0, step=0, log=[])


_reset()


def _rot(s) -> np.ndarray:
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cup_pose(s) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(rim centre, probe axis, ring basis 3x2) of the scanner state, with the cup at its measured travel."""
    R = _rot(s)
    origin = np.asarray(getattr(s, "origin", s.pos), dtype=float)
    q = float(s.joints.get("probe", 0.0)) if hasattr(s, "joints") else 0.0
    c = origin + R @ np.array([RIM_X + q, 0.0, 0.0])
    return c, R[:, 0], R[:, 1:3]


def contact(s) -> dict:
    """What the scanner's probe is doing on the skin: rim gap, tilt, pushed, on_skin."""
    c, a, B = cup_pose(s)
    ring = [c + 0.0125 * (B[:, 0] * u + B[:, 1] * v) for u, v in _RING]
    gap = min(skin_sd(p) for p in ring)
    n = skin_normal(c)
    tilt = math.degrees(math.acos(float(np.clip(-a @ n, -1.0, 1.0))))
    pushed = float(s.joints.get("probe", 0.0)) <= -CUP_PUSHED
    return {"c": c, "gap": gap, "tilt": tilt, "pushed": pushed,
            "on_skin": bool(pushed and GAP_MIN <= gap <= GAP_MAX and tilt <= TILT_MAX)}


def _scan_value(sc) -> float:
    if sc is None or not sc["start_ok"] or sc["void"]:
        return 0.0
    n = len(sc["bins"])
    if n < 5:
        return 0.0
    v = 0.6 * n / NBINS
    if HAIRLINE_STOP[0] <= sc["far"] <= HAIRLINE_STOP[1]:
        v += 0.2
    if sc["ear"]:
        v += 0.2
    return round(v, 4)


def in_cradle(s) -> bool:
    o = np.asarray(getattr(s, "origin", s.pos), dtype=float)
    R = _rot(s)
    yaw_err = math.degrees(abs(math.atan2(R[1, 0], R[0, 0])))
    up_err = float(belimits.tilt_deg(s))
    return bool(np.hypot(o[0] - SCANNER_START[0], o[1] - SCANNER_START[1]) <= IN_CRADLE_XY
                and abs(o[2] - SCANNER_START[2]) <= IN_CRADLE_Z and yaw_err <= IN_CRADLE_DEG
                and up_err <= IN_CRADLE_DEG)


def _released(s) -> bool:
    """Both grippers clear of the scanner. In the simulator mechkit.released() checks every finger pad is 3 cm from it;
    always, the button must have sprung back out (the grip is what holds it in)."""
    button_out = float(s.joints.get("button", 0.0)) < 0.001
    if mechkit._ENV.get("env") is None:  # offline replays have no finger points: the button is the evidence
        return button_out
    return button_out and bool(mechkit.released(s, clearance=0.03))


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    s = objects["scanner"]
    if s.low[2] < TOP + 0.004:  # touched the table: dropped or set down off its cradle
        _ST["fail"] = True
    k = contact(s)
    c = k["c"]
    speed = 0.0 if _ST["prev_c"] is None else float(np.linalg.norm(c - _ST["prev_c"])) / CONTROL_DT
    _ST["prev_c"] = c
    pressed = float(s.joints.get("button", 0.0)) >= PRESSED
    sc = _ST["scan"]
    if pressed and sc is None:
        sc = _ST["scan"] = {"started": False, "start_ok": False, "void": False, "bins": set(), "far": 0.0,
                            "ear_run": 0, "ear": False}
    if not pressed and sc is not None:
        _ST["best"] = max(_ST["best"], _scan_value(sc))
        sc = _ST["scan"] = None
    if sc is not None and k["on_skin"]:
        ac = arc_coord(c)
        if not sc["started"]:
            sc["started"] = True
            sc["start_ok"] = ac is not None and abs(ac[0]) <= START_TOL and abs(ac[1]) <= START_TOL
        if ac is not None:
            s_arc, dz = ac
            if abs(dz) > VOID_OFF_LINE or s_arc > VOID_ON_HAIR:
                sc["void"] = True
            elif abs(dz) <= LINE_TOL:
                sc["far"] = max(sc["far"], s_arc)
                if 0.0 <= s_arc < BIN * NBINS and speed <= MAX_SPEED:
                    sc["bins"].add(int(s_arc // BIN))
        if np.linalg.norm(c - T_POINT) <= EAR_TOL:
            sc["ear_run"] += 1
            if sc["ear_run"] >= EAR_STEPS:
                sc["ear"] = True
        else:
            sc["ear_run"] = 0
    elif sc is not None:
        sc["ear_run"] = 0
    ok_end = in_cradle(s) and _released(s) and s.resting()
    _ST["still"] = _ST["still"] + 1 if ok_end else 0


def score(objects):
    """Best valid scan: 0.6 x bins/10 (0 under 5 bins), +0.2 for stopping with the rim on the temple hairline, +0.2 for
    the 0.5 s touch behind the ear in the same press. 0 unless the scanner ends released and still in its cradle for
    1 s, and 0 for the whole run if it ever touches the table."""
    _update(objects)
    if _ST["fail"] or _ST["still"] < STILL_STEPS:
        return 0.0
    return max(_ST["best"], _scan_value(_ST["scan"]))


def success(objects):
    return score(objects) >= 1.0 and objects["scanner"].resting()


@on_step
def _lamp(env):  # scorer side: the scan lamp shows the button state (the demo never reads it)
    try:
        s = mechkit._mech_state(env, SCANNER) if hasattr(mechkit, "_mech_state") else None
    except Exception:  # noqa: BLE001
        s = None
    if s is None:
        return
    set_lamp(SCANNER, "scan", "red" if float(s.joints.get("button", 0.0)) >= PRESSED else "off")


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps its own parts off the manikin: no arm part may enter the head and ears (plan outline, 1 cm margin, as
# two rectangles: the skull front to back, and its widest part with the ears) or the shoulders.
ARM_KEEP_OUT = [
    {"name": "manikin head, front to back, 1 cm margin", "x": (0.220, 0.435), "y": (-0.068, 0.068), "z_top": TOP + 0.31},
    {"name": "manikin head sides and ears, 1 cm margin", "x": (0.255, 0.400), "y": (-0.100, 0.100), "z_top": TOP + 0.31},
    {"name": "manikin shoulders", "x": (0.285, 0.435), "y": (-0.170, 0.170), "z_top": TOP + 0.08},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (right arm only; the left arm holds READY). It reads the scanner's live pose and button travel from the
# simulator, and computes the cup's gap and tilt from the fixed head geometry above.
demo = placeholder_demo
PUSH = 0.002  # cup pushed in 2 mm while scanning (1.6 N)
LEAN = 0.5  # rad: the grip leaned about the scanner's axis, wrist toward the robot's right, for reach at the forehead
SWEEP_END = 0.056  # cup centre 56 mm along the arc: rim edge 1.5 mm short of the hairline


def grip(yaw: float, lean: float) -> np.ndarray:
    """Top-down grip turned to yaw, leaned about the scanner axis (which is horizontal, along yaw)."""
    return axis_angle_matrix(np.array([0, 0, 1.0]), yaw) @ axis_angle_matrix(np.array([1.0, 0, 0]), lean) @ grip_rot()


def contact_origin(surface: np.ndarray, axis: np.ndarray, push: float = PUSH) -> np.ndarray:
    """Scanner origin (= grasp centre) that puts the rim flat on the skin at this surface point with the cup pushed in
    by `push`, the probe axis `axis` (horizontal). Found from the smooth head profile."""
    a = axis / np.linalg.norm(axis)
    side = np.array([-a[1], a[0], 0.0])
    up = np.array([0.0, 0.0, 1.0])

    def min_gap(d):
        c = surface - d * a
        return min(skin_sd(c + 0.0125 * (side * u + up * v)) for u, v in _RING)

    lo, hi = -0.01, 0.02
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if min_gap(mid) > 0.0 else (mid, hi)
    rim = surface - hi * a
    return rim + push * a - RIM_X * a


def _forehead_pose(s_arc: float, out: float = 0.0):
    phi = s_arc / R_LINE
    S = forehead_point(phi)
    a = np.array([math.cos(phi), math.sin(phi), 0.0])
    return contact_origin(S, a) - out * a, phi


def _ear_pose(out: float = 0.0):
    a = -np.array([T_NORMAL[0], T_NORMAL[1], 0.0])
    a /= np.linalg.norm(a)
    return contact_origin(T_POINT, a) - out * a, math.atan2(a[1], a[0])


class _Track:
    """Moves the right hand so the scanner (not the hand) follows a target: in a live run, the error between the
    scanner's measured rim point and its target is integrated into an offset on the hand command, which cancels arm
    sag and any shift of the scanner in the grip. Offline (dry runs) the hand follows the target directly."""

    def __init__(self, ctx):
        self.ctx, self.R = ctx, ctx.right
        self.off = np.zeros(3)
        self.dyaw = 0.0
        self.active = False

    def scanner(self):
        return self.ctx.obj("scanner")

    def step(self, origin, yaw, lean, finger=None, gain=0.25):
        origin = np.asarray(origin, dtype=float)
        if self.active and not self.R.offline:
            s = self.scanner()
            Rm = _rot(s)
            o = np.asarray(s.origin, dtype=float)
            a_t = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            err = (origin + RIM_X * a_t) - (o + Rm[:, 0] * RIM_X)
            self.off = self.off + gain * err
            n = float(np.linalg.norm(self.off))
            if n > 0.035:
                self.off *= 0.035 / n
            yaw_m = math.atan2(Rm[1, 0], Rm[0, 0])
            e = (yaw - yaw_m + math.pi) % (2 * math.pi) - math.pi
            self.dyaw = float(np.clip(self.dyaw + gain * e, -0.2, 0.2))
        f = self.R.finger if finger is None else finger
        self.R._set_cartesian(origin + self.off, grip(yaw + self.dyaw, lean), f)

    def go(self, origin, yaw, lean, duration, finger=None):
        """Smoothstep from the current target to this one over duration (s)."""
        p0 = getattr(self, "p", None)
        p0 = self.R.pos - self.off if p0 is None else p0
        y0, l0 = getattr(self, "yaw", yaw), getattr(self, "lean", lean)
        f0 = self.R.finger
        f1 = f0 if finger is None else finger
        steps = max(1, round(duration / CONTROL_DT))
        target = np.asarray(origin, dtype=float)
        for i in range(1, steps + 1):
            u = 3 * (i / steps) ** 2 - 2 * (i / steps) ** 3
            self.step(p0 + u * (target - p0), y0 + u * (yaw - y0), l0 + u * (lean - l0), f0 + u * (f1 - f0))
            yield
        self.p, self.yaw, self.lean = target, yaw, lean

    def hold(self, duration):
        for _ in range(max(1, round(duration / CONTROL_DT))):
            self.step(self.p, self.yaw, self.lean)
            yield


def _carry_time(a, b, v=0.08):
    """Seconds for a smoothstep move whose peak speed (1.5 x mean) is v."""
    return max(0.6, 1.5 * float(np.linalg.norm(np.asarray(b) - np.asarray(a))) / v)


def demo_script(ctx):
    """Right arm: pick the scanner from its cradle (the grip presses the button), press the probe flat on the forehead
    midline, slide it 56 mm across to the left temple hairline at 15 mm/s, lift off, go round the ear and touch the probe
    flat behind the ear lobe for 2.5 s, put the scanner back in the cradle, open and park at READY."""
    R = ctx.right
    tr = _Track(ctx)
    pick = np.array(SCANNER_START)
    above = pick + np.array([0.0, 0.0, 0.060])
    # 1) grasp: top-down, pads across the 28 mm handle; the -y pad pushes the button in
    yield from R.move(above, 3.5, FINGER_OPEN)
    yield from R.trim_to_measured(above)
    tr.p, tr.yaw, tr.lean = above, 0.0, 0.0
    tr.off = R.pos - above
    yield from tr.go(pick, 0.0, 0.0, 2.0)
    for attempt in range(2):
        yield from R.grip(FINGER_CLOSED, 1.0)
        yield from R.hold(0.3)
        pressed = R.offline or float(ctx.obj("scanner").joints.get("button", 0.0)) >= PRESSED
        if R.holding(0.016) and pressed:  # pads stop on the 28 mm handle; a miss closes to 0
            break
        yield from R.grip(FINGER_OPEN, 0.6)
        yield from tr.go(pick + np.array([0, 0, 0.03]), 0.0, 0.0, 0.8)
        yield from tr.go(pick, 0.0, 0.0, 0.8)
    tr.active = True
    # 2) lift 60 mm at 50 mm/s, then carry to 30 mm out from the forehead start, round the face's right side
    lift = pick + np.array([0.0, 0.0, 0.060])
    yield from tr.go(lift, 0.0, 0.0, 2.5)
    start_out, _ = _forehead_pose(0.0, out=0.030)
    via = np.array([0.140, -0.150, TOP + 0.200])
    yield from tr.go(via, 0.0, LEAN, _carry_time(lift, via))
    yield from tr.go(start_out, 0.0, LEAN, _carry_time(via, start_out))
    yield from tr.hold(2.5)
    # 3) approach along the forehead normal at 10 mm/s until the cup is pushed 2 mm, and settle
    start, _ = _forehead_pose(0.0)
    yield from tr.go(start, 0.0, LEAN, 6.0)
    yield from tr.hold(1.8)
    # 4) sweep across to the hairline at 15 mm/s (constant speed), yaw following the forehead
    steps = round(SWEEP_END / 0.015 / CONTROL_DT)
    for i in range(1, steps + 1):
        p, phi = _forehead_pose(SWEEP_END * i / steps)
        tr.step(p, phi, LEAN)
        tr.p, tr.yaw, tr.lean = p, phi, LEAN
        yield
    yield from tr.hold(3.5)  # rim on the hairline
    # 5) lift off 30 mm along the normal, go round the ear on waypoints 3 cm or more from the head, touch behind it
    end_out, phi = _forehead_pose(SWEEP_END, out=0.030)
    yield from tr.go(end_out, phi, LEAN, 1.5)
    side = np.array([0.205, -0.175, TOP + 0.175])
    ear_out, ear_yaw = _ear_pose(out=0.035)
    behind = np.array([ear_out[0] + 0.010, -0.205, ear_out[2]])
    yield from tr.go(side, math.radians(70), 0.2, _carry_time(end_out, side))
    yield from tr.go(behind, ear_yaw, 0.0, _carry_time(side, behind))
    yield from tr.go(ear_out, ear_yaw, 0.0, _carry_time(behind, ear_out))
    yield from tr.hold(0.3)
    ear, _ = _ear_pose()
    yield from tr.go(ear, ear_yaw, 0.0, 3.6)
    yield from tr.hold(2.5)
    away, _ = _ear_pose(out=0.040)
    yield from tr.go(away, ear_yaw, 0.0, 4.0 * 1.5)
    # 6) back to the cradle: out to the side, round to above it at yaw 0, lower to 1 mm over the saddles, release
    back = np.array([0.300, -0.250, TOP + 0.150])
    yield from tr.go(back, math.radians(45), 0.0, _carry_time(away, back, 0.06))
    yield from tr.go(above, 0.0, 0.0, _carry_time(back, above, 0.06))
    yield from tr.hold(2.0)
    place = pick + np.array([0.0, 0.0, 0.001])
    yield from tr.go(place, 0.0, 0.0, 3.5)
    yield from tr.hold(1.5)
    tr.active = False
    yield from R.grip(FINGER_OPEN, 1.0)
    yield from R.hold(0.3)
    yield from R.move(R.pos + np.array([0.0, 0.0, 0.080]), 1.6)
    # 7) park at READY, gripper open
    yield from R.home(3.0)
    yield from R.hold(1.0)
