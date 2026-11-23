"""On the station table: a dark walnut tabletop closet bay (70 cm inner rod span, 30 cm tall, back panel front at x 45.5 cm,
side walls at y +-35 cm) carrying a fixed 32 mm chrome closet rod at x 37.0 cm, 25 cm above the table. A 40 g
brushed-steel and chrome captive anti-theft security ring (38 mm ID upper loop, lower centered keyhole housing with a
22 x 22 mm upper window above a 12 mm captive stem slot) rides freely on the rod with unactuated lateral slide and
pendulum swing, starting at y +4.5 cm. In front of the closet on a charcoal wooden valet tray (centered at x 22.6 cm,
y -4.0 cm), a full-size 17 in (43.1 cm) natural maple hotel ball-top suit hanger (140 g, 16 mm chrome ball on a 6 mm
stem, no hook) leans 12 deg backward against two outer tray pillars.

The task: steady the free-sliding, free-swinging captive ring with one hand while the other hand lifts the wooden
ball-top hanger, threads the 16 mm chrome ball through the 22 x 22 mm upper keyhole window, lowers the 6 mm stem into
the 12 mm lower captive collar so the ball seats on the inner cradle ledges, and releases both parts so the hanger hangs
freely clear of the table and closet walls.

Requires -E object_jitter=0 (the closet and valet tray are fixed; the starting layout matches the setup description).

Run it:  inspect-robots run --embodiment yam_arena -E task=hotel_ball_top_hanger_into_captive_closet_ring.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Hang the wooden ball-top hanger in the chrome closet ring on the rail, then let go." \\
           --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, ball, box, cyl, on_reset, tube  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long: shows both arms parked clear
CONTROL_DT = 0.05  # s per control step (-E control_hz=20)

# Colours (sRGB, high contrast at 320x176)
WALNUT = (0.24, 0.19, 0.16)
CHROME = (0.90, 0.92, 0.95)
STEEL = (0.46, 0.50, 0.56)
MAPLE = (0.88, 0.73, 0.48)
CHARCOAL = (0.22, 0.24, 0.28)


def _quat_wxyz(R) -> tuple[float, float, float, float]:
    q = ik.matrix_to_quat_xyzw(np.asarray(R, dtype=float))
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def _rx(angle: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), float(angle))


def _ry(angle: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), float(angle))


# ---------------------------------------------------------------------------------------------------------------------
# CLOSET: fixed dark walnut tabletop closet bay + 32 mm OD chrome steel rod + captive security ring (Nahanco HHSRA100)
ROD_X = 0.370
ROD_Z = 0.250
ROD_R = 0.016  # 32 mm OD (standard 1-1/4 in closet rod); front face x = 0.354 (20 mm clear of READY pad tips)
STILE_Y = 0.350  # inner faces of side walls at y = +-0.350 m (700 mm clear rod span)
STILE_W = 0.018  # 18 mm walnut side wall thickness
BACK_FRONT_X = 0.455  # front face of dark walnut back panel (85 mm behind rod axis)
BACK_T = 0.012
CLOSET_H = 0.300
SIDE_X0 = 0.354  # front face of side stiles (flush with rod front, 20 mm clear of READY pad tips)
SIDE_X1 = BACK_FRONT_X + BACK_T  # 0.467

CLOSET_BASE_SHAPES = (
    box(
        ((SIDE_X0 + SIDE_X1) / 2.0 - ROD_X, +(STILE_Y + STILE_W / 2.0), CLOSET_H / 2.0),
        (SIDE_X1 - SIDE_X0, STILE_W, CLOSET_H),
        WALNUT,
    ),
    box(
        ((SIDE_X0 + SIDE_X1) / 2.0 - ROD_X, -(STILE_Y + STILE_W / 2.0), CLOSET_H / 2.0),
        (SIDE_X1 - SIDE_X0, STILE_W, CLOSET_H),
        WALNUT,
    ),
    box(
        (BACK_FRONT_X + BACK_T / 2.0 - ROD_X, 0.0, CLOSET_H / 2.0),
        (BACK_T, 2.0 * (STILE_Y + STILE_W), CLOSET_H),
        WALNUT,
    ),
    cyl((0.0, 0.0, ROD_Z), ROD_R, 2.0 * STILE_Y, CHROME, axis="y"),
)

# RING: chained joints on CLOSET (Nahanco HHSRA100 A-ring, 38 mm ID loop + centered lower keyhole housing).
# Centered symmetrically in x around x_local = 0 so unloaded and loaded ring hang plumb.
# Pocket interior: x in [-0.011, +0.011], y in [-0.011, +0.011], z in [-0.0635, -0.0245].
# Upper front keyhole window: y in [-0.011, +0.011] (22 mm), z in [-0.0465, -0.0245] (22 mm).
# Lower front & floor captive stem slot: y in [-0.006, +0.006] (12 mm), z in [-0.0685, -0.0465].
# Outer side cheeks: x in [-0.016, +0.016] (32 mm), y in +-[0.011, 0.024] (13 mm solid grasp cheeks).
RING_Y0 = 0.045
RING_SLIDE_MAX_Y = 0.312
RING_SWING_LIMIT = 0.95
RING_WINDOW_Z = (-0.0465, -0.0245)
RING_FLOOR_TOP_Z = -0.0635
RING_LO = np.array([-0.023, -0.024, -0.0685], dtype=float)
RING_HI = np.array([+0.023, +0.024, +0.0230], dtype=float)

RING_SWING_SHAPES = (
    tube((0.0, 0.0, 0.0), 0.019, 0.023, 0.010, CHROME, axis="y", sides=24),  # 38 mm ID upper loop (3 mm radial gap)
    box((0.0, 0.0, -0.0220), (0.032, 0.048, 0.005), STEEL),  # top cap above window (3.5 mm clear of rod)
    box((0.0, +0.0175, -0.0465), (0.032, 0.013, 0.044), STEEL),  # left cheek (+y)
    box((0.0, -0.0175, -0.0465), (0.032, 0.013, 0.044), STEEL),  # right cheek (-y)
    box((+0.0135, 0.0, -0.0465), (0.005, 0.022, 0.044), STEEL),  # solid rear stop wall
    box((-0.0135, +0.0085, -0.0575), (0.005, 0.005, 0.022), STEEL),  # front lower captive lip (+y)
    box((-0.0135, -0.0085, -0.0575), (0.005, 0.005, 0.022), STEEL),  # front lower captive lip (-y)
    box((0.0, +0.0085, -0.0660), (0.022, 0.005, 0.005), STEEL),  # bottom cradle ledge (+y)
    box((0.0, -0.0085, -0.0660), (0.022, 0.005, 0.005), STEEL),  # bottom cradle ledge (-y)
)

RING_SLIDE = Joint(
    "ring_slide",
    "slide",
    axis="y",
    at=(0.0, RING_Y0, ROD_Z),
    low=-RING_SLIDE_MAX_Y - RING_Y0,
    high=+RING_SLIDE_MAX_Y - RING_Y0,
    spring=0.0,
    hold=0.0,
    damping=0.25,
    mass=0.015,
    shapes=(dataclasses.replace(cyl((0.0, 0.0, 0.0), 0.006, 0.012, CHROME, axis="y"), visible=False),),
)
RING_SWING = Joint(
    "ring_swing",
    "hinge",
    axis="y",
    parent="ring_slide",
    at=(0.0, 0.0, 0.0),
    low=-RING_SWING_LIMIT,
    high=+RING_SWING_LIMIT,
    spring=0.0,
    hold=0.0,
    damping=0.002,
    mass=0.025,
    friction=0.75,
    shapes=RING_SWING_SHAPES,
)

CLOSET = Mech(
    "closet",
    pos=(ROD_X, 0.0, TOP),
    mass=None,
    friction=0.8,
    shapes=CLOSET_BASE_SHAPES,
    joints=(RING_SLIDE, RING_SWING),
)

# ---------------------------------------------------------------------------------------------------------------------
# HANGER: free Mech modelling Nahanco 60017BEC natural maple 17 in anti-theft ball-top hotel suit hanger.
# Local origin (0, 0, 0) is the center of the 16 mm chrome ball. Spawn shapes incorporate the 12 deg backward recline
# against the valet tray's rear pillars so the hanger spawns at rest with <0.5 mm settle drop and no tipping.
START_LEAN_DEG = 12.0
START_LEAN_RAD = math.radians(START_LEAN_DEG)
SHOULDER_SLOPE_DEG = 20.0
SHOULDER_SLOPE_RAD = math.radians(SHOULDER_SLOPE_DEG)

BALL_R = 0.008  # 16 mm diameter chrome ball (3.0 mm/side clearance in 22 mm window/pocket)
STEM_R = 0.003  # 6 mm diameter chrome stem (3.0 mm/side clearance in 12 mm lower slot)
WOOD_T = 0.013  # 1/2 in (13 mm) flat maple body thickness
SH_H = 0.022  # 22 mm shoulder height perpendicular to slope
SH_LEN = 0.204  # each shoulder bar length -> 431.2 mm (17 in) overall span
SH_YC = 0.116
SH_ZC = -0.055 - (SH_YC - 0.020) * math.tan(SHOULDER_SLOPE_RAD)
PANT_BAR_Z = -0.128
PANT_BAR_R = 0.005
PANT_BAR_LEN = 0.376

# Rotation from upright hanger reference frame to spawn (12 deg reclined) frame
R_LEAN = _ry(START_LEAN_RAD)
STEM_AXIS_LOCAL = R_LEAN @ np.array([0.0, 0.0, 1.0])  # unit vector pointing straight up along stem when upright
UPRIGHT_ROT = R_LEAN.T  # world rotation of HANGER when hanging vertically upright (stem along +Z)


def _hanger_shapes(R_base: np.ndarray) -> tuple:
    def xb(at, size, color, R_extra=np.eye(3)):
        return box(tuple(float(v) for v in (R_base @ np.array(at))), size, color, tilt=_quat_wxyz(R_base @ R_extra))

    def xc(at, r, h, color, axis="z"):
        return cyl(tuple(float(v) for v in (R_base @ np.array(at))), r, h, color, axis=axis, tilt=_quat_wxyz(R_base))

    def xball(at, r, color):
        return ball(tuple(float(v) for v in (R_base @ np.array(at))), r, color, tilt=_quat_wxyz(R_base), sides=24, rows=8)

    return (
        xball((0.0, 0.0, 0.0), BALL_R, CHROME),
        xc((0.0, 0.0, -0.025), STEM_R, 0.044, CHROME, axis="z"),
        xb((0.0, 0.0, -0.055), (WOOD_T, 0.048, 0.026), MAPLE),
        xb((0.0, +SH_YC, SH_ZC), (WOOD_T, SH_LEN, SH_H), MAPLE, _rx(-SHOULDER_SLOPE_RAD)),
        xb((0.0, -SH_YC, SH_ZC), (WOOD_T, SH_LEN, SH_H), MAPLE, _rx(+SHOULDER_SLOPE_RAD)),
        xc((0.0, 0.0, PANT_BAR_Z), PANT_BAR_R, PANT_BAR_LEN, MAPLE, axis="y"),
    )


HANGER_SHAPES = _hanger_shapes(R_LEAN)
HANGER_UPRIGHT_SHAPES = _hanger_shapes(np.eye(3))  # used by check.py clearance assertions

# Start pose of HANGER ball center (plain floats, resting 0.4 mm above valet tray base, 0.4 mm ahead of rear pillars)
BALL_START_X = 0.245
BALL_START_Y = -0.040
TRAY_BASE_H = 0.008
BALL_START_Z = round(TOP + TRAY_BASE_H + 0.13356 + 0.0004, 5)  # 0.88946 m (TOP + 0.14196 m)

HANGER = Mech(
    "hanger",
    pos=(BALL_START_X, BALL_START_Y, BALL_START_Z),
    mass=0.140,
    friction=0.80,
    shapes=HANGER_SHAPES,
)

# ---------------------------------------------------------------------------------------------------------------------
# VALET_TRAY: fixed charcoal wooden counter tray supporting the hanger at 12 deg lean with an open center bay behind
# the right shoulder grasp point.
TRAY_X0 = 0.1975  # front edge of tray
TRAY_X1 = 0.2540  # back edge of tray (56.5 mm deep, 100 mm clear of closet stiles)
TRAY_W = 0.440  # 440 mm wide along y (covers full 431 mm hanger span)
LIP_T = 0.012
LIP_H = 0.016
LIP_BACK_X = 0.2097  # 0.8 mm clear ahead of hanger's bottom front edge
PILLAR_FRONT_X = 0.2342  # 0.4 mm clear behind hanger's reclined outer shoulders
PILLAR_D = 0.018
PILLAR_W = 0.024
PILLAR_H = 0.054
PILLAR_Y_OFF = 0.145  # pillars at y = BALL_START_Y +- 0.145 -> 266 mm open center bay behind grasp

TRAY_CX = (TRAY_X0 + TRAY_X1) / 2.0
VALET_TRAY = Mech(
    "valet_tray",
    pos=(TRAY_CX, BALL_START_Y, TOP),
    mass=None,
    friction=0.85,
    shapes=(
        box((0.0, 0.0, TRAY_BASE_H / 2.0), (TRAY_X1 - TRAY_X0, TRAY_W, TRAY_BASE_H), CHARCOAL),
        box((LIP_BACK_X - LIP_T / 2.0 - TRAY_CX, 0.0, TRAY_BASE_H + LIP_H / 2.0), (LIP_T, TRAY_W, LIP_H), CHARCOAL),
        box(
            (PILLAR_FRONT_X + PILLAR_D / 2.0 - TRAY_CX, +PILLAR_Y_OFF, TRAY_BASE_H + PILLAR_H / 2.0),
            (PILLAR_D, PILLAR_W, PILLAR_H),
            CHARCOAL,
        ),
        box(
            (PILLAR_FRONT_X + PILLAR_D / 2.0 - TRAY_CX, -PILLAR_Y_OFF, TRAY_BASE_H + PILLAR_H / 2.0),
            (PILLAR_D, PILLAR_W, PILLAR_H),
            CHARCOAL,
        ),
    ),
)

OBJECTS = [CLOSET, VALET_TRAY, HANGER]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
POCKET_X_E = 0.009  # ball center inside 22 mm deep pocket (Latch E)
POCKET_Y_E = 0.009
POCKET_Z_E = (-0.0630, -0.0240)
ENTER_STEPS = 6  # 0.3 s at 20 Hz

SEAT_X = 0.008  # ball seated in lower captive collar
SEAT_Y = 0.006
SEAT_Z = (-0.0620, -0.0545)  # resting z_rel is -0.05836 m; top at -0.0545 is strictly at/below window bottom (-0.0465)

MIN_CLEAR_TABLE = 0.035  # 3.5 cm daylight gap under lowest point of hanger
MIN_STILE_CLEAR = 0.005  # 5 mm clearance to each closet side wall
TILT_FULL_DEG = 18.0
TILT_PART_DEG = 35.0
SWING_FULL_DEG = 15.0
SWING_PART_DEG = 30.0

DROP_BELOW_TABLE = 0.020  # lowest point < TOP - 2 cm for 0.5 s latches D (dropped off table)
DROP_STEPS = 10  # 0.5 s at 20 Hz

STILL_LIN = 0.015  # 1.5 cm/s
STILL_ANG = 0.15  # 0.15 rad/s
STILL_STEPS = 40  # 2.0 s continuous stillness & release
RELEASE_CLEAR = 0.030  # 3.0 cm from all finger pads to both hanger and ring

_ST: dict = {}
OFFLINE_RELEASED = True  # offline checks/replays have no live env; check.py sets False for the "still held" test


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(
        last=None,
        enter_streak=0,
        entered=False,
        drop_streak=0,
        dropped=False,
        still_streak=0,
        prev_slide=None,
        prev_swing=None,
        last_diag={},
    )


_reset()


def _ring_body_pose(closet_state) -> tuple[np.ndarray, np.ndarray]:
    bodies = getattr(closet_state, "bodies", None) or {}
    if "ring_swing" in bodies:
        R, p = bodies["ring_swing"]
        return np.asarray(R, dtype=float), np.asarray(p, dtype=float)
    return mechkit.body_poses(CLOSET, closet_state)["ring_swing"]


def _ring_released(closet_state) -> bool:
    if mechkit._ENV.get("env") is None:
        return bool(OFFLINE_RELEASED)
    try:
        pts = mechkit.finger_points()
        R_ring, p_ring = _ring_body_pose(closet_state)
        q = (np.asarray(pts, dtype=float) - p_ring) @ R_ring
        d = np.maximum(np.maximum(RING_LO - q, q - RING_HI), 0.0)
        return float(np.linalg.norm(d, axis=1).min()) >= RELEASE_CLEAR
    except Exception:  # noqa: BLE001
        return False


def _both_released(objects) -> bool:
    if mechkit._ENV.get("env") is None:
        return bool(OFFLINE_RELEASED)
    return bool(mechkit.released(objects["hanger"], clearance=RELEASE_CLEAR)) and _ring_released(objects["closet"])


def _eval_geometry(objects) -> dict:
    h = objects["hanger"]
    c = objects["closet"]
    R_ring, p_ring = _ring_body_pose(c)
    p_ball = np.asarray(getattr(h, "origin", h.pos), dtype=float)
    rel = R_ring.T @ (p_ball - p_ring)
    x_rel, y_rel, z_rel = float(rel[0]), float(rel[1]), float(rel[2])

    R_h = h.rotation() if hasattr(h, "rotation") else belimits._rot(h.quat_xyzw)
    stem_up = R_h @ STEM_AXIS_LOCAL
    tilt = math.degrees(math.acos(float(np.clip(stem_up[2], -1.0, 1.0))))
    swing_rad = float(c.joints.get("ring_swing", 0.0))
    slide_m = float(c.joints.get("ring_slide", 0.0))
    swing_deg = abs(math.degrees(swing_rad))

    low_z = float(h.low[2])
    table_clear = low_z - TOP
    stile_clear = min(STILE_Y - float(h.high[1]), float(h.low[1]) - (-STILE_Y))

    in_pocket = abs(x_rel) <= POCKET_X_E and abs(y_rel) <= POCKET_Y_E and POCKET_Z_E[0] <= z_rel <= POCKET_Z_E[1]
    in_seat = abs(x_rel) <= SEAT_X and abs(y_rel) <= SEAT_Y and SEAT_Z[0] <= z_rel <= SEAT_Z[1]
    suspended = table_clear >= MIN_CLEAR_TABLE

    return {
        "rel": (x_rel, y_rel, z_rel),
        "tilt_deg": tilt,
        "swing_deg": swing_deg,
        "slide_m": slide_m,
        "swing_rad": swing_rad,
        "low_z": low_z,
        "table_clear": table_clear,
        "stile_clear": stile_clear,
        "in_pocket": bool(in_pocket),
        "in_seat": bool(in_seat),
        "suspended": bool(suspended),
    }


def _update(objects) -> dict:
    if objects is _ST["last"]:
        return _ST["last_diag"]
    _ST["last"] = objects

    d = _eval_geometry(objects)
    h = objects["hanger"]

    # Latch E: ball inside keyhole pocket for >= 0.3 s
    if d["in_pocket"]:
        _ST["enter_streak"] += 1
        if _ST["enter_streak"] >= ENTER_STEPS:
            _ST["entered"] = True
    else:
        _ST["enter_streak"] = 0

    # Latch D: hanger lowest point < TOP - 2 cm for >= 0.5 s
    if d["low_z"] < TOP - DROP_BELOW_TABLE:
        _ST["drop_streak"] += 1
        if _ST["drop_streak"] >= DROP_STEPS:
            _ST["dropped"] = True
    else:
        _ST["drop_streak"] = 0

    # Ring velocities from finite difference of chained joint states
    if _ST["prev_slide"] is None:
        ring_lin_v, ring_ang_v = 0.0, 0.0
    else:
        ring_lin_v = abs(d["slide_m"] - _ST["prev_slide"]) / CONTROL_DT
        ring_ang_v = abs(d["swing_rad"] - _ST["prev_swing"]) / CONTROL_DT
    _ST["prev_slide"] = d["slide_m"]
    _ST["prev_swing"] = d["swing_rad"]

    h_lin_v = float(np.linalg.norm(h.lin_vel))
    h_ang_v = float(np.linalg.norm(h.ang_vel))
    still_now = (
        h_lin_v < STILL_LIN
        and h_ang_v < STILL_ANG
        and ring_lin_v < STILL_LIN
        and ring_ang_v < STILL_ANG
    )
    rel_now = _both_released(objects)

    if still_now and rel_now:
        _ST["still_streak"] += 1
    else:
        _ST["still_streak"] = 0

    d.update(
        entered=_ST["entered"],
        dropped=_ST["dropped"],
        still_now=still_now,
        released=rel_now,
        still_streak=_ST["still_streak"],
    )
    _ST["last_diag"] = d
    return d


def score(objects) -> float:
    """1.0: seated in lower captive collar, suspended >= 3.5 cm above table, tilt <= 18 deg, |ring_swing| <= 15 deg,
    stile clearance >= 5 mm, released (>= 3 cm clear of both hanger and ring), still (< 1.5 cm/s, < 0.15 rad/s) for
    2.0 s, and never dropped off the table.
    0.5: seated and suspended >= 3.5 cm, released and still for 2.0 s, never dropped, but tilt in 18..35 deg,
    |ring_swing| in 15..30 deg, or stile clearance < 5 mm.
    0.25: ball entered keyhole pocket for >= 0.3 s (Latch E), never dropped, released and still for 2.0 s, but not
    seated suspended within the 0.5 limits (e.g. resting back on tray/table or tilted > 35 deg).
    0.0: dropped off table (Latch D), still held, still moving, ball never entered pocket, draped on rod, or idle."""
    d = _update(objects)
    if d["dropped"] or not d["released"] or d["still_streak"] < STILL_STEPS:
        return 0.0
    if d["in_seat"] and d["suspended"]:
        if (
            d["tilt_deg"] <= TILT_FULL_DEG
            and d["swing_deg"] <= SWING_FULL_DEG
            and d["stile_clear"] >= MIN_STILE_CLEAR
        ):
            return 1.0
        if d["tilt_deg"] <= TILT_PART_DEG and d["swing_deg"] <= SWING_PART_DEG:
            return 0.5
    if d["entered"]:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------------
# Robot keep-out regions around the closet woodwork (matches the Bounded Embodiment envelope).
ARM_KEEP_OUT = [
    {"name": "closet back panel", "x": (0.430, 0.485), "y": (-0.390, 0.390), "z_top": TOP + 0.310},
    {"name": "closet left stile", "x": (0.345, 0.485), "y": (0.340, 0.390), "z_top": TOP + 0.310},
    {"name": "closet right stile", "x": (0.345, 0.485), "y": (-0.390, -0.340), "z_top": TOP + 0.310},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted bimanual demo (closedloop).
# Left arm catches and steadies the free-sliding, free-swinging ring from front-left (yaw -70 deg, pitch 90 deg so the
# wrist and link4 stay well in front of the closet rod while clamping the 32 mm cheek without squirt); right arm lifts
# the leaned wooden hanger high above the valet tray pillars before rotating upright, threads the 16 mm ball through
# the 22 x 22 mm upper window in the live ring frame, lowers the 6 mm stem into the 12 mm captive collar, releases
# straight back, and both arms park at READY.
demo = placeholder_demo

RING_GOAL_Y = RING_Y0  # catch and steady the ring plumb at its starting y (+4.5 cm; 8.9 cm clear of left stile)
LEFT_CHEEK_Z = TOP + ROD_Z - 0.056  # TOP + 0.194 m: lower +y cheek, clear of rod and hanger crown
R_LEFT = grip_rot(yaw=math.radians(-70.0), pitch=math.pi / 2.0, roll=0.0)  # front-left approach, all links clear of rod
LEFT_APP = R_LEFT[:, 2]  # unit approach direction (+0.342, -0.940, 0)
LEFT_TCP_LOCAL = np.array([+0.0005, 0.0265, -0.056], dtype=float)  # solid bite on +y cheek, clear of ball & stem path
CLOSE_LEFT = -0.013  # 26 mm commanded gap -> steady hold on the 33 mm angled cheek without corner squirt

GRASP_Y_LOCAL = -0.055  # 55 mm right of chrome stem on right wooden shoulder
GRASP_Z_UPRIGHT = -0.055 - (abs(GRASP_Y_LOCAL) - 0.020) * math.tan(SHOULDER_SLOPE_RAD)
GRASP_UPRIGHT_LOCAL = np.array([0.0, GRASP_Y_LOCAL, GRASP_Z_UPRIGHT], dtype=float)
GRASP_START_WORLD = np.array([BALL_START_X, BALL_START_Y, BALL_START_Z], dtype=float) + R_LEAN @ GRASP_UPRIGHT_LOCAL

R_RIGHT_UPRIGHT = grip_rot(yaw=0.0, pitch=math.pi / 2.0, roll=-math.pi / 2.0 + SHOULDER_SLOPE_RAD)
R_RIGHT_PICK = R_LEAN @ R_RIGHT_UPRIGHT  # approach perpendicular to 12 deg reclined hanger, pads flush on 20 deg shoulder

OPEN_LEFT = -0.032  # 64 mm gap (clears 32 mm cheek cleanly at 20 deg approach angle)
OPEN_RIGHT = -0.026  # 52 mm gap for pick inside valet tray
OPEN_RIGHT_RELEASE = -0.036  # 72 mm gap for clean straight-back release off wooden shoulder

WINDOW_Z_LOCAL = -0.0355  # vertical center of 22 x 22 mm upper keyhole window
SEAT_RELEASE_Z_LOCAL = -0.0572  # ~1.0 mm above exact -0.05821 resting height on bottom cradle ledges


def _right_tcp_for_ball(ball_world: np.ndarray, offset: np.ndarray | None = None) -> np.ndarray:
    """Right TCP target that places the upright hanger's ball center at ``ball_world``."""
    base = np.asarray(ball_world, dtype=float) + GRASP_UPRIGHT_LOCAL
    return base if offset is None else base + np.asarray(offset, dtype=float)


def _ball_error_offset(
    ctx,
    planned_ball_world: np.ndarray,
    cur_off: np.ndarray,
    gain: float = 0.40,
    clamp_x_push: bool = False,
    clamp_z_down: bool = False,
) -> np.ndarray:
    """Closed-loop ball-center position correction from live simulator state (no-op offline)."""
    if ctx.right.offline:
        return cur_off
    h = ctx.obj("hanger")
    actual_ball = np.asarray(h.origin, dtype=float)
    err = np.asarray(planned_ball_world, dtype=float) - actual_ball
    if clamp_x_push or clamp_z_down:
        c = ctx.obj("closet")
        R_ring, p_ring = _ring_body_pose(c)
        rel = R_ring.T @ (actual_ball - p_ring)
        if clamp_x_push and rel[0] >= 0.0005 and err[0] > 0.0:
            err[0] = 0.0
        if clamp_z_down and rel[2] <= SEAT_RELEASE_Z_LOCAL + 0.0005 and err[2] < 0.0:
            err[2] = 0.0
    new_off = cur_off + gain * err
    norm = float(np.linalg.norm(new_off))
    if norm > 0.035:
        new_off = new_off * (0.035 / norm)
    return new_off


def _ring_world_point(ctx, local_pt: tuple[float, float, float], fallback_y: float) -> np.ndarray:
    """World coordinates of a point in ring_swing's frame (live in sim, nominal offline)."""
    if ctx.right.offline:
        return np.array([ROD_X + local_pt[0], fallback_y + local_pt[1], TOP + ROD_Z + local_pt[2]], dtype=float)
    c = ctx.obj("closet")
    R_ring, p_ring = _ring_body_pose(c)
    return p_ring + R_ring @ np.asarray(local_pt, dtype=float)


def _move_hanger_ball_in_ring(
    ctx,
    local_start: tuple[float, float, float],
    local_end: tuple[float, float, float],
    fallback_y: float,
    duration: float,
    off_box: list[np.ndarray],
    gain: float = 0.20,
    clamp_x_push: bool = False,
    clamp_z_down: bool = False,
):
    """Smoothstep motion of the upright hanger ball in the live ring_swing frame with anti-windup ball tracking."""
    R = ctx.right
    p0 = np.asarray(local_start, dtype=float)
    p1 = np.asarray(local_end, dtype=float)
    steps = max(1, round(float(duration) / CONTROL_DT))
    for i in range(1, steps + 1):
        s = ik.smoothstep(i / steps)
        loc_goal = p0 + s * (p1 - p0)
        goal_ball = _ring_world_point(ctx, tuple(loc_goal), fallback_y)
        if not R.offline and i % 2 == 0:
            off_box[0] = _ball_error_offset(
                ctx,
                goal_ball,
                off_box[0],
                gain=gain,
                clamp_x_push=clamp_x_push,
                clamp_z_down=clamp_z_down,
            )
        R._set_cartesian(_right_tcp_for_ball(goal_ball, off_box[0]), R_RIGHT_UPRIGHT, R.finger)
        yield


def demo_script(ctx):
    """Bimanual demo: left arm catches and steadies the sliding/swinging chrome ring from +y; right arm grasps the
    wooden hanger's right shoulder in the valet tray, lifts clear above the tray pillars and uprights it, threads the
    16 mm ball through the 22 x 22 mm upper window, lowers the 6 mm stem into the 12 mm lower captive collar, releases
    both hands straight back, and parks both arms at READY."""
    L, R = ctx.left, ctx.right

    # Read live start poses (offline returns the fixed nominal start poses)
    left_grasp = _ring_world_point(ctx, tuple(LEFT_TCP_LOCAL), RING_Y0)

    if R.offline:
        grasp_pt = GRASP_START_WORLD.copy()
    else:
        h0 = ctx.obj("hanger")
        R_h0 = h0.rotation()
        grasp_pt = np.asarray(h0.origin, dtype=float) + R_h0 @ (R_LEAN @ GRASP_UPRIGHT_LOCAL)

    # 1) Move left arm to ring pre-grasp (+y cheek, wrist below rod) and right arm to hanger pre-grasp
    left_pre = left_grasp - 0.060 * LEFT_APP

    right_approach_dir = R_RIGHT_PICK[:, 2]  # unit vector pointing along leaned +x into hanger
    right_pre = grasp_pt - 0.055 * right_approach_dir
    right_entry = np.array([0.165, -0.145, TOP + 0.145], dtype=float)  # clean clearance waypoint from READY

    def _left_catch():
        # Swing well in front of the rod (x <= 0.28) before aligning to the -70 deg front-left cheek approach
        left_turn = np.array([0.275, 0.165, LEFT_CHEEK_Z + 0.020], dtype=float)
        R_left_mid = grip_rot(yaw=math.radians(-70.0), pitch=0.35 * math.pi, roll=0.0)
        yield from L.move(left_turn, 1.4, OPEN_LEFT, rot=R_left_mid)
        yield from L.move(left_pre, 1.6, OPEN_LEFT, rot=R_LEFT)
        yield from L.trim_to_measured(left_pre)
        l_sag = np.zeros(3) if L.offline else (L.pos - L.measured_pos)
        yield from L.move(left_grasp + l_sag, 1.4, OPEN_LEFT, rot=R_LEFT)
        yield from L.grip(CLOSE_LEFT, 0.8)
        yield from L.hold(0.35)
        if not L.holding(0.026):  # 32 mm cheek; retry once if missed
            yield from L.grip(OPEN_LEFT, 0.6)
            yield from L.move(left_pre + l_sag, 0.8, OPEN_LEFT, rot=R_LEFT)
            yield from L.trim_to_measured(left_pre)
            l_sag = np.zeros(3) if L.offline else (L.pos - L.measured_pos)
            yield from L.move(left_grasp + l_sag, 1.0, OPEN_LEFT, rot=R_LEFT)
            yield from L.grip(CLOSE_LEFT, 0.8)
            yield from L.hold(0.35)
        yield from L.hold(0.5)

    def _right_pick():
        yield from R.move(right_entry, 2.0, OPEN_RIGHT, rot=R_RIGHT_PICK)
        yield from R.move(right_pre, 1.6, OPEN_RIGHT, rot=R_RIGHT_PICK)
        yield from R.trim_to_measured(right_pre)
        r_sag = np.zeros(3) if R.offline else (R.pos - R.measured_pos)
        yield from R.move(grasp_pt + r_sag, 1.4, OPEN_RIGHT, rot=R_RIGHT_PICK)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.35)
        if not R.holding(0.015):  # 22 mm wooden shoulder; retry once if missed
            yield from R.grip(OPEN_RIGHT, 0.6)
            yield from R.move(right_pre + r_sag, 0.8, OPEN_RIGHT, rot=R_RIGHT_PICK)
            yield from R.trim_to_measured(right_pre)
            r_sag = np.zeros(3) if R.offline else (R.pos - R.measured_pos)
            yield from R.move(grasp_pt + r_sag, 1.0, OPEN_RIGHT, rot=R_RIGHT_PICK)
            yield from R.grip(FINGER_CLOSED, 0.8)
            yield from R.hold(0.35)
        # First lift 95 mm straight up while still leaned 12 deg so the hanger clears the 16 mm front lip cleanly
        # (+0.84 mm gap) and rises +43 mm above the 62 mm valet tray rear pillars, THEN pitch upright high above the tray.
        lift_clear = grasp_pt + r_sag + np.array([0.0, 0.0, 0.095], dtype=float)
        yield from R.move(lift_clear, 1.5, FINGER_CLOSED, rot=R_RIGHT_PICK)
        lift_upright = lift_clear + np.array([0.0, 0.0, 0.005], dtype=float)
        yield from R.move(lift_upright, 1.0, FINGER_CLOSED, rot=R_RIGHT_UPRIGHT)
        yield from R.hold(0.25)

    yield from ctx.both(_left_catch(), _right_pick())

    # 2) Carry hanger smoothly to pre-insert (32 mm ahead of the live keyhole window), staying ~38 mm over tray pillars
    pre_local = (-0.032, 0.0, WINDOW_Z_LOCAL)
    pocket_local = (0.000, 0.0, WINDOW_Z_LOCAL)
    seat_local = (0.000, 0.0, SEAT_RELEASE_Z_LOCAL)

    if R.offline:
        off_box = [np.zeros(3, dtype=float)]
    else:
        actual_ball0 = np.asarray(ctx.obj("hanger").origin, dtype=float)
        off0 = R.pos - GRASP_UPRIGHT_LOCAL - actual_ball0
        n0 = float(np.linalg.norm(off0))
        if n0 > 0.035:
            off0 = off0 * (0.035 / n0)
        off_box = [off0]

    ball_pre_insert = _ring_world_point(ctx, pre_local, RING_GOAL_Y)
    yield from R.move(_right_tcp_for_ball(ball_pre_insert, off_box[0]), 2.2, FINGER_CLOSED, rot=R_RIGHT_UPRIGHT)
    yield from R.hold(0.35)
    # Trim ball center to exact live pre-insert alignment (cancels wrist sag and shoulder grip offset)
    for _ in range(20):
        live_pre = _ring_world_point(ctx, pre_local, RING_GOAL_Y)
        off_box[0] = _ball_error_offset(ctx, live_pre, off_box[0], gain=0.38)
        R._set_cartesian(_right_tcp_for_ball(live_pre, off_box[0]), R_RIGHT_UPRIGHT, FINGER_CLOSED)
        yield
    yield from R.hold(0.25)

    # 3) Thread the 16 mm ball smoothly along +x through the 22 x 22 mm upper window into pocket center (~16 mm/s)
    yield from _move_hanger_ball_in_ring(
        ctx, pre_local, pocket_local, RING_GOAL_Y, 2.0, off_box, gain=0.22, clamp_x_push=True
    )
    yield from R.hold(0.5)  # Latch E cleanly confirmed inside pocket

    # 4) Lower the 6 mm stem 22 mm down the 12 mm lower captive slot until ball sits ~1 mm above bottom cradle ledges
    yield from _move_hanger_ball_in_ring(
        ctx, pocket_local, seat_local, RING_GOAL_Y, 1.8, off_box, gain=0.18, clamp_x_push=True, clamp_z_down=True
    )
    yield from R.hold(0.4)

    # 5) Release right arm first: open wide and back straight out along -x (zero dy/dz so open pads clear sloped wood)
    yield from R.grip(OPEN_RIGHT_RELEASE, 0.8)
    yield from R.hold(0.45)
    right_retreat = R.pos + np.array([-0.085, 0.0, 0.0], dtype=float)
    yield from R.move(right_retreat, 1.5, OPEN_RIGHT_RELEASE, rot=R_RIGHT_UPRIGHT)
    right_park_via = np.array([0.240, -0.230, TOP + 0.230], dtype=float)
    yield from R.move(right_park_via, 1.6, FINGER_OPEN, rot=grip_rot(yaw=0.0, pitch=0.35, roll=0.0))

    # 6) Release left arm smoothly off the ring cheek purely along -LEFT_APP (staying clear of rod and hanger)
    yield from L.grip(OPEN_LEFT, 0.8)
    yield from L.hold(0.35)
    left_retreat = L.pos - 0.075 * LEFT_APP
    yield from L.move(left_retreat, 1.5, OPEN_LEFT, rot=R_LEFT)
    left_park_via = np.array([0.265, 0.215, TOP + 0.255], dtype=float)
    yield from L.move(left_park_via, 1.5, FINGER_OPEN, rot=grip_rot(yaw=-math.pi / 4.0, pitch=0.35, roll=0.0))

    # 7) Park both arms cleanly at READY (open grippers, well clear of closet and hanger) and hold for settle + SUCCESS_HOLD_S
    yield from ctx.both(L.home(2.5), R.home(2.5))
    yield from ctx.both(L.hold(13.0), R.hold(13.0))
