"""On the station table: a 14 inch laptop (silver, 326 x 225 mm base) stands open, turned so its screen faces away from
the robot and its hinge is toward it; the lid is open 110 degrees, so it leans 20 degrees toward the arms, top edge at
x 0.222, 21 cm above the table. A black webcam (C920 class: a 94 mm wide pill-shaped body on a hinged mounting clip)
stands on the table in front of the left arm, on its clip's front lip and its folded-out rear foot, lens toward +x.

The task: clip the webcam onto the middle of the screen's top edge, lens facing the keyboard, with the clip's front
lip hanging in front of the screen and its hinged rear foot folded against the lid's back, then let go. The lid is held
only by its friction hinge (a detent spring here), so pushing it, or pressing the camera down off the lid line, moves
it; a foot left open lets the camera tip back on the edge.

Requires -E object_jitter=0 (the laptop is fixed in place, and the demo plans from the fixed layout and the live
webcam, foot and lid readings).

Run it:  inspect-robots run --embodiment yam_arena -E task=clip_a_webcam_onto_a_laptop_screen.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "Clip the webcam onto the middle of the laptop screen's top edge, lens toward the
           keyboard, foot folded against the back, then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401  (shared kit; the checks here are the clip's own geometry)
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, on_reset  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the demo has parked by then
CONTROL_DT = 0.05  # s per control step (-E control_hz=20)

SILVER = (0.78, 0.79, 0.81)  # anodised aluminium / silver plastic laptop shell
DECK = (0.10, 0.10, 0.11)  # black keyboard deck and display
BLACK = (0.07, 0.07, 0.08)  # webcam ABS
RING = (0.80, 0.80, 0.82)  # the lens's silver ring
LENS = (0.02, 0.03, 0.06)

# ---------------------------------------------------------------------------------------------------------------------
# Laptop (fixed): after the Acer Chromebook 314 CB314-3H, 326.4 x 225 x 20 mm closed. Station frame, metres.
HINGE = (0.295, 0.0, TOP + 0.010)  # lid hinge axis (along y), 5 mm inside the base's near edge
BASE_SIZE = (0.225, 0.3264, 0.013)
LID_T, LID_W, LID_L = 0.007, 0.3264, 0.213  # lid 7 mm thick at its top edge, 213 mm hinge to top edge
LID_MASS = 0.45
OPEN_DEG = 110.0  # lid opening at the start: leans 20 deg past upright toward the robot
LEAN0 = math.radians(OPEN_DEG - 90.0)  # rotation of the lid about -y from upright
LID_SPRING = 60.0  # N m/rad, the friction hinge's detent spring
LID_STEP_DEG = 1.5  # detent spacing
LID_STOPS_DEG = (90.0, 135.0)  # opening range
# Lid joint value: + closes the lid (top edge toward the keyboard). Opening angle = OPEN_DEG - degrees(value).
LID_LOW = math.radians(OPEN_DEG - LID_STOPS_DEG[1])  # -25 deg (open to 135)
LID_HIGH = math.radians(OPEN_DEG - LID_STOPS_DEG[0])  # +20 deg (closed to 90)
LID_DETENTS = tuple(float(math.radians(LID_STEP_DEG * k)) for k in range(-16, 14))  # -24 .. +19.5 deg, 30 stops


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _qy(a: float) -> tuple[float, float, float, float]:
    return (float(math.cos(a / 2)), 0.0, float(math.sin(a / 2)), 0.0)


def _f3(v) -> tuple[float, float, float]:
    return (float(v[0]), float(v[1]), float(v[2]))


TILT = -LEAN0  # rotation about +y that takes upright to the lid's lean (top toward -x)
R_LID0 = _ry(TILT)
U0 = R_LID0 @ np.array([0.0, 0.0, 1.0])  # along the lid, hinge to top edge: (-0.342, 0, 0.940)
N0 = R_LID0 @ np.array([1.0, 0.0, 0.0])  # out of the screen face, toward the keyboard: (0.940, 0, 0.342)
EDGE = np.asarray(HINGE) + LID_L * U0  # middle of the lid's top face, the throat's seat: (0.222, 0, TOP + 0.210)

_LID_SHAPES = [
    box(_f3(R_LID0 @ [0.0, 0.0, LID_L / 2]), (LID_T, LID_W, LID_L), SILVER, tilt=_qy(TILT)),  # lid shell
    box(_f3(R_LID0 @ [LID_T / 2 + 0.0002, 0.0, LID_L / 2 + 0.004]), (0.0004, 0.300, 0.188), DECK, tilt=_qy(TILT)),
    cyl((0.0, 0.0, 0.0), 0.005, 0.28, SILVER, axis="y"),  # hinge barrel
]
_LID_SHAPES[1] = mechkit.dataclasses.replace(_LID_SHAPES[1], collide=False)  # display glass, look only
_BASE_SHAPES = [
    box((BASE_SIZE[0] / 2 - 0.005, 0.0, -0.0035), BASE_SIZE, SILVER),  # base, 13 mm, top at TOP + 0.013
    box((0.065, 0.0, 0.0035), (0.110, 0.280, 0.001), DECK),  # keyboard deck, near the hinge (look only)
]
_BASE_SHAPES[1] = mechkit.dataclasses.replace(_BASE_SHAPES[1], collide=False)
LAPTOP = Mech(
    "laptop", pos=_f3(HINGE), mass=None, friction=0.8, shapes=_BASE_SHAPES,
    joints=[Joint("lid", "hinge", axis="y", at=(0.0, 0.0, 0.0), low=float(LID_LOW), high=float(LID_HIGH),
                  spring=LID_SPRING, damping=1.0, mass=LID_MASS, detents=LID_DETENTS, shapes=_LID_SHAPES)],
)
BASE_RECT = ((HINGE[0] - 0.005, HINGE[0] - 0.005 + BASE_SIZE[0]), (-BASE_SIZE[1] / 2, BASE_SIZE[1] / 2))
BASE_TOP = TOP + 0.013

# ---------------------------------------------------------------------------------------------------------------------
# Webcam (free): after the Logitech C920 (C920e, 960-001360), 94 x 71 x 43.3 mm with its clip, 162 g. Its frame is the
# clip throat's top: the point that sits on the middle of the lid's top face. Clip parts are laid leaning 20 deg (the
# lid's lean at 110 deg open) with the body level, as the camera's tilt hinge holds them. Clip coordinates (n, y, u):
# n out of the screen (toward the lens side), u up the lid.
CAM_MASS, FOOT_MASS = 0.142, 0.020
BODY_C = (-0.004, 0.0, 0.0245)  # body centre: 4 mm behind and 24.5 mm above the throat top
BODY_D, BODY_H, BODY_FLAT = 0.024, 0.029, 0.065  # 24 mm deep, 29 mm tall, 65 mm flat plus two r 14.5 mm ends
BODY_W = BODY_FLAT + BODY_H  # 94 mm
HEAD = ((-0.0085, 0.0105), 0.040, (0.0, 0.008))  # n range, width, u range: rests on the lid's top face
LIP = ((0.0045, 0.0085), 0.040, (-0.022, 0.0))  # front lip with its rubber pad: inner face 4.5 mm in front of the throat
# centre (a 1 mm gap to the screen face), so the camera hangs back onto it only 2 to 3 deg once let go
FOOT_HINGE = (-0.007, 0.001)  # (n, u) of the foot's hinge axis, at the head's back edge
FOOT_T, FOOT_W, FOOT_L = 0.005, 0.036, 0.052  # plate; folded, its inner face is 4.5 mm behind the throat centre
FOOT_TOP = 0.001  # plate's top end above the hinge (u), folded
FOOT_SPRING = 3.0  # N m/rad, the foot's stiff hinge as a detent spring
FOOT_STEP_DEG = 3.0
FOOT_STOPS_DEG = (-8.0, 140.0)  # foot opening range (0 = folded flat, + swings away from the lid back)


def _clip(n: float, u: float, y: float = 0.0) -> np.ndarray:
    """Webcam frame point from clip coordinates (n out of the screen, u up the lid, y along the edge)."""
    return R_LID0 @ np.array([n, y, u])


def foot_dir(open_deg: float) -> np.ndarray:
    """Unit vector along the foot from its hinge toward its free end, in the webcam frame, at this opening."""
    return _ry(TILT + math.radians(open_deg)) @ np.array([0.0, 0.0, -1.0])


def foot_out(open_deg: float) -> np.ndarray:
    """Unit outward normal of the foot's outer face (away from the lid back when folded), in the webcam frame."""
    return _ry(TILT + math.radians(open_deg)) @ np.array([-1.0, 0.0, 0.0])


_FH = _clip(*FOOT_HINGE)
_LIP_BOTTOM = _clip(LIP[0][0], LIP[2][0])  # inner bottom corner of the lip (its lowest point, leaning 20 deg)


def _foot_end(open_deg: float) -> np.ndarray:
    """Lowest point of the foot's free end at this opening (webcam frame)."""
    d, o = foot_dir(open_deg), foot_out(open_deg)
    ends = [_FH + (FOOT_L - FOOT_TOP) * d + s * FOOT_T / 2 * o for s in (-1.0, 1.0)]
    return min(ends, key=lambda p: p[2])


def _standing_open() -> float:
    """Foot opening (deg) at which the foot's end is as low as the lip's: the camera stands level on both."""
    lo, hi = 60.0, 120.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if _foot_end(mid)[2] < _LIP_BOTTOM[2]:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


FOOT_START = float(FOOT_STEP_DEG * round(_standing_open() / FOOT_STEP_DEG))  # 93 deg: on a detent, rest of its joint
FOOT_LOW = math.radians(FOOT_STOPS_DEG[0] - FOOT_START)
FOOT_HIGH = math.radians(FOOT_STOPS_DEG[1] - FOOT_START)
_k0 = math.ceil((FOOT_STOPS_DEG[0] + 2.0 - FOOT_START) / FOOT_STEP_DEG)
_k1 = math.floor((FOOT_STOPS_DEG[1] - 2.0 - FOOT_START) / FOOT_STEP_DEG)
FOOT_DETENTS = tuple(float(math.radians(FOOT_STEP_DEG * k)) for k in range(_k0, _k1 + 1))
FOLDED_VALUE = math.radians(-FOOT_START)  # foot joint value when folded flat (open 0)


def foot_open_deg(value: float) -> float:
    return FOOT_START + math.degrees(value)


_qc = _qy(TILT)
_CAM_SHAPES = [
    box(_f3(np.asarray(BODY_C)), (BODY_D, BODY_FLAT, BODY_H), BLACK),  # body, flat middle
    cyl(_f3(np.asarray(BODY_C) + [0, BODY_FLAT / 2, 0]), BODY_H / 2, BODY_D, BLACK, axis="x", sides=16),  # round ends
    cyl(_f3(np.asarray(BODY_C) + [0, -BODY_FLAT / 2, 0]), BODY_H / 2, BODY_D, BLACK, axis="x", sides=16),
    box(_f3(_clip(sum(HEAD[0]) / 2, sum(HEAD[2]) / 2)), (HEAD[0][1] - HEAD[0][0], HEAD[1], HEAD[2][1] - HEAD[2][0]),
        BLACK, tilt=_qc),  # clip head, on the edge
    box(_f3(_clip(sum(LIP[0]) / 2, sum(LIP[2]) / 2)), (LIP[0][1] - LIP[0][0], LIP[1], LIP[2][1] - LIP[2][0]),
        BLACK, tilt=_qc),  # front lip
    cyl((-0.002, 0.0, 0.0095), 0.005, 0.030, BLACK, axis="y", sides=12),  # tilt hinge barrel, body to clip head
    cyl(_f3(np.asarray(BODY_C) + [BODY_D / 2 + 0.0008, 0, 0]), 0.011, 0.0016, RING, axis="x"),  # lens ring (look)
    cyl(_f3(np.asarray(BODY_C) + [BODY_D / 2 + 0.0018, 0, 0]), 0.008, 0.0010, LENS, axis="x"),  # lens glass (look)
]
_CAM_SHAPES[6] = mechkit.dataclasses.replace(_CAM_SHAPES[6], collide=False)
_CAM_SHAPES[7] = mechkit.dataclasses.replace(_CAM_SHAPES[7], collide=False)
_qf = _qy(TILT + math.radians(FOOT_START))
_FOOT_SHAPES = [box(_f3(foot_dir(FOOT_START) * (FOOT_L / 2 - FOOT_TOP)), (FOOT_T, FOOT_W, FOOT_L), BLACK, tilt=_qf)]

# Start: standing on the lip's bottom and the foot's end, body level, lens toward +x, body centre near (0.175, 0.19).
START_XY = (0.179, 0.19)
WEBCAM = Mech(
    "webcam", pos=(float(START_XY[0]), float(START_XY[1]), float(TOP)), mass=CAM_MASS, friction=1.0, shapes=_CAM_SHAPES,
    joints=[Joint("foot", "hinge", axis="y", at=_f3(_FH), low=float(FOOT_LOW), high=float(FOOT_HIGH),
                  spring=FOOT_SPRING, damping=0.005, mass=FOOT_MASS, detents=FOOT_DETENTS, shapes=_FOOT_SHAPES)],
)

_low = float(WEBCAM.points({"foot": 0.0})[:, 2].min())  # lowest collider point, webcam frame
START = (float(START_XY[0]), float(START_XY[1]), float(TOP - _low + 0.0003))
WEBCAM = mechkit.dataclasses.replace(WEBCAM, pos=START)

OBJECTS = [LAPTOP, WEBCAM]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
EDGE_TOL = 0.003  # throat top within 3 mm of the lid's top face, along the lid
STRADDLE = 0.004  # throat centre within 4 mm of the lid's mid-plane (lip in front, foot behind)
LIP_BELOW = 0.010  # lip's bottom at least 10 mm below the top edge, on the screen side
YAW_TOL = 10.0  # deg, lens square to the screen
CENTRE_TOL = 0.020  # body centre within 20 mm of the lid's centre line (y 0)
FOLD_TOL = 6.0  # deg, foot within this of lying flat on the lid back (its end within about 6 mm of it)
TILT_BANDS = (4.0, 10.0, 20.0)  # deg: full credit, half, quarter (quarter for a camera hooked on with its foot open)
LID_LIMIT = 2.0  # deg off its start
LID_STEPS = 10  # 0.5 s at 20 Hz
DROP_STEPS = 10  # 0.5 s resting on the table or the keyboard deck after it was lifted
LIFTED = 0.030  # m above its start: it has been lifted
STILL_STEPS = 20  # 1 s
STILL_D, STILL_A = 0.002, 2.0  # m and deg: the most the webcam may move over that last 1 s (measured poses, not the
# solver's instantaneous velocities, which jitter on a resting articulation in PhysX)
_ST: dict = {}


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, lid_n=0, lid_bad=False, lifted=False, drop_n=0, dropped=False, still_n=0, step=0, poses=[])


_reset()


def lid_frame(lid_value: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Hinge point, unit vector up the lid and unit normal out of the screen, at this lid joint value (station)."""
    r = _ry(TILT + float(lid_value))
    return np.asarray(HINGE, float), r @ np.array([0.0, 0.0, 1.0]), r @ np.array([1.0, 0.0, 0.0])


def cam_pose(s) -> dict:
    """The webcam's parts in the station frame from its state: throat point, clip axes, body centre, lens direction,
    lip bottom, foot direction."""
    rot = s.rotation()
    org = np.asarray(s.origin, float)
    rc = rot @ R_LID0
    fopen = foot_open_deg(float(s.joints.get("foot", 0.0)))
    return {"throat": org, "u": rc @ [0, 0, 1.0], "n": rc @ [1.0, 0, 0], "body": rot @ np.asarray(BODY_C) + org,
            "front": rot @ np.array([1.0, 0, 0]), "up": rot @ np.array([0, 0, 1.0]),
            "lip": rot @ _LIP_BOTTOM + org, "foot_dir": rot @ foot_dir(fopen), "foot_open": fopen}


def tilt_deg(c) -> float:
    """Angle of the body's top from level (deg): what an angle gauge on the top reads."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(c["up"][2])))))


def yaw_err_deg(c) -> float:
    f = c["front"]
    return abs(math.degrees(math.atan2(f[1], f[0])))


def foot_to_lid_deg(c, lid_value: float) -> float:
    """Angle (deg) between the foot and the lid back; + when the free end stands away from the lid."""
    _, u, n = lid_frame(lid_value)
    d = c["foot_dir"]
    return math.degrees(math.atan2(-float(d @ n), -float(d @ u)))


def seated(c, lid_value: float) -> bool:
    """Head on the top edge, throat straddling the lid, lip hanging 10 mm or more below the edge on the screen side,
    lens square to the screen and the body over the middle of the edge."""
    h, u, n = lid_frame(lid_value)
    t = c["throat"] - h
    lip = c["lip"] - h
    return bool(abs(float(t @ u) - LID_L) <= EDGE_TOL and abs(float(t @ n)) <= STRADDLE
                and float(lip @ n) > LID_T / 2 and float(lip @ u) <= LID_L - LIP_BELOW
                and yaw_err_deg(c) <= YAW_TOL and abs(float(c["body"][1]) - HINGE[1]) <= CENTRE_TOL)


def _released(s) -> bool:
    if mechkit._ENV.get("env") is None:  # offline (check.py, the MuJoCo replay): no fingers to read
        return True
    return bool(mechkit.released(s, clearance=0.03))


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    lid = math.degrees(float(objects["laptop"].joints.get("lid", 0.0)))
    _ST["lid_n"] = _ST["lid_n"] + 1 if abs(lid) > LID_LIMIT else 0
    if _ST["lid_n"] >= LID_STEPS:
        _ST["lid_bad"] = True
    s = objects["webcam"]
    low = float(s.low[2])
    if low > START[2] + _low + LIFTED:
        _ST["lifted"] = True
    x, y = float(s.pos[0]), float(s.pos[1])
    on_deck = BASE_RECT[0][0] <= x <= BASE_RECT[0][1] and BASE_RECT[1][0] <= y <= BASE_RECT[1][1] and low < BASE_TOP + 0.005
    resting = low < TOP + 0.005 or on_deck
    _ST["drop_n"] = _ST["drop_n"] + 1 if (_ST["lifted"] and resting) else 0
    if _ST["drop_n"] >= DROP_STEPS:
        _ST["dropped"] = True
    # Released and still: both hands 3 cm or more away for the last 1 s, and over that second the clip throat moved
    # under 2 mm and the body turned under 2 deg (from the read poses, as a video or a dial gauge would judge it).
    if _released(s):
        _ST["poses"].append((np.asarray(s.origin, float).copy(), s.rotation()))
        _ST["poses"] = _ST["poses"][-(STILL_STEPS + 1):]
    else:
        _ST["poses"] = []
    _ST["still_n"] = still_steps(_ST["poses"])


def still_steps(poses) -> int:
    """How many of the last released steps (up to STILL_STEPS) the webcam stayed within STILL_D and STILL_A of its
    newest pose."""
    if not poses:
        return 0
    o1, r1 = poses[-1]
    n = 0
    for o, r in reversed(poses[:-1]):
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r1.T @ r)) - 1.0) / 2.0))))
        if float(np.linalg.norm(o - o1)) > STILL_D or turn > STILL_A:
            break
        n += 1
    return n


def score(objects):
    """0 if the lid was ever over 2 deg off its start for 0.5 s, or the webcam, once lifted, rested on the table or
    deck for 0.5 s; 0 unless it has been let go and still (under 2 mm and 2 deg of motion) for the last 1 s and is seated. Then 1.0 with the foot folded
    (within 6 deg of the lid back) and the top within 4 deg of level; 0.5 folded and 4 to 10 deg; 0.25 for any seated
    camera within 20 deg (foot open included); 0 beyond."""
    _update(objects)
    if _ST["lid_bad"] or _ST["dropped"] or _ST["still_n"] < STILL_STEPS:
        return 0.0
    lid = float(objects["laptop"].joints.get("lid", 0.0))
    c = cam_pose(objects["webcam"])
    if not seated(c, lid):
        return 0.0
    tilt = tilt_deg(c)
    folded = abs(foot_to_lid_deg(c, lid)) <= FOLD_TOL
    if folded and tilt <= TILT_BANDS[0]:
        return 1.0
    if folded and tilt <= TILT_BANDS[1]:
        return 0.5
    return 0.25 if tilt <= TILT_BANDS[2] else 0.0


PARKED = 0.10  # m: success() (which ends the episode after SUCCESS_HOLD_S) waits until every finger pad is this far off


def success(objects):
    """A full score with both hands parked at least 10 cm from the webcam, so the video ends on the parked arms."""
    if score(objects) < 1.0:
        return False
    if mechkit._ENV.get("env") is None:
        return True
    return bool(mechkit.released(objects["webcam"], clearance=PARKED))


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps its own parts off the keyboard deck and away from the screen face: no arm part may pass over the base
# beyond the lid's top edge (x 0.26 on) below 20 cm above the table.
ARM_KEEP_OUT = [
    {"name": "keyboard deck and screen face", "x": (0.26, 0.52), "y": (-0.20, 0.20), "z_top": TOP + 0.20},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo. It reads the webcam's pose, the foot's angle and the lid's angle from the simulator.
demo = placeholder_demo
OPEN50 = -0.025  # one finger joint for a 50 mm pad gap
GRASP_DY = 0.025  # left grasp 25 mm left of the body centre (off the lens; 0.04 N m about the pad line)
L_LEAN = -0.5  # rad, left hand leaned about the pad line (x) toward its own base, for height
V_CARRY = 0.07  # m/s peak for free moves (gate limit 0.10)
L_PARK = (0.20, 0.21, TOP + 0.20)  # behind the lid, short of READY (0.323, +-0.305, TOP + 0.26), before the joint swing
R_PARK = (0.17, -0.22, TOP + 0.23)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


L_ROT = rot_x(L_LEAN) @ grip_rot(yaw=-math.pi / 2)  # pads close along x, across the body's 24 mm front and back
# (yaw -90 deg rather than +90: at +90 the left wrist roll runs onto its -2.09 rad stop at the seat)


def seat_throat(lid_value: float = 0.0) -> np.ndarray:
    h, u, _ = lid_frame(lid_value)
    return h + LID_L * u


def grasp_from_throat(t) -> np.ndarray:
    return np.asarray(t, float) + np.asarray(BODY_C) + np.array([0.0, GRASP_DY, 0.0])


# Right hand: closed fingertips on the foot's outer face, R_PUSH from its hinge, swung round the hinge.
R_PUSH = 0.030  # m from the foot hinge along the foot
R_Y = -0.012  # push 12 mm right of the camera's centre
TIP = 0.013  # closed pad tips reach this far past the tool point
R_LEAN = 0.35  # rad, right hand leaned about its pad line toward its own base


def foot_contact(t, open_deg: float, press: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Point on the seated foot's outer face (station) and the face's outward normal, at this opening; ``press`` m
    into the face."""
    t = np.asarray(t, float)
    p = t + _FH + R_PUSH * foot_dir(open_deg) + (FOOT_T / 2 - press) * foot_out(open_deg)
    p = p + np.array([0.0, R_Y, 0.0])
    return p, foot_out(open_deg)


def push_pitch(open_deg: float) -> float:
    """Hand pitch while pushing at this opening: along the face normal where reachable, capped at 1.15 rad."""
    o = foot_out(open_deg)
    want = math.atan2(-o[0], o[2])  # approach (sin p, 0, -cos p) along -o
    return float(min(max(want, 0.35), 1.15))


def r_rot(open_deg: float) -> np.ndarray:
    return rot_x(R_LEAN) @ grip_rot(pitch=push_pitch(open_deg))


_APPROACH_LOCAL = grip_rot().T @ np.array([0.0, 0.0, -1.0])  # the approach axis in the gripper frame


def push_tcp(t, open_deg: float, press: float = 0.0, stand: float = 0.0) -> np.ndarray:
    p, o = foot_contact(t, open_deg, press)
    r = r_rot(open_deg)
    approach = r @ _APPROACH_LOCAL
    return p + stand * o - TIP * approach


def _go(arm, pos, rot, finger=None, v=V_CARRY, min_s=0.6):
    d = float(np.linalg.norm(np.asarray(pos, float) - arm.pos))
    yield from arm.move(pos, max(min_s, 1.5 * d / v), finger, rot=rot)


SEAT_BIAS = -0.0005  # m: the left hand holds the throat this far behind the lid's mid-plane while the foot is folded


def _hold_seat(ctx, L, lid_value: float, done: list):
    """Left hand: keep the camera's throat SEAT_BIAS from the lid's mid-plane (screen normal only; along the lid it
    stays where it landed) until ``done[0]``. An integrated offset along the normal cancels the arm's give."""
    _, _, nrm = lid_frame(lid_value)
    base = L.pos.copy()
    dn = 0.0
    while not done[0]:
        if not L.offline:
            th = np.asarray(ctx.obj("webcam").origin, float)
            err = SEAT_BIAS - float((th - seat_throat(lid_value)) @ nrm)
            dn = float(np.clip(dn + 0.25 * err, -0.004, 0.004))
        yield from L.move(base + dn * nrm, CONTROL_DT, rot=L_ROT)


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    cam0 = ctx.obj("webcam")
    t0 = np.asarray(cam0.origin, float)
    g0 = grasp_from_throat(t0)
    # 1) left hand over the body, pads across its front and back; down, close, lift
    yield from _go(L, g0 + [0, 0, 0.06], L_ROT, OPEN50)
    yield from L.trim_to_measured(g0 + [0, 0, 0.06])
    for _ in range(2):
        yield from _go(L, g0, L_ROT, OPEN50, v=0.04)
        yield from L.grip(FINGER_CLOSED, 0.8)
        yield from L.hold(0.4)
        if L.holding(0.012):  # pads stopped on the 24 mm body
            break
        yield from L.grip(OPEN50, 0.6)
        yield from _go(L, g0 + [0, 0, 0.06], L_ROT)
    yield from _go(L, g0 + [0, 0, 0.08], L_ROT, v=0.04)
    # 2) carry level to 30 mm above the seat along the lid line, then lower along the lid until the head lands
    lid = float(ctx.obj("laptop").joints.get("lid", 0.0))
    _, u, _ = lid_frame(lid)
    seat = grasp_from_throat(seat_throat(lid))
    pre = seat + 0.030 * u
    yield from _go(L, (pre[0] - 0.02, pre[1] + 0.06, pre[2] + 0.012), L_ROT)
    yield from _go(L, pre, L_ROT, v=0.04)
    yield from L.hold(0.4)
    # 2b) lower the camera's read throat down the lid line onto the edge: the hand command carries an integrated offset
    # that cancels arm sag and grip shift, so the throat stays on the lid's mid-plane and the lip slides down 1 mm in
    # front of the screen. Stops when the throat is on the edge (within 1 mm) or stalls on something.
    edge = seat_throat(lid)
    off = np.zeros(3)
    s_mm, best, stall = 30.0, 1e9, 0
    for k in range(400):
        tgt = edge + (s_mm / 1000.0) * u
        if not L.offline:
            th = np.asarray(ctx.obj("webcam").origin, float)
            off = np.clip(off + 0.25 * (tgt - th), -0.03, 0.03)
            now_u = float((th - edge) @ u) * 1000.0
            if now_u <= 1.0:
                break
            stall = stall + 1 if now_u > best - 0.05 else 0
            best = min(best, now_u)
            if stall >= 40 and s_mm < now_u - 4.0:  # 2 s without progress: landed on something else
                break
        if s_mm > -3.0:
            s_mm -= 8.0 * CONTROL_DT  # 8 mm/s down the lid
        elif L.offline:
            break
        yield from L.move(grasp_from_throat(tgt) + off, CONTROL_DT, rot=L_ROT)
    yield from L.hold(0.3)
    yield from L.hold(0.5)
    # 3) right hand: closed fingers onto the open foot's outer face, then round the hinge until it lies on the lid back
    # (the left hand keeps servoing the throat 0.5 mm behind the lid's mid-plane meanwhile: the foot push otherwise
    # shoves the camera 1 mm toward the screen, the foot meets the lid back 1.6 deg open and springs to its 3 deg notch)
    c = cam_pose(ctx.obj("webcam"))
    t, f0 = c["throat"], c["foot_open"]
    if L.offline:  # offline dry runs never move the webcam: plan the fold from its nominal seated pose
        t, f0 = seat_throat(lid), FOOT_START
    done = [False]

    def fold():
        yield from R.grip(FINGER_CLOSED, 0.6)
        yield from _go(R, push_tcp(t, f0, stand=0.030) + [-0.03, 0, 0.03], r_rot(f0))
        yield from _go(R, push_tcp(t, f0, stand=0.012), r_rot(f0), v=0.03)
        yield from _go(R, push_tcp(t, f0, stand=0.0), r_rot(f0), v=0.01)
        n = 50
        for i in range(1, n + 1):
            a = f0 + (-5.0 - f0) * i / n  # sweep the opening from where it is to 5 deg past folded (the lid stops it)
            target = push_tcp(t, a, press=0.002)
            yield from R.move(target, 0.1, rot=r_rot(a))
        yield from R.hold(0.6)
        for k in range(3):  # the foot must reach its flat notch (under 1.5 deg open), not stop on the 3 deg one
            if R.offline or cam_pose(ctx.obj("webcam"))["foot_open"] < 1.5:
                break
            a = -5.0 - 3.0 * (k + 1)
            yield from R.move(push_tcp(t, a, press=0.002), 0.6, rot=r_rot(a))
            yield from R.hold(0.6)
        end = R.pos.copy()
        yield from _go(R, end + [-0.03, 0, 0.0], R.rot, v=0.03)
        yield from _go(R, end + [-0.05, 0, 0.05], R.rot, v=0.05)
        done[0] = True

    yield from ctx.both(_hold_seat(ctx, L, lid, done), fold())
    # 4) left hand lets go and rises straight up
    yield from L.grip(OPEN50, 1.0)
    yield from L.hold(0.5)
    p = L.pos.copy()
    yield from _go(L, p + [0, 0, 0.04], L_ROT, v=0.03)  # open pad tips rise clear of the body's top
    yield from _go(L, p + [-0.03, 0.10, 0.04], L_ROT, v=0.05)
    # 5) both hands park at READY, grippers open: a slow Cartesian move toward it, then the joint-space swing
    yield from R.grip(OPEN50, 0.6)
    yield from ctx.both(_go(L, L_PARK, L.rot), _go(R, R_PARK, R.rot))
    yield from ctx.both(L.home(6.0), R.home(6.0))
    yield from ctx.wait(10.0)  # success() must hold 8 s (SUCCESS_HOLD_S) before the script ends
