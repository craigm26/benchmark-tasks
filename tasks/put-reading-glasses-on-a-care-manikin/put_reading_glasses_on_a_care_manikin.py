"""On the station table: a full-size vinyl nursing-skills manikin head (190 mm front to back, 140 mm across at the
ears, chin 125 mm and crown 310 mm above the table) on a grey neck and stand, facing the robot at x 0.345, y -0.070,
with closed eyelids, a nose with sloped flanks, and two ears, each a root standing out from the skull and a pinna with a
rim. In front of it, under the chin, a light grey overbed tray (170 x 175 x 38 mm) holds a resident's black acetate reading glasses
(140 mm across, 25 g), open, standing on their lens bottoms and earpiece tips, temples pointing away from the robot.

The task: put the glasses on the manikin as a care aide does. Both nose pads on the sides of the nose, each temple over
its ear with the bent earpiece hooked down behind the ear root, then let go. A frame that touches an eyelid at any time
scores 0.

Requires -E object_jitter=0 (the head and tray are fixed; the scoring windows are measured from the head's landmarks).

Run it:  inspect-robots run --embodiment yam_arena -E task=put_reading_glasses_on_a_care_manikin.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "Put the reading glasses on the manikin: pads on the nose, both earpieces
           behind the ears, then let go." --scorer task_score ...
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
from mechkit import Joint, Mech, box, cyl, hull, on_reset, sweep  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 15.0  # Isaac ends the episode once success() holds this long: long enough to show both arms parked
CONTROL_DT = 0.05  # s per control step: the runs use -E control_hz=20

SKIN = (0.86, 0.69, 0.58)
LID = (0.78, 0.60, 0.50)
EAR = (0.83, 0.65, 0.55)
HAIR = (0.20, 0.14, 0.10)
BROW = (0.22, 0.15, 0.10)
LIPS = (0.62, 0.32, 0.30)
STAND = (0.45, 0.46, 0.48)
TRAY = (0.85, 0.86, 0.88)
ACETATE = (0.07, 0.07, 0.08)
LENS = (0.55, 0.65, 0.75)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def _quat_wxyz(R) -> tuple:
    """(w, x, y, z) of a rotation matrix, plain floats."""
    q = ik.matrix_to_quat_xyzw(np.asarray(R, dtype=float))
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def _rz(a):
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), float(a))


def _ry(a):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), float(a))


# ---------------------------------------------------------------------------------------------------------------------
# The head. World landmarks (m, z above TOP). H is the head's vertical axis; "out" is the distance from it in y.
HX, HY = 0.345, -0.070
CZ = 0.205  # centre of the skull ellipsoid above the table
AX, AY, AZ = 0.095, 0.070, 0.105  # skull semi-axes: 190 deep, 140 across, 210 tall
FACE_X = 0.266  # the face plane under the brow (eye sockets, cheeks)
FACE_Z = (0.150, 0.207)  # band of the face plane
NASION = (0.258, 0.203)  # (x, z) top of the nose ridge
NOSE_TIP = (0.234, 0.158)
RIDGE_HALF = 0.0025  # 5 mm wide ridge
FLANK_DEG = 40.0  # flanks 40 deg off the midplane
_TAN = math.tan(math.radians(FLANK_DEG))
_DXDZ = (NASION[0] - NOSE_TIP[0]) / (NASION[1] - NOSE_TIP[1])  # ridge slope: x per z
LID_C = (0.271, 0.032, 0.193)  # lid centres (x, out, z); the lids' fronts are at x 0.263
LID_R = (0.008, 0.013, 0.0065)
ROOT_X = (0.324, 0.346)  # ear root, front to back
ROOT_OUT = (0.062, 0.088)
ROOT_TOP = 0.197
PINNA_C = (0.340, 0.181)  # (x, z) centre of the pinna outline; 32 x 62 mm, top 0.212
PINNA_R = (0.016, 0.031)
PINNA_IN = 0.087  # pinna inner face at its front, flaring outward 30 deg going back
PINNA_FLARE = 0.30
CHIN_Z = 0.125


def ridge_x(z_above: float) -> float:
    """x of the nose ridge at this height above TOP."""
    return NASION[0] - (NASION[1] - z_above) * _DXDZ


def _ell_ring(dz: float, n: int = 24, clip_x: float | None = None) -> list:
    """Skull section at dz from the centre, world (x, y, z above TOP) relative to the Mech origin (HX, HY, TOP)."""
    k = 1.0 - (dz / AZ) ** 2
    if k <= 0:
        return [(0.0, 0.0, CZ + dz)]
    s = math.sqrt(k)
    pts = []
    for a in np.linspace(0.0, 2 * math.pi, n, endpoint=False):
        x = HX + AX * s * math.cos(a)
        y = AY * s * math.sin(a)
        if clip_x is not None:
            x = max(x, clip_x)
        pts.append((x - HX, y, CZ + dz))
    return pts


def _face_clip(z_above: float):
    return FACE_X if FACE_Z[0] - 1e-9 <= z_above <= FACE_Z[1] + 1e-9 else None


def _skull_shapes() -> list:
    out = []
    dzs = [-0.055, -0.040, -0.025, -0.012, 0.002, 0.020, 0.040, 0.060, 0.078, 0.093, 0.105]
    for a, b in zip(dzs[:-1], dzs[1:]):
        ring = _ell_ring(a, clip_x=_face_clip(CZ + a)) + _ell_ring(b, clip_x=_face_clip(CZ + b))
        out.append(hull((0, 0, 0), ring, SKIN))
    # jaw and chin: from the lowest skull ring down to the chin, narrower (58 mm half width at the angle of the jaw)
    z0 = CZ - 0.055
    jaw = _ell_ring(-0.055, clip_x=FACE_X)
    for a in np.linspace(0.0, 2 * math.pi, 20, endpoint=False):
        x = 0.318 + 0.060 * math.cos(a)
        jaw.append((max(x, 0.262) - HX, 0.045 * math.sin(a), CHIN_Z))
    del z0
    out.append(hull((0, 0, 0), jaw, SKIN))
    # brow ridge: the glabella stands 14 mm proud of the eye sockets (x 0.252 at 0.212)
    brow = []
    for y in (-0.048, -0.030, 0.0, 0.030, 0.048):
        xf = 0.252 + 0.008 * (abs(y) / 0.048) ** 2
        brow += [(xf - HX, y, 0.212), (xf + 0.004 - HX, y, 0.218), (xf + 0.006 - HX, y, 0.206),
                 (0.285 - HX, y, 0.204), (0.285 - HX, y, 0.222)]
    out.append(hull((0, 0, 0), brow, SKIN))
    return out


def _nose_shape():
    pts = []
    for s in (-1, 1):
        pts += [(NASION[0] - HX, s * RIDGE_HALF, NASION[1]), (NOSE_TIP[0] - HX, s * RIDGE_HALF, NOSE_TIP[1]),
                (0.238 - HX, s * 0.006, 0.152), (0.262 - HX, s * 0.006, 0.150)]
        for z in (0.203, 0.185):  # base on the face plane, on the 40 deg flank planes
            x = 0.268
            pts.append((x - HX, s * (RIDGE_HALF + (x - ridge_x(z)) * _TAN), z))
        pts.append((0.266 - HX, s * 0.017, 0.160))
    return hull((0, 0, 0), pts, SKIN)


def _lid_shapes() -> list:
    out = []
    for s in (-1, 1):
        pts = []
        for a in np.linspace(0, 2 * math.pi, 10, endpoint=False):
            for b in (-1.2, -0.6, 0.0, 0.6, 1.2):
                pts.append((LID_C[0] - LID_R[0] * math.cos(b) * math.cos(a) - HX,
                            s * (LID_C[1] + LID_R[1] * math.cos(b) * math.sin(a)), LID_C[2] + LID_R[2] * math.sin(b)))
        out.append(hull((0, 0, 0), pts, LID))
    return out


def pinna_in(x: float) -> float:
    """Out-distance of the pinna's inner face at this x (flares outward going back)."""
    return PINNA_IN + max(0.0, x - ROOT_X[0]) * PINNA_FLARE


ROOTS: dict = {}  # the two ear-root hulls (+1 left, -1 right), for the offline clearance checks


def _ear_shapes() -> list:
    out = []
    for s in (-1, 1):
        r = []
        for x in ROOT_X:
            for o in ROOT_OUT:
                r += [(x - HX, s * o, 0.150)]
        r += [(ROOT_X[0] - HX, s * o, ROOT_TOP) for o in ROOT_OUT]
        r += [(ROOT_X[1] - 0.001 - HX, s * o, ROOT_TOP) for o in ROOT_OUT]  # back face: a wall with a 1 mm round
        r += [(ROOT_X[1] - HX, s * o, ROOT_TOP - 0.001) for o in ROOT_OUT]
        root = hull((0, 0, 0), r, EAR)
        out.append(root)
        ROOTS[s] = root
        # pinna: three curved 6 mm plates round the outline (top-front, back, lobe)
        for a0, a1 in ((60.0, 200.0), (-70.0, 60.0), (200.0, 290.0)):
            pts = []
            for a in np.linspace(math.radians(a0), math.radians(a1), 7):
                x = PINNA_C[0] + PINNA_R[0] * math.cos(a)
                z = PINNA_C[1] + PINNA_R[1] * math.sin(a)
                for o in (pinna_in(x), pinna_in(x) + 0.006):
                    pts.append((x - HX, s * o, z))
            for o in (pinna_in(PINNA_C[0]), pinna_in(PINNA_C[0]) + 0.006):
                pts.append((PINNA_C[0] - HX, s * o, PINNA_C[1]))
            out.append(hull((0, 0, 0), pts, EAR))
        # helix rim: a 3.5 mm round rim over the top and back, on the outer side of the plate
        rim = []
        for a in np.linspace(math.radians(150.0), math.radians(-60.0), 13):
            x = PINNA_C[0] + (PINNA_R[0] - 0.002) * math.cos(a)
            z = PINNA_C[1] + (PINNA_R[1] - 0.002) * math.sin(a)
            rim.append((x - HX, s * (pinna_in(x) + 0.0065), z))
        out.append(sweep((0, 0, 0), rim, 0.0035, EAR, ends="round"))
    return out


def _face_looks() -> list:
    out = []
    for s in (-1, 1):
        brow = [(0.2535 + 0.0003 * i * i - HX, s * (0.012 + 0.008 * i), 0.2185 - 0.0006 * i * i) for i in range(5)]
        out.append(_look(sweep((0, 0, 0), brow, 0.0022, BROW)))
        lash = [(LID_C[0] - LID_R[0] * math.cos(t) - HX + 0.0004, s * (LID_C[1] + LID_R[1] * math.sin(t)),
                 LID_C[2] - 0.001) for t in np.linspace(-1.1, 1.1, 4)]
        out.append(_look(sweep((0, 0, 0), lash, 0.0014, BROW)))
    lips = [(0.258 + 0.00004 * (a * 1000) ** 2 / 1000 - HX, a, 0.139) for a in np.linspace(-0.020, 0.020, 5)]
    out.append(_look(sweep((0, 0, 0), lips, 0.0032, LIPS)))
    # hair: a cap over the crown and back, 2 mm proud, look-only
    hair = []
    for dz in (0.045, 0.075, 0.107):
        for p in _ell_ring(dz, 20):
            x, y, z = p
            if x + HX > 0.262 or dz > 0.08:
                k = 1.03
                hair.append(((x) * k, y * k, z + 0.002))
    for p in _ell_ring(0.0, 20):
        if p[0] + HX > 0.36:
            hair.append((p[0] * 1.03, p[1] * 1.03, p[2]))
    out.append(_look(hull((0, 0, 0), hair, HAIR)))
    return out


def _stand_shapes() -> list:
    return [cyl((0.005, 0.0, 0.0075), 0.075, 0.015, STAND, sides=24),
            cyl((0.005, 0.0, 0.015 + 0.0625), 0.050, 0.125, STAND, sides=24)]


HEAD = Mech("manikin_head", pos=(HX, HY, TOP), mass=None, friction=1.0,
            shapes=[*_stand_shapes(), *_skull_shapes(), _nose_shape(), *_lid_shapes(), *_ear_shapes(), *_face_looks()])
LID_SHAPES = _lid_shapes()  # for the scorer's lid test (same geometry as in HEAD)

TRAY_BOX = mechkit.Mech("bedside_tray", pos=(0.205, HY, TOP), mass=None, friction=0.6,
                        shapes=[box((0.0, 0.0, 0.019), (0.170, 0.175, 0.038), TRAY)])
TRAY_TOP = 0.038

# ---------------------------------------------------------------------------------------------------------------------
# The glasses. Frame origin: the midpoint between the two nose pads ("the pad line"), x toward the temples, z up.
# Worn, each pad lies on its nose flank at 190 mm, 8 mm either side of the midline.
PAD_Z = 0.190
PAD_OUT = 0.008
_N = np.array([-_TAN * 1.0, 1.0, _TAN * _DXDZ])
FLANK_N = _N / np.linalg.norm(_N)  # outward normal of the +y flank (mirror y for the other)
_PAD_SURF_X = ridge_x(PAD_Z) + (PAD_OUT - RIDGE_HALF) / _TAN
_PAD_C = np.array([_PAD_SURF_X, PAD_OUT, PAD_Z]) + FLANK_N * 0.0013  # pad centre 1 mm + 0.3 mm off the flank
WORN = np.array([float(_PAD_C[0]), HY, TOP + float(_PAD_C[2])])  # glasses origin when worn
PAD_LOCAL_Y = float(_PAD_C[1])
HINGE_X = 0.253 - float(WORN[0])  # hinges at world x 0.253 when worn
HINGE_OUT = 0.068
TEMPLE_Z = ROOT_TOP + 0.0042 - float(_PAD_C[2])  # temple centre: underside 0.2 mm over the root tops when worn
SPLAY = math.radians(6.0)  # open stop at 96 deg: each temple splays 6 deg outward
TEMPLE_L, TEMPLE_W, TEMPLE_H = 0.098, 0.004, 0.008  # hinge to bend; earpiece 35 mm more
EAR_L, EAR_H = 0.035, 0.006
BEND = math.radians(75.0)  # earpiece bent 75 deg down: hugs the back wall of the ear root
LENS_W, LENS_H, LENS_T = 0.050, 0.032, 0.002
LENS_BACK = 0.244 - float(WORN[0])  # lens back faces at world x 0.244 when worn
LENS_ZC = 0.003
LENS_YC = 0.033

GLASS_BOXES: list = []  # (body, centre, size, R) of every collider, for the scorer's lid test


def _gbox(body, centre, size, color, R=None):
    R = np.eye(3) if R is None else np.asarray(R, dtype=float)
    GLASS_BOXES.append((body, np.asarray(centre, dtype=float), np.asarray(size, dtype=float), R))
    return box(tuple(float(v) for v in centre), tuple(float(v) for v in size), color, tilt=_quat_wxyz(R))


def _front_shapes() -> list:
    sh = []
    xc = LENS_BACK - LENS_T / 2
    for s in (-1, 1):
        sh.append(_gbox("base", (xc, s * LENS_YC, LENS_ZC), (LENS_T, LENS_W, LENS_H), LENS))
        # rims (look-only), 5 mm acetate round each lens
        yc, zc = s * LENS_YC, LENS_ZC
        for dz, h in ((LENS_H / 2 + 0.0015, 0.005), (-LENS_H / 2 - 0.001, 0.003)):
            sh.append(_look(box((xc - 0.0005, yc, zc + dz), (0.004, LENS_W + 0.006, h), ACETATE)))
        for dy in (LENS_W / 2 + 0.0015, -LENS_W / 2 - 0.0015):
            sh.append(_look(box((xc - 0.0005, yc + dy, zc), (0.004, 0.003, LENS_H), ACETATE)))
        # endpiece: lens rim to hinge
        sh.append(_gbox("base", (HINGE_X - 0.004, s * 0.0635, TEMPLE_Z), (0.014, 0.011, 0.009), ACETATE))
        # pad on its arm, the pad's thin axis along the flank normal
        n = FLANK_N * np.array([1.0, s, 1.0])
        z = np.array([0.0, 0.0, 1.0]) - n[2] * n
        z /= np.linalg.norm(z)
        R = np.column_stack([n, np.cross(z, n), z])
        sh.append(_gbox("base", (0.0, s * PAD_LOCAL_Y, 0.0), (0.002, 0.006, 0.010), ACETATE, R))
        arm_end = (xc - 0.001, s * (LENS_YC - LENS_W / 2 + 0.002), 0.006)
        sh.append(_look(mechkit.rod((0.0, s * PAD_LOCAL_Y, 0.002), arm_end, 0.0008, ACETATE)))
    sh.append(_gbox("base", (LENS_BACK - 0.003, 0.0, 0.0115), (0.004, 0.018, 0.006), ACETATE))  # bridge
    return sh


def _temple_shapes(s: int) -> list:
    """Temple and earpiece in the hinge's frame (origin at the hinge), open at the 96 deg stop."""
    yaw = s * SPLAY
    d = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    Rt = _rz(yaw)
    Re = Rt @ _ry(BEND)  # earpiece bent BEND down
    e = Re @ np.array([1.0, 0.0, 0.0])
    bend = d * TEMPLE_L
    body = "left_temple" if s > 0 else "right_temple"
    return [_gbox(body, d * TEMPLE_L / 2, (TEMPLE_L, TEMPLE_W, TEMPLE_H), ACETATE, Rt),
            _gbox(body, bend + e * EAR_L / 2 - Re @ np.array([0.0, 0.0, 0.001]), (EAR_L, TEMPLE_W, EAR_H), ACETATE, Re)]


def temple_points(s: int) -> dict:
    """Key temple points in the hinge frame: bend (top of the bend on the centreline), tip (earpiece end, centreline)."""
    yaw = s * SPLAY
    d = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    e = _rz(yaw) @ _ry(BEND) @ np.array([1.0, 0.0, 0.0])
    return {"d": d, "bend": d * TEMPLE_L, "tip": d * TEMPLE_L + e * EAR_L}


OPEN_STOP = math.radians(96.0)
FOLD = OPEN_STOP  # a temple folds 96 deg from its open stop to lie behind the front
HOLD = 0.004  # N m hinge friction
LEFT_T = Joint("left_temple", "hinge", axis="z", at=(HINGE_X, HINGE_OUT, TEMPLE_Z), low=-FOLD, high=0.0,
               mass=0.0055, hold=HOLD, friction=0.6, shapes=_temple_shapes(+1))
RIGHT_T = Joint("right_temple", "hinge", axis="z", at=(HINGE_X, -HINGE_OUT, TEMPLE_Z), low=0.0, high=FOLD,
                mass=0.0055, hold=HOLD, friction=0.6, shapes=_temple_shapes(-1))

# Start: open on the tray in front of the resident, under the chin, standing on lens bottoms and earpiece tips;
# hinges at x 0.147, centred on the head axis, so each arm takes the temple on its own side.
_JOINT_AT = {"base": np.zeros(3), "left_temple": np.array(LEFT_T.at), "right_temple": np.array(RIGHT_T.at)}
LOW_LOCAL = min(float((_JOINT_AT[b] + c + R @ (np.array([sx, sy, sz]) * size))[2])
                for b, c, size, R in GLASS_BOXES for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5))
START = (round(0.147 - HINGE_X, 6), HY, round(TOP + TRAY_TOP - LOW_LOCAL + 0.0005, 6))

GLASSES = Mech("glasses", pos=tuple(float(v) for v in START), mass=0.014, friction=0.9,
               shapes=_front_shapes(), joints=[LEFT_T, RIGHT_T])

OBJECTS = [HEAD, TRAY_BOX, GLASSES]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring
STILL_STEPS = 20  # the last 1 s
LIN_STILL, ANG_STILL = 0.003, math.radians(3.0)
UP_DEG = 30.0
LID_GAP = 0.001  # any glasses collider within 1 mm of a lid: E
TABLE_GAP = 0.003  # glasses' lowest point under 3 mm over the table: X
PAD_GAP = (-0.0015, 0.003)  # pad face within 3 mm of the flank (1.5 mm into it allowed for contact offsets)
BRIDGE_MID, BRIDGE_ALONG = 0.006, 0.008
TEMPLE_OPEN_DEG = 10.0
OVER_ROOT = (-0.001, 0.004)
BEND_BEHIND = (-0.006, 0.015)  # a hooked earpiece rests on the root's back corner, its bend up to 6 mm forward
TIP_BELOW = 0.005
TIP_OUT = (0.072, 0.086)
FIT_DEG = 4.0
LENS_CLEAR = 0.006

_ST: dict = {}
OFFLINE_RELEASED = True  # offline replays have no finger points; check.py sets False for its "still held" case


@on_reset
def _reset(env=None):
    _ST.clear()
    _ST.update(last=None, eyelid=False, table=False, still=0, now=0.0, parts={})


_reset()


def _rot(s) -> np.ndarray:
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _lid_points() -> list:
    pts = []
    for s in (-1, 1):
        for a in np.linspace(-math.pi / 2, math.pi / 2, 7):
            for b in np.linspace(-1.2, 1.2, 5):
                pts.append(np.array([LID_C[0] - LID_R[0] * math.cos(b) * math.cos(a),
                                     HY + s * (LID_C[1] + LID_R[1] * math.cos(b) * math.sin(a)),
                                     TOP + LID_C[2] + LID_R[2] * math.sin(b)]))
    return pts


LID_POINTS = _lid_points()


def _world_boxes(s) -> list:
    """(centre, half size, R) of every glasses collider, in the world."""
    R0, o = _rot(s), np.asarray(getattr(s, "origin", s.pos), dtype=float)
    bodies = mechkit.body_poses(GLASSES, s)
    out = []
    for body, c, size, R in GLASS_BOXES:
        Rb, ob = (R0, o) if body == "base" else bodies[body]
        out.append((ob + Rb @ c, size / 2, Rb @ R))
    return out


def _pt_box(p, c, h, R) -> float:
    q = R.T @ (p - c)
    d = np.maximum(np.abs(q) - h, 0.0)
    return float(np.linalg.norm(d))


def lid_clearance(s) -> float:
    """Smallest distance (m) from any glasses collider to either closed eyelid."""
    boxes = _world_boxes(s)
    return min(_pt_box(p, c, h, R) for p in LID_POINTS for (c, h, R) in boxes)


def flank_gap(p) -> float:
    """Signed distance (m) of a world point from the nearer nose flank plane; positive outside the nose."""
    z = float(p[2]) - TOP
    off = abs(float(p[1]) - HY)
    return float((off - RIDGE_HALF - (float(p[0]) - ridge_x(z)) * _TAN) / np.linalg.norm(_N))


def _temple_eval(s, side: int) -> dict:
    """What one temple is doing: open angle error, underside over the root top, bend behind the root, tip depth and
    out-distance."""
    name = "left_temple" if side > 0 else "right_temple"
    q = float(s.joints.get(name, 0.0))
    Rb, ob = mechkit.body_poses(GLASSES, s)[name]
    tp = temple_points(side)
    bend = ob + Rb @ tp["bend"]
    tip = ob + Rb @ tp["tip"]
    d = Rb @ tp["d"]
    xm = 0.5 * (ROOT_X[0] + ROOT_X[1])
    if abs(d[0]) > 1e-6:
        t = (xm - ob[0]) / d[0]
        over = ob + t * d
        under = float(over[2] - TEMPLE_H / 2 - (TOP + ROOT_TOP))
    else:
        under = 1.0
    return {"open_err_deg": abs(math.degrees(q)), "under": under, "bend_behind": float(bend[0] - ROOT_X[1]),
            "tip_below": float(TOP + ROOT_TOP - tip[2]), "tip_out": abs(float(tip[1]) - HY)}


def ear_ok(e: dict) -> bool:
    return bool(e["open_err_deg"] <= TEMPLE_OPEN_DEG and OVER_ROOT[0] <= e["under"] <= OVER_ROOT[1]
                and BEND_BEHIND[0] <= e["bend_behind"] <= BEND_BEHIND[1] and e["tip_below"] >= TIP_BELOW
                and TIP_OUT[0] <= e["tip_out"] <= TIP_OUT[1])


def worn_parts(s) -> dict:
    """The scored pieces of a glasses state (ignoring release and stillness)."""
    R = _rot(s)
    o = np.asarray(getattr(s, "origin", s.pos), dtype=float)
    up = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    pads = []
    for side in (-1, 1):
        n = FLANK_N * np.array([1.0, side, 1.0])
        face = o + R @ (np.array([0.0, side * PAD_LOCAL_Y, 0.0]) - n * 0.001)  # the pad's nose-side face
        pads.append(flank_gap(face))
    bridge = o + R @ np.array([LENS_BACK - 0.003, 0.0, 0.0115])
    along = math.hypot(o[2] - WORN[2], o[0] - WORN[0])
    b = bool(all(PAD_GAP[0] <= g <= PAD_GAP[1] for g in pads) and abs(bridge[1] - HY) <= BRIDGE_MID
             and along <= BRIDGE_ALONG and up <= UP_DEG)
    ears = [_temple_eval(s, side) for side in (1, -1)]
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    lid = lid_clearance(s)
    fit = abs(roll) <= FIT_DEG and abs(yaw) <= FIT_DEG and lid >= LENS_CLEAR
    return {"up": up, "pads": pads, "B": b, "ears": ears, "ear_ok": [ear_ok(e) for e in ears], "roll": roll,
            "yaw": yaw, "lid": lid, "fit": bool(fit)}


def worn_value(p: dict) -> float:
    if p["up"] > UP_DEG or not p["B"]:
        return 0.0
    v = 0.4 + 0.2 * sum(p["ear_ok"])
    if all(p["ear_ok"]) and p["fit"]:
        v += 0.2
    return round(v, 4)


def _released(s) -> bool:
    if mechkit._ENV.get("env") is None:
        return bool(OFFLINE_RELEASED)
    return bool(mechkit.released(s, clearance=0.03))


def _update(objects) -> None:
    """Advance the latches once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    s = objects["glasses"]
    if float(s.low[2]) < TOP + TABLE_GAP:
        _ST["table"] = True
    if lid_clearance(s) < LID_GAP:
        _ST["eyelid"] = True
    p = worn_parts(s)
    _ST["parts"] = p
    _ST["now"] = worn_value(p)
    still = (float(np.linalg.norm(s.lin_vel)) < LIN_STILL and float(np.linalg.norm(s.ang_vel)) < ANG_STILL)
    ok = _ST["now"] > 0.0 and still and _released(s)
    _ST["still"] = _ST["still"] + 1 if ok else 0


def score(objects):
    """0 for the whole run if a glasses collider ever comes within 1 mm of an eyelid or the glasses touch the table.
    Otherwise, once released, still and front up for the last 1 s: 0.4 for the pads on the nose, +0.2 per earpiece
    hooked behind its ear, +0.2 for a level fit clear of the face (with both ears). 0 otherwise."""
    _update(objects)
    if _ST["eyelid"] or _ST["table"] or _ST["still"] < STILL_STEPS:
        return 0.0
    return float(_ST["now"])


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps its own parts away from the eyes: no arm part may enter the box round both lids (to 4.6 cm
# across, from in front of the nose tip back past the lids, up to the top of the brow).
ARM_KEEP_OUT = [
    {"name": "eyes and lids, up to the brow", "x": (0.215, 0.285), "y": (HY - 0.046, HY + 0.046), "z_top": TOP + 0.23},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo. Both arms: each holds its own side's temple (the left 35 mm behind the hinge, the right 20 mm), pads
# on the temple's side faces, approach leaned forward about the pad-closing line (y): the left arm 45 deg, the right
# 15 deg, so the two forearms, which converge on temples 14 cm apart, stay apart (the MuJoCo arm meshes overlap at
# equal leans), and the left fingers pass behind the hinge end instead of landing on it. The demo reads the glasses'
# live pose and hinge angles (to align the grasps and to servo the frame's position, attitude and open temples onto its
# worn mark), the fixed head layout, the encoders, tracking error and the gripper gap.
demo = placeholder_demo
LEANS = {1: math.radians(45.0), -1: math.radians(15.0)}  # left 45, right 15: leans 30 deg apart keep the forearms apart
TIP = math.radians(18.0)
GRIP_BACK = {1: 0.035, -1: 0.020}  # grip centre behind the hinge, along the temple
OPEN20 = -0.010  # finger command for a 20 mm pad gap
OPEN8 = -0.004
BACK_OFF = 0.012  # after opening, each gripper slides this far back along its temple before rising
SPREAD = 0.001  # each grip pushed 1 mm outward to preload the open stops
FOLD_BAND = (math.radians(1.0), math.radians(2.0))  # the hinge servo keeps each temple 1 to 2 deg off its stop or closer
WIDEN_MAX = 0.004  # at most 4 mm: more would drag the pads along a temple already on its stop


def grip_local(side: int) -> np.ndarray:
    """The grip centre on a temple, in the glasses frame (temples at their open stops)."""
    d = temple_points(side)["d"]
    return np.array([HINGE_X, side * HINGE_OUT, TEMPLE_Z]) + d * GRIP_BACK[side]


def tcp_for(origin, tip: float, side: int, spread: float = 0.0, Rg=None) -> tuple[np.ndarray, np.ndarray]:
    """TCP position and rotation that hold the glasses with their origin at `origin`, temples tipped up by `tip`."""
    R = _ry(-tip) if Rg is None else Rg @ _ry(-tip)
    p = np.asarray(origin, dtype=float) + R @ (grip_local(side) + np.array([0.0, side * spread, 0.0]))
    return p, R @ grip_rot(pitch=LEANS[side])


# Glasses-origin path (world), temple tip angle, duration (s). WORN is the origin worn. The glasses start under the
# chin, so the lift goes straight up in front of the face; the tip and the lowering turn about the nose-pad line.
LIFT = np.array(START) + np.array([0.0, 0.0, 0.032])
HOVER = np.array([-0.004, 0.0, 0.006])  # pads 3.5 mm clear of the flanks at the slide end
FRONT = np.array([START[0], HY, WORN[2] + HOVER[2]])
SLIDE_END = WORN + HOVER
SEAT = WORN + np.array([0.001, 0.0, 0.003])  # 1 mm back, 3 mm up: pads 1.2 mm off, earpieces 2.7 mm behind the roots


def _smooth_time(a, b, v):
    return max(1.0, 1.5 * float(np.linalg.norm(np.asarray(b) - np.asarray(a))) / v)


PLAN = [  # (origin, tip, seconds)
    (LIFT, 0.0, 1.5 * 0.032 / 0.020),
    (FRONT, 0.0, _smooth_time(LIFT, FRONT, 0.03)),
    (FRONT, TIP, 2.0),
    (SLIDE_END, TIP, _smooth_time(FRONT, SLIDE_END, 0.02)),
    (SLIDE_END, TIP, 1.0),
    (SEAT, 0.0, 2.5),
]


def _rest_tip(Rg) -> float:
    """The temples' upward tip (rad) of a frame resting with rotation Rg (pitch about y only)."""
    Rg = np.asarray(Rg, dtype=float)
    return float(math.atan2(-Rg[0, 2], Rg[0, 0]))


class _Pair:
    """Moves both hands so the glasses follow a planned origin and tip. Live, the error between the glasses' measured
    origin and the plan is integrated into an offset on both hand commands (cancels arm sag and grip slip); offline the
    hands follow the plan directly."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.off = np.zeros(3)
        self.active = False
        self.p, self.t = np.array(START, dtype=float), 0.0
        self.spread = 0.0
        self.widen = {1: 0.0, -1: 0.0}  # per hand, extra outward offset that keeps its temple on the open stop
        self.roll, self.yaw, self.pitch = 0.0, 0.0, 0.0  # integrated attitude corrections (rad) from the measured frame

    def _cmd(self, p, t):
        if self.active and not self.ctx.right.offline:
            s = self.ctx.obj("glasses")
            err = np.asarray(p, dtype=float) - np.asarray(s.origin, dtype=float)
            self.off = self.off + 0.15 * err
            n = float(np.linalg.norm(self.off))
            if n > 0.030:
                self.off *= 0.030 / n
            Re = _rot(s) @ _ry(-t).T  # measured attitude against the plan: level the frame in roll and yaw
            self.roll = float(np.clip(self.roll + 0.12 * math.atan2(Re[2, 1], Re[2, 2]), -0.25, 0.25))
            self.yaw = float(np.clip(self.yaw + 0.12 * math.atan2(Re[1, 0], Re[0, 0]), -0.25, 0.25))
            self.pitch = float(np.clip(self.pitch + 0.12 * (t - _rest_tip(_rot(s))), -0.25, 0.25))
            # a temple folded off its open stop (positive fold) means that hand sits too far in: move it out
            for side, name in ((1, "left_temple"), (-1, "right_temple")):
                fold = -side * float(s.joints.get(name, 0.0))
                if fold > FOLD_BAND[1]:  # folded: move that hand out; back on the stop: ease it back in
                    w = self.widen[side] + 0.3 * GRIP_BACK[side] * (fold - FOLD_BAND[1])
                elif fold < FOLD_BAND[0]:
                    w = self.widen[side] - 0.3 * GRIP_BACK[side] * (FOLD_BAND[0] - fold)
                else:
                    w = self.widen[side]
                self.widen[side] = min(max(w, 0.0), WIDEN_MAX)
        for side, arm in ((1, self.ctx.left), (-1, self.ctx.right)):
            Rc = _rz(-self.yaw) @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -self.roll)
            pos, rot = tcp_for(np.asarray(p) + self.off, t + self.pitch, side, self.spread + self.widen[side], Rg=Rc)
            arm._set_cartesian(pos, rot, arm.finger)

    def go(self, p, t, seconds):
        p0, t0 = self.p.copy(), self.t
        n = max(1, round(seconds / CONTROL_DT))
        for i in range(1, n + 1):
            u = ik.smoothstep(i / n)
            self._cmd(p0 + u * (np.asarray(p) - p0), t0 + u * (t - t0))
            yield
        self.p, self.t = np.asarray(p, dtype=float).copy(), t

    def hold(self, seconds):
        for _ in range(max(1, round(seconds / CONTROL_DT))):
            self._cmd(self.p, self.t)
            yield


def _chain(*gens):
    for g in gens:
        yield from g


def demo_script(ctx):
    """Grasp both temples on the tray under the chin, lift, raise in front of the face, tip the temples up 18
    deg about the nose-pad line, slide back over the ears onto the nose, lower the temples behind the ears, let go,
    back off and park both arms at READY."""
    L, R = ctx.left, ctx.right
    g = ctx.obj("glasses")  # live pose and hinge angles: align each grasp to them
    Rg = _rot(g)
    o = np.asarray(g.origin, dtype=float)
    pair = _Pair(ctx)
    pair.p = o.copy()
    above = {}
    for side, arm in ((1, L), (-1, R)):
        pos, rot = tcp_for(o, 0.0, side, Rg=Rg)
        above[side] = (pos + np.array([0.0, 0.0, 0.045]), pos, rot)
    # 1) above each grip point, open 20 mm, trim sag, descend to the temple, close. Each arm takes its own side's
    # temple: the glasses lie centred on the head axis, so neither arm crosses the midline of the pair.
    yield from ctx.both(L.move(above[1][0], 7.0, OPEN20, rot=above[1][2]),
                        R.move(above[-1][0], 7.0, OPEN20, rot=above[-1][2]))
    yield from ctx.both(L.trim_to_measured(above[1][0]), R.trim_to_measured(above[-1][0]))
    yield from ctx.both(L.move(above[1][1], 2.0, rot=above[1][2]), R.move(above[-1][1], 2.0, rot=above[-1][2]))
    yield from ctx.both(L.grip(FINGER_CLOSED, 1.0), R.grip(FINGER_CLOSED, 1.0))
    yield from ctx.both(L.hold(0.3), R.hold(0.3))
    for side, arm in ((1, L), (-1, R)):
        if not arm.holding(0.0015):  # pads stop on the 4 mm temple; a miss closes to 0: reopen, re-centre, close
            yield from arm.grip(OPEN20, 0.6)
            yield from arm.move(above[side][1], 0.6, rot=above[side][2])
            yield from arm.grip(FINGER_CLOSED, 1.0)
            yield from arm.hold(0.3)
    # 2) spread 1 mm to load both temples on their open stops, then follow the plan
    pair.p, pair.t = o.copy(), _rest_tip(Rg)  # the frame rests tipped on the tray; the lift levels it
    for i in range(1, 11):
        pair.spread = SPREAD * i / 10
        yield from pair.hold(CONTROL_DT)
    pair.active = True
    for p, t, sec in PLAN:
        yield from pair.go(p, t, sec)
    yield from pair.hold(0.5)
    # 7) let the frame settle onto the nose and ear roots under its own weight, still in hand
    pair.active = False
    yield from ctx.both(L.hold(1.0), R.hold(1.0))
    # 8) open only to 8 mm (each pad moves 2 mm off the temple), wait, back each gripper 12 mm off along its own
    # temple (its fingers leave the hinge end behind them), rise 40 mm, then 40 mm outward; then park
    yield from ctx.both(L.grip(OPEN8, 1.0), R.grip(OPEN8, 1.0))
    yield from ctx.both(L.hold(0.5), R.hold(0.5))
    back = {side: BACK_OFF * temple_points(side)["d"] for side in (1, -1)}
    yield from ctx.both(L.move(L.pos + back[1], 1.5, rot=L.rot), R.move(R.pos + back[-1], 1.5, rot=R.rot))
    up = np.array([0.0, 0.0, 0.040])
    yield from ctx.both(L.move(L.pos + up, 2.0, rot=L.rot), R.move(R.pos + up, 2.0, rot=R.rot))
    yield from ctx.both(L.move(L.pos + np.array([0, 0.04, 0]), 1.5, rot=L.rot),
                        R.move(R.pos + np.array([0, -0.04, 0]), 1.5, rot=R.rot))
    yield from ctx.both(L.home(8.0), R.home(8.0))
    yield from ctx.both(L.hold(5.0), R.hold(5.0))
