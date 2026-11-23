"""On the station table: a black flatbed scanner (Canon CanoScan LiDE 400 size, 250 x 367 x 42 mm) lies with its long
side across the robot's view and its lid raised at the far edge, leaning back on its stop. The glass is recessed 1.5 mm
below a black bezel; a white arrow on the bezel marks the front right corner of the glass (the robot's right). In front
of the scanner a red cloth pocket hardcover (164 x 113 x 21 mm closed, 228 g) stands on its tail, opened in a V of
130 degrees with its pages facing the robot, the way a new hardcover rests when it is set down open.

Requires -E object_jitter=0 (the layout is fixed).

Run it:  inspect-robots run --embodiment yam_arena -E task=flatbed_scan_open_book_face_down.py -E object_jitter=0 \\
           --instruction "Copy the right-hand page: lay the open book face down on the scanner glass, that page
           square in the arrow corner." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401  (shared kit; this task scores page corners from the book's joint state)
import mechkit as mk  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, hull  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0

# -- colours ------------------------------------------------------------------------------------------------------------
BLACK = (0.09, 0.09, 0.10)      # scanner body, bezel and lid
GLASS = (0.30, 0.42, 0.50)      # platen glass, blue grey so it reads apart from the black bezel
WHITE = (0.95, 0.95, 0.93)      # alignment arrow and the lid's white document pad
RED = (0.78, 0.10, 0.12)        # cloth case
CREAM = (0.97, 0.94, 0.84)      # text block (pages)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def _qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _qy(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# -- the scanner: Canon CanoScan LiDE 400 (2996C010), 250 x 367 x 42 mm ---------------------------------------------------
SC_X = 0.340                                  # body x 0.215..0.465, y -0.1835..0.1835
SC_D, SC_W = 0.250, 0.367
GLASS_X = (0.232, 0.448)                      # A4/Letter platen 216 x 297 mm
GLASS_Y = (-0.1485, 0.1485)
GLASS_Z = 0.0265                              # glass top above the table
BEZEL_Z = 0.028                               # bezel top: the glass sits 1.5 mm below it
ARROW = (GLASS_X[0], GLASS_Y[0])              # the alignment arrow's corner (front right in the robot's view)
HINGE_X, HINGE_Z = 0.465, 0.035               # lid hinge axis (along y) at the back edge
LID_T = 0.014
LID_OPEN = math.radians(100.0)                # authored open: 10 degrees past upright, resting on its stop


def _scanner_base():
    x0 = -SC_X
    bz = (GLASS_Z - 0.0015 + BEZEL_Z) / 2
    bh = BEZEL_Z - (GLASS_Z - 0.0015)
    gy = GLASS_Y[1]
    side_w = SC_W / 2 - gy
    shapes = [
        box((0.0, 0.0, (GLASS_Z - 0.0015) / 2), (SC_D, SC_W, GLASS_Z - 0.0015), BLACK),
        box((0.0, 0.0, GLASS_Z - 0.00075), (GLASS_X[1] - GLASS_X[0], 2 * gy, 0.0015), GLASS),
        box((x0 + (0.215 + GLASS_X[0]) / 2, 0.0, bz), (GLASS_X[0] - 0.215, SC_W, bh), BLACK),
        box((x0 + (GLASS_X[1] + 0.465) / 2, 0.0, bz), (0.465 - GLASS_X[1], SC_W, bh), BLACK),
        box((0.0, gy + side_w / 2, bz), (GLASS_X[1] - GLASS_X[0], side_w, bh), BLACK),
        box((0.0, -gy - side_w / 2, bz), (GLASS_X[1] - GLASS_X[0], side_w, bh), BLACK),
        # the white alignment arrow on the front bezel, pointing at the corner of the glass (look only, 12 mm)
        _look(hull((x0 + ARROW[0] - 0.003, ARROW[1] + 0.002, BEZEL_Z),
                   [(0.0, 0.0, 0.0), (-0.012, 0.004, 0.0), (-0.004, 0.012, 0.0),
                    (0.0, 0.0, 0.0006), (-0.012, 0.004, 0.0006), (-0.004, 0.012, 0.0006)], WHITE)),
        _look(box((x0 + 0.209, ARROW[1] + 0.012, BEZEL_Z + 0.0003), (0.004, 0.016, 0.0006), WHITE)),
        # hinge barrels on the back edge (look only)
        _look(cyl((HINGE_X - SC_X, 0.13, HINGE_Z), 0.007, 0.05, BLACK, axis="y")),
        _look(cyl((HINGE_X - SC_X, -0.13, HINGE_Z), 0.007, 0.05, BLACK, axis="y")),
    ]
    return shapes


def _lid_shapes():
    """The lid in its authored open pose (joint 0): hinged along y at the back edge, 100 degrees open."""
    d = np.array([-math.cos(LID_OPEN), 0.0, math.sin(LID_OPEN)])     # hinge to free edge
    n = np.array([-math.sin(LID_OPEN), 0.0, -math.cos(LID_OPEN)])    # the lid's underside normal (faces the robot)
    centre = d * (SC_D / 2)
    pad = d * (SC_D / 2) + n * (LID_T / 2 + 0.0003)
    tilt = _qy(LID_OPEN)
    return [
        box(tuple(centre), (SC_D, SC_W, LID_T), BLACK, tilt=tilt),
        _look(box(tuple(pad), (0.222, 0.310, 0.0006), WHITE, tilt=tilt)),
    ]


SCANNER = Mech(
    "scanner", pos=(SC_X, 0.0, TOP), mass=None, shapes=_scanner_base(),
    joints=[Joint("lid", "hinge", tuple(_lid_shapes()), axis="y", at=(HINGE_X - SC_X, 0.0, HINGE_Z),
                  low=-LID_OPEN, high=0.0, damping=0.5, mass=0.35)],
)

# -- the book: Everyman's Library Pocket Poets hardcover, 164 x 113 x 21 mm, 228 g ----------------------------------------
H_BOARD, W_BOARD, T_BOARD = 0.164, 0.113, 0.0025     # each cloth-covered board
H_TEXT, W_TEXT, T_TEXT = 0.158, 0.110, 0.008         # each half of the sewn text block (3 mm squares)
SQUARE = 0.003
V_DEG = 130.0                                         # opening at the start
HALF = math.radians(V_DEG / 2)
U_L = np.array([-math.cos(HALF), math.sin(HALF), 0.0])     # spine to left fore-edge at the start
U_R = np.array([-math.cos(HALF), -math.sin(HALF), 0.0])    # spine to right fore-edge
N_L = np.array([-U_L[1], U_L[0], 0.0])                     # left page face normal (into the V)
N_R = np.array([U_R[1], -U_R[0], 0.0])                     # right page face normal (into the V)
A_L = math.pi - HALF                                       # direction angles of the two halves
A_R = math.pi + HALF
FLAT = math.radians(180.0 - V_DEG)                         # spine joint value with the book open flat (+70 deg)
SPINE_HIGH = FLAT + math.radians(3.0)
SPINE_LOW = -math.radians(106.0)                           # closed
SPINE_SPRING = 0.02                                        # N m/rad toward the 130 degree V the sewn spine has set to
HALF_MASS = 0.114


def _half_shapes(u, nrm, ang, sgn):
    """One half: the board on the outside (t -2.5..0 mm), the text block inside (t 0..8 mm), from the spine axis."""
    ex = u
    board_c = u * (W_BOARD / 2) - nrm * (T_BOARD / 2) + np.array([0.0, 0.0, H_BOARD / 2])
    text_c = u * (W_TEXT / 2) + nrm * (T_TEXT / 2) + np.array([0.0, 0.0, SQUARE + H_TEXT / 2])
    _ = (ex, sgn)
    return [
        box(tuple(board_c), (W_BOARD, T_BOARD, H_BOARD), RED, tilt=_qz(ang)),
        box(tuple(text_c), (W_TEXT, T_TEXT, H_TEXT), CREAM, tilt=_qz(ang)),
    ]


# The spine axis lands 110 mm (one text block) inboard of the glass's right edge, plus 2.5 mm.
SPINE_Y = GLASS_Y[0] + 0.0025 + W_TEXT
BOOK_ORIGIN = (0.200, SPINE_Y, TOP)                 # spine axis at the table, tail down

BOOK = Mech(
    "book", pos=BOOK_ORIGIN, mass=HALF_MASS, friction=0.6,
    shapes=[*_half_shapes(U_L, N_L, A_L, 1.0),
            _look(cyl((0.0045, 0.0, H_BOARD / 2), 0.0045, H_BOARD, RED))],      # cloth spine (look only)
    joints=[Joint("spine", "hinge", tuple(_half_shapes(U_R, N_R, A_R, -1.0)), axis="z", at=(0.0, 0.0, 0.0),
                  low=SPINE_LOW, high=SPINE_HIGH, spring=SPINE_SPRING, damping=0.01, mass=HALF_MASS)],
)

OBJECTS = [SCANNER, BOOK]

# -- page geometry for the scorer ---------------------------------------------------------------------------------------


def _page_local(u, nrm):
    """The four page-face corners of one half's text block in its body frame: (spine, head), (fore, head),
    (spine, tail), (fore, tail)."""
    out = []
    for s in (0.0, W_TEXT):
        for h in (SQUARE + H_TEXT, SQUARE):
            out.append(u * s + nrm * T_TEXT + np.array([0.0, 0.0, h]))
    return np.array([out[0], out[2], out[1], out[3]])


PAGE_L = _page_local(U_L, N_L)
PAGE_R = _page_local(U_R, N_R)


def book_pages(state):
    """World page-face corners (4 x 3 each, order spine-head, fore-head, spine-tail, fore-tail) and outward page
    normals for the left and right halves, from the book's pose and spine angle."""
    rw = mk._quat_xyzw_matrix(np.asarray(state.quat_xyzw, dtype=float))
    o = np.asarray(state.origin, dtype=float)
    q = float(state.joints.get("spine", 0.0))
    rr = rw @ Rz(q)
    left = PAGE_L @ rw.T + o
    right = PAGE_R @ rr.T + o
    return left, right, rw @ N_L, rr @ N_R


# -- scoring ------------------------------------------------------------------------------------------------------------
FACE_TILT_DEG = 3.0           # both page faces within 3 degrees of the glass
FACE_GAP = 0.003              # every page corner within 3 mm of the glass top
REG_IN = 0.004                # head and fore-edge 0 to 4 mm inside the glass edges at the arrow
REG_SLACK = 0.0005            # contact offset at the bezel lip
DROP_FALL = 0.10              # DROP: the book's lowest point falls 10 cm or more within 0.2 s, hands off it
DROP_WINDOW = 4               # control steps (0.2 s; a 10 cm free fall takes 0.14 s)
DROP_HANDS_OFF = 0.02         # neither gripper's pads within 2 cm of the book at the end of the fall
HELD_TILT_DEG = 30.0          # 0.2 band: resting face down within 30 deg and open within 20 deg of flat
HELD_OPEN_DEG = 20.0
FINAL_STEPS = 20              # the end result must hold for the last 1 s (the lowest band over those steps counts)

_LATCH: dict = {}


@mk.on_reset
def _reset(env=None):
    _LATCH.clear()


def opening_deg(state) -> float:
    return V_DEG + math.degrees(float(state.joints.get("spine", 0.0)))


def _tilt(nrm) -> float:
    """Degrees between an outward page normal and straight down."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(nrm[2])))))


def _on_scanner_xy(p) -> bool:
    return SC_X - SC_D / 2 <= p[0] <= SC_X + SC_D / 2 and -SC_W / 2 <= p[1] <= SC_W / 2


def face_down(state) -> bool:
    left, right, nl, nr = book_pages(state)
    pts = np.vstack([left, right])
    return bool(_tilt(nl) <= FACE_TILT_DEG and _tilt(nr) <= FACE_TILT_DEG
                and all(_on_scanner_xy(p) for p in pts)
                and np.all(np.abs(pts[:, 2] - (TOP + GLASS_Z)) <= FACE_GAP))


def registration(state) -> tuple[bool, bool]:
    """(head edge registered, fore-edge registered) for the right page: both head corners 0 to 4 mm inside the
    glass's front edge, both fore-edge corners 0 to 4 mm inside its right edge."""
    _, right, _, _ = book_pages(state)
    head = right[:2, 0]
    fore = right[[1, 3], 1]
    lo_x, lo_y = ARROW[0] - REG_SLACK, ARROW[1] - REG_SLACK
    ok_head = bool(np.all(head >= lo_x) and np.all(head <= ARROW[0] + REG_IN))
    ok_fore = bool(np.all(fore >= lo_y) and np.all(fore <= ARROW[1] + REG_IN))
    return ok_head, ok_fore


def observe(objects):
    """Update the run latch (DROP) and the last 1 s of end bands from one step's book state. Only an unheld fall
    that could dent the boards or chip the glass ends the run; the book may narrow, be set back on the table,
    regripped or moved quickly in hand without losing anything."""
    s = _LATCH
    b = objects["book"]
    low = float(b.low[2])
    lows = s.setdefault("lows", [])
    lows.append(low)
    del lows[:-(DROP_WINDOW + 1)]
    if max(lows) - low >= DROP_FALL and mk.released(b, clearance=DROP_HANDS_OFF):
        s["drop"] = True
    s.setdefault("bands", []).append(band(objects))
    del s["bands"][:-FINAL_STEPS]


def grade(objects) -> float:
    """The run's score: 0 after a DROP, else the lowest end band over the last 1 s (so a book that slides into or
    out of the corner at the very end is judged by where it stayed)."""
    if _LATCH.get("drop"):
        return 0.0
    bands = _LATCH.get("bands") or [band(objects)]
    return min(bands)


def band(objects) -> float:
    """The final placement's band for one step."""
    b = objects["book"]
    if face_down(b):
        h, f = registration(b)
        return 1.0 if (h and f) else (0.6 if (h or f) else 0.4)
    _, _, nl, nr = book_pages(b)
    if (max(_tilt(nl), _tilt(nr)) <= HELD_TILT_DEG and opening_deg(b) >= 180.0 - HELD_OPEN_DEG
            and on_scanner(b) and resting(b)):
        return 0.2
    return 0.0


def on_scanner(state) -> bool:
    """Every page corner over the scanner body."""
    left, right, _, _ = book_pages(state)
    return all(_on_scanner_xy(p) for p in np.vstack([left, right]))


REST_UP = 0.010               # a book whose lowest point is more than 1 cm above the bezel is still held up
REST_SPEED = 0.02             # m/s


def resting(state) -> bool:
    """The book is down on the scanner, not held up in the air: its lowest point within 1 cm of the bezel top and
    moving under 2 cm/s."""
    return bool(float(state.low[2]) <= TOP + BEZEL_Z + REST_UP
                and float(np.linalg.norm(state.lin_vel)) < REST_SPEED)


def score(objects):
    """0 for the run only on DROP: the book's lowest point falls 10 cm or more within 0.2 s with neither gripper
    within 2 cm of it (dropped, not carried). Narrowing the spread, setting the book back on the table, regripping
    and fast moves in hand cost nothing. Otherwise the final placement, which must hold for the last 1 s (the lowest
    band over the final 20 steps counts): 1.0 when the book lies face down (both page faces within 3 degrees of the
    glass, every page corner within 3 mm of the glass top and over the scanner) with the right page's head and
    fore-edge each 0 to 4 mm inside the glass edges at the arrow; 0.6 face down with one of the two edges registered;
    0.4 face down with neither; 0.2 not face down but resting still on the scanner (lowest point within 1 cm of the
    bezel top, every page corner over it) with both pages facing down within 30 degrees and the book open within
    20 degrees of flat; 0 otherwise, including a book still held up in the air."""
    step = mk.step_count()
    if step == 0 or _LATCH.get("step") != step:      # score() and success() share a step: update once
        _LATCH["step"] = step
        observe(objects)
    return grade(objects)


def success(objects):
    """Score 1.0, the book still, and both grippers 10 cm clear of it (so the run ends with the hands away)."""
    b = objects["book"]
    return (score(objects) == 1.0 and float(np.linalg.norm(b.lin_vel)) < 0.01
            and mk.released(b, clearance=0.10))


# The instruction forbids no area; the raised lid is kept clear so an arm cannot knock it down onto the book.
ARM_KEEP_OUT = [
    {"name": "raised scanner lid", "x": (0.47, 0.56), "y": (-0.20, 0.20), "z_top": TOP + 0.30},
]

# -- the scripted demo --------------------------------------------------------------------------------------------------
E_GRIP = 0.008                # tool point 8 mm below the head edge (pads take about 21 mm of the leaf)
T_MID = (T_TEXT - T_BOARD) / 2  # mid-thickness of a leaf on its normal
GRIP_IN = 0.012               # 12 mm in from the fore-edge (keeps the forearms apart)
G_L = U_L * (W_BOARD - GRIP_IN) + N_L * T_MID + np.array([0.0, 0.0, H_BOARD - E_GRIP])
G_R = U_R * (W_BOARD - GRIP_IN) + N_R * T_MID + np.array([0.0, 0.0, H_BOARD - E_GRIP])
LEAN = math.radians(15.0)     # each gripper leans outward in its leaf's plane, keeping the two forearms apart


def _lean(rot, nrm, u):
    """Tilt a grip about the leaf normal so the wrist leans out past the fore-edge (pads still across the leaf)."""
    for a in (LEAN, -LEAN):
        r = axis_angle_matrix(np.asarray(nrm, dtype=float), a) @ rot
        if float((r @ np.array([0.0, 0.0, 1.0])) @ u) < 0:      # fingertips toward the spine, wrist outboard
            return r
    raise ValueError("lean")


ROT_L = _lean(grip_rot(yaw=-HALF), N_L, U_L)   # pads across the left leaf
ROT_R = _lean(grip_rot(yaw=+HALF), N_R, U_R)
OPEN_40 = -0.020              # 40 mm between the pads
LEAF_GAP = T_TEXT + T_BOARD   # 10.5 mm
SPEED = 0.07                  # m/s, transfers (the declared limit is 0.12)
SLOW = 0.015                  # m/s, contacts


def tcps(R, o, q):
    """Tool points and orientations of both hands for the book at rotation R, origin o and spine angle q."""
    o = np.asarray(o, dtype=float)
    rr = R @ Rz(q)
    return (o + R @ G_L, R @ ROT_L), (o + rr @ G_R, rr @ ROT_R)


R_FLAT = Rz(-(FLAT / 2))


def final_pose():
    """Book face down flat on the glass, right page's head-fore corner 1.5 mm (head) and 2.5 mm (fore-edge) inside the arrow corner."""
    R = Ry(-math.pi / 2) @ R_FLAT
    local = Rz(FLAT) @ PAGE_R[1]
    target = np.array([ARROW[0] + 0.0015, ARROW[1] + 0.0025, TOP + GLASS_Z + 0.0002])
    return R, target - R @ local


R_FIN, O_FIN = final_pose()
TAIL_LOCAL = np.array([0.0, 0.0, SQUARE]) + N_L * T_TEXT           # page-face tail line at the spine (on the axis line)
P_TAIL = O_FIN + R_FIN @ TAIL_LOCAL


def landing_pose(beta, lift=0.0):
    """The book pivoted about its page-face tail line on the glass, head raised to flip angle beta (90 deg = flat),
    raised by ``lift``."""
    Rp = Ry(math.pi / 2 - beta)
    R = Rp @ R_FIN
    o = P_TAIL + Rp @ (O_FIN - P_TAIL) + np.array([0.0, 0.0, lift])
    return R, o


demo = placeholder_demo


def _pages_now(ctx):
    """Page corners of the book as it lies now (a state read). If it does not read face down (an offline dry run's
    mock still shows the book standing), plan from the intended landing instead."""
    left, right, nl, nr = book_pages(ctx.obj("book"))
    if nl[2] > -0.8 or nr[2] > -0.8:
        left = PAGE_L @ R_FIN.T + O_FIN
        right = PAGE_R @ (R_FIN @ Rz(FLAT)).T + O_FIN
    return left, right


def _dur(a, b, v=SPEED, lo=0.5):
    return max(lo, 1.5 * float(np.linalg.norm(np.asarray(a) - np.asarray(b))) / v)


def _pair(ctx, R, o, q, secs, finger=None):
    (pl, rl), (pr, rrr) = tcps(R, o, q)
    yield from ctx.both(ctx.left.move(pl, secs, finger, rot=rl), ctx.right.move(pr, secs, finger, rot=rrr))


def _reg_offsets(ctx):
    """How far the right page's head and fore-edge lie inside the glass edges at the arrow: (min, max) over the two
    corners of each edge, in metres (a state read)."""
    _, right = _pages_now(ctx)
    head = right[:2, 0] - ARROW[0]
    fore = right[[1, 3], 1] - ARROW[1]
    return (float(head.min()), float(head.max())), (float(fore.min()), float(fore.max()))


def _in_band(off):
    return off[0] >= -REG_SLACK and off[1] <= SQUARE_OK


def _push_head(ctx, head_off, over, down):
    """Both hands push the tail edges toward the robot (-x) with closed, forward-leaning fingertips."""
    L, Rr = ctx.left, ctx.right
    left, right = _pages_now(ctx)
    tail_x = float(max(right[2:, 0].max(), left[2:, 0].max())) + SQUARE
    zp = TOP + GLASS_Z + PUSH_TIP_Z + 0.013
    sx = tail_x + FINGER_HALF + 0.005
    yl, yr = SPINE_Y + 0.085, SPINE_Y - 0.060
    tilt = grip_rot(pitch=PUSH_PITCH)
    yield from ctx.both(L.move((sx, yl, zp + 0.02), _dur(L.pos, (sx, yl, zp + 0.02)), FINGER_CLOSED, rot=tilt),
                        Rr.move((sx, yr, zp + 0.02), _dur(Rr.pos, (sx, yr, zp + 0.02)), FINGER_CLOSED, rot=tilt))
    yield from ctx.both(L.move((sx, yl, zp), 1.2, rot=tilt), Rr.move((sx, yr, zp), 1.2, rot=tilt))
    depth = 0.005 + max(0.0, head_off[1]) + over
    yield from ctx.both(
        L.press_until((sx, yl, zp), max_depth=depth, err_mm=PUSH_LAG_MM, min_depth=0.004, direction=(-1.0, 0.0, 0.0),
                      speed=SLOW, hold=PUSH_HOLD),
        Rr.press_until((sx, yr, zp), max_depth=depth, err_mm=PUSH_LAG_MM, min_depth=0.004, direction=(-1.0, 0.0, 0.0),
                       speed=SLOW, hold=PUSH_HOLD))
    yield from ctx.both(L.move(L.pos + np.array([0.015, 0.0, 0.0]), 0.8, rot=tilt),
                        Rr.move(Rr.pos + np.array([0.015, 0.0, 0.0]), 0.8, rot=tilt))
    yield from ctx.both(L.move(L.pos + np.array([0.0, 0.0, 0.04]), 1.0, rot=tilt),
                        Rr.move(Rr.pos + np.array([0.0, 0.0, 0.04]), 1.0, rot=tilt))


def _push_fore(ctx, fore_off, over, down):
    """The right hand parks; the left pushes the left fore-edge toward -y with closed fingertips."""
    L, Rr = ctx.left, ctx.right
    left, _ = _pages_now(ctx)
    fore_y = float(left[[1, 3], 1].max()) + SQUARE
    zp = TOP + GLASS_Z + PUSH_TIP_Z + 0.013
    side = grip_rot(yaw=-math.pi / 2)
    xm = float(left[:, 0].mean())
    sy = fore_y + FINGER_HALF + 0.010
    yield from ctx.both(L.move((xm, sy, zp + 0.05), _dur(L.pos, (xm, sy, zp + 0.05)), FINGER_CLOSED, rot=side),
                        Rr.move(PARK_R, _dur(Rr.pos, PARK_R), FINGER_OPEN, rot=down))
    yield from L.move((xm, sy, zp), 1.5, rot=side)
    depth = 0.010 + max(0.0, fore_off[1]) + over
    yield from L.press_until((xm, sy, zp), max_depth=depth, err_mm=PUSH_LAG_MM, min_depth=0.004,
                             direction=(0.0, -1.0, 0.0), speed=SLOW, hold=PUSH_HOLD)
    yield from L.move(L.pos + np.array([0.0, 0.015, 0.0]), 0.8, rot=side)
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.05]), 1.0, rot=side)


def demo_script(ctx):
    """Both arms: pinch each half by its head edge, lift, open the book flat in the air, turn it face down about the
    head edge, set the tail on the glass and lower the head, let go, then square the book into the arrow corner by
    pushing the tail edges and the left fore-edge with closed fingertips until the glass's bezel lip stops it.
    Reads the book's pose once after the release to plan the squaring pushes."""
    L, Rr = ctx.left, ctx.right
    o0 = np.array(BOOK_ORIGIN)
    R0 = np.eye(3)
    up = np.array([0.0, 0.0, 0.04])
    (pl, rl), (pr, rrr) = tcps(R0, o0, 0.0)
    # 1. Hover over each half's head edge, open 40 mm, trim, descend and pinch.
    yield from ctx.both(L.move(pl + up, _dur(L.pos, pl + up), OPEN_40, rot=rl),
                        Rr.move(pr + up, _dur(Rr.pos, pr + up), OPEN_40, rot=rrr))
    yield from ctx.both(L.trim_to_measured(pl + up), Rr.trim_to_measured(pr + up))
    yield from ctx.both(L.move(pl, 2.5, rot=rl), Rr.move(pr, 2.5, rot=rrr))
    for arm, p, r in ((L, pl, rl), (Rr, pr, rrr)):
        for attempt in range(2):
            yield from arm.grip(FINGER_CLOSED, 0.8)
            yield from arm.hold(0.3)
            if arm.holding(LEAF_GAP - 0.004):
                break
            yield from arm.grip(OPEN_40, 0.6)
            yield from arm.move(p - np.array([0.0, 0.0, 0.004 * (attempt + 1)]), 0.6, rot=r)
    # 2. Lift 40 mm together.
    o = o0 + up
    yield from _pair(ctx, R0, o, 0.0, 2.0)
    # 3. Open flat in the air: each hand swings its half about the spine, yawing out 35 degrees.
    n = 14
    for k in range(1, n + 1):
        f = k / n
        yield from _pair(ctx, Rz(-(FLAT / 2) * f), o, FLAT * f, 0.45)
    yield from ctx.wait(0.5)
    # 4. Rise, then turn face down about the head-edge line (both tool points stay put while the wrists pitch).
    o = o + np.array([0.0, 0.0, 0.02])
    yield from _pair(ctx, R_FLAT, o, FLAT, 1.5)
    (pl, _), _ = tcps(R_FLAT, o, FLAT)
    c = np.array([pl[0], 0.0, pl[2]])
    beta_air = math.radians(FLIP_DEG)
    n = 16
    for k in range(1, n + 1):
        b = beta_air * k / n
        Rf = Ry(-b)
        yield from _pair(ctx, Rf @ R_FLAT, c + Rf @ (o - c), FLAT, 0.35)
    Rb, ob = Ry(-beta_air) @ R_FLAT, c + Ry(-beta_air) @ (o - c)
    # 5. Carry over the glass, tail 30 mm up, then down to the glass and pivot the head down about the tail.
    Ra, oa = landing_pose(beta_air, 0.030)
    yield from _pair(ctx, Ra, oa, FLAT, _dur(tcps(Rb, ob, FLAT)[0][0], tcps(Ra, oa, FLAT)[0][0]))
    Ra, oa = landing_pose(beta_air, 0.002)
    yield from _pair(ctx, Ra, oa, FLAT, 2.0)
    n = 6
    for k in range(1, n + 1):
        b = beta_air + (math.radians(RELEASE_DEG) - beta_air) * k / n
        Ra, oa = landing_pose(b, 0.0)
        yield from _pair(ctx, Ra, oa, FLAT, 0.5)
    yield from ctx.wait(0.4)
    # 6. Open the jaws a little, the head settles, and draw each hand straight back off the head edge.
    rel = -(LEAF_GAP + 2 * JAW_DROP) / 2
    yield from ctx.both(L.grip(rel, 1.0), Rr.grip(rel, 1.0))
    yield from ctx.wait(0.5)
    (pl, rl), (pr, rrr) = tcps(Ra, oa, FLAT)
    back = -(Ra @ np.array([0.0, 0.0, -1.0]))       # back out along the fingers (head to tail reversed)
    back = Ra @ np.array([0.0, 0.0, 1.0])
    (_, rl_out), (_, rr_out) = tcps(*landing_pose(math.radians(BACKOUT_DEG)), FLAT)   # wrists unpitch as they back off
    yield from ctx.both(L.move(pl + 0.035 * back, 2.5, rot=rl_out), Rr.move(pr + 0.035 * back, 2.5, rot=rr_out))
    pl2, pr2 = pl + 0.035 * back + np.array([0.0, 0.0, 0.08]), pr + 0.035 * back + np.array([0.0, 0.0, 0.08])
    down = grip_rot(yaw=0.0)
    yield from ctx.both(L.move(pl2, 2.5, rot=down), Rr.move(pr2, 2.5, rot=down))
    yield from ctx.both(L.grip(FINGER_CLOSED, 0.6), Rr.grip(FINGER_CLOSED, 0.6))
    # 7. Square the book: read where it lies, push both tail edges toward the robot until the head stops on the bezel
    #    lip, then push the left fore-edge to the right until the right fore-edge stops on the lip. Each push drives
    #    the fingertips well past the lip line so the arm's lag under the book's sliding friction cannot leave it short;
    #    then read the book again and repeat a push whose edge is still outside the window.
    head_off, fore_off = _reg_offsets(ctx)
    for rnd in range(SQUARE_ROUNDS):
        over = PUSH_OVER + PUSH_OVER_STEP * rnd
        if rnd == 0 or not _in_band(head_off):
            yield from _push_head(ctx, head_off, over, down)
            head_off, fore_off = _reg_offsets(ctx)
        if rnd == 0 or not _in_band(fore_off):
            yield from _push_fore(ctx, fore_off, over, down)
            head_off, fore_off = _reg_offsets(ctx)
        if _in_band(head_off) and _in_band(fore_off):
            break
    side = grip_rot(yaw=-math.pi / 2)
    yield from ctx.both(L.move(L.pos + np.array([0.0, 0.0, 0.06]), 1.2, FINGER_OPEN, rot=side),
                        Rr.move(PARK_R, _dur(Rr.pos, PARK_R), FINGER_OPEN, rot=down))
    # 8. Park both arms well clear and hold still.
    yield from ctx.both(L.home(6.0), Rr.home(6.0))
    yield from ctx.wait(3.0)


FLIP_DEG = 73.0               # flip angle in the air and at tail touch-down (17 degrees short of flat)
RELEASE_DEG = 76.0            # head lowered to 14 degrees short of flat before the jaws open
BACKOUT_DEG = 62.0            # wrist pitch while backing the fingers out from under the head
JAW_DROP = 0.006              # each pad backs 6 mm off the leaf when the jaws open
PUSH_TIP_Z = 0.006            # closed fingertips 6 mm above the glass for the squaring pushes
PUSH_LAG_MM = 30.0            # lag stop off in practice: a push runs to its depth, past the lip line by PUSH_OVER
PUSH_OVER = 0.018             # fingertips driven 18 mm past the lip line: the arm lags about 6 mm sliding the 228 g
                              # book (friction 0.6); the rest presses the edge on the lip with a few newtons
PUSH_OVER_STEP = 0.006        # the retry round pushes 6 mm deeper
PUSH_HOLD = 0.8               # s held at the push depth so the lagging arm finishes the slide
SQUARE_OK = 0.0025            # an edge more than 2.5 mm inside the glass edge is pushed again (the window is 4 mm)
SQUARE_ROUNDS = 2             # one read-and-retry round (keeps the run inside 2,400 steps)
PUSH_PITCH = 0.3             # rad, the squaring push's forward lean
FINGER_HALF = 0.014           # closed fingers reach 14 mm either side of the tool point along x
PARK_R = (0.20, -0.20, TOP + 0.15)
