"""On the station table: a supine adult airway manikin (head away from the robot, chest toward it) with a size 6
cuffless tracheostomy tube in its neck: a blue neck plate, a white 15 mm connector and a used inner cannula (cream
tube, teal hub with a sprung squeeze lever) seated and latched in it. To the right, a clean inner cannula (pale grey
tube, teal hub) hangs in a light blue stand beside the neck, and a red waste tray sits nearer the robot.

The inner cannula is a rigid quarter circle (45 mm radius): it only passes the connector while it turns about its
arc centre, 45 mm from the hub axis. A straight pull or push jams and drags the outer tube out of the neck.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=change_a_trach_inner_cannula_along_its_curve.py \\
           -E object_jitter=0 --instruction "Change the trach's inner cannula: put the used one in the red tray,
           then fit the clean one until it clicks." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import mechkit  # noqa: E402
from belimits import Rect  # noqa: E402,F401
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, arc, ball, bore, box, cyl, hull, lathe, on_reset, released, ring_profile, sweep, tube  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to back off and park after the first full score

SKIN, PLATE_BLUE, WHITE = (0.72, 0.52, 0.40), (0.15, 0.35, 0.80), (0.96, 0.96, 0.94)
CREAM, PALE, TEAL = (0.93, 0.86, 0.62), (0.80, 0.83, 0.86), (0.00, 0.55, 0.52)
CRADLE_BLUE, TRAY_RED = (0.50, 0.76, 0.96), (0.85, 0.10, 0.10)

# ---- the patient and the outer cannula --------------------------------------------------------------------------------
# E is the centre of the connector's top face; C is the centre of the tube's arc. The arc lies in the x-z plane.
E = np.array([0.300, 0.0, TOP + 0.117])
ARC_R = 0.045
C = E - np.array([ARC_R, 0.0, 0.0])
TUBE_R = 0.00425  # 8.5 mm inner cannula
BORE_R = 0.00575  # the connector's mouth: 11.5 mm, the lead in to the outer cannula's bore
LINER_R, LINER_OUT = 0.00475, 0.00675  # outer cannula: 9.5 mm round bore (0.5 mm per side), 13.5 mm outside
CONN_R, CONN_H = 0.00825, 0.008  # 16.5 mm connector, 8 mm proud of the plate
LIP_R, LIP_LOW, LIP_HIGH = 0.0135, -0.0025, -0.0005  # the connector's retaining lip, below E
PLATE = (0.030, 0.084, 0.004)  # neck plate (flange), x by y by z
PLATE_TOP = -0.008  # below E
HEAD_AT, HEAD_R = np.array([0.440, 0.0, TOP + 0.080]), 0.080
CHEST_X, CHEST_Y, CHEST_IN, CHEST_TOP = (0.100, 0.255), 0.150, 0.110, TOP + 0.090
NECK_X, NECK_R, NECK_AXIS = (0.255, 0.365), 0.060, TOP + 0.040
LIFT_STOP, TIE_SPRING, OUTER_MASS = 0.012, 200.0, 0.020

_P0 = np.array([E[0], 0.0, TOP])  # the patient Mech's origin: on the table under E


def _rel(p):
    return tuple(float(v) for v in np.asarray(p, dtype=float) - _P0)


def _hidden(shape):
    return dataclasses.replace(shape, visible=False)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def curved_tube(at, bend_r, degs, r_in, r_out, color, sides=12):
    """A hollow round tube bent about y at ``bend_r``: the outer cannula. ``degs`` are the stations along the bend
    (degrees below ``at`` + (bend_r, 0, 0), turning toward -x) and ``r_in`` the bore radius at each. One convex wedge
    per station pair and sector, packed by mechkit like its own tube: the bore's flats sit at r_in (corners outside),
    and on the outside of the bend the corners sit out by the chord's sag, so the bore is never under r_in."""
    ts = [math.radians(d) for d in degs]
    half = math.pi / sides

    def point(t, phi, r, sag=1.0):
        rho = (bend_r + r * math.cos(phi)) * sag
        return (rho * math.cos(t), r * math.sin(phi), -rho * math.sin(t))

    pieces = []
    for (t0, ra), (t1, rb) in zip(zip(ts, r_in), zip(ts[1:], r_in[1:])):
        for k in range(sides):
            pts = []
            for t, r in ((t0, ra), (t1, rb)):
                for phi in (2 * half * k, 2 * half * (k + 1)):
                    sag = 1.0 / math.cos((t1 - t0) / 2) if math.cos(phi) > 1e-9 else 1.0
                    pts += [point(t, phi, r / math.cos(half), sag), point(t, phi, r_out)]
            pieces.append(mechkit._hull_faces(pts))
    wd, n = mechkit._Welder(), 24
    fine = np.radians(np.linspace(degs[0], degs[-1], 39))
    bore_r = np.interp(fine, ts, r_in)
    ring = [[(point(t, 2 * math.pi * i / n, rr), point(t, 2 * math.pi * i / n, r_out)) for i in range(n)]
            for t, rr in zip(fine, bore_r)]
    for a, b in zip(ring, ring[1:]):
        for i in range(n):
            j = (i + 1) % n
            wd.face([a[i][1], a[j][1], b[j][1], b[i][1]])
            wd.face([a[i][0], b[i][0], b[j][0], a[j][0]])
    for end, flip in ((ring[0], False), (ring[-1], True)):
        for i in range(n):
            j = (i + 1) % n
            f = [end[i][0], end[j][0], end[j][1], end[i][1]]
            wd.face(f[::-1] if flip else f)
    vp, vf = wd.mesh()
    return mechkit._make_mesh("tube", (at[0] - bend_r, at[1], at[2]), color, mechkit.IDENT, "z", pieces,
                              (vp, mechkit._outward_visual(vp, vf)), None)


# The outer cannula: a round bore that follows the 45 mm curve from the connector's mouth to 95 degrees round. It opens
# to the connector's 11.5 mm at the mouth and narrows to LINER_R over the first 6 degrees (4.7 mm of arc).
LINER_DEG = [0.0, 6.0] + [float(d) for d in np.linspace(6.0, 95.0, 10)[1:]]
LINER_BORE = [BORE_R] + [LINER_R] * (len(LINER_DEG) - 1)
OUTER_SHAPES = [
    bore((0, 0, PLATE_TOP - PLATE[2] / 2), PLATE, 0.0115, PLATE_BLUE),
    tube((0, 0, -CONN_H / 2), BORE_R, CONN_R, CONN_H, WHITE, lead_in=0.002),
    ring_profile((0, 0, 0), [(CONN_R, LIP_R, LIP_LOW), (CONN_R, LIP_R, LIP_HIGH)], WHITE),
    curved_tube((0, 0, 0), ARC_R, LINER_DEG, LINER_BORE, LINER_OUT, PLATE_BLUE),
]
OUTER = Joint("outer_cannula", "slide", axis="z", at=_rel(E), low=0.0, high=LIFT_STOP, spring=TIE_SPRING,
              damping=2.0, mass=OUTER_MASS, friction=0.3, shapes=OUTER_SHAPES)
_cx0, _cx1 = CHEST_X[0] - _P0[0], CHEST_X[1] - _P0[0]
_chest = [(x, sy * y, z) for x in (_cx0, _cx1) for sy in (-1, 1)
          for y, z in ((CHEST_Y, 0.0), (CHEST_Y, 0.055), (CHEST_IN, 0.090))]
PATIENT = Mech("patient", pos=tuple(float(v) for v in _P0), friction=0.5, joints=[OUTER], shapes=[
    ball(_rel(HEAD_AT), HEAD_R, SKIN),
    _look(lathe((NECK_X[0] - _P0[0], 0, NECK_AXIS - TOP), [(NECK_R, 0.0), (NECK_R, NECK_X[1] - NECK_X[0])], SKIN, axis="x")),
    _look(hull((0, 0, 0), _chest, SKIN)),
    _hidden(box((0.005, 0, 0.0275), (0.120, 0.100, 0.055), SKIN)),  # under the neck, below the outer cannula
    _hidden(box(((_cx0 + _cx1 - 0.010) / 2, 0, 0.0425), (_cx1 - _cx0 - 0.010, 2 * CHEST_IN, 0.085), SKIN)),
    _hidden(box(((_cx0 + _cx1 - 0.010) / 2, 0, 0.0275), (_cx1 - _cx0 - 0.010, 2 * CHEST_Y, 0.055), SKIN)),
])

# ---- the inner cannula (two, free) --------------------------------------------------------------------------------------
# Hub frame: origin at the centre of the hub's lower face (which sits on the connector's top), w out of the hub (up
# when seated), u toward the lever, v = w x u. The tube leaves along -w and curves toward -u about (u -45 mm, w 0).
HUB_R, HUB_H = 0.0105, 0.006
FIN_U, FIN_V, FIN_W = (-0.009, 0.0091), 0.007, (0.006, 0.028)  # the fixed grip fin: 18.1 x 14 x 22 mm
SLOT_U, SLOT_V, LEVER_V = 0.005, 0.0035, 0.003  # the fin's slot for the paddle (from u 5 mm, 7 mm wide); lever half width
HINGE = np.array([0.0175, 0.0, 0.010])
LEVER_OPEN_STOP, LEVER_SPRING, LEVER_SQUEEZED = 0.47, 0.05, 0.45  # rad, N m/rad, paddle flush with the fin's face
BODY_MASS, LEVER_MASS = 0.025, 0.005
NOSE_TOP = -0.0035  # the latch nose's top face: 1 mm under the lip's underside when seated
TIP = np.array([-ARC_R, 0.0, -ARC_R])  # centre of the tube's far end


def rc(th: float) -> np.ndarray:
    """Hub frame turned ``th`` about the arc's axis (y), the way the hub turns as the cannula is drawn out."""
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


def _rbox(rb, lo, hi, color, origin=(0, 0, 0)):
    a, b = rb @ (np.array(lo) - origin), rb @ (np.array(hi) - origin)
    return box(tuple((a + b) / 2), tuple(np.abs(b - a)), color)


def _rhull(rb, uw, color, origin=(0, 0, 0), half_v=LEVER_V):
    pts = [tuple(rb @ (np.array([u, sv * half_v, w]) - origin)) for u, w in uw for sv in (-1, 1)]
    return hull((0, 0, 0), pts, color)


def cannula(name: str, pos, th: float, tube_color) -> Mech:
    """An inner cannula built turned ``th`` about y (0 or pi/2: the simulator places objects with a yaw only)."""
    rb = rc(th)
    path = [tuple(rb @ np.array([-ARC_R + ARC_R * math.cos(t), 0.0, -ARC_R * math.sin(t)]))
            for t in np.linspace(0.0, math.pi / 2, 13)]
    wdir = rb @ np.array([0.0, 0.0, 1.0])
    axis = "z" if abs(wdir[2]) > 0.5 else "x"
    shapes = [
        sweep((0, 0, 0), path, TUBE_R, tube_color),
        cyl(tuple(wdir * HUB_H / 2), HUB_R, HUB_H, TEAL, axis=axis, sides=24),
        _rbox(rb, (FIN_U[0], -FIN_V, FIN_W[0]), (SLOT_U, FIN_V, FIN_W[1]), TEAL),
        _rbox(rb, (SLOT_U, SLOT_V, FIN_W[0]), (FIN_U[1], FIN_V, FIN_W[1]), TEAL),  # cheeks either side of the slot
        _rbox(rb, (SLOT_U, -FIN_V, FIN_W[0]), (FIN_U[1], -SLOT_V, FIN_W[1]), TEAL),
    ]
    # The lever is drawn squeezed (paddle flush with the fin's face, in the slot) and built turned back to rest.
    back = rc(-LEVER_SQUEEZED)

    def sq(uw):
        return [tuple((back @ (np.array([u, 0.0, w]) - HINGE) + HINGE)[[0, 2]]) for u, w in uw]

    lever = Joint("lever", "hinge", axis="y", at=tuple(float(v) for v in rb @ HINGE), low=-LEVER_OPEN_STOP, high=0.0,
                  spring=LEVER_SPRING, damping=0.002, mass=LEVER_MASS, friction=0.6, shapes=[
                      _rbox(rb, (0.015, -LEVER_V, -0.0065), (0.017, LEVER_V, 0.010), TEAL, HINGE),  # arm down to the nose
                      _rhull(rb, [(0.00975, NOSE_TOP), (0.015, NOSE_TOP), (0.015, -0.0065), (0.01175, -0.0065),
                                  (0.00975, -0.0045)], TEAL, HINGE),  # latch nose, chamfered underneath
                      _rhull(rb, sq([(0.0061, 0.0085), (0.0091, 0.0085), (0.0091, 0.028), (0.0061, 0.028)]), TEAL, HINGE),  # paddle
                      _rhull(rb, sq([(0.0091, 0.0085), (0.0175, 0.0085), (0.0175, 0.011), (0.0091, 0.011)]), TEAL, HINGE),
                  ])
    return Mech(name, pos=tuple(float(v) for v in pos), mass=BODY_MASS, friction=0.4, shapes=shapes, joints=[lever])


CLEAN_TH = 0.0  # the clean cannula hangs upright in its stand, hub up, as it will sit in the connector
CLEAN_HUB = np.array([0.310, -0.100, TOP + 0.056])
BUILT = {"used_cannula": 0.0, "clean_cannula": CLEAN_TH}
USED = cannula("used_cannula", E, 0.0, CREAM)
CLEAN = cannula("clean_cannula", CLEAN_HUB, CLEAN_TH, PALE)

# Stand: two light blue walls with a 13 mm slot between them; the hub rim rests on their tops, the tube hangs in the slot.
SLOT, WALL, STAND_X = 0.013, 0.006, (-0.058, 0.0125)  # slot width, wall thickness, wall ends from the hub axis (x)
STAND_H = float(CLEAN_HUB[2] - TOP) - 0.0002
_sx, _sl = (STAND_X[0] + STAND_X[1]) / 2, STAND_X[1] - STAND_X[0]
CRADLE = Mech("cradle", pos=(float(CLEAN_HUB[0]), float(CLEAN_HUB[1]), TOP), friction=0.4, shapes=[
    box((_sx, -(SLOT + WALL) / 2, STAND_H / 2), (_sl, WALL, STAND_H), CRADLE_BLUE),
    box((_sx, (SLOT + WALL) / 2, STAND_H / 2), (_sl, WALL, STAND_H), CRADLE_BLUE),
    box((STAND_X[0] - 0.002, 0, STAND_H / 2), (0.004, SLOT + 2 * WALL, STAND_H), CRADLE_BLUE),
])
CRADLE_X = (float(CLEAN_HUB[0]) + STAND_X[0] - 0.008, float(CLEAN_HUB[0]) + 0.032)
CRADLE_Y = (float(CLEAN_HUB[1]) - 0.016, float(CLEAN_HUB[1]) + 0.016)

# Red waste tray: 140 x 100 x 30 mm, 3 mm walls, 4 mm floor.
TRAY_AT, TRAY_SIZE, TRAY_WALL, TRAY_FLOOR = (0.190, -0.225), (0.140, 0.110, 0.030), 0.003, 0.004
_tx, _ty, _tz = TRAY_SIZE
TRAY = Mech("waste_tray", pos=(*TRAY_AT, TOP), friction=0.5, shapes=[
    box((0, 0, TRAY_FLOOR / 2), (_tx, _ty, TRAY_FLOOR), TRAY_RED),
    box((-(_tx - TRAY_WALL) / 2, 0, _tz / 2), (TRAY_WALL, _ty, _tz), TRAY_RED),
    box(((_tx - TRAY_WALL) / 2, 0, _tz / 2), (TRAY_WALL, _ty, _tz), TRAY_RED),
    box((0, -(_ty - TRAY_WALL) / 2, _tz / 2), (_tx - 2 * TRAY_WALL, TRAY_WALL, _tz), TRAY_RED),
    box((0, (_ty - TRAY_WALL) / 2, _tz / 2), (_tx - 2 * TRAY_WALL, TRAY_WALL, _tz), TRAY_RED),
])
TRAY_IN_X = (TRAY_AT[0] - _tx / 2 + TRAY_WALL, TRAY_AT[0] + _tx / 2 - TRAY_WALL)
TRAY_IN_Y = (TRAY_AT[1] - _ty / 2 + TRAY_WALL, TRAY_AT[1] + _ty / 2 - TRAY_WALL)

OBJECTS = [PATIENT, TRAY, CRADLE, USED, CLEAN]

# ---- scoring ----------------------------------------------------------------------------------------------------------
SEAT_GAP, SEAT_DEG = 0.002, 5.0  # seated: hub face within 2 mm of the connector top, hub axis within 5 degrees
LATCH_RAD = 0.08  # latched: seated with the lever within this of shut (nose under the lip)
PART_GAP, PART_DEG = 0.025, 30.0  # partly in: hub within 25 mm of the connector top
PLATE_LIFT = 0.005  # D: the neck plate lifted this far at any time (the outer tube dragged out of the neck)
TOUCH = 0.002  # C: the clean cannula came this close to the table, the red tray or the manikin's skin
STILL_SPEED, REST_STEPS = 0.01, 20  # under 1 cm/s for 1 s
RELEASE_CLEAR = 0.03
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it)
PLATE_LO = (-PLATE[0] / 2, -PLATE[1] / 2, float(E[2] - TOP) + PLATE_TOP - PLATE[2])
PLATE_HI = (PLATE[0] / 2, PLATE[1] / 2, float(E[2] - TOP) + 0.0)

_L = {"D": False, "C": False, "still": 0, "seen": None, "steps": 0, "peak_lift": 0.0, "peak_step": 0}


@on_reset
def _clear(_env=None):
    _L.update(D=False, C=False, still=0, seen=None, steps=0, peak_lift=0.0, peak_step=0)


def hub_rot(state, name: str) -> np.ndarray:
    """The hub frame's axes (u, v, w as columns) in the world."""
    return state.rotation() @ rc(BUILT[name])


def lever_open(state) -> float:
    """How far the squeeze lever is open (rad): 0 shut, 0.45 squeezed onto the fin."""
    return -float(state.joints.get("lever", 0.0))


def world_points(state, spec) -> np.ndarray:
    """Every collider point of a cannula in the world, at its live lever angle."""
    return spec.points(dict(state.joints)) @ state.rotation().T + np.asarray(state.origin, dtype=float)


def plate_lift(objects) -> float:
    return float(objects["patient"].joints.get("outer_cannula", 0.0))


def hub_gap(objects, name: str) -> float:
    """Distance from the hub's lower face centre to the centre of the connector's top face."""
    e = E + np.array([0.0, 0.0, plate_lift(objects)])
    return float(np.linalg.norm(np.asarray(objects[name].origin, dtype=float) - e))


def axis_deg(objects, name: str) -> float:
    w = hub_rot(objects[name], name)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(w[2])))))


def seated(objects, name: str = "clean_cannula") -> bool:
    return hub_gap(objects, name) <= SEAT_GAP and axis_deg(objects, name) <= SEAT_DEG


def latched(objects, name: str = "clean_cannula") -> bool:
    return seated(objects, name) and lever_open(objects[name]) <= LATCH_RAD


def partly_in(objects, name: str = "clean_cannula") -> bool:
    return hub_gap(objects, name) <= PART_GAP and axis_deg(objects, name) <= PART_DEG


def in_waste(objects) -> bool:
    """The used cannula wholly inside the red tray's walls and below its rim."""
    p = world_points(objects["used_cannula"], USED)
    return bool(p[:, 0].min() >= TRAY_IN_X[0] and p[:, 0].max() <= TRAY_IN_X[1] and p[:, 1].min() >= TRAY_IN_Y[0]
                and p[:, 1].max() <= TRAY_IN_Y[1] and p[:, 2].max() <= TOP + _tz + 0.002 and p[:, 2].min() >= TOP)


def on_manikin(objects) -> bool:
    """Any part of the used cannula over the manikin: the chest (x 10 to 25.5 cm, |y| 15 cm or less) or the neck and
    head (x 25.5 to 52 cm, |y| 8 cm or less). The stand beside the neck is outside both."""
    p = world_points(objects["used_cannula"], USED)
    x, y = p[:, 0], np.abs(p[:, 1])
    chest = (x >= CHEST_X[0]) & (x <= CHEST_X[1]) & (y <= CHEST_Y)
    neck_head = (x > CHEST_X[1]) & (x <= HEAD_AT[0] + HEAD_R) & (y <= HEAD_R)
    return bool(np.any(chest | neck_head))


def in_channel(p: np.ndarray, lift: float) -> np.ndarray:
    """Which points are inside the outer cannula (the connector, the plate hole or the curved tube under them)."""
    q = p - (C + np.array([0.0, 0.0, lift]))
    rad = np.hypot(q[:, 0], q[:, 2])
    return (np.hypot(q[:, 1], rad - ARC_R) <= LINER_OUT) & (q[:, 0] > -0.012)


def contaminated(objects) -> bool:
    """The clean cannula within 2 mm of the table, the red tray or the manikin's skin (head, chest or neck)."""
    s = objects["clean_cannula"]
    p = world_points(s, CLEAN)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    on_cradle = (x >= CRADLE_X[0]) & (x <= CRADLE_X[1]) & (y >= CRADLE_Y[0]) & (y <= CRADLE_Y[1])
    table = (z <= TOP + TOUCH) & ~on_cradle
    tray = ((x >= TRAY_AT[0] - _tx / 2 - TOUCH) & (x <= TRAY_AT[0] + _tx / 2 + TOUCH) & (y >= TRAY_AT[1] - _ty / 2 - TOUCH)
            & (y <= TRAY_AT[1] + _ty / 2 + TOUCH) & (z <= TOP + _tz + TOUCH))
    head = np.linalg.norm(p - HEAD_AT, axis=1) <= HEAD_R + TOUCH
    chest = ((x >= CHEST_X[0]) & (x <= CHEST_X[1] - 0.010) & (np.abs(y) <= CHEST_Y)
             & (z <= TOP + np.where(np.abs(y) <= CHEST_IN, 0.085, 0.055) + TOUCH))
    neck = ((x > CHEST_X[1] - 0.010) & (x <= NECK_X[1]) & (np.abs(y) <= 0.050) & (z <= TOP + 0.055 + TOUCH)
            & ~in_channel(p, plate_lift(objects)))
    return bool(np.any(table | tray | head | chest | neck))


def _fingers_clear(objects) -> bool:
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return (released(objects["clean_cannula"], RELEASE_CLEAR) and released(objects["used_cannula"], RELEASE_CLEAR)
            and released(objects["patient"], RELEASE_CLEAR, lo=PLATE_LO, hi=PLATE_HI))


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    _L["steps"] += 1
    if plate_lift(objects) > _L["peak_lift"]:  # kept for the run report: the highest lift and when
        _L["peak_lift"], _L["peak_step"] = plate_lift(objects), _L["steps"]
    if plate_lift(objects) >= PLATE_LIFT:
        _L["D"] = True
    if contaminated(objects):
        _L["C"] = True
    both = [objects["used_cannula"], objects["clean_cannula"]]
    _L["still"] = _L["still"] + 1 if all(float(np.linalg.norm(c.lin_vel)) < STILL_SPEED for c in both) else 0


def score(objects):
    """0 if the neck plate ever lifted 5 mm (D), the clean cannula ever touched the table, tray or skin (C), a cannula
    is off the table, or the fingers are not 3 cm clear with everything still for 1 s. Otherwise: 1.0 clean latched,
    used in the tray; 0.75 clean latched, used left on the table or cradle; 0.5 clean latched, used left on the
    manikin; 0.4 clean in the outer tube but not latched (hub within 25 mm), used in the tray; 0.25 used in the tray,
    clean not in; else 0."""
    _update(objects)
    if _L["D"] or _L["C"]:
        return 0.0
    if any(objects[n].origin[2] < TOP - 0.05 for n in ("used_cannula", "clean_cannula")):
        return 0.0
    if _L["still"] < REST_STEPS or not _fingers_clear(objects):
        return 0.0
    waste = in_waste(objects)
    if latched(objects):
        if waste:
            return 1.0
        if latched(objects, "used_cannula") or partly_in(objects, "used_cannula"):
            return 0.0  # cannot happen with one connector; kept so the opposite of the instruction never scores
        return 0.5 if on_manikin(objects) else 0.75
    if waste:
        return 0.4 if partly_in(objects) else 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps off the patient's face ---------------------------------------------------------------------------
FACE_X, FACE_Y = (0.365, 0.52), (-0.09, 0.09)
ARM_KEEP_OUT = [{"name": "patient's head and face", "x": FACE_X, "y": FACE_Y, "z_top": TOP + 0.25}]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
# Tool frame in the hub frame: the gripper points along +v (toward the robot's left when the cannula is seated) and
# its pads close along u, one on the fin and one on the lever's paddle.
LEAN = math.radians(30.0)  # the gripper comes in from the robot's right, pointing 30 degrees down
_ap = np.array([0.0, math.cos(LEAN), -math.sin(LEAN)])
G0 = np.column_stack([[1.0, 0.0, 0.0], np.cross(_ap, [1.0, 0.0, 0.0]), _ap])
GRASP = np.array([0.0, 0.0, 0.025])  # tool point: midway between the fin's face and the squeezed paddle
REACH = np.array([0.004, 0.0, 0.025])  # tool point while the pads are still open round the unsqueezed paddle
OPEN_GAP, REACH_GAP, CLOSE_CMD, HOLD_GAP = 0.050, 0.042, 0.008, 0.014
OUT_DEG = 95.0  # the hub's turn about C when the tip is clear of the connector (its end 3.5 mm above the top)
DROP_ROT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])  # hub frame over the tray: flat, hub toward the patient
DROP_MID = np.array([-0.014, 0.0, -0.0095])  # middle of the cannula's outline in the hub frame
demo = placeholder_demo


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def arc_pose(deg: float, lift: float = 0.0):
    """Hub origin and hub rotation with the cannula turned ``deg`` about the arc centre (0 is seated)."""
    r = rc(math.radians(deg))
    return C + np.array([0.0, 0.0, lift]) + r @ np.array([ARC_R, 0.0, 0.0]), r


def tool(origin, r, at=GRASP):
    """Tool point and gripper rotation that hold a hub at ``origin`` with hub rotation ``r``."""
    return np.asarray(origin, dtype=float) + r @ at, r @ G0


def demo_script(ctx):
    R = ctx.right
    fix, turn = np.zeros(3), np.zeros(3)  # the hand's trim: position (m) and attitude (a small rotation vector)

    def live(name):
        return ctx.obj(name)

    def follow(name, origin, r, secs=0.05, gain=0.3, turn_gain=0.2):
        """One short move that holds the hub at (origin, r), trimming the hand's position and attitude from the
        cannula's read pose. In the 0.5 mm bore a wrist 1 degree off its arc binds the tube, so attitude is trimmed too."""
        nonlocal fix, turn
        if not R.offline:
            s = live(name)
            err = np.asarray(origin, dtype=float) - np.asarray(s.origin, dtype=float)
            fix = np.clip(fix + gain * np.clip(err, -0.003, 0.003), -0.035, 0.035)
            turn = np.clip(turn + turn_gain * np.clip(_rotvec(r @ hub_rot(s, name).T), -0.03, 0.03), -0.2, 0.2)
        p, rot = tool(origin, _rot(turn) @ r)
        yield from R.move(p + fix, secs, rot=rot)

    def carry(name, origin, r, secs):
        """Straight carry of the held cannula's hub to (origin, r), position trimmed each step from its read pose
        (the attitude trim is frozen in free air: it is for the bore)."""
        held = R.rot @ G0.T
        r0 = _rot(turn).T @ held
        o0 = R.pos - fix - held @ GRASP
        n = max(1, round(secs * 20))
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)
            yield from follow(name, o0 + s * (np.asarray(origin, dtype=float) - o0), _slerp(r0, r, s), turn_gain=0.0)

    def grasp(origin, r, back, up=False):
        """Come in along the gripper's own axis, pads open round the fin and paddle, then squeeze."""
        nonlocal fix, turn
        turn = np.zeros(3)
        p, rot = tool(origin, r, REACH)
        pre = p + np.array([0.0, 0.0, back]) if up else p - rot[:, 2] * back
        yield from R.move(pre, 2.0, _finger(OPEN_GAP), rot=rot)
        yield from R.trim_to_measured(pre)
        off = R.pos - pre
        yield from R.move(p + off, 1.5, _finger(REACH_GAP), rot=rot)
        yield from R.trim_to_measured(p)
        off = R.pos - p
        g, _ = tool(origin, r)
        yield from R.move(g + off, 1.2, _finger(CLOSE_CMD), rot=rot)
        yield from R.hold(0.4)
        fix = off.copy()

    def squeezed(name):
        return R.holding(HOLD_GAP) and (R.offline or lever_open(live(name)) >= 0.35)

    # 1) Unlatch and grip the used cannula: a pad on the fin, a pad on the paddle; the squeeze swings the nose out.
    o0, r0 = arc_pose(0.0)
    yield from grasp(o0, r0, 0.045)
    if not squeezed("used_cannula"):
        yield from R.grip(_finger(REACH_GAP), 0.5)
        yield from R.grip(_finger(CLOSE_CMD), 1.0)
        yield from R.hold(0.4)

    # 2) Withdraw along the arc: the hub turns 95 degrees about C. If the plate starts to lift, back off and slow.
    deg, t = 0.0, 0
    while deg < OUT_DEG and t < 400:
        t += 1
        lift = 0.0 if R.offline else plate_lift(ctx.states())
        deg = max(0.0, deg - 2.0) if lift > 0.002 else min(OUT_DEG, deg + OUT_DEG / 120.0)
        yield from follow("used_cannula", *arc_pose(deg))

    # 3) Discard: lift, carry over the red tray, lay the cannula flat, lower it into the tray and let go.
    o, r = arc_pose(OUT_DEG)
    yield from carry("used_cannula", o + np.array([0.0, 0.0, 0.03]), r, 1.0)
    drop = np.array([TRAY_AT[0], TRAY_AT[1] - 0.003, TOP + TRAY_FLOOR + HUB_R + 0.025]) - DROP_ROT @ DROP_MID * np.array([1, 1, 0])
    yield from carry("used_cannula", drop + np.array([0.0, 0.0, 0.10]), DROP_ROT, 3.5)
    yield from carry("used_cannula", drop, DROP_ROT, 2.0)
    yield from R.grip(_finger(OPEN_GAP), 0.8)
    yield from R.hold(1.0)
    if not R.offline:
        in_waste(ctx.states())  # read that it lies inside the walls
    yield from R.move(R.pos + np.array([0.0, 0.0, 0.105]), 1.5, rot=R.rot.copy())
    fix = np.zeros(3)

    # 4) Pick the clean cannula from its stand in the same grasp, coming down over the fin and paddle; lift it out.
    rcl = rc(CLEAN_TH)
    oc = CLEAN_HUB if R.offline else np.asarray(live("clean_cannula").origin, dtype=float)
    yield from grasp(oc, rcl, 0.070, up=True)
    if not squeezed("clean_cannula"):
        yield from R.grip(_finger(REACH_GAP), 0.5)
        yield from R.grip(_finger(CLOSE_CMD), 1.0)
        yield from R.hold(0.4)
    high = oc + np.array([0.0, 0.0, 0.085])
    yield from carry("clean_cannula", high, rcl, 2.5)

    # 5) Insert: bring the tip over the connector from above, then turn the hub back about C, 95 degrees to 0.
    o, r = arc_pose(OUT_DEG)
    yield from carry("clean_cannula", o + np.array([0.0, oc[1], 0.020]), r, 2.5)  # turn it beside the neck
    yield from carry("clean_cannula", o + np.array([0.0, 0.0, 0.020]), r, 2.5)  # cross 2 cm above the connector
    yield from carry("clean_cannula", o, r, 1.0)
    for k in range(10):
        yield from follow("clean_cannula", o, r)
    deg, t = OUT_DEG, 0
    while deg > 0.0 and t < 500:
        t += 1
        lift = 0.0 if R.offline else plate_lift(ctx.states())
        deg = min(OUT_DEG, deg + 1.0) if lift > 0.002 else max(0.0, deg - OUT_DEG / 160.0)
        yield from follow("clean_cannula", *arc_pose(deg))
    for k in range(10):
        yield from follow("clean_cannula", *arc_pose(0.0))
    # Press down the hub axis until the arm lags (the hub is on the connector), then let go: the lever springs shut.
    yield from R.press_until(R.pos.copy(), max_depth=0.002, err_mm=3.0, min_depth=0.0005, speed=0.004)
    yield from R.grip(_finger(REACH_GAP), 0.8)
    yield from R.hold(0.6)
    if not R.offline and not latched(ctx.states()) or (R.offline and R.pessimistic):
        yield from R.grip(_finger(CLOSE_CMD), 0.8)
        yield from R.press_until(R.pos.copy(), max_depth=0.003, err_mm=3.0, min_depth=0.001, speed=0.004)
        yield from R.grip(_finger(REACH_GAP), 0.8)
        yield from R.hold(0.5)

    # 6) Release: open wide, back straight off along the gripper's axis, rise and park.
    yield from R.grip(_finger(OPEN_GAP), 0.5)
    out = R.pos - R.rot[:, 2] * 0.07
    yield from R.move(out, 1.5, rot=R.rot.copy())
    yield from R.move(out + np.array([0.0, -0.06, 0.06]), 1.2, rot=R.rot.copy())
    yield from R.home()
    yield from ctx.wait(2.0)


def _rotvec(m) -> np.ndarray:
    """The small rotation ``m`` as a vector (axis times angle, exact to first order)."""
    return 0.5 * np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])


def _rot(v) -> np.ndarray:
    """Rotation matrix of the rotation vector ``v`` (Rodrigues)."""
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = np.asarray(v, dtype=float) / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def _slerp(r0, r1, s):
    from yam_arena import ik

    return ik.slerp_matrix(r0, r1, s)
