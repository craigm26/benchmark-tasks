"""A laser tracker target on a robot's base plate, at full size. A blue-grey steel base plate carries the base casting
of a new robot arm (its J1 housing, no arm fitted yet) and a Brunson 1.5THDN magnetic drift nest (black anodized aluminium, 40.6 mm across, a magnet under its land). In the nest sits a
1.5 in (38.1 mm) spherically mounted retroreflector (SMR): a polished steel ball with an open cube-corner face (black,
22 mm across) ringed in red. The SMR faces left and 60 degrees up, toward where the tracker stood. The tracker has
moved to the front: a grey post 18 cm in front of the nest carries a laser pointer in a yoke, aimed back at the ball
along the tracker's new line of sight.

The job: turn the SMR in its nest (about two axes, about its own centre) until its face points at the beam post, never
lifting it off its seat and never touching the open optics, then let go.

Requires -E object_jitter=0 (the layout is fixed).

Run it:  inspect-robots run --embodiment yam_arena -E task=turn_laser_tracker_smr_in_drift_nest_to_face_tracker.py \\
           -E object_jitter=0 --instruction "The tracker is now behind the beam post. Turn the SMR in its nest to
           face the post's pointer without lifting it or touching its optics, then let go." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402,F401  (shared kit; scoring here reads the Mech's joint bodies directly)
import mechkit as mk  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, ball, box, cyl, lathe, ring_profile, rod, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0

# -- colours ------------------------------------------------------------------------------------------------------------
PLATE_C = (0.42, 0.48, 0.56)      # painted blue-grey steel base plate
CAST = (0.22, 0.23, 0.25)         # robot base casting
COVER = (0.05, 0.05, 0.06)        # J1 cover
OXIDE = (0.07, 0.07, 0.08)        # black anodized aluminium nest
STEEL = (0.86, 0.87, 0.89)        # polished SMR ball
FACE = (0.01, 0.01, 0.01)         # open cube-corner face
RED = (0.85, 0.06, 0.06)          # SMR red ring, pointer emitter
POST_C = (0.55, 0.56, 0.58)       # post and yoke
POINTER = (0.15, 0.15, 0.17)      # laser pointer body

# -- base plate and robot base ------------------------------------------------------------------------------------------
PLATE_T = 0.015
PLATE = Box("base_plate", pos=(0.27, 0.0, TOP + PLATE_T / 2), size=(0.22, 0.30, PLATE_T), color=PLATE_C)
ROBOT_XY = (0.31, 0.08)           # UR3e-size base: 128 mm flange, base joint housing
ROBOT = Mech("robot_base", pos=(*ROBOT_XY, TOP + PLATE_T), mass=None,
             shapes=[cyl((0.0, 0.0, 0.005), 0.064, 0.010, CAST),                  # mounting flange
                     cyl((0.0, 0.0, 0.055), 0.045, 0.090, CAST),                   # base joint housing
                     cyl((0.0, 0.0, 0.115), 0.045, 0.030, COVER)])                 # J1 cover

# -- the drift nest and the SMR -------------------------------------------------------------------------------------------
NEST_XY = (0.24, -0.06)
R_BALL = 0.01905                  # 1.5 in SMR
CENTRE_H = 0.0286                 # 1.125 in: SMR centre above the nest foot (Brunson 1.5THDN)
NEST_R, NEST_H = 0.0203, 0.0127   # Brunson 1.5THDN: 1.600 in across, 0.5 in tall (maker page)
CENTRE = np.array([NEST_XY[0], NEST_XY[1], TOP + PLATE_T + CENTRE_H])    # ball centre, world
APERTURE_R = 0.011                # open cube corner 22 mm across
FACE_Z = math.sqrt(R_BALL ** 2 - APERTURE_R ** 2)                        # 15.55 mm: the face plane from the centre
EL0 = math.radians(60.0)          # start elevation of the face (fixed layout); start azimuth +y (yaw joint 0)
LIFT_K = 3000.0                   # N/m: magnet pull modelled as a spring to the seat (9 N at 3 mm)
TURN_HOLD = 0.02                  # N m: seat friction against turning (yaw and tilt)
BALL_MASS = 0.150                 # FARO 1.5 in standard SMR, 440C housing: 150 g (FARO SMR specifications)


def _qx(a):
    """(w, x, y, z) quaternion of a turn about +x."""
    return (math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)


def _nest_shapes():
    """Fixed nest in the Mech frame (origin on the plate top under the ball centre): a black anodized aluminium cup with a
    floor, a 12.5 mm bore (1.5 mm round the ball at the rim) and three look-only land balls on a 45 degree cone."""
    out = [cyl((0.0, 0.0, 0.004), NEST_R, 0.008, OXIDE),
           ring_profile((0.0, 0.0, 0.008), [(0.0125, NEST_R, 0.0), (0.0125, NEST_R, NEST_H - 0.008)], OXIDE)]
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 6
        rr = (R_BALL + 0.0025) * math.sin(math.pi / 4)
        out.append(dataclasses.replace(cyl((rr * math.cos(a), rr * math.sin(a), CENTRE_H - rr), 0.002, 0.003, STEEL),
                                       collide=False))
    return out


def _smr_shapes():
    """The SMR in the tilt body's frame (origin at the ball centre). Collides as a full hidden sphere (centre of mass
    on both axes, so neither hinge carries a gravity torque); drawn as a polished ball cut at the face plane, the
    black open cube corner and a red ring, the face pointing +y and EL0 up."""
    tilt = _qx(-(math.pi / 2 - EL0))          # turns +z (the drawn face axis) to (0, cos EL0, sin EL0)
    prof = [(0.0, -R_BALL)] + [(R_BALL * math.sin(a), -R_BALL * math.cos(a))
                               for a in np.linspace(0.45, math.acos(-FACE_Z / R_BALL) - 0.12, 9)]
    prof = [(r, z) for r, z in prof if z < FACE_Z - 1e-4] + [(APERTURE_R, FACE_Z)]
    look = [lathe((0.0, 0.0, 0.0), prof, STEEL, tilt=tilt),
            cyl(tuple(_rot_x(-(math.pi / 2 - EL0)) @ np.array([0.0, 0.0, FACE_Z - 0.0006])), APERTURE_R - 0.0002,
                0.0014, FACE, tilt=tilt),
            tube(tuple(_rot_x(-(math.pi / 2 - EL0)) @ np.array([0.0, 0.0, FACE_Z - 0.0022])), 0.0111, 0.0134,
                 0.0020, RED, tilt=tilt)]
    look = [dataclasses.replace(s, collide=False) for s in look]
    return [dataclasses.replace(ball((0.0, 0.0, 0.0), R_BALL, STEEL, rows=10), visible=False)] + look


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


SPACER = (dataclasses.replace(cyl((0.0, 0.0, 0.0), 0.003, 0.003, STEEL), collide=False),)   # look-only, hidden inside the ball
TILT_LOW, TILT_HIGH = -math.radians(80.0), math.radians(30.0)     # face elevation -20 .. 90 degrees
NEST = Mech(
    "smr_nest", pos=(*NEST_XY, TOP + PLATE_T), mass=None, friction=0.5,
    shapes=_nest_shapes(),
    joints=[
        Joint("lift", "slide", SPACER, axis="z", at=(0.0, 0.0, CENTRE_H), low=0.0, high=0.005, spring=LIFT_K,
              damping=20.0, mass=0.004),
        Joint("yaw", "hinge", SPACER, axis="z", at=(0.0, 0.0, 0.0), low=-3.2, high=3.2, parent="lift",
              hold=TURN_HOLD, hold_moving=0.8 * TURN_HOLD, damping=0.002, mass=0.004, armature=2e-6),
        Joint("tilt", "hinge", tuple(_smr_shapes()), axis="x", at=(0.0, 0.0, 0.0), low=TILT_LOW, high=TILT_HIGH,
              parent="yaw", hold=TURN_HOLD, hold_moving=0.8 * TURN_HOLD, damping=0.002, mass=BALL_MASS - 0.008,
              friction=0.5, armature=2e-6),
    ],
)

# -- the beam post: the tracker's new line of sight, at azimuth 0 (+x) and 15 degrees up from the ball centre --------------
BEAR_AZ, BEAR_EL = 0.0, math.radians(15.0)
POST_DIST = 0.18
POST_XY = (CENTRE[0] + POST_DIST, CENTRE[1])
HEAD = np.array([POST_XY[0], POST_XY[1], CENTRE[2] + POST_DIST * math.tan(BEAR_EL)])   # pointer emitter centre
PTR_R, PTR_L = 0.0080, 0.080      # pen laser pointer body: 16 mm by 80 mm (front half shown past the yoke)
POST = Mech("beam_post", pos=(POST_XY[0], POST_XY[1], TOP), mass=None,
            shapes=[cyl((0.0, 0.0, 0.004), 0.035, 0.008, POST_C),                                  # weighted foot
                    rod((0.0, 0.0, 0.008), (0.0, 0.0, HEAD[2] - TOP - 0.016), 0.006, POST_C),       # upright
                    box((0.0, 0.0, HEAD[2] - TOP - 0.014), (0.016, 0.040, 0.004), POST_C),          # yoke base
                    box((0.0, 0.016, HEAD[2] - TOP - 0.004), (0.016, 0.004, 0.024), POST_C),        # yoke arms
                    box((0.0, -0.016, HEAD[2] - TOP - 0.004), (0.016, 0.004, 0.024), POST_C),
                    cyl((0.035 - PTR_L / 2 + 0.0, 0.0, HEAD[2] - TOP), PTR_R, PTR_L, POINTER, axis="x"),
                    cyl((-0.006, 0.0, HEAD[2] - TOP), PTR_R + 0.0015, 0.004, RED, axis="x")])        # red bezel

OBJECTS = [PLATE, ROBOT, NEST, POST]

# -- measurements -------------------------------------------------------------------------------------------------------
LIFT_MAX = 0.003                  # m: ball off its land by 3 mm or more ...
LIFT_STEPS = 4                    # ... for 0.2 s unbroken scores 0
ZONE_R, ZONE_A0, ZONE_A1 = 0.013, 0.012, 0.030   # aperture zone: within 13 mm of the axis, 12 to 30 mm out
TOUCH_STEPS = 2
CLEAR = 0.03                      # every pad point 3 cm off the ball surface
FINAL_STEPS = 20                  # 1 s at 20 Hz
STILL_DEG = 0.5                   # yaw and tilt each within 0.5 deg over the last 1 s
AZ_TOL = 10.0                     # stage 1: face azimuth within 10 deg of the bearing


def target_dir():
    v = HEAD - CENTRE
    return v / np.linalg.norm(v)


def face_axis(st):
    """(ball centre, unit face axis) in the world from the nest's MechState."""
    rot, org = mk.body_poses(NEST, st)["tilt"]
    d0 = np.array([0.0, math.cos(EL0), math.sin(EL0)])
    return np.asarray(org, dtype=float), rot @ d0


def aim_error_deg(st) -> float:
    _, a = face_axis(st)
    return math.degrees(math.acos(float(np.clip(a @ target_dir(), -1.0, 1.0))))


def azimuth_error_deg(st) -> float:
    _, a = face_axis(st)
    t = target_dir()
    da = math.atan2(a[1], a[0]) - math.atan2(t[1], t[0])
    return abs(math.degrees((da + math.pi) % (2 * math.pi) - math.pi))


def in_aperture_zone(st, pts) -> bool:
    c, a = face_axis(st)
    for p in np.asarray(pts, dtype=float).reshape(-1, 3):
        v = p - c
        along = float(v @ a)
        if ZONE_A0 <= along <= ZONE_A1 and float(np.linalg.norm(v - along * a)) <= ZONE_R:
            return True
    return False


def hands_clear(st, pts) -> bool:
    c, _ = face_axis(st)
    d = np.linalg.norm(np.asarray(pts, dtype=float).reshape(-1, 3) - c, axis=1) - R_BALL
    return bool(d.min() >= CLEAR)


_LATCH: dict = {}


@mk.on_reset
def _reset(env=None):
    _LATCH.clear()


def _pads():
    """Every finger pad point of both arms, or None offline (then nothing counts as let go or as a touch)."""
    try:
        return np.array(mk.finger_points())
    except Exception:  # noqa: BLE001  offline: no live env
        return None


def observe(objects, pts=None):
    """Update the run latches from one step's state and the pad points (None: unknown, never released)."""
    s = _LATCH
    st = objects["smr_nest"]
    lift = float(st.joints["lift"])
    s["lift_run"] = s.get("lift_run", 0) + 1 if lift >= LIFT_MAX else 0
    if s["lift_run"] >= LIFT_STEPS:
        s["lost"] = "SMR lifted off its land"
    touch = pts is not None and in_aperture_zone(st, pts)
    s["touch_run"] = s.get("touch_run", 0) + 1 if touch else 0
    if s["touch_run"] >= TOUCH_STEPS:
        s["lost"] = "a finger in the open optics"
    clear = pts is not None and hands_clear(st, pts)
    s["clear_run"] = s.get("clear_run", 0) + 1 if clear else 0
    hist = s.setdefault("hist", [])
    hist.append((math.degrees(float(st.joints["yaw"])), math.degrees(float(st.joints["tilt"])), aim_error_deg(st),
                 azimuth_error_deg(st)))
    del hist[:-FINAL_STEPS]
    s["n"] = s.get("n", 0) + 1


def grade() -> float:
    s = _LATCH
    hist = s.get("hist", [])
    if s.get("lost") or s.get("clear_run", 0) < FINAL_STEPS or len(hist) < FINAL_STEPS:
        return 0.0
    h = np.array(hist)
    if np.ptp(h[:, 0]) > STILL_DEG or np.ptp(h[:, 1]) > STILL_DEG:
        return 0.0
    aim, az = float(h[:, 2].mean()), float(h[:, 3].mean())
    if aim <= 10.0:
        return 1.0
    if aim <= 20.0:
        return 0.5
    if aim <= 30.0 or az <= AZ_TOL:
        return 0.25
    return 0.0


def score(objects):
    """Aim error A: angle from the SMR's face axis to the line from the ball centre to the pointer's emitter, mean of
    the last 1 s. 0 for the run if the ball is ever 3 mm or more off its land for 0.2 s, or a finger pad point is in
    the aperture zone (within 13 mm of the face axis, 12 to 30 mm out) 2 steps running. Else 0 unless every pad point
    of both arms is 3 cm off the ball and yaw and tilt each moved under 0.5 deg over the last 1 s. Then 1.0 if A <= 10
    deg, 0.5 if <= 20, 0.25 if <= 30 or the face's azimuth is within 10 deg of the bearing, else 0."""
    step = mk.step_count()
    if step == 0 or _LATCH.get("step") != step:     # score() and success() share a step: update once
        _LATCH["step"] = step
        observe(objects, _pads())
    return grade()


def success(objects):
    return score(objects) == 1.0


# -- keep-outs the arms respect (sweep.py checks every arm link offline) ------------------------------------------------
ARM_KEEP_OUT = [
    {"name": "robot base casting", "x": (0.23, 0.39), "y": (0.00, 0.16), "z_top": TOP + 0.25},
    {"name": "beam post and pointer", "x": (0.34, 0.47), "y": (-0.10, -0.02), "z_top": TOP + 0.25},
]

# -- the scripted demo (right arm only; the left arm holds READY) -----------------------------------------------------------
demo = placeholder_demo
PINCH_UP = 0.005                  # tool point 5 mm above the centre: pad tips 6 mm above the cup rim
TIP = 0.013                       # closed fingertips reach 13 mm past the tool point
PHI0 = math.radians(125.0)        # roll contact: 35 deg behind top dead centre, in the face's vertical plane
PRESS = 0.005                     # press 5 mm past the surface
YAW_GOAL = BEAR_AZ - math.pi / 2  # yaw joint value that turns the face from +y to the bearing
TILT_GOAL = BEAR_EL - EL0         # tilt joint value that brings the face down to the bearing's elevation
TILT_OVER = math.radians(5.0)     # roll 5 deg past it: the ball springs back a little as the finger leaves
TILT_REDO = math.radians(4.0)     # still more than 4 deg short of the goal after the finger leaves: roll again


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo_script(ctx):
    """Reads the SMR's yaw and tilt joint values (the nest Mech's joints) from the simulator to plan each turn, plus its
    joint encoders and gripper opening; every position comes from the fixed layout."""
    R = ctx.right
    def read(name, fallback):
        if R.offline:                       # dry run: the mock never turns, so trust the command
            return fallback
        return float(ctx.obj("smr_nest").joints[name])

    above = CENTRE + [0.0, 0.0, 0.08]
    pinch = CENTRE + [0.0, 0.0, PINCH_UP]

    # 1. Yaw: pinch across the ball on the tilt axis (pads close along it, clear of the face), turn the wrist.
    yaw = 0.0
    g_cmd = 0.0
    for _stroke in range(3):
        yaw = read("yaw", yaw)
        err = _wrap(yaw - YAW_GOAL)
        if abs(err) <= math.radians(2.0):
            break
        g = _wrap(yaw + math.pi / 2)        # pads along the tilt axis (Mech x turned by yaw)
        g = g - math.pi if g > math.pi / 2 + 0.3 else (g + math.pi if g < -math.pi / 2 - 0.3 else g)
        yield from R.move(above, 4.0, FINGER_OPEN, yaw=g)
        yield from R.trim_to_measured(above)
        yield from R.move(pinch, 1.5, yaw=g)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.3)
        if not R.holding(0.025):            # pads did not stop on the 38 mm ball: open, back up, try again
            yield from R.grip(FINGER_OPEN, 0.6)
            yield from R.move(above, 1.0, yaw=g)
            continue
        yield from R.move(pinch + [0.0, 0.0, -0.002], 0.3, yaw=g)     # 2 mm downward bias keeps it on its land
        g_cmd = g
        for _ in range(400):                # at most 0.3 deg a step; keep turning until the ball reads the goal
            yaw = _wrap(g_cmd - math.pi / 2) if R.offline else read("yaw", yaw)
            err = _wrap(yaw - YAW_GOAL)
            if abs(err) <= math.radians(1.0) or g_cmd < -0.70:
                break
            g_cmd -= min(math.radians(0.3), err + math.radians(0.5))
            yield from R.move(pinch + [0.0, 0.0, -0.002], 1.0 / 20.0, yaw=g_cmd)
        yield from R.hold(0.4)
        yield from R.grip(FINGER_OPEN, 0.8)
        yield from R.move(above, 1.2, yaw=g_cmd)
    # 2. Tilt: a closed fingertip on the back of the ball rolls it about the tilt axis toward the beam post.
    yield from R.grip(FINGER_CLOSED, 0.6)
    yaw = read("yaw", YAW_GOAL)
    az = yaw + math.pi / 2                  # the face's azimuth now (start azimuth +y)
    u = np.array([math.cos(az), math.sin(az), 0.0])

    def tip_pose(phi, depth):
        n = math.cos(phi) * u + np.array([0.0, 0.0, math.sin(phi)])
        pitch = max(phi - math.pi / 2, 0.0)
        rot = grip_rot(yaw=az, pitch=pitch)
        a = rot @ np.array([0.0, 0.0, 1.0])
        return CENTRE + (R_BALL - depth) * n - TIP * a, rot

    tilt = read("tilt", 0.0)
    phi_cmd = PHI0
    for _stroke in range(3):                # a short or sprung-back roll gets another stroke from behind
        if _stroke and tilt <= TILT_GOAL + TILT_REDO:
            break
        stop = TILT_GOAL - (TILT_OVER if _stroke == 0 else 0.0)
        phi0 = PHI0 if _stroke == 0 else PHI0 + math.radians(10.0)   # touch the ball behind its current top
        p, r = tip_pose(phi0, -0.03)
        yield from R.move(p + [0.0, 0.0, 0.03], 2.5 if _stroke == 0 else 1.5, rot=r)
        yield from R.move(p, 1.2, rot=r)
        yield from R.trim_to_measured(p)
        p3, _ = tip_pose(phi0, -0.003)
        a = r @ np.array([0.0, 0.0, 1.0])
        res = yield from R.press_until(p3, max_depth=0.003 + PRESS + 0.001, err_mm=4.0, min_depth=0.003 + 0.002,
                                       direction=a, rot=r)
        depth = min(PRESS, max(0.002, res.depth - 0.003))
        tilt0 = tilt = read("tilt", tilt)
        lead = math.radians(6.0)
        phi_cmd = phi0
        for _ in range(500):
            tilt = read("tilt", tilt)
            if R.offline:
                tilt = tilt0 + phi_cmd - phi0
            if tilt <= stop:
                break
            phi_mat = phi0 + tilt - tilt0    # the ball point first touched, carried round with the ball
            stalled = phi_cmd < phi_mat - lead + 1e-9
            lead = min(math.radians(20.0), lead + math.radians(0.15)) if stalled else lead
            phi_cmd = max(phi_cmd - math.radians(0.6), phi_mat - lead, stop + phi0 - tilt0 - math.radians(3.0))
            p, r = tip_pose(phi_cmd, depth)
            yield from R.move(p, 1.0 / 20.0, rot=r)
        # stop the finger on the ball point it is touching now (drop the lead that would carry the ball on), unloaded
        phi_cmd = max(phi_cmd, phi0 + tilt - tilt0) if not R.offline else phi_cmd
        p, r = tip_pose(phi_cmd, 0.0)
        yield from R.move(p, 0.6, rot=r)
        yield from R.hold(0.5)
        p_off, r = tip_pose(phi_cmd, -0.03)     # back off along the normal and let the ball settle
        yield from R.move(p_off, 1.2, rot=r)
        yield from R.hold(0.5)
        tilt = read("tilt", stop)
    # 3. Rise and park.
    yield from R.move(CENTRE + [-0.04, 0.0, 0.15], 2.5, rot=r)
    yield from R.home(4.5)
    yield from ctx.wait(3.0)
