"""On the station table: a foam-coloured manikin head (a 12 cm sphere) in front of the arms, with a dark forehead mark
on top, and on the left a grey pad with a white no-touch thermometer (a 4 x 4 x 8 cm block) standing on it. The
thermometer is loose; the head, the mark and the pad are fixed.

Requires -E object_jitter=0: the head column, the reading band and the pad are measured from fixed positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=no_touch_thermometer_hover.py -E object_jitter=0 \\
           --instruction "With the white no-touch thermometer upright, hold its bottom end 2 to 5 cm straight above
           the manikin's dark forehead mark for 3 seconds, never closer than 2 cm to the head, then stand it back
           on its grey pad." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from belimits import Rect, bottom, tilt_deg  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SUCCESS_HOLD_S = 8.0  # the episode may end only after the demo has let go and moved clear

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SKIN, MARK, GREY, WHITE = (0.87, 0.72, 0.60), (0.55, 0.35, 0.25), (0.55, 0.55, 0.58), (0.95, 0.95, 0.95)

# The manikin head: a fixed 6 cm radius sphere; its top (the forehead) is at TOP + 0.12. The mark is a 4 cm wide disc.
HEAD_XY, HEAD_R = (0.22, 0.0), 0.06
HEAD_C = np.array([HEAD_XY[0], HEAD_XY[1], TOP + HEAD_R])
FOREHEAD = TOP + 2 * HEAD_R
HEAD = Sphere("manikin_head", pos=tuple(HEAD_C), radius=HEAD_R, color=SKIN)
MARK_DISC = Cylinder("forehead_mark", pos=(*HEAD_XY, FOREHEAD + 0.001), radius=0.02, height=0.002, color=MARK)

# The stand pad (8 x 8 cm, 4 mm thick) and the thermometer standing on it (4 x 4 x 8 cm; its bottom face is the sensor).
PAD_XY, PAD_H = (0.16, 0.16), 0.004
PAD = Box("stand_pad", pos=(*PAD_XY, TOP + PAD_H / 2), size=(0.08, 0.08, PAD_H), color=GREY)
T_SIZE = (0.04, 0.04, 0.08)
THERM = Box("thermometer", pos=(*PAD_XY, TOP + PAD_H + T_SIZE[2] / 2), size=T_SIZE, mass=0.06, color=WHITE)

OBJECTS = [HEAD, MARK_DISC, PAD, THERM]

SHELL = 0.02  # m: nothing may come within this of the head
BREACH_DIST = HEAD_R + SHELL
BAND = (FOREHEAD + 0.02, FOREHEAD + 0.05)  # the thermometer's bottom must be between these heights
BAND_R = 0.02  # m: its centre within this of the mark, seen from above
READ_TILT, HOME_TILT = 20.0, 15.0  # degrees
READ_STEPS = 60  # 3.0 s at 20 Hz
CONTROL_HZ = 20


class _Run:
    """What the scorer remembers across control steps. It clears on the first call and on a real trial reset (the
    thermometer jumps more than 3 cm in one step and lands back on its spawn point, still)."""

    def __init__(self):
        self.clear()
        self.prev = None

    def clear(self):
        self.breach, self.read, self.run = False, False, 0


_R = _Run()


def _dist_to_head(t) -> float:
    """Metres from the head's centre to the thermometer's world-aligned bounding box."""
    p = np.clip(HEAD_C, t.low, t.high)
    return float(np.linalg.norm(HEAD_C - p))


def _in_band(t) -> bool:
    b = bottom(t, THERM)
    return bool(
        np.hypot(t.pos[0] - HEAD_XY[0], t.pos[1] - HEAD_XY[1]) <= BAND_R
        and BAND[0] <= b <= BAND[1]
        and tilt_deg(t) <= READ_TILT
    )


def _advance(objects) -> None:
    """Call once per control step, from score() only."""
    t = objects["thermometer"]
    spawn = np.asarray(THERM.pos, float)
    d_spawn = float(np.linalg.norm(np.asarray(t.pos, float) - spawn))
    still = float(np.linalg.norm(t.lin_vel)) < 0.01
    if _R.prev is None or (_R.prev > 0.03 and d_spawn < 0.005 and still):
        _R.clear()
    _R.prev = d_spawn
    if _dist_to_head(t) < BREACH_DIST or t.low[2] < TOP - 0.05:
        _R.breach = True
    _R.run = _R.run + 1 if (_in_band(t) and not _R.breach) else 0
    if _R.run >= READ_STEPS:
        _R.read = True


def _home(t) -> bool:
    """Standing upright, still, on the pad: centre within 3 cm of the pad's centre, bottom within 8 mm of the table."""
    return bool(
        np.hypot(t.pos[0] - PAD_XY[0], t.pos[1] - PAD_XY[1]) <= 0.03
        and bottom(t, THERM) <= TOP + 0.008
        and tilt_deg(t) <= HOME_TILT
        and t.resting(max_speed=0.01)
    )


def score(objects):
    """0 if the thermometer ever came within 2 cm of the head or fell off the table. Otherwise 0.5 once a reading was
    made (3 s in a row, bottom 2 to 5 cm above the forehead, over the mark, within 20 degrees of upright), plus 0.5 if it
    then stands still and upright on the pad. A held thermometer (bottom more than 8 mm above the table) is not home."""
    _advance(objects)
    if _R.breach or not _R.read:
        return 0.0
    return 0.5 + (0.5 if _home(objects["thermometer"]) else 0.0)


def success(objects):
    """Read and home. Reads the latches score() set on the previous step; it advances nothing itself."""
    return bool(_R.read and not _R.breach and _home(objects["thermometer"]))


# The robot keeps out too: no part of either arm may enter the column over the head (the head grown by 2 cm, then
# its footprint squared up) below the height a centre-held thermometer needs to hover. Checked offline with the URDF.
ARM_KEEP_OUT = [
    {"name": "head column", "x": (HEAD_XY[0] - BREACH_DIST, HEAD_XY[0] + BREACH_DIST),
     "y": (-BREACH_DIST, BREACH_DIST), "z_top": FOREHEAD + 0.05},
]

HOVER_TOOL_Z = FOREHEAD + 0.035 + T_SIZE[2] / 2  # tool height putting the bottom end 3.5 cm above the forehead
CARRY = (HEAD_XY[0], HEAD_XY[1], HOVER_TOOL_Z)
GRIP_Z = TOP + PAD_H + T_SIZE[2] / 2
HOLD_S = 4.5


def demo():
    """The left arm lifts the thermometer off its pad, carries it over the mark, holds it still 4.5 s with its bottom
    end 3.5 cm above the forehead, carries it back, stands it on the pad and lets go. The right arm holds READY."""
    px, py = PAD_XY
    hx, hy, hz = CARRY
    up = HOVER_TOOL_Z
    return {
        "left": [
            Waypoint(2.5, (px, py, up), FINGER_OPEN, "to the pad"),
            Waypoint(1.5, (px, py, GRIP_Z), FINGER_OPEN, "descend"),
            Waypoint(0.8, (px, py, GRIP_Z), FINGER_CLOSED, "grip"),
            Waypoint(1.5, (px, py, up), FINGER_CLOSED, "lift"),
            Waypoint(3.0, (hx, hy, hz), FINGER_CLOSED, "carry over the mark"),
            Waypoint(HOLD_S, (hx, hy, hz), FINGER_CLOSED, "hold still"),
            Waypoint(3.0, (px, py, up), FINGER_CLOSED, "carry back"),
            Waypoint(1.5, (px, py, GRIP_Z + 0.003), FINGER_CLOSED, "lower"),
            Waypoint(0.8, (px, py, GRIP_Z + 0.003), FINGER_OPEN, "let go"),
            Waypoint(1.5, (px, py, up), FINGER_OPEN, "retreat"),
        ]
    }
