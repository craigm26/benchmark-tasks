"""Standard-library ASCII STL generator for the quick-acting knob and HALT fixture parts.

Run directly as a script to generate millimeter-scale ASCII STLs matching the scene dimensions in
``ext_ai_robot_vibration_shock_ip_testing_9.py``. Importing this module writes no files.
"""

from __future__ import annotations

import math
from pathlib import Path

# Dimensions in mm matching ext_ai_robot_vibration_shock_ip_testing_9.py
PLATE_SIZE_MM = (160.0, 200.0, 12.0)
CROSSBAR_SIZE_MM = (25.0, 180.0, 12.0)
STUD_OFFSET_Y_MM = 75.0
STUD_CLEAR_HOLE_R_MM = 4.5
STAGE_SIZE_MM = (44.0, 44.0, 12.0)
STAGE_PED_R_MM = 9.0
STAGE_PED_H_MM = 10.0
STAGE_PEG_R_MM = 2.1
STAGE_PEG_H_MM = 8.0

KNOB_R_OUT_MM = 11.0
KNOB_R_IN_MM = 9.0
KNOB_H_MM = 18.0
LUG_X_INNER_MM = 3.25
LUG_W_Y_MM = 7.6
LUG_H_Z_MM = 2.2
LUG_Z_OFFSET_MM = 7.7
Y_WALL_Y_INNER_MM = 6.4
Y_WALL_THICK_MM = 3.4
Y_WALL_W_X_MM = 6.0


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v):
    m = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / m, v[1] / m, v[2] / m) if m > 1e-12 else (0.0, 0.0, 1.0)


def _box_tris(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float):
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    v = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (2, 3, 7), (2, 7, 6),
        (1, 2, 6), (1, 6, 5),
        (3, 0, 4), (3, 4, 7),
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def _cyl_tris(cx: float, cy: float, z0: float, z1: float, r: float, sides: int = 36):
    tris = []
    for i in range(sides):
        a0 = 2.0 * math.pi * i / sides
        a1 = 2.0 * math.pi * (i + 1) / sides
        p00 = (cx + r * math.cos(a0), cy + r * math.sin(a0), z0)
        p01 = (cx + r * math.cos(a1), cy + r * math.sin(a1), z0)
        p10 = (cx + r * math.cos(a0), cy + r * math.sin(a0), z1)
        p11 = (cx + r * math.cos(a1), cy + r * math.sin(a1), z1)
        tris.append(((cx, cy, z0), p01, p00))
        tris.append(((cx, cy, z1), p10, p11))
        tris.append((p00, p01, p11))
        tris.append((p00, p11, p10))
    return tris


def _tube_tris(cx: float, cy: float, z0: float, z1: float, r_in: float, r_out: float, sides: int = 36):
    tris = []
    for i in range(sides):
        a0 = 2.0 * math.pi * i / sides
        a1 = 2.0 * math.pi * (i + 1) / sides
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        i0b, i1b = (cx + r_in * c0, cy + r_in * s0, z0), (cx + r_in * c1, cy + r_in * s1, z0)
        o0b, o1b = (cx + r_out * c0, cy + r_out * s0, z0), (cx + r_out * c1, cy + r_out * s1, z0)
        i0t, i1t = (cx + r_in * c0, cy + r_in * s0, z1), (cx + r_in * c1, cy + r_in * s1, z1)
        o0t, o1t = (cx + r_out * c0, cy + r_out * s0, z1), (cx + r_out * c1, cy + r_out * s1, z1)
        tris.extend([(i0b, o1b, o0b), (i0b, i1b, o1b)])
        tris.extend([(i0t, o0t, o1t), (i0t, o1t, i1t)])
        tris.extend([(o0b, o1b, o1t), (o0b, o1t, o0t)])
        tris.extend([(i0b, i1t, i1b), (i0b, i0t, i1t)])
    return tris


def _bored_block_tris(cx: float, cy: float, z0: float, z1: float, sx: float, sy: float, r: float, sides: int = 36):
    hx, hy = sx / 2.0, sy / 2.0
    tris = []
    for i in range(sides):
        a0 = 2.0 * math.pi * i / sides
        a1 = 2.0 * math.pi * (i + 1) / sides
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        # Project ray onto rectangle boundary
        k0 = min(hx / max(abs(c0), 1e-9), hy / max(abs(s0), 1e-9))
        k1 = min(hx / max(abs(c1), 1e-9), hy / max(abs(s1), 1e-9))
        i0b, i1b = (cx + r * c0, cy + r * s0, z0), (cx + r * c1, cy + r * s1, z0)
        o0b, o1b = (cx + k0 * c0, cy + k0 * s0, z0), (cx + k1 * c1, cy + k1 * s1, z0)
        i0t, i1t = (cx + r * c0, cy + r * s0, z1), (cx + r * c1, cy + r * s1, z1)
        o0t, o1t = (cx + k0 * c0, cy + k0 * s0, z1), (cx + k1 * c1, cy + k1 * s1, z1)
        tris.extend([(i0b, o1b, o0b), (i0b, i1b, o1b)])
        tris.extend([(i0t, o0t, o1t), (i0t, o1t, i1t)])
        tris.extend([(o0b, o1b, o1t), (o0b, o1t, o0t)])
        tris.extend([(i0b, i1t, i1b), (i0b, i0t, i1t)])
    return tris


def _write_stl(path: Path, name: str, tris) -> None:
    lines = [f"solid {name}"]
    for p0, p1, p2 in tris:
        n = _norm(_cross(_sub(p1, p0), _sub(p2, p0)))
        lines.append(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}")
        lines.append("    outer loop")
        lines.append(f"      vertex {p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f}")
        lines.append(f"      vertex {p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}")
        lines.append(f"      vertex {p2[0]:.6f} {p2[1]:.6f} {p2[2]:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def build_all(out_dir: Path | None = None) -> list[Path]:
    target = Path(out_dir) if out_dir is not None else Path(__file__).resolve().parent / "stl"
    target.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    # 1. Quick-acting knob replica (22 mm OD x 18 mm tall, relieved bore + opposing half-thread lugs)
    lug_wx = KNOB_R_IN_MM - LUG_X_INNER_MM
    lug_cx = LUG_X_INNER_MM + lug_wx / 2.0
    knob_tris = []
    knob_tris.extend(_tube_tris(0.0, 0.0, 0.0, KNOB_H_MM, KNOB_R_IN_MM, KNOB_R_OUT_MM, sides=36))
    knob_tris.extend(_box_tris(-lug_cx, 0.0, KNOB_H_MM / 2.0 + LUG_Z_OFFSET_MM, lug_wx, LUG_W_Y_MM, LUG_H_Z_MM))
    knob_tris.extend(_box_tris(+lug_cx, 0.0, KNOB_H_MM / 2.0 - LUG_Z_OFFSET_MM, lug_wx, LUG_W_Y_MM, LUG_H_Z_MM))
    y_wall_cy = Y_WALL_Y_INNER_MM + Y_WALL_THICK_MM / 2.0
    knob_tris.extend(_box_tris(0.0, +y_wall_cy, KNOB_H_MM / 2.0, Y_WALL_W_X_MM, Y_WALL_THICK_MM, KNOB_H_MM))
    knob_tris.extend(_box_tris(0.0, -y_wall_cy, KNOB_H_MM / 2.0, Y_WALL_W_X_MM, Y_WALL_THICK_MM, KNOB_H_MM))
    p_knob = target / "knob_replica.stl"
    _write_stl(p_knob, "knob_replica", knob_tris)
    written.append(p_knob)

    # 2. Slotted/bored aluminum crossbar (25 x 180 x 12 mm, 9 mm stud holes at y = +-75 mm)
    cb_tris = []
    cb_tris.extend(_bored_block_tris(0.0, -STUD_OFFSET_Y_MM, 0.0, CROSSBAR_SIZE_MM[2], CROSSBAR_SIZE_MM[0], 30.0, STUD_CLEAR_HOLE_R_MM))
    cb_tris.extend(_bored_block_tris(0.0, +STUD_OFFSET_Y_MM, 0.0, CROSSBAR_SIZE_MM[2], CROSSBAR_SIZE_MM[0], 30.0, STUD_CLEAR_HOLE_R_MM))
    cb_tris.extend(_box_tris(0.0, 0.0, CROSSBAR_SIZE_MM[2] / 2.0, CROSSBAR_SIZE_MM[0], 120.0, CROSSBAR_SIZE_MM[2]))
    p_cb = target / "crossbar.stl"
    _write_stl(p_cb, "crossbar", cb_tris)
    written.append(p_cb)

    # 3. Staging block with pedestal and 5.2 mm alignment peg (44 x 44 x 12 mm + 10 mm pedestal + 10 mm peg)
    st_tris = []
    st_tris.extend(_box_tris(0.0, 0.0, STAGE_SIZE_MM[2] / 2.0, *STAGE_SIZE_MM))
    ped_top = STAGE_SIZE_MM[2] + STAGE_PED_H_MM
    st_tris.extend(_cyl_tris(0.0, 0.0, STAGE_SIZE_MM[2], ped_top, STAGE_PED_R_MM))
    st_tris.extend(_cyl_tris(0.0, 0.0, ped_top, ped_top + STAGE_PEG_H_MM, STAGE_PEG_R_MM))
    p_st = target / "staging_block.stl"
    _write_stl(p_st, "staging_block", st_tris)
    written.append(p_st)

    # 4. Aluminum subplate base (160 x 200 x 12 mm)
    pl_tris = _box_tris(0.0, 0.0, PLATE_SIZE_MM[2] / 2.0, *PLATE_SIZE_MM)
    p_pl = target / "subplate.stl"
    _write_stl(p_pl, "subplate", pl_tris)
    written.append(p_pl)

    return written


if __name__ == "__main__":
    paths = build_all()
    for p in paths:
        print(f"Wrote {p} ({p.stat().st_size} bytes)")
