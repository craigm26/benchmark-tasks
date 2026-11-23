"""Generate ASCII STL files (in millimetres) for the wooden tabletop fixtures in this task.

The hanger (Nahanco 60017BEC), captive chrome security ring (Nahanco HHSRA100), and 32 mm OD chrome closet rod
(standard 1-1/4 in closet pole) are off-the-shelf commercial products. This script writes optional 3D-printable STL
models of the valet tray lean cradle and closet rod mounting stiles from the exact dimensions in the task file so a
real-world rig can reproduce the 12 deg starting hanger lean and 250 mm rod height without woodworking tools.

Runs only when executed as a script; importing this module writes nothing.

Usage:
    python3 fabricate.py --out ./stl
"""

from __future__ import annotations

import argparse
from pathlib import Path

# Dimensions in mm, matching hotel_ball_top_hanger_into_captive_closet_ring.py
TRAY_DEPTH = 56.5  # x: 197.5..254.0 mm
TRAY_WIDTH = 440.0  # y
TRAY_BASE_H = 8.0
LIP_THICK = 12.0
LIP_HEIGHT = 16.0
LIP_X_OFFSET = 209.7 - 12.0 / 2.0 - (197.5 + 254.0) / 2.0  # front lip center in tray x
PILLAR_DEPTH = 18.0
PILLAR_WIDTH = 24.0
PILLAR_HEIGHT = 54.0
PILLAR_X_OFFSET = 234.2 + 18.0 / 2.0 - (197.5 + 254.0) / 2.0
PILLAR_Y_OFFSET = 145.0

STILE_DEPTH = 113.0  # x: 354..467 mm
STILE_THICK = 18.0  # y
STILE_HEIGHT = 300.0  # z
ROD_SOCKET_Z = 250.0  # center of 32 mm rod above table
ROD_SOCKET_X = 370.0 - 354.0  # 16 mm back from stile front edge


def _box_tris(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> list[tuple]:
    """12 outward-facing triangles of an axis-aligned box (all units mm)."""
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = cz - sz / 2.0, cz + sz / 2.0
    v = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    faces = [
        ((0, 0, -1), (0, 3, 2), (0, 2, 1)),
        ((0, 0, +1), (4, 5, 6), (4, 6, 7)),
        ((-1, 0, 0), (0, 4, 7), (0, 7, 3)),
        ((+1, 0, 0), (1, 2, 6), (1, 6, 5)),
        ((0, -1, 0), (0, 1, 5), (0, 5, 4)),
        ((0, +1, 0), (3, 7, 6), (3, 6, 2)),
    ]
    tris = []
    for n, a, b in faces:
        tris.append((n, v[a[0]], v[a[1]], v[a[2]]))
        tris.append((n, v[b[0]], v[b[1]], v[b[2]]))
    return tris


def _write_ascii_stl(path: Path, name: str, tris: list[tuple]) -> None:
    lines = [f"solid {name}"]
    for n, p0, p1, p2 in tris:
        lines.append(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
        lines.append("    outer loop")
        for p in (p0, p1, p2):
            lines.append(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}\n")
    path.write_text("\n".join(lines), encoding="utf-8")


def build_valet_tray() -> list[tuple]:
    """One-piece valet tray with front lip and two rear lean-rest pillars (mm)."""
    tris = []
    tris += _box_tris(0.0, 0.0, TRAY_BASE_H / 2.0, TRAY_DEPTH, TRAY_WIDTH, TRAY_BASE_H)
    tris += _box_tris(LIP_X_OFFSET, 0.0, TRAY_BASE_H + LIP_HEIGHT / 2.0, LIP_THICK, TRAY_WIDTH, LIP_HEIGHT)
    for sign in (-1.0, +1.0):
        tris += _box_tris(
            PILLAR_X_OFFSET,
            sign * PILLAR_Y_OFFSET,
            TRAY_BASE_H + PILLAR_HEIGHT / 2.0,
            PILLAR_DEPTH,
            PILLAR_WIDTH,
            PILLAR_HEIGHT,
        )
    return tris


def build_stile_panel() -> list[tuple]:
    """Side stile panel (print two, or cut from 18 mm walnut plywood; drill a 32.5 mm blind rod socket at x=16, z=250)."""
    return _box_tris(STILE_DEPTH / 2.0, 0.0, STILE_HEIGHT / 2.0, STILE_DEPTH, STILE_THICK, STILE_HEIGHT)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("stl"), help="Output directory for ASCII STL files")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    parts = {
        "valet_tray.stl": ("valet_tray", build_valet_tray()),
        "closet_stile.stl": ("closet_stile", build_stile_panel()),
    }
    for fname, (name, tris) in parts.items():
        dst = args.out / fname
        _write_ascii_stl(dst, name, tris)
        print(f"wrote {dst} ({len(tris)} triangles)")
    print(f"Note: drill a 32.5 mm rod socket in each stile at x={ROD_SOCKET_X:.1f} mm from front edge, z={ROD_SOCKET_Z:.1f} mm up.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
