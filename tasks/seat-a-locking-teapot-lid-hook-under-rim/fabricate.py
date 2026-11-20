#!/usr/bin/env python3
"""Optional STL generator for the modeled locking teapot and lid geometry (mm, ASCII STL).

Real-world trials use an off-the-shelf locking-lid ceramic teapot such as the Seltmann Weiden Trio White 1.25 L
Porcelain Teapot (Article No. 001.686297, EAN 4003106642743; lid EAN 4052212082301) or the Price & Kensington 6-Cup
Gloss White Stoneware Teapot (SKU 0056.719U). Run this script only if you want a 3D-printable replica matching the
exact simulated 70 mm mouth bore and 12 mm underside locking nose for non-fragile rehearsal or calibration.

Usage:
    python3 fabricate.py [output_dir]
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np


def _triangulate_shape(shape) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Extract world-frame triangles (metres) from a mechkit Shape's visual mesh or primitive geometry."""
    import mechkit

    rot = mechkit._quat_wxyz_matrix(shape.tilt)
    at = np.asarray(shape.at, dtype=float)
    tris: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []

    if shape.kind == "mesh":
        pts = np.asarray(shape.visual[0], dtype=float) @ rot.T + at
        counts, idx = shape.visual[1], shape.visual[2]
        k = 0
        for c in counts:
            face = idx[k : k + c]
            k += c
            for i in range(1, c - 1):
                tris.append((pts[face[0]], pts[face[i]], pts[face[i + 1]]))
        return tris

    if shape.kind == "box":
        hx, hy, hz = (s / 2.0 for s in shape.size)
        v = np.array([
            [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
            [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
        ]) @ rot.T + at
        faces = [
            (0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
            (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7),
        ]
        for a, b, c, d in faces:
            tris.append((v[a], v[b], v[c]))
            tris.append((v[a], v[c], v[d]))
        return tris

    if shape.kind == "cyl":
        r, h = shape.size[0], shape.size[2]
        n = 64
        ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        bot = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(n, -h / 2.0)])
        top = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(n, h / 2.0)])
        cb = np.array([[0.0, 0.0, -h / 2.0]])
        ct = np.array([[0.0, 0.0, h / 2.0]])
        pts = mechkit._canon(np.vstack([bot, top, cb, ct]), shape.axis) @ rot.T + at
        ib, it = 2 * n, 2 * n + 1
        for i in range(n):
            j = (i + 1) % n
            tris.append((pts[i], pts[j], pts[n + j]))
            tris.append((pts[i], pts[n + j], pts[n + i]))
            tris.append((pts[ib], pts[j], pts[i]))
            tris.append((pts[it], pts[n + i], pts[n + j]))
        return tris

    raise ValueError(f"unsupported shape kind: {shape.kind!r}")


def _write_ascii_stl(path: Path, solid_name: str, shapes) -> int:
    tris: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for s in shapes:
        if not s.collide:
            continue
        tris.extend(_triangulate_shape(s))
    lines = [f"solid {solid_name}"]
    for a, b, c in tris:
        a_mm, b_mm, c_mm = a * 1000.0, b * 1000.0, c * 1000.0
        n = np.cross(b_mm - a_mm, c_mm - a_mm)
        ln = float(np.linalg.norm(n))
        n = n / ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])
        lines.append(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}")
        lines.append("    outer loop")
        for p in (a_mm, b_mm, c_mm):
            lines.append(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {solid_name}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(tris)


def main() -> int:
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    import seat_a_locking_teapot_lid_hook_under_rim as task

    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else here / "stl_out"
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, shapes in (("teapot_body", task.POT_SHAPES), ("teapot_locking_lid", task.LID_SHAPES)):
        stl_path = out_dir / f"{name}.stl"
        n = _write_ascii_stl(stl_path, name, shapes)
        print(f"wrote {stl_path} ({n} triangles, units: mm)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
