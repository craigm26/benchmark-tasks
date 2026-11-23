"""Printable real-world workshop fixtures and mock-ups for extend-twistguard-probe-shrouds-to-cat-iii.

Generates ASCII STL files (in mm) matching the exact dimensions of extend_twistguard_probe_shrouds_to_cat_iii.py
and the commercial Fluke TL175 TwistGuard probe geometry (130 mm body, 14.4 mm barrel, 15 mm shroud travel from
19 mm CAT II to 4 mm CAT III tip exposure):

  - staging_tray.stl         136 x 220 x 12 mm technician staging tray (4 mm floor, 8 mm walls)
  - probe_cradle_chocks.stl  4 mm V-cradle reset jig + 1.2 mm finger-guard rest pad for repeatable probe start poses
  - panel_mockup.stl         90 x 180 x 140 mm inert 480 V panel housing with 15 mm busbar slots and 12 mm lamp bore
  - shroud_gauge_card.stl    4 mm CAT III pass / 19 mm CAT II fail tip-exposure verification gauge card

Usage (writes files only when run as a script, never on import):
    python3 fabricate.py [out_dir]
"""

import math
import os
import sys

# Exact scene dimensions in mm (from extend_twistguard_probe_shrouds_to_cat_iii.py)
TRAY_X = 136.0
TRAY_Y = 220.0
TRAY_FLOOR_H = 4.0
TRAY_WALL_H = 12.0
TRAY_WALL_T = 8.0

PANEL_D = 90.0
PANEL_W = 180.0
PANEL_H = 140.0
BUSBAR_PITCH = 15.0

CHOCK_H = 4.0


def _box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> list:
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [
        (0, 1, 3, 2),
        (4, 6, 7, 5),
        (0, 4, 5, 1),
        (2, 3, 7, 6),
        (0, 2, 6, 4),
        (1, 5, 7, 3),
    ]
    tris = []
    for a, b, c, d in quads:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def staging_tray() -> list:
    hx, hy = TRAY_X / 2.0, TRAY_Y / 2.0
    wt, wh, fh = TRAY_WALL_T, TRAY_WALL_H, TRAY_FLOOR_H
    tris = _box(-hx, -hy, 0.0, hx, hy, fh)
    tris += _box(-hx, -hy, fh, -hx + wt, hy, wh)
    tris += _box(hx - wt, -hy, fh, hx, hy, wh)
    tris += _box(-hx + wt, -hy, fh, hx - wt, -hy + wt, wh)
    tris += _box(-hx + wt, hy - wt, fh, hx - wt, hy, wh)
    return tris


def _annular_ring(
    cx: float,
    cy: float,
    z0: float,
    z1: float,
    r_in: float,
    r_out_fn,
    sides: int = 24,
) -> list:
    bi, ti, bo, to = [], [], [], []
    for i in range(sides):
        ang = 2.0 * math.pi * i / sides
        ca, sa = math.cos(ang), math.sin(ang)
        ro = r_out_fn(ca, sa)
        bi.append((cx + r_in * ca, cy + r_in * sa, z0))
        ti.append((cx + r_in * ca, cy + r_in * sa, z1))
        bo.append((cx + ro * ca, cy + ro * sa, z0))
        to.append((cx + ro * ca, cy + ro * sa, z1))
    tris = []
    for i in range(sides):
        j = (i + 1) % sides
        tris += [(bo[i], bo[j], to[j]), (bo[i], to[j], to[i])]
        tris += [(bi[i], ti[i], ti[j]), (bi[i], ti[j], bi[j])]
        tris += [(ti[i], to[i], to[j]), (ti[i], to[j], ti[j])]
        tris += [(bi[i], bi[j], bo[j]), (bi[i], bo[j], bo[i])]
    return tris


def probe_cradle_chocks() -> list:
    # Mat-mounted rear barrel cradle ribs (-42 mm, 4.0 mm tall) and finger-guard rest pad (+19 mm, 1.2 mm tall)
    tris = []
    for sy in (-8.5, 8.5):
        tris += _box(-46.0, sy - 2.0, 0.0, -38.0, sy + 2.0, CHOCK_H)
    tris += _box(14.0, -17.0, 0.0, 24.0, 17.0, 1.2)
    return tris


def panel_mockup() -> list:
    hd, hw = PANEL_D / 2.0, PANEL_W / 2.0
    lx, ly, lz_bore = -20.0, -60.0, 125.0
    half_sq = 11.0
    # Lower solid body (0..125 mm) and upper body (125..140 mm) partitioned around the 12 mm lamp bore at (-20, -60, 140)
    tris = _box(-hd, -hw, 0.0, hd, hw, lz_bore)
    tris += _box(-hd, -hw, lz_bore, lx - half_sq, hw, PANEL_H)
    tris += _box(lx + half_sq, -hw, lz_bore, hd, hw, PANEL_H)
    tris += _box(lx - half_sq, -hw, lz_bore, lx + half_sq, ly - half_sq, PANEL_H)
    tris += _box(lx - half_sq, ly + half_sq, lz_bore, lx + half_sq, hw, PANEL_H)
    tris += _annular_ring(lx, ly, lz_bore, PANEL_H, 6.0, lambda ca, sa: half_sq / max(abs(ca), abs(sa)))
    # Top warning lamp collar boss (22 mm OD, 12 mm ID bore at (-20, -60, 140..152) mm)
    tris += _annular_ring(lx, ly, PANEL_H, PANEL_H + 12.0, 6.0, lambda ca, sa: 11.0)
    # Yellow lightning warning placard relief on -X/+Y door (+34..+76 mm, z 61..103 mm)
    tris += _box(-hd - 2.5, 34.0, 61.0, -hd, 76.0, 103.0)
    # Three 15 mm-spaced copper busbars (y = -15, 0, +15 mm; 8 x 6 x 70 mm at z = 33..103 mm)
    for k in (-1, 0, 1):
        yc = k * BUSBAR_PITCH
        tris += _box(-hd - 8.0, yc - 3.0, 33.0, -hd, yc + 3.0, 103.0)
    return tris


def shroud_gauge_card() -> list:
    # 60 x 35 x 2 mm inspection card with raised 4 mm (CAT III) and 19 mm (CAT II) tip-exposure reference ledges
    tris = _box(0.0, 0.0, 0.0, 60.0, 35.0, 2.0)
    tris += _box(8.0, 4.0, 2.0, 12.0, 16.0, 4.0)     # 4.0 mm CAT III pass gauge slot
    tris += _box(24.0, 4.0, 2.0, 43.0, 16.0, 4.0)    # 19.0 mm CAT II bare-tip reference bar
    return tris


def write_stl(path: str, name: str, tris: list) -> None:
    lines = [f"solid {name}"]
    for a, b, c in tris:
        ux, uy, uz = (b[i] - a[i] for i in range(3))
        vx, vy, vz = (c[i] - a[i] for i in range(3))
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        mag = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        lines.append(f" facet normal {nx / mag:.6f} {ny / mag:.6f} {nz / mag:.6f}")
        lines.append("  outer loop")
        for p in (a, b, c):
            lines.append(f"   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
        lines.append("  endloop")
        lines.append(" endfacet")
    lines.append(f"endsolid {name}")
    with open(path, "w", encoding="ascii") as f:
        f.write("\n".join(lines) + "\n")


PARTS = {
    "staging_tray": staging_tray,
    "probe_cradle_chocks": probe_cradle_chocks,
    "panel_mockup": panel_mockup,
    "shroud_gauge_card": shroud_gauge_card,
}


if __name__ == "__main__":
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)
    for part_name, builder in PARTS.items():
        write_stl(os.path.join(out_dir, f"{part_name}.stl"), part_name, builder())
        print(f"wrote {part_name}.stl")
