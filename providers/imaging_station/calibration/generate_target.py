#!/usr/bin/env python3
"""Generate an A4 calibration target SVG (and PDF) for the imaging station.

Layout (portrait A4, 210 x 297 mm):
  - Page centre at (105, 148.5) mm.
  - Centre target = 10 x 10 mm square with cross-hair at the page centre.
    This is the spot the macro camera should be auto-aimed at.
  - Outer fiducial = ArUco DICT_4X4_50 ID=0, 50 x 50 mm, centred 80 mm
    above the page centre, so the offset from fiducial-centre to target-
    centre is exactly (dx=0, dy=+80) mm in print-frame coordinates
    (positive y = downwards on the page). The auto-calibration routine
    detects the ArUco, applies this known offset, and lands on the target.
  - Faint mm tick rulers along all four edges so a print-scale check is
    obvious by eye.

Run:
    python3 generate_target.py
Outputs:
    calibration_target_a4.svg
    calibration_target_a4.pdf  (only if pycairo + Rsvg are available)
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import cv2
import cv2.aruco as aruco

HERE = Path(__file__).resolve().parent

# ----- design parameters (all in mm) ---------------------------------------
PAGE_W = 210.0
PAGE_H = 297.0
CX = PAGE_W / 2
CY = PAGE_H / 2

# ArUco fiducial
ARUCO_DICT = aruco.DICT_4X4_50
ARUCO_ID = 0
ARUCO_SIZE_MM = 50.0
# Offset from page/target centre to fiducial centre.
# dy is negative = fiducial sits *above* the target on the page.
FIDUCIAL_DX_MM = 0.0
FIDUCIAL_DY_MM = -80.0

# Centre target
TARGET_BOX_MM = 10.0     # 1 cm square
CROSS_LEN_MM = 12.0      # cross slightly larger than the box
CROSS_STROKE = 0.25      # mm
BOX_STROKE = 0.35        # mm

# Tick rulers
TICK_MAJOR_MM = 10
TICK_MINOR_MM = 1
TICK_MAJOR_LEN = 4.0
TICK_MINOR_LEN = 1.5
TICK_STROKE = 0.15
PAGE_MARGIN = 6.0  # ruler sits this far in from the paper edge


def aruco_bits(dict_id: int, marker_id: int) -> list[list[int]]:
    """Return the 6x6 cell pattern (inc. 1-cell black border) for the marker.

    Uses cv2.aruco.generateImageMarker at sidePixels=6 with borderBits=1, so
    each pixel of the returned image is exactly one cell.
    """
    d = aruco.getPredefinedDictionary(dict_id)
    img = aruco.generateImageMarker(d, marker_id, 6, borderBits=1)
    # img is uint8, 0=black, 255=white. 1 = black cell, 0 = white cell.
    return [[1 if img[r, c] == 0 else 0 for c in range(6)] for r in range(6)]


def fmt(x: float) -> str:
    return f"{x:.4f}".rstrip("0").rstrip(".")


def build_svg() -> str:
    bits = aruco_bits(ARUCO_DICT, ARUCO_ID)
    n = len(bits)              # 6
    cell = ARUCO_SIZE_MM / n    # mm per cell

    fid_cx = CX + FIDUCIAL_DX_MM
    fid_cy = CY + FIDUCIAL_DY_MM
    fid_x0 = fid_cx - ARUCO_SIZE_MM / 2
    fid_y0 = fid_cy - ARUCO_SIZE_MM / 2

    parts: list[str] = []
    parts.append(
        f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{PAGE_W}mm" height="{PAGE_H}mm" '
        f'viewBox="0 0 {PAGE_W} {PAGE_H}">\n'
    )
    parts.append(
        '<style>\n'
        '  .lbl { font-family: "DejaVu Sans", "Helvetica", sans-serif; '
        'fill: #000; }\n'
        '  .small { font-size: 2.6px; }\n'
        '  .mid   { font-size: 3.4px; }\n'
        '  .big   { font-size: 4.4px; font-weight: 600; }\n'
        '  .faint { stroke: #888; }\n'
        '</style>\n'
    )

    # ----- crop marks at the four corners (helps trim / verify orientation)
    crop = 4.0
    parts.append('<g stroke="#000" stroke-width="0.2" fill="none">\n')
    for (x, y) in [(0, 0), (PAGE_W, 0), (0, PAGE_H), (PAGE_W, PAGE_H)]:
        sx = -1 if x > 0 else 1
        sy = -1 if y > 0 else 1
        parts.append(
            f'  <path d="M{x},{y + sy * 0} L{x + sx * crop},{y}"/>\n'
            f'  <path d="M{x},{y} L{x},{y + sy * crop}"/>\n'
        )
    parts.append('</g>\n')

    # ----- mm tick rulers along all four sides
    parts.append(
        f'<g class="faint" stroke="#888" stroke-width="{TICK_STROKE}">\n'
    )
    # top + bottom rulers (ticks along x)
    for x_mm in range(0, int(PAGE_W) + 1):
        if x_mm == 0 or x_mm == int(PAGE_W):
            continue
        is_major = (x_mm % TICK_MAJOR_MM) == 0
        L = TICK_MAJOR_LEN if is_major else TICK_MINOR_LEN
        # top
        parts.append(
            f'  <line x1="{x_mm}" y1="{PAGE_MARGIN}" '
            f'x2="{x_mm}" y2="{PAGE_MARGIN + L}"/>\n'
        )
        # bottom
        parts.append(
            f'  <line x1="{x_mm}" y1="{PAGE_H - PAGE_MARGIN}" '
            f'x2="{x_mm}" y2="{PAGE_H - PAGE_MARGIN - L}"/>\n'
        )
    # left + right rulers (ticks along y)
    for y_mm in range(0, int(PAGE_H) + 1):
        if y_mm == 0 or y_mm == int(PAGE_H):
            continue
        is_major = (y_mm % TICK_MAJOR_MM) == 0
        L = TICK_MAJOR_LEN if is_major else TICK_MINOR_LEN
        # left
        parts.append(
            f'  <line x1="{PAGE_MARGIN}" y1="{y_mm}" '
            f'x2="{PAGE_MARGIN + L}" y2="{y_mm}"/>\n'
        )
        # right
        parts.append(
            f'  <line x1="{PAGE_W - PAGE_MARGIN}" y1="{y_mm}" '
            f'x2="{PAGE_W - PAGE_MARGIN - L}" y2="{y_mm}"/>\n'
        )
    parts.append('</g>\n')

    # major-tick numeric labels every 50 mm along the top ruler (sanity check)
    parts.append('<g class="lbl small" fill="#444">\n')
    for x_mm in range(50, int(PAGE_W), 50):
        parts.append(
            f'  <text x="{x_mm}" y="{PAGE_MARGIN - 1}" '
            f'text-anchor="middle">{x_mm}</text>\n'
        )
    for y_mm in range(50, int(PAGE_H), 50):
        parts.append(
            f'  <text x="{PAGE_MARGIN - 1}" y="{y_mm + 1}" '
            f'text-anchor="end">{y_mm}</text>\n'
        )
    parts.append('</g>\n')

    # ----- title block (top-left, well clear of fiducial)
    parts.append('<g class="lbl">\n')
    parts.append(
        f'  <text class="big" x="{PAGE_MARGIN + 6}" y="20">'
        f'Imaging-station calibration target</text>\n'
        f'  <text class="mid" x="{PAGE_MARGIN + 6}" y="26">'
        f'A4 portrait — print at 100% scale (no fit-to-page).</text>\n'
        f'  <text class="mid" x="{PAGE_MARGIN + 6}" y="31">'
        f'Verify: the 10&#8211;tick spacing along any ruler edge = 10.0 mm.'
        f'</text>\n'
    )
    parts.append('</g>\n')

    # ----- ArUco fiducial
    parts.append(
        f'<g transform="translate({fmt(fid_x0)},{fmt(fid_y0)})">\n'
    )
    # white background (covers all 6x6, drawn first)
    parts.append(
        f'  <rect x="0" y="0" width="{ARUCO_SIZE_MM}" '
        f'height="{ARUCO_SIZE_MM}" fill="#fff"/>\n'
    )
    parts.append('  <g fill="#000" shape-rendering="crispEdges">\n')
    for r in range(n):
        for c in range(n):
            if bits[r][c] == 1:
                parts.append(
                    f'    <rect x="{fmt(c * cell)}" y="{fmt(r * cell)}" '
                    f'width="{fmt(cell + 0.002)}" '
                    f'height="{fmt(cell + 0.002)}"/>\n'
                )
    parts.append('  </g>\n')
    parts.append('</g>\n')

    # fiducial label
    parts.append('<g class="lbl">\n')
    parts.append(
        f'  <text class="mid" x="{fmt(fid_cx)}" '
        f'y="{fmt(fid_y0 - 2.5)}" text-anchor="middle">'
        f'ArUco DICT_4X4_50 &#183; ID=0 &#183; '
        f'{fmt(ARUCO_SIZE_MM)} mm</text>\n'
    )
    parts.append('</g>\n')

    # ----- offset arrow from fiducial-centre to target-centre
    arrow_y0 = fid_y0 + ARUCO_SIZE_MM + 1.0
    arrow_y1 = CY - TARGET_BOX_MM / 2 - 1.0
    parts.append(
        '<g stroke="#c00" stroke-width="0.35" fill="#c00">\n'
        f'  <line x1="{fmt(CX)}" y1="{fmt(arrow_y0)}" '
        f'x2="{fmt(CX)}" y2="{fmt(arrow_y1)}"/>\n'
        # arrowhead
        f'  <polygon points="'
        f'{fmt(CX)},{fmt(arrow_y1 + 0.0)} '
        f'{fmt(CX - 1.2)},{fmt(arrow_y1 - 2.4)} '
        f'{fmt(CX + 1.2)},{fmt(arrow_y1 - 2.4)}"/>\n'
        '</g>\n'
    )
    # offset readout (slightly off-axis so it doesn't clash with the arrow)
    parts.append('<g class="lbl">\n')
    parts.append(
        f'  <text class="mid" x="{fmt(CX + 2)}" '
        f'y="{fmt((arrow_y0 + arrow_y1) / 2)}" '
        f'fill="#c00">offset = (dx=0.0, dy=+{fmt(-FIDUCIAL_DY_MM)}) mm '
        f'(fiducial &#8594; target)</text>\n'
    )
    parts.append('</g>\n')

    # ----- centre target: 10x10 mm box + cross
    parts.append(
        f'<g stroke="#000" fill="none" stroke-width="{BOX_STROKE}">\n'
        f'  <rect x="{fmt(CX - TARGET_BOX_MM / 2)}" '
        f'y="{fmt(CY - TARGET_BOX_MM / 2)}" '
        f'width="{TARGET_BOX_MM}" height="{TARGET_BOX_MM}"/>\n'
        f'</g>\n'
    )
    parts.append(
        f'<g stroke="#000" stroke-width="{CROSS_STROKE}">\n'
        f'  <line x1="{fmt(CX - CROSS_LEN_MM / 2)}" y1="{fmt(CY)}" '
        f'x2="{fmt(CX + CROSS_LEN_MM / 2)}" y2="{fmt(CY)}"/>\n'
        f'  <line x1="{fmt(CX)}" y1="{fmt(CY - CROSS_LEN_MM / 2)}" '
        f'x2="{fmt(CX)}" y2="{fmt(CY + CROSS_LEN_MM / 2)}"/>\n'
        f'</g>\n'
    )
    # tiny dot bang in the middle so the macro camera has a sub-mm feature
    parts.append(
        f'<circle cx="{fmt(CX)}" cy="{fmt(CY)}" r="0.25" fill="#000"/>\n'
    )
    # centre-target label
    parts.append('<g class="lbl">\n')
    parts.append(
        f'  <text class="mid" x="{fmt(CX)}" '
        f'y="{fmt(CY + TARGET_BOX_MM / 2 + 4)}" '
        f'text-anchor="middle">centre target &#183; '
        f'{fmt(TARGET_BOX_MM)}&#215;{fmt(TARGET_BOX_MM)} mm '
        f'(camera FOV centre)</text>\n'
    )
    parts.append(
        f'  <text class="small" x="{fmt(CX)}" '
        f'y="{fmt(CY + TARGET_BOX_MM / 2 + 7.5)}" '
        f'text-anchor="middle" fill="#444">'
        f'page centre = ({fmt(CX)}, {fmt(CY)}) mm</text>\n'
    )
    parts.append('</g>\n')

    # ----- footer with the full geometry written out (for the cal routine)
    parts.append('<g class="lbl">\n')
    parts.append(
        f'  <text class="small" x="{PAGE_MARGIN + 6}" '
        f'y="{fmt(PAGE_H - PAGE_MARGIN - 4)}" fill="#444">'
        f'fiducial: ArUco DICT_4X4_50 id=0, side={fmt(ARUCO_SIZE_MM)} mm, '
        f'centre=({fmt(fid_cx)}, {fmt(fid_cy)}) mm &#183; '
        f'target: {fmt(TARGET_BOX_MM)}&#215;{fmt(TARGET_BOX_MM)} mm at '
        f'({fmt(CX)}, {fmt(CY)}) mm &#183; '
        f'offset(target-fiducial) = ({fmt(-FIDUCIAL_DX_MM)}, '
        f'{fmt(-FIDUCIAL_DY_MM)}) mm'
        f'</text>\n'
    )
    parts.append('</g>\n')

    parts.append('</svg>\n')
    return "".join(parts)


def svg_to_pdf(svg_path: Path, pdf_path: Path) -> bool:
    """Render SVG to a 210x297 mm PDF using cairo+rsvg.

    Tries the system Python first (where PyGObject + Rsvg are installed
    on this box); silently returns False if no path works.
    """
    code = (
        "import sys, gi, cairo\n"
        "gi.require_version('Rsvg', '2.0')\n"
        "from gi.repository import Rsvg\n"
        "src, dst = sys.argv[1], sys.argv[2]\n"
        # 1 mm = 72/25.4 PostScript points
        "MM = 72.0/25.4\n"
        "W, H = 210.0*MM, 297.0*MM\n"
        "surf = cairo.PDFSurface(dst, W, H)\n"
        "ctx = cairo.Context(surf)\n"
        "ctx.scale(MM, MM)\n"
        "h = Rsvg.Handle.new_from_file(src)\n"
        "rect = Rsvg.Rectangle()\n"
        "rect.x = 0; rect.y = 0; rect.width = 210.0; rect.height = 297.0\n"
        "h.render_document(ctx, rect)\n"
        "surf.finish()\n"
    )
    for py in ("/usr/bin/python3", sys.executable):
        try:
            r = subprocess.run(
                [py, "-c", code, str(svg_path), str(pdf_path)],
                capture_output=True, text=True, timeout=30,
            )
            if r.returncode == 0 and pdf_path.exists():
                return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    return False


def main() -> int:
    svg_path = HERE / "calibration_target_a4.svg"
    pdf_path = HERE / "calibration_target_a4.pdf"

    svg = build_svg()
    svg_path.write_text(svg, encoding="utf-8")
    print(f"wrote {svg_path} ({len(svg):,} bytes)")

    if svg_to_pdf(svg_path, pdf_path):
        print(f"wrote {pdf_path} ({pdf_path.stat().st_size:,} bytes)")
    else:
        print("PDF conversion skipped — install librsvg2 + python3-gi to "
              "auto-render. SVG prints at 1:1 from any browser.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
