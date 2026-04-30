#!/usr/bin/env python3
"""Mirror an STL mesh across one of the principal axes.

Usage:
    python3 mirror_stl.py INPUT.stl OUTPUT.stl [--axis x|y|z]

Mirroring negates the chosen coordinate on every vertex and normal, then
swaps two vertices per triangle to keep the winding outward-facing (CCW).
Handles both ASCII and binary STL.
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def is_binary_stl(path: Path) -> bool:
    size = path.stat().st_size
    if size < 84:
        return False
    with path.open("rb") as f:
        f.seek(80)
        n_triangles = struct.unpack("<I", f.read(4))[0]
    return size == 84 + n_triangles * 50


def mirror_vec(v: tuple[float, float, float], axis: int) -> tuple[float, float, float]:
    out = list(v)
    out[axis] = -out[axis]
    return (out[0], out[1], out[2])


def mirror_binary(src: Path, dst: Path, axis: int) -> int:
    with src.open("rb") as f:
        header = f.read(80)
        (n_triangles,) = struct.unpack("<I", f.read(4))
        triangles_raw = f.read(n_triangles * 50)

    out = bytearray()
    out += header
    out += struct.pack("<I", n_triangles)

    for i in range(n_triangles):
        base = i * 50
        nx, ny, nz, \
            v1x, v1y, v1z, \
            v2x, v2y, v2z, \
            v3x, v3y, v3z = struct.unpack_from("<12f", triangles_raw, base)
        attr = struct.unpack_from("<H", triangles_raw, base + 48)[0]

        n = mirror_vec((nx, ny, nz), axis)
        v1 = mirror_vec((v1x, v1y, v1z), axis)
        v2 = mirror_vec((v2x, v2y, v2z), axis)
        v3 = mirror_vec((v3x, v3y, v3z), axis)
        # Swap v2 <-> v3 to restore CCW winding after the reflection.
        out += struct.pack("<12fH", *n, *v1, *v3, *v2, attr)

    dst.write_bytes(bytes(out))
    return n_triangles


def mirror_ascii(src: Path, dst: Path, axis: int) -> int:
    text = src.read_text()
    lines = text.splitlines()
    out_lines: list[str] = []
    triangle_count = 0
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith("facet normal"):
            parts = stripped.split()
            n = mirror_vec(tuple(float(x) for x in parts[2:5]), axis)
            out_lines.append(f"  facet normal {n[0]:e} {n[1]:e} {n[2]:e}")
            i += 1
            assert lines[i].strip() == "outer loop"
            out_lines.append("    outer loop")
            verts: list[tuple[float, float, float]] = []
            for _ in range(3):
                i += 1
                parts = lines[i].strip().split()
                verts.append(tuple(float(x) for x in parts[1:4]))
            v1 = mirror_vec(verts[0], axis)
            v2 = mirror_vec(verts[1], axis)
            v3 = mirror_vec(verts[2], axis)
            for v in (v1, v3, v2):
                out_lines.append(f"      vertex {v[0]:e} {v[1]:e} {v[2]:e}")
            i += 1
            assert lines[i].strip() == "endloop"
            out_lines.append("    endloop")
            i += 1
            assert lines[i].strip() == "endfacet"
            out_lines.append("  endfacet")
            triangle_count += 1
        else:
            out_lines.append(line)
        i += 1
    dst.write_text("\n".join(out_lines) + "\n")
    return triangle_count


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--axis", choices=list(AXIS_INDEX), default="x")
    args = parser.parse_args(argv)

    if not args.input.exists():
        print(f"input not found: {args.input}", file=sys.stderr)
        return 1

    axis = AXIS_INDEX[args.axis]
    if is_binary_stl(args.input):
        n = mirror_binary(args.input, args.output, axis)
        kind = "binary"
    else:
        n = mirror_ascii(args.input, args.output, axis)
        kind = "ascii"

    print(f"mirrored {n} triangles ({kind} STL) across {args.axis} axis -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
