#!/usr/bin/env python3
"""
Repair an STL mesh: weld duplicate vertices, drop degenerate triangles,
reorient faces consistently, optionally fill open holes, recompute normals.

Built originally to fix `mecatable_simple.stl`, which had 304 triangles stored
with 912 unwelded vertices — adjacent faces never shared verts, so Three.js's
STLLoader rendered each triangle as a shaded island and flat surfaces looked
fragmented. Welding collapses to 152 distinct verts and the mesh becomes
watertight with consistent winding.

Uses trimesh's processing pipeline for robust repair.

Usage:
    python3 fix_mecatable.py --input INPUT.stl --output OUTPUT.stl [--fill-holes]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def _fix_winding_bfs(mesh: trimesh.Trimesh) -> None:
    """Fallback for when graph backends aren't available — propagate consistent
    winding by flooding face adjacency, then flip globally if the resulting
    signed volume is negative."""
    from collections import defaultdict, deque

    faces = mesh.faces.copy()
    n = len(faces)

    edge_to_face: dict[tuple[int, int], list[int]] = defaultdict(list)
    for fi, tri in enumerate(faces):
        for a, b in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]:
            edge_to_face[(min(a, b), max(a, b))].append(fi)

    visited = np.zeros(n, dtype=bool)
    flipped = np.zeros(n, dtype=bool)
    for start in range(n):
        if visited[start]:
            continue
        visited[start] = True
        q = deque([start])
        while q:
            fi = q.popleft()
            tri = faces[fi][::-1] if flipped[fi] else faces[fi]
            for a, b in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]:
                key = (min(a, b), max(a, b))
                neigh = edge_to_face[key]
                if len(neigh) != 2:
                    continue
                other = neigh[0] if neigh[1] == fi else neigh[1]
                if visited[other]:
                    continue
                visited[other] = True
                otri = faces[other][::-1] if flipped[other] else faces[other]
                other_dirs = [
                    (otri[0], otri[1]),
                    (otri[1], otri[2]),
                    (otri[2], otri[0]),
                ]
                # Consistent if the shared edge runs opposite directions
                if (a, b) in other_dirs:
                    flipped[other] = True
                q.append(other)

    faces[flipped] = faces[flipped][:, ::-1]
    mesh.faces = faces

    # Final outward check
    if mesh.is_volume and mesh.volume < 0:
        mesh.invert()


def report(label: str, mesh: trimesh.Trimesh) -> None:
    edges = mesh.edges_sorted
    if len(edges) == 0:
        print(f"  {label}: empty mesh")
        return
    # Boundary edges = edges referenced by exactly one face.
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = int((counts == 1).sum())
    nonman = int((counts > 2).sum())
    zero_area = int((mesh.area_faces < 1e-12).sum())
    watertight = mesh.is_watertight
    winding = mesh.is_winding_consistent
    print(
        f"  {label}: {len(mesh.faces)} tris, {len(mesh.vertices)} verts, "
        f"{zero_area} zero-area, {boundary} boundary, {nonman} non-manifold, "
        f"watertight={watertight}, winding_consistent={winding}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--fill-holes",
        action="store_true",
        help="Attempt to fill open holes (only safe for closed-shell meshes)",
    )
    parser.add_argument(
        "--merge-tol",
        type=float,
        default=1e-4,
        help="Vertex merge tolerance in mesh units (default 1e-4)",
    )
    args = parser.parse_args()

    print(f"Reading {args.input}")
    mesh = trimesh.load(args.input, force="mesh", process=False)
    report("loaded", mesh)

    # Step 1: merge duplicate vertices within tolerance.
    mesh.merge_vertices(merge_tex=False, merge_norm=False, digits_vertex=int(round(-np.log10(args.merge_tol))))
    report("after merge_vertices", mesh)

    # Step 2: drop degenerate / duplicate faces.
    mesh.update_faces(mesh.unique_faces())
    mesh.update_faces(mesh.nondegenerate_faces(height=1e-9))
    mesh.remove_unreferenced_vertices()
    report("after dedupe + drop degenerate", mesh)

    # Step 3: fix winding so adjacent faces agree on normal direction.
    # trimesh's `fix_normals` needs a graph backend (networkx/scipy.csgraph) which
    # isn't in this env. Fall back to: if winding is already consistent (which
    # `merge_vertices` typically achieves), just flip globally to make the
    # volume sign positive (outward-facing). For non-consistent winding we
    # do a manual BFS over face adjacency.
    if mesh.is_winding_consistent:
        # Easy case: just check the signed volume sign and flip if needed.
        if mesh.is_volume and mesh.volume < 0:
            mesh.invert()
            print("  flipped winding globally (signed volume was negative)")
    else:
        _fix_winding_bfs(mesh)
    report("after fix_normals", mesh)

    # Step 4: optionally fill small holes. Only safe for shells that should be
    # closed; for the mecatable, the model has open underside / leg interiors,
    # so leave this off by default.
    if args.fill_holes:
        before = len(mesh.faces)
        mesh.fill_holes()
        print(f"  fill_holes added {len(mesh.faces) - before} faces")
        report("after fill_holes", mesh)

    # Step 5: recompute face normals from the now-consistent winding so the
    # binary STL we write has stored normals matching the geometry.
    mesh.face_normals  # force re-evaluation
    args.output.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output, file_type="stl")
    print(f"Wrote {args.output} ({args.output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
