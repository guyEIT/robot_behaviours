#!/usr/bin/env python3

import argparse
import json
import shutil
import struct
from collections import defaultdict, deque
from pathlib import Path

import numpy as np


STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("v0", "<f4", (3,)),
        ("v1", "<f4", (3,)),
        ("v2", "<f4", (3,)),
        ("attr", "<u2"),
    ]
)


def load_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("rb") as handle:
        header = handle.read(80)
        if len(header) != 80:
            raise ValueError(f"{path} is not a valid binary STL")
        count_bytes = handle.read(4)
        triangle_count = struct.unpack("<I", count_bytes)[0]
        payload = handle.seek(0, 2) - 84
        handle.seek(84)
        if payload % 50 != 0:
            raise ValueError(f"{path} has an invalid STL payload size")
        inferred_count = payload // 50
        if triangle_count == 0 or triangle_count != inferred_count:
            triangle_count = inferred_count
        data = np.fromfile(handle, dtype=STL_DTYPE, count=triangle_count)
    vertices = np.concatenate([data["v0"], data["v1"], data["v2"]], axis=0)
    faces = np.arange(len(vertices), dtype=np.int64).reshape((-1, 3))
    return vertices.astype(np.float64), faces


def dedupe_vertices(
    vertices: np.ndarray, faces: np.ndarray, decimals: int = 6
) -> tuple[np.ndarray, np.ndarray]:
    rounded = np.round(vertices, decimals=decimals)
    unique_vertices, inverse = np.unique(rounded, axis=0, return_inverse=True)
    return unique_vertices.astype(np.float64), inverse[faces]


def keep_large_components(
    vertices: np.ndarray, faces: np.ndarray, min_faces: int
) -> tuple[np.ndarray, np.ndarray, int]:
    vertex_faces: dict[int, list[int]] = defaultdict(list)
    for face_index, tri in enumerate(faces):
        for vertex_index in set(int(v) for v in tri):
            vertex_faces[vertex_index].append(face_index)

    seen = np.zeros(len(faces), dtype=bool)
    keep = np.zeros(len(faces), dtype=bool)
    removed_components = 0

    for start in range(len(faces)):
        if seen[start]:
            continue

        queue = deque([start])
        seen[start] = True
        component = []

        while queue:
            face_index = queue.popleft()
            component.append(face_index)
            for vertex_index in faces[face_index]:
                for neighbor in vertex_faces[int(vertex_index)]:
                    if not seen[neighbor]:
                        seen[neighbor] = True
                        queue.append(neighbor)

        if len(component) >= min_faces:
            keep[component] = True
        else:
            removed_components += 1

    kept_faces = faces[keep]
    used_vertices, inverse = np.unique(kept_faces.reshape(-1), return_inverse=True)
    return vertices[used_vertices], inverse.reshape((-1, 3)), removed_components


def voxel_cluster(
    vertices: np.ndarray, faces: np.ndarray, cell_size: float
) -> tuple[np.ndarray, np.ndarray]:
    keys = np.floor(vertices / cell_size).astype(np.int64)
    _, cluster_index = np.unique(keys, axis=0, return_inverse=True)

    cluster_count = int(cluster_index.max()) + 1
    sums = np.zeros((cluster_count, 3), dtype=np.float64)
    counts = np.bincount(cluster_index, minlength=cluster_count).astype(np.float64)
    np.add.at(sums, cluster_index, vertices)
    clustered_vertices = sums / counts[:, None]

    clustered_faces = cluster_index[faces]
    non_degenerate = (
        (clustered_faces[:, 0] != clustered_faces[:, 1])
        & (clustered_faces[:, 1] != clustered_faces[:, 2])
        & (clustered_faces[:, 0] != clustered_faces[:, 2])
    )
    clustered_faces = clustered_faces[non_degenerate]

    face_records = {}
    for tri in clustered_faces:
        key = tuple(sorted(int(v) for v in tri))
        if key not in face_records:
            face_records[key] = tuple(int(v) for v in tri)

    simplified_faces = np.array(list(face_records.values()), dtype=np.int64)
    used_vertices, inverse = np.unique(simplified_faces.reshape(-1), return_inverse=True)
    return clustered_vertices[used_vertices], inverse.reshape((-1, 3))


def compute_face_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    non_zero = lengths > 0
    normals[non_zero] /= lengths[non_zero][:, None]
    return normals.astype(np.float32)


def compute_vertex_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    face_normals = compute_face_normals(vertices, faces).astype(np.float64)
    vertex_normals = np.zeros_like(vertices, dtype=np.float64)
    for corner in range(3):
        np.add.at(vertex_normals, faces[:, corner], face_normals)
    lengths = np.linalg.norm(vertex_normals, axis=1)
    non_zero = lengths > 0
    vertex_normals[non_zero] /= lengths[non_zero][:, None]
    return vertex_normals.astype(np.float32)


def write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    face_normals = compute_face_normals(vertices, faces)
    triangles = vertices[faces].astype(np.float32)
    with path.open("wb") as handle:
        header = b"Optimized mecatable STL"
        handle.write(header.ljust(80, b" "))
        handle.write(struct.pack("<I", len(faces)))
        records = np.zeros(len(faces), dtype=STL_DTYPE)
        records["normal"] = face_normals
        records["v0"] = triangles[:, 0]
        records["v1"] = triangles[:, 1]
        records["v2"] = triangles[:, 2]
        records.tofile(handle)


def build_gltf_payload(vertices: np.ndarray, faces: np.ndarray) -> tuple[dict, bytes]:
    positions = vertices.astype(np.float32)
    normals = compute_vertex_normals(vertices, faces)

    if len(vertices) <= 65535:
        indices = faces.astype(np.uint16).reshape(-1)
        index_component_type = 5123
    else:
        indices = faces.astype(np.uint32).reshape(-1)
        index_component_type = 5125

    index_bytes = indices.tobytes()
    position_bytes = positions.tobytes()
    normal_bytes = normals.tobytes()

    def pad_chunk(data: bytes) -> bytes:
        return data + (b"\x00" * ((4 - (len(data) % 4)) % 4))

    index_bytes = pad_chunk(index_bytes)
    position_offset = len(index_bytes)
    position_bytes = pad_chunk(position_bytes)
    normal_offset = position_offset + len(position_bytes)
    normal_bytes = pad_chunk(normal_bytes)
    binary_blob = index_bytes + position_bytes + normal_bytes

    position_min = [float(v) for v in positions.min(axis=0)]
    position_max = [float(v) for v in positions.max(axis=0)]

    gltf = {
        "asset": {"version": "2.0", "generator": "optimize_mecatable.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": "mecatable"}],
        "meshes": [
            {
                "name": "mecatable",
                "primitives": [
                    {
                        "attributes": {"POSITION": 1, "NORMAL": 2},
                        "indices": 0,
                        "mode": 4,
                    }
                ],
            }
        ],
        "buffers": [{"byteLength": len(binary_blob), "uri": None}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(index_bytes), "target": 34963},
            {
                "buffer": 0,
                "byteOffset": position_offset,
                "byteLength": len(position_bytes),
                "target": 34962,
            },
            {
                "buffer": 0,
                "byteOffset": normal_offset,
                "byteLength": len(normal_bytes),
                "target": 34962,
            },
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": index_component_type,
                "count": int(indices.size),
                "type": "SCALAR",
                "min": [int(indices.min())],
                "max": [int(indices.max())],
            },
            {
                "bufferView": 1,
                "componentType": 5126,
                "count": int(len(positions)),
                "type": "VEC3",
                "min": position_min,
                "max": position_max,
            },
            {
                "bufferView": 2,
                "componentType": 5126,
                "count": int(len(normals)),
                "type": "VEC3",
            },
        ],
    }
    return gltf, binary_blob


def write_gltf(path: Path, bin_path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    gltf, binary_blob = build_gltf_payload(vertices, faces)
    gltf["buffers"][0]["uri"] = bin_path.name
    path.write_text(json.dumps(gltf, separators=(",", ":")), encoding="utf-8")
    bin_path.write_bytes(binary_blob)


def write_glb(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    gltf, binary_blob = build_gltf_payload(vertices, faces)
    gltf["buffers"][0].pop("uri", None)

    json_chunk = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_chunk += b" " * ((4 - (len(json_chunk) % 4)) % 4)
    binary_blob += b"\x00" * ((4 - (len(binary_blob) % 4)) % 4)

    total_length = 12 + 8 + len(json_chunk) + 8 + len(binary_blob)
    with path.open("wb") as handle:
        handle.write(struct.pack("<4sII", b"glTF", 2, total_length))
        handle.write(struct.pack("<I4s", len(json_chunk), b"JSON"))
        handle.write(json_chunk)
        handle.write(struct.pack("<I4s", len(binary_blob), b"BIN\x00"))
        handle.write(binary_blob)


def main() -> None:
    parser = argparse.ArgumentParser(description="Optimize the mecatable mesh.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--output-gltf", type=Path, required=True)
    parser.add_argument("--output-bin", type=Path, required=True)
    parser.add_argument("--output-glb", type=Path, required=True)
    parser.add_argument("--cell-size", type=float, default=1.0)
    parser.add_argument("--min-component-faces", type=int, default=2048)
    parser.add_argument("--backup-gltf", type=Path)
    parser.add_argument("--replace-gltf", type=Path)
    args = parser.parse_args()

    vertices, faces = load_binary_stl(args.input_stl)
    vertices, faces = dedupe_vertices(vertices, faces)
    vertices, faces, removed_components = keep_large_components(
        vertices, faces, args.min_component_faces
    )
    original_face_count = len(faces)
    vertices, faces = voxel_cluster(vertices, faces, args.cell_size)

    write_binary_stl(args.output_stl, vertices, faces)
    write_gltf(args.output_gltf, args.output_bin, vertices, faces)
    write_glb(args.output_glb, vertices, faces)

    if args.replace_gltf:
        if args.backup_gltf:
            shutil.copy2(args.replace_gltf, args.backup_gltf)
        shutil.copy2(args.output_gltf, args.replace_gltf)

    print(
        json.dumps(
            {
                "removed_components": removed_components,
                "cell_size": args.cell_size,
                "faces_before_simplify": original_face_count,
                "faces_after_simplify": int(len(faces)),
                "vertices_after_simplify": int(len(vertices)),
                "output_stl": str(args.output_stl),
                "output_gltf": str(args.output_gltf),
                "output_glb": str(args.output_glb),
            }
        )
    )


if __name__ == "__main__":
    main()
