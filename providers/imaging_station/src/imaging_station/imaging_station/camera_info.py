"""sensor_msgs/CameraInfo helpers for the NDI bridge node.

The ZowieBox NDI receiver publishes raw video on /imaging_station/image_raw;
ROS image consumers (image_proc, depth pipelines, RViz Camera display) need
a paired CameraInfo so the K matrix and distortion are available. This module
loads the standard camera_info_manager YAML format when a calibration is
available, and synthesises a placeholder otherwise so the topic is alive
from the first frame onward.

Calibration YAML format (matches `camera_calibration_parsers`):

    image_width: 1920
    image_height: 1080
    camera_name: imaging_station
    camera_matrix:
      rows: 3
      cols: 3
      data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    distortion_model: plumb_bob
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [k1, k2, p1, p2, k3]
    rectification_matrix:
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    projection_matrix:
      rows: 3
      cols: 4
      data: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import yaml
from sensor_msgs.msg import CameraInfo


# Placeholder horizontal FOV used when no calibration file is supplied. 50°
# matches a typical macroscope lens close enough that RViz's Camera display
# renders frustum + image without obvious skew. Treat as a sentinel — every
# downstream consumer that needs metric accuracy must be paired with a
# real calibration YAML.
_PLACEHOLDER_HFOV_DEG = 50.0


def _matrix(data: list[float], rows: int, cols: int) -> list[float]:
    if len(data) != rows * cols:
        raise ValueError(
            f"matrix data length {len(data)} != rows*cols {rows * cols}"
        )
    return [float(v) for v in data]


def load_camera_info_yaml(path: Path) -> CameraInfo:
    raw = yaml.safe_load(path.read_text()) or {}
    info = CameraInfo()
    info.width = int(raw.get("image_width", 0))
    info.height = int(raw.get("image_height", 0))
    info.distortion_model = str(raw.get("distortion_model", "plumb_bob"))

    cam = raw.get("camera_matrix", {}) or {}
    info.k = _matrix(cam.get("data", [0.0] * 9), int(cam.get("rows", 3)), int(cam.get("cols", 3)))

    dist = raw.get("distortion_coefficients", {}) or {}
    info.d = list(_matrix(
        dist.get("data", []),
        int(dist.get("rows", 1)),
        int(dist.get("cols", len(dist.get("data", [])) or 5)),
    ))

    rect = raw.get("rectification_matrix", {}) or {}
    info.r = _matrix(
        rect.get("data", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
        int(rect.get("rows", 3)),
        int(rect.get("cols", 3)),
    )

    proj = raw.get("projection_matrix", {}) or {}
    info.p = _matrix(
        proj.get("data", info.k[:3] + [0.0] + info.k[3:6] + [0.0] + info.k[6:9] + [0.0]),
        int(proj.get("rows", 3)),
        int(proj.get("cols", 4)),
    )
    return info


def synthesise_placeholder(width: int, height: int) -> CameraInfo:
    """Build a plumb_bob CameraInfo with zero distortion + 50° HFOV.

    Cheap stand-in until a real calibration yaml lands at
    `providers/imaging_station/calibration/zowiebox_camera_info.yaml`. Good
    enough for RViz frustum visualisation and TF-anchored projection;
    do not use for metric reconstruction.
    """
    cx = width / 2.0
    cy = height / 2.0
    fx = (width / 2.0) / math.tan(math.radians(_PLACEHOLDER_HFOV_DEG / 2.0))
    fy = fx  # square pixels assumption

    info = CameraInfo()
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def resolve_camera_info(
    calibration_path: Optional[Path], width: int, height: int
) -> tuple[CameraInfo, str]:
    """Return (CameraInfo, source-description). Tries the YAML first."""
    if calibration_path and calibration_path.is_file():
        info = load_camera_info_yaml(calibration_path)
        # Trust the YAML's resolution unless it's missing/zero — operators
        # expect the calibrated dims to win over the live frame.
        if info.width == 0 or info.height == 0:
            info.width = width
            info.height = height
        return info, f"loaded {calibration_path}"
    return synthesise_placeholder(width, height), (
        f"synthesised placeholder ({_PLACEHOLDER_HFOV_DEG:.0f}° HFOV, zero distortion)"
    )
