#!/usr/bin/env python3
"""Live-tunable visualisation of the seed-gripper fingers + grip point.

Publishes a latched MarkerArray for RViz, plus a 6-DoF interactive marker
gizmo per pose so the offsets can be dragged in 3D. The same poses are
exposed as ROS parameters (so rqt_reconfigure provides slider controls)
and persisted to / loaded from a YAML file via two services:

    /seed_gripper/save_offsets  std_srvs/srv/Trigger
    /seed_gripper/load_offsets  std_srvs/srv/Trigger
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from builtin_interfaces.msg import Time
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


# ────────────────────────────── math helpers ──────────────────────────────


def quat_from_rpy(r: float, p: float, y: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def rpy_from_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ───────────────────────────── pose entries ──────────────────────────────


@dataclass
class PoseEntry:
    """One tunable pose: parent frame, mesh path (or None for grip sphere), live values."""

    key: str
    parent_frame: str
    label: str
    color: tuple[float, float, float, float]
    mesh_path: str | None  # None means "draw a sphere" (grip point marker)
    xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def pose(self) -> Pose:
        qx, qy, qz, qw = quat_from_rpy(*self.rpy)
        p = Pose()
        p.position.x, p.position.y, p.position.z = self.xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        return p


# ────────────────────────────── the node ─────────────────────────────────


class SeedGripperVisualizer(Node):
    AXIS_KEYS = ("x", "y", "z")
    RPY_KEYS = ("r", "p", "y")
    XYZ_RANGE = (-0.1, 0.1)  # 100 mm in either direction
    RPY_RANGE = (-math.pi, math.pi)

    def __init__(self) -> None:
        super().__init__("seed_gripper_visualizer")

        tools_dir = Path(__file__).resolve().parent
        provider_dir = tools_dir.parent
        default_left = str(provider_dir / "seed_gripper-Finger.stl")
        default_right = str(provider_dir / "seed_gripper-Finger-right.stl")
        default_yaml = str(tools_dir / "seed_gripper_offsets.yaml")

        self.declare_parameter("left_mesh_path", default_left)
        self.declare_parameter("right_mesh_path", default_right)
        self.declare_parameter("offsets_yaml_path", default_yaml)
        self.declare_parameter(
            "stl_scale",
            0.001,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Mesh scale factor (mm->m = 0.001).",
                floating_point_range=[FloatingPointRange(from_value=0.0001, to_value=1.0, step=0.0)],
            ),
        )
        self.declare_parameter("left_parent_frame", "meca_gripper_finger1")
        self.declare_parameter("right_parent_frame", "meca_gripper_finger2")
        self.declare_parameter("grip_parent_frame", "meca_gripper_link")

        # Per-marker visibility — toggle in rqt_reconfigure to hide finger
        # meshes while you tune the grip point (and vice-versa).
        bool_desc = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        self.declare_parameter("show_left_finger", True, bool_desc)
        self.declare_parameter("show_right_finger", True, bool_desc)
        self.declare_parameter("show_grip_point", True, bool_desc)
        self.declare_parameter("show_axes", True, bool_desc)

        # Momentary triggers for save/load. Tick the checkbox to fire once;
        # untick to re-arm. Same effect as calling the matching service.
        self.declare_parameter(
            "save_now", False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="False -> True transition: write offsets to the YAML.",
            ),
        )
        self.declare_parameter(
            "load_now", False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="False -> True transition: re-read offsets from the YAML.",
            ),
        )
        self._save_now_prev = False
        self._load_now_prev = False

        left_mesh = self.get_parameter("left_mesh_path").value
        right_mesh = self.get_parameter("right_mesh_path").value

        self.poses: dict[str, PoseEntry] = {
            "left": PoseEntry(
                key="left",
                parent_frame=self.get_parameter("left_parent_frame").value,
                label="Left finger",
                color=(0.25, 0.75, 1.0, 1.0),
                mesh_path=left_mesh,
            ),
            "right": PoseEntry(
                key="right",
                parent_frame=self.get_parameter("right_parent_frame").value,
                label="Right finger",
                color=(1.0, 0.55, 0.15, 1.0),
                mesh_path=right_mesh,
            ),
            "grip": PoseEntry(
                key="grip",
                parent_frame=self.get_parameter("grip_parent_frame").value,
                label="Grip point",
                color=(0.1, 1.0, 0.1, 1.0),
                mesh_path=None,
            ),
        }

        self._declare_pose_parameters()

        # Auto-load saved offsets if the file exists.
        yaml_path = Path(self.get_parameter("offsets_yaml_path").value)
        if yaml_path.exists():
            self._load_yaml(yaml_path, log=True)

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._marker_pub = self.create_publisher(MarkerArray, "/seed_gripper/visualization", latched_qos)

        self._im_server = InteractiveMarkerServer(self, "seed_gripper_im")
        self._suppress_param_callback = False
        self._build_interactive_markers()

        self.add_on_set_parameters_callback(self._on_param_change)

        self.create_service(Trigger, "/seed_gripper/save_offsets", self._handle_save)
        self.create_service(Trigger, "/seed_gripper/load_offsets", self._handle_load)

        self._publish_markers()
        self.get_logger().info(
            f"seed_gripper_visualizer ready. YAML: {yaml_path}. "
            f"Drag the gizmos in RViz, slide values in rqt_reconfigure, or call "
            f"/seed_gripper/save_offsets when happy."
        )

    # ── parameter wiring ─────────────────────────────────────────────────

    def _declare_pose_parameters(self) -> None:
        for entry in self.poses.values():
            for i, axis in enumerate(self.AXIS_KEYS):
                self.declare_parameter(
                    f"{entry.key}_xyz_{axis}",
                    entry.xyz[i],
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description=f"{entry.label} translation along {axis} [m].",
                        floating_point_range=[
                            FloatingPointRange(
                                from_value=self.XYZ_RANGE[0], to_value=self.XYZ_RANGE[1], step=0.0
                            )
                        ],
                    ),
                )
            for i, axis in enumerate(self.RPY_KEYS):
                self.declare_parameter(
                    f"{entry.key}_rpy_{axis}",
                    entry.rpy[i],
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description=f"{entry.label} rotation {axis} [rad].",
                        floating_point_range=[
                            FloatingPointRange(
                                from_value=self.RPY_RANGE[0], to_value=self.RPY_RANGE[1], step=0.0
                            )
                        ],
                    ),
                )
            for i, axis in enumerate(self.AXIS_KEYS):
                entry.xyz[i] = self.get_parameter(f"{entry.key}_xyz_{axis}").value
            for i, axis in enumerate(self.RPY_KEYS):
                entry.rpy[i] = self.get_parameter(f"{entry.key}_rpy_{axis}").value

    def _on_param_change(self, params) -> SetParametersResult:
        if self._suppress_param_callback:
            return SetParametersResult(successful=True)
        for p in params:
            if p.name == "save_now":
                if bool(p.value) and not self._save_now_prev:
                    self._handle_save(Trigger.Request(), Trigger.Response())
                self._save_now_prev = bool(p.value)
                continue
            if p.name == "load_now":
                if bool(p.value) and not self._load_now_prev:
                    self._handle_load(Trigger.Request(), Trigger.Response())
                self._load_now_prev = bool(p.value)
                continue
            for entry in self.poses.values():
                prefix = f"{entry.key}_"
                if not p.name.startswith(prefix):
                    continue
                tail = p.name[len(prefix):]
                if tail.startswith("xyz_"):
                    idx = self.AXIS_KEYS.index(tail[-1])
                    entry.xyz[idx] = float(p.value)
                elif tail.startswith("rpy_"):
                    idx = self.RPY_KEYS.index(tail[-1])
                    entry.rpy[idx] = float(p.value)
        self._refresh_interactive_markers()
        self._publish_markers()
        return SetParametersResult(successful=True)

    # ── interactive markers ─────────────────────────────────────────────

    def _build_interactive_markers(self) -> None:
        for entry in self.poses.values():
            self._im_server.insert(self._make_interactive_marker(entry), feedback_callback=self._on_im_feedback)
        self._im_server.applyChanges()

    def _refresh_interactive_markers(self) -> None:
        for entry in self.poses.values():
            self._im_server.setPose(entry.key, entry.pose())
        self._im_server.applyChanges()

    def _make_interactive_marker(self, entry: PoseEntry) -> InteractiveMarker:
        im = InteractiveMarker()
        im.header.frame_id = entry.parent_frame
        im.name = entry.key
        im.description = entry.label
        im.scale = 0.04
        im.pose = entry.pose()
        # No visual proxy — the MarkerArray already shows the finger / sphere
        # at the same place; a duplicate proxy here causes z-fighting flicker.
        # Six 6-DoF rings/arrows do all the dragging.
        for axis_q, name in (
            ((1.0, 0.0, 0.0, 1.0), "x"),
            ((0.0, 1.0, 0.0, 1.0), "y"),
            ((0.0, 0.0, 1.0, 1.0), "z"),
        ):
            qx, qy, qz, qw = self._normalize(*axis_q)
            for mode, suffix in (
                (InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
                (InteractiveMarkerControl.MOVE_AXIS, "move"),
            ):
                ctrl = InteractiveMarkerControl()
                ctrl.orientation.x = qx
                ctrl.orientation.y = qy
                ctrl.orientation.z = qz
                ctrl.orientation.w = qw
                ctrl.name = f"{suffix}_{name}"
                ctrl.interaction_mode = mode
                im.controls.append(ctrl)
        return im

    @staticmethod
    def _normalize(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        return x / n, y / n, z / n, w / n

    def _on_im_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        entry = self.poses.get(feedback.marker_name)
        if entry is None:
            return
        p = feedback.pose
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Live preview only: update internal state + MarkerArray. Don't
            # touch parameters or call setPose — both round-trip through
            # set_parameters / RViz and cause flicker mid-drag.
            entry.xyz = [p.position.x, p.position.y, p.position.z]
            r, pp, y = rpy_from_quat(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            )
            entry.rpy = [r, pp, y]
            self._publish_markers()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Drag complete. Persist the final pose:
            #   1) server-side setPose so RViz doesn't snap the gizmo back
            #      to the pre-drag pose on the next sync,
            #   2) mirror into ROS parameters so rqt_reconfigure / save
            #      reflect the new value.
            self._im_server.setPose(feedback.marker_name, p)
            self._im_server.applyChanges()
            self._mirror_pose_to_params(entry)
            self._publish_markers()

    def _mirror_pose_to_params(self, entry: PoseEntry) -> None:
        from rclpy.parameter import Parameter

        params = []
        for i, axis in enumerate(self.AXIS_KEYS):
            params.append(Parameter(f"{entry.key}_xyz_{axis}", Parameter.Type.DOUBLE, entry.xyz[i]))
        for i, axis in enumerate(self.RPY_KEYS):
            params.append(Parameter(f"{entry.key}_rpy_{axis}", Parameter.Type.DOUBLE, entry.rpy[i]))
        self._suppress_param_callback = True
        try:
            self.set_parameters(params)
        finally:
            self._suppress_param_callback = False

    # ── marker publish ──────────────────────────────────────────────────

    def _publish_markers(self) -> None:
        scale = float(self.get_parameter("stl_scale").value)
        visibility = {
            "left": bool(self.get_parameter("show_left_finger").value),
            "right": bool(self.get_parameter("show_right_finger").value),
            "grip": bool(self.get_parameter("show_grip_point").value),
        }
        show_axes = bool(self.get_parameter("show_axes").value)
        msg = MarkerArray()
        # Stamp with t=0 ("use the latest available TF"). Real wall-time
        # stamps race against robot_state_publisher's TF stream and trip
        # extrapolation errors when our clock leads the latest tf by a few
        # microseconds; markers attached to robot links don't need a real
        # timestamp anyway.
        latest_tf = Time()

        for idx, entry in enumerate(self.poses.values()):
            m = Marker()
            m.header = Header(frame_id=entry.parent_frame, stamp=latest_tf)
            m.ns = "seed_gripper"
            m.id = idx
            if not visibility[entry.key]:
                m.action = Marker.DELETE
                msg.markers.append(m)
                continue
            m.action = Marker.ADD
            m.pose = entry.pose()
            m.color = ColorRGBA(r=entry.color[0], g=entry.color[1], b=entry.color[2], a=entry.color[3])

            if entry.mesh_path is not None:
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = f"file://{entry.mesh_path}"
                m.mesh_use_embedded_materials = False
                m.scale = Vector3(x=scale, y=scale, z=scale)
            else:
                m.type = Marker.SPHERE
                m.scale = Vector3(x=0.010, y=0.010, z=0.010)  # 10 mm — readable on screen
            msg.markers.append(m)

        # Coordinate axes at the grip point so orientation is readable.
        grip = self.poses["grip"]
        axes_specs = (
            ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 1.0)),  # X red
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0, 1.0)),  # Y green
            ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)),  # Z blue
        )
        axis_length = 0.030
        for ai, (color, axis_dir) in enumerate(axes_specs):
            arrow = Marker()
            arrow.header = Header(frame_id=grip.parent_frame, stamp=latest_tf)
            arrow.ns = "seed_gripper_axes"
            arrow.id = ai
            if not show_axes:
                arrow.action = Marker.DELETE
                msg.markers.append(arrow)
                continue
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale = Vector3(x=0.003, y=0.005, z=0.0)  # shaft / head diameters
            arrow.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            tip = grip.xyz
            arrow.points = [
                Point(x=tip[0], y=tip[1], z=tip[2]),
                Point(
                    x=tip[0] + axis_length * axis_dir[0],
                    y=tip[1] + axis_length * axis_dir[1],
                    z=tip[2] + axis_length * axis_dir[2],
                ),
            ]
            msg.markers.append(arrow)

        self._marker_pub.publish(msg)

    # ── save / load ─────────────────────────────────────────────────────

    def _handle_save(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        path = Path(self.get_parameter("offsets_yaml_path").value)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "stl_scale": float(self.get_parameter("stl_scale").value),
                **{
                    e.key: {
                        "parent_frame": e.parent_frame,
                        "xyz": [float(v) for v in e.xyz],
                        "rpy": [float(v) for v in e.rpy],
                    }
                    for e in self.poses.values()
                },
            }
            path.write_text(
                "# seed gripper offsets — generated by seed_gripper_visualizer\n"
                "# rpy is intrinsic XYZ (roll, pitch, yaw) in radians; xyz in meters.\n"
                + yaml.safe_dump(data, sort_keys=False)
            )
            response.success = True
            response.message = f"saved to {path}"
            self.get_logger().info(response.message)
        except OSError as err:
            response.success = False
            response.message = f"save failed: {err}"
            self.get_logger().error(response.message)
        return response

    def _handle_load(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        path = Path(self.get_parameter("offsets_yaml_path").value)
        if not path.exists():
            response.success = False
            response.message = f"no file at {path}"
            return response
        try:
            self._load_yaml(path, log=True)
            self._mirror_all_to_params()
            self._refresh_interactive_markers()
            self._publish_markers()
            response.success = True
            response.message = f"loaded {path}"
        except (OSError, yaml.YAMLError) as err:
            response.success = False
            response.message = f"load failed: {err}"
        return response

    def _load_yaml(self, path: Path, log: bool) -> None:
        data = yaml.safe_load(path.read_text()) or {}
        for key, entry in self.poses.items():
            section = data.get(key) or {}
            if "xyz" in section:
                entry.xyz = [float(v) for v in section["xyz"]]
            if "rpy" in section:
                entry.rpy = [float(v) for v in section["rpy"]]
        if log:
            self.get_logger().info(f"loaded offsets from {path}")

    def _mirror_all_to_params(self) -> None:
        for entry in self.poses.values():
            self._mirror_pose_to_params(entry)


def main() -> None:
    rclpy.init()
    node = SeedGripperVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
