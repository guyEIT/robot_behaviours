#!/usr/bin/env python3
"""Live-tunable visualisation of the microscope stand + camera placement.

Same UX as providers/meca500/tools/seed_gripper_visualizer.py — drag in
RViz, slide in rqt_reconfigure, persist to YAML — but tuned for a
parent->stand->camera chain rather than the gripper's three poses.

Each tunable item:
    * appears as a mesh in a latched MarkerArray on /microscope/visualization
    * is exposed as ROS parameters (xyz/rpy doubles with FloatingPointRange
      so rqt_reconfigure renders sliders)
    * has a 6-DoF interactive marker so you can drag it in 3D
    * is broadcast as a static TF every time it changes, so other nodes
      (the imaging-station auto-aim, RViz, the calibration target's
      centre frame) can reference 'microscope_stand' / 'microscope_camera'
      directly

Two services:
    /microscope/save_offsets  std_srvs/srv/Trigger
    /microscope/load_offsets  std_srvs/srv/Trigger
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, TransformStamped, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


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
    """One tunable pose: parent frame, child frame, mesh + colour, live xyz/rpy."""

    key: str
    parent_frame: str
    child_frame: str
    label: str
    color: tuple[float, float, float, float]
    mesh_path: str
    xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def pose(self) -> Pose:
        qx, qy, qz, qw = quat_from_rpy(*self.rpy)
        p = Pose()
        p.position.x, p.position.y, p.position.z = self.xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        return p


@dataclass
class ReferenceMarker:
    """A non-tunable reference fixture rendered for context (robot base, table)."""

    key: str
    frame_id: str
    mesh_path: str
    color: tuple[float, float, float, float]
    xyz: list[float]
    rpy: list[float]
    scale: float


# ────────────────────────────── the node ─────────────────────────────────


class MicroscopeVisualizer(Node):
    AXIS_KEYS = ("x", "y", "z")
    RPY_KEYS = ("r", "p", "y")
    XYZ_RANGE = (-1.0, 1.0)            # the bench is bigger than the gripper
    RPY_RANGE = (-math.pi, math.pi)

    def __init__(self) -> None:
        super().__init__("microscope_visualizer")

        here = Path(__file__).resolve().parent
        provider_dir = here.parent
        models_dir = provider_dir / "models"
        default_stand_mesh = str(models_dir / "microscope-stand.stl")
        default_camera_mesh = str(models_dir / "microscope-camera.stl")
        default_yaml = str(here / "microscope_offsets.yaml")

        self.declare_parameter("stand_mesh_path", default_stand_mesh)
        self.declare_parameter("camera_mesh_path", default_camera_mesh)
        self.declare_parameter("offsets_yaml_path", default_yaml)
        self.declare_parameter(
            "stl_scale",
            0.001,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Mesh scale factor (mm->m = 0.001).",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0001, to_value=1.0, step=0.0)
                ],
            ),
        )
        # Parent / child frame ids — exposed as parameters so the chain can
        # be retargeted (e.g. parent='world' for standalone tuning when no
        # ArUco calibration is running).
        self.declare_parameter("stand_parent_frame", "calibration_target_centre")
        # Empty string means "use stand_parent_frame" — keeps the legacy
        # one-parent-for-both behaviour. Set to e.g. 'calibration_marker' to
        # tune just the camera (and its chained optical frame) directly
        # against the ArUco fiducial — useful when the marker frame is
        # offset from the printed cross-hair and you want to absorb that
        # offset into camera.xyz rather than chasing it via target_in_marker.
        self.declare_parameter("camera_parent_frame", "")
        self.declare_parameter("stand_frame_id", "microscope_stand")
        self.declare_parameter("camera_frame_id", "microscope_camera")

        bool_desc = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        self.declare_parameter("show_stand", True, bool_desc)
        self.declare_parameter("show_camera", True, bool_desc)
        self.declare_parameter("show_axes", True, bool_desc)
        self.declare_parameter("show_meca_base", True, bool_desc)
        self.declare_parameter("show_mecatable", True, bool_desc)

        # Reference fixtures: meca500 base + mecatable, rendered as
        # non-interactive markers so the imaging station can be aligned
        # against the same world the robot lives in. Empty mesh_path
        # disables the marker.
        self.declare_parameter("meca_base_mesh_path", "")
        self.declare_parameter("meca_base_frame", "meca_base_link")
        self.declare_parameter("mecatable_mesh_path", "")
        self.declare_parameter("mecatable_frame", "world")
        self.declare_parameter("mecatable_x", 0.0)
        self.declare_parameter("mecatable_y", 0.08)
        self.declare_parameter("mecatable_z", -0.23)
        self.declare_parameter("mecatable_roll", 1.5708)
        self.declare_parameter("mecatable_pitch", 0.0)
        self.declare_parameter("mecatable_yaw", 1.5708)
        self.declare_parameter("mecatable_scale", 0.001)

        # Momentary save/load triggers (False -> True transitions) — same
        # convention as the seed_gripper visualizer so muscle memory transfers.
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

        stand_parent_frame = self.get_parameter("stand_parent_frame").value
        camera_parent_frame_override = (
            self.get_parameter("camera_parent_frame").value or ""
        ).strip()
        camera_parent_frame = camera_parent_frame_override or stand_parent_frame
        stand_frame_id = self.get_parameter("stand_frame_id").value
        camera_frame_id = self.get_parameter("camera_frame_id").value
        stand_mesh = self.get_parameter("stand_mesh_path").value
        camera_mesh = self.get_parameter("camera_mesh_path").value

        # Both meshes are tuned independently — by default against the same
        # anchor frame (calibration_target_centre, the ArUco-resolved imaging
        # centre), but the camera entry can be re-parented (camera_parent_frame
        # parameter) so its xyz/rpy is interpreted in e.g. calibration_marker
        # directly. xyz/rpy are mesh-pose-in-anchor, never chained.
        self.poses: dict[str, PoseEntry] = {
            "stand": PoseEntry(
                key="stand",
                parent_frame=stand_parent_frame,
                child_frame=stand_frame_id,
                label="Microscope stand",
                color=(0.55, 0.55, 0.65, 1.0),
                mesh_path=stand_mesh,
            ),
            "camera": PoseEntry(
                key="camera",
                parent_frame=camera_parent_frame,
                child_frame=camera_frame_id,
                label="Microscope camera",
                color=(0.20, 0.65, 1.0, 1.0),
                mesh_path=camera_mesh,
            ),
        }

        self._declare_pose_parameters()

        self.references: list[ReferenceMarker] = []
        meca_base_mesh = str(self.get_parameter("meca_base_mesh_path").value)
        if meca_base_mesh:
            self.references.append(
                ReferenceMarker(
                    key="meca_base",
                    frame_id=str(self.get_parameter("meca_base_frame").value),
                    mesh_path=meca_base_mesh,
                    color=(0.85, 0.85, 0.88, 1.0),
                    xyz=[0.0, 0.0, 0.0],
                    rpy=[0.0, 0.0, 0.0],
                    scale=1.0,
                )
            )
        mecatable_mesh = str(self.get_parameter("mecatable_mesh_path").value)
        if mecatable_mesh:
            self.references.append(
                ReferenceMarker(
                    key="mecatable",
                    frame_id=str(self.get_parameter("mecatable_frame").value),
                    mesh_path=mecatable_mesh,
                    color=(0.40, 0.30, 0.20, 1.0),
                    xyz=[
                        float(self.get_parameter("mecatable_x").value),
                        float(self.get_parameter("mecatable_y").value),
                        float(self.get_parameter("mecatable_z").value),
                    ],
                    rpy=[
                        float(self.get_parameter("mecatable_roll").value),
                        float(self.get_parameter("mecatable_pitch").value),
                        float(self.get_parameter("mecatable_yaw").value),
                    ],
                    scale=float(self.get_parameter("mecatable_scale").value),
                )
            )

        yaml_path = Path(self.get_parameter("offsets_yaml_path").value)
        if yaml_path.exists():
            self._load_yaml(yaml_path, log=True)

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/microscope/visualization", latched_qos
        )

        # StaticTransformBroadcaster so other nodes see microscope_stand /
        # microscope_camera as real TF frames as soon as we publish.
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._im_server = InteractiveMarkerServer(self, "microscope_im")
        self._suppress_param_callback = False
        self._build_interactive_markers()

        self.add_on_set_parameters_callback(self._on_param_change)

        self.create_service(Trigger, "/microscope/save_offsets", self._handle_save)
        self.create_service(Trigger, "/microscope/load_offsets", self._handle_load)

        self._publish_static_tfs()
        self._publish_markers()
        self.get_logger().info(
            f"microscope_visualizer ready. YAML: {yaml_path}. "
            f"Drag the gizmos in RViz, slide values in rqt_reconfigure, or call "
            f"/microscope/save_offsets when the rig matches the bench."
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
                                from_value=self.XYZ_RANGE[0],
                                to_value=self.XYZ_RANGE[1],
                                step=0.0,
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
                                from_value=self.RPY_RANGE[0],
                                to_value=self.RPY_RANGE[1],
                                step=0.0,
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
        self._publish_static_tfs()
        self._publish_markers()
        return SetParametersResult(successful=True)

    # ── interactive markers ─────────────────────────────────────────────

    def _build_interactive_markers(self) -> None:
        for entry in self.poses.values():
            self._im_server.insert(
                self._make_interactive_marker(entry),
                feedback_callback=self._on_im_feedback,
            )
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
        # The stand is bigger than the gripper, so give the gizmo more room.
        im.scale = 0.12 if entry.key == "stand" else 0.08
        im.pose = entry.pose()
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
            entry.xyz = [p.position.x, p.position.y, p.position.z]
            r, pp, y = rpy_from_quat(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            )
            entry.rpy = [r, pp, y]
            self._publish_static_tfs()
            self._publish_markers()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._im_server.setPose(feedback.marker_name, p)
            self._im_server.applyChanges()
            self._mirror_pose_to_params(entry)
            self._publish_static_tfs()
            self._publish_markers()

    def _mirror_pose_to_params(self, entry: PoseEntry) -> None:
        from rclpy.parameter import Parameter

        params = []
        for i, axis in enumerate(self.AXIS_KEYS):
            params.append(
                Parameter(f"{entry.key}_xyz_{axis}", Parameter.Type.DOUBLE, entry.xyz[i])
            )
        for i, axis in enumerate(self.RPY_KEYS):
            params.append(
                Parameter(f"{entry.key}_rpy_{axis}", Parameter.Type.DOUBLE, entry.rpy[i])
            )
        self._suppress_param_callback = True
        try:
            self.set_parameters(params)
        finally:
            self._suppress_param_callback = False

    # ── tf broadcast ────────────────────────────────────────────────────

    def _publish_static_tfs(self) -> None:
        """Re-broadcast the parent->stand and stand->camera static TFs.

        Static TFs are latched (TRANSIENT_LOCAL, depth=1) so re-publishing
        on every drag tick is fine — late subscribers always get the
        newest value. This is what lets RViz / downstream nodes see the
        microscope chain update live as the gizmos move.
        """
        stamp = self.get_clock().now().to_msg()
        msgs = []
        for entry in self.poses.values():
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = entry.parent_frame
            t.child_frame_id = entry.child_frame
            t.transform.translation.x = float(entry.xyz[0])
            t.transform.translation.y = float(entry.xyz[1])
            t.transform.translation.z = float(entry.xyz[2])
            qx, qy, qz, qw = quat_from_rpy(*entry.rpy)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            msgs.append(t)
        self._tf_broadcaster.sendTransform(msgs)

    # ── marker publish ──────────────────────────────────────────────────

    def _publish_markers(self) -> None:
        scale = float(self.get_parameter("stl_scale").value)
        visibility = {
            "stand": bool(self.get_parameter("show_stand").value),
            "camera": bool(self.get_parameter("show_camera").value),
        }
        show_axes = bool(self.get_parameter("show_axes").value)
        msg = MarkerArray()
        latest_tf = Time()  # t=0 -> "use latest available TF" (avoids extrapolation)

        for idx, entry in enumerate(self.poses.values()):
            m = Marker()
            # Bake the pose directly into the marker (parented to the parent
            # frame) instead of leaning on a child-frame TF lookup. ROS 2's
            # /tf_static cache doesn't always re-evaluate stale transforms
            # in RViz between re-publishes, so the gizmo could rotate the
            # frame without the mesh visibly following. This way the mesh
            # carries its own transform and RViz updates immediately on the
            # next MarkerArray message.
            m.header = Header(frame_id=entry.parent_frame, stamp=latest_tf)
            m.ns = "microscope"
            m.id = idx
            if not visibility[entry.key]:
                m.action = Marker.DELETE
                msg.markers.append(m)
                continue
            m.action = Marker.ADD
            m.pose = entry.pose()
            m.frame_locked = True
            m.color = ColorRGBA(
                r=entry.color[0], g=entry.color[1], b=entry.color[2], a=entry.color[3]
            )
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = f"file://{entry.mesh_path}"
            m.mesh_use_embedded_materials = False
            m.scale = Vector3(x=scale, y=scale, z=scale)
            msg.markers.append(m)

        # Reference fixtures (robot base + mecatable). Non-interactive,
        # non-tunable — they sit at fixed poses in the parent frames so the
        # imaging-station tune is visible in the same scene as the robot.
        ref_visibility = {
            "meca_base": bool(self.get_parameter("show_meca_base").value),
            "mecatable": bool(self.get_parameter("show_mecatable").value),
        }
        for ref_idx, ref in enumerate(self.references):
            rm = Marker()
            rm.header = Header(frame_id=ref.frame_id, stamp=latest_tf)
            rm.ns = "microscope_reference"
            rm.id = ref_idx
            if not ref_visibility.get(ref.key, True):
                rm.action = Marker.DELETE
                msg.markers.append(rm)
                continue
            rm.action = Marker.ADD
            rm.type = Marker.MESH_RESOURCE
            rm.mesh_resource = f"file://{ref.mesh_path}"
            rm.mesh_use_embedded_materials = False
            rm.scale = Vector3(x=ref.scale, y=ref.scale, z=ref.scale)
            rm.color = ColorRGBA(
                r=ref.color[0], g=ref.color[1], b=ref.color[2], a=ref.color[3]
            )
            rm.pose = Pose()
            rm.pose.position.x = float(ref.xyz[0])
            rm.pose.position.y = float(ref.xyz[1])
            rm.pose.position.z = float(ref.xyz[2])
            qx, qy, qz, qw = quat_from_rpy(*ref.rpy)
            rm.pose.orientation.x = qx
            rm.pose.orientation.y = qy
            rm.pose.orientation.z = qz
            rm.pose.orientation.w = qw
            msg.markers.append(rm)

        # Axes at each entry's origin so the position is readable while
        # tuning. Drawn in parent_frame and parent-axis-aligned (no rotation
        # follow) — the interactive marker gizmo already shows orientation.
        axes_specs = (
            ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),  # X red
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),  # Y green
            ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),  # Z blue
        )
        axis_length = 0.05
        marker_id_offset = 100
        for entry in self.poses.values():
            for ai, (axis_dir, color) in enumerate(axes_specs):
                arrow = Marker()
                arrow.header = Header(frame_id=entry.parent_frame, stamp=latest_tf)
                arrow.ns = f"microscope_axes_{entry.key}"
                arrow.id = marker_id_offset + ai
                if not show_axes:
                    arrow.action = Marker.DELETE
                    msg.markers.append(arrow)
                    continue
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale = Vector3(x=0.004, y=0.008, z=0.0)
                arrow.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                tip = entry.xyz
                arrow.points = [
                    Point(x=tip[0], y=tip[1], z=tip[2]),
                    Point(
                        x=tip[0] + axis_length * axis_dir[0],
                        y=tip[1] + axis_length * axis_dir[1],
                        z=tip[2] + axis_length * axis_dir[2],
                    ),
                ]
                msg.markers.append(arrow)
            marker_id_offset += 10

        self._marker_pub.publish(msg)

    # ── save / load ─────────────────────────────────────────────────────

    def _handle_save(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
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
                "# microscope stand + camera offsets — generated by microscope_visualizer\n"
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

    def _handle_load(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        path = Path(self.get_parameter("offsets_yaml_path").value)
        if not path.exists():
            response.success = False
            response.message = f"no file at {path}"
            return response
        try:
            self._load_yaml(path, log=True)
            self._mirror_all_to_params()
            self._refresh_interactive_markers()
            self._publish_static_tfs()
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
            # parent_frame in YAML is informational — the live chain is
            # determined by the *_parent_frame / *_frame_id parameters so
            # we don't accidentally break the TF tree by loading a file
            # written under a different frame configuration.
        if log:
            self.get_logger().info(f"loaded offsets from {path}")

    def _mirror_all_to_params(self) -> None:
        for entry in self.poses.values():
            self._mirror_pose_to_params(entry)


def main() -> None:
    rclpy.init()
    node = MicroscopeVisualizer()
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
