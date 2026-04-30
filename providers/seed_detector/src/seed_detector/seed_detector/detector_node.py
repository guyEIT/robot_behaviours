#!/usr/bin/env python3
"""ROS 2 node: YOLO seed detection on a RealSense RGB+depth stream."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from seed_detector.detector import SeedDetector, sample_depth_patch


class SeedDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__("seed_detector")

        # ── parameters ────────────────────────────────────────────────────
        default_model = str(
            Path(get_package_share_directory("seed_detector")) / "models" / "last.pt"
        )
        self.declare_parameter(
            "model_path", default_model,
            ParameterDescriptor(description="Absolute path to the YOLO .pt weights"),
        )
        self.declare_parameter(
            "image_topic", "/camera/color/image_raw",
            ParameterDescriptor(description="RGB image input topic"),
        )
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw",
            ParameterDescriptor(description="Depth image aligned to RGB"),
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info",
            ParameterDescriptor(description="CameraInfo for the RGB stream"),
        )
        self.declare_parameter(
            "detections_topic", "~/detections",
            ParameterDescriptor(description="Detection3DArray output topic"),
        )
        self.declare_parameter(
            "markers_topic", "~/markers",
            ParameterDescriptor(description="MarkerArray output topic (debug only)"),
        )
        self.declare_parameter(
            "annotated_image_topic", "~/annotated_image",
            ParameterDescriptor(description="Annotated RGB image topic with bbox overlay (debug only)"),
        )
        self.declare_parameter(
            "confidence_threshold", 0.5,
            ParameterDescriptor(description="YOLO confidence threshold"),
        )
        self.declare_parameter(
            "iou_threshold", 0.45,
            ParameterDescriptor(description="YOLO NMS IoU threshold"),
        )
        self.declare_parameter(
            "detection_rate_hz", 5.0,
            ParameterDescriptor(description="Maximum inference rate; frames between ticks are dropped"),
        )
        self.declare_parameter(
            "device", "",
            ParameterDescriptor(description="Inference device for ultralytics (e.g. 'cpu', 'cuda:0'). Empty = auto."),
        )
        self.declare_parameter(
            "depth_patch_pixels", 5,
            ParameterDescriptor(description="Side length of the depth window around each bbox center"),
        )
        self.declare_parameter(
            "depth_sampling_mode", "foreground",
            ParameterDescriptor(
                description="How to reduce the depth window to one value: "
                            "'foreground' (lower-quantile, biases toward closest object - default) "
                            "or 'median' (legacy; biased behind small targets sitting on a tray)."
            ),
        )
        self.declare_parameter(
            "depth_foreground_quantile", 0.25,
            ParameterDescriptor(
                description="Quantile used by mode='foreground' (0.0=closest valid pixel; 0.5=median). "
                            "Lower = more aggressively foreground-biased. Default 0.25 (lower quartile)."
            ),
        )
        self.declare_parameter(
            "sync_slop_sec", 0.05,
            ParameterDescriptor(description="ApproximateTimeSynchronizer slop between RGB and depth"),
        )
        self.declare_parameter(
            "debug_markers", False,
            ParameterDescriptor(description="Publish a MarkerArray for rviz with one sphere+label per detection"),
        )

        model_path = str(self.get_parameter("model_path").value)
        confidence = float(self.get_parameter("confidence_threshold").value)
        iou = float(self.get_parameter("iou_threshold").value)
        device = str(self.get_parameter("device").value)
        rate_hz = float(self.get_parameter("detection_rate_hz").value)
        self._patch_size = int(self.get_parameter("depth_patch_pixels").value)
        self._depth_sampling_mode = str(self.get_parameter("depth_sampling_mode").value)
        self._depth_foreground_quantile = float(
            self.get_parameter("depth_foreground_quantile").value
        )
        self._debug_markers = bool(self.get_parameter("debug_markers").value)
        self._min_period_sec = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._last_inference_t = 0.0

        image_topic = str(self.get_parameter("image_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        sync_slop = float(self.get_parameter("sync_slop_sec").value)

        # ── load YOLO weights ─────────────────────────────────────────────
        self.get_logger().info(f"loading YOLO weights from {model_path}")
        self._detector = SeedDetector(
            model_path=model_path, confidence=confidence, iou=iou, device=device,
        )
        names = self._detector.class_names
        self.get_logger().info(
            f"model loaded — {len(names)} class(es): {sorted(names.values())}"
        )

        # ── ROS plumbing ──────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._cam_model: Optional[PinholeCameraModel] = None

        # latched-ish QoS for camera_info: keep the most recent message,
        # since it's published once per camera launch and rarely changes.
        info_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, info_qos,
        )

        self._image_sub = Subscriber(self, Image, image_topic)
        self._depth_sub = Subscriber(self, Image, depth_topic)
        self._sync = ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub], queue_size=5, slop=sync_slop,
        )
        self._sync.registerCallback(self._on_frame)

        self._detections_pub = self.create_publisher(
            Detection3DArray, detections_topic, 10,
        )
        if self._debug_markers:
            marker_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._markers_pub = self.create_publisher(
                MarkerArray, markers_topic, marker_qos,
            )
            # Annotated image: regular best-effort 1-deep queue. rqt_image_view
            # / rviz Image displays expect a streaming topic, not a latched one.
            self._annotated_pub = self.create_publisher(Image, annotated_topic, 1)
        else:
            self._markers_pub = None
            self._annotated_pub = None

        self.get_logger().info(
            f"seed_detector ready — image='{image_topic}', depth='{depth_topic}', "
            f"info='{info_topic}', rate={rate_hz} Hz, debug_markers={self._debug_markers}"
        )

    # ── subscribers ───────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._cam_model is None:
            self._cam_model = PinholeCameraModel()
            self.get_logger().info(
                f"got CameraInfo: {msg.width}x{msg.height}, frame_id='{msg.header.frame_id}'"
            )
        self._cam_model.fromCameraInfo(msg)

    def _on_frame(self, image_msg: Image, depth_msg: Image) -> None:
        # Throttle inference. Drop the frame outright; backpressure on the
        # subscriber would queue stale frames.
        now = time.monotonic()
        if self._min_period_sec > 0.0 and (now - self._last_inference_t) < self._min_period_sec:
            return
        if self._cam_model is None:
            self.get_logger().warn(
                "skipping frame — no CameraInfo yet", throttle_duration_sec=5.0,
            )
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            depth, depth_scale = self._depth_to_meters(depth_msg)
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc!r}")
            return

        self._last_inference_t = now
        if self._annotated_pub is not None:
            detections, overlay = self._detector.detect(bgr, with_overlay=True)
        else:
            detections = self._detector.detect(bgr)
            overlay = None

        out_array = Detection3DArray()
        out_array.header = image_msg.header  # camera_color_optical_frame

        markers = MarkerArray() if self._markers_pub is not None else None
        marker_lifetime_sec = max(2.0 * self._min_period_sec, 0.5)

        kept = 0
        for index, det in enumerate(detections):
            depth_m = sample_depth_patch(
                depth, det.cx, det.cy,
                patch=self._patch_size,
                depth_scale_to_meters=depth_scale,
                mode=self._depth_sampling_mode,
                foreground_quantile=self._depth_foreground_quantile,
            )
            if depth_m is None:
                self.get_logger().debug(
                    f"dropping {det.class_name} (score={det.score:.2f}) — "
                    f"no valid depth at ({det.cx:.1f},{det.cy:.1f})"
                )
                continue

            ray = self._cam_model.projectPixelTo3dRay((det.cx, det.cy))
            # projectPixelTo3dRay returns a unit ray in the camera optical
            # frame (Z forward); scale so the Z component equals depth_m
            # (which is the orthographic depth, i.e. the Z value, not the
            # ray length).
            if ray[2] == 0.0:
                continue
            scale = depth_m / ray[2]
            xyz = (ray[0] * scale, ray[1] * scale, ray[2] * scale)

            out_array.detections.append(
                self._build_detection(det, xyz, depth_m, image_msg.header)
            )
            if markers is not None:
                markers.markers.extend(
                    self._build_markers(
                        det, xyz, image_msg.header,
                        index=index, lifetime_sec=marker_lifetime_sec,
                    )
                )
            kept += 1

        self._detections_pub.publish(out_array)
        if markers is not None and self._markers_pub is not None:
            # Always publish (possibly empty) so stale markers get cleared
            # by their lifetime; rviz-side persistence keeps things tidy.
            self._markers_pub.publish(markers)
        if overlay is not None and self._annotated_pub is not None:
            try:
                overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            except Exception as exc:
                self.get_logger().warn(
                    f"failed to encode annotated image: {exc!r}",
                    throttle_duration_sec=5.0,
                )
            else:
                overlay_msg.header = image_msg.header
                self._annotated_pub.publish(overlay_msg)

        if detections:
            self.get_logger().debug(
                f"frame: {len(detections)} raw → {kept} with valid depth"
            )

    # ── helpers ───────────────────────────────────────────────────────────

    def _depth_to_meters(self, msg: Image) -> tuple[np.ndarray, float]:
        """Convert a depth Image into a 2D numpy array + per-pixel meter scale.

        RealSense aligned depth ships as 16UC1 millimetres; some drivers
        publish 32FC1 metres. Pass through as a numpy view; the caller
        applies the scale factor when sampling.
        """
        if msg.encoding in ("16UC1", "mono16"):
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            return depth, 1e-3
        if msg.encoding == "32FC1":
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            return depth, 1.0
        raise ValueError(f"unsupported depth encoding: {msg.encoding!r}")

    def _build_detection(
        self,
        det,
        xyz: tuple[float, float, float],
        depth_m: float,
        header,
    ) -> Detection3D:
        d = Detection3D()
        d.header = header

        # Approximate physical size by projecting the 2D bbox dims to the
        # detection's depth via the camera intrinsics. fx≈fy for the
        # RealSense colour stream, so one factor (depth/fx) covers x and y.
        if self._cam_model is not None and self._cam_model.fx() > 0.0:
            scale_xy = depth_m / self._cam_model.fx()
        else:
            scale_xy = 0.0
        size_x = det.width * scale_xy
        size_y = det.height * scale_xy

        bbox3 = BoundingBox3D()
        bbox3.center = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        # Depth extent is unknown from a single RGB-D frame — assume the
        # object is roughly cubic at the bbox scale (good enough for
        # seed-sized blobs; downstream consumers should treat z as a hint).
        bbox3.size = Vector3(
            x=size_x,
            y=size_y,
            z=max(0.005, 0.5 * (size_x + size_y)),
        )
        d.bbox = bbox3

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(det.class_name)
        hyp.hypothesis.score = float(det.score)
        hyp.pose.pose = bbox3.center
        d.results.append(hyp)
        return d

    def _build_markers(
        self, det, xyz, header, index: int, lifetime_sec: float,
    ) -> list:
        sphere = Marker()
        sphere.header = header
        sphere.ns = "seed_detector"
        sphere.id = 2 * index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = xyz
        sphere.pose.orientation.w = 1.0
        sphere.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1 cm marker
        sphere.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.85)
        sphere.lifetime.sec = int(lifetime_sec)
        sphere.lifetime.nanosec = int((lifetime_sec - int(lifetime_sec)) * 1e9)

        text = Marker()
        text.header = header
        text.ns = "seed_detector"
        text.id = 2 * index + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = xyz
        text.pose.position.z += 0.015  # float the label above the sphere
        text.pose.orientation.w = 1.0
        text.scale.z = 0.012  # text height
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        text.text = f"{det.class_name} {det.score:.2f}"
        text.lifetime = sphere.lifetime
        return [sphere, text]


def main(argv=None):
    rclpy.init(args=argv)
    node = SeedDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
