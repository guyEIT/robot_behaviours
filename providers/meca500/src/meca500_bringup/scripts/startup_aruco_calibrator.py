#!/usr/bin/env python3

import math
from datetime import datetime, timezone

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rot):
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class StartupArucoCalibrator(Node):
    def __init__(self):
        super().__init__("startup_aruco_calibrator")

        self.declare_parameter(
            "parent_frame",
            "meca_base_link",
            ParameterDescriptor(description="Parent frame for calibrated camera transform"),
        )
        self.declare_parameter(
            "camera_frame",
            "camera_link",
            ParameterDescriptor(description="Camera frame to publish in final static transform"),
        )
        self.declare_parameter(
            "calibration_camera_frame",
            "camera_color_optical_frame",
            ParameterDescriptor(description="Camera frame used by the image stream (typically optical frame)"),
        )
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("aruco_dictionary", "DICT_6X6_250")
        self.declare_parameter("marker_id", 23)
        self.declare_parameter("marker_size_m", 0.05)
        self.declare_parameter("averaging_window_s", 5.0)
        self.declare_parameter("marker_parent_x", 0.0)
        self.declare_parameter("marker_parent_y", 0.0)
        self.declare_parameter("marker_parent_z", 0.0)
        self.declare_parameter("marker_parent_roll", 0.0)
        self.declare_parameter("marker_parent_pitch", 0.0)
        self.declare_parameter("marker_parent_yaw", 0.0)
        self.declare_parameter("camera_config_file", "")
        self.declare_parameter("save_to_camera_config", True)
        self.declare_parameter("shutdown_on_complete", True)
        self.declare_parameter("status_topic", "/startup_aruco/status")
        self.declare_parameter("marker_topic", "/startup_aruco/markers")

        self.parent_frame = self.get_parameter("parent_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.calibration_camera_frame = self.get_parameter("calibration_camera_frame").value
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.averaging_window_s = float(self.get_parameter("averaging_window_s").value)
        self.save_to_camera_config = bool(self.get_parameter("save_to_camera_config").value)
        self.camera_config_file = self.get_parameter("camera_config_file").value
        self.shutdown_on_complete = bool(self.get_parameter("shutdown_on_complete").value)
        self.status_topic = self.get_parameter("status_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value

        marker_parent_roll = float(self.get_parameter("marker_parent_roll").value)
        marker_parent_pitch = float(self.get_parameter("marker_parent_pitch").value)
        marker_parent_yaw = float(self.get_parameter("marker_parent_yaw").value)
        marker_parent_x = float(self.get_parameter("marker_parent_x").value)
        marker_parent_y = float(self.get_parameter("marker_parent_y").value)
        marker_parent_z = float(self.get_parameter("marker_parent_z").value)

        self.t_parent_marker = np.eye(4, dtype=np.float64)
        self.t_parent_marker[:3, :3] = rpy_to_matrix(
            marker_parent_roll, marker_parent_pitch, marker_parent_yaw
        )
        self.t_parent_marker[:3, 3] = np.array(
            [marker_parent_x, marker_parent_y, marker_parent_z], dtype=np.float64
        )

        dict_name = self.get_parameter("aruco_dictionary").value
        if not hasattr(cv2.aruco, dict_name):
            raise RuntimeError(f"Unsupported ArUco dictionary: {dict_name}")
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()

        self.k = None
        self.d = None
        self.start_time = None
        self.done = False
        self.parent_camera_samples = []
        self.broadcaster = StaticTransformBroadcaster(self)
        self.shutdown_timer = None
        self.last_progress_publish_time = -1.0
        self.frames_processed = 0
        self.last_no_marker_report_s = -1.0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, latched_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Startup ArUco calibration enabled. Waiting for marker ID {self.marker_id} on "
            f"{self.image_topic} for {self.averaging_window_s:.1f}s averaging."
        )
        self.publish_status("WAITING_CAMERA_INFO", "Waiting for camera intrinsics.")
        self.publish_visual_state(
            state="WAITING_CAMERA_INFO",
            details="Waiting for camera intrinsics",
            sample_count=0,
            translation=None,
            quaternion=None,
        )

    def camera_info_cb(self, msg):
        if self.k is not None:
            return
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.d = np.array(msg.d, dtype=np.float64)
        self.get_logger().info("Received camera intrinsics.")
        self.publish_status("WAITING_MARKER", "Camera intrinsics received.")
        self.publish_visual_state(
            state="WAITING_MARKER",
            details=f"Waiting for marker ID {self.marker_id}",
            sample_count=0,
            translation=None,
            quaternion=None,
        )

    def image_cb(self, msg):
        if self.done or self.k is None:
            return

        gray = self.image_to_gray(msg)
        if gray is None:
            return
        self.frames_processed += 1

        gray = np.ascontiguousarray(gray)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if ids is None:
            if self.last_no_marker_report_s < 0.0 or (now_s - self.last_no_marker_report_s) >= 2.0:
                self.last_no_marker_report_s = now_s
                self.publish_status(
                    "WAITING_MARKER",
                    f"No markers detected yet on {self.image_topic}.",
                    sample_count=len(self.parent_camera_samples),
                )
            return

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self.marker_id)[0]
        if matches.size == 0:
            seen_ids = ",".join(str(int(v)) for v in ids_flat.tolist())
            if self.last_no_marker_report_s < 0.0 or (now_s - self.last_no_marker_report_s) >= 2.0:
                self.last_no_marker_report_s = now_s
                self.publish_status(
                    "WAITING_MARKER",
                    (
                        f"Detected markers [{seen_ids}] but not target ID {self.marker_id}. "
                        f"Check dictionary and marker_id."
                    ),
                    sample_count=len(self.parent_camera_samples),
                )
            return

        idx = int(matches[0])
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[idx]], self.marker_size_m, self.k, self.d
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        t_parent_camera = self.compute_parent_camera_transform(rvec, tvec)
        self.parent_camera_samples.append(t_parent_camera)
        translation = t_parent_camera[:3, 3]
        quat = matrix_to_quaternion(t_parent_camera[:3, :3])

        if self.start_time is None:
            self.start_time = now_s
            self.get_logger().info("Marker detected. Starting averaging window.")
            self.publish_status(
                "COLLECTING",
                f"Marker detected. Collecting samples for {self.averaging_window_s:.1f}s.",
                sample_count=len(self.parent_camera_samples),
            )
            self.publish_visual_state(
                state="COLLECTING",
                details="Marker detected",
                sample_count=len(self.parent_camera_samples),
                translation=translation,
                quaternion=quat,
            )
            return

        elapsed = now_s - self.start_time
        if elapsed < self.averaging_window_s:
            if self.last_progress_publish_time < 0.0 or (now_s - self.last_progress_publish_time) >= 1.0:
                self.last_progress_publish_time = now_s
                self.publish_status(
                    "COLLECTING",
                    f"Collecting samples... {elapsed:.1f}/{self.averaging_window_s:.1f}s",
                    sample_count=len(self.parent_camera_samples),
                )
                self.publish_visual_state(
                    state="COLLECTING",
                    details=f"Averaging window {elapsed:.1f}/{self.averaging_window_s:.1f}s",
                    sample_count=len(self.parent_camera_samples),
                    translation=translation,
                    quaternion=quat,
                )
            return

        self.finish_calibration()

    def image_to_gray(self, msg):
        try:
            if msg.encoding == "mono8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

            if msg.encoding in ("rgb8", "bgr8"):
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            if msg.encoding in ("rgba8", "bgra8"):
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                if msg.encoding == "rgba8":
                    return cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
                return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        except ValueError:
            self.get_logger().warning("Failed to decode image frame for ArUco processing.")
            self.publish_status("ERROR", "Failed to decode image frame.")
            return None

        self.get_logger().warning(f"Unsupported image encoding for calibration: {msg.encoding}")
        self.publish_status("ERROR", f"Unsupported image encoding: {msg.encoding}")
        return None

    def compute_parent_camera_transform(self, rvec, tvec):
        rot_camera_marker, _ = cv2.Rodrigues(rvec)
        t_camera_marker = np.eye(4, dtype=np.float64)
        t_camera_marker[:3, :3] = rot_camera_marker
        t_camera_marker[:3, 3] = np.array(tvec, dtype=np.float64)
        return self.t_parent_marker @ np.linalg.inv(t_camera_marker)

    def finish_calibration(self):
        samples = np.array(self.parent_camera_samples)
        if len(samples) < 5:
            self.get_logger().error(
                f"Calibration failed: only {len(samples)} samples gathered in averaging window."
            )
            self.publish_status("FAILED", "Not enough samples in averaging window.", sample_count=len(samples))
            self.publish_visual_state(
                state="FAILED",
                details=f"Only {len(samples)} samples",
                sample_count=len(samples),
                translation=None,
                quaternion=None,
            )
            return

        translations = samples[:, :3, 3]
        translation_mean = np.mean(translations, axis=0)
        translation_std = np.std(translations, axis=0)
        eps = 1e-6
        mask = np.all(np.abs(translations - translation_mean) <= 2.0 * (translation_std + eps), axis=1)
        filtered = samples[mask]
        if len(filtered) < 3:
            filtered = samples

        avg_translation = np.mean(filtered[:, :3, 3], axis=0)
        quats = []
        for sample in filtered:
            quat = matrix_to_quaternion(sample[:3, :3])
            quats.append(quat)
        quats = np.array(quats)
        ref = quats[0]
        for i in range(len(quats)):
            if np.dot(quats[i], ref) < 0.0:
                quats[i] *= -1.0
        avg_quat = np.mean(quats, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)

        if self.camera_frame != self.calibration_camera_frame:
            converted = self.convert_transform_to_output_frame(avg_translation, avg_quat)
            if converted is None:
                self.publish_status(
                    "FAILED",
                    (
                        f"Could not convert {self.calibration_camera_frame} to {self.camera_frame}. "
                        "Check camera TF tree."
                    ),
                    sample_count=len(filtered),
                )
                return
            avg_translation, avg_quat = converted

        self.publish_static_transform(avg_translation, avg_quat)
        if self.save_to_camera_config and self.camera_config_file:
            self.write_camera_config(avg_translation, avg_quat)

        self.done = True
        self.get_logger().info(
            f"Startup ArUco calibration complete with {len(filtered)} filtered samples."
        )
        self.publish_status(
            "DONE",
            f"Calibration complete with {len(filtered)} filtered samples.",
            sample_count=len(filtered),
            translation=avg_translation,
            quaternion=avg_quat,
        )
        self.publish_visual_state(
            state="DONE",
            details=f"Filtered samples: {len(filtered)}",
            sample_count=len(filtered),
            translation=avg_translation,
            quaternion=avg_quat,
        )
        if self.shutdown_on_complete:
            self.get_logger().info("Shutting down startup calibrator after successful calibration.")
            self.shutdown_timer = self.create_timer(0.5, self._shutdown_once)

    def _shutdown_once(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None
        if rclpy.ok():
            rclpy.shutdown()

    def publish_static_transform(self, translation, quaternion):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.camera_frame
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(quaternion[0])
        msg.transform.rotation.y = float(quaternion[1])
        msg.transform.rotation.z = float(quaternion[2])
        msg.transform.rotation.w = float(quaternion[3])
        self.broadcaster.sendTransform(msg)

    def convert_transform_to_output_frame(self, translation, quaternion):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.calibration_camera_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Failed TF lookup %s <- %s: %s",
                self.camera_frame,
                self.calibration_camera_frame,
                str(exc),
            )
            return None

        t_parent_cal = np.eye(4, dtype=np.float64)
        t_parent_cal[:3, :3] = rpy_to_matrix(*quaternion_to_rpy(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        ))
        t_parent_cal[:3, 3] = np.array(
            [float(translation[0]), float(translation[1]), float(translation[2])], dtype=np.float64
        )

        rot = tf_msg.transform.rotation
        trans = tf_msg.transform.translation
        t_output_cal = np.eye(4, dtype=np.float64)
        t_output_cal[:3, :3] = rpy_to_matrix(*quaternion_to_rpy(rot.x, rot.y, rot.z, rot.w))
        t_output_cal[:3, 3] = np.array([trans.x, trans.y, trans.z], dtype=np.float64)
        t_cal_output = np.linalg.inv(t_output_cal)

        t_parent_output = t_parent_cal @ t_cal_output
        out_translation = t_parent_output[:3, 3]
        out_quat = matrix_to_quaternion(t_parent_output[:3, :3])
        return out_translation, out_quat

    def publish_status(self, state, details, sample_count=0, translation=None, quaternion=None):
        status_msg = String()
        payload = {
            "state": state,
            "details": details,
            "sample_count": int(sample_count),
        }
        if translation is not None:
            payload["translation_xyz_m"] = [float(translation[0]), float(translation[1]), float(translation[2])]
        if quaternion is not None:
            payload["quaternion_xyzw"] = [
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ]
        status_msg.data = yaml.safe_dump(payload, sort_keys=False).strip()
        self.status_pub.publish(status_msg)

    def publish_visual_state(self, state, details, sample_count, translation, quaternion):
        now = self.get_clock().now().to_msg()
        color = {
            "WAITING_CAMERA_INFO": (1.0, 0.5, 0.0),
            "WAITING_MARKER": (1.0, 0.8, 0.0),
            "COLLECTING": (1.0, 1.0, 0.0),
            "DONE": (0.0, 1.0, 0.0),
            "FAILED": (1.0, 0.0, 0.0),
            "ERROR": (1.0, 0.0, 0.0),
        }.get(state, (0.7, 0.7, 0.7))

        text_marker = Marker()
        text_marker.header.frame_id = self.parent_frame
        text_marker.header.stamp = now
        text_marker.ns = "startup_aruco_status"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.20
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.06
        text_marker.color.a = 1.0
        text_marker.color.r = color[0]
        text_marker.color.g = color[1]
        text_marker.color.b = color[2]
        text_marker.text = f"ArUco: {state} | samples: {sample_count}\n{details}"

        marker_list = [text_marker]

        camera_marker = Marker()
        camera_marker.header.frame_id = self.parent_frame
        camera_marker.header.stamp = now
        camera_marker.ns = "startup_aruco_camera"
        camera_marker.id = 1
        if translation is None or quaternion is None:
            camera_marker.action = Marker.DELETE
        else:
            camera_marker.type = Marker.SPHERE
            camera_marker.action = Marker.ADD
            camera_marker.pose.position.x = float(translation[0])
            camera_marker.pose.position.y = float(translation[1])
            camera_marker.pose.position.z = float(translation[2])
            camera_marker.pose.orientation.x = float(quaternion[0])
            camera_marker.pose.orientation.y = float(quaternion[1])
            camera_marker.pose.orientation.z = float(quaternion[2])
            camera_marker.pose.orientation.w = float(quaternion[3])
            camera_marker.scale.x = 0.04
            camera_marker.scale.y = 0.04
            camera_marker.scale.z = 0.04
            camera_marker.color.a = 0.9
            camera_marker.color.r = color[0]
            camera_marker.color.g = color[1]
            camera_marker.color.b = color[2]
        marker_list.append(camera_marker)

        # Show configured marker target pose in parent/world chain for visual debugging.
        target_quat = matrix_to_quaternion(self.t_parent_marker[:3, :3])
        target_arrow = Marker()
        target_arrow.header.frame_id = self.parent_frame
        target_arrow.header.stamp = now
        target_arrow.ns = "startup_aruco_target"
        target_arrow.id = 2
        target_arrow.type = Marker.ARROW
        target_arrow.action = Marker.ADD
        target_arrow.pose.position.x = float(self.t_parent_marker[0, 3])
        target_arrow.pose.position.y = float(self.t_parent_marker[1, 3])
        target_arrow.pose.position.z = float(self.t_parent_marker[2, 3])
        target_arrow.pose.orientation.x = float(target_quat[0])
        target_arrow.pose.orientation.y = float(target_quat[1])
        target_arrow.pose.orientation.z = float(target_quat[2])
        target_arrow.pose.orientation.w = float(target_quat[3])
        target_arrow.scale.x = 0.12
        target_arrow.scale.y = 0.02
        target_arrow.scale.z = 0.02
        target_arrow.color.a = 1.0
        target_arrow.color.r = 0.1
        target_arrow.color.g = 0.8
        target_arrow.color.b = 1.0
        marker_list.append(target_arrow)

        # Lightweight robot-base orientation marker (no full robot model required).
        base_arrow = Marker()
        base_arrow.header.frame_id = self.parent_frame
        base_arrow.header.stamp = now
        base_arrow.ns = "startup_robot_base"
        base_arrow.id = 3
        base_arrow.type = Marker.ARROW
        base_arrow.action = Marker.ADD
        base_arrow.pose.position.x = 0.0
        base_arrow.pose.position.y = 0.0
        base_arrow.pose.position.z = 0.03
        base_arrow.pose.orientation.w = 1.0
        base_arrow.scale.x = 0.18
        base_arrow.scale.y = 0.03
        base_arrow.scale.z = 0.03
        base_arrow.color.a = 1.0
        base_arrow.color.r = 1.0
        base_arrow.color.g = 0.2
        base_arrow.color.b = 0.2
        marker_list.append(base_arrow)

        self.marker_pub.publish(MarkerArray(markers=marker_list))

    def write_camera_config(self, translation, quaternion):
        try:
            with open(self.camera_config_file, "r", encoding="utf-8") as handle:
                config = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            config = {}

        camera_cfg = config.get("camera_calibration", {})
        roll, pitch, yaw = quaternion_to_rpy(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )
        camera_cfg["parent_frame"] = self.parent_frame
        camera_cfg["frame_id"] = self.camera_frame
        camera_cfg["x"] = float(translation[0])
        camera_cfg["y"] = float(translation[1])
        camera_cfg["z"] = float(translation[2])
        camera_cfg["roll"] = float(roll)
        camera_cfg["pitch"] = float(pitch)
        camera_cfg["yaw"] = float(yaw)
        camera_cfg["calibrated_at_utc"] = datetime.now(timezone.utc).isoformat()
        config["camera_calibration"] = camera_cfg

        with open(self.camera_config_file, "w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, sort_keys=False)

        self.get_logger().info(f"Wrote calibrated transform to {self.camera_config_file}")


def main():
    rclpy.init()
    node = StartupArucoCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
