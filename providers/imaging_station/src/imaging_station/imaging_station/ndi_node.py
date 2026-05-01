"""Bridge an NDI source (e.g. ZowieBox-40435) onto a sensor_msgs/Image topic.

Architecture mirrors experiment_collector:

    rclpy.Node ── spawns ──> ndi_av_bridge (C++ subprocess from
                              imaging_station_ndi_bridge package)
        │                           │
        │  reads AVB1 packets       │  dlopens libndi.so.6, receives
        │  over Unix socket         │  NDI frames via NDIlib_recv_*
        ▼                           ▼
    sensor_msgs/Image          libndi.so.6 (NDI 6 SDK)
    /imaging_station/image_raw

Follow-up (not in this PR): an ImagePlate action backend that captures N
frames per site from this stream and writes them under output_root, replacing
the placeholder PNGs from sim_backend.
"""
from __future__ import annotations

import os
import socket
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from . import avahi_discovery, ndi_discovery
from .camera_info import resolve_camera_info
from .ndi_bridge_protocol import (
    FOURCC_BGRA,
    FOURCC_BGRX,
    PacketType,
    fourcc_to_string,
    read_packet,
)
from .ndi_runtime import build_ndi_runtime_env

_BRIDGE_PACKAGE = "imaging_station_ndi_bridge"
_BRIDGE_EXECUTABLE = "ndi_av_bridge"
_RESTART_COOLDOWN_SEC = 5.0
_MAX_RESTARTS_IN_WINDOW = 3
_RESTART_WINDOW_SEC = 30.0


def _resolve_bridge_path() -> Path:
    prefix = Path(get_package_prefix(_BRIDGE_PACKAGE))
    return prefix / "lib" / _BRIDGE_PACKAGE / _BRIDGE_EXECUTABLE


def _wait_for_socket(path: Path, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if path.exists():
            return True
        time.sleep(0.05)
    return False


class NdiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("imaging_station_ndi")

        self.declare_parameter("ndi_source_name", "ZOWIEBOX-40435 (ZowieBox-40435)")
        self.declare_parameter("url_address", "")
        self.declare_parameter("frame_id", "imaging_station_camera_optical_frame")
        self.declare_parameter("target_fps", 0.0)
        self.declare_parameter("discovery_timeout_sec", 5.0)
        self.declare_parameter("bandwidth", "highest")
        self.declare_parameter("socket_path", "")
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("recv_name", "imaging_station NDI bridge")
        self.declare_parameter("startup_timeout_sec", 12.0)

        self._source_name = self.get_parameter("ndi_source_name").value
        # Explicit "host:port" overrides discovery — useful when libndi's
        # built-in mDNS picks the wrong network interface.
        self._url_address_override = (self.get_parameter("url_address").value or "").strip()
        self._frame_id = self.get_parameter("frame_id").value
        self._target_fps = float(self.get_parameter("target_fps").value)
        self._discovery_timeout_sec = float(self.get_parameter("discovery_timeout_sec").value)
        self._bandwidth = self.get_parameter("bandwidth").value
        configured_socket = self.get_parameter("socket_path").value
        self._socket_path = Path(
            configured_socket
            if configured_socket
            else f"/tmp/imaging_station_ndi_{os.getpid()}.sock"
        )
        self._image_topic = self.get_parameter("image_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        camera_info_url = (self.get_parameter("camera_info_url").value or "").strip()
        self._camera_info_path: Path | None = Path(camera_info_url) if camera_info_url else None
        self._cached_camera_info: CameraInfo | None = None
        self._cached_camera_info_dims: tuple[int, int] | None = None
        self._recv_name = self.get_parameter("recv_name").value
        self._startup_timeout_sec = float(self.get_parameter("startup_timeout_sec").value)

        try:
            self._bridge_path = _resolve_bridge_path()
        except PackageNotFoundError as e:
            self.get_logger().fatal(
                f"imaging_station_ndi_bridge package not found: {e}. "
                "Build providers/imaging_station/src/imaging_station_ndi_bridge "
                "and source the install."
            )
            raise

        if not self._bridge_path.is_file() or not os.access(self._bridge_path, os.X_OK):
            self.get_logger().fatal(
                f"NDI bridge binary missing or not executable: {self._bridge_path}"
            )
            raise FileNotFoundError(self._bridge_path)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._image_pub = self.create_publisher(Image, self._image_topic, qos)
        # CameraInfo is light + paired 1:1 with Image; share the BE QoS so
        # subscribers like image_proc can reconcile the streams without
        # reliability-mismatch warnings.
        self._camera_info_pub = self.create_publisher(
            CameraInfo, self._camera_info_topic, qos
        )

        self._stop = threading.Event()
        self._proc: subprocess.Popen | None = None
        self._restart_history: list[float] = []

        # Resolved before each (re)spawn so a flapping source can be re-found
        # without restarting the node.
        self._resolved_url_address: str = ""

        self._worker = threading.Thread(target=self._run_loop, name="ndi-bridge", daemon=True)
        self._worker.start()

    # ------------------------------------------------------------------
    # Discovery: libndi → avahi fallback → manual override
    # ------------------------------------------------------------------
    def _resolve_url_address(self) -> str:
        """Return the host:port to pass to the bridge, or '' to let libndi do mDNS.

        Order:
          1. user-provided `url_address` parameter (always wins)
          2. libndi's built-in discovery (works on simple single-NIC hosts)
          3. avahi-browse fallback (works wherever avahi-daemon is running,
             including multi-interface hosts where libndi's mDNS gets confused)
        """
        if self._url_address_override:
            self.get_logger().info(
                f"Using configured url_address={self._url_address_override}; "
                "skipping discovery."
            )
            return self._url_address_override

        if ndi_discovery.initialize():
            timeout_ms = max(100, int(self._discovery_timeout_sec * 1000))
            sources = ndi_discovery.discover(timeout_ms=timeout_ms)
            names = [s["ndi_name"] for s in sources]
            self.get_logger().info(f"libndi visible sources ({len(names)}): {names}")
            for src in sources:
                if src["ndi_name"] == self._source_name and src.get("url_address"):
                    return src["url_address"]
        else:
            self.get_logger().warn(
                "libndi discovery unavailable — install the NDI 6 SDK if needed."
            )

        # Fallback: system mDNS via avahi-daemon. This is what saves us on
        # multi-interface dev hosts where libndi's own mDNS implementation
        # silently picks the wrong route.
        if avahi_discovery.avahi_available():
            avahi_src = avahi_discovery.find_source(
                self._source_name, timeout_sec=self._discovery_timeout_sec
            )
            if avahi_src is not None:
                self.get_logger().info(
                    f"avahi resolved {self._source_name} → "
                    f"{avahi_src.url_address} (iface={avahi_src.interface})"
                )
                return avahi_src.url_address
            all_avahi = avahi_discovery.discover(timeout_sec=1.0)
            self.get_logger().warn(
                f"avahi visible NDI sources ({len(all_avahi)}): "
                f"{[s.name for s in all_avahi]} — none matched "
                f"'{self._source_name}'."
            )
        else:
            self.get_logger().warn(
                "avahi-browse not installed; no fallback discovery available."
            )

        self.get_logger().warn(
            f"Configured source '{self._source_name}' not visible yet — "
            "bridge will wait for it via libndi mDNS."
        )
        return ""

    # ------------------------------------------------------------------
    # Bridge subprocess lifecycle
    # ------------------------------------------------------------------
    def _spawn_bridge(self) -> subprocess.Popen:
        if self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except OSError:
                pass

        # Re-resolve every time so a source that flapped off the network
        # gets re-found via avahi instead of waiting on libndi mDNS forever.
        self._resolved_url_address = self._resolve_url_address()

        cmd = [
            str(self._bridge_path),
            "--ndi-name", self._source_name,
            "--socket-path", str(self._socket_path),
            "--recv-name", self._recv_name,
            "--startup-timeout", str(self._startup_timeout_sec),
            "--color-format", "bgrx",
        ]
        if self._resolved_url_address:
            cmd += ["--url-address", self._resolved_url_address]
        if self._target_fps > 0.0:
            cmd += ["--max-fps", str(int(self._target_fps))]

        env = build_ndi_runtime_env()
        self.get_logger().info(f"Spawning NDI bridge: {' '.join(cmd)}")
        return subprocess.Popen(cmd, env=env)

    def _connect_to_bridge(self) -> socket.socket | None:
        if not _wait_for_socket(self._socket_path, timeout_sec=10.0):
            self.get_logger().error(
                f"Bridge socket {self._socket_path} did not appear within 10s"
            )
            return None
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(str(self._socket_path))
        except OSError as e:
            self.get_logger().error(f"Failed to connect to bridge socket: {e}")
            sock.close()
            return None
        return sock

    def _should_restart(self) -> bool:
        now = time.monotonic()
        self._restart_history = [
            t for t in self._restart_history if now - t < _RESTART_WINDOW_SEC
        ]
        if len(self._restart_history) >= _MAX_RESTARTS_IN_WINDOW:
            return False
        self._restart_history.append(now)
        return True

    # ------------------------------------------------------------------
    # Main worker
    # ------------------------------------------------------------------
    def _run_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._proc = self._spawn_bridge()
            except Exception as e:
                self.get_logger().error(f"Could not spawn bridge: {e}")
                time.sleep(_RESTART_COOLDOWN_SEC)
                continue

            sock = self._connect_to_bridge()
            if sock is None:
                self._terminate_proc()
                if not self._should_restart():
                    self.get_logger().error(
                        "Too many bridge failures; staying idle until ROS shutdown."
                    )
                    return
                time.sleep(_RESTART_COOLDOWN_SEC)
                continue

            try:
                self._read_and_publish(sock)
            except Exception as e:
                self.get_logger().error(f"Bridge read loop crashed: {e}")
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._terminate_proc()

            if self._stop.is_set():
                return
            if not self._should_restart():
                self.get_logger().error(
                    f"Bridge failed >{_MAX_RESTARTS_IN_WINDOW} times in "
                    f"{_RESTART_WINDOW_SEC}s; giving up."
                )
                return
            self.get_logger().warn(
                f"Bridge exited; restarting in {_RESTART_COOLDOWN_SEC}s."
            )
            time.sleep(_RESTART_COOLDOWN_SEC)

    def _read_and_publish(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            packet = read_packet(sock)
            if packet is None:
                self.get_logger().info("Bridge socket closed (EOF).")
                return

            if packet.packet_type == PacketType.VIDEO:
                self._publish_video(packet)
            elif packet.packet_type == PacketType.STATUS:
                self.get_logger().info(f"bridge: {packet.text}")
            elif packet.packet_type == PacketType.ERROR:
                self.get_logger().error(f"bridge error: {packet.text}")
                return
            elif packet.packet_type == PacketType.EOS:
                self.get_logger().info("bridge sent EOS.")
                return
            # AUDIO packets are intentionally dropped in v1.

    def _publish_video(self, packet) -> None:
        if packet.fourcc not in (FOURCC_BGRX, FOURCC_BGRA):
            self.get_logger().warn_once(
                f"Unsupported FourCC {fourcc_to_string(packet.fourcc)}; "
                "expected BGRX/BGRA. Dropping frame."
            )
            return
        if packet.width <= 0 or packet.height <= 0:
            return

        # BGRX/BGRA → bgr8: drop the trailing X/A channel without copying twice.
        # The bridge already removed line padding (stride == width * 4).
        bgrx = np.frombuffer(packet.payload, dtype=np.uint8)
        expected = packet.width * packet.height * 4
        if bgrx.size != expected:
            self.get_logger().warn(
                f"frame size mismatch: got {bgrx.size}, expected {expected}; dropping"
            )
            return
        bgrx = bgrx.reshape(packet.height, packet.width, 4)
        bgr = np.ascontiguousarray(bgrx[:, :, :3])

        msg = Image()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.height = packet.height
        msg.width = packet.width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = packet.width * 3
        msg.data = bgr.tobytes()
        self._image_pub.publish(msg)

        info = self._get_camera_info(packet.width, packet.height)
        info.header.stamp = now
        info.header.frame_id = self._frame_id
        self._camera_info_pub.publish(info)

    def _get_camera_info(self, width: int, height: int) -> CameraInfo:
        """Lazily resolve and cache the CameraInfo template.

        Recomputed if the live frame resolution diverges from the cached
        one — e.g. ZowieBox switches between 1080p and 720p modes. The
        per-call cost when the cache hits is one dict lookup; we only
        rebuild the (small, immutable) template on resolution change.
        """
        if self._cached_camera_info is None or self._cached_camera_info_dims != (width, height):
            info, source = resolve_camera_info(self._camera_info_path, width, height)
            self.get_logger().info(
                f"CameraInfo {width}x{height}: {source}"
            )
            self._cached_camera_info = info
            self._cached_camera_info_dims = (width, height)
        # Return a copy so per-frame stamp/frame_id mutations don't leak
        # into the cache (CameraInfo is a flat msg — shallow copy is enough).
        cached = self._cached_camera_info
        out = CameraInfo()
        out.height = cached.height
        out.width = cached.width
        out.distortion_model = cached.distortion_model
        out.d = list(cached.d)
        out.k = list(cached.k)
        out.r = list(cached.r)
        out.p = list(cached.p)
        out.binning_x = cached.binning_x
        out.binning_y = cached.binning_y
        out.roi = cached.roi
        return out

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _terminate_proc(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass

    def shutdown(self) -> None:
        self._stop.set()
        self._terminate_proc()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        if self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except OSError:
                pass
        ndi_discovery.shutdown()


def main() -> None:
    rclpy.init()
    node = NdiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
