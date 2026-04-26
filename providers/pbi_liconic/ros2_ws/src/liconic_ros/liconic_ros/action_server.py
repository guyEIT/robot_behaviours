"""ROS 2 action server node for a Liconic STX incubator.

One process, one ExperimentalLiconicBackend, one persistent serial
connection. Plate locations are persisted to a JSON registry so the
server's view of the carousel survives restarts.

Actions:
  ~/take_in         liconic_msgs/action/TakeIn
  ~/fetch           liconic_msgs/action/Fetch

Services:
  ~/set_temperature liconic_msgs/srv/SetTemperature
  ~/set_humidity    liconic_msgs/srv/SetHumidity
  ~/get_status      liconic_msgs/srv/GetStatus

Topics:
  ~/connected       std_msgs/Bool (latched)

Parameters (via `ros2 launch liconic_ros liconic.launch.py …`):
  port              default: /dev/liconic_stx44
  model             default: STX44_IC
  rack_model        default: liconic_rack_12mm_27
  total_cassettes   default: 2
  state_file        default: ~/.local/state/liconic/plate_registry.json
  connect_on_start  default: true
"""

from __future__ import annotations

import functools
import logging
import traceback
from pathlib import Path
from typing import Any, Callable

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy,
    QoSProfile, QoSReliabilityPolicy,
)

from std_msgs.msg import Bool

from liconic_msgs.action import Fetch, TakeIn
from liconic_msgs.msg import PlateLocation
from liconic_msgs.srv import (
    ClearCassettePosition, ClearLoadingTray, GetStatus,
    ResetError, SetHumidity, SetTemperature, StartShaking, StopShaking,
)
from robot_skills_msgs.msg import (
    KeyValue, SkillAdvertisement, SkillDescription, SkillManifest,
)

from .asyncio_bridge import AsyncioBridge
from .machine import LiconicMachine, LiconicMachineError
from .plate_registry import PlateRegistry, PlateRegistryError


DEFAULT_STATE_FILE = "~/.local/state/liconic/plate_registry.json"
# Sim mode keeps its registry well away from the real-hardware path so the
# wipe-on-startup behaviour can't destroy production state by accident.
DEFAULT_STATE_FILE_SIM = "~/.local/state/liconic/plate_registry_sim.json"


class LiconicActionServer(Node):
    def __init__(
        self,
        node_name: str = "liconic_action_server",
        **node_kwargs: Any,
    ) -> None:
        super().__init__(node_name, **node_kwargs)

        self.declare_parameter("port", "/dev/liconic_stx44")
        self.declare_parameter("model", "STX44_IC")
        self.declare_parameter("rack_model", "liconic_rack_12mm_27")
        self.declare_parameter("total_cassettes", 2)
        self.declare_parameter("state_file", DEFAULT_STATE_FILE)
        self.declare_parameter("connect_on_start", True)
        # simulation=true swaps the PLC backend for an in-memory sim (see
        # liconic_ros.sim_backend). Lets BTs exercise TakeIn/Fetch/etc.
        # without real hardware.
        self.declare_parameter("simulation", False)

        port = self._str_param("port")
        model = self._str_param("model")
        rack_model = self._str_param("rack_model")
        total_cassettes = int(
            self.get_parameter("total_cassettes").get_parameter_value().integer_value
        )
        connect_on_start = bool(
            self.get_parameter("connect_on_start").get_parameter_value().bool_value
        )
        simulation = bool(
            self.get_parameter("simulation").get_parameter_value().bool_value
        )

        # Resolve state_file. If the operator passed an explicit value it
        # wins. Otherwise default to *_sim.json under sim mode so the sim's
        # wipe-on-startup behaviour can never touch the real registry —
        # critical when both real-hardware and sim launches share a host.
        state_file_param = self._str_param("state_file")
        if state_file_param == DEFAULT_STATE_FILE and simulation:
            state_file_param = DEFAULT_STATE_FILE_SIM
        state_file = Path(state_file_param).expanduser()

        self._actions_cb_group = ReentrantCallbackGroup()
        self._services_cb_group = MutuallyExclusiveCallbackGroup()

        # In sim mode the registry is ephemeral — the sim backend doesn't
        # model persistent plate locations across server restarts, so a
        # stale entry from a previous sim run would make smoke tests fail
        # on the duplicate plate_name check. Wipe only the sim file.
        if simulation and state_file.exists():
            self.get_logger().info(
                f"simulation=true — clearing existing sim plate registry at {state_file}"
            )
            state_file.unlink()

        self._registry = PlateRegistry(state_file)
        self.get_logger().info(
            f"plate registry: {self._registry.path} "
            f"({len(self._registry.all_cassette_plates())} stored, "
            f"tray={'occupied' if self._registry.loading_tray() else 'empty'})"
        )

        self._machine = LiconicMachine(
            port=port, model=model, rack_model=rack_model,
            total_cassettes=total_cassettes, simulation=simulation,
        )
        if simulation:
            self.get_logger().info(
                "simulation=true — using LiconicSimBackend (no PLC, no serial)"
            )

        self._bridge = AsyncioBridge()
        self._bridge.start()

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._connected_pub = self.create_publisher(Bool, "~/connected", latched_qos)
        self._publish_connected(False)

        self._register_services()
        self._register_actions()
        self._publish_skill_manifest()

        if connect_on_start:
            self._bridge.submit(self._setup_backend())
        else:
            self.get_logger().info(
                "connect_on_start=false — call /set_temperature or an action "
                "to lazy-connect (the backend still connects on first actual use)."
            )

        self.get_logger().info(
            f"liconic_action_server ready (port={port}, model={model}, "
            f"{total_cassettes}x{self._machine.positions_per_cassette} sites)"
        )

    # ---- small helpers ----

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).get_parameter_value().string_value)

    def _publish_connected(self, connected: bool) -> None:
        msg = Bool()
        msg.data = bool(connected)
        self._connected_pub.publish(msg)

    async def _setup_backend(self) -> None:
        try:
            await self._machine.setup()
            self._publish_connected(True)
            self.get_logger().info("Liconic setup complete")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Liconic setup failed: {exc}")
            self._publish_connected(False)

    # ---- service registration ----

    def _register_services(self) -> None:
        self.create_service(
            SetTemperature, "~/set_temperature", self._handle_set_temperature,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            SetHumidity, "~/set_humidity", self._handle_set_humidity,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            GetStatus, "~/get_status", self._handle_get_status,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            ClearLoadingTray, "~/clear_loading_tray",
            self._handle_clear_loading_tray,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            ClearCassettePosition, "~/clear_cassette_position",
            self._handle_clear_cassette_position,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            StartShaking, "~/start_shaking", self._handle_start_shaking,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            StopShaking, "~/stop_shaking", self._handle_stop_shaking,
            callback_group=self._services_cb_group,
        )
        self.create_service(
            ResetError, "~/reset_error", self._handle_reset_error,
            callback_group=self._services_cb_group,
        )

    def _handle_set_temperature(self, request, response):
        try:
            self._bridge.run(
                lambda: self._machine.set_temperature(float(request.temperature_c)),
            )
            response.success = True
            response.message = f"target temperature set to {request.temperature_c:.2f} C"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"set_temperature failed: {exc}")
        return response

    def _handle_set_humidity(self, request, response):
        try:
            self._bridge.run(
                lambda: self._machine.set_humidity(float(request.humidity_fraction)),
            )
            response.success = True
            response.message = (
                f"target humidity set to {request.humidity_fraction * 100:.1f} %"
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"set_humidity failed: {exc}")
        return response

    def _handle_start_shaking(self, request, response):
        try:
            self._bridge.run(
                lambda: self._machine.start_shaking(float(request.frequency_hz)),
            )
            response.success = True
            response.message = f"shaker started at {request.frequency_hz:.1f} Hz"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"start_shaking failed: {exc}")
        return response

    def _handle_stop_shaking(self, request, response):
        try:
            self._bridge.run(lambda: self._machine.stop_shaking())
            response.success = True
            response.message = "shaker stopped"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"stop_shaking failed: {exc}")
        return response

    def _handle_reset_error(self, request, response):
        try:
            before = self._bridge.run(lambda: self._machine.reset_error())
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            response.error_flag_before = ""
            response.error_code_before = ""
            self.get_logger().error(f"reset_error failed: {exc}")
            return response
        response.success = True
        response.error_flag_before = before.get("flag_before", "")
        response.error_code_before = before.get("code_before", "")
        if response.error_flag_before == "1":
            response.message = (
                f"reset ok — cleared latched error code "
                f"{response.error_code_before!r}"
            )
            self.get_logger().info(
                f"reset_error: cleared error code {response.error_code_before}"
            )
        else:
            response.message = "reset ok — no latched error was present"
        return response

    def _handle_clear_cassette_position(self, request, response):
        cassette = int(request.cassette)
        position = int(request.position)
        if cassette < 1 or position < 1:
            response.success = False
            response.message = (
                f"cassette and position must be >= 1 (got {cassette}, {position})"
            )
            response.cleared_plate_name = ""
            return response
        try:
            rec = self._registry.clear_cassette_position(cassette, position)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            response.cleared_plate_name = ""
            self.get_logger().error(f"clear_cassette_position failed: {exc}")
            return response
        response.success = True
        if rec is None:
            response.message = (
                f"cassette {cassette} position {position} was already empty"
            )
            response.cleared_plate_name = ""
        else:
            response.message = (
                f"cleared {rec.plate_name!r} from cassette {cassette} "
                f"position {position}"
            )
            response.cleared_plate_name = rec.plate_name
            self.get_logger().info(
                f"registry: cleared {rec.plate_name!r} from cassette "
                f"{cassette} position {position}"
            )
        return response

    def _handle_clear_loading_tray(self, request, response):
        try:
            prev = self._registry.clear_loading_tray()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            response.cleared_plate_name = ""
            self.get_logger().error(f"clear_loading_tray failed: {exc}")
            return response
        response.success = True
        if prev is None:
            response.message = "loading tray was already empty"
            response.cleared_plate_name = ""
        else:
            response.message = f"cleared {prev.plate_name!r} from loading tray"
            response.cleared_plate_name = prev.plate_name
            self.get_logger().info(
                f"registry: cleared {prev.plate_name!r} from loading tray"
            )
        return response

    def _handle_get_status(self, request, response):
        response.connected = self._machine.connected
        response.model = self._machine.model
        response.climate_supported = self._machine.climate_supported
        try:
            climate = self._bridge.run(lambda: self._machine.read_climate())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"read_climate failed: {exc}")
            nan = float("nan")
            climate = dict(target_t=nan, actual_t=nan, target_h=nan, actual_h=nan)
        response.target_temperature_c = float(climate["target_t"])
        response.actual_temperature_c = float(climate["actual_t"])
        response.target_humidity = float(climate["target_h"])
        response.actual_humidity = float(climate["actual_h"])

        response.plates = [
            _plate_to_msg(rec) for rec in self._registry.all_cassette_plates()
        ]
        tray = self._registry.loading_tray()
        response.loading_tray_plate = (
            _plate_to_msg(tray) if tray is not None else PlateLocation()
        )

        # Error state + shaker frequency — best-effort reads. If the
        # device isn't connected or the read fails, we return blanks /
        # NaN rather than failing the whole status call.
        try:
            err = self._bridge.run(lambda: self._machine.read_error())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"read_error failed: {exc}")
            err = {"flag": "", "code": ""}
        response.error_flag = err.get("flag", "")
        response.error_code = err.get("code", "")

        try:
            response.shaker_frequency_hz = float(
                self._bridge.run(lambda: self._machine.get_shaker_frequency())
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"get_shaker_frequency failed: {exc}")
            response.shaker_frequency_hz = float("nan")

        return response

    # ---- skill advertisement ----

    def _publish_skill_manifest(self) -> None:
        """Publish a latched SkillManifest on `~/skills` so SkillDiscovery
        can pick up the Liconic actions without any hardcoded entry in
        tree_executor.ACTION_REGISTRY. QoS must match
        `robot_skill_server.skill_advertiser.make_skills_qos()`.
        """
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=10.0),
        )
        self._skills_pub = self.create_publisher(SkillManifest, "~/skills", qos)

        ns = self.get_namespace().rstrip("/")
        node_path = f"{ns}/{self.get_name()}" if ns else f"/{self.get_name()}"

        ads: list[SkillAdvertisement] = []

        ad_take_in = SkillAdvertisement()
        ad_take_in.description = SkillDescription(
            name="liconic_take_in",
            display_name="Liconic Take In",
            description="Move a plate from the loading tray into a cassette slot.",
            version="1.0.0",
            robot_id="liconic",
            category="manipulation",
            tags=["liconic", "incubator"],
            action_server_name=f"{node_path}/take_in",
            action_type="liconic_msgs/action/TakeIn",
        )
        ad_take_in.bt_tag = "LiconicTakeIn"
        ad_take_in.goal_defaults = [KeyValue(key="barcode", value="")]
        ads.append(ad_take_in)

        ad_fetch = SkillAdvertisement()
        ad_fetch.description = SkillDescription(
            name="liconic_fetch",
            display_name="Liconic Fetch",
            description="Retrieve a plate by name (or by cassette/position) onto the loading tray.",
            version="1.0.0",
            robot_id="liconic",
            category="manipulation",
            tags=["liconic", "incubator"],
            action_server_name=f"{node_path}/fetch",
            action_type="liconic_msgs/action/Fetch",
        )
        ad_fetch.bt_tag = "LiconicFetch"
        ad_fetch.goal_defaults = [
            KeyValue(key="plate_name", value=""),
            KeyValue(key="cassette", value="0"),
            KeyValue(key="position", value="0"),
        ]
        ad_fetch.output_renames = [KeyValue(key="plate_name", value="plate_name_out")]
        ads.append(ad_fetch)

        msg = SkillManifest()
        msg.skills = ads
        msg.published_at = self.get_clock().now().to_msg()
        msg.source_node = node_path
        self._skills_pub.publish(msg)

    # ---- action registration ----

    def _register_actions(self) -> None:
        self._take_in_action = ActionServer(
            self,
            TakeIn, "~/take_in",
            execute_callback=self._execute_take_in,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._actions_cb_group,
        )
        self._fetch_action = ActionServer(
            self,
            Fetch, "~/fetch",
            execute_callback=self._execute_fetch,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._actions_cb_group,
        )

    def _execute_take_in(self, goal_handle):
        goal = goal_handle.request
        result = TakeIn.Result()
        cassette = int(goal.cassette)
        position = int(goal.position)
        plate_name = str(goal.plate_name).strip()
        barcode = str(goal.barcode)

        if not plate_name:
            return self._fail(goal_handle, TakeIn, result, "plate_name is required")
        if cassette < 1 or position < 1:
            return self._fail(
                goal_handle, TakeIn, result,
                f"cassette and position must be >= 1 (got {cassette}, {position})",
            )
        if self._registry.find_by_name(plate_name) is not None:
            return self._fail(
                goal_handle, TakeIn, result,
                f"plate_name {plate_name!r} is already registered",
            )
        if self._registry.at(cassette, position) is not None:
            return self._fail(
                goal_handle, TakeIn, result,
                f"cassette {cassette} position {position} is already occupied",
            )

        self._publish_feedback(goal_handle, TakeIn, "dispatching")
        try:
            self._bridge.run(
                lambda: self._machine.take_in_plate(
                    cassette=cassette, position=position, barcode=barcode,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"take_in failed: {exc}\n{traceback.format_exc()}"
            )
            return self._fail(
                goal_handle, TakeIn, result, f"{type(exc).__name__}: {exc}"
            )

        try:
            self._registry.place_in_cassette(
                plate_name=plate_name, cassette=cassette,
                position=position, barcode=barcode,
            )
        except PlateRegistryError as exc:
            # Motion succeeded but registry rejected — surface it so an
            # operator reconciles. The plate is physically in the
            # cassette; the registry does not reflect it.
            self.get_logger().error(
                f"take_in motion ok but registry rejected: {exc}"
            )
            return self._fail(
                goal_handle, TakeIn, result,
                f"motion ok but registry update failed: {exc}. "
                "Plate is physically in cassette; registry out of sync.",
            )

        self._publish_feedback(goal_handle, TakeIn, "done")
        goal_handle.succeed()
        result.success = True
        result.message = (
            f"plate {plate_name!r} taken in to cassette {cassette} "
            f"position {position}"
        )
        return result

    def _execute_fetch(self, goal_handle):
        goal = goal_handle.request
        result = Fetch.Result()
        plate_name = str(goal.plate_name).strip()
        cassette = int(goal.cassette)
        position = int(goal.position)

        if plate_name:
            rec = self._registry.find_by_name(plate_name)
            if rec is None:
                return self._fail(
                    goal_handle, Fetch, result,
                    f"plate_name {plate_name!r} not found in registry",
                )
            if rec.cassette == 0:
                return self._fail(
                    goal_handle, Fetch, result,
                    f"plate {plate_name!r} is already on the loading tray",
                )
            cassette, position = rec.cassette, rec.position
        else:
            if cassette < 1 or position < 1:
                return self._fail(
                    goal_handle, Fetch, result,
                    f"cassette and position must be >= 1 when plate_name is empty "
                    f"(got {cassette}, {position})",
                )
            rec = self._registry.at(cassette, position)
            if rec is None:
                return self._fail(
                    goal_handle, Fetch, result,
                    f"no plate registered at cassette {cassette} position {position}",
                )
            plate_name = rec.plate_name

        if self._registry.loading_tray() is not None:
            return self._fail(
                goal_handle, Fetch, result,
                f"loading tray is already occupied by plate "
                f"{self._registry.loading_tray().plate_name!r}",
            )

        self._publish_feedback(goal_handle, Fetch, "dispatching")
        try:
            self._bridge.run(
                lambda: self._machine.fetch_plate(
                    cassette=cassette, position=position,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"fetch failed: {exc}\n{traceback.format_exc()}"
            )
            return self._fail(
                goal_handle, Fetch, result, f"{type(exc).__name__}: {exc}"
            )

        try:
            self._registry.move_cassette_to_tray(cassette=cassette, position=position)
        except PlateRegistryError as exc:
            self.get_logger().error(
                f"fetch motion ok but registry rejected: {exc}"
            )
            return self._fail(
                goal_handle, Fetch, result,
                f"motion ok but registry update failed: {exc}. "
                "Plate is physically on tray; registry out of sync.",
            )

        self._publish_feedback(goal_handle, Fetch, "done")
        goal_handle.succeed()
        result.success = True
        result.message = (
            f"plate {plate_name!r} fetched from cassette {cassette} "
            f"position {position} to loading tray"
        )
        result.plate_name = plate_name
        return result

    # ---- action utilities ----

    def _publish_feedback(self, goal_handle, action_type, stage: str) -> None:
        fb = action_type.Feedback()
        fb.stage = stage
        goal_handle.publish_feedback(fb)

    def _fail(self, goal_handle, action_type, result, message: str):
        self._publish_feedback(goal_handle, action_type, "failed")
        goal_handle.abort()
        result.success = False
        result.message = message
        return result

    # ---- shutdown ----

    def destroy_node(self) -> bool:
        try:
            self._bridge.run(lambda: self._machine.stop())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"machine.stop() raised during shutdown: {exc}")
        self._bridge.stop()
        return super().destroy_node()


def _plate_to_msg(rec) -> PlateLocation:
    msg = PlateLocation()
    msg.plate_name = rec.plate_name
    msg.barcode = rec.barcode
    msg.cassette = int(rec.cassette)
    msg.position = int(rec.position)
    msg.placed_at = rec.placed_at
    return msg


def main(args=None) -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = LiconicActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
