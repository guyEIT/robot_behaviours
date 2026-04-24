"""ROS 2 action server node for the Hamilton STAR.

One process, one ``LiquidHandler``, one persistent USB connection. Actions
and services are dispatched to a dedicated asyncio loop thread; the FSM
gates acceptance and the RW-lock serializes access at per-channel /
per-resource granularity.
"""

from __future__ import annotations

import asyncio
import functools
from typing import Any, Awaitable, Callable, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_srvs.srv import Trigger

from pylabrobot.liquid_handling.backends.hamilton.STAR_chatterbox import (
    STARChatterboxBackend,
)

from hamilton_star_msgs.action import (
    Aspirate, Aspirate96,
    Dispense, Dispense96,
    DropTips, DropTips96,
    HandoffTransfer,
    JogChannel,
    MoveResource,
    PickUpCoreGripper, PickUpTips, PickUpTips96,
    ReturnCoreGripper,
    Transfer,
)
from hamilton_star_msgs.msg import ChannelState as ChannelStateMsg
from hamilton_star_msgs.msg import Event as EventMsg
from hamilton_star_msgs.msg import Handoff as HandoffMsg
from hamilton_star_msgs.msg import Status as StatusMsg
from hamilton_star_msgs.srv import (
    DefineHandoff, DeleteHandoff, GetStatus, InitializeModule, ListHandoffs,
    ListResources, LoadDeck, ResetError, SerializeDeck,
)

from .asyncio_bridge import AsyncioBridge
from . import iswap_handoff
from .handoff_registry import HandoffRecord, HandoffRegistry
from .iswap_handoff import HandoffCalibration
from .machine import Machine
from .machine_fsm import ActionKind, ChannelState, FSMEvent, MachineFSM
from .machine_lock import MachineLock


# Seeded from iswap_calibrations.json (2026-04-22) so launch works out of
# the box. Override any value at runtime via ``ros2 param set
# /hamilton_star_action_server handoff.<name>.<field>``.
_HANDOFF_DEFAULTS: dict[str, dict[str, Any]] = {
    "incubator_handoff": dict(
        x=-216.1, y=170.6, z=123.6,
        plate_width=80.0, rotation="LEFT", wrist="STRAIGHT",
    ),
}


def _infer_channels(
    explicit: list[int],
    resource_count: int,
    num_channels: int,
) -> list[int]:
    """Replicate pylabrobot's auto-channel-selection for pre-lock FSM checks."""
    if explicit:
        return list(explicit)
    return list(range(min(resource_count, num_channels)))


class HamiltonStarActionServer(Node):
    def __init__(
        self,
        node_name: str = "hamilton_star_action_server",
        **node_kwargs: Any,
    ) -> None:
        super().__init__(node_name, **node_kwargs)

        self.declare_parameter("deck_file", "")
        self.declare_parameter("core_grippers", "1000uL-5mL-on-waste")
        self.declare_parameter("on_conflict", "reject")  # "reject" | "queue"
        self.declare_parameter("num_channels", 8)
        self.declare_parameter("disconnect_on_shutdown", False)
        # backend: "star" (real USB) or "simulator" (STARChatterboxBackend)
        self.declare_parameter("backend", "star")

        # Handoff registry — runtime-mutable CRUD for off-deck sites.
        # Seeded here from (1) the legacy ``handoff.<name>.*`` ROS params
        # (so handoffs.yaml keeps working) and (2) ``_HANDOFF_DEFAULTS``
        # as a fallback. Operators add/modify at runtime via
        # ``~/define_handoff``; no node restart required.
        self._handoff_registry = HandoffRegistry()

        self.declare_parameter("handoff_names", ["incubator_handoff"])
        self._handoff_names: list[str] = list(
            self.get_parameter("handoff_names").get_parameter_value().string_array_value
            or ["incubator_handoff"]
        )
        # Declare the legacy per-handoff params for back-compat with any
        # existing ``handoffs.yaml`` that sets them. We read them right
        # after declaration to seed the registry, then never touch the
        # params again (registry is the source of truth).
        for _name in self._handoff_names:
            d = _HANDOFF_DEFAULTS.get(_name, dict(
                x=0.0, y=0.0, z=0.0,
                plate_width=80.0, rotation="LEFT", wrist="STRAIGHT",
            ))
            self.declare_parameter(f"handoff.{_name}.x", float(d["x"]))
            self.declare_parameter(f"handoff.{_name}.y", float(d["y"]))
            self.declare_parameter(f"handoff.{_name}.z", float(d["z"]))
            self.declare_parameter(f"handoff.{_name}.plate_width", float(d["plate_width"]))
            self.declare_parameter(f"handoff.{_name}.rotation", str(d["rotation"]))
            self.declare_parameter(f"handoff.{_name}.wrist", str(d["wrist"]))
            # Seed the registry from the (possibly yaml-overridden) params.
            self._handoff_registry.set(HandoffRecord(
                name=_name,
                x=float(self.get_parameter(f"handoff.{_name}.x").value),
                y=float(self.get_parameter(f"handoff.{_name}.y").value),
                z=float(self.get_parameter(f"handoff.{_name}.z").value),
                plate_width=float(
                    self.get_parameter(f"handoff.{_name}.plate_width").value,
                ),
                rotation=str(self.get_parameter(f"handoff.{_name}.rotation").value),
                wrist=str(self.get_parameter(f"handoff.{_name}.wrist").value),
                # hotel_depth / hotel_clearance_height / grip_direction
                # aren't in the legacy param set; they default from the
                # dataclass and can be overridden at runtime via
                # /define_handoff with replace_existing=True.
            ))

        self._num_channels: int = int(
            self.get_parameter("num_channels").get_parameter_value().integer_value or 8
        )
        self._on_conflict: str = str(
            self.get_parameter("on_conflict").get_parameter_value().string_value or "reject"
        )

        self._actions_cb_group = ReentrantCallbackGroup()
        self._services_cb_group = MutuallyExclusiveCallbackGroup()

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._status_pub = self.create_publisher(StatusMsg, "~/status", latched_qos)
        self._events_pub = self.create_publisher(EventMsg, "~/events", 50)

        self._bridge = AsyncioBridge()
        self._bridge.start()

        deck_file = (
            self.get_parameter("deck_file").get_parameter_value().string_value or None
        )
        core_grippers = (
            self.get_parameter("core_grippers").get_parameter_value().string_value
            or "1000uL-5mL-on-waste"
        )
        backend_kind = (
            self.get_parameter("backend").get_parameter_value().string_value or "star"
        )
        self._machine = Machine(
            core_grippers=core_grippers,
            deck_file=deck_file,
            backend=backend_kind,
            num_channels=self._num_channels,
        )

        self._fsm = MachineFSM(
            num_channels=self._num_channels,
            event_sink=self._emit_event,
        )
        self._lock = MachineLock(num_channels=self._num_channels)

        if deck_file:
            self._fsm.mark_deck_loaded(self._machine.deck_hash)

        self._fsm.connect()
        self._bridge.submit(self._setup_and_arm())

        self._status_timer = self.create_timer(
            1.0, self._publish_status, callback_group=self._services_cb_group,
        )

        self._register_services()
        self._register_actions()

        self.get_logger().info(
            f"hamilton_star_action_server ready (backend={backend_kind}, connecting…)"
        )

    async def _setup_and_arm(self) -> None:
        try:
            await self._machine.setup()
            # Move the 96-head to its park position so it doesn't hang
            # over the deck after setup. Silently skipped on the
            # simulator (chatterbox doesn't model head96_park). The
            # firmware ``H0 MO`` command uses default speeds /
            # accelerations. Safe to call on hardware without a 96-head
            # installed — ``_requires_head96`` will raise and we log.
            try:
                await self._machine.backend.head96_park()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().info(
                    f"head96_park skipped ({type(exc).__name__}): {exc}"
                )
            # Inject handoffs that were seeded into the registry during
            # __init__ (legacy param seeding). Must happen before the
            # FSM goes Idle so goal-callbacks see the full deck tree.
            for record in self._handoff_registry.all():
                try:
                    self._machine.assign_handoff_holder(
                        name=record.name,
                        x=record.x, y=record.y, z=record.z,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"seed-time inject of handoff {record.name!r} "
                        f"failed: {exc}"
                    )
            self._fsm.finish_init(ok=True)
            # lh.setup() internally initializes every module including the
            # iSWAP — record that in the FSM so HandoffTransfer can gate
            # on it. Y-path-clear stays False; the first handoff goal runs
            # C0 FY on demand.
            self._fsm.mark_iswap_initialized(True)
            # Fresh setup() leaves the iSWAP at its post-init pose
            # (Y≈626 on this STAR), which is NOT safe for rotations
            # — rotating from there swings into the back wall with
            # "R002/82 Wrist twist drive movement error". Mark not-
            # safe so the first iSWAP-using action parks at SAFE_Y
            # first. (Opposite of what you'd naively expect; the
            # comment-name-vs-reality gap bit us already.)
            self._fsm.mark_iswap_at_safe_pose(False)
            if not self._fsm.deck_loaded:
                self._fsm.mark_deck_loaded(self._machine.deck_hash)
            self.get_logger().info("STAR setup complete; FSM -> Idle")
        except Exception as exc:  # noqa: BLE001
            self._fsm.finish_init(ok=False, reason=f"setup failed: {exc}")
            self.get_logger().error(f"setup failed: {exc}")

    # ---- event sink / status publisher ----
    def _emit_event(self, event: FSMEvent) -> None:
        msg = EventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.severity = int(event.severity)
        msg.kind = event.kind
        msg.message = event.message
        msg.keys = list(event.details.keys())
        msg.values = [str(v) for v in event.details.values()]
        self._events_pub.publish(msg)

    def _channel_state_to_msg(self, ch: int, st: ChannelState) -> ChannelStateMsg:
        m = ChannelStateMsg()
        m.channel = int(ch)
        if st == ChannelState.FREE:
            m.state = ChannelStateMsg.FREE
        elif st == ChannelState.HAS_TIP:
            m.state = ChannelStateMsg.HAS_TIP
        else:
            m.state = ChannelStateMsg.HOLDS_GRIPPER
        return m

    def _build_status(self) -> StatusMsg:
        snap = self._fsm.snapshot()
        msg = StatusMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.op_state = snap["op_state"]
        msg.connected = snap["connected"]
        msg.initialized = snap["initialized"]
        msg.error_reason = snap["error_reason"]
        msg.deck_loaded = snap["deck_loaded"]
        msg.deck_hash = snap["deck_hash"]
        msg.num_channels = snap["num_channels"]
        msg.channels = [
            self._channel_state_to_msg(ch, st) for ch, st in snap["channels"].items()
        ]
        msg.busy_channels = [int(ch) for ch in snap["busy_channels"]]
        msg.core_parked = snap["core_parked"]
        msg.core_channels = [int(ch) for ch in snap["core_channels"]]
        msg.core96_loaded = snap["core96_loaded"]
        msg.exclusive_op = snap["exclusive_op"]
        msg.shared_goal_count = snap["shared_goal_count"]
        msg.iswap_initialized = bool(snap["iswap_initialized"])
        msg.iswap_y_path_clear = bool(snap["iswap_y_path_clear"])
        msg.iswap_holding_plate = bool(snap["iswap_holding_plate"])
        msg.iswap_drive_faulted = bool(snap["iswap_drive_faulted"])
        msg.iswap_at_safe_pose = bool(snap["iswap_at_safe_pose"])
        msg.last_handoff_stage_reached = int(snap["last_handoff_stage_reached"])
        return msg

    def _publish_status(self) -> None:
        self._status_pub.publish(self._build_status())

    # ---- services ----
    def _register_services(self) -> None:
        mk = functools.partial(
            self.create_service, callback_group=self._services_cb_group,
        )
        mk(Trigger, "~/abort_motion", self._srv_abort_motion)
        mk(ResetError, "~/reset_error", self._srv_reset_error)
        mk(Trigger, "~/acknowledge_plate_released",
           self._srv_acknowledge_plate_released)
        mk(InitializeModule, "~/initialize_module", self._srv_initialize_module)
        mk(LoadDeck, "~/load_deck", self._srv_load_deck)
        mk(SerializeDeck, "~/serialize_deck", self._srv_serialize_deck)
        mk(GetStatus, "~/get_status", self._srv_get_status)
        mk(ListResources, "~/list_resources", self._srv_list_resources)
        mk(DefineHandoff, "~/define_handoff", self._srv_define_handoff)
        mk(ListHandoffs, "~/list_handoffs", self._srv_list_handoffs)
        mk(DeleteHandoff, "~/delete_handoff", self._srv_delete_handoff)

    def _srv_abort_motion(self, _req: Trigger.Request, resp: Trigger.Response):
        gate = self._fsm.begin_abort()
        if not gate.ok:
            resp.success = False
            resp.message = gate.reason
            return resp
        try:
            self._bridge.run(self._machine.abort_motion)
            self._fsm.finish_abort(ok=True)
            resp.success = True
            resp.message = "abort complete"
        except Exception as exc:  # noqa: BLE001
            self._fsm.finish_abort(ok=False, reason=str(exc))
            resp.success = False
            resp.message = f"abort failed: {exc}"
        return resp

    def _srv_acknowledge_plate_released(
        self, _req: Trigger.Request, resp: Trigger.Response,
    ):
        """Operator confirms the iSWAP gripper is empty after a handoff
        failure. Clears ``iswap_holding_plate`` so the next handoff is
        allowed past the FSM gate. Intended to follow physical
        inspection (or a ``pixi run iswap-release`` recovery)."""
        if not self._fsm.iswap_holding_plate:
            resp.success = False
            resp.message = "no plate-held state to acknowledge"
            return resp
        self._fsm.mark_iswap_holding_plate(False)
        self._emit_event(FSMEvent(
            kind="plate_released_ack",
            severity=1,
            message="operator acknowledged plate released",
            details={
                "last_handoff_stage_reached": str(
                    self._fsm.last_handoff_stage_reached,
                ),
            },
        ))
        resp.success = True
        resp.message = "plate-held flag cleared"
        return resp

    def _srv_reset_error(self, req: ResetError.Request, resp: ResetError.Response):
        # Short-circuit when not in Error — preserves the legacy reject
        # behaviour and keeps existing tests passing.
        if self._fsm.state != "Error":
            gate = self._fsm.reset_error(req.acknowledgment)
            resp.success = gate.ok
            resp.message = gate.reason if not gate.ok else "error cleared"
            return resp

        if not (req.acknowledgment or "").strip():
            resp.success = False
            resp.message = "acknowledgment must be non-empty"
            return resp

        # Soft firmware recovery: re-init the iSWAP if needed, then
        # prove it's responsive with a position query. Skipped when the
        # simulator backend is in use — the chatterbox doesn't model
        # these firmware responses, and there's nothing to recover.
        if not isinstance(self._machine.backend, STARChatterboxBackend):
            try:
                self._bridge.run(
                    lambda: iswap_handoff.ensure_iswap_initialized(
                        self._machine.backend,
                    ),
                )
                self._bridge.run(
                    lambda: self._machine.backend.request_iswap_position(),
                )
            except Exception as exc:  # noqa: BLE001
                resp.success = False
                resp.message = (
                    f"soft recovery failed ({exc}); iSWAP still faulted "
                    "— restart the action server or power-cycle the STAR"
                )
                # Leave iswap_drive_faulted True so subsequent goals are
                # rejected at the gate until a human intervenes.
                return resp

        gate = self._fsm.reset_error(req.acknowledgment)
        if not gate.ok:
            resp.success = False
            resp.message = gate.reason
            return resp
        self._fsm.mark_iswap_initialized(True)
        self._fsm.mark_iswap_drive_faulted(False)
        # FI leaves the arm at the post-init Y, NOT our SAFE_Y — the
        # next iSWAP op will re-park. Setting False here is correct.
        self._fsm.mark_iswap_at_safe_pose(False)
        # iswap_holding_plate deliberately preserved — operator must
        # call /acknowledge_plate_released after physical inspection.
        resp.success = True
        resp.message = "error cleared (iSWAP re-initialized)"
        return resp

    def _srv_initialize_module(
        self, req: InitializeModule.Request, resp: InitializeModule.Response,
    ):
        gate = self._fsm.can_accept(ActionKind.INIT_MODULE, {})
        if not gate.ok:
            resp.success = False
            resp.message = gate.reason
            return resp
        self._fsm.on_exclusive_accepted(ActionKind.INIT_MODULE)
        try:
            self._bridge.run(lambda: self._machine.initialize_module(req.module))
            if req.module.lower() == "iswap":
                # Fresh iSWAP init resets the Y-path promise too (the
                # mutator clears it) and flips the initialized flag on.
                self._fsm.mark_iswap_initialized(True)
                # FI clears drive faults, but parks at Y≈626 — NOT
                # safe for rotations on this STAR. The next iSWAP op
                # re-parks at SAFE_Y via the pre-flight.
                self._fsm.mark_iswap_drive_faulted(False)
                self._fsm.mark_iswap_at_safe_pose(False)
            resp.success = True
            resp.message = f"{req.module} initialized"
        except Exception as exc:  # noqa: BLE001
            # If the failed module is the iSWAP, the drive may be in a
            # faulted state that only soft recovery (reset_error) will
            # clear.
            is_iswap = req.module.lower() == "iswap"
            self._fsm.mark_error_and_invalidate(
                f"initialize_module({req.module}) failed: {exc}",
                mark_iswap_faulted=is_iswap,
            )
            resp.success = False
            resp.message = str(exc)
        finally:
            self._fsm.on_exclusive_finished(ActionKind.INIT_MODULE)
        return resp

    def _srv_load_deck(self, req: LoadDeck.Request, resp: LoadDeck.Response):
        gate = self._fsm.can_accept(ActionKind.LOAD_DECK, {})
        if not gate.ok:
            resp.success = False
            resp.message = gate.reason
            return resp
        if not self._fsm.core_parked:
            resp.success = False
            resp.message = "CoRe gripper is not parked; cannot swap deck"
            return resp
        self._fsm.on_exclusive_accepted(ActionKind.LOAD_DECK)
        try:
            deck_hash = self._bridge.run(
                lambda: self._machine.load_deck_from_file(req.deck_file),
            )
            # A fresh deck wipes whatever ResourceHolders we had
            # injected for registered handoffs — re-inject them so
            # callers keep their off-deck-site-as-deck-resource view.
            self._reinject_registered_handoffs()
            # deck_hash is now stale because we just mutated the deck;
            # recompute from the machine.
            deck_hash = self._machine.deck_hash
            self._fsm.mark_deck_loaded(deck_hash)
            resp.success = True
            resp.message = "deck loaded"
            resp.deck_hash = deck_hash
        except Exception as exc:  # noqa: BLE001
            self._fsm.mark_error(f"load_deck failed: {exc}")
            resp.success = False
            resp.message = str(exc)
        finally:
            self._fsm.on_exclusive_finished(ActionKind.LOAD_DECK)
        return resp

    def _reinject_registered_handoffs(self) -> None:
        """Attach a ResourceHolder for every registered handoff onto
        the current deck. Called after startup setup and after
        LoadDeck (which replaces the deck wholesale)."""
        for record in self._handoff_registry.all():
            try:
                self._bridge.submit(
                    self._assign_handoff_holder_coro(record),
                ).result(timeout=5.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"re-injecting handoff {record.name!r} failed: {exc}"
                )

    def _srv_serialize_deck(
        self, _req: SerializeDeck.Request, resp: SerializeDeck.Response,
    ):
        try:
            resp.deck_json = self._machine.serialize_deck_json()
            resp.success = True
            resp.message = ""
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = str(exc)
            resp.deck_json = ""
        return resp

    def _srv_get_status(self, _req: GetStatus.Request, resp: GetStatus.Response):
        resp.status = self._build_status()
        return resp

    def _srv_list_resources(
        self, _req: ListResources.Request, resp: ListResources.Response,
    ):
        try:
            items = self._machine.list_resources()
            resp.names = [n for n, _ in items]
            resp.categories = [c for _, c in items]
            resp.success = True
            resp.message = ""
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = str(exc)
        return resp

    # ---- handoff registry services ----
    def _handoff_msg_from_record(self, r: HandoffRecord) -> HandoffMsg:
        m = HandoffMsg()
        m.name = r.name
        m.x = float(r.x)
        m.y = float(r.y)
        m.z = float(r.z)
        m.plate_width = float(r.plate_width)
        m.rotation = r.rotation
        m.wrist = r.wrist
        m.grip_direction = r.grip_direction
        m.hotel_depth = float(r.hotel_depth)
        m.hotel_clearance_height = float(r.hotel_clearance_height)
        return m

    def _srv_define_handoff(
        self, req: DefineHandoff.Request, resp: DefineHandoff.Response,
    ):
        if not (req.handoff.name or "").strip():
            resp.success = False
            resp.message = "handoff.name is required"
            return resp
        if req.handoff.plate_width <= 0:
            resp.success = False
            resp.message = "handoff.plate_width must be > 0"
            return resp
        record = HandoffRecord(
            name=req.handoff.name,
            x=float(req.handoff.x),
            y=float(req.handoff.y),
            z=float(req.handoff.z),
            plate_width=float(req.handoff.plate_width),
            rotation=req.handoff.rotation or "LEFT",
            wrist=req.handoff.wrist or "STRAIGHT",
            grip_direction=req.handoff.grip_direction or "FRONT",
            hotel_depth=float(req.handoff.hotel_depth or 160.0),
            hotel_clearance_height=float(
                req.handoff.hotel_clearance_height or 7.5,
            ),
        )
        # Refuse names that conflict with an existing on-deck resource
        # we haven't registered ourselves — we don't want to yank a real
        # plate/carrier out of the deck tree by accident.
        if record.name not in self._handoff_registry:
            try:
                self._machine.lh.deck.get_resource(record.name)
                resp.success = False
                resp.message = (
                    f"name {record.name!r} clashes with an existing deck "
                    "resource; pick a different handoff name"
                )
                return resp
            except Exception:
                pass  # name is free — proceed

        ok = self._handoff_registry.set(
            record, replace_existing=bool(req.replace_existing),
        )
        if not ok:
            resp.success = False
            resp.message = (
                f"handoff {record.name!r} already exists "
                "(pass replace_existing=true to overwrite)"
            )
            return resp
        # Inject the deck-tree view so ``MoveResource.to = <name>`` just
        # works. Done under the bridge because deck mutation may cascade
        # through pylabrobot on some implementations — safest to keep
        # all deck writes on the same loop as LoadDeck.
        try:
            self._bridge.submit(self._assign_handoff_holder_coro(record)).result(timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            # Roll back the registry insertion so state stays consistent.
            self._handoff_registry.delete(record.name)
            resp.success = False
            resp.message = f"deck injection failed: {exc}"
            return resp
        self._emit_event(FSMEvent(
            kind="handoff_defined",
            message=f"registered handoff {record.name!r}",
            details={
                "name": record.name,
                "x": f"{record.x:.2f}",
                "y": f"{record.y:.2f}",
                "z": f"{record.z:.2f}",
                "hotel_depth": f"{record.hotel_depth:.1f}",
            },
        ))
        resp.success = True
        resp.message = f"handoff {record.name!r} defined"
        return resp

    async def _assign_handoff_holder_coro(self, record: HandoffRecord) -> None:
        self._machine.assign_handoff_holder(
            name=record.name, x=record.x, y=record.y, z=record.z,
        )

    async def _unassign_handoff_holder_coro(self, name: str) -> bool:
        return self._machine.unassign_handoff_holder(name)

    def _srv_list_handoffs(
        self, _req: ListHandoffs.Request, resp: ListHandoffs.Response,
    ):
        resp.handoffs = [
            self._handoff_msg_from_record(r)
            for r in self._handoff_registry.all()
        ]
        return resp

    def _srv_delete_handoff(
        self, req: DeleteHandoff.Request, resp: DeleteHandoff.Response,
    ):
        if not (req.name or "").strip():
            resp.success = False
            resp.message = "name is required"
            return resp
        ok = self._handoff_registry.delete(req.name)
        if ok:
            try:
                self._bridge.submit(
                    self._unassign_handoff_holder_coro(req.name),
                ).result(timeout=5.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"handoff {req.name!r} removed from registry but deck "
                    f"holder detach raised: {exc}"
                )
        resp.success = bool(ok)
        resp.message = (
            f"handoff {req.name!r} deleted" if ok
            else f"handoff {req.name!r} not registered"
        )
        if ok:
            self._emit_event(FSMEvent(
                kind="handoff_deleted",
                message=f"deleted handoff {req.name!r}",
                details={"name": req.name},
            ))
        return resp

    # ---- action registration ----
    def _register_actions(self) -> None:
        # shared (per-channel concurrent)
        self._mk_shared_action(
            PickUpTips, "~/pick_up_tips", ActionKind.PICK_UP_TIPS,
            self._exec_pick_up_tips,
        )
        self._mk_shared_action(
            DropTips, "~/drop_tips", ActionKind.DROP_TIPS,
            self._exec_drop_tips,
        )
        self._mk_shared_action(
            Aspirate, "~/aspirate", ActionKind.ASPIRATE, self._exec_aspirate,
        )
        self._mk_shared_action(
            Dispense, "~/dispense", ActionKind.DISPENSE, self._exec_dispense,
        )
        self._mk_shared_action(
            Transfer, "~/transfer", ActionKind.TRANSFER, self._exec_transfer,
        )
        # exclusive
        self._mk_exclusive_action(
            PickUpTips96, "~/pick_up_tips96", ActionKind.PICK_UP_TIPS_96,
            self._exec_pick_up_tips96,
        )
        self._mk_exclusive_action(
            DropTips96, "~/drop_tips96", ActionKind.DROP_TIPS_96,
            self._exec_drop_tips96,
        )
        self._mk_exclusive_action(
            Aspirate96, "~/aspirate96", ActionKind.ASPIRATE_96, self._exec_aspirate96,
        )
        self._mk_exclusive_action(
            Dispense96, "~/dispense96", ActionKind.DISPENSE_96, self._exec_dispense96,
        )
        self._mk_exclusive_action(
            MoveResource, "~/move_resource", ActionKind.MOVE_RESOURCE,
            self._exec_move_resource,
        )
        self._mk_exclusive_action(
            PickUpCoreGripper, "~/pick_up_core_gripper",
            ActionKind.PICK_UP_CORE_GRIPPER, self._exec_pick_up_core_gripper,
        )
        self._mk_exclusive_action(
            ReturnCoreGripper, "~/return_core_gripper",
            ActionKind.RETURN_CORE_GRIPPER, self._exec_return_core_gripper,
        )
        self._mk_exclusive_action(
            JogChannel, "~/jog_channel", ActionKind.JOG_CHANNEL,
            self._exec_jog_channel,
        )
        # HandoffTransfer needs feedback publication during the 6-stage
        # motion, so it gets a dedicated registration that runs its own
        # execute wrapper (see ``_execute_handoff``).
        self._handoff_action = ActionServer(
            self,
            HandoffTransfer,
            "~/handoff_transfer",
            execute_callback=self._execute_handoff,
            goal_callback=functools.partial(
                self._goal_callback, ActionKind.HANDOFF_TRANSFER, False,
            ),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._actions_cb_group,
        )

    def _mk_shared_action(
        self,
        action_type: Any,
        name: str,
        kind: ActionKind,
        executor: Callable[[Any], dict[str, Any]],
    ) -> ActionServer:
        return ActionServer(
            self,
            action_type,
            name,
            execute_callback=functools.partial(
                self._execute_shared, kind, executor,
            ),
            goal_callback=functools.partial(self._goal_callback, kind, True),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._actions_cb_group,
        )

    def _mk_exclusive_action(
        self,
        action_type: Any,
        name: str,
        kind: ActionKind,
        executor: Callable[[Any], dict[str, Any]],
    ) -> ActionServer:
        return ActionServer(
            self,
            action_type,
            name,
            execute_callback=functools.partial(
                self._execute_exclusive, kind, executor,
            ),
            goal_callback=functools.partial(self._goal_callback, kind, False),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._actions_cb_group,
        )

    def _goal_callback(
        self, kind: ActionKind, _is_shared: bool, goal_request: Any,
    ) -> GoalResponse:
        params = self._params_for_gate(kind, goal_request)
        gate = self._fsm.can_accept(kind, params)
        if gate.ok:
            return GoalResponse.ACCEPT
        if self._on_conflict == "queue":
            # ambiguous: we can only queue if the failure is purely a
            # concurrency conflict, not a FSM-substate violation. Conservative
            # behaviour: still reject so operators see issues immediately.
            self.get_logger().warn(
                f"rejecting {kind.value}: {gate.reason} (on_conflict=queue not honored for FSM substate violations)"
            )
        return GoalResponse.REJECT

    def _params_for_gate(self, kind: ActionKind, goal: Any) -> dict[str, Any]:
        """Extract the fields the FSM cares about from each goal type."""
        params: dict[str, Any] = {}
        if kind in (ActionKind.PICK_UP_TIPS, ActionKind.DROP_TIPS):
            # try both `resources`-like fields and fall back to implicit
            params["use_channels"] = _infer_channels(
                list(goal.use_channels or []),
                len(goal.wells or []),
                self._num_channels,
            )
        elif kind in (ActionKind.ASPIRATE, ActionKind.DISPENSE):
            params["use_channels"] = _infer_channels(
                list(goal.use_channels or []),
                len(goal.resources or []),
                self._num_channels,
            )
        elif kind == ActionKind.TRANSFER:
            # transfer uses a single source aspirate then N dispenses; one channel
            params["use_channels"] = [0]
        elif kind == ActionKind.PICK_UP_CORE_GRIPPER:
            params["front_channel"] = int(goal.front_channel)
        elif kind == ActionKind.MOVE_RESOURCE:
            params["transport"] = goal.transport or "auto"
            # If either endpoint names a registered handoff (or the
            # caller forced ``use_unsafe_hotel``), apply the iSWAP-
            # hotel gates below — the iSWAP needs to be healthy and
            # empty-handed to execute the hotel path at either end.
            is_hotel = bool(
                (goal.resource and goal.resource in self._handoff_registry)
                or (goal.to and goal.to in self._handoff_registry)
                or getattr(goal, "use_unsafe_hotel", False)
            )
            params["use_unsafe_hotel"] = is_hotel
            params["to_handoff"] = goal.to if is_hotel else ""
        elif kind == ActionKind.HANDOFF_TRANSFER:
            params["direction"] = goal.direction or ""
            params["calibration_name"] = goal.calibration_name or ""
            params["on_deck_resource"] = goal.on_deck_resource or ""
        return params

    # ---- shared / exclusive execution wrappers ----
    def _execute_shared(
        self,
        kind: ActionKind,
        executor: Callable[[Any], dict[str, Any]],
        goal_handle: Any,
    ):
        goal = goal_handle.request
        params = self._params_for_gate(kind, goal)
        channels: list[int] = params.get("use_channels") or []
        resources = self._resources_for_lock(kind, goal)
        self._fsm.on_shared_accepted(kind)
        result = self._result_for(kind)
        try:
            def _coro():
                return self._guarded_shared(
                    kind, executor, goal, channels, resources, goal_handle,
                )
            out = self._bridge.run(_coro, cancel_check=lambda: goal_handle.is_cancel_requested)
            self._populate_result(result, kind, goal, out, success=True, message=out.get("message", ""))
            goal_handle.succeed()
        except asyncio.CancelledError:
            self._populate_result(result, kind, goal, {}, success=False, message="cancelled")
            goal_handle.canceled()
        except Exception as exc:  # noqa: BLE001
            # Shared actions (pipetting) don't touch the iSWAP drives,
            # so default pessimistic invalidation (init=False, y_path=False)
            # is enough — no iSWAP fault flag.
            self._fsm.mark_error_and_invalidate(
                f"{kind.value} failed: {exc}",
                mark_iswap_faulted=False,
            )
            self._populate_result(result, kind, goal, {}, success=False, message=str(exc))
            goal_handle.abort()
        finally:
            self._fsm.on_shared_finished(kind)
        return result

    async def _guarded_shared(
        self,
        kind: ActionKind,
        executor: Callable[[Any], Awaitable[dict[str, Any]]],
        goal: Any,
        channels: list[int],
        resources: list[str],
        goal_handle: Any,
    ) -> dict[str, Any]:
        async with self._lock.shared(channels=channels, resources=resources):
            return await executor(goal)

    def _execute_exclusive(
        self,
        kind: ActionKind,
        executor: Callable[[Any], dict[str, Any]],
        goal_handle: Any,
    ):
        goal = goal_handle.request
        resources = self._resources_for_lock(kind, goal)
        self._fsm.on_exclusive_accepted(kind)
        result = self._result_for(kind)
        try:
            def _coro():
                return self._guarded_exclusive(kind, executor, goal, resources)
            out = self._bridge.run(_coro, cancel_check=lambda: goal_handle.is_cancel_requested)
            self._populate_result(result, kind, goal, out, success=True, message=out.get("message", ""))
            goal_handle.succeed()
        except asyncio.CancelledError:
            self._populate_result(result, kind, goal, {}, success=False, message="cancelled")
            goal_handle.canceled()
        except Exception as exc:  # noqa: BLE001
            # Exclusive ops that might touch the iSWAP (MoveResource with
            # transport=iswap, JogChannel near iSWAP Y-range) don't
            # self-identify here, so we don't set mark_iswap_faulted — the
            # HandoffTransfer executor flags iSWAP faults explicitly via
            # its own except block.
            import sys, traceback
            print(
                f"[{kind.value}] exception {type(exc).__name__}: {exc!r}\n"
                + "".join(traceback.format_exception(exc)),
                file=sys.stderr, flush=True,
            )
            detail = f"{type(exc).__name__}: {exc}" if not str(exc) else str(exc)
            self._fsm.mark_error_and_invalidate(
                f"{kind.value} failed: {detail}",
                mark_iswap_faulted=False,
            )
            self._populate_result(result, kind, goal, {}, success=False, message=detail)
            goal_handle.abort()
        finally:
            self._fsm.on_exclusive_finished(kind)
        return result

    async def _guarded_exclusive(
        self,
        kind: ActionKind,
        executor: Callable[[Any], Awaitable[dict[str, Any]]],
        goal: Any,
        resources: list[str],
    ) -> dict[str, Any]:
        async with self._lock.exclusive(resources=resources):
            return await executor(goal)

    def _resources_for_lock(self, kind: ActionKind, goal: Any) -> list[str]:
        if kind == ActionKind.PICK_UP_TIPS or kind == ActionKind.DROP_TIPS:
            return [goal.target if kind == ActionKind.DROP_TIPS else goal.tip_rack]
        if kind in (ActionKind.ASPIRATE, ActionKind.DISPENSE):
            return list(goal.resources or [])
        if kind == ActionKind.TRANSFER:
            return [goal.source, *list(goal.targets or [])]
        if kind in (
            ActionKind.PICK_UP_TIPS_96, ActionKind.DROP_TIPS_96,
            ActionKind.ASPIRATE_96, ActionKind.DISPENSE_96,
        ):
            name = getattr(goal, "tip_rack", None) or getattr(goal, "resource", None) or getattr(goal, "target", None)
            return [name] if name else []
        if kind == ActionKind.MOVE_RESOURCE:
            # `to` is a deck-tree resource name for both on-deck and
            # off-deck handoff destinations (registered handoffs are
            # auto-injected as deck ResourceHolders) — lock both ends.
            return [goal.resource, goal.to] if goal.to else [goal.resource]
        if kind in (ActionKind.PICK_UP_CORE_GRIPPER, ActionKind.RETURN_CORE_GRIPPER):
            return [goal.gripper_resource or "core_grippers"]
        if kind == ActionKind.HANDOFF_TRANSFER:
            # Lock the on-deck endpoint. The off-deck handoff has no deck
            # resource to contend for.
            return [goal.on_deck_resource] if goal.on_deck_resource else []
        return []

    def _result_for(self, kind: ActionKind) -> Any:
        m = {
            ActionKind.PICK_UP_TIPS: PickUpTips.Result,
            ActionKind.DROP_TIPS: DropTips.Result,
            ActionKind.ASPIRATE: Aspirate.Result,
            ActionKind.DISPENSE: Dispense.Result,
            ActionKind.TRANSFER: Transfer.Result,
            ActionKind.PICK_UP_TIPS_96: PickUpTips96.Result,
            ActionKind.DROP_TIPS_96: DropTips96.Result,
            ActionKind.ASPIRATE_96: Aspirate96.Result,
            ActionKind.DISPENSE_96: Dispense96.Result,
            ActionKind.MOVE_RESOURCE: MoveResource.Result,
            ActionKind.PICK_UP_CORE_GRIPPER: PickUpCoreGripper.Result,
            ActionKind.RETURN_CORE_GRIPPER: ReturnCoreGripper.Result,
            ActionKind.JOG_CHANNEL: JogChannel.Result,
        }
        return m[kind]()

    def _populate_result(
        self,
        result: Any,
        kind: ActionKind,
        goal: Any,
        out: dict[str, Any],
        success: bool,
        message: str,
    ) -> None:
        result.success = success
        result.message = message
        if hasattr(result, "warnings"):
            result.warnings = list(out.get("warnings", []))
        if hasattr(result, "channels_used"):
            result.channels_used = list(out.get("channels_used", []))
        if hasattr(result, "final_volumes"):
            result.final_volumes = list(out.get("final_volumes", []))
        if hasattr(result, "final_x"):
            result.final_x = float(out.get("final_x", 0.0))
            result.final_y = float(out.get("final_y", 0.0))
            result.final_z = float(out.get("final_z", 0.0))

    # ---- action executors (all run on asyncio loop) ----
    async def _exec_pick_up_tips(self, g) -> dict[str, Any]:
        channels = _infer_channels(list(g.use_channels or []), len(g.wells or []), self._num_channels)
        await self._machine.pick_up_tips(
            g.tip_rack, list(g.wells), list(g.use_channels or []) or None,
            list(g.offsets_z or []) or None,
        )
        self._fsm.mark_tips_picked_up(channels)
        return {"channels_used": channels}

    async def _exec_drop_tips(self, g) -> dict[str, Any]:
        channels = _infer_channels(list(g.use_channels or []), len(g.wells or []), self._num_channels)
        await self._machine.drop_tips(
            g.target, list(g.wells), list(g.use_channels or []) or None,
            allow_nonzero_volume=bool(g.allow_nonzero_volume),
        )
        self._fsm.mark_tips_dropped(channels)
        return {"channels_used": channels}

    async def _exec_aspirate(self, g) -> dict[str, Any]:
        channels = _infer_channels(list(g.use_channels or []), len(g.resources or []), self._num_channels)
        await self._machine.aspirate(
            list(g.resources), list(g.volumes),
            list(g.use_channels or []) or None,
            list(g.flow_rates or []) or None,
            list(g.offsets_z or []) or None,
            list(g.liquid_height or []) or None,
        )
        return {"channels_used": channels, "final_volumes": list(g.volumes)}

    async def _exec_dispense(self, g) -> dict[str, Any]:
        channels = _infer_channels(list(g.use_channels or []), len(g.resources or []), self._num_channels)
        await self._machine.dispense(
            list(g.resources), list(g.volumes),
            list(g.use_channels or []) or None,
            list(g.flow_rates or []) or None,
            list(g.offsets_z or []) or None,
            list(g.liquid_height or []) or None,
            list(g.blow_out_air_volume or []) or None,
        )
        return {"channels_used": channels, "final_volumes": list(g.volumes)}

    async def _exec_transfer(self, g) -> dict[str, Any]:
        await self._machine.transfer(
            g.source, list(g.targets), g.source_volume,
            list(g.target_volumes or []) or None,
            g.aspiration_flow_rate or None,
            g.dispense_flow_rate or None,
        )
        return {"channels_used": [0]}

    async def _exec_pick_up_tips96(self, g) -> dict[str, Any]:
        await self._machine.pick_up_tips96(g.tip_rack, g.offset)
        self._fsm.mark_core96_loaded(True)
        return {}

    async def _exec_drop_tips96(self, g) -> dict[str, Any]:
        await self._machine.drop_tips96(g.target, g.offset, bool(g.allow_nonzero_volume))
        self._fsm.mark_core96_loaded(False)
        return {}

    async def _exec_aspirate96(self, g) -> dict[str, Any]:
        await self._machine.aspirate96(g.resource, g.volume, g.offset, g.flow_rate or None)
        return {}

    async def _exec_dispense96(self, g) -> dict[str, Any]:
        await self._machine.dispense96(
            g.resource, g.volume, g.offset, g.flow_rate or None,
            g.blow_out_air_volume or None,
        )
        return {}

    async def _exec_move_resource(self, g) -> dict[str, Any]:
        # iSWAP prep — firmware ``PP``/``PQ``/``PO``/``PI`` atomics
        # plan motion from the CURRENT pose, and the post-init pose
        # (Y≈626) + a direct rotation to e.g. front-left binds the
        # wrist drive against the back wall ("R002/82 Wrist twist
        # drive movement error" / end-stop crash). Park the arm at a
        # known-good MIDDLE Y — not ``park_iswap`` (PG), which moves
        # to a far-back Y and also hits the wall — so the firmware's
        # own pre-rotation starts from clearance on both sides.
        #
        # iSWAP SAFE_Y (330 mm) is the empirically-determined middle;
        # see ``iswap_handoff.SAFE_Y`` and the memory note
        # "Rotate only at safe back Y≈330mm".
        transport = (g.transport or "auto").lower()
        uses_iswap = (
            transport == "iswap"
            or transport == "auto"
            or bool(getattr(g, "use_unsafe_hotel", False))
            or (g.resource and g.resource in self._handoff_registry)
            or (g.to and g.to in self._handoff_registry)
        )
        if uses_iswap and not isinstance(
            self._machine.backend, STARChatterboxBackend,
        ):
            if not self._fsm.iswap_y_path_clear:
                await self._machine.free_iswap_y_range()
                self._fsm.mark_iswap_y_path_clear(True)
            if not self._fsm.iswap_at_safe_pose:
                await self._machine.backend.move_iswap_y(
                    iswap_handoff.SAFE_Y,
                )
                self._fsm.mark_iswap_at_safe_pose(True)

        # Off-deck handoff transfers route through the 6-stage
        # manual-jog recipe (``iswap_handoff.transfer``). The atomic
        # hotel path (pylabrobot's ``use_unsafe_hotel=True``) can't
        # express the composite shoulder+wrist orientation this STAR's
        # incubator needs: every ``grip_direction`` enum value maps to
        # a fixed shoulder-AND-wrist pair, and for a left-side dock the
        # required combo is shoulder LEFT + wrist STRAIGHT, which isn't
        # any of FRONT/BACK/LEFT/RIGHT. The manual recipe rotates each
        # drive independently, reading from the ``rotation`` / ``wrist``
        # fields on the registered handoff record.
        src_record = self._handoff_registry.get(g.resource) if g.resource else None
        dst_record = self._handoff_registry.get(g.to) if g.to else None
        if src_record is not None or dst_record is not None:
            if dst_record is not None:
                record = dst_record
                direction = "to_handoff"
                on_deck_resource = g.resource
            else:
                record = src_record  # type: ignore[assignment]
                direction = "from_handoff"
                on_deck_resource = g.to
            calibration = HandoffCalibration(
                x=record.x, y=record.y, z=record.z,
                plate_width=record.plate_width,
                rotation=record.rotation,
                wrist=record.wrist,
            )
            pdft = (
                float(g.pickup_distance_from_top)
                if getattr(g, "pickup_distance_from_top", 0.0) > 0
                else iswap_handoff.DEFAULT_PICKUP_DISTANCE_FROM_TOP
            )
            offset = (
                float(getattr(g.destination_offset, "x", 0.0))
                if dst_record is not None
                else float(getattr(g.pickup_offset, "x", 0.0)),
                float(getattr(g.destination_offset, "y", 0.0))
                if dst_record is not None
                else float(getattr(g.pickup_offset, "y", 0.0)),
                float(getattr(g.destination_offset, "z", 0.0))
                if dst_record is not None
                else float(getattr(g.pickup_offset, "z", 0.0)),
            )

            def _on_stage(stage: int, _name: str) -> None:
                if stage == 3:
                    self._fsm.mark_iswap_holding_plate(True)
                elif stage == 5:
                    self._fsm.mark_iswap_holding_plate(False)

            self._fsm.mark_iswap_at_safe_pose(False)
            await self._machine.handoff_transfer(
                calibration=calibration,
                on_deck_resource=on_deck_resource,
                direction=direction,
                pickup_distance_from_top=pdft,
                handoff_offset=offset,
                on_stage=_on_stage,
            )
            # Stage 6 ends with park_iswap → arm at far-back Y, not SAFE_Y.
            self._fsm.mark_iswap_at_safe_pose(False)
            return {}

        # On-deck → on-deck: pylabrobot's atomic ``move_resource``
        # with firmware collision-control + end-of-move fold-up.
        kwargs: dict[str, Any] = {
            "transport": g.transport or "auto",
            "pickup_direction": g.pickup_direction or "",
            "drop_direction": g.drop_direction or "",
            "pickup_offset": g.pickup_offset,
            "destination_offset": g.destination_offset,
            "iswap_collision_control_level": 1,
            "iswap_fold_up_sequence_at_the_end_of_process": True,
        }
        if getattr(g, "pickup_distance_from_top", 0.0) > 0:
            kwargs["pickup_distance_from_top"] = float(g.pickup_distance_from_top)
        if getattr(g, "use_unsafe_hotel", False):
            # Explicit hotel mode without a registered handoff — caller
            # knows the geometry suits pylabrobot's fixed-orientation
            # atomic.
            kwargs["use_unsafe_hotel"] = True
            if float(getattr(g, "hotel_depth", 0.0)) > 0:
                kwargs["hotel_depth"] = float(g.hotel_depth)
            if float(getattr(g, "hotel_clearance_height", 0.0)) > 0:
                kwargs["hotel_clearance_height"] = float(g.hotel_clearance_height)

        await self._machine.move_resource(g.resource, g.to, **kwargs)
        return {}

    async def _exec_pick_up_core_gripper(self, g) -> dict[str, Any]:
        front = int(g.front_channel)
        await self._machine.pick_up_core_gripper(
            g.gripper_resource or "core_grippers", front,
        )
        self._fsm.mark_core_gripper_picked_up(front)
        return {"channels_used": [front - 1, front]}

    async def _exec_return_core_gripper(self, g) -> dict[str, Any]:
        await self._machine.return_core_gripper(g.gripper_resource or "core_grippers")
        self._fsm.mark_core_gripper_returned()
        return {}

    async def _exec_jog_channel(self, g) -> dict[str, Any]:
        await self._machine.jog_channel(int(g.channel), g.axis, float(g.target))
        return {}

    # ---- HandoffTransfer (dedicated path so we can publish feedback) ----
    def _lookup_handoff(self, name: str) -> HandoffRecord:
        """Resolve a handoff name against the runtime registry."""
        if not (name or "").strip():
            raise ValueError("calibration_name is required")
        record = self._handoff_registry.get(name)
        if record is None:
            raise ValueError(
                f"handoff {name!r} is not registered — call "
                f"/define_handoff first (known: {self._handoff_registry.names()})"
            )
        return record

    def _read_handoff_calibration(self, name: str) -> HandoffCalibration:
        """Back-compat projection from the registry to the shape
        :class:`iswap_handoff.HandoffCalibration` expects."""
        r = self._lookup_handoff(name)
        return HandoffCalibration(
            x=r.x, y=r.y, z=r.z,
            plate_width=r.plate_width,
            rotation=r.rotation,
            wrist=r.wrist,
        )

    def _execute_handoff(self, goal_handle):
        kind = ActionKind.HANDOFF_TRANSFER
        goal = goal_handle.request
        resources = self._resources_for_lock(kind, goal)
        self._fsm.on_exclusive_accepted(kind)
        result = HandoffTransfer.Result()
        try:
            calibration = self._read_handoff_calibration(goal.calibration_name)
            # Optional per-goal override of plate_width (keeps calibration
            # immutable on disk but lets a caller test a tighter grip).
            if float(goal.plate_width) > 0:
                calibration = HandoffCalibration(
                    x=calibration.x, y=calibration.y, z=calibration.z,
                    plate_width=float(goal.plate_width),
                    rotation=calibration.rotation,
                    wrist=calibration.wrist,
                )

            # Pre-flight: measure firmware reality, don't trust flags.
            # request_iswap_position raises on a faulted drive, which
            # routes to the except block below (mark_iswap_faulted=True).
            # Skipped when the simulator backend is in use — chatterbox
            # doesn't model these firmware responses.
            if not isinstance(self._machine.backend, STARChatterboxBackend):
                self._bridge.run(
                    lambda: self._machine.backend.request_iswap_position(),
                )
                # Park to a known-good MIDDLE Y when the arm pose is
                # unknown — not ``park_iswap`` (PG), which moves to a
                # far-back Y and hits the back wall on this STAR
                # ("R002/82 wrist twist drive movement error"). Stage
                # 1 of the handoff recipe then rotates from there.
                if not self._fsm.iswap_at_safe_pose:
                    self._bridge.run(
                        lambda: self._machine.backend.move_iswap_y(
                            iswap_handoff.SAFE_Y,
                        ),
                    )
                    self._fsm.mark_iswap_at_safe_pose(True)

            def _coro():
                return self._guarded_handoff(goal_handle, goal, calibration, resources)

            out = self._bridge.run(
                _coro, cancel_check=lambda: goal_handle.is_cancel_requested,
            )
            # Clean completion — clear the diagnostic breadcrumb.
            self._fsm.mark_handoff_stage(0)
            result.success = True
            result.message = out.get("message", "")
            goal_handle.succeed()
        except asyncio.CancelledError:
            result.success = False
            result.message = "cancelled"
            goal_handle.canceled()
        except Exception as exc:  # noqa: BLE001
            # HandoffTransfer drives the iSWAP through 6 stages of manual
            # jogs — any firmware exception is an iSWAP fault. Preserve
            # iswap_holding_plate (operator-acknowledged) and
            # last_handoff_stage_reached (post-mortem) while flipping
            # drive-faulted + clearing at-safe-pose via the helper.
            self._fsm.mark_error_and_invalidate(
                f"handoff_transfer failed: {exc}",
                mark_iswap_faulted=True,
            )
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
        finally:
            self._fsm.on_exclusive_finished(kind)
        return result

    async def _guarded_handoff(self, goal_handle, goal, calibration, resources):
        async with self._lock.exclusive(resources=resources):
            return await self._run_handoff(goal_handle, goal, calibration)

    async def _run_handoff(self, goal_handle, goal, calibration: HandoffCalibration):
        # Re-run C0 FY on demand so the iSWAP Y path is clear. Any
        # pipetting action between handoffs flips iswap_y_path_clear off.
        if not self._fsm.iswap_y_path_clear:
            await self._machine.free_iswap_y_range()
            self._fsm.mark_iswap_y_path_clear(True)

        def on_stage(stage: int, name: str) -> None:
            fb = HandoffTransfer.Feedback()
            fb.stage = int(stage)
            fb.stage_name = name
            goal_handle.publish_feedback(fb)
            # Diagnostic breadcrumb — preserved across failures so an
            # operator can tell where we crashed.
            self._fsm.mark_handoff_stage(stage)
            # The arm leaves the safe pose the instant stage 1 starts
            # (park Y / rotate). Clear the flag so any crash in 2..5
            # forces the next handoff to re-park pre-flight.
            if stage == 1:
                self._fsm.mark_iswap_at_safe_pose(False)
            # stage 3 = close on plate; stage 5 = open to release.
            if stage == 3:
                self._fsm.mark_iswap_holding_plate(True)
            elif stage == 5:
                self._fsm.mark_iswap_holding_plate(False)
            # Stage 6 ends with park_iswap — from that point the arm is
            # back at a safe pose. Set the flag here (rather than
            # post-return) so feedback observers see it immediately.
            elif stage == 6:
                self._fsm.mark_iswap_at_safe_pose(True)

        pickup_distance = float(
            goal.pickup_distance_from_top
            or iswap_handoff.DEFAULT_PICKUP_DISTANCE_FROM_TOP
        )
        safe_y = float(goal.safe_y or iswap_handoff.SAFE_Y)
        traverse_z = float(goal.traverse_z or iswap_handoff.TRAVERSE_Z)
        offset = (
            float(goal.handoff_offset.x),
            float(goal.handoff_offset.y),
            float(goal.handoff_offset.z),
        )

        await self._machine.handoff_transfer(
            calibration=calibration,
            on_deck_resource=goal.on_deck_resource,
            direction=goal.direction,
            pickup_distance_from_top=pickup_distance,
            safe_y=safe_y,
            traverse_z=traverse_z,
            handoff_offset=offset,
            on_stage=on_stage,
        )
        return {"message": f"handoff_transfer {goal.direction} ok"}

    # ---- shutdown ----
    def destroy_node(self) -> bool:
        if (
            self.get_parameter("disconnect_on_shutdown")
            .get_parameter_value()
            .bool_value
        ):
            try:
                self._bridge.submit(self._machine.stop()).result(timeout=5.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"stop() failed on shutdown: {exc}")
        self._bridge.stop()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HamiltonStarActionServer()
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
