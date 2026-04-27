#!/usr/bin/env python3
"""bb_operator — operator-facing sidecar that mutates the persistent
blackboard of the currently active long-lived BT task.

Why a separate node: the BtExecutor is busy ticking the tree on the same
ROS executor, and we don't want operator service calls to block (or be
blocked by) tree execution. The sidecar opens the same SQLite file as the
tree — both processes coordinate via WAL.

Track the active task by subscribing to ``/skill_server/task_state``. Every
service is structurally the same: open the per-task DB at
``~/.local/state/skill_server/tasks/{task_id}/state.db``, mutate one or
more ``persistent.*`` keys, close the DB. No campaign logic — that lives
in the BT.

Concurrency note: read-modify-write of list-valued blackboard keys
(plate_queue) is racy with the tree's PopFromQueue. For 10–30 plates with
hours-cadence cycles, the race window is tiny. A future "atomic_append"
helper would close it, but is not required for the week-2 DoD.
"""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import String

from robot_skills_msgs.msg import LogEvent, TaskState
from robot_skills_msgs.srv import (
    AddPlate,
    OperatorDecision,
    PauseCampaign,
    RetirePlate,
)

from robot_skill_server.persistent_blackboard import (
    PersistentStore,
    task_db_path,
)


_ACTIVE_TASK_STATES = {"RUNNING", "AWAITING_APPROVAL", "PAUSED"}
_VALID_DECISIONS = {"retry", "skip-as-success", "skip-as-failure", "abort-tree"}


class BbOperator(Node):
    """Operator-side blackboard writer."""

    def __init__(self):
        super().__init__("bb_operator")
        cbg = ReentrantCallbackGroup()

        self._current_task_id: Optional[str] = None
        self._current_task_status: str = "IDLE"

        self._task_state_sub = self.create_subscription(
            TaskState,
            "/skill_server/task_state",
            self._on_task_state,
            10,
            callback_group=cbg,
        )

        self._log_pub = self.create_publisher(
            LogEvent, "/skill_server/log_events", 10
        )

        # Latched JSON snapshot of the active task's persistent blackboard.
        # The dashboard subscribes to this — one tiny topic, one source of
        # truth for every campaign view (plate queue, plates index, paused
        # flag, operator-decision queue, alerts, etc).
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._persistent_state_pub = self.create_publisher(
            String, "/skill_server/persistent_state", latched_qos,
        )
        # Periodic re-publish so writes from the BT executor (which doesn't
        # publish here itself) eventually reach subscribers. 1 Hz is plenty
        # for a campaign cadence measured in hours; bumps to event-driven
        # via _republish_state on every successful service handler.
        self.create_timer(1.0, self._tick_persistent_state, callback_group=cbg)
        self._last_state_json: str = ""

        # Services — all routed through the same `_with_active_task` helper.
        self._add_srv = self.create_service(
            AddPlate, "/bb_operator/add_plate", self._on_add_plate,
            callback_group=cbg,
        )
        self._retire_srv = self.create_service(
            RetirePlate, "/bb_operator/retire_plate", self._on_retire_plate,
            callback_group=cbg,
        )
        self._pause_srv = self.create_service(
            PauseCampaign, "/bb_operator/pause_campaign", self._on_pause,
            callback_group=cbg,
        )
        self._decision_srv = self.create_service(
            OperatorDecision, "/bb_operator/operator_decision",
            self._on_operator_decision, callback_group=cbg,
        )

        self.get_logger().info(
            "bb_operator started — services live under /bb_operator/*"
        )

    # ── task tracking ─────────────────────────────────────────────────────

    def _on_task_state(self, msg: TaskState) -> None:
        if msg.status in _ACTIVE_TASK_STATES:
            if msg.task_id != self._current_task_id:
                self.get_logger().info(
                    f"now tracking active task {msg.task_id!r} "
                    f"({msg.task_name!r}) status={msg.status}"
                )
            self._current_task_id = msg.task_id
            self._current_task_status = msg.status
        elif msg.status in {"SUCCESS", "FAILURE", "HALTED"}:
            # Don't drop _current_task_id immediately on terminal — leave it
            # so a late-arriving service call can still address the just-ended
            # task. The task DB itself is the source of truth (gone if reaped).
            self._current_task_status = msg.status
        # Refresh the latched snapshot so the dashboard sees task-status
        # transitions promptly without waiting for the 1 Hz timer tick.
        self._republish_state(force=False)

    def _open_active_store(self) -> Optional[PersistentStore]:
        if not self._current_task_id:
            return None
        path = task_db_path(self._current_task_id)
        if not path.is_file():
            return None
        return PersistentStore(path)

    # ── service handlers ──────────────────────────────────────────────────

    def _on_add_plate(self, request: AddPlate.Request, response: AddPlate.Response):
        store = self._open_active_store()
        if store is None:
            response.success = False
            response.message = "no active task — start a campaign first"
            return response
        try:
            queue = store.get_kv("persistent.plate_queue") or []
            if not isinstance(queue, list):
                queue = []
            now = time.time()
            plate = {
                "name": request.plate_name,
                "barcode": request.barcode,
                "target_cycles": int(request.target_cycles),
                "cadence_min": int(request.cadence_min),
                "imaging_protocol": request.imaging_protocol,
                "liconic_slot": int(request.liconic_slot),
                "cycle": 0,
                "next_due_at": now,        # first cycle runs immediately
                "retiring": False,
                "added_at": now,
            }
            queue.append(plate)
            store.set_kv("persistent.plate_queue", queue)
            # Side index by name so RetirePlate can find the plate without
            # walking the queue.
            plates = store.get_kv("persistent.plates") or {}
            if not isinstance(plates, dict):
                plates = {}
            plates[request.plate_name] = plate
            store.set_kv("persistent.plates", plates)
            self._publish_log(
                "plate_added",
                f"AddPlate {request.plate_name!r} into task {self._current_task_id}",
            )
            response.success = True
            response.message = f"queued plate {request.plate_name!r}"
            response.task_id = self._current_task_id or ""
        finally:
            store.close()
        # After every successful mutation, push a fresh latched snapshot
        # so subscribers (dashboard) react without waiting for the timer.
        self._republish_state(force=True)
        return response

    def _on_retire_plate(self, request: RetirePlate.Request,
                         response: RetirePlate.Response):
        store = self._open_active_store()
        if store is None:
            response.success = False
            response.message = "no active task"
            return response
        try:
            plates = store.get_kv("persistent.plates") or {}
            if not isinstance(plates, dict) or request.plate_name not in plates:
                response.success = False
                response.message = f"unknown plate {request.plate_name!r}"
                return response
            plates[request.plate_name]["retiring"] = True
            plates[request.plate_name]["retire_reason"] = request.reason
            store.set_kv("persistent.plates", plates)
            # Also flag any matching entry still on the queue so AdvancePlate
            # picks it up at the end of the in-flight cycle. (Operators can
            # call RetirePlate while the plate is on-deck mid-cycle.)
            queue = store.get_kv("persistent.plate_queue") or []
            if isinstance(queue, list):
                for entry in queue:
                    if isinstance(entry, dict) and entry.get("name") == request.plate_name:
                        entry["retiring"] = True
                store.set_kv("persistent.plate_queue", queue)
            self._publish_log(
                "plate_retiring",
                f"RetirePlate {request.plate_name!r}: {request.reason}",
            )
            response.success = True
            response.message = (
                f"plate {request.plate_name!r} marked retiring; will not be "
                f"re-queued after current cycle"
            )
        finally:
            store.close()
        # After every successful mutation, push a fresh latched snapshot
        # so subscribers (dashboard) react without waiting for the timer.
        self._republish_state(force=True)
        return response

    def _on_pause(self, request: PauseCampaign.Request,
                  response: PauseCampaign.Response):
        store = self._open_active_store()
        if store is None:
            response.success = False
            response.message = "no active task"
            return response
        try:
            store.set_kv("persistent.paused", bool(request.paused))
            store.set_kv("persistent.pause_reason", request.reason)
            verb = "paused" if request.paused else "resumed"
            self._publish_log(
                f"campaign_{verb}",
                f"Campaign {verb}: {request.reason}",
                severity="warn" if request.paused else "info",
            )
            response.success = True
            response.message = f"campaign {verb}"
        finally:
            store.close()
        # After every successful mutation, push a fresh latched snapshot
        # so subscribers (dashboard) react without waiting for the timer.
        self._republish_state(force=True)
        return response

    def _on_operator_decision(self, request: OperatorDecision.Request,
                              response: OperatorDecision.Response):
        if request.choice not in _VALID_DECISIONS:
            response.accepted = False
            response.message = (
                f"choice must be one of {sorted(_VALID_DECISIONS)}; "
                f"got {request.choice!r}"
            )
            return response
        store = self._open_active_store()
        if store is None:
            response.accepted = False
            response.message = "no active task"
            return response
        try:
            decisions = store.get_kv("persistent.operator_decisions") or {}
            if not isinstance(decisions, dict):
                decisions = {}
            decisions[request.node_path] = {
                "choice": request.choice,
                "reason": request.reason,
                "decided_at": time.time(),
            }
            store.set_kv("persistent.operator_decisions", decisions)
            self._publish_log(
                "operator_decision",
                f"OperatorDecision {request.choice!r} on {request.node_path!r}: "
                f"{request.reason}",
                severity="warn",
            )
            response.accepted = True
            response.message = f"decision recorded for {request.node_path!r}"
        finally:
            store.close()
        # After every successful mutation, push a fresh latched snapshot
        # so subscribers (dashboard) react without waiting for the timer.
        self._republish_state(force=True)
        return response

    # ── persistent-state mirror ───────────────────────────────────────────

    def _tick_persistent_state(self) -> None:
        """Periodic re-publish of the active task's persistent blackboard
        snapshot. No-op when no task is active or the snapshot is unchanged
        since the last publish — saves rosbridge bandwidth."""
        self._republish_state(force=False)

    def _republish_state(self, force: bool = False) -> None:
        snapshot = self._build_state_snapshot()
        encoded = json.dumps(snapshot, sort_keys=True, default=str)
        if not force and encoded == self._last_state_json:
            return
        self._last_state_json = encoded
        self._persistent_state_pub.publish(String(data=encoded))

    def _build_state_snapshot(self) -> dict:
        """Return a JSON-serialisable snapshot of the active task's
        persistent blackboard plus a couple of derived fields the dashboard
        needs (active task, paused flag)."""
        snap = {
            "task_id": self._current_task_id or "",
            "task_status": self._current_task_status,
            "paused": False,
            "persistent": {},
            "snapshot_at": time.time(),
        }
        store = self._open_active_store()
        if store is None:
            return snap
        try:
            kv = store.all_kv()
            snap["persistent"] = kv
            paused = kv.get("persistent.paused")
            snap["paused"] = bool(paused) if paused is not None else False
        finally:
            store.close()
        return snap

    # ── helpers ───────────────────────────────────────────────────────────

    def _publish_log(self, event: str, message: str, severity: str = "info") -> None:
        msg = LogEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.event_name = event
        msg.severity = severity
        msg.message = message
        msg.tags = ["operator"]
        try:
            msg.task_id = self._current_task_id or ""
        except AttributeError:
            pass
        self._log_pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = BbOperator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
