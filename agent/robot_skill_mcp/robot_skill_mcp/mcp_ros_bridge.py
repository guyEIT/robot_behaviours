"""ROS2 ↔ asyncio bridge powering the BT executor MCP server.

The MCP server runs on asyncio in the main thread. rclpy needs its own
executor spinning to process subscriptions, action callbacks, and service
responses. This module owns that lifecycle:

  - Owns one rclpy Node, ActionClient, four service Clients, and four
    subscriptions (task_state, available_trees, active_bt_xml, log_events).
  - Spawns a daemon thread running MultiThreadedExecutor.spin().
  - Caches the latest subscription payloads behind a lock so MCP tools can
    answer "what's the current status?" without round-tripping ROS.
  - Exposes async helpers that bridge rclpy.task.Future to asyncio.Future
    via add_done_callback + loop.call_soon_threadsafe.
"""

from __future__ import annotations

import asyncio
import collections
import json
import threading
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from robot_skills_msgs.action import ExecuteBehaviorTree
from robot_skills_msgs.msg import DryRunStatus, LogEvent, TaskState
from robot_skills_msgs.srv import (
    ApproveDryRun,
    ComposeTask,
    GetSkillDescriptions,
    RegisterCompoundSkill,
    RegisterScript,
)


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class _Caches:
    task_state: dict[str, Any] | None = None
    available_trees: dict[str, dict[str, Any]] = field(default_factory=dict)
    active_bt_xml: str = ""
    recent_log_events: collections.deque = field(
        default_factory=lambda: collections.deque(maxlen=200)
    )
    dryrun_status: dict[str, Any] | None = None


class MCPRosBridge:
    def __init__(self, node_name: str = "bt_executor_mcp") -> None:
        rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)

        self._caches = _Caches()
        self._cache_lock = threading.RLock()
        self._active_goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_client = ActionClient(
            self._node,
            ExecuteBehaviorTree,
            "/skill_server/execute_behavior_tree",
        )
        self._compose_client: Client = self._node.create_client(
            ComposeTask, "/skill_server/compose_task"
        )
        self._register_client: Client = self._node.create_client(
            RegisterCompoundSkill, "/skill_server/register_compound_skill"
        )
        self._descriptions_client: Client = self._node.create_client(
            GetSkillDescriptions, "/skill_server/get_skill_descriptions"
        )
        self._approve_client: Client = self._node.create_client(
            ApproveDryRun, "/skill_server/approve_dry_run"
        )

        self._node.create_subscription(
            TaskState, "/skill_server/task_state", self._on_task_state, 10
        )
        self._node.create_subscription(
            String, "/skill_server/available_trees", self._on_available_trees, LATCHED_QOS
        )
        self._node.create_subscription(
            String, "/skill_server/active_bt_xml", self._on_active_bt_xml, LATCHED_QOS
        )
        self._node.create_subscription(
            LogEvent, "/skill_server/log_events", self._on_log_event, 50
        )
        self._node.create_subscription(
            DryRunStatus, "/skill_server/dryrun_status", self._on_dryrun_status, LATCHED_QOS
        )

        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="mcp-rclpy-spin", daemon=True
        )
        self._spin_thread.start()

    # ── Subscription callbacks (run on rclpy executor thread) ──────────────

    def _on_task_state(self, msg: TaskState) -> None:
        snap = {
            "task_id": msg.task_id,
            "task_name": msg.task_name,
            "status": msg.status,
            "current_skill": msg.current_skill,
            "current_bt_node": msg.current_bt_node,
            "progress": float(msg.progress),
            "completed_skills": list(msg.completed_skills),
            "failed_skills": list(msg.failed_skills),
            "elapsed_sec": float(msg.elapsed_sec),
            "error_message": msg.error_message,
            "error_skill": msg.error_skill,
        }
        with self._cache_lock:
            self._caches.task_state = snap

    def _on_available_trees(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data) if msg.data else []
        except json.JSONDecodeError:
            return
        trees: dict[str, dict[str, Any]] = {}
        if isinstance(payload, list):
            for entry in payload:
                if not isinstance(entry, dict):
                    continue
                name = entry.get("name") or entry.get("filename")
                if not name:
                    continue
                trees[name] = entry
        with self._cache_lock:
            self._caches.available_trees = trees

    def _on_active_bt_xml(self, msg: String) -> None:
        with self._cache_lock:
            self._caches.active_bt_xml = msg.data or ""

    def _on_log_event(self, msg: LogEvent) -> None:
        snap = {
            "stamp_sec": msg.stamp.sec + msg.stamp.nanosec / 1e9,
            "event_name": msg.event_name,
            "severity": msg.severity,
            "message": msg.message,
            "task_id": msg.task_id,
            "skill_name": msg.skill_name,
            "tags": list(msg.tags),
        }
        with self._cache_lock:
            self._caches.recent_log_events.append(snap)

    def _on_dryrun_status(self, msg: DryRunStatus) -> None:
        # An empty task_id means "no goal pending" — clear the cache so
        # callers don't mistake a stale entry for a fresh approval prompt.
        if not msg.task_id:
            with self._cache_lock:
                self._caches.dryrun_status = None
            return
        snap = {
            "task_id": msg.task_id,
            "tree_name": msg.tree_name,
            "sim_status": msg.sim_status,
            "message": msg.message,
            "sim_duration_sec": float(msg.sim_duration_sec),
            "completed_skills": list(msg.completed_skills),
            "failed_skills": list(msg.failed_skills),
            "published_at_sec": (
                msg.published_at.sec + msg.published_at.nanosec / 1e9
            ),
        }
        with self._cache_lock:
            self._caches.dryrun_status = snap

    # ── Cache accessors (called from MCP tool handlers, asyncio thread) ────

    def list_trees(self) -> list[dict[str, Any]]:
        with self._cache_lock:
            return [
                {
                    "name": entry.get("name") or entry.get("filename"),
                    "label": entry.get("label", entry.get("name", "")),
                    "filename": entry.get("filename", ""),
                }
                for entry in self._caches.available_trees.values()
            ]

    def get_tree_xml(self, name: str) -> str | None:
        with self._cache_lock:
            entry = self._caches.available_trees.get(name)
            if entry is None:
                return None
            return entry.get("xml")

    def get_task_status(self) -> dict[str, Any] | None:
        with self._cache_lock:
            if self._caches.task_state is None:
                return None
            return dict(self._caches.task_state)

    def get_active_tree_xml(self) -> str:
        with self._cache_lock:
            return self._caches.active_bt_xml

    def get_recent_log_events(self, limit: int) -> list[dict[str, Any]]:
        with self._cache_lock:
            events = list(self._caches.recent_log_events)
        if limit > 0:
            events = events[-limit:]
        return events

    # ── Async future bridging ──────────────────────────────────────────────

    @staticmethod
    async def _await_ros_future(ros_future):
        loop = asyncio.get_running_loop()
        asyncio_future: asyncio.Future = loop.create_future()

        def _done(f):
            if asyncio_future.done():
                return
            try:
                result = f.result()
                loop.call_soon_threadsafe(
                    lambda: asyncio_future.done() or asyncio_future.set_result(result)
                )
            except Exception as exc:
                loop.call_soon_threadsafe(
                    lambda: asyncio_future.done() or asyncio_future.set_exception(exc)
                )

        ros_future.add_done_callback(_done)
        return await asyncio_future

    # ── Service / action helpers ───────────────────────────────────────────

    async def call_get_skill_descriptions(
        self,
        filter_categories: list[str],
        filter_tags: list[str],
        include_compounds: bool,
        include_pddl: bool,
    ) -> dict[str, Any]:
        if not self._descriptions_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("get_skill_descriptions service not available")
        req = GetSkillDescriptions.Request()
        req.filter_categories = filter_categories
        req.filter_tags = filter_tags
        req.include_compounds = include_compounds
        req.include_pddl = include_pddl
        resp = await self._await_ros_future(self._descriptions_client.call_async(req))
        return {
            "success": resp.success,
            "message": resp.message,
            "skills": [_skill_description_to_dict(s) for s in resp.skills],
        }

    async def call_compose_task(
        self,
        task_name: str,
        task_description: str,
        steps: list[dict[str, Any]],
        sequential: bool,
        add_precondition_checks: bool,
    ) -> dict[str, Any]:
        if not self._compose_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("compose_task service not available")
        from robot_skills_msgs.msg import TaskStep

        req = ComposeTask.Request()
        req.task_name = task_name
        req.task_description = task_description
        req.sequential = sequential
        req.add_precondition_checks = add_precondition_checks
        for s in steps:
            step = TaskStep()
            step.skill_name = s.get("skill_name", "")
            step.parameters_json = s.get("parameters_json", "")
            step.input_blackboard_keys = s.get("input_blackboard_keys", []) or []
            step.output_blackboard_keys = s.get("output_blackboard_keys", []) or []
            step.retry_on_failure = bool(s.get("retry_on_failure", False))
            step.max_retries = int(s.get("max_retries", 0))
            step.condition_expression = s.get("condition_expression", "")
            step.description = s.get("description", "")
            step.robot_id = s.get("robot_id", "")
            req.steps.append(step)
        resp = await self._await_ros_future(self._compose_client.call_async(req))
        return {
            "success": resp.success,
            "message": resp.message,
            "bt_xml": resp.bt_xml,
            "warnings": list(resp.warnings),
        }

    async def call_register_script(
        self,
        name: str,
        kind: str,
        source: str,
        input_schema: str,
        output_schema: str,
        description: str,
    ) -> dict[str, Any]:
        """Register a script with the local agent-host script_action_server.

        Discovers the per-session register_script service by scanning
        ``/script_action_server_*/register_script`` on the local DDS graph.
        Multiple agents are isolated by session-id suffix; we pick the first
        match (this MCP server only ever runs alongside one script server).
        """
        target = None
        for srv_name, types in self._node.get_service_names_and_types():
            if (
                srv_name.startswith("/script_action_server_")
                and srv_name.endswith("/register_script")
                and "robot_skills_msgs/srv/RegisterScript" in types
            ):
                target = srv_name
                break
        if target is None:
            raise RuntimeError(
                "no /script_action_server_*/register_script service on the "
                "local DDS graph — is robot_script_server running?"
            )
        client = self._node.create_client(RegisterScript, target)
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"{target} service not ready")
        req = RegisterScript.Request()
        req.name = name
        req.kind = kind
        req.source = source
        req.input_schema_json = input_schema
        req.output_schema_json = output_schema
        req.description = description
        resp = await self._await_ros_future(client.call_async(req))
        return {
            "success": resp.success,
            "message": resp.message,
            "service": target,
        }

    async def call_register_compound_skill(
        self,
        name: str,
        bt_xml: str,
        description: str,
        tags: list[str],
        persist: bool,
    ) -> dict[str, Any]:
        if not self._register_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("register_compound_skill service not available")
        from robot_skills_msgs.msg import SkillDescription

        req = RegisterCompoundSkill.Request()
        sd = SkillDescription()
        sd.name = name
        sd.display_name = name
        sd.description = description
        sd.category = "compound"
        sd.tags = tags
        sd.is_compound = True
        sd.bt_xml = bt_xml
        req.skill_description = sd
        req.bt_xml = bt_xml
        req.persist = persist
        resp = await self._await_ros_future(self._register_client.call_async(req))
        return {
            "success": resp.success,
            "message": resp.message,
            "registered_name": resp.registered_name,
        }

    async def execute_tree(
        self,
        tree_xml: str,
        tree_name: str,
        tick_rate_hz: float,
        wait_for_completion: bool,
        timeout_sec: float | None,
        target_mode: int = 0,
        sim_tree_xml: str = "",
    ) -> dict[str, Any]:
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("execute_behavior_tree action server not available")
        with self._goal_lock:
            if self._active_goal_handle is not None:
                raise RuntimeError(
                    "another tree is already executing; call cancel_execution first"
                )
        goal = ExecuteBehaviorTree.Goal()
        goal.tree_xml = tree_xml
        goal.tree_name = tree_name
        goal.tick_rate_hz = float(tick_rate_hz)
        goal.enable_groot_monitor = False
        goal.groot_zmq_port = 1666
        goal.target_mode = int(target_mode)
        goal.sim_tree_xml = sim_tree_xml or ""

        send_future = self._action_client.send_goal_async(goal)
        goal_handle = await self._await_ros_future(send_future)
        if not goal_handle.accepted:
            return {"accepted": False, "message": "goal rejected by server"}
        with self._goal_lock:
            self._active_goal_handle = goal_handle
        if not wait_for_completion:
            return {
                "accepted": True,
                "wait_for_completion": False,
                "message": "goal accepted; poll get_task_status for progress",
            }
        try:
            result_future = goal_handle.get_result_async()
            if timeout_sec is not None and timeout_sec > 0:
                wrapped = asyncio.wait_for(
                    self._await_ros_future(result_future), timeout=timeout_sec
                )
                result_msg = await wrapped
            else:
                result_msg = await self._await_ros_future(result_future)
        except asyncio.TimeoutError:
            try:
                goal_handle.cancel_goal_async()
            finally:
                with self._goal_lock:
                    self._active_goal_handle = None
            return {
                "accepted": True,
                "timed_out": True,
                "message": f"timed out after {timeout_sec}s; cancellation requested",
            }
        with self._goal_lock:
            self._active_goal_handle = None
        result = result_msg.result
        return {
            "accepted": True,
            "success": result.success,
            "final_status": result.final_status,
            "sim_final_status": getattr(result, "sim_final_status", "") or "",
            "real_final_status": getattr(result, "real_final_status", "") or "",
            "total_execution_time_sec": float(result.total_execution_time_sec),
            "message": result.message,
        }

    async def approve_dry_run(
        self,
        approve: bool,
        task_id: str = "",
        reason: str = "",
    ) -> dict[str, Any]:
        if not self._approve_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("approve_dry_run service not available")
        req = ApproveDryRun.Request()
        req.task_id = task_id
        req.approve = bool(approve)
        req.reason = reason
        future = self._approve_client.call_async(req)
        resp = await self._await_ros_future(future)
        return {
            "accepted": bool(resp.accepted),
            "message": resp.message,
        }

    def get_dryrun_status(self) -> dict[str, Any] | None:
        with self._cache_lock:
            snap = self._caches.dryrun_status
            return dict(snap) if snap is not None else None

    async def cancel_execution(self) -> dict[str, Any]:
        with self._goal_lock:
            handle = self._active_goal_handle
        if handle is None:
            return {"cancelled": False, "reason": "no active goal"}
        cancel_future = handle.cancel_goal_async()
        cancel_response = await self._await_ros_future(cancel_future)
        # CancelResponse.return_code: 0=ACCEPT, 1=REJECT, 2=UNKNOWN_GOAL_ID, 3=TERMINATED
        accepted = bool(getattr(cancel_response, "goals_canceling", []))
        return {
            "cancelled": accepted,
            "reason": "cancel accepted" if accepted else "cancel rejected by server",
        }

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def _skill_description_to_dict(sd) -> dict[str, Any]:
    return {
        "name": sd.name,
        "display_name": sd.display_name,
        "description": sd.description,
        "version": sd.version,
        "robot_id": sd.robot_id,
        "category": sd.category,
        "tags": list(sd.tags),
        "preconditions": list(sd.preconditions),
        "postconditions": list(sd.postconditions),
        "effects": list(sd.effects),
        "constraints": list(sd.constraints),
        "pddl_action": sd.pddl_action,
        "action_server_name": sd.action_server_name,
        "action_type": sd.action_type,
        "parameters_schema": sd.parameters_schema,
        "is_compound": sd.is_compound,
        "component_skills": list(sd.component_skills),
    }
