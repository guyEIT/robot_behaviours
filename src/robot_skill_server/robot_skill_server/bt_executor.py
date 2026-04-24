"""
BtExecutor - Executes BehaviorTree XML as a ROS2 Action Server.

Parses BT.CPP v4 XML and executes trees directly in Python using ROS2
action clients. No C++ dependency, no subprocess, no BT.CPP library.

Action: /skill_server/execute_behavior_tree (ExecuteBehaviorTree.action)
Topic:  /skill_server/task_state (TaskState.msg)
Topic:  /skill_server/log_events (LogEvent.msg)
Topic:  /skill_server/active_bt_xml (String, transient_local)
"""

from __future__ import annotations

import asyncio
import json
import os
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
import rclpy.time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from std_msgs.msg import String

from robot_skills_msgs.action import ExecuteBehaviorTree
from robot_skills_msgs.msg import LogEvent, TaskState

from robot_skill_server.tree_executor import (
    Blackboard,
    ExecutionContext,
    NodeStatus,
    count_action_nodes,
    get_main_tree_name,
    parse_trees,
)


def _humanise(s: str) -> str:
    return s.replace("_", " ").capitalize() if s else s


class BtExecutor(Node):
    """Executes BT XML directly using the Python tree executor."""

    def __init__(self):
        super().__init__("bt_executor")

        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            ExecuteBehaviorTree,
            "/skill_server/execute_behavior_tree",
            execute_callback=self._execute_bt,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self._task_state_pub = self.create_publisher(
            TaskState, "/skill_server/task_state", 10
        )
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._active_bt_xml_pub = self.create_publisher(
            String, "/skill_server/active_bt_xml", latched_qos
        )
        self._log_event_pub = self.create_publisher(
            LogEvent, "/skill_server/log_events", 10
        )

        self._current_task_id: Optional[str] = None
        self._current_ctx: Optional[ExecutionContext] = None
        self._task_started_at = None
        self._total_tasks_executed = 0
        self._last_task_result = "IDLE"

        # ── Available trees (scanned from disk, published as latched JSON) ──
        latched_trees_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._available_trees_pub = self.create_publisher(
            String, "/skill_server/available_trees", latched_trees_qos
        )
        self._trees_dir = self._find_trees_dir()
        self._last_tree_scan: dict[str, float] = {}  # filename → mtime
        self._publish_available_trees()
        # Re-scan every 2 seconds for new/changed files
        self.create_timer(2.0, self._check_trees_changed)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("bt_executor")
        self._diag_updater.add("executor_status", self._produce_diagnostics)

        self.get_logger().info(
            "BtExecutor started on /skill_server/execute_behavior_tree"
        )

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received BT execution request: '{goal_request.tree_name}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("BT execution cancel requested")
        if self._current_ctx:
            self._current_ctx.cancelled = True
        return CancelResponse.ACCEPT

    async def _execute_bt(self, goal_handle) -> ExecuteBehaviorTree.Result:
        goal = goal_handle.request
        result = ExecuteBehaviorTree.Result()
        task_id = "t" + uuid.uuid4().hex[:8]
        self._current_task_id = task_id
        self._task_started_at = self.get_clock().now().to_msg()
        start_time = time.time()

        self.get_logger().info(
            f"Executing BT '{goal.tree_name}' (id={task_id})"
        )

        self._active_bt_xml_pub.publish(String(data=goal.tree_xml))

        self._publish_log_event(
            "task_started",
            f"Task '{goal.tree_name}' started",
            task_id=task_id,
        )
        self._publish_task_state(
            task_id=task_id,
            task_name=goal.tree_name,
            status="RUNNING",
        )

        try:
            # Parse the XML into tree nodes
            trees = parse_trees(goal.tree_xml)
            main_tree_name = get_main_tree_name(goal.tree_xml)
            main_tree = trees.get(main_tree_name)

            if not main_tree:
                raise ValueError(
                    f"Main tree '{main_tree_name}' not found in XML. "
                    f"Available: {list(trees.keys())}"
                )

            # Create execution context
            ctx = ExecutionContext(self, trees)
            ctx.total_action_nodes = count_action_nodes(main_tree)
            self._current_ctx = ctx

            # Execute the tree
            bb = Blackboard()
            tree_status = await main_tree.tick(bb, ctx)

            self._current_ctx = None
            elapsed = time.time() - start_time

            if ctx.cancelled:
                final_status = "HALTED"
                success = False
                await main_tree.halt(ctx)
            elif tree_status == NodeStatus.SUCCESS:
                final_status = "SUCCESS"
                success = True
            else:
                final_status = "FAILURE"
                success = False

        except Exception as e:
            import traceback
            self._current_ctx = None
            elapsed = time.time() - start_time
            self.get_logger().error(
                f"Tree execution error: {type(e).__name__}: {e}\n"
                f"{traceback.format_exc()}"
            )
            result.success = False
            result.final_status = "ERROR"
            result.message = f"Tree execution error: {e}"
            result.total_execution_time_sec = elapsed
            self._publish_log_event(
                "task_error", str(e), severity="error", task_id=task_id
            )
            self._publish_task_state(
                task_id=task_id,
                task_name=goal.tree_name,
                status="FAILURE",
                error_message=str(e),
            )
            goal_handle.abort(result)
            self._current_task_id = None
            return result

        result.success = success
        result.final_status = final_status
        result.total_execution_time_sec = elapsed
        result.message = (
            f"BT '{goal.tree_name}' completed: {final_status} "
            f"in {elapsed:.2f}s"
        )

        self._total_tasks_executed += 1
        self._last_task_result = final_status
        self.get_logger().info(result.message)

        self._publish_task_state(
            task_id=task_id,
            task_name=goal.tree_name,
            status=final_status,
            current_node=ctx.current_skill if not success else "",
            progress=1.0 if success else ctx.progress,
            completed_skills=ctx.completed_skills,
            failed_skills=ctx.failed_skills,
            error_message=(
                ctx.failed_skills[-1] if ctx.failed_skills else ""
            ) if not success else "",
        )

        if final_status == "HALTED" and goal_handle.is_cancel_requested:
            self._publish_log_event(
                "task_cancelled",
                f"Task '{goal.tree_name}' cancelled after {elapsed:.1f}s",
                severity="warn",
                task_id=task_id,
            )
            goal_handle.canceled(result)
        elif success:
            self._publish_log_event(
                "task_completed",
                f"Task '{goal.tree_name}' completed in {elapsed:.1f}s",
                task_id=task_id,
            )
            goal_handle.succeed(result)
        else:
            self._publish_log_event(
                "task_failed",
                f"Task '{goal.tree_name}' failed after {elapsed:.1f}s",
                severity="error",
                task_id=task_id,
            )
            goal_handle.abort(result)

        self._current_task_id = None
        return result

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _publish_task_state(
        self,
        task_id: str,
        task_name: str,
        status: str,
        current_node: str = "",
        progress: float = 0.0,
        completed_skills: list[str] | None = None,
        failed_skills: list[str] | None = None,
        error_message: str = "",
    ):
        now = self.get_clock().now().to_msg()
        msg = TaskState()
        msg.task_id = task_id
        msg.task_name = task_name
        msg.status = status
        msg.current_skill = current_node
        msg.current_bt_node = current_node
        msg.progress = progress
        msg.started_at = self._task_started_at or now
        msg.updated_at = now
        msg.elapsed_sec = (
            (self.get_clock().now().nanoseconds
             - rclpy.time.Time.from_msg(msg.started_at).nanoseconds) / 1e9
            if self._task_started_at else 0.0
        )
        msg.completed_skills = completed_skills or []
        msg.failed_skills = failed_skills or []
        msg.error_message = error_message
        self._task_state_pub.publish(msg)

    def _publish_log_event(
        self,
        event_name: str,
        message: str,
        severity: str = "info",
        task_id: str = "",
    ):
        log_msg = LogEvent()
        log_msg.stamp = self.get_clock().now().to_msg()
        log_msg.event_name = event_name
        log_msg.severity = severity
        log_msg.message = message
        log_msg.task_id = task_id
        log_msg.tags = ["human"]
        self._log_event_pub.publish(log_msg)

    # ── Tree discovery ─────────────────────────────────────────────────────

    def _find_trees_dir(self) -> Optional[str]:
        """Find the behavior trees directory. Prefer source (volume-mounted)
        over installed share so new files appear immediately."""
        # Source directory (volume-mounted, always up to date)
        for candidate in [
            "/home/ws/src/robot_behaviors/trees",
            os.path.join(os.path.dirname(__file__), "..", "..", "..",
                         "robot_behaviors", "trees"),
        ]:
            if os.path.isdir(candidate):
                return os.path.realpath(candidate)
        # Fallback: installed share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            d = os.path.join(
                get_package_share_directory("robot_behaviors"), "trees"
            )
            if os.path.isdir(d):
                return d
        except Exception:
            pass
        return None

    def _scan_trees(self) -> list[dict]:
        """Scan the trees directory and return metadata + XML for each tree."""
        if not self._trees_dir or not os.path.isdir(self._trees_dir):
            return []

        trees = []
        for f in sorted(Path(self._trees_dir).glob("*.xml")):
            try:
                xml_text = f.read_text()
                tree_name = get_main_tree_name(xml_text)
                # Derive a human-readable label from filename
                label = f.stem.replace("_", " ").title()
                trees.append({
                    "name": tree_name or f.stem,
                    "label": label,
                    "filename": f.name,
                    "xml": xml_text,
                })
            except Exception as e:
                self.get_logger().warning(f"Failed to parse {f.name}: {e}")
        return trees

    def _publish_available_trees(self):
        """Publish the current tree list as JSON."""
        trees = self._scan_trees()
        msg = String()
        msg.data = json.dumps(trees)
        self._available_trees_pub.publish(msg)
        # Track mtimes for change detection
        if self._trees_dir:
            self._last_tree_scan = {
                f.name: f.stat().st_mtime
                for f in Path(self._trees_dir).glob("*.xml")
            }
        self.get_logger().info(
            f"Published {len(trees)} available trees from {self._trees_dir}"
        )

    def _check_trees_changed(self):
        """Re-publish if any tree files were added, removed, or modified."""
        if not self._trees_dir or not os.path.isdir(self._trees_dir):
            return
        current = {
            f.name: f.stat().st_mtime
            for f in Path(self._trees_dir).glob("*.xml")
        }
        if current != self._last_tree_scan:
            self._publish_available_trees()

    def _produce_diagnostics(self, stat):
        if self._current_task_id:
            stat.summary(
                DiagnosticStatus.OK,
                f"Running task {self._current_task_id}",
            )
        elif self._last_task_result == "FAILURE":
            stat.summary(DiagnosticStatus.WARN, "Last task failed")
        else:
            stat.summary(DiagnosticStatus.OK, "Idle")

        stat.add("current_task", self._current_task_id or "none")
        stat.add("total_executed", str(self._total_tasks_executed))
        stat.add("last_result", self._last_task_result)
        return stat
