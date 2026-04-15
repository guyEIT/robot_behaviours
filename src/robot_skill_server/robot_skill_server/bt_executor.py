"""
BtExecutor - Action bridge between the dashboard and TreeExecutionServer.

Translates between the framework's ExecuteBehaviorTree action (used by the
dashboard and BehaviorLibrary) and the library's ExecuteTree action (served
by the C++ RobotTreeServer).

Action: /skill_server/execute_behavior_tree (ExecuteBehaviorTree.action)
Topic:  /skill_server/task_state (TaskState.msg)
Topic:  /skill_server/log_events (LogEvent.msg)
Topic:  /skill_server/active_bt_xml (String, transient_local)
"""

from __future__ import annotations

import time
import uuid
from typing import Optional

import rclpy
import rclpy.time
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from std_msgs.msg import String

from btcpp_ros2_interfaces.action import ExecuteTree
from robot_skills_msgs.action import ExecuteBehaviorTree
from robot_skills_msgs.msg import LogEvent, TaskState


def humanise(s: str) -> str:
    """Convert snake_case to 'Title case': 'go_to_observe' -> 'Go to observe'."""
    return s.replace("_", " ").capitalize() if s else s


class BtExecutor(Node):
    """Action bridge: ExecuteBehaviorTree -> ExecuteTree."""

    def __init__(self):
        super().__init__("bt_executor")

        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter(
            "execute_tree_action_name", "/skill_server/execute_tree"
        )

        callback_group = ReentrantCallbackGroup()

        # ── Our action server (dashboard-facing) ─────────────────────────────
        self._action_server = ActionServer(
            self,
            ExecuteBehaviorTree,
            "/skill_server/execute_behavior_tree",
            execute_callback=self._execute_bt,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        # ── Action client to the C++ TreeExecutionServer ─────────────────────
        action_name = self.get_parameter("execute_tree_action_name").value
        self._tree_client = ActionClient(
            self,
            ExecuteTree,
            action_name,
            callback_group=callback_group,
        )

        # ── Publishers ───────────────────────────────────────────────────────
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

        # ── State ────────────────────────────────────────────────────────────
        self._current_task_id: Optional[str] = None
        self._current_goal_handle = None  # ExecuteTree goal handle
        self._task_started_at = None
        self._total_tasks_executed = 0
        self._last_task_result = "IDLE"

        # ── Diagnostics ──────────────────────────────────────────────────────
        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("bt_executor")
        self._diag_updater.add("executor_status", self._produce_diagnostics)

        self.get_logger().info(
            "BtExecutor started on /skill_server/execute_behavior_tree"
        )

    # ── Action server callbacks ──────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received BT execution request: '{goal_request.tree_name}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("BT execution cancel requested")
        # Forward cancellation to the C++ server
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute_bt(self, goal_handle) -> ExecuteBehaviorTree.Result:
        """Bridge: receive ExecuteBehaviorTree, forward as ExecuteTree."""
        goal = goal_handle.request
        result = ExecuteBehaviorTree.Result()
        task_id = "t" + str(uuid.uuid4()).replace("-", "")[:8]
        self._current_task_id = task_id
        self._task_started_at = self.get_clock().now().to_msg()
        start_time = time.time()

        self.get_logger().info(
            f"Executing BT '{goal.tree_name}' (id={task_id})"
        )

        # Publish active BT XML for the web dashboard
        self._active_bt_xml_pub.publish(String(data=goal.tree_xml))

        # Publish task_started log event
        self._publish_log_event(
            "task_started",
            f"Task '{goal.tree_name}' started",
            task_id=task_id,
        )

        # Publish initial task state
        self._publish_task_state(
            task_id=task_id,
            task_name=goal.tree_name,
            status="RUNNING",
            current_node="",
        )

        # ── Wait for the C++ tree server ─────────────────────────────────────
        if not self._tree_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("TreeExecutionServer not available")
            result.success = False
            result.final_status = "ERROR"
            result.message = "TreeExecutionServer not available"
            result.total_execution_time_sec = time.time() - start_time
            self._publish_log_event(
                "task_error",
                result.message,
                severity="error",
                task_id=task_id,
            )
            goal_handle.abort(result)
            self._current_task_id = None
            return result

        # ── Send ExecuteTree goal ────────────────────────────────────────────
        tree_goal = ExecuteTree.Goal()
        tree_goal.target_tree = goal.tree_name
        tree_goal.payload = goal.tree_xml

        send_goal_future = self._tree_client.send_goal_async(
            tree_goal,
            feedback_callback=lambda fb: self._on_tree_feedback(
                fb, goal_handle, goal.tree_name, task_id, start_time
            ),
        )
        send_goal_future = await send_goal_future

        if not send_goal_future.accepted:
            self.get_logger().error("ExecuteTree goal was rejected")
            result.success = False
            result.final_status = "ERROR"
            result.message = "Tree execution goal rejected by server"
            result.total_execution_time_sec = time.time() - start_time
            goal_handle.abort(result)
            self._current_task_id = None
            return result

        self._current_goal_handle = send_goal_future

        # ── Wait for result ──────────────────────────────────────────────────
        try:
            get_result_future = send_goal_future.get_result_async()
            tree_result = await get_result_future
        except Exception as e:
            self.get_logger().error(f"Error getting tree result: {e}")
            elapsed = time.time() - start_time
            result.success = False
            result.final_status = "ERROR"
            result.message = f"Tree execution error: {e}"
            result.total_execution_time_sec = elapsed
            self._publish_log_event(
                "task_error",
                result.message,
                severity="error",
                task_id=task_id,
            )
            goal_handle.abort(result)
            self._current_task_id = None
            self._current_goal_handle = None
            return result

        self._current_goal_handle = None
        elapsed_total = time.time() - start_time

        # ── Map result ───────────────────────────────────────────────────────
        node_status = tree_result.result.node_status.status
        return_message = tree_result.result.return_message

        # GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        # btcpp_ros2_interfaces NodeStatus: 0=IDLE, 1=RUNNING, 2=SUCCESS,
        # 3=FAILURE, 4=SKIPPED
        from action_msgs.msg import GoalStatus
        if tree_result.status == GoalStatus.STATUS_CANCELED:
            final_status = "HALTED"
            success = False
        elif node_status == 2:  # NodeStatus.SUCCESS
            final_status = "SUCCESS"
            success = True
        elif node_status == 3:  # NodeStatus.FAILURE
            final_status = "FAILURE"
            success = False
        else:
            final_status = "ERROR"
            success = False

        result.success = success
        result.final_status = final_status
        result.total_execution_time_sec = elapsed_total
        result.message = (
            f"BT '{goal.tree_name}' completed: {final_status} "
            f"in {elapsed_total:.2f}s"
            + (f" ({return_message})" if return_message else "")
        )

        self._total_tasks_executed += 1
        self._last_task_result = final_status
        self.get_logger().info(result.message)

        # Final task state
        self._publish_task_state(
            task_id=task_id,
            task_name=goal.tree_name,
            status=final_status,
            current_node="",
            progress=1.0 if success else 0.0,
        )

        # Task-level log event
        if final_status == "HALTED":
            self._publish_log_event(
                "task_cancelled",
                f"Task '{goal.tree_name}' cancelled after {elapsed_total:.1f}s",
                severity="warn",
                task_id=task_id,
            )
            goal_handle.canceled(result)
        elif success:
            self._publish_log_event(
                "task_completed",
                f"Task '{goal.tree_name}' completed in {elapsed_total:.1f}s",
                task_id=task_id,
            )
            goal_handle.succeed(result)
        else:
            self._publish_log_event(
                "task_failed",
                f"Task '{goal.tree_name}' failed after {elapsed_total:.1f}s"
                + (f": {return_message}" if return_message else ""),
                severity="error",
                task_id=task_id,
            )
            goal_handle.succeed(result)

        self._current_task_id = None
        return result

    # ── Feedback handling ────────────────────────────────────────────────────

    def _on_tree_feedback(self, feedback_msg, goal_handle, tree_name,
                          task_id, start_time):
        """Parse feedback from C++ server, publish TaskState and LogEvent."""
        message = feedback_msg.feedback.message
        elapsed = time.time() - start_time

        # Parse "node_name|progress|event" format from RobotTreeServer
        parts = message.split("|", 2) if message else []
        node_name = parts[0] if len(parts) > 0 else ""
        progress = 0.0
        event = ""
        if len(parts) > 1:
            try:
                progress = float(parts[1])
            except ValueError:
                pass
        if len(parts) > 2:
            event = parts[2]

        human_name = humanise(node_name)

        # Publish TaskState
        self._publish_task_state(
            task_id=task_id,
            task_name=tree_name,
            status="RUNNING",
            current_node=node_name,
            progress=progress,
        )

        # Publish skill transition log events
        if event == "started":
            self._publish_log_event(
                "skill_started",
                f"Running: {human_name}",
                severity="debug",
                task_id=task_id,
                skill_name=node_name,
            )
        elif event == "completed":
            self._publish_log_event(
                "skill_completed",
                f"Completed: {human_name}",
                task_id=task_id,
                skill_name=node_name,
            )
        elif event == "failed":
            self._publish_log_event(
                "skill_failed",
                f"Failed: {human_name}",
                severity="error",
                task_id=task_id,
                skill_name=node_name,
            )

        # Forward as ExecuteBehaviorTree feedback
        feedback = ExecuteBehaviorTree.Feedback()
        feedback.current_node_name = node_name
        feedback.tree_status = "RUNNING"
        feedback.elapsed_time_sec = elapsed
        feedback.status_message = (
            f"Running {human_name} ({progress * 100:.0f}%)"
            if node_name else f"Running... ({elapsed:.1f}s)"
        )
        goal_handle.publish_feedback(feedback)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _publish_task_state(
        self,
        task_id: str,
        task_name: str,
        status: str,
        current_node: str,
        progress: float = 0.0,
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
        self._task_state_pub.publish(msg)

    def _publish_log_event(
        self,
        event_name: str,
        message: str,
        severity: str = "info",
        task_id: str = "",
        skill_name: str = "",
    ):
        log_msg = LogEvent()
        log_msg.stamp = self.get_clock().now().to_msg()
        log_msg.event_name = event_name
        log_msg.severity = severity
        log_msg.message = message
        log_msg.task_id = task_id
        log_msg.skill_name = skill_name
        log_msg.tags = ["human"]
        self._log_event_pub.publish(log_msg)

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
