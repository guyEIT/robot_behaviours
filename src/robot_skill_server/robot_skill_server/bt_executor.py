"""
BtExecutor - Executes BehaviorTree.CPP v4 trees as a ROS2 Action Server.

Provides the ExecuteBehaviorTree action:
  - Accepts BT XML string
  - Loads robot_bt_nodes plugin (C++ BT nodes)
  - Ticks the tree at configured rate
  - Publishes feedback with current node and status
  - Supports Groot2 live monitoring via ZMQ
  - Handles cancellation with graceful halt

Action: /skill_server/execute_behavior_tree (ExecuteBehaviorTree.action)
Topic:  /skill_server/task_state (TaskState.msg)

Note: This Python node spawns a C++ BT executor subprocess for the actual
BT.CPP execution, since BT.CPP is a C++ library. The subprocess communicates
results back via ROS2 topics. Alternatively, use rclpy bindings if available.
"""

from __future__ import annotations

import os
import signal
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
import rclpy.time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus

from std_msgs.msg import String

from robot_skills_msgs.action import ExecuteBehaviorTree
from robot_skills_msgs.msg import TaskState


class BtExecutor(Node):
    """
    ROS2 Action Server that executes BehaviorTree.CPP v4 XML trees.

    Architecture:
    - Python action server handles goal management, cancellation, feedback
    - C++ bt_runner executable (in robot_skill_server) handles actual BT execution
    - Communication via ROS2 topics between Python wrapper and C++ runner
    """

    def __init__(self):
        super().__init__("bt_executor")

        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("groot_zmq_port", 1666)
        self.declare_parameter("bt_runner_executable", "bt_runner")
        self.declare_parameter("bt_shutdown_timeout_s", 3.0)

        callback_group = ReentrantCallbackGroup()
        self._callback_group = callback_group

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
        # Publish active BT XML for the web dashboard (latched via transient_local)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._active_bt_xml_pub = self.create_publisher(
            String, "/skill_server/active_bt_xml", latched_qos
        )

        self._current_task_id: Optional[str] = None
        self._cancel_requested = threading.Event()
        self._task_started_at = None
        self._total_tasks_executed = 0
        self._last_task_result = "IDLE"
        self._bt_runner_available = True

        # Diagnostics
        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID("bt_executor")
        self._diag_updater.add("executor_status", self._produce_diagnostics)

        self.get_logger().info(
            "BtExecutor started on /skill_server/execute_behavior_tree"
        )

    def _goal_callback(self, goal_request):
        """Accept all incoming goals (could add queue logic here)."""
        self.get_logger().info(
            f"Received BT execution request: '{goal_request.tree_name}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info("BT execution cancel requested")
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    async def _execute_bt(self, goal_handle) -> ExecuteBehaviorTree.Result:
        """Execute a behavior tree and publish feedback."""
        goal = goal_handle.request
        result = ExecuteBehaviorTree.Result()
        task_id = "t" + str(uuid.uuid4()).replace("-", "")[:8]
        self._current_task_id = task_id
        self._cancel_requested.clear()
        self._task_started_at = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Executing BT '{goal.tree_name}' (id={task_id})"
        )

        # Publish active BT XML for the web dashboard
        self._active_bt_xml_pub.publish(String(data=goal.tree_xml))

        # Publish initial task state
        self._publish_task_state(
            task_id=task_id,
            task_name=goal.tree_name,
            status="RUNNING",
            current_node="",
        )

        # Write BT XML to a temp file for the C++ runner
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".xml", delete=False, prefix=f"bt_{task_id}_"
        ) as f:
            f.write(goal.tree_xml)
            xml_path = f.name

        try:
            start_time = time.time()

            # Launch the C++ BT runner
            bt_runner_exec = self.get_parameter("bt_runner_executable").value
            tick_rate = self.get_parameter("tick_rate_hz").value
            groot_port = self.get_parameter("groot_zmq_port").value

            cmd = [
                bt_runner_exec,
                "--tree-file", xml_path,
                "--tick-rate", str(tick_rate),
                "--task-id", task_id,
            ]
            if goal.enable_groot_monitor:
                cmd += ["--groot-port", str(groot_port)]

            self.get_logger().info(f"Launching bt_runner: {' '.join(cmd)}")
            self.get_logger().debug(f"BT XML written to: {xml_path}")

            # Track current BT node from the C++ runner's published TaskState.
            # bt_runner publishes to /skill_server/bt_runner_status/<task_id>.
            final_status = "FAILURE"
            current_node_ref: list[str] = ["initializing"]

            def _on_runner_status(msg: TaskState) -> None:
                if msg.current_bt_node:
                    current_node_ref[0] = msg.current_bt_node

            status_sub = self.create_subscription(
                TaskState,
                f"/skill_server/bt_runner_status/{task_id}",
                _on_runner_status,
                10,
                callback_group=self._callback_group,
            )

            # Poll the runner process
            feedback = ExecuteBehaviorTree.Feedback()
            shutdown_timeout = self.get_parameter("bt_shutdown_timeout_s").value

            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    preexec_fn=os.setsid,  # new process group — ensures all children are killed
                )

                while proc.poll() is None:
                    if self._cancel_requested.is_set():
                        try:
                            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                            proc.wait(timeout=shutdown_timeout)
                        except subprocess.TimeoutExpired:
                            self.get_logger().warning(
                                "bt_runner did not terminate gracefully, killing"
                            )
                            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                            proc.wait()
                        self.destroy_subscription(status_sub)
                        result.success = False
                        result.final_status = "HALTED"
                        result.message = "Task cancelled by request"
                        result.total_execution_time_sec = time.time() - start_time
                        goal_handle.canceled(result)
                        return result

                    # Publish feedback
                    elapsed = time.time() - start_time
                    feedback.elapsed_time_sec = elapsed
                    feedback.tree_status = "RUNNING"
                    feedback.current_node_name = current_node_ref[0]
                    feedback.status_message = f"Running... ({elapsed:.1f}s)"
                    goal_handle.publish_feedback(feedback)

                    time.sleep(1.0 / max(tick_rate, 1.0))

                # Process finished — clean up status subscription
                self.destroy_subscription(status_sub)
                stdout, stderr = proc.communicate()
                exit_code = proc.returncode

                if exit_code == 0:
                    final_status = "SUCCESS"
                elif exit_code == 1:
                    final_status = "FAILURE"
                else:
                    final_status = "ERROR"

                if stdout:
                    for line in stdout.splitlines():
                        if line.strip():
                            self.get_logger().debug(f"bt_runner: {line}")
                if stderr:
                    import re
                    _ansi_re = re.compile(r"\x1b\[[0-9;]*m")
                    lines = stderr.splitlines()
                    for line in lines[-50:]:  # tail — most relevant context
                        stripped = line.strip()
                        if not stripped:
                            continue
                        clean = _ansi_re.sub("", stripped)
                        # ROS2 logs go to stderr; route by level
                        if "[ERROR]" in clean:
                            self.get_logger().error(f"bt_runner: {clean}")
                        elif "[WARN]" in clean:
                            self.get_logger().warning(f"bt_runner: {clean}")
                        else:
                            self.get_logger().info(f"bt_runner: {clean}")
                    if len(lines) > 50:
                        self.get_logger().warning(
                            f"bt_runner stderr had {len(lines)} lines; showing last 50"
                        )

            except FileNotFoundError:
                self._bt_runner_available = False
                self.get_logger().error(
                    f"bt_runner executable '{bt_runner_exec}' not found. "
                    "Build the bt_runner target in robot_skill_server: "
                    "colcon build --packages-select robot_skill_server"
                )
                final_status = "ERROR"

            except OSError as e:
                self.get_logger().error(
                    f"Failed to launch bt_runner: {e}"
                )
                final_status = "ERROR"

            elapsed_total = time.time() - start_time

            # Final feedback
            feedback.tree_status = final_status
            feedback.elapsed_time_sec = elapsed_total
            feedback.status_message = f"Completed: {final_status}"
            goal_handle.publish_feedback(feedback)

            # Keep the last BT XML visible — it will be replaced on next execution

            # Publish final task state
            self._publish_task_state(
                task_id=task_id,
                task_name=goal.tree_name,
                status=final_status,
                current_node="",
            )

            result.success = final_status == "SUCCESS"
            result.final_status = final_status
            result.total_execution_time_sec = elapsed_total
            result.message = (
                f"BT '{goal.tree_name}' completed: {final_status} "
                f"in {elapsed_total:.2f}s"
            )

            self._total_tasks_executed += 1
            self._last_task_result = final_status

            self.get_logger().info(result.message)
            goal_handle.succeed(result)
            return result

        except Exception as e:
            # Publish error state if _execute_bt crashes unexpectedly
            self.get_logger().error(f"BT execution crashed: {e}")
            elapsed_total = time.time() - start_time
            self._publish_task_state(
                task_id=task_id,
                task_name=goal.tree_name,
                status="FAILURE",
                current_node="",
                error_message=str(e),
            )
            result.success = False
            result.final_status = "ERROR"
            result.total_execution_time_sec = elapsed_total
            result.message = f"BT execution crashed: {e}"
            self._last_task_result = "ERROR"
            goal_handle.abort(result)
            return result
        finally:
            # Clean up temp file
            Path(xml_path).unlink(missing_ok=True)
            self._current_task_id = None

    def _produce_diagnostics(self, stat):
        """Publish diagnostic status for the BT executor."""
        if not self._bt_runner_available:
            stat.summary(
                DiagnosticStatus.ERROR,
                "bt_runner executable not found - build robot_skill_server",
            )
        elif self._current_task_id:
            stat.summary(DiagnosticStatus.OK, f"Running task {self._current_task_id}")
        elif self._last_task_result == "FAILURE":
            stat.summary(DiagnosticStatus.WARN, "Last task failed")
        else:
            stat.summary(DiagnosticStatus.OK, "Idle")

        stat.add("current_task", self._current_task_id or "none")
        stat.add("total_executed", str(self._total_tasks_executed))
        stat.add("last_result", self._last_task_result)
        stat.add("bt_runner_available", str(self._bt_runner_available))
        return stat

    def _publish_task_state(
        self,
        task_id: str,
        task_name: str,
        status: str,
        current_node: str,
        error_message: str = "",
        error_skill: str = "",
    ):
        """Publish current task state for monitoring."""
        now = self.get_clock().now().to_msg()
        msg = TaskState()
        msg.task_id = task_id
        msg.task_name = task_name
        msg.status = status
        msg.current_skill = current_node
        msg.current_bt_node = current_node
        msg.started_at = self._task_started_at if self._task_started_at else now
        msg.updated_at = now
        msg.elapsed_sec = (
            (self.get_clock().now().nanoseconds -
             rclpy.time.Time.from_msg(msg.started_at).nanoseconds) / 1e9
            if self._task_started_at else 0.0
        )
        msg.completed_skills = []
        msg.failed_skills = []
        msg.error_message = error_message
        msg.error_skill = error_skill
        self._task_state_pub.publish(msg)
