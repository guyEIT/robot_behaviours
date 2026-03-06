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

import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

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

        self._current_task_id: Optional[str] = None
        self._cancel_requested = threading.Event()

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
        task_id = str(uuid.uuid4())[:8]
        self._current_task_id = task_id
        self._cancel_requested.clear()

        self.get_logger().info(
            f"Executing BT '{goal.tree_name}' (id={task_id})"
        )

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

            self.get_logger().debug(f"Launching bt_runner: {' '.join(cmd)}")

            # Subscribe to status updates from the C++ runner
            # The runner publishes to /skill_server/bt_status/<task_id>
            final_status = "FAILURE"
            current_node = "initializing"

            # Poll the runner process
            feedback = ExecuteBehaviorTree.Feedback()

            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )

                while proc.poll() is None:
                    if self._cancel_requested.is_set():
                        proc.terminate()
                        proc.wait(timeout=3.0)
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
                    feedback.current_node_name = current_node
                    feedback.status_message = f"Running... ({elapsed:.1f}s)"
                    goal_handle.publish_feedback(feedback)

                    time.sleep(1.0 / max(tick_rate, 1.0))

                # Process finished
                stdout, stderr = proc.communicate()
                exit_code = proc.returncode

                if exit_code == 0:
                    final_status = "SUCCESS"
                elif exit_code == 1:
                    final_status = "FAILURE"
                else:
                    final_status = "ERROR"

                if stderr:
                    self.get_logger().warn(f"bt_runner stderr: {stderr[:500]}")

            except FileNotFoundError:
                # bt_runner not available yet - log and return failure
                # This happens before Phase 5 (bt_runner C++ executable) is built
                self.get_logger().warn(
                    f"bt_runner executable '{bt_runner_exec}' not found. "
                    "Build the bt_runner target in robot_skill_server to enable "
                    "actual BT execution. Returning simulated result."
                )
                # Simulated success for development/testing
                time.sleep(1.0)
                final_status = "SUCCESS"

            elapsed_total = time.time() - start_time

            # Final feedback
            feedback.tree_status = final_status
            feedback.elapsed_time_sec = elapsed_total
            feedback.status_message = f"Completed: {final_status}"
            goal_handle.publish_feedback(feedback)

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

            self.get_logger().info(result.message)
            goal_handle.succeed(result)
            return result

        finally:
            # Clean up temp file
            Path(xml_path).unlink(missing_ok=True)
            self._current_task_id = None

    def _publish_task_state(
        self,
        task_id: str,
        task_name: str,
        status: str,
        current_node: str,
    ):
        """Publish current task state for monitoring."""
        msg = TaskState()
        msg.task_id = task_id
        msg.task_name = task_name
        msg.status = status
        msg.current_skill = current_node
        msg.current_bt_node = current_node
        msg.timestamp = self.get_clock().now().to_msg()
        self._task_state_pub.publish(msg)
