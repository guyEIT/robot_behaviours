#!/usr/bin/env python3
"""FrankaGripperControl skill atom.

Action server: <robot_namespace>/skill_atoms/franka_gripper_control
Action type:    robot_skills_msgs/action/FrankaGripperControl

Translates FrankaGripperControl goals onto upstream franka_gripper's action
endpoints under <gripper_namespace>/{move,grasp,homing}. STOP is handled by
cancelling any in-flight upstream goal — franka_gripper exposes Stop as a
service rather than an action, but cancellation has the same end effect for
the active goal.

In sim mode (parameter `simulate_grasp:=true`) the skill short-circuits and
returns success without contacting franka_gripper, so lab-sim behaviour
trees that exercise the gripper end-to-end don't require a running gripper
node (the sim does not advertise these actions).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_skill_advertise import SkillAdvertiser
from robot_skills_msgs.action import FrankaGripperControl
from robot_skills_msgs.msg import (
    KeyValue,
    SkillAdvertisement,
    SkillDescription,
)

from franka_msgs.action import Grasp, Homing, Move


_VALID_MODES = ("MOVE", "GRASP", "HOMING", "STOP")


@dataclass
class _Inflight:
    handle: object
    mode: str


class FrankaGripperSkillNode(Node):
    def __init__(self) -> None:
        super().__init__("franka_gripper_skill")

        self.declare_parameter("robot_namespace", "/fr3")
        self.declare_parameter("robot_id", "fr3")
        self.declare_parameter("gripper_namespace", "/franka_gripper")
        self.declare_parameter("publish_manifest", True)
        self.declare_parameter("simulate_grasp", False)
        # franka_gripper actions are non-realtime — give them generous timeouts.
        self.declare_parameter("server_wait_timeout_s", 5.0)
        self.declare_parameter("goal_timeout_s", 30.0)

        self._robot_ns = (
            self.get_parameter("robot_namespace").get_parameter_value().string_value
        )
        self._robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        gripper_ns = (
            self.get_parameter("gripper_namespace").get_parameter_value().string_value
        ).rstrip("/")
        self._simulate_grasp = (
            self.get_parameter("simulate_grasp").get_parameter_value().bool_value
        )

        action_path = (self._robot_ns.rstrip("/")
                       + "/skill_atoms/franka_gripper_control")

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FrankaGripperControl,
            action_path,
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        self._move_client = ActionClient(self, Move, f"{gripper_ns}/move",
                                         callback_group=self._cb_group)
        self._grasp_client = ActionClient(self, Grasp, f"{gripper_ns}/grasp",
                                          callback_group=self._cb_group)
        self._homing_client = ActionClient(self, Homing, f"{gripper_ns}/homing",
                                           callback_group=self._cb_group)

        self._inflight_lock = threading.Lock()
        self._inflight: _Inflight | None = None

        if self.get_parameter("publish_manifest").get_parameter_value().bool_value:
            ad = SkillAdvertisement()
            ad.description = SkillDescription(
                name="franka_gripper_control",
                display_name="Franka Gripper Control",
                description=(
                    "Control the Franka Hand via franka_gripper's "
                    "Move/Grasp/Homing actions; STOP cancels the "
                    "in-flight upstream goal."
                ),
                version="1.0.0",
                robot_id=self._robot_id,
                category="manipulation",
                tags=["gripper", "manipulation", "franka"],
                action_server_name=action_path,
                action_type="robot_skills_msgs/action/FrankaGripperControl",
            )
            ad.bt_tag = "FrankaGripperControl"
            ad.goal_defaults = [
                KeyValue(key="mode", value="MOVE"),
                KeyValue(key="width", value="0.04"),
                KeyValue(key="speed", value="0.1"),
                KeyValue(key="force", value="20.0"),
                KeyValue(key="epsilon_inner", value="0.005"),
                KeyValue(key="epsilon_outer", value="0.005"),
            ]
            self._advertiser = SkillAdvertiser(self, [ad])

        self.get_logger().info(
            f"FrankaGripperSkill ready: action={action_path} "
            f"gripper_ns={gripper_ns} simulate_grasp={self._simulate_grasp}"
        )

    def _execute(self, goal_handle):
        request: FrankaGripperControl.Goal = goal_handle.request
        mode = (request.mode or "MOVE").upper()
        result = FrankaGripperControl.Result()

        if mode not in _VALID_MODES:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"Unknown mode {request.mode!r}; expected one of {_VALID_MODES}"
            )
            return result

        if self._simulate_grasp:
            self.get_logger().info(
                f"[sim] FrankaGripperControl mode={mode} short-circuiting to success"
            )
            goal_handle.succeed()
            result.success = True
            result.message = f"sim: {mode} accepted"
            result.final_width = float(request.width) if mode != "HOMING" else 0.04
            result.object_grasped = (mode == "GRASP")
            return result

        if mode == "STOP":
            cancelled = self._cancel_inflight()
            goal_handle.succeed()
            result.success = True
            result.message = (
                "stopped in-flight goal" if cancelled else "no in-flight goal"
            )
            result.final_width = 0.0
            result.object_grasped = False
            return result

        if mode == "MOVE":
            return self._dispatch(
                goal_handle, mode, self._move_client,
                self._build_move_goal(request),
                result,
            )
        if mode == "GRASP":
            return self._dispatch(
                goal_handle, mode, self._grasp_client,
                self._build_grasp_goal(request),
                result,
            )
        # HOMING
        return self._dispatch(
            goal_handle, mode, self._homing_client,
            Homing.Goal(),
            result,
        )

    def _build_move_goal(self, req: FrankaGripperControl.Goal) -> Move.Goal:
        g = Move.Goal()
        g.width = float(req.width)
        g.speed = float(req.speed) if req.speed > 0.0 else 0.1
        return g

    def _build_grasp_goal(self, req: FrankaGripperControl.Goal) -> Grasp.Goal:
        g = Grasp.Goal()
        g.width = float(req.width)
        g.speed = float(req.speed) if req.speed > 0.0 else 0.1
        g.force = float(req.force) if req.force > 0.0 else 20.0
        g.epsilon.inner = float(req.epsilon_inner) if req.epsilon_inner > 0.0 else 0.005
        g.epsilon.outer = float(req.epsilon_outer) if req.epsilon_outer > 0.0 else 0.005
        return g

    def _dispatch(self, goal_handle, mode, client, goal, result):
        timeout = (
            self.get_parameter("server_wait_timeout_s").get_parameter_value().double_value
        )
        if not client.wait_for_server(timeout_sec=timeout):
            goal_handle.abort()
            result.success = False
            result.message = (
                f"franka_gripper {mode.lower()} server unreachable "
                f"after {timeout:g}s"
            )
            return result

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda _f: None)
        # Block until accepted/rejected — rclpy spinning happens via the MT executor.
        send_future_done = threading.Event()
        def _on_send(_f):
            send_future_done.set()
        send_future.add_done_callback(_on_send)
        if not send_future_done.wait(timeout=timeout):
            goal_handle.abort()
            result.success = False
            result.message = f"franka_gripper {mode.lower()} send_goal timed out"
            return result

        upstream_handle = send_future.result()
        if upstream_handle is None or not upstream_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = f"franka_gripper {mode.lower()} rejected goal"
            return result

        with self._inflight_lock:
            self._inflight = _Inflight(handle=upstream_handle, mode=mode)

        try:
            res_future = upstream_handle.get_result_async()
            done = threading.Event()
            res_future.add_done_callback(lambda _f: done.set())
            goal_timeout = (
                self.get_parameter("goal_timeout_s").get_parameter_value().double_value
            )
            if not done.wait(timeout=goal_timeout):
                upstream_handle.cancel_goal_async()
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"franka_gripper {mode.lower()} timed out after {goal_timeout:g}s"
                )
                return result
            wrapped = res_future.result()
            upstream_result = wrapped.result if wrapped is not None else None
        finally:
            with self._inflight_lock:
                self._inflight = None

        if upstream_result is None:
            goal_handle.abort()
            result.success = False
            result.message = f"franka_gripper {mode.lower()} returned no result"
            return result

        result.success = bool(upstream_result.success)
        result.message = upstream_result.error or ""
        result.final_width = float(getattr(upstream_result, "current_width", 0.0))
        result.object_grasped = (mode == "GRASP" and result.success)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _cancel_inflight(self) -> bool:
        with self._inflight_lock:
            inflight = self._inflight
        if inflight is None:
            return False
        try:
            inflight.handle.cancel_goal_async()
            return True
        except Exception as exc:
            self.get_logger().warn(f"cancel of {inflight.mode} failed: {exc}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = FrankaGripperSkillNode()
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
