#!/usr/bin/env python3
"""Hosts RecordRosbag + StopRecording action servers in one Python process.

Records via ``ros2 bag record`` subprocesses (not in-process
``rosbag2_py.Recorder``): the C++ recorder calls ``rclcpp::init()``
which raises "context is already initialized" the second time it is
constructed inside an existing rclpy interpreter. A subprocess per
recording sidesteps that — each one has its own context.

Both atoms share the in-process ``_recorder_registry`` keyed by
``bag_path`` so StopRecording can find the PID started by RecordRosbag.
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_skills_msgs.action import RecordRosbag, StopRecording

from robot_arm_skills import _recorder_registry as registry


class RecordRosbagNode(Node):
    def __init__(self) -> None:
        super().__init__("record_rosbag_skill")
        self.declare_parameter("default_output_dir", "/tmp/rosbags")
        self.declare_parameter("default_storage_id", "mcap")

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            RecordRosbag,
            "/skill_atoms/record_rosbag",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "RecordRosbag (ros2 bag record): action /skill_atoms/record_rosbag"
        )

    def _execute(self, goal_handle):
        goal = goal_handle.request

        out_dir = (
            self.get_parameter("default_output_dir")
            .get_parameter_value().string_value
        )
        bag_path = (
            goal.output_path if goal.output_path
            else f"{out_dir}/recording_{int(time.time())}"
        )
        Path(bag_path).parent.mkdir(parents=True, exist_ok=True)

        storage_id = (
            self.get_parameter("default_storage_id")
            .get_parameter_value().string_value
        )

        cmd = ["ros2", "bag", "record", "-o", bag_path, "-s", storage_id]
        if goal.topics:
            cmd += list(goal.topics)
        else:
            cmd += ["-a"]

        # `--max-duration`-equivalent: ros2 bag record has --max-duration
        # in seconds (--max-duration 60.0). Use it when the goal sets one.
        if goal.duration_sec > 0.0:
            cmd += ["--max-duration", f"{goal.duration_sec:g}"]

        # New process group so SIGINT to the group reaches all child
        # processes (storage plugins, message converters) cleanly.
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

        registry.register(bag_path, process)

        result = RecordRosbag.Result()
        result.bag_path = bag_path
        result.recorded_duration_sec = float(goal.duration_sec)
        result.num_messages = 0
        result.recording_pid = process.pid
        result.success = True
        result.message = (
            f"Recording to '{bag_path}' (storage={storage_id}, pid={process.pid})"
            + (
                f" — auto-stops after {goal.duration_sec:g}s"
                if goal.duration_sec > 0.0 else " — until StopRecording"
            )
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result


class StopRecordingNode(Node):
    def __init__(self) -> None:
        super().__init__("stop_recording_skill")
        self.declare_parameter("default_timeout_sec", 5.0)

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            StopRecording,
            "/skill_atoms/stop_recording",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "StopRecording (ros2 bag record): action /skill_atoms/stop_recording"
        )

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = StopRecording.Result()

        if not goal.bag_path:
            result.success = False
            result.message = "bag_path is required to stop a recorder"
            goal_handle.abort()
            return result

        entry = registry.pop(goal.bag_path)
        if entry is None:
            result.success = True
            result.message = (
                f"No active recorder for bag_path={goal.bag_path!r}"
            )
            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        timeout = (
            goal.timeout_sec if goal.timeout_sec > 0.0
            else self.get_parameter("default_timeout_sec")
            .get_parameter_value().double_value
        )

        proc = entry.process
        self.get_logger().info(
            f"StopRecording: SIGINT pid={proc.pid} "
            f"(bag={goal.bag_path}, timeout={timeout:.1f}s)"
        )

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            # Already exited.
            pass

        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                f"Recorder pid={proc.pid} did not exit within "
                f"{timeout:.1f}s after SIGINT — escalating to SIGKILL"
            )
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

        result.success = True
        result.stopped_pid = proc.pid
        result.message = (
            f"Recording '{goal.bag_path}' stopped (pid={proc.pid})"
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    record_node = RecordRosbagNode()
    stop_node = StopRecordingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(record_node)
    executor.add_node(stop_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop any lingering recorders so daemon subprocesses exit cleanly.
        for bag in list(registry.list_bags()):
            entry = registry.pop(bag)
            if entry is not None:
                try:
                    os.killpg(os.getpgid(entry.process.pid), signal.SIGINT)
                    entry.process.wait(timeout=3.0)
                except Exception:
                    pass
        executor.shutdown()
        record_node.destroy_node()
        stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
