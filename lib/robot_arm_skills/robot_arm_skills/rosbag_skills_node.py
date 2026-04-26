#!/usr/bin/env python3
"""Hosts RecordRosbag + StopRecording action servers in one Python process.

Both atoms share the in-process ``_recorder_registry`` keyed by
``bag_path`` — running them as separate ``Node()`` processes would put
them in separate interpreters with disjoint registries, so they share
this single executable.

The advertise manifest is still published per-action (one
``SkillAdvertisement`` per skill, both inside the same SkillManifest)
because that's what SkillDiscovery expects. Only one is published per
process — wired by the per-robot proxy.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rosbag2_py import RecordOptions, Recorder, StorageOptions

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
            "RecordRosbag (rosbag2_py): action /skill_atoms/record_rosbag"
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
        storage = StorageOptions(uri=bag_path, storage_id=storage_id)
        rec_opts = RecordOptions()
        if goal.topics:
            rec_opts.topics = list(goal.topics)
            rec_opts.all_topics = False
        else:
            rec_opts.all_topics = True
        rec_opts.disable_keyboard_controls = True

        recorder = Recorder()

        # record() blocks for the lifetime of the recording — daemon thread.
        # Recorder.cancel() (called by StopRecording) wakes the blocked
        # thread and lets it return.
        def _run():
            try:
                recorder.record(storage, rec_opts)
            except Exception as e:  # surface in the parent log
                self.get_logger().error(
                    f"Recorder thread for {bag_path} crashed: {e}"
                )

        thread = threading.Thread(
            target=_run, name=f"rosbag-{bag_path}", daemon=True
        )
        thread.start()

        # `--max-duration`-equivalent: stop the recorder via a separate
        # daemon thread once duration_sec elapses.
        if goal.duration_sec > 0.0:
            def _auto_stop():
                time.sleep(goal.duration_sec)
                entry = registry.pop(bag_path)
                if entry is not None:
                    entry.recorder.stop()
            threading.Thread(
                target=_auto_stop,
                name=f"rosbag-autostop-{bag_path}",
                daemon=True,
            ).start()

        registry.register(bag_path, recorder, thread)

        result = RecordRosbag.Result()
        result.bag_path = bag_path
        result.recorded_duration_sec = float(goal.duration_sec)
        result.num_messages = 0
        result.recording_pid = os.getpid()
        result.success = True
        result.message = (
            f"Recording to '{bag_path}' (storage={storage_id})"
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
            "StopRecording (rosbag2_py): action /skill_atoms/stop_recording"
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

        self.get_logger().info(
            f"StopRecording: stop '{goal.bag_path}' (timeout={timeout:.1f}s)"
        )
        entry.recorder.stop()
        entry.thread.join(timeout=timeout)

        if entry.thread.is_alive():
            result.success = False
            result.stopped_pid = goal.recording_pid
            result.message = (
                f"Recorder thread for '{goal.bag_path}' did not exit "
                f"within {timeout:.1f}s after stop()"
            )
            self.get_logger().warning(result.message)
            goal_handle.abort()
            return result

        result.success = True
        result.stopped_pid = goal.recording_pid
        result.message = f"Recording '{goal.bag_path}' stopped"
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
        # Stop any lingering recorders so daemon threads exit cleanly.
        for bag in list(registry.list_bags()):
            entry = registry.pop(bag)
            if entry is not None:
                entry.recorder.stop()
        executor.shutdown()
        record_node.destroy_node()
        stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
