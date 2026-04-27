#!/usr/bin/env python3
"""ROS 2 action-server node wrapping ImagingSim.

Exposes:

  - Action ``~/image_plate`` of type ``robot_skills_msgs/action/ImagePlate``
  - Latched manifest topic so SkillDiscovery picks up the ``ImagePlate`` BT
    tag without a hardcoded entry in ``tree_executor.ACTION_REGISTRY``

Parameters:

  - ``output_root`` (string)        — override default state dir for captures
  - ``delay_per_site_sec`` (float)  — sim cadence per site
  - ``default_site_count`` (int)    — used when goal.site_count == 0
  - ``robot_id`` (string, default ``imaging_sim``) — appears in the skill
    description; lets a downstream BT distinguish between sim and real
    backends.
"""

from __future__ import annotations

import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from robot_skill_advertise import SkillAdvertiser
from robot_skills_msgs.action import ImagePlate
from robot_skills_msgs.msg import (
    KeyValue,
    SkillAdvertisement,
    SkillDescription,
)

from imaging_station.sim_backend import ImagingSim


class ImagingSimNode(Node):
    """Action-server-shaped wrapper over ImagingSim."""

    def __init__(self):
        super().__init__("imaging_station_sim")

        self.declare_parameter("output_root", "")
        self.declare_parameter("delay_per_site_sec", 0.5)
        self.declare_parameter("default_site_count", 3)
        self.declare_parameter("robot_id", "imaging_sim")

        output_root = self.get_parameter("output_root").value
        delay = float(self.get_parameter("delay_per_site_sec").value)
        site_count = int(self.get_parameter("default_site_count").value)

        self._cancel_lock = threading.Lock()
        self._cancel_requested = False

        self._sim = ImagingSim(
            output_root=Path(output_root) if output_root else None,
            delay_per_site_sec=delay,
            default_site_count=site_count,
            halt_check=self._halt_check,
        )

        cbg = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ImagePlate,
            "~/image_plate",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cbg,
        )

        self._publish_skill_manifest()
        self.get_logger().info(
            f"imaging_station sim ready — output_root={self._sim.output_root}, "
            f"delay_per_site={self._sim.delay_per_site_sec}s, "
            f"default_sites={self._sim.default_site_count}"
        )

    # ── action plumbing ───────────────────────────────────────────────────

    def _on_goal(self, _goal):
        # Reset cancel flag for the upcoming execution. The sim accepts one
        # capture at a time; concurrent goals would need queueing (real
        # imagers can't run two captures in parallel anyway).
        with self._cancel_lock:
            self._cancel_requested = False
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        with self._cancel_lock:
            self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _halt_check(self) -> bool:
        with self._cancel_lock:
            return self._cancel_requested

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = ImagePlate.Result()

        def _on_progress(done: int, total: int, stage: str) -> None:
            fb = ImagePlate.Feedback()
            fb.sites_completed = int(done)
            fb.sites_total = int(total)
            fb.current_stage = stage
            try:
                goal_handle.publish_feedback(fb)
            except Exception:
                # Goal handle gone (cancelled or aborted): swallow; the
                # next halt_check tick will exit the capture loop.
                pass

        try:
            capture = self._sim.capture(
                plate_name=goal.plate_name,
                protocol=goal.protocol,
                site_count=int(goal.site_count),
                output_root=Path(goal.output_root) if goal.output_root else None,
                on_progress=_on_progress,
            )
        except Exception as exc:
            self.get_logger().error(f"capture failed: {exc!r}")
            result.success = False
            result.message = f"capture error: {exc}"
            goal_handle.abort()
            return result

        if self._halt_check():
            self.get_logger().info(
                f"capture cancelled mid-run for plate {goal.plate_name!r}"
            )
            result.success = False
            result.message = "cancelled"
            result.image_uris = capture.uris  # whatever we got before halt
            goal_handle.canceled()
            return result

        result.success = True
        result.message = (
            f"captured {len(capture.sites)} site(s) at {capture.metadata['cycle_dir']}"
        )
        result.image_uris = capture.uris
        result.metadata_json = capture.metadata_json
        goal_handle.succeed()
        return result

    # ── skill self-advertisement ──────────────────────────────────────────

    def _publish_skill_manifest(self) -> None:
        ns = self.get_namespace().rstrip("/")
        node_path = f"{ns}/{self.get_name()}" if ns else f"/{self.get_name()}"
        robot_id = self.get_parameter("robot_id").value or "imaging_sim"

        ad = SkillAdvertisement()
        ad.description = SkillDescription(
            name="image_plate",
            display_name="Image Plate",
            description=(
                "Capture an image set of a plate currently held at the "
                "imaging station. Stub sim variant writes placeholder PNGs."
            ),
            version="0.1.0",
            robot_id=robot_id,
            category="perception",
            tags=["imaging", "sim"],
            action_server_name=f"{node_path}/image_plate",
            action_type="robot_skills_msgs/action/ImagePlate",
            idempotent=True,
        )
        ad.bt_tag = "ImagePlate"
        ad.goal_defaults = [
            KeyValue(key="protocol", value=""),
            KeyValue(key="site_count", value="0"),
            KeyValue(key="output_root", value=""),
        ]

        self._skill_advertiser = SkillAdvertiser(self, [ad])


def main(argv=None):
    rclpy.init(args=argv)
    node = ImagingSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
