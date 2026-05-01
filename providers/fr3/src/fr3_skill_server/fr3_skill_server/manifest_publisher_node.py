#!/usr/bin/env python3
"""Combined SkillManifest publisher for the FR3 proxy.

Mirrors meca500_skill_server.manifest_publisher_node, but the gripper skill
is FrankaGripperControl (mapped to franka_gripper's Move/Grasp/Homing/Stop
actions) instead of the ros2_control GripperControl. The MoveResource /
HandoffTransfer / etc. liquid-handler skills are not part of this manifest.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from robot_skill_advertise import SkillAdvertiser
from robot_skills_msgs.msg import KeyValue, SkillAdvertisement, SkillDescription


ARM_SKILLS = [
    {
        "name": "move_to_named_config",
        "display_name": "Move to Named Configuration",
        "bt_tag": "MoveToNamedConfig",
        "action_type": "robot_skills_msgs/action/MoveToNamedConfig",
        "category": "motion",
        "tags": ["motion", "joint-space", "named"],
        "defaults": [("velocity_scaling", "0.3"), ("acceleration_scaling", "0.3")],
    },
    {
        "name": "move_to_cartesian_pose",
        "display_name": "Move to Cartesian Pose",
        "bt_tag": "MoveToCartesianPose",
        "action_type": "robot_skills_msgs/action/MoveToCartesianPose",
        "category": "motion",
        "tags": ["motion", "cartesian"],
        "defaults": [("velocity_scaling", "0.3"), ("acceleration_scaling", "0.3")],
    },
    {
        "name": "move_to_joint_config",
        "display_name": "Move to Joint Configuration",
        "bt_tag": "MoveToJointConfig",
        "action_type": "robot_skills_msgs/action/MoveToJointConfig",
        "category": "motion",
        "tags": ["motion", "joint-space"],
        "defaults": [("velocity_scaling", "0.3"), ("acceleration_scaling", "0.3")],
    },
    {
        "name": "move_cartesian_linear",
        "display_name": "Move Cartesian Linear",
        "bt_tag": "MoveCartesianLinear",
        "action_type": "robot_skills_msgs/action/MoveCartesianLinear",
        "category": "motion",
        "tags": ["motion", "cartesian", "linear"],
        "defaults": [("velocity_scaling", "0.1"), ("step_size", "0.005")],
    },
    {
        "name": "franka_gripper_control",
        "display_name": "Franka Gripper Control",
        "bt_tag": "FrankaGripperControl",
        "action_type": "robot_skills_msgs/action/FrankaGripperControl",
        "category": "manipulation",
        "tags": ["gripper", "manipulation", "franka"],
        "defaults": [
            ("mode", "MOVE"),
            ("width", "0.04"),
            ("speed", "0.1"),
            ("force", "20.0"),
            ("epsilon_inner", "0.005"),
            ("epsilon_outer", "0.005"),
        ],
    },
    {
        "name": "detect_object",
        "display_name": "Detect Object",
        "bt_tag": "DetectObject",
        "action_type": "robot_skills_msgs/action/DetectObject",
        "category": "perception",
        "tags": ["perception", "vision"],
        "defaults": [("confidence_threshold", "0.7"), ("timeout_sec", "5.0")],
        "output_renames": [("detections", "detected_objects")],
        "post_process": "_detect_object_post",
    },
    {
        "name": "capture_point_cloud",
        "display_name": "Capture Point Cloud",
        "bt_tag": "CapturePointCloud",
        "action_type": "robot_skills_msgs/action/CapturePointCloud",
        "category": "perception",
        "tags": ["perception", "point-cloud"],
        "defaults": [("timeout_sec", "5.0")],
    },
    {
        "name": "set_digital_io",
        "display_name": "Set Digital I/O",
        "bt_tag": "SetDigitalIO",
        "action_type": "robot_skills_msgs/action/SetDigitalIO",
        "category": "io",
        "tags": ["io", "digital", "gpio"],
        "defaults": [],
    },
    {
        "name": "check_collision",
        "display_name": "Check Collision",
        "bt_tag": "CheckCollision",
        "action_type": "robot_skills_msgs/action/CheckCollision",
        "category": "utility",
        "tags": ["safety", "collision"],
        "defaults": [],
    },
    {
        "name": "update_planning_scene",
        "display_name": "Update Planning Scene",
        "bt_tag": "UpdatePlanningScene",
        "action_type": "robot_skills_msgs/action/UpdatePlanningScene",
        "category": "utility",
        "tags": ["planning-scene"],
        "defaults": [("operation", "add"), ("shape_type", "box")],
    },
    {
        "name": "robot_enable",
        "display_name": "Robot Enable",
        "bt_tag": "RobotEnable",
        "action_type": "robot_skills_msgs/action/RobotEnable",
        "category": "utility",
        "tags": ["control", "lifecycle"],
        "defaults": [("enable", "true")],
    },
    {
        "name": "check_system_ready",
        "display_name": "Check System Ready",
        "bt_tag": "CheckSystemReady",
        "action_type": "robot_skills_msgs/action/CheckSystemReady",
        "category": "utility",
        "tags": ["safety", "lifecycle"],
        "defaults": [("timeout_sec", "5.0")],
    },
]


class ManifestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fr3_skill_server")

        self.declare_parameter("robot_namespace", "/fr3")
        self.declare_parameter("robot_id", "fr3")

        ns = self.get_parameter("robot_namespace").get_parameter_value().string_value
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        ns = ns.rstrip("/")

        ads: list[SkillAdvertisement] = []
        for skill in ARM_SKILLS:
            ad = SkillAdvertisement()
            ad.description = SkillDescription(
                name=skill["name"],
                display_name=skill["display_name"],
                description=f"FR3 {skill['display_name'].lower()}",
                version="1.0.0",
                robot_id=robot_id,
                category=skill["category"],
                tags=skill["tags"],
                action_server_name=f"{ns}/skill_atoms/{skill['name']}",
                action_type=skill["action_type"],
            )
            ad.bt_tag = skill["bt_tag"]
            if skill["defaults"]:
                ad.goal_defaults = [
                    KeyValue(key=k, value=v) for k, v in skill["defaults"]
                ]
            if skill.get("output_renames"):
                ad.output_renames = [
                    KeyValue(key=k, value=v) for k, v in skill["output_renames"]
                ]
            if skill.get("post_process"):
                ad.post_process_id = skill["post_process"]
            ads.append(ad)

        self._advertiser = SkillAdvertiser(self, ads)
        self.get_logger().info(
            f"fr3_skill_server published combined manifest with "
            f"{len(ads)} skills under robot_id={robot_id!r}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManifestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
