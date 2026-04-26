"""Meca500 single-process skill server proxy.

Composes the 13 robot_arm_skills components into one process under the
/meca500 namespace, with each atom configured to suppress its individual
SkillManifest. A small Python ManifestPublisher node emits one combined
manifest at /meca500_skill_server/skills.

Per-atom parameters (planning group, controller paths, workspace bounds,
named configs) are read from robots.yaml — the same source of truth used
by skill_atoms_remote.launch.py — so the proxy is parameterised identically
to the legacy fleet.

Phase 4 will collapse this into a C++ proxy that hosts the action servers
directly (no ComponentContainer indirection) and shares one MoveGroupInterface.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _common_params(robot_cfg: dict, robot_id: str, ns: str) -> dict:
    ws = robot_cfg.get("workspace", {})
    return {
        "robot_namespace": ns,
        "robot_id": robot_id,
        "publish_manifest": False,
        "default_planning_group": robot_cfg.get("planning_group", "arm"),
        "default_velocity_scaling": robot_cfg.get("velocity_scaling", 0.3),
        "workspace_min_x": ws.get("min_x", -0.6),
        "workspace_max_x": ws.get("max_x", 0.6),
        "workspace_min_y": ws.get("min_y", -0.6),
        "workspace_max_y": ws.get("max_y", 0.6),
        "workspace_min_z": ws.get("min_z", -0.05),
        "workspace_max_z": ws.get("max_z", 0.7),
    }


def _setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    robots_config_path = LaunchConfiguration("robots_config").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    with open(robots_config_path) as f:
        robots_yaml = yaml.safe_load(f)

    robots = robots_yaml.get("robots", {})
    if robot_id not in robots:
        raise RuntimeError(
            f"Robot {robot_id!r} not in {robots_config_path}; "
            f"available: {sorted(robots.keys())}"
        )
    robot_cfg = robots[robot_id]
    ns = robot_cfg.get("namespace", f"/{robot_id}")
    has_gripper = robot_cfg.get("has_gripper", False)
    arm_controller_action = robot_cfg.get(
        "arm_controller_action",
        "/joint_trajectory_controller/follow_joint_trajectory",
    )
    allowed_named_configs = robot_cfg.get("allowed_named_configs", ["home"])
    check_systems = robot_cfg.get(
        "check_systems", ["move_group", "joint_states"]
    )

    common = _common_params(robot_cfg, robot_id, ns)

    def comp(plugin: str, name: str, extra: dict | None = None) -> ComposableNode:
        params = {**common, **(extra or {})}
        return ComposableNode(
            package="robot_arm_skills",
            plugin=plugin,
            name=f"{robot_id}_{name}",
            parameters=[params],
        )

    composable = [
        comp("robot_arm_skills::MoveToNamedConfigSkill", "move_to_named_config",
             {"allowed_named_configs": allowed_named_configs}),
        comp("robot_arm_skills::MoveToCartesianPoseSkill", "move_to_cartesian_pose"),
        comp("robot_arm_skills::MoveToJointConfigSkill", "move_to_joint_config"),
        comp("robot_arm_skills::MoveCartesianLinearSkill", "move_cartesian_linear",
             {"min_fraction": 0.95}),
        comp("robot_arm_skills::DetectObjectSkill", "detect_object"),
        comp("robot_arm_skills::CapturePointCloudSkill", "capture_point_cloud",
             {"default_camera_topic": "/camera/depth/color/points"}),
        comp("robot_arm_skills::SetDigitalIOSkill", "set_digital_io"),
        comp("robot_arm_skills::CheckCollisionSkill", "check_collision"),
        comp("robot_arm_skills::UpdatePlanningSceneSkill", "update_planning_scene"),
        comp("robot_arm_skills::RobotEnableSkill", "robot_enable", {
            "controllers_to_activate": [
                arm_controller_action.rsplit("/", 1)[-1]
                if "/" in arm_controller_action
                else "joint_trajectory_controller",
            ] + (["gripper_controller"] if has_gripper else []),
        }),
        comp("robot_arm_skills::RecordRosbagSkill", "record_rosbag",
             {"default_output_dir": f"/tmp/rosbags/{robot_id}"}),
    ]

    check_extra = {
        "default_systems": check_systems,
        "move_group_action": "/move_action",
        "arm_controller_action": arm_controller_action,
        "joint_states_topic": "/joint_states",
        "camera_topic": "/camera/color/image_raw",
    }
    if has_gripper:
        check_extra["gripper_controller_action"] = "/gripper_controller/gripper_cmd"
    composable.append(comp(
        "robot_arm_skills::CheckSystemReadySkill", "check_system_ready",
        check_extra,
    ))

    if has_gripper:
        composable.append(comp(
            "robot_arm_skills::GripperControlSkill", "gripper_control",
            {
                "gripper_action_server": "/gripper_controller/gripper_cmd",
                "open_position": 0.0055,
                "default_force_limit": 10.0,
            },
        ))

    container = ComposableNodeContainer(
        name=f"{robot_id}_skill_server_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    manifest_publisher = Node(
        package="meca500_skill_server",
        executable="meca500_manifest_publisher",
        name=f"{robot_id}_skill_server",
        output="screen",
        parameters=[{
            "robot_namespace": ns,
            "robot_id": robot_id,
        }],
    )

    return [container, manifest_publisher]


def generate_launch_description():
    try:
        default_robots_config = os.path.join(
            get_package_share_directory("robot_skill_server"),
            "config", "robots.yaml",
        )
    except Exception:
        default_robots_config = "config/robots.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="meca500",
            description="Robot name from robots.yaml — drives namespace, planning group, controller paths.",
        ),
        DeclareLaunchArgument(
            "robots_config",
            default_value=default_robots_config,
            description="Path to robots.yaml.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS2 log level (debug, info, warn, error).",
        ),
        OpaqueFunction(function=_setup),
    ])
