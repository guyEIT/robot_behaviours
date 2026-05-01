"""FR3 single-process skill server proxy.

Mirrors meca500_skill_server.launch.py: composes the 12 robot_arm_skills
components (no GripperControlSkill) under the /fr3 namespace, plus a
FrankaGripperSkill atom that maps onto franka_gripper's Move/Grasp/Homing/
Stop actions. A single combined SkillManifest is published at
/fr3_skill_server/skills.

Per-atom parameters come from robots.yaml (robot_id="fr3" or
"fr3_lab_sim"), so the proxy is parameterised identically to the legacy
fleet.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def _common_params(robot_cfg: dict, robot_id: str, ns: str) -> dict:
    ws = robot_cfg.get("workspace", {})
    return {
        "robot_namespace": ns,
        "robot_id": robot_id,
        "publish_manifest": False,
        "default_planning_group": robot_cfg.get("planning_group", "fr3_arm"),
        "default_velocity_scaling": robot_cfg.get("velocity_scaling", 0.3),
        "workspace_min_x": ws.get("min_x", -0.85),
        "workspace_max_x": ws.get("max_x", 0.85),
        "workspace_min_y": ws.get("min_y", -0.85),
        "workspace_max_y": ws.get("max_y", 0.85),
        "workspace_min_z": ws.get("min_z", 0.0),
        "workspace_max_z": ws.get("max_z", 1.2),
    }


def _setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    robots_config_path = LaunchConfiguration("robots_config").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    namespace_prefix = LaunchConfiguration("namespace_prefix").perform(context)

    with open(robots_config_path) as f:
        robots_yaml = yaml.safe_load(f)

    robots = robots_yaml.get("robots", {})
    if robot_id not in robots:
        raise RuntimeError(
            f"Robot {robot_id!r} not in {robots_config_path}; "
            f"available: {sorted(robots.keys())}"
        )
    robot_cfg = robots[robot_id]
    base_ns = robot_cfg.get("namespace", f"/{robot_id}")
    if namespace_prefix:
        if not namespace_prefix.startswith("/"):
            namespace_prefix = "/" + namespace_prefix
        ns = namespace_prefix + base_ns
    else:
        ns = base_ns
    has_gripper = robot_cfg.get("has_gripper", True)
    arm_controller_action = robot_cfg.get(
        "arm_controller_action",
        "/fr3_arm_controller/follow_joint_trajectory",
    )
    allowed_named_configs = robot_cfg.get("allowed_named_configs", ["home", "ready"])
    check_systems = robot_cfg.get(
        "check_systems", ["move_group", "joint_states"]
    )
    sim_perception = bool(robot_cfg.get("simulate_perception", False))
    simulate_grasp = bool(robot_cfg.get("simulate_perception", False))

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
        comp("robot_arm_skills::DetectObjectSkill", "detect_object",
             {"simulate_perception": sim_perception}),
        comp("robot_arm_skills::CapturePointCloudSkill", "capture_point_cloud",
             {"default_camera_topic": "/camera/depth/color/points",
              "simulate_perception": sim_perception}),
        comp("robot_arm_skills::SetDigitalIOSkill", "set_digital_io"),
        comp("robot_arm_skills::CheckCollisionSkill", "check_collision"),
        comp("robot_arm_skills::UpdatePlanningSceneSkill", "update_planning_scene"),
        comp("robot_arm_skills::RobotEnableSkill", "robot_enable", {
            "controllers_to_activate": ["fr3_arm_controller"],
        }),
    ]

    check_extra = {
        "default_systems": check_systems,
        "move_group_action": "/move_action",
        "arm_controller_action": arm_controller_action,
        "joint_states_topic": "/joint_states",
        "camera_topic": "/camera/color/image_raw",
    }
    composable.append(comp(
        "robot_arm_skills::CheckSystemReadySkill", "check_system_ready",
        check_extra,
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

    # FrankaGripperSkill is a Python ament-python package — runs as a
    # standalone Node, not inside the C++ component container.
    franka_gripper_skill = None
    if has_gripper:
        franka_gripper_skill = Node(
            package="franka_gripper_skill",
            executable="franka_gripper_skill_node",
            name=f"{robot_id}_franka_gripper_control",
            output="screen",
            parameters=[{
                "robot_namespace": ns,
                "robot_id": robot_id,
                "gripper_namespace": "/franka_gripper",
                "simulate_grasp": simulate_grasp,
                # Combined manifest is published by fr3_manifest_publisher;
                # suppress the per-atom manifest so SkillDiscovery sees one entry.
                "publish_manifest": False,
            }],
        )

    manifest_publisher = Node(
        package="fr3_skill_server",
        executable="fr3_manifest_publisher",
        name=f"{robot_id}_skill_server",
        output="screen",
        parameters=[{
            "robot_namespace": ns,
            "robot_id": robot_id,
        }],
    )

    rosbag_skills = Node(
        package="robot_arm_skills",
        executable="rosbag_skills_node",
        name=f"{robot_id}_rosbag_skills",
        output="screen",
        parameters=[{
            "default_output_dir": f"/tmp/rosbags/{robot_id}",
        }],
    )

    actions = [container, manifest_publisher, rosbag_skills]
    if franka_gripper_skill is not None:
        actions.append(franka_gripper_skill)
    if namespace_prefix:
        return [GroupAction([PushRosNamespace(namespace_prefix), *actions])]
    return actions


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
            default_value="fr3",
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
        DeclareLaunchArgument(
            "namespace_prefix",
            default_value="",
            description=(
                "Top-level namespace prefix for the skill server + atoms. "
                "Default empty = real-mode paths."
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
