"""
Shared launch helpers for the Robot Skills Framework.

Imported by robot_system.launch.py and skill_atoms_remote.launch.py.
"""

from launch_ros.actions import Node


def build_robot_atom_nodes(robot_id: str, robot_cfg: dict, log_level, ns: str) -> list:
    """Return a list of skill atom Node actions for one robot.

    IMPORTANT: We do NOT set the ROS2 `namespace` on these nodes. MoveGroupInterface
    and other MoveIt2 connections use absolute topic names (e.g. /move_group,
    /joint_states). Pushing the node into a namespace would break those connections.
    Instead, the `robot_namespace` ROS parameter is used by SkillBase to prefix
    only the skill atom's own action server name (e.g. /meca500/skill_atoms/...).

    Args:
        robot_id:  Registry identifier, e.g. "meca500" or "franka"
        robot_cfg: Dict entry from robots.yaml for this robot
        log_level: Log level string or LaunchConfiguration
        ns:        Robot namespace string, e.g. "/meca500" or "/franka"
                   Used for robot_namespace param (action server prefix) only.
    """
    planning_group = robot_cfg.get("planning_group", "arm")
    velocity_scaling = robot_cfg.get("velocity_scaling", 0.3)
    ws = robot_cfg.get("workspace", {})
    has_gripper = robot_cfg.get("has_gripper", False)
    arm_controller_action = robot_cfg.get(
        "arm_controller_action",
        "/joint_trajectory_controller/follow_joint_trajectory",
    )
    allowed_named_configs = robot_cfg.get("allowed_named_configs", ["home"])
    check_systems = robot_cfg.get(
        "check_systems", ["move_group", "joint_states"]
    )

    # Parameters injected into every skill atom for this robot.
    # `robot_namespace` prefixes the skill's own action server (in SkillBase).
    # `robot_id` is included in the registration with SkillRegistry.
    common_params = {
        "robot_namespace": ns,
        "robot_id": robot_id,
        "default_planning_group": planning_group,
        "default_velocity_scaling": velocity_scaling,
        "workspace_min_x": ws.get("min_x", -0.6),
        "workspace_max_x": ws.get("max_x", 0.6),
        "workspace_min_y": ws.get("min_y", -0.6),
        "workspace_max_y": ws.get("max_y", 0.6),
        "workspace_min_z": ws.get("min_z", -0.05),
        "workspace_max_z": ws.get("max_z", 0.7),
    }

    def atom(executable, name_suffix, extra_params=None):
        return Node(
            package="robot_skill_atoms",
            executable=executable,
            name=f"{robot_id}_{name_suffix}",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{**common_params, **(extra_params or {})}],
        )

    nodes = [
        atom("move_to_named_config_skill_node", "move_to_named_config_skill",
             {"allowed_named_configs": allowed_named_configs}),
        atom("move_to_cartesian_pose_skill_node", "move_to_cartesian_pose_skill"),
        atom("move_to_joint_config_skill_node",  "move_to_joint_config_skill"),
        atom("move_cartesian_linear_skill_node",  "move_cartesian_linear_skill",
             {"min_fraction": 0.95}),
        atom("detect_object_skill_node",          "detect_object_skill"),
        atom("update_planning_scene_skill_node",  "update_planning_scene_skill"),
        atom("check_collision_skill_node",        "check_collision_skill"),
        atom("robot_enable_skill_node",           "robot_enable_skill",
             {"controllers_to_activate": [
                 arm_controller_action.rsplit("/", 1)[-1]
                 if "/" in arm_controller_action
                 else "joint_trajectory_controller"
             ]}),
        atom("capture_point_cloud_skill_node", "capture_point_cloud_skill",
             {"default_camera_topic": "/camera/depth/color/points"}),
        atom("set_digital_io_skill_node",  "set_digital_io_skill"),
        atom("record_rosbag_skill_node",   "record_rosbag_skill",
             {"default_output_dir": f"/tmp/rosbags/{robot_id}"}),
    ]

    # CheckSystemReady — topic paths point at the robot's MoveIt2 instance
    # (at root namespace on whichever PC runs MoveIt2 for this robot)
    check_params = {
        **common_params,
        "default_systems": check_systems,
        "move_group_action":    "/move_action",
        "arm_controller_action": arm_controller_action,
        "joint_states_topic":   "/joint_states",
        "camera_topic":         "/camera/color/image_raw",
    }
    if has_gripper:
        check_params["gripper_controller_action"] = "/gripper_controller/gripper_cmd"

    nodes.append(Node(
        package="robot_skill_atoms",
        executable="check_system_ready_skill_node",
        name=f"{robot_id}_check_system_ready_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[check_params],
    ))

    if has_gripper:
        nodes.append(atom(
            "gripper_control_skill_node", "gripper_control_skill",
            {"gripper_action_server": "/gripper_controller/gripper_cmd"},
        ))

    return nodes
