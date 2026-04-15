"""
Robot Skills Framework - Main launch file.

Usage:
  ros2 launch robot_skill_server skill_server.launch.py
  ros2 launch robot_skill_server skill_server.launch.py hardware_mode:=real
  ros2 launch robot_skill_server skill_server.launch.py log_level:=debug

Arguments:
  hardware_mode:  'sim' (default) or 'real'
  use_rviz:       'true' (default) or 'false'
  use_rosboard:   'true' (default) or 'false'
  log_level:      'debug', 'info' (default), 'warn', 'error'
  groot_port:     Groot2 ZMQ port (default: 1666)
  robot_ip:       Robot IP for real mode (default: 10.6.105.53)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


# ── Robot-specific presets ────────────────────────────────────────────────────
# Each preset maps hardware_mode → planning group, controller names, etc.
ROBOT_PRESETS = {
    "real": {
        "planning_group": "meca500_arm",
        "velocity_scaling": 0.1,
        "arm_controller_action": "/joint_trajectory_controller/follow_joint_trajectory",
        "allowed_named_configs": ["home"],
        "has_gripper": False,
        "check_systems": ["move_group", "joint_states", "joint_trajectory_controller"],
    },
    "sim": {
        "planning_group": "arm",
        "velocity_scaling": 0.3,
        "arm_controller_action": "/arm_controller/follow_joint_trajectory",
        "allowed_named_configs": ["home", "ready", "stow", "observe"],
        "has_gripper": True,
        "check_systems": [
            "move_group", "joint_states",
            "arm_controller", "gripper_controller",
        ],
    },
}


def _setup_nodes(context, *args, **kwargs):
    """Resolve hardware_mode at launch time and return robot-specific nodes."""
    hw_mode = LaunchConfiguration("hardware_mode").perform(context)
    log_level = LaunchConfiguration("log_level")
    groot_port = LaunchConfiguration("groot_port")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rosboard = LaunchConfiguration("use_rosboard")
    use_realsense = LaunchConfiguration("use_realsense")

    preset = ROBOT_PRESETS[hw_mode]

    # ── Core: Skill Server (Python orchestrator) ──────────────────────────
    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "groot_zmq_port": groot_port,
                "tick_rate_hz": 10.0,
                "bt_runner_executable": PathJoinSubstitution([
                    FindPackageShare("robot_skill_server"), "..", "..",
                    "lib", "robot_skill_server", "bt_runner"
                ]),
            }
        ],
    )

    # ── Shared parameters for motion skill atoms ─────────────────────────
    motion_params = {
        "default_planning_group": preset["planning_group"],
        "default_velocity_scaling": preset["velocity_scaling"],
        # Workspace bounds (500 mm reach robot)
        "workspace_min_x": -0.6,
        "workspace_max_x": 0.6,
        "workspace_min_y": -0.6,
        "workspace_max_y": 0.6,
        "workspace_min_z": -0.05,
        "workspace_max_z": 0.7,
    }

    # ── Skill Atoms ──────────────────────────────────────────────────────
    move_to_named_config_node = Node(
        package="robot_skill_atoms",
        executable="move_to_named_config_skill_node",
        name="move_to_named_config_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            **motion_params,
            "allowed_named_configs": preset["allowed_named_configs"],
        }],
    )

    move_to_cartesian_pose_node = Node(
        package="robot_skill_atoms",
        executable="move_to_cartesian_pose_skill_node",
        name="move_to_cartesian_pose_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[motion_params],
    )

    detect_object_node = Node(
        package="robot_skill_atoms",
        executable="detect_object_skill_node",
        name="detect_object_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    move_to_joint_config_node = Node(
        package="robot_skill_atoms",
        executable="move_to_joint_config_skill_node",
        name="move_to_joint_config_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[motion_params],
    )

    move_cartesian_linear_node = Node(
        package="robot_skill_atoms",
        executable="move_cartesian_linear_skill_node",
        name="move_cartesian_linear_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            **motion_params,
            "min_fraction": 0.95,
        }],
    )

    update_planning_scene_node = Node(
        package="robot_skill_atoms",
        executable="update_planning_scene_skill_node",
        name="update_planning_scene_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    check_collision_node = Node(
        package="robot_skill_atoms",
        executable="check_collision_skill_node",
        name="check_collision_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"default_planning_group": preset["planning_group"]}],
    )

    robot_enable_node = Node(
        package="robot_skill_atoms",
        executable="robot_enable_skill_node",
        name="robot_enable_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "controllers_to_activate": ["joint_trajectory_controller"],
        }],
    )

    capture_point_cloud_node = Node(
        package="robot_skill_atoms",
        executable="capture_point_cloud_skill_node",
        name="capture_point_cloud_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "default_camera_topic": "/camera/depth/color/points",
        }],
    )

    set_digital_io_node = Node(
        package="robot_skill_atoms",
        executable="set_digital_io_skill_node",
        name="set_digital_io_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    record_rosbag_node = Node(
        package="robot_skill_atoms",
        executable="record_rosbag_skill_node",
        name="record_rosbag_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "default_output_dir": "/tmp/rosbags",
        }],
    )

    # ── CheckSystemReady ─────────────────────────────────────────────────
    check_params = {
        "default_systems": preset["check_systems"],
        "move_group_action": "/move_action",
        "arm_controller_action": preset["arm_controller_action"],
        "joint_states_topic": "/joint_states",
        "camera_topic": "/camera/color/image_raw",
    }
    if preset["has_gripper"]:
        check_params["gripper_controller_action"] = "/gripper_controller/gripper_cmd"

    check_system_ready_node = Node(
        package="robot_skill_atoms",
        executable="check_system_ready_skill_node",
        name="check_system_ready_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[check_params],
    )

    # ── Diagnostic Aggregator ────────────────────────────────────────────
    diag_aggregator_config = PathJoinSubstitution([
        FindPackageShare("robot_skill_server"),
        "config",
        "diagnostics_aggregator.yaml",
    ])

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[diag_aggregator_config],
    )

    # ── Intel RealSense camera ───────────────────────────────────────────
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        output="screen",
        condition=IfCondition(use_realsense),
        parameters=[
            {
                "depth_width": 640,
                "depth_height": 480,
                "depth_fps": 30.0,
                "color_width": 640,
                "color_height": 480,
                "color_fps": 30.0,
                "enable_pointcloud": True,
                "pointcloud_texture_stream": "RS2_STREAM_COLOR",
                "align_depth.enable": True,
            }
        ],
    )

    # ── Rosboard web UI ──────────────────────────────────────────────────
    rosboard_process = ExecuteProcess(
        cmd=["rosboard_node", "--port", "8888"],
        output="screen",
        condition=IfCondition(use_rosboard),
    )

    nodes = [
        skill_server_node,
        move_to_named_config_node,
        move_to_cartesian_pose_node,
        detect_object_node,
        move_to_joint_config_node,
        move_cartesian_linear_node,
        update_planning_scene_node,
        check_collision_node,
        robot_enable_node,
        capture_point_cloud_node,
        set_digital_io_node,
        record_rosbag_node,
        check_system_ready_node,
        diagnostic_aggregator_node,
        realsense_node,
        rosboard_process,
    ]

    if hw_mode == "real":
        # ── Real mode: Robot Dashboard (rosbridge + web UI) ──────────
        dashboard_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("robot_dashboard"),
                    "launch",
                    "dashboard.launch.py",
                ])
            ),
        )
        nodes.append(dashboard_launch)

        monitoring_info = LogInfo(
            msg=[
                "\n╔══════════════════════════════════════════════════════╗\n"
                "║  Hardware mode:    ", hw_mode, "                          ║\n"
                "║  Planning group:   ", preset["planning_group"], "                    ║\n"
                "║  Dashboard:        http://localhost:8081              ║\n"
                "║  Rosboard Web UI:  http://localhost:8888              ║\n"
                "║  Log level:        ", log_level, "                          ║\n"
                "╚══════════════════════════════════════════════════════════╝"
            ]
        )
    else:
        # ── Sim mode: RViz + Groot2 ──────────────────────────────────
        rviz_config = PathJoinSubstitution([
            FindPackageShare("robot_skill_server"),
            "config",
            "skill_server.rviz",
        ])
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        )
        groot2_process = ExecuteProcess(
            cmd=["groot2", "--appimage-extract-and-run"],
            output="screen",
        )
        nodes.extend([rviz_node, groot2_process])

        monitoring_info = LogInfo(
            msg=[
                "\n╔══════════════════════════════════════════════════════╗\n"
                "║  Hardware mode:    ", hw_mode, "                          ║\n"
                "║  Planning group:   ", preset["planning_group"], "                    ║\n"
                "║  Rosboard Web UI:  http://localhost:8888              ║\n"
                "║  Groot2 ZMQ port:  ", groot_port, "                          ║\n"
                "║  Log level:        ", log_level, "                          ║\n"
                "╚══════════════════════════════════════════════════════════╝"
            ]
        )

    nodes.append(monitoring_info)

    # Only launch gripper node when the robot has a gripper
    if preset["has_gripper"]:
        gripper_control_node = Node(
            package="robot_skill_atoms",
            executable="gripper_control_skill_node",
            name="gripper_control_skill",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{
                "gripper_action_server": "/gripper_controller/gripper_cmd",
            }],
        )
        nodes.append(gripper_control_node)

    return nodes


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    hardware_mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value=EnvironmentVariable("ROBOT_HARDWARE_MODE", default_value="real"),
        description="Hardware mode: 'sim' (fake controllers) or 'real' (physical robot)",
        choices=["sim", "real"],
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )
    use_rosboard_arg = DeclareLaunchArgument(
        "use_rosboard",
        default_value="true",
        description="Launch rosboard web UI on port 8888",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS2 log level: debug, info, warn, error",
    )
    groot_port_arg = DeclareLaunchArgument(
        "groot_port",
        default_value="1666",
        description="Groot2 ZMQ publisher port for live BT monitoring",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.6.105.53",
        description="IP address of physical Meca500 (used in real mode only)",
    )
    use_realsense_arg = DeclareLaunchArgument(
        "use_realsense",
        default_value="false",
        description="Launch Intel RealSense camera driver",
    )

    return LaunchDescription([
        hardware_mode_arg,
        use_rviz_arg,
        use_rosboard_arg,
        log_level_arg,
        groot_port_arg,
        robot_ip_arg,
        use_realsense_arg,
        OpaqueFunction(function=_setup_nodes),
    ])
