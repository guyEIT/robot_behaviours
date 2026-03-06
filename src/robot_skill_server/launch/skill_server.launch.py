"""
Robot Skills Framework - Main launch file.

Usage:
  ros2 launch robot_skill_server skill_server.launch.py
  ros2 launch robot_skill_server skill_server.launch.py hardware_mode:=real

Arguments:
  hardware_mode:  'sim' (default) or 'real'
  use_rviz:       'true' (default) or 'false'
  groot_port:     Groot2 ZMQ port (default: 1666)
  robot_ip:       Robot IP for real mode (default: 192.168.0.100)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    hardware_mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value=EnvironmentVariable("ROBOT_HARDWARE_MODE", default_value="sim"),
        description="Hardware mode: 'sim' (fake controllers) or 'real' (physical robot)",
        choices=["sim", "real"],
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )
    groot_port_arg = DeclareLaunchArgument(
        "groot_port",
        default_value="1666",
        description="Groot2 ZMQ publisher port for live BT monitoring",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.100",
        description="IP address of physical Meca500 (used in real mode only)",
    )
    use_realsense_arg = DeclareLaunchArgument(
        "use_realsense",
        default_value="false",
        description="Launch Intel RealSense camera driver",
    )

    hardware_mode = LaunchConfiguration("hardware_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    groot_port = LaunchConfiguration("groot_port")
    use_realsense = LaunchConfiguration("use_realsense")

    # ── Core: Skill Server (Python orchestrator) ──────────────────────────────
    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server",
        output="screen",
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

    # ── Skill Atoms (C++ action servers) ─────────────────────────────────────
    skill_atoms_node = Node(
        package="robot_skill_atoms",
        executable="skills_node",
        name="skill_atoms",
        output="screen",
        parameters=[
            {
                "default_planning_group": "arm",
                "default_velocity_scaling": 0.3,
                "allowed_named_configs": ["home", "ready", "stow", "observe"],
                "gripper_action_server": "/gripper_controller/gripper_cmd",
                # Workspace bounds for Meca500 (500mm reach)
                "workspace_min_x": -0.6,
                "workspace_max_x": 0.6,
                "workspace_min_y": -0.6,
                "workspace_max_y": 0.6,
                "workspace_min_z": -0.05,
                "workspace_max_z": 0.7,
            }
        ],
    )

    # ── MoveIt2 (motion planning) ─────────────────────────────────────────────
    # In practice, include the robot-specific MoveIt2 launch here
    # For now, log a reminder
    moveit_reminder = LogInfo(
        msg="[skill_server.launch] Remember to launch MoveIt2 move_group separately: "
            "ros2 launch <your_robot_moveit_config> move_group.launch.py"
    )

    # ── Intel RealSense camera ────────────────────────────────────────────────
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

    # ── RViz2 ─────────────────────────────────────────────────────────────────
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

    # ── Groot2 reminder ───────────────────────────────────────────────────────
    groot_reminder = LogInfo(
        msg=[
            "\n╔══════════════════════════════════════════════╗\n"
            "║  Groot2 BT Monitor: Run 'groot2' in terminal ║\n"
            "║  Connect to ZMQ port: ", groot_port, "              ║\n"
            "╚══════════════════════════════════════════════╝"
        ]
    )

    return LaunchDescription([
        hardware_mode_arg,
        use_rviz_arg,
        groot_port_arg,
        robot_ip_arg,
        use_realsense_arg,
        # Nodes
        skill_server_node,
        skill_atoms_node,
        moveit_reminder,
        realsense_node,
        rviz_node,
        groot_reminder,
    ])
