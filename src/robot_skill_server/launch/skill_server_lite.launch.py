"""
Robot Skills Framework - Lightweight simulation launch file.

Launches mock skill atoms instead of real MoveIt2-dependent ones.
Single container, no MoveIt2, no ros2_control, no RealSense.

Usage:
  ros2 launch robot_skill_server skill_server_lite.launch.py
  ros2 launch robot_skill_server skill_server_lite.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS2 log level: debug, info, warn, error",
    )
    use_rosboard_arg = DeclareLaunchArgument(
        "use_rosboard",
        default_value="false",
        description="Launch rosboard web UI on port 8888",
    )
    use_dashboard_arg = DeclareLaunchArgument(
        "use_dashboard",
        default_value="true",
        description="Launch web dashboard",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
        description="rosbridge WebSocket port",
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port",
        default_value=os.environ.get("DASHBOARD_PORT", "8080"),
        description="Dashboard HTTP port",
    )

    log_level = LaunchConfiguration("log_level")
    use_rosboard = LaunchConfiguration("use_rosboard")
    use_dashboard = LaunchConfiguration("use_dashboard")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    dashboard_port = LaunchConfiguration("dashboard_port")

    # ── Mock params ──────────────────────────────────────────────────────────
    mock_params = PathJoinSubstitution([
        FindPackageShare("robot_mock_skill_atoms"), "config", "mock_params.yaml"
    ])

    # ── Core: Skill Server (Python orchestrator + tree executor) ───────────
    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "execute_tree_action_name": "/skill_server/execute_tree",
        }],
    )

    # ── bb_operator sidecar — operator-facing services that mutate the
    #    persistent blackboard of the active long-lived BT task. Stays up
    #    even when no campaign is running so /bb_operator/* is always
    #    discoverable. Services no-op (return success=False) until a long
    #    tree is running and registered.
    bb_operator_node = Node(
        package="robot_skill_server",
        executable="bb_operator_node",
        name="bb_operator",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── imaging_station sim — exposes `ImagePlate` for the campaign tree
    #    so a full Liconic ↔ Hamilton ↔ Imager ↔ Hamilton ↔ Liconic loop
    #    runs in lite sim. Writes placeholder PNGs under
    #    ~/.local/state/imaging_station/. Replace with the real driver
    #    when imaging hardware is in.
    imaging_station_sim_node = Node(
        package="imaging_station",
        executable="imaging_station_sim_node",
        name="imaging_station_sim",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "delay_per_site_sec": 0.5,
            "default_site_count": 3,
            "robot_id": "imaging_sim",
        }],
    )

    # ── Mock Skill Atoms ─────────────────────────────────────────────────────
    mock_move_to_named_config = Node(
        package="robot_mock_skill_atoms",
        executable="mock_move_to_named_config_node",
        name="mock_move_to_named_config_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_move_to_cartesian_pose = Node(
        package="robot_mock_skill_atoms",
        executable="mock_move_to_cartesian_pose_node",
        name="mock_move_to_cartesian_pose_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_gripper_control = Node(
        package="robot_mock_skill_atoms",
        executable="mock_gripper_control_node",
        name="mock_gripper_control_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_detect_object = Node(
        package="robot_mock_skill_atoms",
        executable="mock_detect_object_node",
        name="mock_detect_object_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_move_to_joint_config = Node(
        package="robot_mock_skill_atoms",
        executable="mock_move_to_joint_config_node",
        name="mock_move_to_joint_config_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_move_cartesian_linear = Node(
        package="robot_mock_skill_atoms",
        executable="mock_move_cartesian_linear_node",
        name="mock_move_cartesian_linear_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_capture_point_cloud = Node(
        package="robot_mock_skill_atoms",
        executable="mock_capture_point_cloud_node",
        name="mock_capture_point_cloud_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_set_digital_io = Node(
        package="robot_mock_skill_atoms",
        executable="mock_set_digital_io_node",
        name="mock_set_digital_io_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_check_collision = Node(
        package="robot_mock_skill_atoms",
        executable="mock_check_collision_node",
        name="mock_check_collision_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_update_planning_scene = Node(
        package="robot_mock_skill_atoms",
        executable="mock_update_planning_scene_node",
        name="mock_update_planning_scene_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_robot_enable = Node(
        package="robot_mock_skill_atoms",
        executable="mock_robot_enable_node",
        name="mock_robot_enable_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_record_rosbag = Node(
        package="robot_mock_skill_atoms",
        executable="mock_record_rosbag_node",
        name="mock_record_rosbag_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    mock_check_system_ready = Node(
        package="robot_mock_skill_atoms",
        executable="mock_check_system_ready_node",
        name="mock_check_system_ready_skill",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params],
    )

    # ── Mock Joint State Publisher ───────────────────────────────────────────
    # Drives the Meca500 6-DoF arm by default — the dashboard renders the
    # actual customer-facing arm rather than a placeholder Panda. The mock
    # atoms below stay robot-agnostic; only the displayed kinematic chain
    # changes. To switch back to a Panda layout, override `joint_names` /
    # `initial_positions` / `arm_joint_count` on the launch CLI.
    meca500_joint_params = {
        "publish_rate": 50.0,
        "interpolation_speed": 1.0,
        "joint_names": [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
        ],
        # All-zero home — matches the Meca500's "home" named pose where
        # axes align vertically. Operators can land any other named config
        # via the mock_move_to_named_config skill.
        "initial_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "arm_joint_count": 6,
    }

    mock_joint_state_pub = Node(
        package="robot_mock_skill_atoms",
        executable="mock_joint_state_publisher_node",
        name="mock_joint_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[mock_params, meca500_joint_params],
    )

    # ── Static TF: world -> meca_base_link ───────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "meca_base_link",
        ],
    )

    # ── Robot State Publisher (joint_states -> TF) ─────────────────────────
    # The Meca500 .xacro is plain URDF (no xacro features) so we can read it
    # straight from disk. Meshes are resolved via package:// — robot_state_
    # publisher needs `meca500_description` to be discoverable in the env,
    # which the lite-native feature in pixi.toml ensures.
    meca500_urdf = os.path.join(
        get_package_share_directory("meca500_description"),
        "urdf", "meca500.xacro",
    )
    with open(meca500_urdf, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ── Diagnostic Aggregator ────────────────────────────────────────────────
    diag_aggregator_config = PathJoinSubstitution([
        FindPackageShare("robot_skill_server"), "config", "diagnostics_aggregator.yaml",
    ])

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[diag_aggregator_config],
    )

    # ── Rosboard web UI ──────────────────────────────────────────────────────
    rosboard_process = ExecuteProcess(
        cmd=["rosboard_node", "--port", "8888"],
        output="screen",
        condition=IfCondition(use_rosboard),
    )

    # ── Web Dashboard ───────────────────────────────────────────────────────
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_dashboard"), "launch", "dashboard.launch.py"
            ])
        ),
        launch_arguments={
            "rosbridge_port": rosbridge_port,
            "dashboard_port": dashboard_port,
        }.items(),
        condition=IfCondition(use_dashboard),
    )

    # ── Info banner ──────────────────────────────────────────────────────────
    banner = LogInfo(
        msg=[
            "\n"
            "╔══════════════════════════════════════════════════════════════╗\n"
            "║  LITE SIM MODE - Mock skill atoms (no MoveIt2/ros2_control) ║\n"
            "║                                                              ║\n"
            "║  Web Dashboard:     http://localhost:", dashboard_port, "                    ║\n"
            "║  rosbridge WS:     ws://localhost:", rosbridge_port, "                       ║\n"
            "╚══════════════════════════════════════════════════════════════╝"
        ]
    )

    return LaunchDescription([
        log_level_arg,
        use_rosboard_arg,
        use_dashboard_arg,
        rosbridge_port_arg,
        dashboard_port_arg,
        # Static transforms + robot model
        static_tf,
        robot_state_publisher,
        # Mock joint state publisher
        mock_joint_state_pub,
        # Mock skill atoms
        mock_move_to_named_config,
        mock_move_to_cartesian_pose,
        mock_move_to_joint_config,
        mock_move_cartesian_linear,
        mock_gripper_control,
        mock_detect_object,
        mock_capture_point_cloud,
        mock_set_digital_io,
        mock_check_collision,
        mock_update_planning_scene,
        mock_robot_enable,
        mock_record_rosbag,
        mock_check_system_ready,
        # Tree execution server + orchestrator
        skill_server_node,
        bb_operator_node,
        imaging_station_sim_node,
        # Monitoring
        diagnostic_aggregator_node,
        rosboard_process,
        dashboard_launch,
        banner,
    ])
