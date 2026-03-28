"""
Robot Skills Framework - Simulation Launch File

Launches a complete MoveIt2 simulation using the Franka Panda robot
with mock_components hardware. Provides identical ROS2 interfaces to
the real Meca500 setup so the skill framework can be tested without hardware.

Provides:
  /joint_states                          - robot joint state
  /move_group/...                        - MoveIt2 planning actions/services
  /arm_controller/follow_joint_trajectory - arm trajectory execution
  /gripper_controller/gripper_cmd        - gripper control action
  /robot_description                     - URDF parameter
  TF tree: world → panda_link0 → ... → panda_hand

Usage:
  # Standalone:
  ros2 launch robot_sim_config sim.launch.py

  # From sim container (auto-runs on startup):
  docker compose -f .devcontainer/docker-compose.sim.yml up

  # Then start the skill framework in dev container:
  ros2 launch robot_skill_server skill_server.launch.py

Launch arguments:
  use_rviz      : true/false (default: true)
  log_level     : debug/info/warn/error (default: info)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── Package paths ────────────────────────────────────────────────────────
    sim_pkg = get_package_share_directory("robot_sim_config")
    sim_pkg_path = Path(sim_pkg)

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with MoveIt2 plugin for visualization",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS2 log level: debug, info, warn, error",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")

    # ── MoveIt2 configuration via MoveItConfigsBuilder ─────────────────────
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="robot_sim_config",
        )
        .robot_description(file_path="urdf/sim_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/sim_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    ros2_controllers_yaml = str(sim_pkg_path / "config" / "ros2_controllers.yaml")

    # ── Node 1: robot_state_publisher ─────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 2: static_transform_publisher (world → panda_link0) ─────────────
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # ── Node 3: ros2_control_node (controller_manager) ───────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_yaml,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 4-6: Controller spawners ─────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="gripper_controller_spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    # ── Node 7: MoveIt2 move_group ────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 8: RViz2 with MoveIt2 plugin ────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare("moveit_ros_visualization"),
        "motion_planning_panda",
        "config",
        "moveit.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ── Startup banner ────────────────────────────────────────────────────────
    startup_info = LogInfo(
        msg="""
\u2554\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2557
\u2551         Robot Skills Framework - Simulation Server           \u2551
\u2560\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2563
\u2551  Robot:     Franka Panda (mock hardware)                     \u2551
\u2551  Planning:  group='arm', configs: home/ready/stow/observe    \u2551
\u2551  MoveIt2:   /move_group/*                                    \u2551
\u2551  Arm:       /arm_controller/follow_joint_trajectory          \u2551
\u2551  Gripper:   /gripper_controller/gripper_cmd                  \u2551
\u2551                                                              \u2551
\u2551  Waiting for move_group to be ready (~30s)...                \u2551
\u2551                                                              \u2551
\u2551  Then launch skill_server in dev container:                  \u2551
\u2551    ros2 launch robot_skill_server skill_server.launch.py     \u2551
\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u255d"""
    )

    return LaunchDescription([
        use_rviz_arg,
        log_level_arg,
        startup_info,
        # Infrastructure
        static_tf_node,
        robot_state_publisher_node,
        # Hardware
        ros2_control_node,
        # Controllers (with delay to let controller_manager start)
        TimerAction(
            period=2.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        ),
        # Motion planning (starts after controllers are ready)
        TimerAction(
            period=4.0,
            actions=[move_group_node],
        ),
        # Visualization
        TimerAction(
            period=5.0,
            actions=[rviz_node],
        ),
    ])
