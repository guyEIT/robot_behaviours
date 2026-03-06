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
  rviz_config   : path to rviz config (default: built-in MoveIt config)
  log_level     : debug/info/warn/error (default: info)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    # ── Robot description (URDF from xacro) ──────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        str(sim_pkg_path / "urdf" / "sim_robot.urdf.xacro"),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── Semantic robot description (SRDF) ─────────────────────────────────────
    robot_description_semantic_content = (
        sim_pkg_path / "config" / "sim_robot.srdf"
    ).read_text()
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # ── Load config files ─────────────────────────────────────────────────────
    kinematics_yaml = str(sim_pkg_path / "config" / "kinematics.yaml")
    joint_limits_yaml = str(sim_pkg_path / "config" / "joint_limits.yaml")
    moveit_controllers_yaml = str(sim_pkg_path / "config" / "moveit_controllers.yaml")
    ompl_planning_yaml = str(sim_pkg_path / "config" / "ompl_planning.yaml")
    moveit_yaml = str(sim_pkg_path / "config" / "moveit.yaml")
    ros2_controllers_yaml = str(sim_pkg_path / "config" / "ros2_controllers.yaml")

    # ── Node 1: robot_state_publisher ─────────────────────────────────────────
    # Publishes /robot_description parameter and TF transforms
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 2: static_transform_publisher (world → panda_link0) ─────────────
    # Required because the SRDF virtual joint connects world → panda_link0
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # ── Node 3: ros2_control_node (controller_manager) ───────────────────────
    # Loads the mock hardware system and manages all controllers
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 4-6: Controller spawners ─────────────────────────────────────────
    # Must spawn AFTER controller_manager is ready

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
    # The central motion planning server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": {}},  # Will be loaded from kinematics_yaml
            kinematics_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            ompl_planning_yaml,
            moveit_yaml,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            # Planning pipeline config
            {
                "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
                "default_planning_pipeline": "ompl",
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Node 8: RViz2 with MoveIt2 plugin ────────────────────────────────────
    # Uses the default MoveIt2 RViz config if no custom one is provided
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
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": False},
        ],
    )

    # ── Startup banner ────────────────────────────────────────────────────────
    startup_info = LogInfo(
        msg="""
╔══════════════════════════════════════════════════════════════╗
║         Robot Skills Framework - Simulation Server           ║
╠══════════════════════════════════════════════════════════════╣
║  Robot:     Franka Panda (mock hardware)                     ║
║  Planning:  group='arm', configs: home/ready/stow/observe    ║
║  MoveIt2:   /move_group/*                                    ║
║  Arm:       /arm_controller/follow_joint_trajectory          ║
║  Gripper:   /gripper_controller/gripper_cmd                  ║
║                                                              ║
║  Waiting for move_group to be ready (~30s)...                ║
║                                                              ║
║  Then launch skill_server in dev container:                  ║
║    ros2 launch robot_skill_server skill_server.launch.py     ║
╚══════════════════════════════════════════════════════════════╝"""
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
