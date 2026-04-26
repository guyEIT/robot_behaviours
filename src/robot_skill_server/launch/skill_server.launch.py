"""Robot Skills Framework — main skill server launch.

Phase 4 collapsed the per-skill executable fleet into a single per-robot
proxy (meca500_skill_server). This file is now a thin wrapper that picks
the right robot_id from `hardware_mode` and delegates to the proxy launch.

Usage:
  ros2 launch robot_skill_server skill_server.launch.py
  ros2 launch robot_skill_server skill_server.launch.py hardware_mode:=real
  ros2 launch robot_skill_server skill_server.launch.py log_level:=debug

Arguments:
  hardware_mode:  'sim' (default) or 'real'
  log_level:      'debug', 'info' (default), 'warn', 'error'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    robot_id = LaunchConfiguration("robot_id")

    proxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_skill_server"),
            "/launch/meca500_skill_server.launch.py",
        ]),
        launch_arguments={
            "robot_id": robot_id,
            "log_level": log_level,
        }.items(),
    )

    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_mode", default_value="sim",
            description="'sim' or 'real' (kept for back-compat; current "
                        "behaviour is identical for either — set robot_id "
                        "directly to control namespace).",
        ),
        DeclareLaunchArgument(
            "robot_id", default_value="meca500",
            description="Robot name from robots.yaml (drives namespace + "
                        "controller paths).",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS2 log level: debug, info, warn, error.",
        ),
        # back-compat no-ops kept so existing pixi run real-native-up cmd lines
        # don't error on unknown arg names.
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_rosboard", default_value="false"),
        DeclareLaunchArgument("use_realsense", default_value="false"),
        DeclareLaunchArgument("groot_port", default_value="1666"),
        DeclareLaunchArgument("robot_ip", default_value="10.6.105.53"),
        proxy,
        skill_server_node,
    ])
