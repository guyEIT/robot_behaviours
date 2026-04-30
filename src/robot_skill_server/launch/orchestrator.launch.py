"""One-shot launch for the orchestrator host: skill_server + bb_operator + dashboard.

Brings up:
  - skill_server_node — BtExecutor + SkillDiscovery + TaskComposer +
    SkillRegistry compound-skill catalog.
  - bb_operator_node — operator-facing services for long-lived campaigns
    (/bb_operator/{add_plate,retire_plate,pause_campaign,operator_decision}).
  - rosbridge_websocket on :9090 + static dashboard on :8081 (via
    robot_dashboard/dashboard.launch.py).

No skill atoms run here — they live on the robot/instrument hosts and
self-register with the SkillRegistry over DDS. Pair with
`pixi run meca500-real-up` (Meca host) and any provider tasks
(`liconic-up`, `hamilton-up`, …) on their respective boxes.

    pixi run orchestrator-run
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    dashboard_port = LaunchConfiguration("dashboard_port")
    log_level = LaunchConfiguration("log_level")

    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    bb_operator = Node(
        package="robot_skill_server",
        executable="bb_operator_node",
        name="bb_operator",
        output="screen",
    )

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("robot_dashboard"),
            "/launch/dashboard.launch.py",
        ]),
        launch_arguments={
            "rosbridge_port": rosbridge_port,
            "dashboard_port": dashboard_port,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("dashboard_port", default_value="8081"),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS2 log level: debug, info, warn, error.",
        ),
        skill_server,
        bb_operator,
        dashboard,
    ])
