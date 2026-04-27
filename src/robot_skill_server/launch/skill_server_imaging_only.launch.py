"""Minimal launch — just enough to exercise the imaging_station sim through
the BT executor.

Brings up:
  - imaging_station_sim (advertises ImagePlate)
  - skill_server_node (BtExecutor + SkillRegistry + SkillDiscovery)
  - bb_operator (operator services for long-lived campaigns)

No mock arm atoms, no URDF, no MoveIt — useful when validating the
persistence / resume layer or smoke-testing imaging in isolation. Submit
``test_imaging_sim.xml`` from another shell to exercise the flow end-to-end.

    pixi run -e local-dev bash -c \\
        "source .pixi/colcon/install/setup.bash && \\
         ros2 launch robot_skill_server skill_server_imaging_only.launch.py"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = DeclareLaunchArgument("log_level", default_value="info")
    use_dashboard = DeclareLaunchArgument(
        "use_dashboard", default_value="true",
        description="Launch the dashboard (rosbridge + static UI)",
    )
    rosbridge_port = DeclareLaunchArgument("rosbridge_port", default_value="9090")
    dashboard_port = DeclareLaunchArgument("dashboard_port", default_value="8081")

    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    bb_operator_node = Node(
        package="robot_skill_server",
        executable="bb_operator_node",
        name="bb_operator",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    imaging_station_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("imaging_station"),
            "/launch/imaging_station_sim.launch.py",
        ]),
        launch_arguments={"delay_per_site_sec": "0.2"}.items(),
    )

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("robot_dashboard"),
            "/launch/dashboard.launch.py",
        ]),
        launch_arguments={
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
            "dashboard_port": LaunchConfiguration("dashboard_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_dashboard")),
    )

    return LaunchDescription([
        log_level,
        use_dashboard,
        rosbridge_port,
        dashboard_port,
        imaging_station_sim,
        skill_server_node,
        bb_operator_node,
        dashboard,
    ])
