"""One-shot launch: Meca500 sim (MoveIt + mock_components) + proxy + orchestrator.

Phase 2 validation harness. Replaces the legacy skill_atoms_remote.launch.py
fleet with the new meca500_skill_server proxy (composes the 13 atoms into
one process, emits one combined manifest).

After this comes up, submit test_meca500_sim.xml to
/skill_server/execute_behavior_tree to verify the proxy alone is enough
for the test tree to pass.

    pixi run meca500-proxy-test
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "true",
            "enable_realsense": "false",
        }.items(),
    )

    # Proxy comes up after MoveIt — atoms call wait_for_server on /move_action
    # at startup. Delay slightly so the first call lands cleanly.
    proxy_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("meca500_skill_server"),
                    "/launch/meca500_skill_server.launch.py",
                ]),
            ),
        ],
    )

    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    return LaunchDescription([moveit_launch, proxy_launch, skill_server])
