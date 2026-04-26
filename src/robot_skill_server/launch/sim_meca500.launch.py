"""One-shot launch: Meca500 sim (MoveIt + mock_components) + meca500_skill_server
proxy + orchestrator.

Brings up:
  - meca500_bringup/moveit.launch.py with simulation:=true (URDF swaps in
    mock_components/GenericSystem, no real robot needed).
  - meca500_skill_server.launch.py — the 13 arm skill atoms composed in
    one process under /meca500/skill_atoms/<name>, advertising one
    combined manifest at /meca500_skill_server/skills.
  - skill_server_node (BtExecutor + SkillDiscovery + TaskComposer +
    SkillRegistry compound-skill catalog).

Submit test_meca500_sim.xml or test_meca500_proxy_v2.xml to
/skill_server/execute_behavior_tree after the atoms register
(~3-5s after launch). The phase-3 BT executor resolves
(robot_id="meca500", bt_tag) → action server name from the runtime
registry, so trees can drop server_name= overrides and just carry
robot_id="meca500".

    pixi run meca500-sim-test
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
                launch_arguments={"robot_id": "meca500"}.items(),
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
