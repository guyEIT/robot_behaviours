"""One-shot launch: Meca500 sim (MoveIt + mock_components) + skill atoms +
skill_server orchestrator.

Brings up:
  - meca500_bringup/moveit.launch.py with simulation:=true (so the URDF
    swaps in mock_components/GenericSystem instead of meca500_hardware,
    no real robot needed) and enable_realsense:=false (no camera in sim).
  - skill_atoms_remote.launch.py with robot_name:=meca500 — launches the
    13 generic arm skill atoms under /meca500/skill_atoms/<name>, all
    parameterized for Meca via robots.yaml.
  - skill_server_node (BtExecutor, SkillRegistry, TaskComposer).

Submit test_meca500_sim.xml to /skill_server/execute_behavior_tree after
the atoms register (~3-5s after launch).

Run in the meca500-host pixi env:

    pixi run meca500-sim-test

Equivalent to running meca500-sim-up + meca500-up + skill_server_node
in three terminals.
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

    # Atoms come up after MoveIt — they call wait_for_server on /move_action
    # at ~5s timeout per skill. Delay slightly so the first call lands cleanly.
    atoms_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("robot_skill_server"),
                    "/launch/skill_atoms_remote.launch.py",
                ]),
                launch_arguments={"robot_name": "meca500"}.items(),
            ),
        ],
    )

    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    return LaunchDescription([moveit_launch, atoms_launch, skill_server])
