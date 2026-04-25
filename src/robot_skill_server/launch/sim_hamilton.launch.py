"""One-shot launch: Hamilton STAR sim + skill_server orchestrator.

Brings up:
  - hamilton_star_bringup action server (backend:=simulator, no hardware)
  - skill_server_node (orchestrator: BtExecutor, SkillRegistry, TaskComposer)

Submit BTs to /skill_server/execute_behavior_tree afterwards (e.g.
test_hamilton_sim.xml). Run in the liconic-host pixi env:

    pixi run hamilton-sim-test

Equivalent to running `pixi run hamilton-sim-up` and `skill_server_node`
in two terminals.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hamilton_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("hamilton_star_bringup"),
            "/launch/action_server.launch.py",
        ]),
        launch_arguments={"backend": "simulator"}.items(),
    )

    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    return LaunchDescription([hamilton_launch, skill_server])
