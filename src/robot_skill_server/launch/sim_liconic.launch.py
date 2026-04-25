"""One-shot launch: Liconic STX44 sim + skill_server orchestrator.

Brings up:
  - liconic_ros action server with simulation:=true (LiconicSimBackend,
    no serial port, no PLC)
  - skill_server_node (orchestrator: BtExecutor, SkillRegistry, TaskComposer)

Submit BTs to /skill_server/execute_behavior_tree afterwards (e.g.
test_liconic_smoke.xml). Run in the liconic-host pixi env:

    pixi run liconic-sim-test

Equivalent to running `pixi run liconic-sim-up` and `skill_server_node`
in two terminals.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    liconic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("liconic_ros"),
            "/launch/liconic.launch.py",
        ]),
        launch_arguments={"simulation": "true"}.items(),
    )

    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    return LaunchDescription([liconic_launch, skill_server])
