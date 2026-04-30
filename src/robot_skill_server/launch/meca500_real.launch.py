"""One-shot launch for the Meca500 robot host: real MoveIt2 + skill atoms.

Brings up:
  - meca500_bringup/moveit.launch.py with simulation:=false (talks TCP to
    the physical Meca500 at robot_ip).
  - meca500_skill_server.launch.py — the arm skill atoms composed in one
    process under /meca500/skill_atoms/<name>, advertising the combined
    manifest at /meca500_skill_server/skills.

Pair with `pixi run orchestrator-run` on the orchestrator host (or the
same box) to drive behavior trees from the dashboard or via
`ros2 action send_goal /skill_server/execute_behavior_tree …`.

    pixi run meca500-real-up

Override the IP if your robot isn't on the default subnet:

    pixi run meca500-real-up robot_ip:=10.x.x.x
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    enable_realsense = LaunchConfiguration("enable_realsense")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "false",
            "robot_ip": robot_ip,
            "enable_realsense": enable_realsense,
        }.items(),
    )

    # Atoms come up after MoveIt — they wait_for_server on /move_action at
    # startup. Delay slightly so the first call lands cleanly.
    atoms_launch = TimerAction(
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip", default_value="10.6.105.53",
            description="Meca500 IP address",
        ),
        DeclareLaunchArgument(
            "enable_realsense", default_value="",
            description="Override: launch Intel RealSense (true/false). "
                        "Empty uses meca500_bringup config file default.",
        ),
        moveit_launch,
        atoms_launch,
    ])
