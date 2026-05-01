"""One-shot launch for the FR3 robot host: real MoveIt2 + skill atoms.

Brings up:
  - fr3_bringup/moveit.launch.py with simulation:=false (talks libfranka over
    the network to the physical FR3 at robot_ip; spawns franka_gripper_node
    when load_gripper:=true).
  - fr3_skill_server.launch.py — robot_id:=fr3 (atoms under /fr3/skill_atoms,
    franka_gripper_skill bridges to /franka_gripper/{move,grasp,homing}).

Pair with `pixi run orchestrator-run` on the orchestrator host to drive
behavior trees from the dashboard.

    pixi run fr3-real-up

Override the IP if your robot isn't at the default:

    pixi run fr3-real-up robot_ip:=10.x.x.x
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    enable_rviz = LaunchConfiguration("enable_rviz")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fr3_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "false",
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "enable_rviz": enable_rviz,
        }.items(),
    )

    atoms_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("fr3_skill_server"),
                    "/launch/fr3_skill_server.launch.py",
                ]),
                launch_arguments={"robot_id": "fr3"}.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            description="FR3 IP address or hostname (e.g. 192.168.1.45)."),
        DeclareLaunchArgument(
            "load_gripper", default_value="true",
            description="Whether to spawn franka_gripper_node for the Franka Hand."),
        DeclareLaunchArgument(
            "enable_rviz", default_value="true",
            description="Launch RViz2 alongside MoveIt."),
        moveit_launch,
        atoms_launch,
    ])
