"""One-shot launch: FR3 sim (MoveIt + mock_components) + fr3_skill_server
proxy + orchestrator.

Brings up:
  - fr3_bringup/moveit.launch.py with simulation:=true (URDF swaps in
    mock_components/GenericSystem; load_gripper:=true uses the upstream
    fake_gripper_state_publisher so the joint state stream stays consistent).
  - fr3_skill_server.launch.py — robot_id:=fr3_lab_sim (root namespace,
    simulate_perception:=true, gripper short-circuit via simulate_grasp).
  - skill_server_node (BtExecutor + SkillDiscovery + TaskComposer).

Submit test_fr3_sim.xml after the atoms register (~3-5s after launch):

    pixi run fr3-sim-test
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("fr3_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "true",
            # No real gripper in sim — fake_gripper_state_publisher only
            # publishes joint states, not the franka_gripper actions.
            # franka_gripper_skill short-circuits via simulate_grasp.
            "load_gripper": "true",
            "enable_rviz": "true",
        }.items(),
    )

    proxy_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("fr3_skill_server"),
                    "/launch/fr3_skill_server.launch.py",
                ]),
                launch_arguments={"robot_id": "fr3_lab_sim"}.items(),
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
