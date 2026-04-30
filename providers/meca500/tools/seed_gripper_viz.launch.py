"""Launch the seed-gripper visualisation alongside the Meca500 MoveIt stack.

Brings up:
    * meca500_bringup/moveit.launch.py (sim defaults: fake_hardware, no RealSense)
    * seed_gripper_visualizer.py (custom Python node from the same tools/ dir)
    * rqt_reconfigure (slider panel for live offset tuning)

In the bringup's RViz window, add two displays once:
    * Add -> By topic -> /seed_gripper/visualization -> MarkerArray
    * Add -> InteractiveMarkers, topic = /seed_gripper_im/update

Save:
    ros2 service call /seed_gripper/save_offsets std_srvs/srv/Trigger {}
Load (also runs automatically on startup if the YAML exists):
    ros2 service call /seed_gripper/load_offsets std_srvs/srv/Trigger {}
"""
from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TOOLS_DIR = Path(__file__).resolve().parent
VISUALIZER_SCRIPT = str(TOOLS_DIR / "seed_gripper_visualizer.py")
DEFAULT_OFFSETS_YAML = str(TOOLS_DIR / "seed_gripper_offsets.yaml")
DEFAULT_LEFT_STL = str(TOOLS_DIR.parent / "seed_gripper-Finger.stl")
DEFAULT_RIGHT_STL = str(TOOLS_DIR.parent / "seed_gripper-Finger-right.stl")


def generate_launch_description() -> LaunchDescription:
    simulation = LaunchConfiguration("simulation")
    enable_realsense = LaunchConfiguration("enable_realsense")
    enable_rqt = LaunchConfiguration("enable_rqt")
    offsets_yaml = LaunchConfiguration("offsets_yaml")
    left_mesh = LaunchConfiguration("left_mesh")
    right_mesh = LaunchConfiguration("right_mesh")

    bringup_launch = PathJoinSubstitution([
        FindPackageShare("meca500_bringup"),
        "launch",
        "moveit.launch.py",
    ])

    visualizer = ExecuteProcess(
        cmd=[
            "python3", VISUALIZER_SCRIPT,
            "--ros-args",
            "-r", "__node:=seed_gripper_visualizer",
            "-p", ["offsets_yaml_path:=", offsets_yaml],
            "-p", ["left_mesh_path:=", left_mesh],
            "-p", ["right_mesh_path:=", right_mesh],
        ],
        output="screen",
    )

    rqt = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen",
        condition=IfCondition(enable_rqt),
    )

    return LaunchDescription([
        DeclareLaunchArgument("simulation", default_value="true",
                              description="Use mock_components/GenericSystem instead of real Meca500."),
        DeclareLaunchArgument("enable_realsense", default_value="false",
                              description="Bring up the RealSense camera (override the bringup default)."),
        DeclareLaunchArgument("enable_rqt", default_value="true",
                              description="Auto-spawn rqt_reconfigure for the slider panel."),
        DeclareLaunchArgument("offsets_yaml", default_value=DEFAULT_OFFSETS_YAML,
                              description="YAML file for save_offsets / load_offsets services."),
        DeclareLaunchArgument("left_mesh", default_value=DEFAULT_LEFT_STL,
                              description="Absolute path to the left-finger STL."),
        DeclareLaunchArgument("right_mesh", default_value=DEFAULT_RIGHT_STL,
                              description="Absolute path to the right-finger STL (mirrored)."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_launch]),
            launch_arguments={
                "simulation": simulation,
                "enable_realsense": enable_realsense,
                "enable_startup_aruco_calibration": "false",
            }.items(),
        ),
        visualizer,
        rqt,
    ])
