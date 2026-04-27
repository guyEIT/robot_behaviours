"""Launch the imaging-station simulation node.

Run standalone:
    ros2 launch imaging_station imaging_station_sim.launch.py

Or include from a higher-level launch (lab_full / skill_server_lite) so
the campaign tree's `ImagePlate` step has a server to talk to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level for the sim node",
    )
    delay = DeclareLaunchArgument(
        "delay_per_site_sec", default_value="0.5",
        description="Synthetic delay per site capture; 0 for instant",
    )
    site_count = DeclareLaunchArgument(
        "default_site_count", default_value="3",
        description="Sites captured when goal.site_count == 0",
    )
    robot_id = DeclareLaunchArgument(
        "robot_id", default_value="imaging_sim",
        description="Robot id reported in the SkillDescription manifest",
    )
    output_root = DeclareLaunchArgument(
        "output_root", default_value="",
        description="Override the default capture root "
                    "(~/.local/state/imaging_station/)",
    )

    sim_node = Node(
        package="imaging_station",
        executable="imaging_station_sim_node",
        name="imaging_station_sim",
        output="screen",
        arguments=["--ros-args", "--log-level",
                   LaunchConfiguration("log_level")],
        parameters=[{
            "delay_per_site_sec": LaunchConfiguration("delay_per_site_sec"),
            "default_site_count": LaunchConfiguration("default_site_count"),
            "robot_id": LaunchConfiguration("robot_id"),
            "output_root": LaunchConfiguration("output_root"),
        }],
    )

    return LaunchDescription([
        log_level, delay, site_count, robot_id, output_root,
        sim_node,
    ])
