"""Launch the Hamilton STAR action server.

Typical use:

    ros2 launch hamilton_star_bringup action_server.launch.py \
        deck_file:=$PWD/ros2_ws/src/hamilton_star_bringup/config/star_deck.json
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("hamilton_star_bringup"))
    default_deck = str(bringup_share / "config" / "star_deck.json")
    default_handoffs = str(bringup_share / "config" / "handoffs.yaml")

    deck_file = LaunchConfiguration("deck_file")
    core_grippers = LaunchConfiguration("core_grippers")
    on_conflict = LaunchConfiguration("on_conflict")
    num_channels = LaunchConfiguration("num_channels")
    disconnect_on_shutdown = LaunchConfiguration("disconnect_on_shutdown")
    backend = LaunchConfiguration("backend")
    handoffs_file = LaunchConfiguration("handoffs_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "deck_file", default_value=default_deck,
            description="Path to pylabrobot-serialized deck JSON; empty to start bare.",
        ),
        DeclareLaunchArgument(
            "core_grippers", default_value="1000uL-5mL-on-waste",
            description="CoRe II gripper parking config; ignored if deck_file is set.",
        ),
        DeclareLaunchArgument(
            "on_conflict", default_value="reject",
            description="Behaviour when goals conflict with FSM state: reject|queue.",
        ),
        DeclareLaunchArgument(
            "num_channels", default_value="8",
            description="Number of pipetting channels on this STAR.",
        ),
        DeclareLaunchArgument(
            "disconnect_on_shutdown", default_value="false",
            description=(
                "If true, call lh.stop() on SIGINT. Default false; "
                "next startup then pays a full end-stop recalibration."
            ),
        ),
        DeclareLaunchArgument(
            "backend", default_value="star",
            description=(
                "Which pylabrobot backend to drive: 'star' (real USB) or "
                "'simulator' (STARChatterboxBackend, no hardware required)."
            ),
        ),
        DeclareLaunchArgument(
            "handoffs_file", default_value=default_handoffs,
            description=(
                "YAML with iSWAP handoff calibrations loaded as ROS params "
                "under handoff.<name>.*. Defaults to the packaged "
                "handoffs.yaml; baked-in defaults also work if omitted."
            ),
        ),
        Node(
            package="hamilton_star_ros",
            executable="action_server",
            name="hamilton_star_action_server",
            output="screen",
            parameters=[
                {
                    "deck_file": deck_file,
                    "core_grippers": core_grippers,
                    "on_conflict": on_conflict,
                    "num_channels": num_channels,
                    "disconnect_on_shutdown": disconnect_on_shutdown,
                    "backend": backend,
                },
                handoffs_file,
            ],
        ),
    ])
