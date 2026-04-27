"""Launch the Liconic STX action server.

Typical use:

    ros2 launch liconic_ros liconic.launch.py
    ros2 launch liconic_ros liconic.launch.py port:=/dev/ttyUSB0
    ros2 launch liconic_ros liconic.launch.py \\
        model:=STX44_HR rack_model:=liconic_rack_22mm_17
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    model = LaunchConfiguration("model")
    rack_model = LaunchConfiguration("rack_model")
    total_cassettes = LaunchConfiguration("total_cassettes")
    state_file = LaunchConfiguration("state_file")
    connect_on_start = LaunchConfiguration("connect_on_start")
    simulation = LaunchConfiguration("simulation")
    namespace_prefix = LaunchConfiguration("namespace_prefix")
    time_compression = LaunchConfiguration("time_compression")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="/dev/liconic_stx44",
            description="Serial port of the Liconic PLC (see install-liconic-udev.sh).",
        ),
        DeclareLaunchArgument(
            "model", default_value="STX44_IC",
            description="Liconic model code (STX44_IC, STX44_HR, STX44_NC, ...).",
        ),
        DeclareLaunchArgument(
            "rack_model", default_value="liconic_rack_12mm_27",
            description="pylabrobot.storage.liconic.racks factory name.",
        ),
        DeclareLaunchArgument(
            "total_cassettes", default_value="2",
            description="Number of cassettes physically installed.",
        ),
        DeclareLaunchArgument(
            "state_file",
            default_value="~/.local/state/liconic/plate_registry.json",
            description="Persistent plate-location registry (JSON). "
                        "Expanded with ~-resolution at node start.",
        ),
        DeclareLaunchArgument(
            "connect_on_start", default_value="true",
            description="If false, the backend connects lazily on first use.",
        ),
        DeclareLaunchArgument(
            "simulation", default_value="false",
            description="If true, use the in-memory LiconicSimBackend — no "
                        "serial port, no hardware required. Lets BTs exercise "
                        "TakeIn/Fetch/SetTemperature/etc. for integration "
                        "testing. Default is false (real PLC over serial).",
        ),
        DeclareLaunchArgument(
            "namespace_prefix", default_value="",
            description=(
                "Top-level namespace prefix (e.g. \"/sim\"). Pushed via "
                "PushRosNamespace so the action server lives at "
                "<prefix>/liconic_action_server/<name>. Default empty = "
                "real-mode paths."
            ),
        ),
        DeclareLaunchArgument(
            "time_compression", default_value="0.0",
            description=(
                "Sim-only: factor to compress real-world durations passed to "
                "the LiconicSimBackend (e.g. multi-hour incubations collapse "
                "to seconds for dry-runs). 0.0 = no compression. Honoured "
                "only when simulation:=true."
            ),
        ),
        GroupAction([
            PushRosNamespace(namespace_prefix),
            Node(
                package="liconic_ros",
                executable="action_server",
                name="liconic_action_server",
                output="screen",
                parameters=[{
                    "port": port,
                    "model": model,
                    "rack_model": rack_model,
                    "total_cassettes": total_cassettes,
                    "state_file": state_file,
                    "connect_on_start": connect_on_start,
                    "simulation": simulation,
                    "time_compression": time_compression,
                }],
            ),
        ]),
    ])
