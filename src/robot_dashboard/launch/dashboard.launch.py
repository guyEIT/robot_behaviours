"""
Robot Dashboard launch file.

Starts:
  1. rosbridge_websocket — WebSocket bridge on port 9090
  2. Static file server — serves the React dashboard on port 8080
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_dashboard")
    www_dir = os.path.join(pkg_share, "www")
    config_file = os.path.join(pkg_share, "config", "rosbridge_params.yaml")

    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
        description="rosbridge WebSocket port",
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port",
        default_value=os.environ.get("DASHBOARD_PORT", "8081"),
        description="Dashboard HTTP port",
    )

    rosbridge_port = LaunchConfiguration("rosbridge_port")

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[config_file, {"port": rosbridge_port}],
    )

    # rosapi is required for roslibjs getTopics/getServices calls
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    # Write runtime config so the frontend knows the rosbridge port
    env_config = ExecuteProcess(
        cmd=[
            "bash", "-c",
            [
                'echo "window.ENV_CONFIG = { ROSBRIDGE_PORT: \\"',
                rosbridge_port,
                '\\" };" > ',
                os.path.join(www_dir, "env-config.js"),
            ],
        ],
        output="screen",
    )

    # Serve the built React app with a thin no-cache wrapper around
    # http.server. Vanilla `python -m http.server` lets the browser cache
    # `index.html` indefinitely, so users keep seeing stale UI after
    # `pixi run dashboard-build`. The wrapper sets Cache-Control: no-store
    # on entry-point HTML/JS and immutable on hashed /assets/ bundles.
    dashboard_port = LaunchConfiguration("dashboard_port")
    serve_script = os.path.join(pkg_share, "scripts", "serve_dashboard.py")
    static_server = ExecuteProcess(
        cmd=[
            "python3", serve_script,
            dashboard_port,
            "--directory", www_dir,
            "--bind", "0.0.0.0",
        ],
        output="screen",
    )

    info_msg = LogInfo(
        msg=[
            "\n"
            "  Robot Dashboard:  http://localhost:", dashboard_port, "\n"
            "  rosbridge WS:    ws://localhost:", rosbridge_port, "\n"
        ]
    )

    return LaunchDescription([
        rosbridge_port_arg,
        dashboard_port_arg,
        env_config,
        rosbridge_node,
        rosapi_node,
        static_server,
        info_msg,
    ])
