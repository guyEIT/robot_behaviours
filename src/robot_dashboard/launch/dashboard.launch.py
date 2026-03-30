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
        "rosbridge_port", default_value="9090", description="rosbridge WebSocket port"
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port", default_value="8080", description="Dashboard HTTP port"
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[config_file],
    )

    # rosapi is required for roslibjs getTopics/getServices calls
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    # Serve the built React app with Python's http.server
    dashboard_port = LaunchConfiguration("dashboard_port")
    static_server = ExecuteProcess(
        cmd=[
            "python3", "-m", "http.server",
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
            "  rosbridge WS:    ws://localhost:9090\n"
        ]
    )

    return LaunchDescription([
        rosbridge_port_arg,
        dashboard_port_arg,
        rosbridge_node,
        rosapi_node,
        static_server,
        info_msg,
    ])
