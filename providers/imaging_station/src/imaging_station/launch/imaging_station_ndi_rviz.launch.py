"""Launch the imaging-station NDI bridge node + RViz preview.

Brings up the NDI receiver and an RViz instance pre-configured with an Image
display subscribed to /imaging_station/image_raw — one-shot way to see live
ZowieBox frames while developing.

    pixi run imaging-ndi-rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("imaging_station")
    default_rviz_config = os.path.join(pkg_share, "rviz", "imaging_station_ndi.rviz")

    ndi_source_name = DeclareLaunchArgument(
        "ndi_source_name",
        default_value="ZOWIEBOX-40435 (ZowieBox-40435)",
        description="NDI source advertised name to subscribe to",
    )
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config file",
    )
    log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level for the NDI node",
    )

    ndi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("imaging_station"),
                "launch",
                "imaging_station_ndi.launch.py",
            ]),
        ]),
        launch_arguments={
            "ndi_source_name": LaunchConfiguration("ndi_source_name"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="imaging_station_ndi_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        ndi_source_name, rviz_config, log_level,
        ndi_launch,
        rviz_node,
    ])
