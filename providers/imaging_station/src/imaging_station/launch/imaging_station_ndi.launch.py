"""Launch the imaging-station NDI bridge node.

Run standalone:
    ros2 launch imaging_station imaging_station_ndi.launch.py

Connects to the configured NDI source (default: ZowieBox-40435) via the
bundled ndi_av_bridge subprocess and republishes frames as
sensor_msgs/Image on /imaging_station/image_raw.

Sibling launch file: imaging_station_sim.launch.py drives the placeholder
ImagePlate sim. The two are independent for now; a follow-up will wire a
real ImagePlate backend that captures from this stream.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level for the NDI node",
    )
    ndi_source_name = DeclareLaunchArgument(
        "ndi_source_name",
        default_value="ZOWIEBOX-40435 (ZowieBox-40435)",
        description="NDI source advertised name to subscribe to",
    )
    url_address = DeclareLaunchArgument(
        "url_address",
        default_value="",
        description="Optional host:port override; bypasses mDNS discovery "
                    "(useful when libndi can't see the source on multi-NIC hosts)",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="imaging_station_camera_optical_frame",
        description="frame_id stamped on every published Image. The static "
                    "TF microscope_camera -> imaging_station_camera_optical_frame "
                    "(0.195 m along +Z) is published by imaging_station_scene.launch.py.",
    )
    target_fps = DeclareLaunchArgument(
        "target_fps", default_value="0.0",
        description="Cap publish rate (0 = pass through every NDI frame)",
    )
    discovery_timeout_sec = DeclareLaunchArgument(
        "discovery_timeout_sec", default_value="5.0",
        description="How long to wait for NDI source discovery at startup",
    )
    bandwidth = DeclareLaunchArgument(
        "bandwidth", default_value="highest",
        description="NDI receiver bandwidth (highest|lowest)",
    )
    image_topic = DeclareLaunchArgument(
        "image_topic", default_value="image_raw",
        description="Topic to publish under /imaging_station/",
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="camera_info",
        description="CameraInfo topic name (under /imaging_station/). "
                    "Paired 1:1 with image_raw via matching header.",
    )
    camera_info_url = DeclareLaunchArgument(
        "camera_info_url",
        default_value=PathJoinSubstitution([
            FindPackageShare("imaging_station"),
            "calibration",
            "zowiebox_camera_info.yaml",
        ]),
        description="Path to a camera_calibration_parsers-format YAML. "
                    "If empty or missing, the node synthesises a 50° HFOV "
                    "pinhole CameraInfo from the live frame dimensions.",
    )
    startup_timeout_sec = DeclareLaunchArgument(
        "startup_timeout_sec", default_value="12.0",
        description="Bridge waits this long for the NDI source to come up",
    )

    ndi_node = Node(
        package="imaging_station",
        executable="imaging_station_ndi_node",
        name="imaging_station_ndi",
        namespace="imaging_station",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "ndi_source_name": LaunchConfiguration("ndi_source_name"),
            "url_address": LaunchConfiguration("url_address"),
            "frame_id": LaunchConfiguration("frame_id"),
            "target_fps": LaunchConfiguration("target_fps"),
            "discovery_timeout_sec": LaunchConfiguration("discovery_timeout_sec"),
            "bandwidth": LaunchConfiguration("bandwidth"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "startup_timeout_sec": LaunchConfiguration("startup_timeout_sec"),
        }],
    )

    return LaunchDescription([
        log_level, ndi_source_name, url_address, frame_id, target_fps,
        discovery_timeout_sec, bandwidth, image_topic,
        camera_info_topic, camera_info_url, startup_timeout_sec,
        ndi_node,
    ])
