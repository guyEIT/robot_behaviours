"""Launch the YOLO seed detector against a RealSense camera.

Standalone:
    ros2 launch seed_detector seed_detector.launch.py debug_markers:=true

Override the model:
    ros2 launch seed_detector seed_detector.launch.py model_path:=/abs/path/to/last.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level for the seed detector node",
    )
    model_path = DeclareLaunchArgument(
        "model_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("seed_detector"), "models", "last.pt"]
        ),
        description="Absolute path to the YOLO .pt weights",
    )
    image_topic = DeclareLaunchArgument(
        "image_topic", default_value="/camera/color/image_raw",
        description="RGB image input topic",
    )
    depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/aligned_depth_to_color/image_raw",
        description="Depth image input topic (must be aligned to RGB)",
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/color/camera_info",
        description="CameraInfo topic for the RGB stream",
    )
    detections_topic = DeclareLaunchArgument(
        "detections_topic", default_value="~/detections",
        description="Detection3DArray output topic",
    )
    markers_topic = DeclareLaunchArgument(
        "markers_topic", default_value="~/markers",
        description="MarkerArray output topic (debug only)",
    )
    annotated_image_topic = DeclareLaunchArgument(
        "annotated_image_topic", default_value="~/annotated_image",
        description="Annotated RGB image topic with bbox overlay (debug only)",
    )
    confidence = DeclareLaunchArgument(
        "confidence_threshold", default_value="0.5",
        description="YOLO confidence threshold",
    )
    iou = DeclareLaunchArgument(
        "iou_threshold", default_value="0.45",
        description="YOLO NMS IoU threshold",
    )
    rate = DeclareLaunchArgument(
        "detection_rate_hz", default_value="5.0",
        description="Maximum inference rate; intermediate frames are dropped",
    )
    device = DeclareLaunchArgument(
        "device", default_value="",
        description="Inference device for ultralytics (cpu / cuda:0). Empty = auto",
    )
    patch = DeclareLaunchArgument(
        "depth_patch_pixels", default_value="5",
        description="Window size for the depth sample around each bbox center",
    )
    depth_sampling_mode = DeclareLaunchArgument(
        "depth_sampling_mode", default_value="foreground",
        description="'foreground' (lower-quantile, default) or 'median' (legacy)",
    )
    depth_foreground_quantile = DeclareLaunchArgument(
        "depth_foreground_quantile", default_value="0.25",
        description="Quantile for foreground mode (0.0=closest pixel; 0.5=median)",
    )
    slop = DeclareLaunchArgument(
        "sync_slop_sec", default_value="0.05",
        description="Allowed RGB↔depth time mismatch for ApproximateTimeSynchronizer",
    )
    debug_markers = DeclareLaunchArgument(
        "debug_markers", default_value="false",
        description="Publish a MarkerArray for rviz: one sphere + label per detection",
    )

    detector_node = Node(
        package="seed_detector",
        executable="seed_detector_node",
        name="seed_detector",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "markers_topic": LaunchConfiguration("markers_topic"),
            "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "iou_threshold": LaunchConfiguration("iou_threshold"),
            "detection_rate_hz": LaunchConfiguration("detection_rate_hz"),
            "device": LaunchConfiguration("device"),
            "depth_patch_pixels": LaunchConfiguration("depth_patch_pixels"),
            "depth_sampling_mode": LaunchConfiguration("depth_sampling_mode"),
            "depth_foreground_quantile": LaunchConfiguration("depth_foreground_quantile"),
            "sync_slop_sec": LaunchConfiguration("sync_slop_sec"),
            "debug_markers": LaunchConfiguration("debug_markers"),
        }],
    )

    return LaunchDescription([
        log_level, model_path, image_topic, depth_topic, camera_info_topic,
        detections_topic, markers_topic, annotated_image_topic,
        confidence, iou, rate, device,
        patch, depth_sampling_mode, depth_foreground_quantile,
        slop, debug_markers,
        detector_node,
    ])
