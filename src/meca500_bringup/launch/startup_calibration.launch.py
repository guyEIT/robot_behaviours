import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    camera_config_file = LaunchConfiguration("camera_config_file")
    aruco_config_file = LaunchConfiguration("aruco_config_file")
    enable_realsense = LaunchConfiguration("enable_realsense")
    realsense_camera_name = LaunchConfiguration("realsense_camera_name")
    realsense_camera_namespace = LaunchConfiguration("realsense_camera_namespace")
    realsense_parent_frame = LaunchConfiguration("realsense_parent_frame")
    realsense_frame_id = LaunchConfiguration("realsense_frame_id")
    calibration_camera_frame = LaunchConfiguration("calibration_camera_frame")
    base_frame = LaunchConfiguration("base_frame")
    world_frame = LaunchConfiguration("world_frame")
    base_x = LaunchConfiguration("base_x")
    base_y = LaunchConfiguration("base_y")
    base_z = LaunchConfiguration("base_z")
    enable_startup_aruco_calibration = LaunchConfiguration("enable_startup_aruco_calibration")
    save_startup_calibration = LaunchConfiguration("save_startup_calibration")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    status_topic = LaunchConfiguration("status_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    ignore_existing_camera_config = LaunchConfiguration("ignore_existing_camera_config")
    shutdown_on_complete = LaunchConfiguration("shutdown_on_complete")

    with open(camera_config_file.perform(context), "r", encoding="utf-8") as config_handle:
        camera_config = yaml.safe_load(config_handle) or {}
    camera_config = camera_config.get("camera_calibration", {})

    with open(aruco_config_file.perform(context), "r", encoding="utf-8") as aruco_config_handle:
        aruco_config = yaml.safe_load(aruco_config_handle) or {}
    aruco_config = aruco_config.get("aruco_target", {})

    ignore_existing_camera_config_value = (
        ignore_existing_camera_config.perform(context).strip().lower() in ("1", "true", "yes", "on")
    )

    def override_or_config(override_value, config_key, fallback_value):
        value = override_value.perform(context)
        if value != "":
            return value
        if ignore_existing_camera_config_value:
            return str(fallback_value)
        return str(camera_config.get(config_key, fallback_value))

    enable_realsense_value = override_or_config(enable_realsense, "enable_realsense", "true").lower()
    realsense_camera_name_value = override_or_config(realsense_camera_name, "camera_name", "camera")
    realsense_camera_namespace_value = override_or_config(
        realsense_camera_namespace, "camera_namespace", ""
    )
    base_frame_value = base_frame.perform(context)
    world_frame_value = world_frame.perform(context)
    base_x_value = base_x.perform(context)
    base_y_value = base_y.perform(context)
    base_z_value = base_z.perform(context)
    realsense_parent_frame_value = override_or_config(
        realsense_parent_frame, "parent_frame", base_frame_value
    )
    default_camera_frame_id = "camera_link" if realsense_camera_name_value == "camera" else f"{realsense_camera_name_value}_link"
    realsense_frame_id_value = override_or_config(
        realsense_frame_id, "frame_id", default_camera_frame_id
    )
    default_calibration_frame = f"{realsense_camera_name_value}_color_optical_frame"
    calibration_camera_frame_value = override_or_config(
        calibration_camera_frame, "calibration_frame_id", default_calibration_frame
    )

    enable_startup_aruco_calibration_value = str(enable_startup_aruco_calibration.perform(context))
    if enable_startup_aruco_calibration_value == "":
        enable_startup_aruco_calibration_value = str(
            aruco_config.get("enable_startup_calibration", True)
        )
    enable_startup_aruco_calibration_value = enable_startup_aruco_calibration_value.lower()

    save_startup_calibration_value = str(save_startup_calibration.perform(context))
    if save_startup_calibration_value == "":
        save_startup_calibration_value = "true"
    save_startup_calibration_value = save_startup_calibration_value.lower()

    aruco_dictionary_value = str(aruco_config.get("dictionary", "DICT_6X6_250"))
    aruco_marker_id_value = int(aruco_config.get("marker_id", 23))
    aruco_marker_size_value = float(aruco_config.get("marker_size_m", 0.05))
    aruco_averaging_window_value = float(aruco_config.get("averaging_window_s", 5.0))
    marker_in_parent = aruco_config.get("marker_in_parent", {})
    marker_parent_x = float(marker_in_parent.get("x", 0.0))
    marker_parent_y = float(marker_in_parent.get("y", 0.0))
    marker_parent_z = float(marker_in_parent.get("z", 0.0))
    marker_parent_roll = float(marker_in_parent.get("roll", 0.0))
    marker_parent_pitch = float(marker_in_parent.get("pitch", 0.0))
    marker_parent_yaw = float(marker_in_parent.get("yaw", 0.0))
    camera_namespace_clean = realsense_camera_namespace_value.strip("/")
    if camera_namespace_clean:
        camera_topic_prefix = f"/{camera_namespace_clean}/{realsense_camera_name_value}"
    else:
        camera_topic_prefix = f"/{realsense_camera_name_value}"
    image_topic_value = f"{camera_topic_prefix}/color/image_raw"
    camera_info_topic_value = f"{camera_topic_prefix}/color/camera_info"

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        condition=IfCondition(enable_realsense_value),
        launch_arguments={
            "camera_name": realsense_camera_name_value,
            "camera_namespace": realsense_camera_namespace_value,
            "enable_color": "true",
            "enable_depth": "true",
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_rgbd": "true",
        }.items(),
    )

    startup_aruco_calibrator_node = Node(
        package="meca500_bringup",
        executable="startup_aruco_calibrator.py",
        name="startup_aruco_calibrator",
        output="screen",
        condition=IfCondition(enable_startup_aruco_calibration_value),
        parameters=[
            {
                "parent_frame": realsense_parent_frame_value,
                "camera_frame": realsense_frame_id_value,
                "calibration_camera_frame": calibration_camera_frame_value,
                "image_topic": image_topic_value,
                "camera_info_topic": camera_info_topic_value,
                "aruco_dictionary": aruco_dictionary_value,
                "marker_id": aruco_marker_id_value,
                "marker_size_m": aruco_marker_size_value,
                "averaging_window_s": aruco_averaging_window_value,
                "marker_parent_x": marker_parent_x,
                "marker_parent_y": marker_parent_y,
                "marker_parent_z": marker_parent_z,
                "marker_parent_roll": marker_parent_roll,
                "marker_parent_pitch": marker_parent_pitch,
                "marker_parent_yaw": marker_parent_yaw,
                "camera_config_file": camera_config_file.perform(context),
                "save_to_camera_config": save_startup_calibration_value == "true",
                "shutdown_on_complete": shutdown_on_complete,
                "status_topic": status_topic,
                "marker_topic": marker_topic,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_startup_calibration",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config_file],
        output="log",
    )

    world_to_base_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base_startup_cal",
        arguments=[
            "--x",
            base_x_value,
            "--y",
            base_y_value,
            "--z",
            base_z_value,
            "--frame-id",
            world_frame_value,
            "--child-frame-id",
            base_frame_value,
        ],
    )

    return [
        world_to_base_tf_node,
        realsense_node,
        startup_aruco_calibrator_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("meca500_bringup"), "config", "camera_calibration.yaml"]
                ),
                description="YAML file for camera calibration transform I/O",
            ),
            DeclareLaunchArgument(
                "aruco_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("meca500_bringup"), "config", "aruco_target.yaml"]
                ),
                description="YAML file for startup ArUco calibration target settings",
            ),
            DeclareLaunchArgument(
                "enable_realsense",
                default_value="",
                description="Override: launch Intel RealSense (true/false). Empty uses camera config file.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_name",
                default_value="",
                description="Override: RealSense camera_name. Empty uses camera config file.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_namespace",
                default_value="",
                description="Override: RealSense camera_namespace. Empty uses camera config file.",
            ),
            DeclareLaunchArgument(
                "realsense_parent_frame",
                default_value="",
                description="Override: parent frame for camera transform. Empty uses camera config file.",
            ),
            DeclareLaunchArgument(
                "realsense_frame_id",
                default_value="",
                description="Override: output camera frame id for calibrated static transform. Empty uses camera config file or camera_link.",
            ),
            DeclareLaunchArgument(
                "calibration_camera_frame",
                default_value="",
                description="Override: camera frame used for ArUco solve (typically *_color_optical_frame). Empty uses camera config file or camera_name_color_optical_frame.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="meca_base_link",
                description="Robot base frame id.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="world",
                description="World frame id used for startup calibration.",
            ),
            DeclareLaunchArgument(
                "base_x",
                default_value="0.0",
                description="Robot base X offset in world/scene [m]. Tune this at the end of the launch command.",
            ),
            DeclareLaunchArgument(
                "base_y",
                default_value="0.0",
                description="Robot base Y offset in world/scene [m]. Tune this at the end of the launch command.",
            ),
            DeclareLaunchArgument(
                "base_z",
                default_value="0.0",
                description="Robot base Z offset in world/scene [m]. Tune this at the end of the launch command.",
            ),
            DeclareLaunchArgument(
                "enable_startup_aruco_calibration",
                default_value="true",
                description="Run startup ArUco camera calibration.",
            ),
            DeclareLaunchArgument(
                "save_startup_calibration",
                default_value="true",
                description="Save calibrated camera transform back to camera_config_file",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz with startup calibration visualization profile.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("meca500_bringup"), "config", "rviz", "startup_calibration.rviz"]
                ),
                description="RViz config file for startup calibration.",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/startup_aruco/status",
                description="Status topic for startup ArUco calibration progress.",
            ),
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/startup_aruco/markers",
                description="Marker topic for startup ArUco calibration visualization.",
            ),
            DeclareLaunchArgument(
                "ignore_existing_camera_config",
                default_value="true",
                description="Ignore existing camera_calibration values when resolving startup calibration defaults.",
            ),
            DeclareLaunchArgument(
                "shutdown_on_complete",
                default_value="false",
                description="If true, stop calibrator after success. Keep false to leave TF active and verify point cloud alignment in RViz.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
