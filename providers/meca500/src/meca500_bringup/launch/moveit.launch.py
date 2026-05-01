import os

import yaml

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

# Default location for the seed-gripper offsets YAML, which the
# seed_gripper_visualizer writes via /seed_gripper/save_offsets.
#
# pixi-build-ros installs this launch file as a hard copy under
# .pixi/envs/.../share/meca500_bringup/launch/, so __file__ doesn't have a
# stable relative path to providers/meca500/tools/. Resolve via the
# workspace root marker (pixi.toml) instead — works for both source-tree
# and installed copies, since the install path is itself nested under the
# workspace root.
def _find_workspace_root():
    env_root = os.environ.get("PIXI_PROJECT_ROOT") or os.environ.get(
        "PIXI_WORKSPACE_ROOT"
    )
    if env_root and os.path.isfile(os.path.join(env_root, "pixi.toml")):
        return env_root
    here = os.path.dirname(os.path.abspath(__file__))
    while here != os.path.dirname(here):
        if os.path.isfile(os.path.join(here, "pixi.toml")):
            return here
        here = os.path.dirname(here)
    return None


_WORKSPACE_ROOT = _find_workspace_root() or ""
_PROVIDER_DIR = os.path.join(_WORKSPACE_ROOT, "providers", "meca500") if _WORKSPACE_ROOT else ""
_DEFAULT_SEED_GRIPPER_OFFSETS_YAML = (
    os.path.join(_PROVIDER_DIR, "tools", "seed_gripper_offsets.yaml")
    if _PROVIDER_DIR else ""
)
_DEFAULT_SEED_GRIPPER_LEFT_MESH = (
    os.path.join(_PROVIDER_DIR, "seed_gripper-Finger.stl")
    if _PROVIDER_DIR else ""
)
_DEFAULT_SEED_GRIPPER_RIGHT_MESH = (
    os.path.join(_PROVIDER_DIR, "seed_gripper-Finger-right.stl")
    if _PROVIDER_DIR else ""
)
_DEFAULT_IMAGING_STATION_SCENE_LAUNCH = (
    os.path.join(
        _WORKSPACE_ROOT, "providers", "imaging_station", "launch",
        "imaging_station_scene.launch.py",
    )
    if _WORKSPACE_ROOT else ""
)


def _seed_gripper_mappings(use_seed_gripper, yaml_path, left_mesh, right_mesh):
    """Build the xacro:arg mappings consumed by meca500.xacro's seed-gripper
    overlay. When use_seed_gripper is falsy or the YAML is missing we still
    return a complete dict with safe zeros so xacro:arg defaults are
    overridden deterministically.
    """
    use = str(use_seed_gripper).strip().lower() in ("true", "1", "yes", "on")
    mappings = {
        "use_seed_gripper": "true" if use else "false",
        "left_mesh": left_mesh or "",
        "right_mesh": right_mesh or "",
        "stl_scale": "0.001",
    }
    for key in ("left", "right", "grip"):
        for axis in ("x", "y", "z"):
            mappings[f"{key}_xyz_{axis}"] = "0.0"
        for axis in ("r", "p", "y"):
            mappings[f"{key}_rpy_{axis}"] = "0.0"
    if not use:
        return mappings

    if yaml_path and os.path.isfile(yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if "stl_scale" in data:
            mappings["stl_scale"] = str(float(data["stl_scale"]))
        for key in ("left", "right", "grip"):
            section = data.get(key) or {}
            xyz = section.get("xyz") or [0.0, 0.0, 0.0]
            rpy = section.get("rpy") or [0.0, 0.0, 0.0]
            for i, axis in enumerate(("x", "y", "z")):
                mappings[f"{key}_xyz_{axis}"] = str(float(xyz[i]))
            for i, axis in enumerate(("r", "p", "y")):
                mappings[f"{key}_rpy_{axis}"] = str(float(rpy[i]))
    return mappings


def launch_setup(context, *args, **kwargs):
    # --------------------------
    # Launch arguments
    # --------------------------
    servo = LaunchConfiguration("servo")
    robot_controller = LaunchConfiguration("robot_controller")
    simulation = LaunchConfiguration("simulation")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    connect_timeout_ms = LaunchConfiguration("connect_timeout_ms")
    response_timeout_ms = LaunchConfiguration("response_timeout_ms")
    control_port = LaunchConfiguration("control_port")
    hardware_type = LaunchConfiguration("hardware_type")
    auto_home = LaunchConfiguration("auto_home")
    use_seed_gripper = LaunchConfiguration("use_seed_gripper")
    seed_gripper_offsets_yaml = LaunchConfiguration("seed_gripper_offsets_yaml")
    seed_gripper_left_mesh = LaunchConfiguration("seed_gripper_left_mesh")
    seed_gripper_right_mesh = LaunchConfiguration("seed_gripper_right_mesh")
    camera_config_file = LaunchConfiguration("camera_config_file")
    aruco_config_file = LaunchConfiguration("aruco_config_file")
    enable_realsense = LaunchConfiguration("enable_realsense")
    realsense_camera_name = LaunchConfiguration("realsense_camera_name")
    realsense_camera_namespace = LaunchConfiguration("realsense_camera_namespace")
    realsense_parent_frame = LaunchConfiguration("realsense_parent_frame")
    realsense_frame_id = LaunchConfiguration("realsense_frame_id")
    calibration_camera_frame = LaunchConfiguration("calibration_camera_frame")
    show_table = LaunchConfiguration("show_table")
    table_mesh_path = LaunchConfiguration("table_mesh_path")
    table_frame = LaunchConfiguration("table_frame")
    table_x = LaunchConfiguration("table_x")
    table_y = LaunchConfiguration("table_y")
    table_z = LaunchConfiguration("table_z")
    table_roll = LaunchConfiguration("table_roll")
    table_pitch = LaunchConfiguration("table_pitch")
    table_yaw = LaunchConfiguration("table_yaw")
    table_scale = LaunchConfiguration("table_scale")
    base_x = LaunchConfiguration("base_x")
    base_y = LaunchConfiguration("base_y")
    base_z = LaunchConfiguration("base_z")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    enable_startup_aruco_calibration = LaunchConfiguration("enable_startup_aruco_calibration")
    save_startup_calibration = LaunchConfiguration("save_startup_calibration")
    enable_moveit_pointcloud = LaunchConfiguration("enable_moveit_pointcloud")
    moveit_pointcloud_max_range = LaunchConfiguration("moveit_pointcloud_max_range")
    moveit_pointcloud_subsample = LaunchConfiguration("moveit_pointcloud_subsample")
    moveit_pointcloud_max_update_rate = LaunchConfiguration("moveit_pointcloud_max_update_rate")

    with open(camera_config_file.perform(context), "r", encoding="utf-8") as config_handle:
        camera_config = yaml.safe_load(config_handle) or {}
    camera_config = camera_config.get("camera_calibration", {})
    with open(aruco_config_file.perform(context), "r", encoding="utf-8") as aruco_config_handle:
        aruco_config = yaml.safe_load(aruco_config_handle) or {}
    aruco_config = aruco_config.get("aruco_target", {})
    table_config_file = LaunchConfiguration("table_config_file")
    with open(table_config_file.perform(context), "r", encoding="utf-8") as table_config_handle:
        table_config = yaml.safe_load(table_config_handle) or {}
    table_config = table_config.get("table", {})

    def override_or_config(override_value, config_key, fallback_value):
        value = override_value.perform(context)
        if value != "":
            return value
        return str(camera_config.get(config_key, fallback_value))

    def table_override_or_config(override_value, config_key, fallback_value):
        value = override_value.perform(context)
        if value != "":
            return value
        return str(table_config.get(config_key, fallback_value))

    enable_realsense_value = override_or_config(enable_realsense, "enable_realsense", "true").lower()
    realsense_camera_name_value = override_or_config(realsense_camera_name, "camera_name", "camera")
    realsense_camera_namespace_value = override_or_config(
        realsense_camera_namespace, "camera_namespace", ""
    )
    realsense_parent_frame_value = override_or_config(realsense_parent_frame, "parent_frame", "meca_base_link")
    default_camera_frame_id = "camera_link" if realsense_camera_name_value == "camera" else f"{realsense_camera_name_value}_link"
    realsense_frame_id_value = override_or_config(
        realsense_frame_id, "frame_id", default_camera_frame_id
    )
    default_calibration_frame = f"{realsense_camera_name_value}_color_optical_frame"
    calibration_camera_frame_value = override_or_config(
        calibration_camera_frame, "calibration_frame_id", default_calibration_frame
    )
    show_table_value = table_override_or_config(show_table, "show", "true").lower()
    table_mesh_path_value = table_override_or_config(table_mesh_path, "mesh_path", "")
    if not table_mesh_path_value:
        # Default to the package mesh when neither launch arg nor config file provides a path
        table_mesh_path_value = PathJoinSubstitution(
            [FindPackageShare("meca500_bringup"), "meshes", "mecatable_simple.stl"]
        ).perform(context)
    table_frame_value = table_override_or_config(table_frame, "frame_id", "world")
    table_x_value = float(table_override_or_config(table_x, "x", "0.0"))
    table_y_value = float(table_override_or_config(table_y, "y", "0.0"))
    table_z_value = float(table_override_or_config(table_z, "z", "0.0"))
    table_roll_value = float(table_override_or_config(table_roll, "roll", "0.0"))
    table_pitch_value = float(table_override_or_config(table_pitch, "pitch", "0.0"))
    table_yaw_value = float(table_override_or_config(table_yaw, "yaw", "0.0"))
    table_scale_value = float(table_override_or_config(table_scale, "scale", "0.001"))
    base_x_value = base_x.perform(context)
    base_y_value = base_y.perform(context)
    base_z_value = base_z.perform(context)
    camera_x_value = override_or_config(camera_x, "x", "0.0")
    camera_y_value = override_or_config(camera_y, "y", "0.0")
    camera_z_value = override_or_config(camera_z, "z", "0.0")
    camera_roll_value = override_or_config(camera_roll, "roll", "0.0")
    camera_pitch_value = override_or_config(camera_pitch, "pitch", "0.0")
    camera_yaw_value = override_or_config(camera_yaw, "yaw", "0.0")
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
    point_cloud_topic_value = f"{camera_topic_prefix}/depth/color/points"
    cropped_cloud_topic_value = f"{camera_topic_prefix}/depth/color/points_cropped"
    enable_moveit_pointcloud_value = str(enable_moveit_pointcloud.perform(context)).strip().lower()
    moveit_pointcloud_max_range_value = float(moveit_pointcloud_max_range.perform(context))
    moveit_pointcloud_subsample_value = int(moveit_pointcloud_subsample.perform(context))
    moveit_pointcloud_max_update_rate_value = float(moveit_pointcloud_max_update_rate.perform(context))
    moveit_pointcloud_params = {}
    if enable_moveit_pointcloud_value in ("1", "true", "yes", "on"):
        moveit_pointcloud_params = {
            "octomap_frame": "world",
            "octomap_resolution": 0.02,
            "max_range": moveit_pointcloud_max_range_value,
            "sensors": ["camera_pointcloud"],
            "camera_pointcloud": {
                "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
                "point_cloud_topic": cropped_cloud_topic_value,
                "max_range": moveit_pointcloud_max_range_value,
                "max_update_rate": moveit_pointcloud_max_update_rate_value,
                "point_subsample": moveit_pointcloud_subsample_value,
                "padding_offset": 0.03,
                "padding_scale": 1.0,
                "filtered_cloud_topic": "filtered_cloud",
            },
        }

    # --------------------------
    # MoveIt configuration
    # --------------------------
    seed_gripper_mappings = _seed_gripper_mappings(
        use_seed_gripper.perform(context),
        seed_gripper_offsets_yaml.perform(context),
        seed_gripper_left_mesh.perform(context),
        seed_gripper_right_mesh.perform(context),
    )
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="meca500",
            package_name="meca500_moveit",
        )
        .robot_description(
            # moveit_configs_utils does NOT call .perform(context) on
            # LaunchConfiguration objects in `mappings`; it ends up passing
            # their __repr__ to xacro, which silently ignores them. Resolve
            # to strings explicitly so `$(arg simulation)` in the URDF xacro
            # actually receives "true"/"false".
            mappings={
                "hardware_type": hardware_type.perform(context),
                "simulation": simulation.perform(context),
                "robot_ip": robot_ip.perform(context),
                "robot_port": robot_port.perform(context),
                "connect_timeout_ms": connect_timeout_ms.perform(context),
                "response_timeout_ms": response_timeout_ms.perform(context),
                "control_port": control_port.perform(context),
                "auto_home": auto_home.perform(context),
                **seed_gripper_mappings,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    # --------------------------
    # MoveIt Servo parameters
    # --------------------------
    servo_params = {
        "moveit_servo": ParameterBuilder("meca500_moveit")
        .yaml("config/servo.yaml")
        .to_dict()
    }

    planning_group_name = {"planning_group_name": "meca500"}
    pilz_cartesian_limits = ParameterBuilder("meca500_moveit").yaml(
        "config/pilz_cartesian_limits.yaml"
    ).to_dict()

    # --------------------------
    # Controller config
    # --------------------------
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("meca500_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    # --------------------------
    # RViz config
    # --------------------------
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("meca500_bringup"),
            "config",
            "rviz",
            "moveit.rviz",
        ]
    )

    # --------------------------
    # Nodes
    # --------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=[
            "--x",
            base_x_value,
            "--y",
            base_y_value,
            "--z",
            base_z_value,
            "--frame-id",
            "world",
            "--child-frame-id",
            "meca_base_link",
        ],
    )

    camera_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera_to_robot",
        condition=UnlessCondition(enable_startup_aruco_calibration_value),
        arguments=[
            "--x",
            camera_x_value,
            "--y",
            camera_y_value,
            "--z",
            camera_z_value,
            "--roll",
            camera_roll_value,
            "--pitch",
            camera_pitch_value,
            "--yaw",
            camera_yaw_value,
            "--frame-id",
            realsense_parent_frame_value,
            "--child-frame-id",
            realsense_frame_id_value,
        ],
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
            }
        ],
    )

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

    crop_pointcloud_node = Node(
        package="meca500_bringup",
        executable="crop_pointcloud",
        name="crop_pointcloud",
        output="log",
        condition=IfCondition(enable_realsense_value),
        parameters=[
            {
                "input_topic": point_cloud_topic_value,
                "output_topic": cropped_cloud_topic_value,
                "reference_frame": "world",
                "range": 0.4,
            }
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            controllers_yaml,
            moveit_config.robot_description,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60.0",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_controller,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60.0",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60.0",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            pilz_cartesian_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_pointcloud_params,
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        condition=IfCondition(servo),
        parameters=[
            servo_params,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_scene_monitor,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        arguments=["-d", rviz_config],
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            pilz_cartesian_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
        ],
    )

    table_scene_publisher = Node(
        package="meca500_bringup",
        executable="publish_table_scene.py",
        name="publish_table_scene",
        output="screen",
        condition=IfCondition(show_table_value),
        parameters=[
            {
                "object_id": "mecatable",
                "frame_id": table_frame_value,
                "mesh_path": table_mesh_path_value,
                "mesh_scale": table_scale_value,
                "x": table_x_value,
                "y": table_y_value,
                "z": table_z_value,
                "roll": table_roll_value,
                "pitch": table_pitch_value,
                "yaw": table_yaw_value,
                "startup_delay_s": 3.0,
                "publish_period_s": 1.0,
                "publish_attempts": 10,
            }
        ],
    )

    # Imaging-station scene: TF chain + microscope CollisionObjects from
    # the saved imaging-calibration / microscope-tune yamls. Falls back
    # silently if the launch file or yamls aren't present (lite-native
    # callers that don't have the imaging-station provider on disk).
    show_imaging_station_value = LaunchConfiguration(
        "show_imaging_station"
    ).perform(context).strip().lower() in ("1", "true", "yes", "on")
    imaging_station_scene = None
    if show_imaging_station_value and _DEFAULT_IMAGING_STATION_SCENE_LAUNCH and \
            os.path.isfile(_DEFAULT_IMAGING_STATION_SCENE_LAUNCH):
        imaging_station_scene = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_DEFAULT_IMAGING_STATION_SCENE_LAUNCH),
        )

    # --------------------------
    # Launch ordering
    # --------------------------
    extras_after_gripper = [move_group, servo_node, rviz, table_scene_publisher]
    if imaging_station_scene is not None:
        extras_after_gripper.append(imaging_station_scene)
    return [
        static_tf_node,
        camera_static_tf_node,
        startup_aruco_calibrator_node,
        realsense_node,
        crop_pointcloud_node,
        ros2_control_node,
        robot_state_publisher,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[robot_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[gripper_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_controller_spawner,
                on_exit=extras_after_gripper,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo",
                default_value="false",
                description="Launch MoveIt Servo",
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="joint_trajectory_controller",
                description="ros2_control controller to use",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Run with simulated hardware",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.6.105.53",
                description="Meca500 IP address",
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value="10000",
                description="Robot command port",
            ),
            DeclareLaunchArgument(
                "connect_timeout_ms",
                default_value="2000",
                description="Timeout in milliseconds for the initial TCP connection to the robot.",
            ),
            DeclareLaunchArgument(
                "response_timeout_ms",
                default_value="30000",
                description="Timeout in milliseconds while waiting for robot responses during activation/homing.",
            ),
            DeclareLaunchArgument(
                "control_port",
                default_value="10001",
                description="Host control port",
            ),
            DeclareLaunchArgument(
                'hardware_type',
                default_value='meca500_hardware',
                description='Hardware plugin to load'
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="false",
                description="Issue the Meca500 Home command on hardware "
                            "activation. Off by default — the robot retains "
                            "absolute joint encoders across power cycles, "
                            "so once-per-power-on homing is sufficient and "
                            "skipping it shaves seconds off MoveIt bringup. "
                            "Set true on the first activation after a fresh "
                            "robot power-on.",
            ),
            DeclareLaunchArgument(
                "use_seed_gripper",
                default_value="true",
                description="Bake the 3D-printed seed-gripper geometry into the URDF "
                            "(suppresses the Schunk MEGP 25E finger meshes and adds "
                            "seed_finger_left/right + seed_gripper_tip_link). Set false "
                            "to fall back to the stock Schunk geometry.",
            ),
            DeclareLaunchArgument(
                "seed_gripper_offsets_yaml",
                default_value=_DEFAULT_SEED_GRIPPER_OFFSETS_YAML,
                description="Path to the YAML written by the seed_gripper_visualizer's "
                            "/seed_gripper/save_offsets service. Empty / missing = use "
                            "the xacro defaults (zeros).",
            ),
            DeclareLaunchArgument(
                "seed_gripper_left_mesh",
                default_value=_DEFAULT_SEED_GRIPPER_LEFT_MESH,
                description="Absolute path to the left seed-finger STL.",
            ),
            DeclareLaunchArgument(
                "seed_gripper_right_mesh",
                default_value=_DEFAULT_SEED_GRIPPER_RIGHT_MESH,
                description="Absolute path to the right (mirrored) seed-finger STL.",
            ),
            DeclareLaunchArgument(
                "camera_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("meca500_bringup"), "config", "camera_calibration.yaml"]
                ),
                description="YAML file for RealSense launch and camera calibration transform",
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
                description="Override: launch Intel RealSense (true/false). Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_name",
                default_value="",
                description="Override: RealSense camera_name. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_namespace",
                default_value="",
                description="Override: RealSense camera_namespace. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "realsense_parent_frame",
                default_value="",
                description="Override: parent frame for camera transform. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "realsense_frame_id",
                default_value="",
                description="Override: output camera frame id for calibrated static transform. Empty uses config file or camera_link.",
            ),
            DeclareLaunchArgument(
                "calibration_camera_frame",
                default_value="",
                description="Override: camera frame used for ArUco solve (typically *_color_optical_frame). Empty uses config file or camera_name_color_optical_frame.",
            ),
            DeclareLaunchArgument(
                "table_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("meca500_bringup"), "config", "table.yaml"]
                ),
                description="YAML file with table pose, mesh path, and scale settings.",
            ),
            DeclareLaunchArgument(
                "show_table",
                default_value="",
                description="Override: publish the table mesh (true/false). Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "show_imaging_station",
                default_value="true",
                description="Publish the imaging-station TF chain + microscope STL "
                            "collision objects from the saved imaging-calibration / "
                            "microscope-tune yamls. Set false to skip the include "
                            "entirely (e.g. lite-native callers without the imaging "
                            "provider on disk).",
            ),
            DeclareLaunchArgument(
                "table_mesh_path",
                default_value="",
                description="Override: path to STL mesh. Empty uses config file or default package mesh.",
            ),
            DeclareLaunchArgument(
                "table_frame",
                default_value="",
                description="Override: frame id for table collision object. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_x",
                default_value="",
                description="Override: table X position [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_y",
                default_value="",
                description="Override: table Y position [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_z",
                default_value="",
                description="Override: table Z position [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_roll",
                default_value="",
                description="Override: table roll [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_pitch",
                default_value="",
                description="Override: table pitch [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_yaw",
                default_value="",
                description="Override: table yaw [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "table_scale",
                default_value="",
                description="Override: STL scale factor. Empty uses config file.",
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
                "camera_x",
                default_value="",
                description="Override: camera x [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "camera_y",
                default_value="",
                description="Override: camera y [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "camera_z",
                default_value="",
                description="Override: camera z [m]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "camera_roll",
                default_value="",
                description="Override: camera roll [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "camera_pitch",
                default_value="",
                description="Override: camera pitch [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "camera_yaw",
                default_value="",
                description="Override: camera yaw [rad]. Empty uses config file.",
            ),
            DeclareLaunchArgument(
                "enable_startup_aruco_calibration",
                default_value="",
                description="Override: enable startup ArUco camera calibration. Empty uses aruco config file.",
            ),
            DeclareLaunchArgument(
                "save_startup_calibration",
                default_value="true",
                description="Save calibrated camera transform back to camera_config_file",
            ),
            DeclareLaunchArgument(
                "enable_moveit_pointcloud",
                default_value="false",
                description="Enable MoveIt occupancy map updates from RealSense point cloud.",
            ),
            DeclareLaunchArgument(
                "moveit_pointcloud_max_range",
                default_value="0.4",
                description="Max sensor range in meters used for MoveIt pointcloud occupancy updates.",
            ),
            DeclareLaunchArgument(
                "moveit_pointcloud_subsample",
                default_value="8",
                description="Point subsampling factor for MoveIt pointcloud occupancy updates.",
            ),
            DeclareLaunchArgument(
                "moveit_pointcloud_max_update_rate",
                default_value="0.5",
                description="Max update rate in Hz for MoveIt pointcloud occupancy updates.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
