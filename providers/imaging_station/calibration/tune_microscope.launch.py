"""One-shot launch for tuning the microscope stand + camera placement.

Mirrors providers/meca500/tools/tune_gripper.launch.py, but for the imaging
station's two fixtures. Brings up:

    * a static TF world -> meca_base_link so the meca arm can be visualised
      alongside the rig (skip with publish_world_tf:=false if the meca500
      bringup is already running and owns this TF)
    * microscope_visualizer (MarkerArray + InteractiveMarkers + parameter
      sliders + StaticTF for the microscope chain)
    * rqt_reconfigure (sliders for stand / camera xyz/rpy)
    * rviz2 with microscope.rviz

By default the stand parents to calibration_target_centre — the ArUco-
resolved imaging centre. The launch is fully standalone: it replays the
previously-saved calibration from camera_calibration.yaml and
imaging_calibration.yaml as static TFs at startup, so meca500-calibrate-up
does NOT need to be running. Re-run that whenever the camera moves; tune
this CAD whenever the rig itself moves. Pass stand_parent_frame:=world to
tune against the world origin instead.

Save tuned offsets:
    ros2 service call /microscope/save_offsets std_srvs/srv/Trigger {}

Run alongside an existing meca500 launch (which already publishes world):
    ros2 launch providers/imaging_station/calibration/tune_microscope.launch.py \
        publish_world_tf:=false
"""
from __future__ import annotations

from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
PROVIDER_DIR = HERE.parent
MODELS_DIR = PROVIDER_DIR / "models"
VISUALIZER_SCRIPT = str(HERE / "microscope_visualizer.py")
RVIZ_CONFIG = str(HERE / "microscope.rviz")
DEFAULT_OFFSETS_YAML = str(HERE / "microscope_offsets.yaml")
DEFAULT_STAND_STL = str(MODELS_DIR / "microscope-stand.stl")
DEFAULT_CAMERA_STL = str(MODELS_DIR / "microscope-camera.stl")

# Reference fixtures: meca500 base mesh + mecatable. Resolved against the
# source tree so RViz can mesh-load them without colcon-installing them
# into a share dir we'd then have to re-resolve.
_WS_ROOT = HERE.parents[2]  # providers/imaging_station/calibration -> WS root
DEFAULT_MECA_BASE_MESH = str(
    _WS_ROOT / "providers" / "meca500" / "src" / "meca500_description"
    / "meshes" / "visual" / "meca_500_r3_base.dae"
)
DEFAULT_MECATABLE_MESH = str(
    _WS_ROOT / "providers" / "meca500" / "src" / "meca500_bringup"
    / "meshes" / "mecatable_simple.stl"
)
def _resolve_meca_bringup_config(filename: str) -> str:
    """Locate a meca500_bringup config yaml, preferring the install copy.

    The startup calibrator writes back to the install share dir
    (FindPackageShare("meca500_bringup")/config/<file>), so check that
    first. Fall back to the source tree for fresh checkouts.
    """
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("meca500_bringup")) / "config" / filename
        )
    except Exception:
        pass
    candidates.append(
        _WS_ROOT / "providers" / "meca500" / "src" / "meca500_bringup"
        / "config" / filename
    )
    for c in candidates:
        if c.is_file():
            return str(c)
    return str(candidates[0])


DEFAULT_TABLE_YAML = _resolve_meca_bringup_config("table.yaml")
DEFAULT_CAMERA_CALIBRATION_YAML = _resolve_meca_bringup_config("camera_calibration.yaml")
DEFAULT_IMAGING_CALIBRATION_YAML = _resolve_meca_bringup_config("imaging_calibration.yaml")


def _load_table_pose(path: str) -> dict:
    """Read providers/meca500/.../table.yaml; return {} if missing."""
    p = Path(path)
    if not p.is_file():
        return {}
    data = yaml.safe_load(p.read_text()) or {}
    return data.get("table", {}) or {}


def _load_camera_calibration(path: str) -> dict:
    p = Path(path)
    if not p.is_file():
        return {}
    return (yaml.safe_load(p.read_text()) or {}).get("camera_calibration", {}) or {}


def _load_imaging_calibration(path: str) -> dict:
    p = Path(path)
    if not p.is_file():
        return {}
    return (yaml.safe_load(p.read_text()) or {}).get("imaging_calibration", {}) or {}


def _static_tf_node(name: str, parent: str, child: str, pose: dict):
    """Build a tf2_ros static_transform_publisher Node from a pose dict.

    Returns None if pose is empty / missing required keys.
    """
    if not pose or not parent or not child:
        return None
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(pose.get("x", 0.0)),
            "--y", str(pose.get("y", 0.0)),
            "--z", str(pose.get("z", 0.0)),
            "--roll", str(pose.get("roll", 0.0)),
            "--pitch", str(pose.get("pitch", 0.0)),
            "--yaw", str(pose.get("yaw", 0.0)),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
    )


def generate_launch_description() -> LaunchDescription:
    enable_rqt = LaunchConfiguration("enable_rqt")
    enable_rviz = LaunchConfiguration("enable_rviz")
    publish_world_tf = LaunchConfiguration("publish_world_tf")
    offsets_yaml = LaunchConfiguration("offsets_yaml")
    stand_mesh = LaunchConfiguration("stand_mesh")
    camera_mesh = LaunchConfiguration("camera_mesh")
    stand_parent_frame = LaunchConfiguration("stand_parent_frame")
    base_x = LaunchConfiguration("base_x")
    base_y = LaunchConfiguration("base_y")
    base_z = LaunchConfiguration("base_z")
    meca_base_mesh = LaunchConfiguration("meca_base_mesh")
    mecatable_mesh = LaunchConfiguration("mecatable_mesh")

    # Replay the previously-calibrated chain from disk so tune-microscope
    # is fully standalone (no live camera / ArUco needed). Built from the
    # two yamls written by meca500-calibrate-up:
    #   camera_calibration.yaml   -> meca_base_link -> camera_link
    #   imaging_calibration.yaml  -> camera_link -> calibration_marker -> centre
    # If imaging_calibration.yaml is missing (ran the old calibrator that
    # didn't persist the imaging fiducial), the launch falls back to
    # parenting the stand on `world` and prints a hint about re-running cal.
    camera_cal = _load_camera_calibration(DEFAULT_CAMERA_CALIBRATION_YAML)
    imaging_cal = _load_imaging_calibration(DEFAULT_IMAGING_CALIBRATION_YAML)
    has_imaging_cal = bool(imaging_cal)
    default_stand_parent_frame = (
        imaging_cal.get("target_frame_id", "calibration_target_centre")
        if has_imaging_cal
        else "world"
    )
    if not has_imaging_cal:
        print(
            "[tune-microscope] imaging_calibration.yaml not found at "
            f"{DEFAULT_IMAGING_CALIBRATION_YAML} — anchoring stand on "
            "'world' instead of 'calibration_target_centre'. Re-run "
            "`pixi run meca500-calibrate-up` once with the updated "
            "calibrator to persist the imaging-fiducial detection."
        )

    cam_parent = camera_cal.get("parent_frame", "meca_base_link")
    cam_child = camera_cal.get("frame_id", "camera_link")
    camera_extrinsic_tf_node = _static_tf_node(
        "static_tf_camera_extrinsic_replay",
        cam_parent,
        cam_child,
        camera_cal,
    )

    imaging_marker_tf_node = _static_tf_node(
        "static_tf_imaging_marker_replay",
        imaging_cal.get("parent_frame", cam_child),
        imaging_cal.get("marker_frame_id", "calibration_marker"),
        imaging_cal.get("marker_in_camera", {}),
    )
    imaging_target_tf_node = _static_tf_node(
        "static_tf_imaging_target_replay",
        imaging_cal.get("marker_frame_id", "calibration_marker"),
        imaging_cal.get("target_frame_id", "calibration_target_centre"),
        imaging_cal.get("target_in_marker", {}),
    )

    # Read table.yaml at launch time so the mecatable lands at the same pose
    # MoveIt would publish it. Falls back to the schema defaults if the file
    # isn't present.
    table_pose = _load_table_pose(DEFAULT_TABLE_YAML)
    mecatable_x_default = str(table_pose.get("x", 0.0))
    mecatable_y_default = str(table_pose.get("y", 0.08))
    mecatable_z_default = str(table_pose.get("z", -0.23))
    mecatable_roll_default = str(table_pose.get("roll", 1.5708))
    mecatable_pitch_default = str(table_pose.get("pitch", 0.0))
    mecatable_yaw_default = str(table_pose.get("yaw", 1.5708))
    mecatable_scale_default = str(table_pose.get("scale", 0.001))

    # Standalone TF root: world -> meca_base_link. Skip when running on top
    # of an active meca500 bringup that already owns this transform.
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base_microscope_tune",
        condition=IfCondition(publish_world_tf),
        arguments=[
            "--x", base_x,
            "--y", base_y,
            "--z", base_z,
            "--frame-id", "world",
            "--child-frame-id", "meca_base_link",
        ],
    )

    visualizer = ExecuteProcess(
        cmd=[
            "python3", VISUALIZER_SCRIPT,
            "--ros-args",
            "-r", "__node:=microscope_visualizer",
            "-p", ["offsets_yaml_path:=", offsets_yaml],
            "-p", ["stand_mesh_path:=", stand_mesh],
            "-p", ["camera_mesh_path:=", camera_mesh],
            "-p", ["stand_parent_frame:=", stand_parent_frame],
            "-p", ["meca_base_mesh_path:=", meca_base_mesh],
            "-p", ["mecatable_mesh_path:=", mecatable_mesh],
            "-p", f"mecatable_x:={mecatable_x_default}",
            "-p", f"mecatable_y:={mecatable_y_default}",
            "-p", f"mecatable_z:={mecatable_z_default}",
            "-p", f"mecatable_roll:={mecatable_roll_default}",
            "-p", f"mecatable_pitch:={mecatable_pitch_default}",
            "-p", f"mecatable_yaw:={mecatable_yaw_default}",
            "-p", f"mecatable_scale:={mecatable_scale_default}",
        ],
        output="screen",
    )

    # Delay rqt so the visualizer's parameters are already declared and
    # discoverable when the GUI populates its node list.
    rqt = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rqt_reconfigure",
                executable="rqt_reconfigure",
                name="rqt_reconfigure",
                output="screen",
                condition=IfCondition(enable_rqt),
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_microscope_tune",
        output="screen",
        arguments=["-d", RVIZ_CONFIG],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_rqt", default_value="true",
            description="Auto-spawn rqt_reconfigure for the slider panel."),
        DeclareLaunchArgument(
            "enable_rviz", default_value="true",
            description="Spawn RViz with microscope.rviz."),
        DeclareLaunchArgument(
            "publish_world_tf", default_value="true",
            description="Publish world->meca_base_link static TF (set false "
                        "when an existing meca500 launch already owns this)."),
        DeclareLaunchArgument(
            "offsets_yaml", default_value=DEFAULT_OFFSETS_YAML,
            description="YAML for save_offsets / load_offsets services."),
        DeclareLaunchArgument(
            "stand_mesh", default_value=DEFAULT_STAND_STL,
            description="Absolute path to the microscope-stand STL."),
        DeclareLaunchArgument(
            "camera_mesh", default_value=DEFAULT_CAMERA_STL,
            description="Absolute path to the microscope-camera STL."),
        DeclareLaunchArgument(
            "stand_parent_frame", default_value="calibration_target_centre",
            description="Parent frame the microscope_stand TF is published in. "
                        "Default ties the rig to the ArUco-calibrated imaging "
                        "centre — requires meca500-calibrate-up running. "
                        "Pass stand_parent_frame:=world for standalone tuning."),
        DeclareLaunchArgument(
            "base_x", default_value="0.0",
            description="meca_base_link x offset in world frame [m]."),
        DeclareLaunchArgument(
            "base_y", default_value="0.0",
            description="meca_base_link y offset in world frame [m]."),
        DeclareLaunchArgument(
            "base_z", default_value="0.0",
            description="meca_base_link z offset in world frame [m]."),
        DeclareLaunchArgument(
            "meca_base_mesh", default_value=DEFAULT_MECA_BASE_MESH,
            description="Mesh file for the meca500 base (rendered as a static reference). "
                        "Set to an empty string to hide."),
        DeclareLaunchArgument(
            "mecatable_mesh", default_value=DEFAULT_MECATABLE_MESH,
            description="Mesh file for the mecatable (rendered as a static reference at "
                        "the pose from table.yaml). Set to an empty string to hide."),
        world_to_base,
        # Replay calibration chain from yaml; absent/empty yamls are skipped.
        *[n for n in (
            camera_extrinsic_tf_node,
            imaging_marker_tf_node,
            imaging_target_tf_node,
        ) if n is not None],
        visualizer,
        rqt,
        rviz,
    ])
