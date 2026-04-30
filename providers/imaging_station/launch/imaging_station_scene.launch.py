"""Publish the imaging-station TF frames + collision-object meshes.

Same pattern as providers/meca500/.../publish_table_scene.py for the
mecatable: after meca500-calibrate-up + tune-microscope have been run, the
two saved yamls contain everything needed to reconstruct the imaging-station
geometry. This launch is meant to be included from moveit.launch.py so the
imaging station is always in the planning scene.

Reads:
  - providers/meca500/src/meca500_bringup/config/imaging_calibration.yaml
        -> camera_link -> calibration_marker -> calibration_target_centre
  - providers/imaging_station/calibration/microscope_offsets.yaml
        -> calibration_target_centre -> microscope_stand / microscope_camera

Publishes:
  * static TFs for every link in the chain
  * MoveIt CollisionObjects for the microscope-stand and microscope-camera
    STLs (each parented to its own TF frame at identity pose, so the mesh
    sits exactly where the calibrated frame sits)

Disable the whole include with `enable_imaging_station_scene:=false`. Skip
just the collision objects with `imaging_station_collision:=false` when you
want the TFs but not the planning-scene additions (e.g. lite RViz inspection).
"""
from __future__ import annotations

from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
PROVIDER_DIR = HERE.parent
MODELS_DIR = PROVIDER_DIR / "models"


def _find_workspace_root() -> Path:
    """Walk up to the workspace root (the dir containing pixi.toml).

    pixi-build-ros copies launch files into .pixi/envs/.../share/, so
    __file__ doesn't reach the source tree directly when we resolve
    sibling provider files. Walking up still works because pixi installs
    under the workspace.
    """
    here = Path(__file__).resolve().parent
    for anchor in [here, *here.parents]:
        if (anchor / "pixi.toml").exists():
            return anchor
    return Path.cwd()


_WS_ROOT = _find_workspace_root()


def _resolve_meca_bringup_config(filename: str) -> str:
    """Locate a meca500_bringup config yaml.

    The startup calibrator writes back to the *install* share dir (via
    FindPackageShare), so prefer that copy. Fall back to the source-tree
    copy when running off a fresh checkout that hasn't been built yet.
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


DEFAULT_IMAGING_CALIBRATION_YAML = _resolve_meca_bringup_config("imaging_calibration.yaml")
DEFAULT_MICROSCOPE_OFFSETS_YAML = str(
    _WS_ROOT / "providers" / "imaging_station" / "calibration"
    / "microscope_offsets.yaml"
)
DEFAULT_STAND_MESH = str(MODELS_DIR / "microscope-stand.stl")
DEFAULT_CAMERA_MESH = str(MODELS_DIR / "microscope-camera.stl")


def _load_yaml_section(path: str, key: str) -> dict:
    p = Path(path)
    if not p.is_file():
        return {}
    return (yaml.safe_load(p.read_text()) or {}).get(key, {}) or {}


def _static_tf_node(name: str, parent: str, child: str, pose: dict):
    """Build a tf2_ros static_transform_publisher Node from a pose dict."""
    if not parent or not child:
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


def _offsets_pose(section: dict) -> dict:
    """Convert {xyz: [...], rpy: [...]} into a flat dict the tf node accepts."""
    xyz = section.get("xyz") or [0.0, 0.0, 0.0]
    rpy = section.get("rpy") or [0.0, 0.0, 0.0]
    return {
        "x": float(xyz[0]),
        "y": float(xyz[1]),
        "z": float(xyz[2]),
        "roll": float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw": float(rpy[2]),
    }


def _collision_object_node(
    name: str, object_id: str, frame_id: str, mesh_path: str, scale: float
):
    """Run publish_table_scene.py from meca500_bringup with the right params.

    Reuses the mecatable publisher because it's already a generic
    "STL + pose -> CollisionObject" tool. Pose is identity since the TF
    frame already places the mesh at the correct world position.
    """
    return Node(
        package="meca500_bringup",
        executable="publish_table_scene.py",
        name=name,
        output="screen",
        parameters=[
            {
                "object_id": object_id,
                "frame_id": frame_id,
                "mesh_path": mesh_path,
                "mesh_scale": scale,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                # Mirror the mecatable defaults — give the move_group time
                # to come up before pushing the collision object.
                "startup_delay_s": 3.0,
                "publish_period_s": 1.0,
                "publish_attempts": 10,
            }
        ],
    )


def launch_setup(context, *args, **kwargs):
    enable_collision = LaunchConfiguration("imaging_station_collision")
    imaging_calibration_file = LaunchConfiguration("imaging_calibration_file").perform(context)
    microscope_offsets_file = LaunchConfiguration("microscope_offsets_file").perform(context)
    stand_mesh = LaunchConfiguration("stand_mesh").perform(context)
    camera_mesh = LaunchConfiguration("camera_mesh").perform(context)
    stand_scale = float(LaunchConfiguration("stand_scale").perform(context))
    camera_scale = float(LaunchConfiguration("camera_scale").perform(context))

    imaging_cal = _load_yaml_section(imaging_calibration_file, "imaging_calibration")
    offsets = yaml.safe_load(Path(microscope_offsets_file).read_text() or "") if Path(microscope_offsets_file).is_file() else {}
    stand_section = (offsets or {}).get("stand", {}) or {}
    camera_section = (offsets or {}).get("camera", {}) or {}

    target_frame = imaging_cal.get("target_frame_id", "calibration_target_centre")
    marker_frame = imaging_cal.get("marker_frame_id", "calibration_marker")
    camera_parent = imaging_cal.get("parent_frame", "camera_link")
    stand_frame = "microscope_stand"
    camera_frame = "microscope_camera"

    nodes = []

    # Calibration replay: camera_link -> marker -> centre.
    if imaging_cal:
        nodes.extend(
            n for n in (
                _static_tf_node(
                    "static_tf_imaging_marker",
                    camera_parent,
                    marker_frame,
                    imaging_cal.get("marker_in_camera", {}),
                ),
                _static_tf_node(
                    "static_tf_imaging_target_centre",
                    marker_frame,
                    target_frame,
                    imaging_cal.get("target_in_marker", {}),
                ),
            ) if n is not None
        )
    else:
        # No saved imaging calibration -> publish a stub at world origin so
        # microscope_stand / microscope_camera (which usually parent on
        # calibration_target_centre) still have a reachable parent. The rig
        # will appear at world origin instead of the imaging plane until cal
        # runs. Warn loudly so the operator can spot it.
        print(
            "[imaging_station_scene] WARNING: "
            f"{imaging_calibration_file} not found. "
            "The microscope CAD will be anchored at world origin. "
            "Run `pixi run meca500-calibrate-up` (with both fiducials in "
            "view) to write imaging_calibration.yaml and put the rig at "
            "the optical centre."
        )
        nodes.append(
            _static_tf_node(
                "static_tf_imaging_target_centre_fallback",
                "world",
                target_frame,
                {},
            )
        )

    # Microscope frame poses from the saved tune.
    if stand_section:
        stand_parent = stand_section.get("parent_frame", target_frame)
        nodes.append(
            _static_tf_node(
                "static_tf_microscope_stand",
                stand_parent,
                stand_frame,
                _offsets_pose(stand_section),
            )
        )
    if camera_section:
        camera_parent_frame = camera_section.get("parent_frame", target_frame)
        nodes.append(
            _static_tf_node(
                "static_tf_microscope_camera",
                camera_parent_frame,
                camera_frame,
                _offsets_pose(camera_section),
            )
        )

    # Collision objects. publish_table_scene.py is a generic STL pusher in
    # meca500_bringup; conditioning on enable_collision lets you keep the
    # TFs but skip the MoveIt scene write (e.g. when running tune-microscope
    # which has no move_group).
    if Path(stand_mesh).is_file():
        nodes.append(
            Node(
                package="meca500_bringup",
                executable="publish_table_scene.py",
                name="publish_microscope_stand_scene",
                output="screen",
                condition=IfCondition(enable_collision),
                parameters=[{
                    "object_id": stand_frame,
                    "frame_id": stand_frame,
                    "mesh_path": stand_mesh,
                    "mesh_scale": stand_scale,
                    "x": 0.0, "y": 0.0, "z": 0.0,
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                    "startup_delay_s": 3.0,
                    "publish_period_s": 1.0,
                    "publish_attempts": 10,
                }],
            )
        )
    if Path(camera_mesh).is_file():
        nodes.append(
            Node(
                package="meca500_bringup",
                executable="publish_table_scene.py",
                name="publish_microscope_camera_scene",
                output="screen",
                condition=IfCondition(enable_collision),
                parameters=[{
                    "object_id": camera_frame,
                    "frame_id": camera_frame,
                    "mesh_path": camera_mesh,
                    "mesh_scale": camera_scale,
                    "x": 0.0, "y": 0.0, "z": 0.0,
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                    "startup_delay_s": 3.0,
                    "publish_period_s": 1.0,
                    "publish_attempts": 10,
                }],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "imaging_calibration_file",
            default_value=DEFAULT_IMAGING_CALIBRATION_YAML,
            description="YAML written by meca500-calibrate-up with the "
                        "averaged camera->marker plus the static marker->centre offset.",
        ),
        DeclareLaunchArgument(
            "microscope_offsets_file",
            default_value=DEFAULT_MICROSCOPE_OFFSETS_YAML,
            description="YAML written by tune-microscope with the calibrated "
                        "centre->stand and centre->camera poses.",
        ),
        DeclareLaunchArgument(
            "stand_mesh",
            default_value=DEFAULT_STAND_MESH,
            description="Absolute path to the microscope-stand STL.",
        ),
        DeclareLaunchArgument(
            "camera_mesh",
            default_value=DEFAULT_CAMERA_MESH,
            description="Absolute path to the microscope-camera STL.",
        ),
        DeclareLaunchArgument(
            "stand_scale", default_value="0.001",
            description="STL unit scale for the microscope-stand mesh (mm->m).",
        ),
        DeclareLaunchArgument(
            "camera_scale", default_value="0.001",
            description="STL unit scale for the microscope-camera mesh (mm->m).",
        ),
        DeclareLaunchArgument(
            "imaging_station_collision", default_value="true",
            description="Push the microscope STLs as MoveIt collision objects "
                        "in addition to the TFs.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
