"""One-shot launch for tuning the seed-gripper finger placement.

Lightweight visualisation: NO MoveIt, NO ros2_control, NO controller_manager.
Just enough to display the URDF + drag the gripper joints + see the seed-finger
geometry and the corrected grip point.

The robot_description served to RViz / robot_state_publisher is the upstream
meca500.xacro expanded with use_seed_gripper=true, so the Schunk finger
meshes are suppressed and seed_finger_left/right + seed_gripper_tip_link are
materialised. Offsets come from providers/meca500/tools/seed_gripper_offsets.yaml
- the same file the main bringup (meca500-moveit-run / -real-up / -sim-test)
reads, so RViz + MoveIt + the tuning launch all share one source of truth.

Workflow: tune live with rqt_reconfigure / interactive markers -> save the
YAML -> close & relaunch -> URDF reflects the new zero/grip point. The same
relaunch updates every other launch that consumes the YAML.

Brings up:
    * robot_state_publisher (URDF = meca500.xacro with seed-gripper overlay on)
    * joint_state_publisher_gui (slider window for every robot joint - use
      meca_gripper_finger1_joint to open/close the gripper)
    * seed_gripper_visualizer (MarkerArray + InteractiveMarkers, with save/load
      services and rqt-tunable parameters)
    * rqt_reconfigure (sliders for the finger / grip-point offsets)
    * rviz2 with seed_gripper.rviz (RobotModel + MarkerArray + InteractiveMarkers)

Save tuned offsets:
    ros2 service call /seed_gripper/save_offsets std_srvs/srv/Trigger {}
"""
from __future__ import annotations

from pathlib import Path

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TOOLS_DIR = Path(__file__).resolve().parent
PROVIDER_DIR = TOOLS_DIR.parent
VISUALIZER_SCRIPT = str(TOOLS_DIR / "seed_gripper_visualizer.py")
RVIZ_CONFIG = str(TOOLS_DIR / "seed_gripper.rviz")
DEFAULT_OFFSETS_YAML = str(TOOLS_DIR / "seed_gripper_offsets.yaml")
DEFAULT_LEFT_STL = str(PROVIDER_DIR / "seed_gripper-Finger.stl")
DEFAULT_RIGHT_STL = str(PROVIDER_DIR / "seed_gripper-Finger-right.stl")
URDF_XACRO = str(
    PROVIDER_DIR / "src" / "meca500_description" / "urdf" / "meca500.xacro"
)


_AXES = ("x", "y", "z")
_RPY = ("r", "p", "y")


def _load_offsets(path: Path) -> dict:
    """Read seed_gripper_offsets.yaml; return {} if missing or empty."""
    if not path.exists():
        return {}
    return yaml.safe_load(path.read_text()) or {}


def _build_robot_description(yaml_path: Path, left_mesh: str, right_mesh: str) -> str:
    """Expand meca500.xacro with use_seed_gripper=true and the saved offsets."""
    data = _load_offsets(yaml_path)

    mappings: dict[str, str] = {
        "use_seed_gripper": "true",
        "left_mesh": left_mesh,
        "right_mesh": right_mesh,
        "stl_scale": str(float(data.get("stl_scale", 0.001))),
    }
    for key in ("left", "right", "grip"):
        section = data.get(key) or {}
        xyz = section.get("xyz") or [0.0, 0.0, 0.0]
        rpy = section.get("rpy") or [0.0, 0.0, 0.0]
        for i, axis in enumerate(_AXES):
            mappings[f"{key}_xyz_{axis}"] = str(float(xyz[i]))
        for i, axis in enumerate(_RPY):
            mappings[f"{key}_rpy_{axis}"] = str(float(rpy[i]))

    return xacro.process_file(URDF_XACRO, mappings=mappings).toxml()


def generate_launch_description() -> LaunchDescription:
    enable_rqt = LaunchConfiguration("enable_rqt")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_jsp_gui = LaunchConfiguration("enable_jsp_gui")
    offsets_yaml = LaunchConfiguration("offsets_yaml")
    left_mesh = LaunchConfiguration("left_mesh")
    right_mesh = LaunchConfiguration("right_mesh")

    # Resolve the YAML / mesh args at launch time (LaunchConfiguration values
    # become real strings only inside an OpaqueFunction, but the defaults
    # here are static so we can read them directly via the declared default).
    yaml_default = Path(DEFAULT_OFFSETS_YAML)
    robot_description = {
        "robot_description": _build_robot_description(
            yaml_default, DEFAULT_LEFT_STL, DEFAULT_RIGHT_STL,
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(enable_jsp_gui),
    )

    visualizer = ExecuteProcess(
        cmd=[
            "python3", VISUALIZER_SCRIPT,
            "--ros-args",
            "-r", "__node:=seed_gripper_visualizer",
            "-p", ["offsets_yaml_path:=", offsets_yaml],
            "-p", ["left_mesh_path:=", left_mesh],
            "-p", ["right_mesh_path:=", right_mesh],
        ],
        output="screen",
    )

    # Delay rqt so the visualizer's parameters are already declared and
    # discoverable when the GUI populates its node list — otherwise it can
    # come up empty and need a manual refresh.
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
        name="rviz2_seed_gripper",
        output="screen",
        arguments=["-d", RVIZ_CONFIG],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_rqt", default_value="true",
                              description="Auto-spawn rqt_reconfigure for the slider panel."),
        DeclareLaunchArgument("enable_rviz", default_value="true",
                              description="Spawn RViz with the seed_gripper.rviz config."),
        DeclareLaunchArgument("enable_jsp_gui", default_value="true",
                              description="Spawn joint_state_publisher_gui (slider window for arm + gripper joints)."),
        DeclareLaunchArgument("offsets_yaml", default_value=DEFAULT_OFFSETS_YAML,
                              description="YAML for save_offsets / load_offsets services."),
        DeclareLaunchArgument("left_mesh", default_value=DEFAULT_LEFT_STL,
                              description="Absolute path to the left-finger STL."),
        DeclareLaunchArgument("right_mesh", default_value=DEFAULT_RIGHT_STL,
                              description="Absolute path to the right-finger STL (mirrored)."),
        rsp,
        jsp_gui,
        visualizer,
        rqt,
        rviz,
    ])
