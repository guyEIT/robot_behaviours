"""
Robot Skills Framework — Remote Robot Skill Atoms Launch.

Run this on any robot PC that is NOT the skill-server PC (e.g. the Franka PC).
Starts skill atoms for the specified robot; they self-register with the central
SkillRegistry on the skill-server PC over DDS.

Requirements:
  - Same ROS_DOMAIN_ID=0 as the skill-server PC
  - ROS_LOCALHOST_ONLY=0 and ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET on all PCs
  - The robot's MoveIt2 instance must already be running on this PC

Usage:
  ros2 launch robot_skill_server skill_atoms_remote.launch.py robot_name:=franka
  ros2 launch robot_skill_server skill_atoms_remote.launch.py \\
      robot_name:=franka robots_config:=/path/to/robots.yaml
"""

import os
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

try:
    _pkg_share = get_package_share_directory("robot_skill_server")
    _pkg_python = os.path.join(_pkg_share, "..", "..", "..", "lib",
                               "python3", "dist-packages")
    if _pkg_python not in sys.path:
        sys.path.insert(0, _pkg_python)
    from robot_skill_server.launch_helpers import build_robot_atom_nodes
except Exception:
    import importlib.util as _ilu
    _src = os.path.join(os.path.dirname(__file__), "..",
                        "robot_skill_server", "launch_helpers.py")
    _spec = _ilu.spec_from_file_location("launch_helpers", _src)
    _mod = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    build_robot_atom_nodes = _mod.build_robot_atom_nodes


def _setup_nodes(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    robots_config_path = LaunchConfiguration("robots_config").perform(context)

    with open(robots_config_path) as f:
        robots_yaml = yaml.safe_load(f)

    robots: dict = robots_yaml.get("robots", {})

    if robot_name not in robots:
        raise RuntimeError(
            f"Robot '{robot_name}' not found in {robots_config_path}. "
            f"Available: {list(robots.keys())}"
        )

    robot_cfg = robots[robot_name]
    ns = robot_cfg.get("namespace", f"/{robot_name}")

    nodes = build_robot_atom_nodes(robot_name, robot_cfg, log_level, ns)

    nodes.append(LogInfo(
        msg=(
            f"\n╔══════════════════════════════════════════════════════╗\n"
            f"║  Remote Skill Atoms: {robot_name:<33}║\n"
            f"╠══════════════════════════════════════════════════════╣\n"
            f"║  Namespace:    {ns:<38}║\n"
            f"║  Planning:     {robot_cfg.get('planning_group', 'arm'):<38}║\n"
            f"║  Registering with central skill server over DDS      ║\n"
            f"╚══════════════════════════════════════════════════════╝"
        )
    ))

    return nodes


def generate_launch_description():
    try:
        default_robots_config = os.path.join(
            get_package_share_directory("robot_skill_server"),
            "config", "robots.yaml"
        )
    except Exception:
        default_robots_config = "config/robots.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            description="Robot name from robots.yaml (e.g. 'franka', 'meca500')",
        ),
        DeclareLaunchArgument(
            "robots_config",
            default_value=EnvironmentVariable(
                "ROBOTS_CONFIG", default_value=default_robots_config
            ),
            description="Path to robots.yaml configuration file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS2 log level: debug, info, warn, error",
        ),
        OpaqueFunction(function=_setup_nodes),
    ])
