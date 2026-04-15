"""
Robot Skills Framework — Multi-Robot System Launch.

Reads src/robot_skill_server/config/robots.yaml and:
  - Launches skill atoms for every robot with `local: true` (runs on this PC)
  - Starts the central skill server (registry, composer, BT executor)
  - Passes the full robot list to bt_runner so per-robot BT node types are registered
  - Starts the robot dashboard

Remote robots (local: false) run skill_atoms_remote.launch.py on their own PCs
and self-register with the central registry over DDS.

Usage:
  ros2 launch robot_skill_server robot_system.launch.py
  ros2 launch robot_skill_server robot_system.launch.py robots_config:=/path/to/robots.yaml
  ros2 launch robot_skill_server robot_system.launch.py log_level:=debug
"""

import os
import sys
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Add the package to sys.path so the shared helper can be imported at launch time
try:
    _pkg_share = get_package_share_directory("robot_skill_server")
    _pkg_python = os.path.join(_pkg_share, "..", "..", "..", "lib",
                               "python3", "dist-packages")
    if _pkg_python not in sys.path:
        sys.path.insert(0, _pkg_python)
    from robot_skill_server.launch_helpers import build_robot_atom_nodes
except Exception:
    # Fallback: import directly from source tree if not installed
    import importlib.util as _ilu
    _src = os.path.join(os.path.dirname(__file__), "..",
                        "robot_skill_server", "launch_helpers.py")
    _spec = _ilu.spec_from_file_location("launch_helpers", _src)
    _mod = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    build_robot_atom_nodes = _mod.build_robot_atom_nodes


def _setup_nodes(context, *args, **kwargs):
    """Load robots.yaml and build the full node graph."""
    log_level = LaunchConfiguration("log_level").perform(context)
    robots_config_path = LaunchConfiguration("robots_config").perform(context)
    groot_port = LaunchConfiguration("groot_port").perform(context)

    with open(robots_config_path) as f:
        robots_yaml = yaml.safe_load(f)

    robots: dict = robots_yaml.get("robots", {})
    if not robots:
        raise RuntimeError(f"No robots defined in {robots_config_path}")

    # "meca500:/meca500,franka:/franka" — passed to bt_runner so it registers
    # BT node types for ALL robots, including remote ones
    robot_configs_str = ",".join(
        f"{rid}:{cfg['namespace']}" for rid, cfg in robots.items()
    )

    bt_runner_exec = os.path.join(
        get_package_share_directory("robot_skill_server"),
        "..", "..", "lib", "robot_skill_server", "bt_runner"
    )

    # ── Skill Server ─────────────────────────────────────────────────────────
    skill_server_node = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "groot_zmq_port": int(groot_port),
            "tick_rate_hz": 10.0,
            "bt_runner_executable": bt_runner_exec,
            "robot_configs": robot_configs_str,
        }],
    )

    # ── Dashboard ─────────────────────────────────────────────────────────────
    dashboard_share = get_package_share_directory("robot_dashboard")
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dashboard_share, "launch", "dashboard.launch.py")
        ),
    )

    all_nodes = [skill_server_node, dashboard_launch]

    # ── Skill atoms for local robots ─────────────────────────────────────────
    local_robot_ids = []
    remote_robot_ids = []
    for robot_id, robot_cfg in robots.items():
        ns = robot_cfg.get("namespace", f"/{robot_id}")
        if robot_cfg.get("local", False):
            local_robot_ids.append(robot_id)
            all_nodes.extend(build_robot_atom_nodes(robot_id, robot_cfg, log_level, ns))
        else:
            remote_robot_ids.append(robot_id)

    # ── Info banner ───────────────────────────────────────────────────────────
    all_nodes.append(LogInfo(
        msg=(
            f"\n╔══════════════════════════════════════════════════════╗\n"
            f"║  Multi-Robot Skills Framework                        ║\n"
            f"╠══════════════════════════════════════════════════════╣\n"
            f"║  Local robots:   {', '.join(local_robot_ids) or 'none':<36}║\n"
            f"║  Remote robots:  {', '.join(remote_robot_ids) or 'none':<36}║\n"
            f"║  Config:         {robots_config_path:<36}║\n"
            f"║  Dashboard:      http://localhost:8081                ║\n"
            f"╚══════════════════════════════════════════════════════╝"
        )
    ))

    return all_nodes


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
        DeclareLaunchArgument(
            "groot_port",
            default_value="1666",
            description="Groot2 ZMQ publisher port for live BT monitoring",
        ),
        OpaqueFunction(function=_setup_nodes),
    ])
