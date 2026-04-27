"""Lab "full": real-mode + sim-mode action servers running side-by-side.

This is the dev-box test bed for the SIM_THEN_REAL workflow. It brings up:
  - A real-mode Meca500 stack (MoveIt2 + atoms) under bare paths
  - A sim-mode Meca500 stack under /sim (PushRosNamespace)
  - Real Hamilton STAR action server at /hamilton_star_action_server/*
  - Sim  Hamilton STAR action server at /sim/hamilton_star_action_server/*
  - Real Liconic action server at /liconic_action_server/*
  - Sim  Liconic action server at /sim/liconic_action_server/*
  - The skill_server orchestrator (BtExecutor + SkillRegistry + TaskComposer)
  - The web dashboard (rosbridge :9090, static UI :8081)

On a dev box without physical hardware the "real" side still runs against
mock_components / simulator backends — the point of this launch is to exercise
the orchestrator's namespace-rewriting and dry-run flow, not to drive metal.

Run:

    pixi run lab-up

Then submit a goal with target_mode=MODE_SIM_THEN_REAL — the executor runs
the tree against the /sim/* atoms first, publishes DryRunStatus, parks until
the operator (dashboard / MCP / ROS service caller) calls
/skill_server/approve_dry_run, then re-runs against the real-mode atoms.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SIM_PREFIX = "/sim"


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    dashboard_port = LaunchConfiguration("dashboard_port")
    enable_real_meca500_moveit = LaunchConfiguration("enable_real_meca500_moveit")

    # ── Real-mode Meca500 (MoveIt2 + atoms; bare paths) ──────────────────
    # On a hardware box this would be `simulation:=false` and connect to the
    # actual Meca over its IP. The dev-box default uses mock_components so
    # both sides can coexist on one machine.
    real_meca500_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "true",
            "enable_realsense": "false",
        }.items(),
        condition=None,
    )
    real_meca500_atoms = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("meca500_skill_server"),
                    "/launch/meca500_skill_server.launch.py",
                ]),
                launch_arguments={
                    "robot_id": "meca500",
                    # No namespace_prefix → bare /meca500/skill_atoms/* paths,
                    # manifest at /meca500_skill_server/skills (visible to
                    # SkillDiscovery — sim manifest is filtered).
                }.items(),
            ),
        ],
    )

    # ── Sim-mode Meca500 (under /sim) ────────────────────────────────────
    # Two MoveIt instances on one box is heavy; the sim atoms reuse the real
    # MoveIt by default (action paths still resolve under /sim because the
    # skill atoms re-broadcast their advertised paths there). If you need a
    # truly independent sim MoveIt, set enable_real_meca500_moveit:=false and
    # run a second MoveIt manually under /sim.
    sim_meca500_atoms = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("meca500_skill_server"),
                    "/launch/meca500_skill_server.launch.py",
                ]),
                launch_arguments={
                    "robot_id": "meca500",
                    "namespace_prefix": SIM_PREFIX,
                }.items(),
            ),
        ],
    )

    # ── Hamilton STAR (real + sim) ───────────────────────────────────────
    real_hamilton = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("hamilton_star_bringup"),
            "/launch/action_server.launch.py",
        ]),
        # Real-side defaults to the simulator backend on the dev box too —
        # there's no real STAR attached. Override to backend:=star on a
        # production box.
        launch_arguments={"backend": "simulator"}.items(),
    )
    sim_hamilton = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("hamilton_star_bringup"),
            "/launch/action_server.launch.py",
        ]),
        launch_arguments={
            "backend": "simulator",
            "namespace_prefix": SIM_PREFIX,
        }.items(),
    )

    # ── Liconic (real + sim) ─────────────────────────────────────────────
    real_liconic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("liconic_ros"),
            "/launch/liconic.launch.py",
        ]),
        # Same caveat as Hamilton: dev box defaults to sim. Override
        # simulation:=false on a real-hardware box.
        launch_arguments={"simulation": "true"}.items(),
    )
    sim_liconic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("liconic_ros"),
            "/launch/liconic.launch.py",
        ]),
        launch_arguments={
            "simulation": "true",
            "namespace_prefix": SIM_PREFIX,
            # Aggressive compression so dry-runs return in seconds even when
            # the real-mode tree contains multi-hour incubation steps.
            "time_compression": "3600.0",
        }.items(),
    )

    # ── Imaging station sim — ImagePlate action for the campaign tree.
    # Stays as a sim until real imager hardware lands; the action interface
    # is the same so swapping in a real driver is a launch-file change.
    imaging_station_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("imaging_station"),
            "/launch/imaging_station_sim.launch.py",
        ]),
    )

    # ── Orchestrator ──────────────────────────────────────────────────────
    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    # bb_operator sidecar — operator-facing services for long-lived campaigns.
    bb_operator = Node(
        package="robot_skill_server",
        executable="bb_operator_node",
        name="bb_operator",
        output="screen",
    )

    # ── Dashboard ─────────────────────────────────────────────────────────
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("robot_dashboard"),
            "/launch/dashboard.launch.py",
        ]),
        launch_arguments={
            "rosbridge_port": rosbridge_port,
            "dashboard_port": dashboard_port,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("dashboard_port", default_value="8081"),
        DeclareLaunchArgument(
            "enable_real_meca500_moveit", default_value="true",
            description=(
                "Bring up MoveIt2 against mock_components for the real-mode "
                "Meca500 stack. Set to false if you're running an external "
                "MoveIt instance and only need the atoms here."
            ),
        ),
        real_meca500_moveit,
        real_meca500_atoms,
        sim_meca500_atoms,
        real_hamilton,
        sim_hamilton,
        real_liconic,
        sim_liconic,
        imaging_station_sim,
        skill_server,
        bb_operator,
        dashboard,
    ])
