"""One-shot lab sim: all three providers + atoms + orchestrator + dashboard.

This is the "click any preset, hit Run" entry point. Brings up:
  - Meca500 MoveIt2 against mock_components/GenericSystem
  - Meca500 skill atoms under /meca500/skill_atoms/<name>
  - Hamilton STAR action server with STARChatterboxBackend
  - Liconic STX44 action server with LiconicSimBackend
  - Skill server orchestrator (BtExecutor, SkillRegistry, TaskComposer)
  - Web dashboard (rosbridge :9090, static UI :8081)

Run from the lab-sim pixi env:

    pixi run lab-sim-up

Then open http://localhost:8081 and pick any preset:
    test_meca500_sim, test_hamilton_sim, test_liconic_smoke

Notes:
  - Meca500 atoms wait until MoveIt2 has loaded (~4s) before launching;
    otherwise the first wait_for_server call times out before move_group
    finishes registering.
  - Only Meca500 atoms self-register with the SkillRegistry. Hamilton +
    Liconic expose their own actions directly under
    /hamilton_star_action_server/<name> and /liconic_action_server/<name>;
    BTs reach them via tree_executor's ACTION_REGISTRY defaults.
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


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    dashboard_port = LaunchConfiguration("dashboard_port")

    # ── Meca500 sim (MoveIt2 + mock_components/GenericSystem) ────────────
    meca500_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "true",
            "enable_realsense": "false",
        }.items(),
    )

    # ── Skill atoms at root namespace (/skill_atoms/<name>) ──────────────
    # Uses the `lab_sim` robot from robots.yaml, which is identical to the
    # meca500 entry except `namespace: ""` — so atoms advertise under bare
    # `/skill_atoms/<name>` paths. Every preset BT in the repo references
    # actions at that root path (without `server_name=` overrides), so this
    # is what makes "click any preset, hit Run" work end-to-end in lab-sim.
    # Real-hardware deployments use `robot_name:=meca500` instead, which
    # picks up the `/meca500` namespace.
    #
    # Wait so MoveIt's /move_action is up before atoms call wait_for_server.
    meca500_atoms = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("robot_skill_server"),
                    "/launch/skill_atoms_remote.launch.py",
                ]),
                launch_arguments={"robot_name": "lab_sim"}.items(),
            ),
        ],
    )

    # ── Hamilton STAR sim ─────────────────────────────────────────────────
    hamilton = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("hamilton_star_bringup"),
            "/launch/action_server.launch.py",
        ]),
        launch_arguments={"backend": "simulator"}.items(),
    )

    # ── Liconic STX44 sim ─────────────────────────────────────────────────
    liconic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("liconic_ros"),
            "/launch/liconic.launch.py",
        ]),
        launch_arguments={"simulation": "true"}.items(),
    )

    # ── Skill server orchestrator (BtExecutor + SkillRegistry) ───────────
    skill_server = Node(
        package="robot_skill_server",
        executable="skill_server_node",
        name="skill_server_node",
        output="screen",
    )

    # ── Web dashboard (rosbridge + static UI) ────────────────────────────
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
        meca500_moveit,
        meca500_atoms,
        hamilton,
        liconic,
        skill_server,
        dashboard,
    ])
