"""One-shot launch for the Meca500 robot host: real MoveIt2 + skill atoms.

Brings up:
  - meca500_bringup/moveit.launch.py with simulation:=false (talks TCP to
    the physical Meca500 at robot_ip).
  - meca500_skill_server.launch.py — the arm skill atoms composed in one
    process under /meca500/skill_atoms/<name>, advertising the combined
    manifest at /meca500_skill_server/skills.

Pair with `pixi run orchestrator-run` on the orchestrator host (or the
same box) to drive behavior trees from the dashboard or via
`ros2 action send_goal /skill_server/execute_behavior_tree …`.

    pixi run meca500-real-up

Override the IP if your robot isn't on the default subnet:

    pixi run meca500-real-up robot_ip:=10.x.x.x
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    enable_realsense = LaunchConfiguration("enable_realsense")
    enable_ndi_bridge = LaunchConfiguration("enable_ndi_bridge")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("meca500_bringup"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "simulation": "false",
            "robot_ip": robot_ip,
            "enable_realsense": enable_realsense,
        }.items(),
    )

    # Atoms come up after MoveIt — they wait_for_server on /move_action at
    # startup. Delay slightly so the first call lands cleanly.
    atoms_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("meca500_skill_server"),
                    "/launch/meca500_skill_server.launch.py",
                ]),
                launch_arguments={"robot_id": "meca500"}.items(),
            ),
        ],
    )

    # ZowieBox NDI receiver. On by default — the bridge node fatals cleanly
    # if `pixi run setup-ndi` hasn't been run on this host (no libndi.so.6
    # / no ndi_av_bridge binary) without taking down the rest of the
    # launch, so leaving it on costs only a startup error log when the
    # imager is offline. Disable explicitly with enable_ndi_bridge:=false.
    # The static TF microscope_camera → imaging_station_camera_optical_frame
    # is published by imaging_station_scene.launch.py regardless of this
    # flag, so the camera position is always visible in RViz.
    ndi_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("imaging_station"),
            "/launch/imaging_station_ndi.launch.py",
        ]),
        condition=IfCondition(enable_ndi_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip", default_value="10.6.105.53",
            description="Meca500 IP address",
        ),
        DeclareLaunchArgument(
            "enable_realsense", default_value="",
            description="Override: launch Intel RealSense (true/false). "
                        "Empty uses meca500_bringup config file default.",
        ),
        DeclareLaunchArgument(
            "enable_ndi_bridge", default_value="true",
            description="Spawn the imaging_station NDI receiver to publish "
                        "the ZowieBox image on /imaging_station/image_raw. "
                        "Requires `pixi run -e meca500-host setup-ndi` once "
                        "per host; set false to skip when the imager / "
                        "NDI SDK isn't on this box.",
        ),
        moveit_launch,
        atoms_launch,
        ndi_bridge_launch,
    ])
