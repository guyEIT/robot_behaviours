"""FR3 + Franka Hand bringup, mirroring the Meca500 launch shape.

Args:
  simulation:=true|false       (default false) — translates to upstream's `use_fake_hardware`.
  robot_ip:=<host-or-IP>       (default 'dont-connect' in sim; required string in real mode).
  load_gripper:=true|false     (default true) — spawns franka_gripper_node on real hardware,
                                                 fake_gripper_state_publisher in sim.
  arm_id:=fr3                  (default fr3) — passed through as `robot_type` to franka_description.
  enable_rviz:=true|false      (default true).
  namespace:=''                 (default '').

In sim, the controllers YAML is overridden to use `command_interfaces: [position]` so
`mock_components/GenericSystem` actually moves joints under JointTrajectoryController
(the upstream effort-only YAML doesn't move with mock_components since there's no dynamics).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def _load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def _setup(context, *args, **kwargs):
    simulation = LaunchConfiguration("simulation").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    load_gripper = LaunchConfiguration("load_gripper").perform(context)
    arm_id = LaunchConfiguration("arm_id").perform(context)
    enable_rviz = LaunchConfiguration("enable_rviz").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)

    use_fake_hardware = "true" if simulation.lower() == "true" else "false"

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", arm_id, f"{arm_id}.urdf.xacro",
    )
    robot_description_config = Command([
        FindExecutable(name="xacro"), " ", franka_xacro_file,
        " hand:=", load_gripper,
        " robot_ip:=", robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " ros2_control:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_config, value_type=str)
    }

    franka_srdf_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", arm_id, f"{arm_id}.srdf.xacro",
    )
    robot_description_semantic_config = Command([
        FindExecutable(name="xacro"), " ", franka_srdf_file,
        " hand:=", load_gripper,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = _load_yaml("franka_fr3_moveit_config", "config/kinematics.yaml")
    kinematics_config = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = _load_yaml(
        "franka_fr3_moveit_config", "config/fr3_joint_limits.yaml"
    )
    joint_limits_config = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = _load_yaml(
        "franka_fr3_moveit_config", "config/ompl_planning.yaml"
    )
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = _load_yaml(
        "franka_fr3_moveit_config", "config/fr3_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Latched publishers so the skill component_container (which builds
        # MoveGroupInterface) can pick up robot_description + SRDF without
        # being passed them as parameters. Same flags meca500 uses via
        # MoveItConfigsBuilder.planning_scene_monitor().
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("franka_fr3_moveit_config"), "rviz", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
        condition=IfCondition(enable_rviz),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )

    if simulation.lower() == "true":
        ros2_controllers_path = os.path.join(
            get_package_share_directory("fr3_bringup"),
            "config", "fr3_ros_controllers_sim.yaml",
        )
        joint_states_remap = []
    else:
        ros2_controllers_path = os.path.join(
            get_package_share_directory("franka_fr3_moveit_config"),
            "config", "fr3_ros_controllers.yaml",
        )
        joint_states_remap = [("joint_states", "franka/joint_states")]

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=joint_states_remap,
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    spawner_args = [
        "--controller-manager-timeout", "60",
    ]
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["fr3_arm_controller", *spawner_args],
        output="screen",
    )
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", *spawner_args],
        output="screen",
    )
    spawn_franka_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["franka_robot_state_broadcaster", *spawner_args],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("simulation")),
    )

    # In sim mode mock_components publishes joint_states directly via the
    # joint_state_broadcaster, so we don't need a joint_state_publisher.
    # On real hardware franka_hardware remaps to /franka/joint_states and
    # franka_gripper publishes to /franka_gripper/joint_states; the merger
    # node combines them on /joint_states.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
        parameters=[{
            "source_list": ["franka/joint_states", "franka_gripper/joint_states"],
            "rate": 30,
        }],
        condition=UnlessCondition(LaunchConfiguration("simulation")),
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_gripper"), "launch", "gripper.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "robot_type": arm_id,
            "namespace": namespace,
        }.items(),
        condition=IfCondition(LaunchConfiguration("load_gripper")),
    )

    return [
        rviz_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        spawn_jsb,
        spawn_arm_controller,
        spawn_franka_state_broadcaster,
        joint_state_publisher,
        gripper_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "simulation", default_value="false",
            description="Use mock_components/GenericSystem instead of franka_hardware/FrankaHardwareInterface."),
        DeclareLaunchArgument(
            "robot_ip", default_value="dont-connect",
            description="Hostname or IP address of the FR3. Ignored in sim mode."),
        DeclareLaunchArgument(
            "load_gripper", default_value="true",
            description="Whether to spawn the Franka Hand gripper node."),
        DeclareLaunchArgument(
            "arm_id", default_value="fr3",
            description="Robot type / arm id passed to franka_description xacro (fr3 only for this provider)."),
        DeclareLaunchArgument(
            "enable_rviz", default_value="true",
            description="Launch RViz2 with the franka_fr3_moveit_config rviz file."),
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Namespace for the FR3 nodes. Empty by default."),
        OpaqueFunction(function=_setup),
    ])
