/**
 * @brief BehaviorTree.CPP v4 plugin registration for all robot BT nodes.
 *
 * This shared library is loaded at runtime by TreeExecutionServer via the
 * `plugins` parameter. Uses BT_REGISTER_ROS_NODES so the executor provides
 * proper BT::RosNodeParams (node handle, timeouts) to action-based nodes.
 *
 * Node class definitions live in:
 *   - robot_bt_nodes/bt_skill_nodes.hpp          (ROS2 action-based skill nodes)
 *   - robot_bt_nodes/bt_utility_nodes.hpp         (synchronous utility nodes)
 *   - robot_bt_nodes/bt_human_interaction_nodes.hpp (human interaction nodes)
 *
 * Multi-robot support: nodes are registered once without robot-specific
 * suffixes. Use the `server_name` input port in BT XML to target a specific
 * robot's action server (e.g. server_name="/meca500/skill_atoms/...").
 */

#include "robot_bt_nodes/bt_skill_nodes.hpp"
#include "robot_bt_nodes/bt_utility_nodes.hpp"
#include "robot_bt_nodes/bt_human_interaction_nodes.hpp"

#include "behaviortree_ros2/plugins.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration — BT_REGISTER_ROS_NODES provides RosNodeParams
// ─────────────────────────────────────────────────────────────────────────────

BT_REGISTER_ROS_NODES(factory, params)
{
  // Helper: set default action server name for each node type.
  // Multi-robot trees override this via the server_name input port in XML.
  auto with_action = [&](const std::string& action_name) {
    auto p = params;
    p.default_port_value = "/skill_atoms" + action_name;
    return p;
  };

  // ── Action-based skill nodes ──────────────────────────────────────────────
  factory.registerNodeType<robot_bt_nodes::MoveToNamedConfigNode>(
    "MoveToNamedConfig", with_action("/move_to_named_config"));
  factory.registerNodeType<robot_bt_nodes::MoveToCartesianPoseNode>(
    "MoveToCartesianPose", with_action("/move_to_cartesian_pose"));
  factory.registerNodeType<robot_bt_nodes::MoveToJointConfigNode>(
    "MoveToJointConfig", with_action("/move_to_joint_config"));
  factory.registerNodeType<robot_bt_nodes::MoveCartesianLinearNode>(
    "MoveCartesianLinear", with_action("/move_cartesian_linear"));
  factory.registerNodeType<robot_bt_nodes::GripperControlNode>(
    "GripperControl", with_action("/gripper_control"));
  factory.registerNodeType<robot_bt_nodes::DetectObjectNode>(
    "DetectObject", with_action("/detect_object"));
  factory.registerNodeType<robot_bt_nodes::CapturePointCloudNode>(
    "CapturePointCloud", with_action("/capture_point_cloud"));
  factory.registerNodeType<robot_bt_nodes::SetDigitalIONode>(
    "SetDigitalIO", with_action("/set_digital_io"));
  factory.registerNodeType<robot_bt_nodes::CheckCollisionNode>(
    "CheckCollision", with_action("/check_collision"));
  factory.registerNodeType<robot_bt_nodes::UpdatePlanningSceneNode>(
    "UpdatePlanningScene", with_action("/update_planning_scene"));
  factory.registerNodeType<robot_bt_nodes::RobotEnableNode>(
    "RobotEnable", with_action("/robot_enable"));
  factory.registerNodeType<robot_bt_nodes::RecordRosbagNode>(
    "RecordRosbag", with_action("/record_rosbag"));
  factory.registerNodeType<robot_bt_nodes::CheckSystemReadyNode>(
    "CheckSystemReady", with_action("/check_system_ready"));

  // ── Synchronous utility nodes (no action server, no params needed) ────────
  factory.registerNodeType<robot_bt_nodes::EmergencyStop>("EmergencyStop");
  factory.registerNodeType<robot_bt_nodes::SetVelocityOverride>("SetVelocityOverride");
  factory.registerNodeType<robot_bt_nodes::LookupTransform>("LookupTransform");
  factory.registerNodeType<robot_bt_nodes::PublishStaticTF>("PublishStaticTF");
  factory.registerNodeType<robot_bt_nodes::GetCurrentPose>("GetCurrentPose");
  factory.registerNodeType<robot_bt_nodes::LogEvent>("LogEvent");
  factory.registerNodeType<robot_bt_nodes::WaitForDuration>("WaitForDuration");
  factory.registerNodeType<robot_bt_nodes::TransformPose>("TransformPose");
  factory.registerNodeType<robot_bt_nodes::CheckGraspSuccess>("CheckGraspSuccess");
  factory.registerNodeType<robot_bt_nodes::SetPose>("SetPose");
  factory.registerNodeType<robot_bt_nodes::ComputePreGraspPose>("ComputePreGraspPose");

  // ── Human interaction nodes ───────────────────────────────────────────────
  factory.registerNodeType<robot_bt_nodes::HumanNotification>("HumanNotification");
  factory.registerNodeType<robot_bt_nodes::HumanWarning>("HumanWarning");
  factory.registerNodeType<robot_bt_nodes::HumanConfirm>("HumanConfirm");
  factory.registerNodeType<robot_bt_nodes::HumanInput>("HumanInput");
  factory.registerNodeType<robot_bt_nodes::HumanTask>("HumanTask");
}
