/**
 * @brief BehaviorTree.CPP v4 plugin registration for all robot BT nodes.
 *
 * This shared library is loaded at runtime by the BT executor.
 * Node class definitions are in robot_bt_nodes/bt_skill_nodes.hpp.
 *
 * All nodes are automatically visible in Groot2.
 */

#include "robot_bt_nodes/bt_skill_nodes.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration - BT_REGISTER_NODES is the BT.CPP entry point
// ─────────────────────────────────────────────────────────────────────────────

// This macro must be in global scope.
// BT.ROS2 RosActionNodes need RosNodeParams (containing the ROS2 node handle)
// at registration time. When loaded as a plugin, the executor must provide
// these params via the factory's shared blackboard.
BT_REGISTER_NODES(factory)
{
  BT::RosNodeParams default_params;
  factory.registerNodeType<robot_bt_nodes::MoveToNamedConfigNode>(
    "MoveToNamedConfig", default_params);
  factory.registerNodeType<robot_bt_nodes::MoveToCartesianPoseNode>(
    "MoveToCartesianPose", default_params);
  factory.registerNodeType<robot_bt_nodes::GripperControlNode>(
    "GripperControl", default_params);
  factory.registerNodeType<robot_bt_nodes::DetectObjectNode>(
    "DetectObject", default_params);
}
