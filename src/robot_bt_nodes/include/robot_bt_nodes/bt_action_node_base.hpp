#pragma once

/**
 * @brief Convenience header: includes BehaviorTree.ROS2 action node base
 * and common utilities used by all robot BT nodes.
 *
 * BehaviorTree.ROS2's RosActionNode handles:
 * - Sending goals to ROS2 action servers
 * - Receiving feedback and updating BT tick
 * - Clean cancellation on BT halt
 * - Timeout handling
 */

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

// All robot BT nodes live in this namespace
namespace robot_bt_nodes
{

/**
 * @brief Common configuration shared by all robot BT action nodes.
 * Stored in the BT Blackboard under key "ros_node_config".
 */
inline BT::RosNodeParams makeNodeParams(
  rclcpp::Node::SharedPtr node,
  const std::string & server_timeout = "5",   // seconds as string for BT port
  const std::string & wait_for_server = "true")
{
  BT::RosNodeParams params;
  params.nh = node;
  params.server_timeout = std::chrono::milliseconds(
    static_cast<int>(std::stod(server_timeout) * 1000));
  params.wait_for_server_timeout = std::chrono::milliseconds(
    wait_for_server == "true" ? 5000 : 0);
  return params;
}

}  // namespace robot_bt_nodes
