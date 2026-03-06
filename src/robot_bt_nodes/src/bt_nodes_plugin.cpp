/**
 * @brief BehaviorTree.CPP v4 plugin registration for all robot BT nodes.
 *
 * This shared library is loaded at runtime by the BT executor.
 * Each node wraps a ROS2 action server via BT::RosActionNode<ActionT>.
 *
 * To add a new skill:
 * 1. Create a class inheriting BT::RosActionNode<YourActionType>
 * 2. Implement providedPorts(), setGoal(), onResultReceived()
 * 3. Register it below with BT::RegisterRosAction<>()
 *
 * All nodes are automatically visible in Groot2.
 */

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rclcpp/rclcpp.hpp"

// ROS2 action message types
#include "robot_skills_msgs/action/move_to_named_config.hpp"
#include "robot_skills_msgs/action/move_to_cartesian_pose.hpp"
#include "robot_skills_msgs/action/gripper_control.hpp"
#include "robot_skills_msgs/action/detect_object.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_bt_nodes
{

// ─────────────────────────────────────────────────────────────────────────────
// MoveToNamedConfig BT node
// ─────────────────────────────────────────────────────────────────────────────

class MoveToNamedConfigNode
  : public BT::RosActionNode<robot_skills_msgs::action::MoveToNamedConfig>
{
public:
  using MoveToNamedConfig = robot_skills_msgs::action::MoveToNamedConfig;

  MoveToNamedConfigNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<MoveToNamedConfig>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("config_name", "home",
        "Named joint configuration (e.g. home, ready, observe, stow)"),
      BT::InputPort<double>("velocity_scaling", 0.3,
        "Velocity scaling factor 0.01-1.0"),
      BT::InputPort<double>("acceleration_scaling", 0.3,
        "Acceleration scaling factor 0.01-1.0"),
      BT::InputPort<std::string>("planning_group", "arm",
        "MoveIt2 planning group name"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.config_name = getInput<std::string>("config_name").value();
    goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.3);
    goal.acceleration_scaling = getInput<double>("acceleration_scaling").value_or(0.3);
    goal.planning_group = getInput<std::string>("planning_group").value_or("arm");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      RCLCPP_INFO(logger(), "MoveToNamedConfig '%s' succeeded: %s",
        getInput<std::string>("config_name").value().c_str(),
        result.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger(), "MoveToNamedConfig failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "MoveToNamedConfig action error: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_DEBUG(logger(), "MoveToNamedConfig: %.0f%% - %s",
      feedback->progress * 100.0, feedback->status_message.c_str());
    return BT::NodeStatus::RUNNING;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// MoveToCartesianPose BT node
// ─────────────────────────────────────────────────────────────────────────────

class MoveToCartesianPoseNode
  : public BT::RosActionNode<robot_skills_msgs::action::MoveToCartesianPose>
{
public:
  using MoveToCartesianPose = robot_skills_msgs::action::MoveToCartesianPose;

  MoveToCartesianPoseNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<MoveToCartesianPose>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose",
        "Target end-effector pose (geometry_msgs/PoseStamped)"),
      BT::InputPort<double>("velocity_scaling", 0.3,
        "Velocity scaling factor 0.01-1.0"),
      BT::InputPort<double>("acceleration_scaling", 0.3,
        "Acceleration scaling factor 0.01-1.0"),
      BT::InputPort<bool>("plan_only", false,
        "If true, plan but do not execute"),
      BT::InputPort<std::string>("planning_group", "arm",
        "MoveIt2 planning group name"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose",
        "Actual pose reached after execution"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!pose) {
      RCLCPP_ERROR(logger(), "MoveToCartesianPose: missing required port 'target_pose'");
      return false;
    }
    goal.target_pose = pose.value();
    goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.3);
    goal.acceleration_scaling = getInput<double>("acceleration_scaling").value_or(0.3);
    goal.plan_only = getInput<bool>("plan_only").value_or(false);
    goal.planning_group = getInput<std::string>("planning_group").value_or("arm");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      setOutput("final_pose", result.result->final_pose);
      RCLCPP_INFO(logger(), "MoveToCartesianPose succeeded in %.2fs",
        result.result->execution_time_sec);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger(), "MoveToCartesianPose failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "MoveToCartesianPose action error: %d",
      static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_DEBUG(logger(), "MoveToCartesianPose: %.0f%% - %s",
      feedback->progress * 100.0, feedback->status_message.c_str());
    return BT::NodeStatus::RUNNING;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// GripperControl BT node
// ─────────────────────────────────────────────────────────────────────────────

class GripperControlNode
  : public BT::RosActionNode<robot_skills_msgs::action::GripperControl>
{
public:
  using GripperControl = robot_skills_msgs::action::GripperControl;

  GripperControlNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<GripperControl>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command", "open",
        "Gripper command: open, close, set_position"),
      BT::InputPort<double>("position", 1.0,
        "Target position (0.0=closed, 1.0=open), used with set_position"),
      BT::InputPort<double>("force_limit", 0.0,
        "Max grip force in Newtons (0 = default)"),
      BT::OutputPort<bool>("object_grasped",
        "True if object was detected during close (force stall)"),
      BT::OutputPort<double>("final_position",
        "Actual gripper position achieved"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.command = getInput<std::string>("command").value_or("open");
    goal.position = getInput<double>("position").value_or(1.0);
    goal.force_limit = getInput<double>("force_limit").value_or(0.0);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    setOutput("object_grasped", result.result->object_grasped);
    setOutput("final_position", result.result->final_position);

    if (result.result->success) {
      RCLCPP_INFO(logger(), "GripperControl '%s' succeeded (pos=%.3f, grasped=%s)",
        getInput<std::string>("command").value().c_str(),
        result.result->final_position,
        result.result->object_grasped ? "yes" : "no");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger(), "GripperControl failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GripperControl action error: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// DetectObject BT node
// ─────────────────────────────────────────────────────────────────────────────

class DetectObjectNode
  : public BT::RosActionNode<robot_skills_msgs::action::DetectObject>
{
public:
  using DetectObject = robot_skills_msgs::action::DetectObject;
  using DetectedObjectVec =
    std::vector<robot_skills_msgs::msg::DetectedObject>;

  DetectObjectNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<DetectObject>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_class", "",
        "Object class to detect (empty = any)"),
      BT::InputPort<double>("confidence_threshold", 0.7,
        "Minimum detection confidence 0.0-1.0"),
      BT::InputPort<int>("max_detections", 5,
        "Max number of objects to return"),
      BT::InputPort<double>("timeout_sec", 5.0,
        "Detection timeout in seconds"),
      BT::InputPort<bool>("require_pose", true,
        "If true, compute 3D pose for detections"),
      BT::OutputPort<DetectedObjectVec>("detected_objects",
        "Vector of detected objects with poses"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("best_object_pose",
        "Pose of the highest-confidence detection"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.object_class = getInput<std::string>("object_class").value_or("");
    goal.confidence_threshold = getInput<double>("confidence_threshold").value_or(0.7);
    goal.max_detections = getInput<int>("max_detections").value_or(5);
    goal.timeout_sec = getInput<double>("timeout_sec").value_or(5.0);
    goal.require_pose = getInput<bool>("require_pose").value_or(true);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success && !result.result->detections.empty()) {
      setOutput("detected_objects", result.result->detections);
      // Best detection = highest confidence
      const auto & best = *std::max_element(
        result.result->detections.begin(),
        result.result->detections.end(),
        [](const auto & a, const auto & b) {
          return a.confidence < b.confidence;
        });
      setOutput("best_object_pose", best.pose);

      RCLCPP_INFO(logger(), "DetectObject: found %zu object(s) (best: '%s' conf=%.2f)",
        result.result->detections.size(),
        best.class_name.c_str(),
        best.confidence);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(logger(), "DetectObject: no objects found - %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "DetectObject action error: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_DEBUG(logger(), "DetectObject: %d detections so far - %s",
      feedback->detections_so_far, feedback->status_message.c_str());
    return BT::NodeStatus::RUNNING;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration - BT_REGISTER_NODES is the BT.CPP entry point
// ─────────────────────────────────────────────────────────────────────────────

}  // namespace robot_bt_nodes

// This macro must be in global scope
BT_REGISTER_NODES(factory)
{
  // These nodes need the ROS2 node handle - stored in BT blackboard
  // The executor (robot_skill_server) provides the RosNodeParams
  // when loading this plugin.

  // Each registration specifies:
  //   - BT node type name (used in XML)
  //   - The C++ class
  //   - The ROS2 action server name
  //   - The RosNodeParams (provided by executor at load time)

  // NOTE: The actual registration with RosNodeParams happens in the executor.
  // This file provides the classes; the executor registers them.
  // See robot_skill_server/src/bt_executor.py for registration.

  (void)factory;  // Factory populated by executor, not here
}
