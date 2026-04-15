#pragma once

/**
 * @brief BehaviorTree.CPP v4 node definitions for all robot skills.
 *
 * Each node wraps a ROS2 action server via BT::RosActionNode<ActionT>.
 * Registered as a plugin via bt_nodes_plugin.cpp, loaded at runtime by
 * TreeExecutionServer.
 */

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_skills_msgs/action/move_to_named_config.hpp"
#include "robot_skills_msgs/action/move_to_cartesian_pose.hpp"
#include "robot_skills_msgs/action/move_to_joint_config.hpp"
#include "robot_skills_msgs/action/move_cartesian_linear.hpp"
#include "robot_skills_msgs/action/gripper_control.hpp"
#include "robot_skills_msgs/action/detect_object.hpp"
#include "robot_skills_msgs/action/capture_point_cloud.hpp"
#include "robot_skills_msgs/action/set_digital_io.hpp"
#include "robot_skills_msgs/action/check_collision.hpp"
#include "robot_skills_msgs/action/update_planning_scene.hpp"
#include "robot_skills_msgs/action/robot_enable.hpp"
#include "robot_skills_msgs/action/record_rosbag.hpp"
#include "robot_skills_msgs/action/check_system_ready.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

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
      BT::InputPort<std::string>("planning_group", "",
        "MoveIt2 planning group name (empty = use skill atom default)"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto config_name = getInput<std::string>("config_name");
    if (!config_name) {
      RCLCPP_ERROR(logger(), "MoveToNamedConfig: missing required port 'config_name'");
      return false;
    }
    goal.config_name = config_name.value();
    goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.3);
    goal.acceleration_scaling = getInput<double>("acceleration_scaling").value_or(0.3);
    goal.planning_group = getInput<std::string>("planning_group").value_or("");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      RCLCPP_INFO(logger(), "MoveToNamedConfig '%s' succeeded: %s",
        getInput<std::string>("config_name").value_or("unknown").c_str(),
        result.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(logger(), "MoveToNamedConfig failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "MoveToNamedConfig action error (connection/timeout): %d", static_cast<int>(error));
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
      BT::InputPort<std::string>("planning_group", "",
        "MoveIt2 planning group name (empty = use skill atom default)"),
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
    goal.planning_group = getInput<std::string>("planning_group").value_or("");
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
    RCLCPP_WARN(logger(), "MoveToCartesianPose failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "MoveToCartesianPose action error (connection/timeout): %d",
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
        getInput<std::string>("command").value_or("unknown").c_str(),
        result.result->final_position,
        result.result->object_grasped ? "yes" : "no");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(logger(), "GripperControl failed: %s",
      result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GripperControl action error (connection/timeout): %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_DEBUG(logger(), "GripperControl: pos=%.3f force=%.2f",
      feedback->current_position, feedback->current_force);
    return BT::NodeStatus::RUNNING;
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
    RCLCPP_ERROR(logger(), "DetectObject action error (connection/timeout): %d", static_cast<int>(error));
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
// MoveToJointConfig BT node
// ─────────────────────────────────────────────────────────────────────────────

class MoveToJointConfigNode
  : public BT::RosActionNode<robot_skills_msgs::action::MoveToJointConfig>
{
public:
  using MoveToJointConfig = robot_skills_msgs::action::MoveToJointConfig;

  MoveToJointConfigNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<MoveToJointConfig>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("joint_positions", "Target joint angles (radians)"),
      BT::InputPort<double>("velocity_scaling", 0.3, "Velocity scaling 0.01-1.0"),
      BT::InputPort<double>("acceleration_scaling", 0.3, "Acceleration scaling 0.01-1.0"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto joints = getInput<std::vector<double>>("joint_positions");
    if (!joints) { RCLCPP_ERROR(logger(), "MoveToJointConfig: missing joint_positions"); return false; }
    goal.joint_positions = joints.value();
    goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.3);
    goal.acceleration_scaling = getInput<double>("acceleration_scaling").value_or(0.3);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  { return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// MoveCartesianLinear BT node
// ─────────────────────────────────────────────────────────────────────────────

class MoveCartesianLinearNode
  : public BT::RosActionNode<robot_skills_msgs::action::MoveCartesianLinear>
{
public:
  using MoveCartesianLinear = robot_skills_msgs::action::MoveCartesianLinear;

  MoveCartesianLinearNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<MoveCartesianLinear>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target end-effector pose"),
      BT::InputPort<double>("velocity_scaling", 0.1, "Velocity scaling 0.01-1.0"),
      BT::InputPort<double>("step_size", 0.005, "Cartesian step size (meters)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose", "Actual pose reached"),
      BT::OutputPort<double>("fraction_achieved", "Fraction of path completed 0.0-1.0"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!pose) { RCLCPP_ERROR(logger(), "MoveCartesianLinear: missing target_pose"); return false; }
    goal.target_pose = pose.value();
    goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.1);
    goal.step_size = getInput<double>("step_size").value_or(0.005);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      setOutput("final_pose", result.result->final_pose);
      setOutput("fraction_achieved", result.result->fraction_achieved);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// CapturePointCloud BT node
// ─────────────────────────────────────────────────────────────────────────────

class CapturePointCloudNode
  : public BT::RosActionNode<robot_skills_msgs::action::CapturePointCloud>
{
public:
  using CapturePointCloud = robot_skills_msgs::action::CapturePointCloud;

  CapturePointCloudNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<CapturePointCloud>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_sec", 5.0, "Max wait for capture"),
      BT::InputPort<bool>("apply_filters", true, "Apply voxel/passthrough filters"),
      BT::OutputPort<int>("num_points", "Number of points captured"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.timeout_sec = getInput<double>("timeout_sec").value_or(5.0);
    goal.apply_filters = getInput<bool>("apply_filters").value_or(true);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      setOutput("num_points", result.result->num_points);
      RCLCPP_INFO(logger(), "CapturePointCloud: %d points", result.result->num_points);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// SetDigitalIO BT node
// ─────────────────────────────────────────────────────────────────────────────

class SetDigitalIONode
  : public BT::RosActionNode<robot_skills_msgs::action::SetDigitalIO>
{
public:
  using SetDigitalIO = robot_skills_msgs::action::SetDigitalIO;

  SetDigitalIONode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<SetDigitalIO>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pin_name", "Logical pin name"),
      BT::InputPort<bool>("value", true, "True=HIGH, False=LOW"),
      BT::InputPort<bool>("read_only", false, "If true, just read the pin"),
      BT::OutputPort<bool>("current_value", "Pin state after operation"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto pin = getInput<std::string>("pin_name");
    if (!pin) { RCLCPP_ERROR(logger(), "SetDigitalIO: missing pin_name"); return false; }
    goal.pin_name = pin.value();
    goal.value = getInput<bool>("value").value_or(true);
    goal.read_only = getInput<bool>("read_only").value_or(false);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    setOutput("current_value", result.result->current_value);
    return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// CheckCollision BT node
// ─────────────────────────────────────────────────────────────────────────────

class CheckCollisionNode
  : public BT::RosActionNode<robot_skills_msgs::action::CheckCollision>
{
public:
  using CheckCollision = robot_skills_msgs::action::CheckCollision;

  CheckCollisionNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<CheckCollision>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("joint_positions", "Joints to check (empty = current)"),
      BT::OutputPort<bool>("in_collision", "True if in collision"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto joints = getInput<std::vector<double>>("joint_positions");
    if (joints) goal.joint_positions = joints.value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    setOutput("in_collision", result.result->in_collision);
    // SUCCESS means the check completed (not that it's collision-free)
    // Use the output port to branch in the BT
    return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// UpdatePlanningScene BT node
// ─────────────────────────────────────────────────────────────────────────────

class UpdatePlanningSceneNode
  : public BT::RosActionNode<robot_skills_msgs::action::UpdatePlanningScene>
{
public:
  using UpdatePlanningScene = robot_skills_msgs::action::UpdatePlanningScene;

  UpdatePlanningSceneNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<UpdatePlanningScene>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_id", "Unique ID for the collision object"),
      BT::InputPort<std::string>("operation", "add", "add, remove, or clear_all"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Object pose (for add)"),
      BT::InputPort<std::string>("shape_type", "box", "box, cylinder, or sphere"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto id = getInput<std::string>("object_id");
    if (!id) { RCLCPP_ERROR(logger(), "UpdatePlanningScene: missing object_id"); return false; }
    goal.object_id = id.value();
    goal.operation = getInput<std::string>("operation").value_or("add");
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    if (pose) goal.pose = pose.value();
    goal.shape_type = getInput<std::string>("shape_type").value_or("box");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  { return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// RobotEnable BT node
// ─────────────────────────────────────────────────────────────────────────────

class RobotEnableNode
  : public BT::RosActionNode<robot_skills_msgs::action::RobotEnable>
{
public:
  using RobotEnable = robot_skills_msgs::action::RobotEnable;

  RobotEnableNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<RobotEnable>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", true, "true=enable (servo on), false=disable"),
      BT::OutputPort<bool>("is_enabled", "Robot state after operation"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.enable = getInput<bool>("enable").value_or(true);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    setOutput("is_enabled", result.result->is_enabled);
    return result.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// RecordRosbag BT node
// ─────────────────────────────────────────────────────────────────────────────

class RecordRosbagNode
  : public BT::RosActionNode<robot_skills_msgs::action::RecordRosbag>
{
public:
  using RecordRosbag = robot_skills_msgs::action::RecordRosbag;

  RecordRosbagNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<RecordRosbag>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("output_path", "/tmp/recording.bag", "Bag file path"),
      BT::InputPort<double>("duration_sec", 5.0, "Recording duration (0=until cancelled)"),
      BT::OutputPort<std::string>("bag_path", "Actual recorded bag path"),
      BT::OutputPort<int>("num_messages", "Total messages recorded"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    goal.output_path = getInput<std::string>("output_path").value_or("/tmp/recording.bag");
    goal.duration_sec = getInput<double>("duration_sec").value_or(5.0);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.result->success) {
      setOutput("bag_path", result.result->bag_path);
      setOutput("num_messages", static_cast<int>(result.result->num_messages));
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode) override { return BT::NodeStatus::FAILURE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// CheckSystemReady BT node
// ─────────────────────────────────────────────────────────────────────────────

class CheckSystemReadyNode
  : public BT::RosActionNode<robot_skills_msgs::action::CheckSystemReady>
{
public:
  using CheckSystemReady = robot_skills_msgs::action::CheckSystemReady;

  CheckSystemReadyNode(const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<CheckSystemReady>(name, config, params) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("required_systems",
        "Systems to check (empty = all defaults)"),
      BT::InputPort<double>("timeout_sec", 5.0,
        "Max wait per system in seconds"),
      BT::OutputPort<std::vector<std::string>>("available_systems",
        "Systems that responded"),
      BT::OutputPort<std::vector<std::string>>("unavailable_systems",
        "Systems that did not respond"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto systems = getInput<std::vector<std::string>>("required_systems");
    if (systems) goal.required_systems = systems.value();
    goal.timeout_sec = getInput<double>("timeout_sec").value_or(5.0);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    setOutput("available_systems", result.result->available_systems);
    setOutput("unavailable_systems", result.result->unavailable_systems);

    if (result.result->success) {
      RCLCPP_INFO(logger(), "CheckSystemReady: %s", result.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(logger(), "CheckSystemReady: %s", result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "CheckSystemReady action error (connection/timeout): %d",
      static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_DEBUG(logger(), "CheckSystemReady: %.0f%% - checking %s",
      feedback->progress * 100.0, feedback->current_check.c_str());
    return BT::NodeStatus::RUNNING;
  }
};

}  // namespace robot_bt_nodes
