/**
 * @brief Synchronous BT utility nodes for the Robot Skills Framework.
 *
 * These are lightweight, non-action-server BT nodes used as building blocks
 * in behavior trees. They run synchronously (no ROS2 action client) and
 * operate on the BT blackboard.
 *
 * Categories:
 *   - Safety: EmergencyStop, SetVelocityOverride
 *   - TF / Pose: LookupTransform, PublishStaticTF, GetCurrentPose, TransformPose,
 *                SetPose, ComputePreGraspPose
 *   - Grasp logic: CheckGraspSuccess
 *   - General: LogEvent, WaitForDuration
 */

#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_bt_nodes
{

// ── EmergencyStop: halt all motion immediately ───────────────────────────────
class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::cerr << "[EMERGENCY STOP] All motion halted" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// ── SetVelocityOverride: set a global velocity scaling factor ────────────────
class SetVelocityOverride : public BT::SyncActionNode
{
public:
  SetVelocityOverride(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("scaling", 1.0, "Velocity scaling 0.0-1.0"),
      BT::OutputPort<double>("velocity_override", "Current override value"),
    };
  }

  BT::NodeStatus tick() override
  {
    double scaling = getInput<double>("scaling").value_or(1.0);
    scaling = std::clamp(scaling, 0.0, 1.0);
    setOutput("velocity_override", scaling);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── LookupTransform: query TF between two frames ────────────────────────────
class LookupTransform : public BT::SyncActionNode
{
public:
  LookupTransform(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("source_frame", "world", "Source frame"),
      BT::InputPort<std::string>("target_frame", "panda_hand", "Target frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Transform as PoseStamped"),
    };
  }

  BT::NodeStatus tick() override
  {
    // Mock: return identity. Real: use tf2_ros::Buffer::lookupTransform
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getInput<std::string>("source_frame").value_or("world");
    pose.pose.orientation.w = 1.0;
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── PublishStaticTF: broadcast a static transform ────────────────────────────
class PublishStaticTF : public BT::SyncActionNode
{
public:
  PublishStaticTF(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("parent_frame", "world", "Parent frame"),
      BT::InputPort<std::string>("child_frame", "Child frame"),
      BT::InputPort<double>("x", 0.0, "Translation X"),
      BT::InputPort<double>("y", 0.0, "Translation Y"),
      BT::InputPort<double>("z", 0.0, "Translation Z"),
    };
  }

  BT::NodeStatus tick() override
  {
    // In a real system, this would publish to /tf_static via a
    // tf2_ros::StaticTransformBroadcaster. Mock: just succeeds.
    return BT::NodeStatus::SUCCESS;
  }
};

// ── GetCurrentPose: read current end-effector pose ───────────────────────────
class GetCurrentPose : public BT::SyncActionNode
{
public:
  GetCurrentPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("frame_id", "world", "Reference frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Current EEF pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    // Mock: return a plausible default pose. Real: FK from /joint_states.
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getInput<std::string>("frame_id").value_or("world");
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    pose.pose.orientation.w = 1.0;
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── LogEvent: publish a structured event for traceability ────────────────────
class LogEvent : public BT::SyncActionNode
{
public:
  LogEvent(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("event_name", "Event name"),
      BT::InputPort<std::string>("severity", "info", "info, warn, or error"),
      BT::InputPort<std::string>("message", "", "Human-readable description"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto event = getInput<std::string>("event_name").value_or("unknown");
    auto sev = getInput<std::string>("severity").value_or("info");
    auto msg = getInput<std::string>("message").value_or("");
    std::cout << "[EVENT:" << sev << "] " << event << " - " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// ── WaitForDuration: pause execution for a fixed time ────────────────────────
class WaitForDuration : public BT::SyncActionNode
{
public:
  WaitForDuration(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("seconds", 1.0, "Duration to wait (seconds)") };
  }

  BT::NodeStatus tick() override
  {
    double sec = getInput<double>("seconds").value_or(1.0);
    std::this_thread::sleep_for(std::chrono::duration<double>(sec));
    return BT::NodeStatus::SUCCESS;
  }
};

// ── TransformPose: convert a pose from one TF frame to another ───────────────
class TransformPose : public BT::SyncActionNode
{
public:
  TransformPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose", "Pose to transform"),
      BT::InputPort<std::string>("target_frame", "world", "Target frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Transformed pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    // In mock/sim mode, just copy the pose and update the frame_id
    auto input = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
    if (!input) {
      throw BT::RuntimeError("TransformPose: missing input_pose: ", input.error());
    }
    auto output = input.value();
    output.header.frame_id = getInput<std::string>("target_frame").value_or("world");
    setOutput("output_pose", output);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── CheckGraspSuccess: verify gripper actually grasped an object ─────────────
class CheckGraspSuccess : public BT::SyncActionNode
{
public:
  CheckGraspSuccess(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("object_grasped", "From GripperControl output"),
      BT::InputPort<double>("final_position", "Gripper position from GripperControl"),
      BT::InputPort<double>("min_grasp_width", 0.001, "Min width to consider grasp valid"),
    };
  }

  BT::NodeStatus tick() override
  {
    bool grasped = getInput<bool>("object_grasped").value_or(false);
    double min_width = getInput<double>("min_grasp_width").value_or(0.001);
    double pos = getInput<double>("final_position").value_or(0.0);

    if (grasped && pos > min_width) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

// ── SetPose: puts a fixed PoseStamped on the blackboard ──────────────────────
class SetPose : public BT::SyncActionNode
{
public:
  SetPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", 0.0, "Position X"),
      BT::InputPort<double>("y", 0.0, "Position Y"),
      BT::InputPort<double>("z", 0.0, "Position Z"),
      BT::InputPort<std::string>("frame_id", "world", "Frame ID"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getInput<std::string>("frame_id").value_or("world");
    pose.pose.position.x = getInput<double>("x").value_or(0.0);
    pose.pose.position.y = getInput<double>("y").value_or(0.0);
    pose.pose.position.z = getInput<double>("z").value_or(0.0);
    pose.pose.orientation.w = 1.0;
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

// ── ComputePreGraspPose: offset a pose along Z for approach ──────────────────
class ComputePreGraspPose : public BT::SyncActionNode
{
public:
  ComputePreGraspPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose"),
      BT::InputPort<double>("z_offset_m", 0.05, "Z offset above input pose (meters)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto input_pose = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
    if (!input_pose) {
      throw BT::RuntimeError("ComputePreGraspPose: missing input_pose: ",
                             input_pose.error());
    }

    double z_offset = 0.05;
    getInput("z_offset_m", z_offset);

    auto output = input_pose.value();
    output.pose.position.z += z_offset;

    setOutput("output_pose", output);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace robot_bt_nodes
