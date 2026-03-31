/**
 * @brief C++ BehaviorTree.CPP v4 runner for the Robot Skills Framework.
 *
 * This executable is spawned by BtExecutor (Python) to actually run BT XML.
 * It:
 * 1. Loads the robot_bt_nodes plugin (C++ BT nodes)
 * 2. Parses the BT XML
 * 3. Registers all robot skill BT nodes with ROS2 action server connections
 * 4. Ticks the tree until SUCCESS/FAILURE/HALTED
 * 5. Optionally enables Groot2 ZMQ monitoring
 * 6. Publishes status to ROS2 topics for the Python wrapper to read
 * 7. Returns exit code: 0=SUCCESS, 1=FAILURE, 2=ERROR
 *
 * Usage:
 *   bt_runner --tree-file <path.xml> [--tick-rate <hz>] [--groot-port <port>]
 *             [--task-id <id>]
 */

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

// Robot BT node class definitions (shared with bt_nodes_plugin)
#include "robot_bt_nodes/bt_skill_nodes.hpp"

#include "robot_skills_msgs/msg/task_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ── EmergencyStop: halt all motion immediately ───────────────────────────────
class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // In a real system, this would call a service or publish to an e-stop topic.
    // In mock/sim, just log the event.
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
    // In a real system, this would publish to /skill_server/events.
    // Mock: just log to stdout.
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
    double pos = getInput<double>("final_position").value_or(0.0);
    double min_width = getInput<double>("min_grasp_width").value_or(0.001);

    if (grasped) {
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

// ── ComputePreGraspPose: synchronous BT node ─────────────────────────────────
// Takes an input pose and offsets it along Z to produce a pre-grasp approach pose.
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

namespace
{

struct RunnerConfig
{
  std::string tree_file;
  double tick_rate_hz = 10.0;
  int groot_port = 0;  // 0 = disabled
  std::string task_id;
};

RunnerConfig parseArgs(int argc, char ** argv)
{
  RunnerConfig cfg;
  for (int i = 1; i < argc - 1; ++i) {
    const std::string arg = argv[i];
    const std::string val = argv[i + 1];
    if (arg == "--tree-file") { cfg.tree_file = val; ++i; }
    else if (arg == "--tick-rate") { cfg.tick_rate_hz = std::stod(val); ++i; }
    else if (arg == "--groot-port") { cfg.groot_port = std::stoi(val); ++i; }
    else if (arg == "--task-id") { cfg.task_id = val; ++i; }
  }
  return cfg;
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open tree file: " + path);
  }
  std::ostringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto cfg = parseArgs(argc, argv);

  if (cfg.tree_file.empty()) {
    std::cerr << "Usage: bt_runner --tree-file <path.xml> "
              << "[--tick-rate <hz>] [--groot-port <port>] [--task-id <id>]\n";
    return 2;
  }

  auto node = std::make_shared<rclcpp::Node>(
    "bt_runner_" + (cfg.task_id.empty() ? "default" : cfg.task_id));

  RCLCPP_INFO(node->get_logger(), "bt_runner starting: %s", cfg.tree_file.c_str());

  // ── Build the BT factory and register all robot skill nodes ──────────────
  BT::BehaviorTreeFactory factory;

  // RosNodeParams - shared ROS2 node for all BT action clients
  BT::RosNodeParams ros_params;
  ros_params.nh = node;
  ros_params.server_timeout = std::chrono::seconds(30);
  ros_params.wait_for_server_timeout = std::chrono::seconds(10);

  // Register robot BT nodes with their action server names
  auto make_params = [&](const std::string& action_name) {
    auto p = ros_params;
    p.default_port_value = action_name;
    return p;
  };

  // Action-based skill nodes
  factory.registerNodeType<robot_bt_nodes::MoveToNamedConfigNode>(
    "MoveToNamedConfig", make_params("/skill_atoms/move_to_named_config"));
  factory.registerNodeType<robot_bt_nodes::MoveToCartesianPoseNode>(
    "MoveToCartesianPose", make_params("/skill_atoms/move_to_cartesian_pose"));
  factory.registerNodeType<robot_bt_nodes::MoveToJointConfigNode>(
    "MoveToJointConfig", make_params("/skill_atoms/move_to_joint_config"));
  factory.registerNodeType<robot_bt_nodes::MoveCartesianLinearNode>(
    "MoveCartesianLinear", make_params("/skill_atoms/move_cartesian_linear"));
  factory.registerNodeType<robot_bt_nodes::GripperControlNode>(
    "GripperControl", make_params("/skill_atoms/gripper_control"));
  factory.registerNodeType<robot_bt_nodes::DetectObjectNode>(
    "DetectObject", make_params("/skill_atoms/detect_object"));
  factory.registerNodeType<robot_bt_nodes::CapturePointCloudNode>(
    "CapturePointCloud", make_params("/skill_atoms/capture_point_cloud"));
  factory.registerNodeType<robot_bt_nodes::SetDigitalIONode>(
    "SetDigitalIO", make_params("/skill_atoms/set_digital_io"));
  factory.registerNodeType<robot_bt_nodes::CheckCollisionNode>(
    "CheckCollision", make_params("/skill_atoms/check_collision"));
  factory.registerNodeType<robot_bt_nodes::UpdatePlanningSceneNode>(
    "UpdatePlanningScene", make_params("/skill_atoms/update_planning_scene"));
  factory.registerNodeType<robot_bt_nodes::RobotEnableNode>(
    "RobotEnable", make_params("/skill_atoms/robot_enable"));
  factory.registerNodeType<robot_bt_nodes::RecordRosbagNode>(
    "RecordRosbag", make_params("/skill_atoms/record_rosbag"));

  // Synchronous utility nodes (no action server needed)
  factory.registerNodeType<ComputePreGraspPose>("ComputePreGraspPose");
  factory.registerNodeType<SetPose>("SetPose");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<TransformPose>("TransformPose");
  factory.registerNodeType<CheckGraspSuccess>("CheckGraspSuccess");
  factory.registerNodeType<EmergencyStop>("EmergencyStop");
  factory.registerNodeType<SetVelocityOverride>("SetVelocityOverride");
  factory.registerNodeType<LookupTransform>("LookupTransform");
  factory.registerNodeType<PublishStaticTF>("PublishStaticTF");
  factory.registerNodeType<GetCurrentPose>("GetCurrentPose");
  factory.registerNodeType<LogEvent>("LogEvent");

  // ── Load the tree XML ─────────────────────────────────────────────────────
  std::string tree_xml;
  try {
    tree_xml = readFile(cfg.tree_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to read tree file: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }

  BT::Tree tree;
  try {
    tree = factory.createTreeFromText(tree_xml);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse BT XML: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(node->get_logger(), "Tree loaded successfully");

  // ── Optional Groot2 monitoring ────────────────────────────────────────────
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  if (cfg.groot_port > 0) {
    try {
      groot_publisher = std::make_unique<BT::Groot2Publisher>(
        tree, static_cast<unsigned>(cfg.groot_port));
      RCLCPP_INFO(node->get_logger(),
        "Groot2 monitoring enabled on port %d", cfg.groot_port);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(),
        "Failed to start Groot2 publisher: %s", e.what());
    }
  }

  // ── Status publisher for progress tracking ───────────────────────────────
  auto status_pub = node->create_publisher<robot_skills_msgs::msg::TaskState>(
    "/skill_server/bt_runner_status/" + cfg.task_id, 10);

  // ── Spin the tree ─────────────────────────────────────────────────────────
  const auto tick_interval = std::chrono::duration<double>(1.0 / cfg.tick_rate_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  // Spin ROS2 in background thread for DDS communication.
  // BT::RosActionNode also has an internal executor, but the global spin
  // is needed for parameter events and DDS discovery.
  auto ros_thread = std::thread([&node]() {
    rclcpp::spin(node);
  });

  // Count total action nodes and track completed/failed across ticks
  int total_actions = 0;
  for (const auto & subtree : tree.subtrees) {
    for (const auto & bt_node : subtree->nodes) {
      if (bt_node->type() == BT::NodeType::ACTION) {
        total_actions++;
      }
    }
  }

  // Track which nodes have completed/failed (BT.CPP resets node status
  // after a sequence moves past them, so we must track cumulatively)
  std::set<std::string> ever_completed, ever_failed;
  std::string last_running_node;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickExactlyOnce();

    // Scan nodes for current state
    std::string running_node_name;
    for (const auto & subtree : tree.subtrees) {
      for (const auto & bt_node : subtree->nodes) {
        if (bt_node->type() == BT::NodeType::ACTION) {
          auto node_status = bt_node->status();
          if (node_status == BT::NodeStatus::SUCCESS) {
            ever_completed.insert(bt_node->name());
          } else if (node_status == BT::NodeStatus::FAILURE) {
            ever_failed.insert(bt_node->name());
          } else if (node_status == BT::NodeStatus::RUNNING) {
            running_node_name = bt_node->name();
          }
        }
      }
    }

    if (!running_node_name.empty()) {
      last_running_node = running_node_name;
    }

    // Publish progress
    robot_skills_msgs::msg::TaskState state_msg;
    state_msg.task_id = cfg.task_id;
    state_msg.status = "RUNNING";
    state_msg.current_skill = running_node_name.empty() ? last_running_node : running_node_name;
    state_msg.current_bt_node = state_msg.current_skill;
    state_msg.progress = total_actions > 0
      ? static_cast<double>(ever_completed.size()) / static_cast<double>(total_actions)
      : 0.0;
    state_msg.completed_skills.assign(ever_completed.begin(), ever_completed.end());
    state_msg.failed_skills.assign(ever_failed.begin(), ever_failed.end());
    state_msg.updated_at = node->now();
    status_pub->publish(state_msg);

    RCLCPP_DEBUG(node->get_logger(),
      "Tick: progress=%.0f%% running='%s' completed=%zu/%d failed=%zu",
      state_msg.progress * 100.0, state_msg.current_skill.c_str(),
      ever_completed.size(), total_actions, ever_failed.size());

    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(tick_interval);
    }
  }

  // Publish final status
  {
    robot_skills_msgs::msg::TaskState final_msg;
    final_msg.task_id = cfg.task_id;
    final_msg.status = (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
    final_msg.current_skill = last_running_node;
    final_msg.current_bt_node = last_running_node;
    final_msg.progress = (status == BT::NodeStatus::SUCCESS) ? 1.0 :
      (total_actions > 0 ? static_cast<double>(ever_completed.size()) / total_actions : 0.0);
    final_msg.completed_skills.assign(ever_completed.begin(), ever_completed.end());
    final_msg.failed_skills.assign(ever_failed.begin(), ever_failed.end());
    final_msg.updated_at = node->now();
    status_pub->publish(final_msg);
    // Small delay to ensure the message is sent before shutdown
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Halt the tree cleanly
  tree.haltTree();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  // Return exit code based on final status
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Tree completed: SUCCESS");
    return 0;
  } else {
    RCLCPP_WARN(node->get_logger(), "Tree completed: FAILURE");
    return 1;
  }
}
