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

// Robot BT node class definitions
#include "robot_bt_nodes/bt_skill_nodes.hpp"
#include "robot_bt_nodes/bt_utility_nodes.hpp"
#include "robot_bt_nodes/bt_human_interaction_nodes.hpp"

#include "robot_skills_msgs/msg/task_state.hpp"
#include "robot_skills_msgs/msg/log_event.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

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
  factory.registerNodeType<robot_bt_nodes::CheckSystemReadyNode>(
    "CheckSystemReady", make_params("/skill_atoms/check_system_ready"));

  // Synchronous utility nodes (no action server needed)
  factory.registerNodeType<robot_bt_nodes::ComputePreGraspPose>("ComputePreGraspPose");
  factory.registerNodeType<robot_bt_nodes::SetPose>("SetPose");
  factory.registerNodeType<robot_bt_nodes::WaitForDuration>("WaitForDuration");
  factory.registerNodeType<robot_bt_nodes::TransformPose>("TransformPose");
  factory.registerNodeType<robot_bt_nodes::CheckGraspSuccess>("CheckGraspSuccess");
  factory.registerNodeType<robot_bt_nodes::EmergencyStop>("EmergencyStop");
  factory.registerNodeType<robot_bt_nodes::SetVelocityOverride>("SetVelocityOverride");
  factory.registerNodeType<robot_bt_nodes::LookupTransform>("LookupTransform");
  factory.registerNodeType<robot_bt_nodes::PublishStaticTF>("PublishStaticTF");
  factory.registerNodeType<robot_bt_nodes::GetCurrentPose>("GetCurrentPose");
  factory.registerNodeType<robot_bt_nodes::LogEvent>("LogEvent");

  // Human interaction nodes
  factory.registerNodeType<robot_bt_nodes::HumanNotification>("HumanNotification");
  factory.registerNodeType<robot_bt_nodes::HumanWarning>("HumanWarning");
  factory.registerNodeType<robot_bt_nodes::HumanConfirm>("HumanConfirm");
  factory.registerNodeType<robot_bt_nodes::HumanInput>("HumanInput");
  factory.registerNodeType<robot_bt_nodes::HumanTask>("HumanTask");

  // ── Pre-register shared behavior trees for <include> / <SubTree> ─────────
  // Load all .xml files from robot_behaviors/trees/ so that any tree can
  // reference shared subtrees (e.g. experiment_wrapper.xml) via <include>.
  try {
    auto behaviors_dir = ament_index_cpp::get_package_share_directory("robot_behaviors");
    auto trees_dir = std::filesystem::path(behaviors_dir) / "trees";
    if (std::filesystem::is_directory(trees_dir)) {
      for (const auto & entry : std::filesystem::directory_iterator(trees_dir)) {
        if (entry.path().extension() == ".xml") {
          try {
            factory.registerBehaviorTreeFromFile(entry.path().string());
            RCLCPP_DEBUG(node->get_logger(), "Pre-registered tree: %s",
              entry.path().filename().c_str());
          } catch (const std::exception & e) {
            // Some trees may define main_tree_to_execute and conflict; skip
            RCLCPP_DEBUG(node->get_logger(), "Skipped pre-registering %s: %s",
              entry.path().filename().c_str(), e.what());
          }
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(),
      "Could not pre-register behavior trees: %s", e.what());
  }

  // ── Load the tree XML ─────────────────────────────────────────────────────
  // Pre-registered subtrees (above) are available for <SubTree ID="..."/>
  // references. createTreeFromFile also resolves <include> relative to the
  // file's directory.
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(cfg.tree_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load BT from '%s': %s",
      cfg.tree_file.c_str(), e.what());
    rclcpp::shutdown();
    return 2;
  }

  // Store the ROS2 node on the blackboard for human interaction nodes
  tree.rootBlackboard()->set("ros_node", node);

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

  // ── Human-readable log publisher ──────────────────────────────────────────
  auto log_pub = node->create_publisher<robot_skills_msgs::msg::LogEvent>(
    "/skill_server/log_events", 10);

  // Convert snake_case BT node names to readable form: "go_to_observe" -> "Go to observe"
  auto humanise = [](const std::string & s) -> std::string {
    if (s.empty()) return s;
    std::string out = s;
    for (auto & c : out) { if (c == '_') c = ' '; }
    out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
  };

  auto publish_human_log = [&](const std::string & event,
                               const std::string & message,
                               const std::string & severity = "info",
                               const std::string & skill = "") {
    robot_skills_msgs::msg::LogEvent log_msg;
    log_msg.stamp = node->now();
    log_msg.event_name = event;
    log_msg.message = message;
    log_msg.severity = severity;
    log_msg.task_id = cfg.task_id;
    log_msg.skill_name = skill;
    log_msg.tags = {"human"};
    log_pub->publish(log_msg);
  };

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
  std::string prev_running_node;  // for transition detection
  size_t prev_completed_count = 0;
  size_t prev_failed_count = 0;

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

    // ── Human log: skill transitions (not every tick) ───────────────────
    if (!running_node_name.empty() && running_node_name != prev_running_node) {
      publish_human_log("skill_started",
        "Running: " + humanise(running_node_name), "debug", running_node_name);
      prev_running_node = running_node_name;
    }
    if (ever_completed.size() > prev_completed_count) {
      if (!prev_running_node.empty() && ever_completed.count(prev_running_node)) {
        publish_human_log("skill_completed",
          "Completed: " + humanise(prev_running_node), "info", prev_running_node);
      }
      prev_completed_count = ever_completed.size();
    }
    if (ever_failed.size() > prev_failed_count) {
      publish_human_log("skill_failed",
        "Failed: " + humanise(last_running_node), "error", last_running_node);
      prev_failed_count = ever_failed.size();
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

    // Populate error fields on failure
    if (status != BT::NodeStatus::SUCCESS && !ever_failed.empty()) {
      // The last node that was running when the tree failed is the cause
      final_msg.error_skill = last_running_node;
      final_msg.error_message = "'" + humanise(last_running_node) + "' failed";
      // If there are multiple failures, list them
      if (ever_failed.size() > 1) {
        std::string all_failed;
        for (const auto & f : ever_failed) {
          if (!all_failed.empty()) all_failed += ", ";
          all_failed += f;
        }
        final_msg.error_message += " (failed nodes: " + all_failed + ")";
      }
    }

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
