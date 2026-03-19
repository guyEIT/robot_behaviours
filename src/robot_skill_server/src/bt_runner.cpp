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

  // Register robot BT nodes directly with proper ROS node params
  factory.registerNodeType<robot_bt_nodes::MoveToNamedConfigNode>(
    "MoveToNamedConfig", ros_params);
  factory.registerNodeType<robot_bt_nodes::MoveToCartesianPoseNode>(
    "MoveToCartesianPose", ros_params);
  factory.registerNodeType<robot_bt_nodes::GripperControlNode>(
    "GripperControl", ros_params);
  factory.registerNodeType<robot_bt_nodes::DetectObjectNode>(
    "DetectObject", ros_params);

  // ComputePreGraspPose (synchronous - no action server needed)
  factory.registerNodeType<ComputePreGraspPose>("ComputePreGraspPose");

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

  // Spin ROS2 in background thread
  auto ros_thread = std::thread([&node]() {
    rclcpp::spin(node);
  });

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickWhileRunning(
      std::chrono::duration_cast<std::chrono::milliseconds>(tick_interval));

    // Compute progress from tree node statuses
    int total_actions = 0, completed = 0, failed = 0;
    std::string running_node_name;
    std::vector<std::string> completed_names, failed_names;

    for (const auto & subtree : tree.subtrees) {
      for (const auto & bt_node : subtree->nodes) {
        if (bt_node->type() == BT::NodeType::ACTION) {
          total_actions++;
          auto node_status = bt_node->status();
          if (node_status == BT::NodeStatus::SUCCESS) {
            completed++;
            completed_names.push_back(bt_node->name());
          } else if (node_status == BT::NodeStatus::FAILURE) {
            failed++;
            failed_names.push_back(bt_node->name());
          } else if (node_status == BT::NodeStatus::RUNNING) {
            running_node_name = bt_node->name();
          }
        }
      }
    }

    // Publish progress
    robot_skills_msgs::msg::TaskState state_msg;
    state_msg.task_id = cfg.task_id;
    state_msg.status = "RUNNING";
    state_msg.current_skill = running_node_name;
    state_msg.current_bt_node = running_node_name;
    state_msg.progress = total_actions > 0
      ? static_cast<double>(completed) / static_cast<double>(total_actions)
      : 0.0;
    state_msg.completed_skills = completed_names;
    state_msg.failed_skills = failed_names;
    state_msg.updated_at = node->now();
    status_pub->publish(state_msg);

    RCLCPP_DEBUG(node->get_logger(),
      "Tick: progress=%.0f%% running='%s' completed=%d/%d failed=%d",
      state_msg.progress * 100.0, running_node_name.c_str(),
      completed, total_actions, failed);

    std::this_thread::sleep_for(tick_interval);
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
