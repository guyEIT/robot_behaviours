/**
 * @brief BehaviorTree execution server for the Robot Skills Framework.
 *
 * Thin subclass of BT::TreeExecutionServer (from BehaviorTree.ROS2).
 * The base class provides: tick loop, tree.sleep(), Groot2 monitoring,
 * ExecuteTree action server, cancellation, tree halting, plugin loading,
 * tree XML loading from directories, and parameter-driven config reload.
 *
 * This subclass adds:
 *   - Dynamic tree registration from XML payload (dashboard sends raw XML)
 *   - ros_node blackboard entry for human interaction nodes
 *   - Progress tracking (completed/total action nodes)
 *   - Skill transition detection (started/completed/failed events in feedback)
 */

#include <optional>
#include <set>
#include <string>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotTreeServer : public BT::TreeExecutionServer
{
public:
  explicit RobotTreeServer(const rclcpp::NodeOptions& options)
    : TreeExecutionServer(options)
  {}

  explicit RobotTreeServer(const rclcpp::Node::SharedPtr& node)
    : TreeExecutionServer(node)
  {}

protected:
  bool onGoalReceived(const std::string& /*tree_name*/,
                      const std::string& payload) override
  {
    // Dashboard sends raw BT XML via the payload field.
    // Register it into the factory so createTree(tree_name) can find it.
    if (!payload.empty())
    {
      factory().registerBehaviorTreeFromText(payload);
    }
    // Reset progress tracking state
    completed_.clear();
    failed_.clear();
    current_node_.clear();
    event_.clear();
    total_actions_ = 0;
    prev_completed_ = 0;
    prev_failed_ = 0;
    return true;
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    // Human interaction nodes retrieve the ROS2 node from the blackboard
    tree.rootBlackboard()->set("ros_node", node());

    // Count action nodes for progress tracking
    for (const auto& subtree : tree.subtrees)
    {
      for (const auto& n : subtree->nodes)
      {
        if (n->type() == BT::NodeType::ACTION)
        {
          total_actions_++;
        }
      }
    }
  }

  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus /*status*/) override
  {
    // Scan tree for progress and detect skill transitions
    std::string running;
    for (const auto& subtree : tree().subtrees)
    {
      for (const auto& n : subtree->nodes)
      {
        if (n->type() != BT::NodeType::ACTION)
        {
          continue;
        }
        auto node_status = n->status();
        if (node_status == BT::NodeStatus::SUCCESS)
        {
          completed_.insert(n->name());
        }
        else if (node_status == BT::NodeStatus::FAILURE)
        {
          failed_.insert(n->name());
        }
        else if (node_status == BT::NodeStatus::RUNNING)
        {
          running = n->name();
        }
      }
    }

    // Detect transitions for skill logging
    event_.clear();
    if (!running.empty() && running != current_node_)
    {
      event_ = "started";
    }
    if (completed_.size() > prev_completed_)
    {
      event_ = "completed";
      prev_completed_ = completed_.size();
    }
    if (failed_.size() > prev_failed_)
    {
      event_ = "failed";
      prev_failed_ = failed_.size();
    }
    if (!running.empty())
    {
      current_node_ = running;
    }

    return std::nullopt;  // continue execution
  }

  std::optional<std::string> onLoopFeedback() override
  {
    // Format: "node_name|progress_fraction|event"
    // The Python bridge parses this, humanises the node name, and publishes
    // TaskState + LogEvent messages.
    double progress = total_actions_ > 0
      ? static_cast<double>(completed_.size()) / static_cast<double>(total_actions_)
      : 0.0;
    return current_node_ + "|" + std::to_string(progress) + "|" + event_;
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                      bool was_cancelled) override
  {
    if (was_cancelled)
    {
      return "Tree cancelled";
    }
    if (status == BT::NodeStatus::SUCCESS)
    {
      return "Tree completed successfully";
    }
    // Build failure message with the failed node name
    std::string msg = "Tree failed";
    if (!current_node_.empty())
    {
      msg += " at '" + current_node_ + "'";
    }
    if (!failed_.empty())
    {
      msg += " (failed nodes:";
      for (const auto& f : failed_)
      {
        msg += " " + f;
      }
      msg += ")";
    }
    return msg;
  }

private:
  std::set<std::string> completed_;
  std::set<std::string> failed_;
  std::string current_node_;
  std::string event_;  // "started", "completed", "failed", or ""
  int total_actions_ = 0;
  size_t prev_completed_ = 0;
  size_t prev_failed_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto server = std::make_shared<RobotTreeServer>(options);
  rclcpp::spin(server->node());
  rclcpp::shutdown();
  return 0;
}
