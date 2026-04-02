/**
 * @brief Human interaction BT nodes for the Robot Skills Framework.
 *
 * Non-blocking (async):
 *   - HumanNotification: fire-and-forget info message
 *   - HumanWarning: fire-and-forget with severity
 *
 * Blocking (sync):
 *   - HumanConfirm: blocks until operator confirms/rejects
 *   - HumanInput: blocks until operator provides a value
 *   - HumanTask: assigns a task, blocks until operator reports done/failed
 *
 * All nodes publish to /skill_server/human_prompts.
 * Blocking nodes subscribe to /skill_server/human_responses.
 * Nodes obtain the ROS2 node from the blackboard key "ros_node".
 */

#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_skills_msgs/msg/human_prompt.hpp"
#include "robot_skills_msgs/msg/human_response.hpp"
#include "robot_skills_msgs/msg/log_event.hpp"

namespace robot_bt_nodes
{

namespace detail
{

/// Generate a simple unique prompt ID (node_name + counter + random suffix)
inline std::string generatePromptId(const std::string & node_name)
{
  static std::atomic<uint64_t> counter{0};
  auto count = counter.fetch_add(1);
  std::random_device rd;
  std::ostringstream ss;
  ss << node_name << "_" << count << "_" << (rd() % 10000);
  return ss.str();
}

/// Lazy-init a publisher on the shared ROS2 node, cached in the member pointer.
inline rclcpp::Publisher<robot_skills_msgs::msg::HumanPrompt>::SharedPtr
getOrCreatePublisher(
  rclcpp::Publisher<robot_skills_msgs::msg::HumanPrompt>::SharedPtr & cached,
  const BT::NodeConfiguration & config)
{
  if (!cached) {
    auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    cached = ros_node->create_publisher<robot_skills_msgs::msg::HumanPrompt>(
      "/skill_server/human_prompts", 10);
  }
  return cached;
}

/// Publish a LogEvent with "human" tag for operator-facing log stream.
inline void publishHumanLog(
  rclcpp::Publisher<robot_skills_msgs::msg::LogEvent>::SharedPtr & cached,
  const BT::NodeConfiguration & config,
  const std::string & event_name,
  const std::string & message,
  const std::string & severity = "info",
  const std::string & skill_name = "")
{
  try {
    if (!cached) {
      auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      cached = ros_node->create_publisher<robot_skills_msgs::msg::LogEvent>(
        "/skill_server/log_events", 10);
    }
    robot_skills_msgs::msg::LogEvent msg;
    msg.stamp = rclcpp::Clock().now();
    msg.event_name = event_name;
    msg.severity = severity;
    msg.message = message;
    msg.skill_name = skill_name;
    msg.tags = {"human"};
    cached->publish(msg);
  } catch (...) {}
}

}  // namespace detail

// ── HumanNotification: fire-and-forget info message ─────────────────────────
class HumanNotification : public BT::SyncActionNode
{
public:
  HumanNotification(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("title", "Notification title"),
      BT::InputPort<std::string>("message", "", "Notification body"),
      BT::InputPort<std::string>("task_id", "", "Associated task ID"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto pub = detail::getOrCreatePublisher(pub_, config());
    robot_skills_msgs::msg::HumanPrompt msg;
    msg.stamp = rclcpp::Clock().now();
    msg.prompt_id = detail::generatePromptId(name());
    msg.prompt_type = "notification";
    msg.title = getInput<std::string>("title").value_or("Notification");
    msg.message = getInput<std::string>("message").value_or("");
    msg.severity = "info";
    msg.bt_node_name = name();
    msg.task_id = getInput<std::string>("task_id").value_or("");
    pub->publish(msg);
    detail::publishHumanLog(log_pub_, config(), "notification",
      msg.title + ": " + msg.message, "info", name());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<robot_skills_msgs::msg::HumanPrompt>::SharedPtr pub_;
  rclcpp::Publisher<robot_skills_msgs::msg::LogEvent>::SharedPtr log_pub_;
};

// ── HumanWarning: fire-and-forget with severity ─────────────────────────────
class HumanWarning : public BT::SyncActionNode
{
public:
  HumanWarning(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("title", "Warning title"),
      BT::InputPort<std::string>("message", "", "Warning body"),
      BT::InputPort<std::string>("severity", "warning", "info|warning|error|critical"),
      BT::InputPort<std::string>("task_id", "", "Associated task ID"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto pub = detail::getOrCreatePublisher(pub_, config());
    robot_skills_msgs::msg::HumanPrompt msg;
    msg.stamp = rclcpp::Clock().now();
    msg.prompt_id = detail::generatePromptId(name());
    msg.prompt_type = "warning";
    msg.title = getInput<std::string>("title").value_or("Warning");
    msg.message = getInput<std::string>("message").value_or("");
    msg.severity = getInput<std::string>("severity").value_or("warning");
    msg.bt_node_name = name();
    msg.task_id = getInput<std::string>("task_id").value_or("");
    pub->publish(msg);
    detail::publishHumanLog(log_pub_, config(), "warning",
      msg.title + ": " + msg.message, msg.severity, name());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<robot_skills_msgs::msg::HumanPrompt>::SharedPtr pub_;
  rclcpp::Publisher<robot_skills_msgs::msg::LogEvent>::SharedPtr log_pub_;
};

// ── Base for blocking human interaction nodes ────────────────────────────────
class HumanBlockingBase : public BT::StatefulActionNode
{
public:
  HumanBlockingBase(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

protected:
  /// Publish a prompt and subscribe for the response.
  void startPrompt(const robot_skills_msgs::msg::HumanPrompt & prompt)
  {
    prompt_id_ = prompt.prompt_id;
    start_time_ = std::chrono::steady_clock::now();
    timeout_sec_ = prompt.timeout_sec;
    response_received_.store(false);

    auto ros_node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");

    // Publisher (cached)
    if (!pub_) {
      pub_ = ros_node->create_publisher<robot_skills_msgs::msg::HumanPrompt>(
        "/skill_server/human_prompts", 10);
    }

    // Subscriber for responses
    sub_ = ros_node->create_subscription<robot_skills_msgs::msg::HumanResponse>(
      "/skill_server/human_responses", 10,
      [this](const robot_skills_msgs::msg::HumanResponse::SharedPtr msg) {
        if (msg->prompt_id == prompt_id_) {
          std::lock_guard<std::mutex> lock(mtx_);
          response_ = *msg;
          response_received_.store(true);
        }
      });

    pub_->publish(prompt);

    detail::publishHumanLog(log_pub_, config(),
      prompt.prompt_type + "_requested",
      "Awaiting human: " + prompt.title + " — " + prompt.message,
      "info", prompt.bt_node_name);
  }

  /// Check for response or timeout. Returns RUNNING, SUCCESS, or FAILURE.
  struct CheckResult {
    BT::NodeStatus status;
    robot_skills_msgs::msg::HumanResponse response;
  };

  CheckResult checkResponse()
  {
    // Check for response
    if (response_received_.load()) {
      std::lock_guard<std::mutex> lock(mtx_);
      auto resp = response_;
      cleanup();
      std::string outcome = resp.accepted ? "accepted" : "rejected";
      std::string log_msg = "Human " + outcome;
      if (!resp.value.empty()) log_msg += " (value: " + resp.value + ")";
      detail::publishHumanLog(log_pub_, config(), "human_response", log_msg,
        resp.accepted ? "info" : "warn", name());
      return {resp.accepted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE, resp};
    }

    // Check timeout
    if (timeout_sec_ > 0.0) {
      auto elapsed = std::chrono::steady_clock::now() - start_time_;
      double elapsed_sec = std::chrono::duration<double>(elapsed).count();
      if (elapsed_sec >= timeout_sec_) {
        publishDismiss();
        cleanup();
        detail::publishHumanLog(log_pub_, config(), "human_timeout",
          "No response — timed out after " + std::to_string(static_cast<int>(timeout_sec_)) + "s",
          "warn", name());
        return {BT::NodeStatus::FAILURE, robot_skills_msgs::msg::HumanResponse()};
      }
    }

    return {BT::NodeStatus::RUNNING, robot_skills_msgs::msg::HumanResponse()};
  }

  void cleanup()
  {
    if (sub_) {
      auto ros_node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      // Reset subscription (destroy it)
      sub_.reset();
    }
  }

  void publishDismiss()
  {
    if (pub_) {
      robot_skills_msgs::msg::HumanPrompt dismiss;
      dismiss.stamp = rclcpp::Clock().now();
      dismiss.prompt_id = prompt_id_;
      dismiss.prompt_type = "dismiss";
      dismiss.bt_node_name = name();
      pub_->publish(dismiss);
    }
  }

  std::string prompt_id_;
  double timeout_sec_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  std::atomic<bool> response_received_{false};
  std::mutex mtx_;
  robot_skills_msgs::msg::HumanResponse response_;
  rclcpp::Publisher<robot_skills_msgs::msg::HumanPrompt>::SharedPtr pub_;
  rclcpp::Publisher<robot_skills_msgs::msg::LogEvent>::SharedPtr log_pub_;
  rclcpp::Subscription<robot_skills_msgs::msg::HumanResponse>::SharedPtr sub_;
};

// ── HumanConfirm: blocks until operator confirms/rejects ────────────────────
class HumanConfirm : public HumanBlockingBase
{
public:
  HumanConfirm(const std::string & name, const BT::NodeConfiguration & config)
  : HumanBlockingBase(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("title", "Confirm title"),
      BT::InputPort<std::string>("message", "", "What to confirm"),
      BT::InputPort<double>("timeout_sec", 0.0, "Timeout in seconds (0=none)"),
      BT::InputPort<std::string>("task_id", "", "Associated task ID"),
      BT::OutputPort<bool>("confirmed", "True if operator confirmed"),
    };
  }

  BT::NodeStatus onStart() override
  {
    robot_skills_msgs::msg::HumanPrompt prompt;
    prompt.stamp = rclcpp::Clock().now();
    prompt.prompt_id = detail::generatePromptId(name());
    prompt.prompt_type = "confirm";
    prompt.title = getInput<std::string>("title").value_or("Confirm");
    prompt.message = getInput<std::string>("message").value_or("");
    prompt.severity = "info";
    prompt.timeout_sec = getInput<double>("timeout_sec").value_or(0.0);
    prompt.bt_node_name = name();
    prompt.task_id = getInput<std::string>("task_id").value_or("");
    startPrompt(prompt);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto result = checkResponse();
    if (result.status != BT::NodeStatus::RUNNING) {
      setOutput("confirmed", result.response.accepted);
    }
    return result.status;
  }

  void onHalted() override
  {
    publishDismiss();
    cleanup();
  }
};

// ── HumanInput: blocks until operator provides a value ──────────────────────
class HumanInput : public HumanBlockingBase
{
public:
  HumanInput(const std::string & name, const BT::NodeConfiguration & config)
  : HumanBlockingBase(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("title", "Input prompt title"),
      BT::InputPort<std::string>("message", "", "What to enter"),
      BT::InputPort<std::string>("input_type", "text", "text|number|choice"),
      BT::InputPort<std::string>("choices", "", "Semicolon-delimited options for choice type"),
      BT::InputPort<std::string>("default_value", "", "Default value"),
      BT::InputPort<double>("timeout_sec", 0.0, "Timeout in seconds (0=none)"),
      BT::InputPort<std::string>("task_id", "", "Associated task ID"),
      BT::OutputPort<std::string>("value", "The value entered by the operator"),
    };
  }

  BT::NodeStatus onStart() override
  {
    robot_skills_msgs::msg::HumanPrompt prompt;
    prompt.stamp = rclcpp::Clock().now();
    prompt.prompt_id = detail::generatePromptId(name());
    prompt.prompt_type = "input";
    prompt.title = getInput<std::string>("title").value_or("Input");
    prompt.message = getInput<std::string>("message").value_or("");
    prompt.severity = "info";
    prompt.input_type = getInput<std::string>("input_type").value_or("text");
    prompt.default_value = getInput<std::string>("default_value").value_or("");
    prompt.timeout_sec = getInput<double>("timeout_sec").value_or(0.0);
    prompt.bt_node_name = name();
    prompt.task_id = getInput<std::string>("task_id").value_or("");

    // Parse semicolon-delimited choices
    auto choices_str = getInput<std::string>("choices").value_or("");
    if (!choices_str.empty()) {
      std::istringstream iss(choices_str);
      std::string token;
      while (std::getline(iss, token, ';')) {
        if (!token.empty()) {
          prompt.choices.push_back(token);
        }
      }
    }

    startPrompt(prompt);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto result = checkResponse();
    if (result.status != BT::NodeStatus::RUNNING) {
      setOutput("value", result.response.value);
    }
    return result.status;
  }

  void onHalted() override
  {
    publishDismiss();
    cleanup();
  }
};

// ── HumanTask: assigns a physical task, blocks until done/failed ────────────
class HumanTask : public HumanBlockingBase
{
public:
  HumanTask(const std::string & name, const BT::NodeConfiguration & config)
  : HumanBlockingBase(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("title", "Task title"),
      BT::InputPort<std::string>("message", "", "Task instructions"),
      BT::InputPort<double>("timeout_sec", 0.0, "Timeout in seconds (0=none)"),
      BT::InputPort<std::string>("task_id", "", "Associated task ID"),
      BT::OutputPort<bool>("completed", "True if operator marked done"),
      BT::OutputPort<std::string>("notes", "Optional notes from operator"),
    };
  }

  BT::NodeStatus onStart() override
  {
    robot_skills_msgs::msg::HumanPrompt prompt;
    prompt.stamp = rclcpp::Clock().now();
    prompt.prompt_id = detail::generatePromptId(name());
    prompt.prompt_type = "task";
    prompt.title = getInput<std::string>("title").value_or("Task");
    prompt.message = getInput<std::string>("message").value_or("");
    prompt.severity = "info";
    prompt.timeout_sec = getInput<double>("timeout_sec").value_or(0.0);
    prompt.bt_node_name = name();
    prompt.task_id = getInput<std::string>("task_id").value_or("");
    startPrompt(prompt);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto result = checkResponse();
    if (result.status != BT::NodeStatus::RUNNING) {
      setOutput("completed", result.response.accepted);
      setOutput("notes", result.response.value);
    }
    return result.status;
  }

  void onHalted() override
  {
    publishDismiss();
    cleanup();
  }
};

}  // namespace robot_bt_nodes
