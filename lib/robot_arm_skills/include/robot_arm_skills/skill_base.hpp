#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "robot_skills_msgs/msg/skill_advertisement.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "robot_skill_advertise/skill_advertiser.hpp"

namespace robot_arm_skills
{

/**
 * @brief Abstract base class for all robot skill action servers.
 *
 * Every skill is a ROS2 Action Server that:
 * 1. Self-describes its capabilities (preconditions, postconditions, effects, constraints)
 * 2. Registers with the SkillRegistry on startup
 * 3. Checks preconditions before executing
 * 4. Publishes status feedback during execution
 *
 * Usage: Derive from SkillBase<ActionType> and implement:
 *   - getDescription()  -> skill metadata
 *   - checkPreconditions() -> safety checks before execution
 *   - executeGoal() -> the actual skill logic
 */
template <typename ActionT>
class SkillBase : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;

  ~SkillBase() override
  {
    // Join all tracked execution threads before destruction
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto & entry : execution_threads_) {
      if (entry.thread.joinable()) {
        entry.thread.join();
      }
    }
  }

  explicit SkillBase(
    const std::string & node_name,
    const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    action_name_(action_name)
  {
    // Multi-robot: optional namespace prefix for action server and registry identity.
    // robot_namespace: e.g. "/meca500" or "/franka" — prepended to all action server names.
    // robot_id:        e.g. "meca500" or "franka"   — used in SkillDescription.robot_id.
    //                  Derived from robot_namespace if left empty.
    this->declare_parameter("robot_namespace", "");
    this->declare_parameter("robot_id", "");

    // When the atom is composed inside a per-robot proxy (meca500_skill_server,
    // panda_skill_server, …), the proxy emits one combined manifest covering
    // all atoms it hosts. Setting publish_manifest:=false here suppresses the
    // per-atom manifest so SkillDiscovery doesn't see duplicate entries.
    this->declare_parameter("publish_manifest", true);

    const auto ns = this->get_parameter("robot_namespace").as_string();
    if (!ns.empty()) {
      // "/meca500" + "/skill_atoms/move_to_named_config" -> "/meca500/skill_atoms/..."
      action_name_ = ns + action_name_;
    }

    // Deferred initialization: shared_from_this() is not available in the
    // constructor, so schedule initialize() on the next executor spin.
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        if (!initialized_) {
          initialized_ = true;
          initialize();
        }
      });
  }

  /**
   * @brief Initialize the action server and register with SkillRegistry.
   * Call after construction (cannot be done in constructor due to shared_from_this).
   */
  void initialize()
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<ActionT>(
      this,
      action_name_,
      std::bind(&SkillBase::handleGoal, this, _1, _2),
      std::bind(&SkillBase::handleCancel, this, _1),
      std::bind(&SkillBase::handleAccepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Skill '%s' action server started on '%s'",
      this->get_name(), action_name_.c_str());

    // Setup diagnostics (publishes to /diagnostics at 1 Hz)
    diagnostic_updater_.setHardwareID(this->get_name());
    diagnostic_updater_.add("skill_status",
      [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        produceDiagnostics(stat);
      });

    // Publish a latched SkillManifest on `~/skills` for SkillDiscovery to
    // pick up. A per-robot proxy (e.g. meca500_skill_server) sets
    // publish_manifest:=false on each composed atom so it can emit a single
    // combined manifest instead. DDS QoS liveliness handles eviction when
    // an atom dies — no heartbeat needed.
    if (this->get_parameter("publish_manifest").as_bool()) {
      publishManifest();
    }
  }

  /**
   * @brief Return the skill's self-description metadata.
   * Override in derived classes to provide skill-specific metadata.
   */
  virtual robot_skills_msgs::msg::SkillDescription getDescription() = 0;

  /**
   * @brief Return the BT-binding additions on top of getDescription().
   * Override in derived classes when defaults / output renames / a
   * post-processor are required. Default is an empty advertisement;
   * SkillDiscovery will then PascalCase the description.name as bt_tag
   * and derive inputs from the action Goal field names.
   */
  virtual robot_skills_msgs::msg::SkillAdvertisement getAdvertisement()
  {
    return robot_skills_msgs::msg::SkillAdvertisement{};
  }

  /**
   * @brief Check if preconditions are met before executing.
   * Override to add skill-specific safety checks.
   * @return pair<bool, string>: {ok, reason_if_not_ok}
   */
  virtual std::pair<bool, std::string> checkPreconditions(
    [[maybe_unused]] const std::shared_ptr<const Goal> goal)
  {
    return {true, ""};
  }

  /**
   * @brief Execute the skill.
   * Override to implement skill logic. Should:
   * - Check goal_handle->is_canceling() periodically
   * - Call goal_handle->publish_feedback() during execution
   * - Return result on completion
   */
  virtual std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) = 0;

  /**
   * @brief Produce diagnostic status for this skill.
   * Override in derived classes for skill-specific diagnostics.
   */
  virtual void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (is_stub_) {
      stat.summary(DiagnosticStatus::WARN, "STUB - not fully implemented");
    } else if (failure_count_.load() > 0 && !last_execution_success_.load()) {
      stat.summary(DiagnosticStatus::WARN, "Last execution failed");
    } else {
      stat.summary(DiagnosticStatus::OK, "Operational");
    }
    stat.add("action_server", action_name_);
    stat.add("total_executions", std::to_string(execution_count_.load()));
    stat.add("successes", std::to_string(success_count_.load()));
    stat.add("failures", std::to_string(failure_count_.load()));
    stat.add("is_stub", is_stub_ ? "true" : "false");
    {
      std::lock_guard<std::mutex> lock(error_msg_mutex_);
      stat.add("last_error", last_error_message_);
    }
  }

protected:
  std::string action_name_;
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  diagnostic_updater::Updater diagnostic_updater_{this};
  bool is_stub_{false};

  // Execution tracking for diagnostics
  std::atomic<uint32_t> execution_count_{0};
  std::atomic<uint32_t> success_count_{0};
  std::atomic<uint32_t> failure_count_{0};
  std::atomic<bool> last_execution_success_{true};
  std::mutex error_msg_mutex_;
  std::string last_error_message_;

  /**
   * @brief Publish a latched SkillManifest on `~/skills` for SkillDiscovery.
   *
   * Combines getDescription() with getAdvertisement(), filling in
   * action_server_name and robot_id derived from the node's parameters.
   */
  void publishManifest()
  {
    robot_skills_msgs::msg::SkillAdvertisement ad = getAdvertisement();
    ad.description = getDescription();
    ad.description.action_server_name = action_name_;

    std::string rid = this->get_parameter("robot_id").as_string();
    if (rid.empty()) {
      const auto ns = this->get_parameter("robot_namespace").as_string();
      if (!ns.empty()) {
        rid = (ns[0] == '/') ? ns.substr(1) : ns;
      }
    }
    ad.description.robot_id = rid;

    advertiser_ = std::make_unique<robot_skill_advertise::SkillAdvertiser>(
      this, std::vector<robot_skills_msgs::msg::SkillAdvertisement>{ad});
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Goal> goal)
  {
    auto [ok, reason] = checkPreconditions(goal);
    if (!ok) {
      RCLCPP_WARN(this->get_logger(),
        "Skill '%s' rejected goal: precondition failed: %s",
        this->get_name(), reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Skill '%s' received cancel request",
      this->get_name());
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  struct TrackedThread {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> done;
  };

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Execute in a tracked thread so we can join on shutdown
    std::lock_guard<std::mutex> lock(threads_mutex_);

    // Clean up finished threads — join completed ones and remove them
    for (auto & entry : execution_threads_) {
      if (entry.done->load() && entry.thread.joinable()) {
        entry.thread.join();
      }
    }
    execution_threads_.erase(
      std::remove_if(execution_threads_.begin(), execution_threads_.end(),
        [](const TrackedThread & entry) { return !entry.thread.joinable(); }),
      execution_threads_.end());

    auto done = std::make_shared<std::atomic<bool>>(false);
    TrackedThread entry;
    entry.done = done;
    entry.thread = std::thread([this, goal_handle, done]() {
      RCLCPP_DEBUG(this->get_logger(), "Skill '%s' execution thread started",
        this->get_name());
      auto result = executeGoal(goal_handle);

      // Track execution stats for diagnostics
      execution_count_++;
      if (result && result->success) {
        success_count_++;
        last_execution_success_ = true;
        {
          std::lock_guard<std::mutex> lock(error_msg_mutex_);
          last_error_message_ = "";
        }
      } else {
        failure_count_++;
        last_execution_success_ = false;
        if (result) {
          std::lock_guard<std::mutex> lock(error_msg_mutex_);
          last_error_message_ = result->message;
        }
      }

      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Skill '%s' cancelled", this->get_name());
        goal_handle->canceled(result);
      } else {
        goal_handle->succeed(result);
      }

      done->store(true);
    });
    execution_threads_.push_back(std::move(entry));
  }

  std::mutex threads_mutex_;
  std::vector<TrackedThread> execution_threads_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<robot_skill_advertise::SkillAdvertiser> advertiser_;
  bool initialized_{false};
};

}  // namespace robot_arm_skills
