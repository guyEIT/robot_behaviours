#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"
#include "robot_skills_msgs/srv/register_skill.hpp"

namespace robot_skill_atoms
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

  explicit SkillBase(
    const std::string & node_name,
    const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    action_name_(action_name)
  {
    // Declare common parameters
    this->declare_parameter("skill_registry_service", "/skill_server/register_skill");
    this->declare_parameter("registration_timeout_sec", 5.0);
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

    registerWithRegistry();
  }

  /**
   * @brief Return the skill's self-description metadata.
   * Override in derived classes to provide skill-specific metadata.
   */
  virtual robot_skills_msgs::msg::SkillDescription getDescription() = 0;

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

protected:
  std::string action_name_;
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  /**
   * @brief Register this skill with the central SkillRegistry.
   * Called automatically during initialize().
   */
  void registerWithRegistry()
  {
    const std::string registry_service =
      this->get_parameter("skill_registry_service").as_string();

    auto client = this->create_client<robot_skills_msgs::srv::RegisterSkill>(registry_service);

    if (!client->wait_for_service(std::chrono::seconds(
          static_cast<int>(this->get_parameter("registration_timeout_sec").as_double()))))
    {
      RCLCPP_WARN(this->get_logger(),
        "SkillRegistry service '%s' not available. Skill '%s' will not be registered. "
        "Start robot_skill_server to enable skill discovery.",
        registry_service.c_str(), this->get_name());
      return;
    }

    auto request = std::make_shared<robot_skills_msgs::srv::RegisterSkill::Request>();
    request->description = getDescription();
    request->description.action_server_name = action_name_;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future,
          std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Skill '%s' registered successfully",
          this->get_name());
      } else {
        RCLCPP_WARN(this->get_logger(), "Skill '%s' registration failed: %s",
          this->get_name(), future.get()->message.c_str());
      }
    }
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

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Execute in a detached thread so action server is non-blocking
    std::thread([this, goal_handle]() {
      auto result = executeGoal(goal_handle);
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->succeed(result);
      }
    }).detach();
  }
};

}  // namespace robot_skill_atoms
