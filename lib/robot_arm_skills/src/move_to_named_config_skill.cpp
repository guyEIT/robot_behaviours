#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "robot_skills_msgs/action/move_to_named_config.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

// MoveIt2
#include "moveit/move_group_interface/move_group_interface.h"

namespace robot_arm_skills
{

class MoveToNamedConfigSkill
  : public SkillBase<robot_skills_msgs::action::MoveToNamedConfig>
{
public:
  using MoveToNamedConfig = robot_skills_msgs::action::MoveToNamedConfig;
  using Base = SkillBase<MoveToNamedConfig>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MoveToNamedConfigSkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<MoveToNamedConfig>(
      "move_to_named_config_skill",
      "/skill_atoms/move_to_named_config",
      options)
  {
    this->declare_parameter("default_planning_group", "arm");
    this->declare_parameter("default_velocity_scaling", 0.3);
    this->declare_parameter("allowed_named_configs",
      std::vector<std::string>{"home", "ready", "stow", "observe"});
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_to_named_config";
    desc.display_name = "Move to Named Configuration";
    desc.description =
      "Move the robot arm to a predefined joint configuration by name. "
      "Available configurations depend on the robot's SRDF definition. "
      "Typical configs: 'home' (retracted), 'ready' (work position), "
      "'observe' (camera pointing down), 'stow' (compact storage).";
    desc.version = "1.0.0";
    desc.category = "motion";
    desc.tags = {"motion", "joint_space", "named_config"};

    // Planning metadata
    desc.preconditions = {
      "robot_initialized",
      "no_estop_active",
      "planning_scene_valid"
    };
    desc.postconditions = {
      "robot_at_named_config"
    };
    desc.effects = {
      "(robot-at-config ?config_name)"
    };
    desc.constraints = {
      "config_name must be defined in robot SRDF",
      "velocity_scaling must be 0.0-1.0",
      "reduce velocity_scaling near humans or obstacles"
    };

    // JSON Schema for goal parameters
    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["config_name"],
      "properties": {
        "config_name": {
          "type": "string",
          "description": "Name of the joint configuration (e.g. home, ready, observe, stow)",
          "enum": ["home", "ready", "stow", "observe"]
        },
        "velocity_scaling": {
          "type": "number",
          "description": "Velocity scaling factor (0.0-1.0)",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.3
        },
        "acceleration_scaling": {
          "type": "number",
          "description": "Acceleration scaling factor (0.0-1.0)",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.3
        }
      }
    })json";

    // PDDL action fragment
    desc.pddl_action = R"(
(:action move_to_named_config
  :parameters (?robot - robot ?config - named_config)
  :precondition (and
    (robot-initialized ?robot)
    (not (estop-active))
  )
  :effect (and
    (robot-at-config ?robot ?config)
    (not (robot-at-config ?robot ?prev_config))
  )
))";

    desc.action_type = "robot_skills_msgs/action/MoveToNamedConfig";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    const auto allowed = this->get_parameter("allowed_named_configs")
      .as_string_array();
    const bool config_allowed = std::find(allowed.begin(), allowed.end(),
      goal->config_name) != allowed.end();

    if (!config_allowed) {
      return {false, "config_name '" + goal->config_name +
        "' not in allowed list: " + [&]() {
          std::string s;
          for (const auto & c : allowed) s += c + " ";
          return s;
        }()};
    }

    if (goal->velocity_scaling < 0.01 || goal->velocity_scaling > 1.0) {
      return {false, "velocity_scaling must be between 0.01 and 1.0"};
    }

    if (goal->acceleration_scaling > 0.0 &&
        (goal->acceleration_scaling < 0.01 || goal->acceleration_scaling > 1.0))
    {
      return {false, "acceleration_scaling must be between 0.01 and 1.0"};
    }

    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<MoveToNamedConfig::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(),
      "Executing MoveToNamedConfig: config='%s' velocity=%.2f",
      goal->config_name.c_str(), goal->velocity_scaling);

    const std::string planning_group = goal->planning_group.empty()
      ? this->get_parameter("default_planning_group").as_string()
      : goal->planning_group;

    const double velocity_scaling = goal->velocity_scaling > 0.0
      ? goal->velocity_scaling
      : this->get_parameter("default_velocity_scaling").as_double();

    // Publish initial feedback
    auto feedback = std::make_shared<MoveToNamedConfig::Feedback>();
    feedback->progress = 0.0;
    feedback->status_message = "Planning motion to '" + goal->config_name + "'";
    goal_handle->publish_feedback(feedback);

    try {
      // Initialize MoveGroupInterface
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), planning_group);

      move_group.setMaxVelocityScalingFactor(velocity_scaling);
      move_group.setMaxAccelerationScalingFactor(
        goal->acceleration_scaling > 0.0 ? goal->acceleration_scaling : velocity_scaling);

      // Set target configuration
      move_group.setNamedTarget(goal->config_name);

      // Check for cancellation before planning
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled before planning";
        return result;
      }

      feedback->progress = 0.1;
      feedback->status_message = "Planning...";
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Planning motion to '%s'...", goal->config_name.c_str());

      // Plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_result = move_group.plan(plan);

      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->message = "Planning failed with error: " +
          std::to_string(planning_result.val);
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      RCLCPP_DEBUG(this->get_logger(), "Plan found for '%s', executing...", goal->config_name.c_str());
      feedback->progress = 0.3;
      feedback->status_message = "Plan found, executing...";
      goal_handle->publish_feedback(feedback);

      // Check for cancellation before executing
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled after planning";
        return result;
      }

      const auto start_time = this->now();

      // Execute
      const auto exec_result = move_group.execute(plan);

      result->execution_time_sec = (this->now() - start_time).seconds();

      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->message = "Execution failed with error: " +
          std::to_string(exec_result.val);
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      feedback->progress = 1.0;
      feedback->status_message = "Reached '" + goal->config_name + "'";
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Successfully moved to '" + goal->config_name + "' in " +
        std::to_string(result->execution_time_sec) + "s";
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception during execution: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_arm_skills

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::MoveToNamedConfigSkill)
