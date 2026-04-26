#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "robot_skills_msgs/action/move_to_joint_config.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

namespace robot_arm_skills
{

class MoveToJointConfigSkill
  : public SkillBase<robot_skills_msgs::action::MoveToJointConfig>
{
public:
  using MoveToJointConfig = robot_skills_msgs::action::MoveToJointConfig;
  using Base = SkillBase<MoveToJointConfig>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MoveToJointConfigSkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<MoveToJointConfig>(
      "move_to_joint_config_skill",
      "/skill_atoms/move_to_joint_config",
      options)
  {
    this->declare_parameter("default_planning_group", "arm");
    this->declare_parameter("default_velocity_scaling", 0.3);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_to_joint_config";
    desc.display_name = "Move to Joint Configuration";
    desc.description =
      "Move the robot arm to arbitrary joint angles specified in radians. "
      "The number of joint values must match the planning group's DOF.";
    desc.version = "1.0.0";
    desc.category = "motion";
    desc.tags = {"motion", "joint_space", "joint_config"};

    desc.preconditions = {
      "robot_initialized",
      "no_estop_active",
      "planning_scene_valid"
    };
    desc.postconditions = {"robot_at_joint_config"};

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["joint_positions"],
      "properties": {
        "joint_positions": {
          "type": "array",
          "items": {"type": "number"},
          "description": "Target joint angles in radians"
        },
        "velocity_scaling": {
          "type": "number",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.3
        },
        "acceleration_scaling": {
          "type": "number",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.3
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/MoveToJointConfig";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    if (goal->joint_positions.empty()) {
      return {false, "joint_positions must not be empty"};
    }

    if (goal->velocity_scaling > 0.0 &&
        (goal->velocity_scaling < 0.01 || goal->velocity_scaling > 1.0))
    {
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
    auto result = std::make_shared<MoveToJointConfig::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string planning_group = goal->planning_group.empty()
      ? this->get_parameter("default_planning_group").as_string()
      : goal->planning_group;

    const double velocity_scaling = goal->velocity_scaling > 0.0
      ? goal->velocity_scaling
      : this->get_parameter("default_velocity_scaling").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Executing MoveToJointConfig: %zu joints, vel=%.2f",
      goal->joint_positions.size(), velocity_scaling);

    auto feedback = std::make_shared<MoveToJointConfig::Feedback>();
    feedback->progress = 0.0;
    feedback->status_message = "Planning joint motion...";
    goal_handle->publish_feedback(feedback);

    try {
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), planning_group);

      // Validate DOF
      const auto active_joints = move_group.getActiveJoints();
      if (goal->joint_positions.size() != active_joints.size()) {
        result->success = false;
        result->message = "Expected " + std::to_string(active_joints.size()) +
          " joint values, got " + std::to_string(goal->joint_positions.size());
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      move_group.setMaxVelocityScalingFactor(velocity_scaling);
      move_group.setMaxAccelerationScalingFactor(
        goal->acceleration_scaling > 0.0 ? goal->acceleration_scaling : velocity_scaling);

      move_group.setJointValueTarget(goal->joint_positions);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled before planning";
        return result;
      }

      feedback->progress = 0.1;
      feedback->status_message = "Planning...";
      goal_handle->publish_feedback(feedback);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_result = move_group.plan(plan);

      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->message = "Planning failed with error: " +
          std::to_string(planning_result.val);
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      feedback->progress = 0.3;
      feedback->status_message = "Plan found, executing...";
      goal_handle->publish_feedback(feedback);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled after planning";
        return result;
      }

      const auto start_time = this->now();
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
      feedback->status_message = "Reached joint configuration";
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Successfully moved to joint config in " +
        std::to_string(result->execution_time_sec) + "s";
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::MoveToJointConfigSkill)
