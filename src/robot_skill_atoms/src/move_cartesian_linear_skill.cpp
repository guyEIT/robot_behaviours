#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "robot_skills_msgs/action/move_cartesian_linear.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_skill_atoms
{

class MoveCartesianLinearSkill
  : public SkillBase<robot_skills_msgs::action::MoveCartesianLinear>
{
public:
  using MoveCartesianLinear = robot_skills_msgs::action::MoveCartesianLinear;
  using Base = SkillBase<MoveCartesianLinear>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MoveCartesianLinearSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<MoveCartesianLinear>(
      "move_cartesian_linear_skill",
      "/skill_atoms/move_cartesian_linear",
      options)
  {
    this->declare_parameter("default_planning_group", "arm");
    this->declare_parameter("default_velocity_scaling", 0.1);
    this->declare_parameter("default_step_size", 0.005);
    this->declare_parameter("default_max_deviation", 0.0);
    this->declare_parameter("min_fraction", 0.95);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_cartesian_linear";
    desc.display_name = "Move Cartesian Linear";
    desc.description =
      "Move the end-effector in a straight line to a target pose using "
      "MoveIt2 Cartesian path planning. Useful for approach/retreat motions.";
    desc.version = "1.0.0";
    desc.category = "motion";
    desc.tags = {"motion", "cartesian", "linear", "straight_line"};

    desc.preconditions = {
      "robot_initialized",
      "no_estop_active",
      "planning_scene_valid"
    };
    desc.postconditions = {"end_effector_at_target_pose"};

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["target_pose"],
      "properties": {
        "target_pose": {
          "type": "object",
          "description": "Target end-effector pose (geometry_msgs/PoseStamped)"
        },
        "velocity_scaling": {
          "type": "number",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.1
        },
        "step_size": {
          "type": "number",
          "description": "Cartesian step size in meters",
          "default": 0.005
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/MoveCartesianLinear";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    if (goal->velocity_scaling > 0.0 &&
        (goal->velocity_scaling < 0.01 || goal->velocity_scaling > 1.0))
    {
      return {false, "velocity_scaling must be between 0.01 and 1.0"};
    }
    if (goal->step_size > 0.0 && goal->step_size < 0.0001) {
      return {false, "step_size must be >= 0.0001 meters"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<MoveCartesianLinear::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string planning_group = goal->planning_group.empty()
      ? this->get_parameter("default_planning_group").as_string()
      : goal->planning_group;

    const double velocity_scaling = goal->velocity_scaling > 0.0
      ? goal->velocity_scaling
      : this->get_parameter("default_velocity_scaling").as_double();

    const double step_size = goal->step_size > 0.0
      ? goal->step_size
      : this->get_parameter("default_step_size").as_double();

    const double max_deviation = goal->max_deviation > 0.0
      ? goal->max_deviation
      : this->get_parameter("default_max_deviation").as_double();

    const double min_fraction = this->get_parameter("min_fraction").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Executing MoveCartesianLinear: [%.3f, %.3f, %.3f] vel=%.2f step=%.4f",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      velocity_scaling, step_size);

    auto feedback = std::make_shared<MoveCartesianLinear::Feedback>();
    feedback->progress = 0.0;
    feedback->status_message = "Computing Cartesian path...";
    goal_handle->publish_feedback(feedback);

    try {
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), planning_group);

      move_group.setMaxVelocityScalingFactor(velocity_scaling);
      move_group.setMaxAccelerationScalingFactor(velocity_scaling);

      // Build waypoint list (single target)
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(goal->target_pose.pose);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled before planning";
        return result;
      }

      // Compute Cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      const double fraction = move_group.computeCartesianPath(
        waypoints, step_size, max_deviation, trajectory_msg);

      result->fraction_achieved = fraction;

      RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.3f", fraction);

      if (fraction < min_fraction) {
        result->success = false;
        result->message = "Cartesian path only achieved " +
          std::to_string(fraction * 100.0) + "% (min " +
          std::to_string(min_fraction * 100.0) + "%)";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      feedback->progress = 0.3;
      feedback->status_message = "Path computed (" +
        std::to_string(static_cast<int>(fraction * 100.0)) + "%), executing...";
      goal_handle->publish_feedback(feedback);

      // Apply time parameterization for velocity scaling
      auto robot_model = move_group.getRobotModel();
      robot_trajectory::RobotTrajectory rt(robot_model, planning_group);
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      totg.computeTimeStamps(rt, velocity_scaling, velocity_scaling);
      rt.getRobotTrajectoryMsg(trajectory_msg);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled after planning";
        return result;
      }

      const auto start_time = this->now();
      const auto exec_result = move_group.execute(trajectory_msg);
      result->execution_time_sec = (this->now() - start_time).seconds();

      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->message = "Trajectory execution failed";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      result->final_pose = goal->target_pose;
      result->success = true;
      result->message = "Linear motion completed in " +
        std::to_string(result->execution_time_sec) + "s (fraction: " +
        std::to_string(fraction) + ")";

      feedback->progress = 1.0;
      feedback->status_message = "Reached target pose";
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::MoveCartesianLinearSkill)
