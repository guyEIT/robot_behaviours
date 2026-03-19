#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "robot_skills_msgs/action/move_to_cartesian_pose.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_skill_atoms
{

class MoveToCartesianPoseSkill
  : public SkillBase<robot_skills_msgs::action::MoveToCartesianPose>
{
public:
  using MoveToCartesianPose = robot_skills_msgs::action::MoveToCartesianPose;
  using Base = SkillBase<MoveToCartesianPose>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MoveToCartesianPoseSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<MoveToCartesianPose>(
      "move_to_cartesian_pose_skill",
      "/skill_atoms/move_to_cartesian_pose",
      options)
  {
    this->declare_parameter("default_planning_group", "arm");
    this->declare_parameter("default_velocity_scaling", 0.3);
    this->declare_parameter("position_tolerance_m", 0.005);
    this->declare_parameter("orientation_tolerance_rad", 0.01);
    this->declare_parameter("workspace_min_x", -1.0);
    this->declare_parameter("workspace_max_x", 1.0);
    this->declare_parameter("workspace_min_y", -1.0);
    this->declare_parameter("workspace_max_y", 1.0);
    this->declare_parameter("workspace_min_z", 0.0);
    this->declare_parameter("workspace_max_z", 1.5);

  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_to_cartesian_pose";
    desc.display_name = "Move to Cartesian Pose";
    desc.description =
      "Move the robot end-effector to a specified 6-DOF Cartesian pose in 3D space. "
      "The pose can be specified in any TF frame. Uses MoveIt2 motion planning. "
      "Suitable for pre-grasp positioning, placing, or any pose-based motion.";
    desc.version = "1.0.0";
    desc.category = "motion";
    desc.tags = {"motion", "cartesian", "pose", "end_effector"};

    desc.preconditions = {
      "robot_initialized",
      "no_estop_active",
      "planning_scene_valid",
      "target_pose_in_workspace"
    };
    desc.postconditions = {
      "end_effector_at_target_pose"
    };
    desc.effects = {
      "(end-effector-at ?pose)"
    };
    desc.constraints = {
      "target_pose must be within robot workspace bounds",
      "velocity_scaling must be 0.0-1.0",
      "pose must be reachable (IK solution exists)"
    };

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["target_pose"],
      "properties": {
        "target_pose": {
          "type": "object",
          "description": "Target end-effector pose (geometry_msgs/PoseStamped)",
          "properties": {
            "header": {"type": "object"},
            "pose": {
              "type": "object",
              "properties": {
                "position": {"type": "object", "properties": {"x": {}, "y": {}, "z": {}}},
                "orientation": {"type": "object", "properties": {"x": {}, "y": {}, "z": {}, "w": {}}}
              }
            }
          }
        },
        "velocity_scaling": {
          "type": "number",
          "minimum": 0.01,
          "maximum": 1.0,
          "default": 0.3
        },
        "plan_only": {
          "type": "boolean",
          "description": "If true, plan but do not execute",
          "default": false
        }
      }
    })json";

    desc.pddl_action = R"(
(:action move_to_cartesian_pose
  :parameters (?robot - robot ?pose - pose)
  :precondition (and
    (robot-initialized ?robot)
    (not (estop-active))
    (pose-in-workspace ?pose)
  )
  :effect (and
    (end-effector-at ?robot ?pose)
  )
))";

    desc.action_type = "robot_skills_msgs/action/MoveToCartesianPose";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    const auto & p = goal->target_pose.pose.position;

    // Basic workspace bounds check
    if (p.x < this->get_parameter("workspace_min_x").as_double() ||
        p.x > this->get_parameter("workspace_max_x").as_double() ||
        p.y < this->get_parameter("workspace_min_y").as_double() ||
        p.y > this->get_parameter("workspace_max_y").as_double() ||
        p.z < this->get_parameter("workspace_min_z").as_double() ||
        p.z > this->get_parameter("workspace_max_z").as_double())
    {
      return {false, "Target pose is outside workspace bounds"};
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
    auto result = std::make_shared<MoveToCartesianPose::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string planning_group = goal->planning_group.empty()
      ? this->get_parameter("default_planning_group").as_string()
      : goal->planning_group;

    const double velocity_scaling = goal->velocity_scaling > 0.0
      ? goal->velocity_scaling
      : this->get_parameter("default_velocity_scaling").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Executing MoveToCartesianPose: [%.3f, %.3f, %.3f] vel=%.2f",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      velocity_scaling);

    auto feedback = std::make_shared<MoveToCartesianPose::Feedback>();
    feedback->progress = 0.0;
    feedback->status_message = "Planning Cartesian motion...";
    goal_handle->publish_feedback(feedback);

    try {
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), planning_group);

      move_group.setMaxVelocityScalingFactor(velocity_scaling);
      move_group.setMaxAccelerationScalingFactor(
        goal->acceleration_scaling > 0.0 ? goal->acceleration_scaling : velocity_scaling);

      move_group.setGoalPositionTolerance(
        this->get_parameter("position_tolerance_m").as_double());
      move_group.setGoalOrientationTolerance(
        this->get_parameter("orientation_tolerance_rad").as_double());

      // Handle planning time
      if (goal->planning_time > 0.0) {
        move_group.setPlanningTime(goal->planning_time);
      }

      move_group.setPoseTarget(goal->target_pose);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled before planning";
        return result;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_result = move_group.plan(plan);

      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        result->message = "Planning failed - no IK solution or collision detected";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      if (goal->plan_only) {
        result->success = true;
        result->message = "Plan found (plan_only=true, not executing)";
        return result;
      }

      feedback->progress = 0.3;
      feedback->status_message = "Plan found, executing trajectory...";
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
        result->message = "Trajectory execution failed";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      result->final_pose = goal->target_pose;
      result->success = true;
      result->message = "Successfully moved to Cartesian pose in " +
        std::to_string(result->execution_time_sec) + "s";

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
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::MoveToCartesianPoseSkill)
