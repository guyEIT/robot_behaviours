#include "robot_mock_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "robot_skills_msgs/action/move_to_cartesian_pose.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_mock_skill_atoms
{

class MockMoveToCartesianPose
  : public robot_skill_atoms::SkillBase<robot_skills_msgs::action::MoveToCartesianPose>
{
public:
  using MoveToCartesianPose = robot_skills_msgs::action::MoveToCartesianPose;
  using Base = robot_skill_atoms::SkillBase<MoveToCartesianPose>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MockMoveToCartesianPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_move_to_cartesian_pose_skill", "/skill_atoms/move_to_cartesian_pose", options)
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
    this->declare_parameter("move_to_cartesian_pose_delay", 1.5);
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
      "robot_initialized", "no_estop_active",
      "planning_scene_valid", "target_pose_in_workspace"
    };
    desc.postconditions = {"end_effector_at_target_pose"};
    desc.effects = {"(end-effector-at ?pose)"};
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
    if (p.x < this->get_parameter("workspace_min_x").as_double() ||
        p.x > this->get_parameter("workspace_max_x").as_double() ||
        p.y < this->get_parameter("workspace_min_y").as_double() ||
        p.y > this->get_parameter("workspace_max_y").as_double() ||
        p.z < this->get_parameter("workspace_min_z").as_double() ||
        p.z > this->get_parameter("workspace_max_z").as_double())
    {
      return {false, "Target pose is outside workspace bounds"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<MoveToCartesianPose::Result>();
    const auto & goal = goal_handle->get_goal();
    const double delay = this->get_parameter("move_to_cartesian_pose_delay").as_double();

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] MoveToCartesianPose: [%.3f, %.3f, %.3f] (simulating %.1fs)",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      delay);

    const int steps = 10;
    const auto step_duration = std::chrono::duration<double>(delay / steps);
    auto feedback = std::make_shared<MoveToCartesianPose::Feedback>();

    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled";
        return result;
      }
      feedback->progress = static_cast<double>(i) / steps;
      feedback->status_message = (i < steps)
        ? "Moving to Cartesian pose..."
        : "Reached target pose";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(step_duration);
    }

    result->final_pose = goal->target_pose;
    result->success = true;
    result->execution_time_sec = delay;
    result->message = "[MOCK] Moved to Cartesian pose in " + std::to_string(delay) + "s";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockMoveToCartesianPose)
