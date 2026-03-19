#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "robot_skills_msgs/action/gripper_control.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_skill_atoms
{

class GripperControlSkill
  : public SkillBase<robot_skills_msgs::action::GripperControl>
{
public:
  using GripperControl = robot_skills_msgs::action::GripperControl;
  using GripperCommand = control_msgs::action::GripperCommand;
  using Base = SkillBase<GripperControl>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit GripperControlSkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<GripperControl>(
      "gripper_control_skill",
      "/skill_atoms/gripper_control",
      options)
  {
    this->declare_parameter("gripper_action_server", "/gripper_controller/gripper_cmd");
    this->declare_parameter("open_position", 0.085);   // meters
    this->declare_parameter("closed_position", 0.0);   // meters
    this->declare_parameter("default_force_limit", 50.0); // Newtons
    this->declare_parameter("default_speed", 0.1);    // m/s
    this->declare_parameter("action_server_timeout_s", 3.0);
    this->declare_parameter("send_goal_timeout_s", 10.0);
    this->declare_parameter("result_timeout_s", 15.0);

    // Create the gripper action client once and reuse across goals.
    const std::string gripper_action =
      this->get_parameter("gripper_action_server").as_string();
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      shared_from_this(), gripper_action);
    RCLCPP_INFO(this->get_logger(),
      "Gripper client created for action server: %s", gripper_action.c_str());
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "gripper_control";
    desc.display_name = "Gripper Control";
    desc.description =
      "Control the robot gripper: open, close, or set to a specific position. "
      "Close command with force limit allows detecting when an object is grasped "
      "(force-controlled grasping). Object grasped is indicated in result.";
    desc.version = "1.0.0";
    desc.category = "manipulation";
    desc.tags = {"manipulation", "gripper", "grasp", "release"};

    desc.preconditions = {
      "robot_initialized",
      "no_estop_active",
      "gripper_controller_running"
    };
    desc.postconditions = {
      "gripper_at_position"
    };
    desc.effects = {
      "(gripper-open ?robot)",
      "(gripper-closed ?robot)",
      "(object-grasped ?robot ?object)"
    };
    desc.constraints = {
      "position must be 0.0-1.0 (normalized) for set_position command",
      "force_limit in Newtons, 0 = use default",
      "do not close gripper while EEF is inside an object"
    };

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["command"],
      "properties": {
        "command": {
          "type": "string",
          "enum": ["open", "close", "set_position"],
          "description": "Gripper command"
        },
        "position": {
          "type": "number",
          "description": "Target position (0.0=closed, 1.0=open), used with set_position",
          "minimum": 0.0,
          "maximum": 1.0,
          "default": 0.5
        },
        "force_limit": {
          "type": "number",
          "description": "Maximum grip force in Newtons (0 = default)",
          "minimum": 0.0,
          "default": 0.0
        }
      }
    })json";

    desc.pddl_action = R"(
(:action close-gripper
  :parameters (?robot - robot)
  :precondition (and
    (robot-initialized ?robot)
    (not (estop-active))
    (gripper-open ?robot)
  )
  :effect (and
    (gripper-closed ?robot)
    (not (gripper-open ?robot))
  )
))";

    desc.action_type = "robot_skills_msgs/action/GripperControl";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    const std::set<std::string> valid_commands = {"open", "close", "set_position"};
    if (valid_commands.find(goal->command) == valid_commands.end()) {
      return {false, "Invalid command '" + goal->command +
        "'. Must be: open, close, set_position"};
    }
    if (goal->command == "set_position" &&
        (goal->position < 0.0 || goal->position > 1.0))
    {
      return {false, "position must be 0.0-1.0 for set_position command"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<GripperControl::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(), "Gripper command: '%s'", goal->command.c_str());

    auto feedback = std::make_shared<GripperControl::Feedback>();
    feedback->current_position = -1.0;  // Unknown until controller responds
    goal_handle->publish_feedback(feedback);

    try {
      // Compute target position in meters
      const double open_pos = this->get_parameter("open_position").as_double();
      const double closed_pos = this->get_parameter("closed_position").as_double();
      double target_pos_m = 0.0;

      if (goal->command == "open") {
        target_pos_m = open_pos;
      } else if (goal->command == "close") {
        target_pos_m = closed_pos;
      } else {
        // Normalize 0-1 to actual range
        target_pos_m = closed_pos + goal->position * (open_pos - closed_pos);
      }

      const double force = goal->force_limit > 0.0
        ? goal->force_limit
        : this->get_parameter("default_force_limit").as_double();

      // Send to gripper controller via ros2_control action
      const std::string gripper_action =
        this->get_parameter("gripper_action_server").as_string();
      const int action_timeout =
        static_cast<int>(this->get_parameter("action_server_timeout_s").as_double());
      const int send_timeout =
        static_cast<int>(this->get_parameter("send_goal_timeout_s").as_double());
      const int result_timeout =
        static_cast<int>(this->get_parameter("result_timeout_s").as_double());

      if (!gripper_client_->wait_for_action_server(std::chrono::seconds(action_timeout))) {
        result->success = false;
        result->message = "Gripper controller action server not available: " + gripper_action;
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Gripper command cancelled before execution";
        return result;
      }

      GripperCommand::Goal gripper_goal;
      gripper_goal.command.position = target_pos_m;
      gripper_goal.command.max_effort = force;

      // Feedback: report current state
      goal_handle->publish_feedback(feedback);

      auto future = gripper_client_->async_send_goal(gripper_goal);
      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future,
            std::chrono::seconds(send_timeout)) != rclcpp::FutureReturnCode::SUCCESS)
      {
        result->success = false;
        result->message = "Gripper goal timed out";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      auto goal_handle_gripper = future.get();
      if (!goal_handle_gripper) {
        result->success = false;
        result->message = "Gripper goal rejected";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      if (goal_handle->is_canceling()) {
        gripper_client_->async_cancel_goal(goal_handle_gripper);
        result->success = false;
        result->message = "Gripper command cancelled during execution";
        return result;
      }

      // Feedback: waiting for result
      goal_handle->publish_feedback(feedback);

      auto result_future = gripper_client_->async_get_result(goal_handle_gripper);
      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result_future,
            std::chrono::seconds(result_timeout)) != rclcpp::FutureReturnCode::SUCCESS)
      {
        result->success = false;
        result->message = "Gripper result timed out";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      const auto & gripper_result = result_future.get();
      result->final_position = gripper_result.result->position;
      result->object_grasped = gripper_result.result->stalled && goal->command == "close";
      result->success = true;
      result->message = result->object_grasped
        ? "Gripper closed - object grasped (stall detected)"
        : "Gripper '" + goal->command + "' complete";

      RCLCPP_INFO(this->get_logger(), "%s (pos=%.4f)",
        result->message.c_str(), result->final_position);
    } catch (const std::exception & e) {
      result->success = false;
      result->message = std::string("Gripper control exception: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::GripperControlSkill)
