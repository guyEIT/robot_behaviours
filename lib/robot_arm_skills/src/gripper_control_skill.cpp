#include "robot_arm_skills/skill_base.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "robot_skills_msgs/action/gripper_control.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_arm_skills
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

    // Sim mode: bypass the gripper action server entirely. Open/close
    // returns success immediately; close returns object_grasped=true so
    // pick-and-place trees can verify grasp in lab-sim where no physical
    // object is held.
    this->declare_parameter("simulate_grasp", false);

    // Action client is created lazily in executeGoal() because
    // shared_from_this() is not available during construction.
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

    if (this->get_parameter("simulate_grasp").as_bool()) {
      RCLCPP_INFO(this->get_logger(),
        "[SIM] GripperControl: command='%s' (skipping real gripper action)",
        goal->command.c_str());
      const double open_pos = this->get_parameter("open_position").as_double();
      const double closed_pos = this->get_parameter("closed_position").as_double();
      result->success = true;
      if (goal->command == "open") {
        result->final_position = open_pos;
        result->object_grasped = false;
        result->message = "[SIM] Gripper opened";
      } else if (goal->command == "close") {
        result->final_position = closed_pos + 0.005;  // pretend object width
        result->object_grasped = true;                // fake successful grasp
        result->message = "[SIM] Gripper closed on object";
      } else {
        result->final_position =
          closed_pos + goal->position * (open_pos - closed_pos);
        result->object_grasped = false;
        result->message = "[SIM] Gripper at position " +
          std::to_string(result->final_position);
      }
      return result;
    }

    // Lazy-create the action client (shared_from_this() not available in ctor)
    if (!gripper_client_) {
      const std::string gripper_action =
        this->get_parameter("gripper_action_server").as_string();
      gripper_client_ = rclcpp_action::create_client<GripperCommand>(
        shared_from_this(), gripper_action);
      RCLCPP_INFO(this->get_logger(),
        "Gripper client created for action server: %s", gripper_action.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Gripper command: '%s'", goal->command.c_str());

    auto feedback = std::make_shared<GripperControl::Feedback>();
    feedback->current_position = -1.0;
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
        target_pos_m = closed_pos + goal->position * (open_pos - closed_pos);
      }

      const double force = goal->force_limit > 0.0
        ? goal->force_limit
        : this->get_parameter("default_force_limit").as_double();

      const std::string gripper_action =
        this->get_parameter("gripper_action_server").as_string();
      const int action_timeout =
        static_cast<int>(this->get_parameter("action_server_timeout_s").as_double());
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

      // Use callback-based SendGoalOptions so the launch executor processes
      // everything — no spin_until_future_complete or async_get_result needed.
      std::promise<bool> goal_accepted_promise;
      auto goal_accepted_future = goal_accepted_promise.get_future();

      struct ControllerResult {
        double position = 0.0;
        bool stalled = false;
        bool succeeded = false;
      };
      std::promise<ControllerResult> result_promise;
      auto ctrl_result_future = result_promise.get_future();

      auto send_goal_options =
        rclcpp_action::Client<GripperCommand>::SendGoalOptions();

      send_goal_options.goal_response_callback =
        [this, &goal_accepted_promise](
          const rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr & gh)
        {
          goal_accepted_promise.set_value(gh != nullptr);
        };

      send_goal_options.result_callback =
        [this, &result_promise](
          const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult & wr)
        {
          ControllerResult cr;
          cr.succeeded = (wr.code == rclcpp_action::ResultCode::SUCCEEDED);
          cr.position = wr.result->position;
          cr.stalled = wr.result->stalled;
          result_promise.set_value(cr);
        };

      gripper_client_->async_send_goal(gripper_goal, send_goal_options);

      // Wait for goal acceptance
      if (goal_accepted_future.wait_for(std::chrono::seconds(action_timeout)) !=
          std::future_status::ready || !goal_accepted_future.get())
      {
        result->success = false;
        result->message = "Gripper goal rejected or timed out";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      RCLCPP_INFO(this->get_logger(), "Gripper goal accepted, waiting for result...");

      // Wait for controller result
      {
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::seconds(result_timeout);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
          if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Gripper command cancelled during execution";
            return result;
          }
          if (ctrl_result_future.wait_for(std::chrono::milliseconds(50)) ==
              std::future_status::ready) { break; }
        }
        if (ctrl_result_future.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready)
        {
          result->success = false;
          result->message = "Gripper result timed out";
          RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
          return result;
        }
      }

      auto ctrl_result = ctrl_result_future.get();
      result->final_position = ctrl_result.position;
      result->object_grasped = ctrl_result.stalled && goal->command == "close";
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

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::GripperControlSkill)
