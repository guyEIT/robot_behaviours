#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "robot_skills_msgs/action/check_system_ready.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_arm_skills
{

class CheckSystemReadySkill
  : public SkillBase<robot_skills_msgs::action::CheckSystemReady>
{
public:
  using CheckSystemReady = robot_skills_msgs::action::CheckSystemReady;
  using Base = SkillBase<CheckSystemReady>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit CheckSystemReadySkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<CheckSystemReady>(
      "check_system_ready_skill",
      "/skill_atoms/check_system_ready",
      options)
  {
    // All known systems and their default check state
    this->declare_parameter("default_systems", std::vector<std::string>{
      "move_group", "joint_states", "arm_controller", "gripper_controller"});
    this->declare_parameter("move_group_action", "/move_action");
    this->declare_parameter("arm_controller_action",
      "/arm_controller/follow_joint_trajectory");
    this->declare_parameter("gripper_controller_action",
      "/gripper_controller/gripper_cmd");
    this->declare_parameter("joint_states_topic", "/joint_states");
    this->declare_parameter("camera_topic", "/camera/color/image_raw");
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "check_system_ready";
    desc.display_name = "Check System Ready";
    desc.description =
      "Verify that required robot subsystems (move_group, controllers, sensors) "
      "are available and responsive. Use at the start of behavior trees to "
      "ensure the robot driver is online before attempting motion. "
      "Can be wrapped in RetryUntilSuccessful to wait for systems to come up.";
    desc.version = "1.0.0";
    desc.category = "utility";
    desc.tags = {"utility", "health", "readiness", "startup"};

    desc.preconditions = {};  // No preconditions — this IS the precondition check
    desc.postconditions = {"systems_verified"};
    desc.effects = {"(systems-ready)"};
    desc.constraints = {"timeout_sec should be > 0"};

    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "required_systems": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Systems to check. Empty = check defaults. Known: move_group, joint_states, arm_controller, gripper_controller, camera",
          "default": []
        },
        "timeout_sec": {
          "type": "number",
          "description": "Max wait per system in seconds",
          "minimum": 0.1,
          "default": 5.0
        }
      }
    })json";

    desc.pddl_action = R"(
(:action check_system_ready
  :parameters (?robot - robot)
  :precondition ()
  :effect (systems-ready ?robot)
))";

    desc.action_type = "robot_skills_msgs/action/CheckSystemReady";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    if (goal->timeout_sec < 0.0) {
      return {false, "timeout_sec must be >= 0"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<CheckSystemReady::Result>();
    const auto & goal = goal_handle->get_goal();

    // Determine which systems to check
    std::vector<std::string> systems_to_check = goal->required_systems;
    if (systems_to_check.empty()) {
      systems_to_check = this->get_parameter("default_systems").as_string_array();
    }

    const double timeout = goal->timeout_sec > 0.0 ? goal->timeout_sec : 5.0;
    const auto timeout_dur = std::chrono::duration<double>(timeout);

    RCLCPP_INFO(this->get_logger(),
      "Checking %zu systems with %.1fs timeout each",
      systems_to_check.size(), timeout);

    auto feedback = std::make_shared<CheckSystemReady::Feedback>();
    int checked = 0;

    for (const auto & system : systems_to_check) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled during system checks";
        return result;
      }

      feedback->current_check = system;
      feedback->progress = static_cast<float>(checked) /
        static_cast<float>(systems_to_check.size());
      feedback->status_message = "Checking " + system + "...";
      goal_handle->publish_feedback(feedback);

      bool available = checkSystem(system, timeout_dur);

      if (available) {
        result->available_systems.push_back(system);
        RCLCPP_INFO(this->get_logger(), "  [OK] %s", system.c_str());
      } else {
        result->unavailable_systems.push_back(system);
        RCLCPP_WARN(this->get_logger(), "  [MISSING] %s", system.c_str());
      }

      checked++;
    }

    // Final feedback
    feedback->progress = 1.0;

    if (result->unavailable_systems.empty()) {
      result->success = true;
      result->message = "All " + std::to_string(systems_to_check.size()) +
        " systems ready";
      feedback->status_message = result->message;
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    } else {
      result->success = false;
      std::string missing;
      for (const auto & s : result->unavailable_systems) {
        if (!missing.empty()) missing += ", ";
        missing += s;
      }
      result->message = "Unavailable: " + missing;
      feedback->status_message = result->message;
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
    }

    goal_handle->publish_feedback(feedback);
    return result;
  }

private:
  bool checkSystem(const std::string & system,
    std::chrono::duration<double> timeout)
  {
    if (system == "move_group") {
      return checkActionServer<moveit_msgs::action::MoveGroup>(
        this->get_parameter("move_group_action").as_string(), timeout);
    } else if (system == "arm_controller") {
      return checkActionServer<control_msgs::action::FollowJointTrajectory>(
        this->get_parameter("arm_controller_action").as_string(), timeout);
    } else if (system == "gripper_controller") {
      return checkActionServer<control_msgs::action::GripperCommand>(
        this->get_parameter("gripper_controller_action").as_string(), timeout);
    } else if (system == "joint_states") {
      return checkTopic(
        this->get_parameter("joint_states_topic").as_string());
    } else if (system == "camera") {
      return checkTopic(
        this->get_parameter("camera_topic").as_string());
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown system '%s', skipping",
        system.c_str());
      return false;
    }
  }

  template <typename ActionT>
  bool checkActionServer(const std::string & action_name,
    std::chrono::duration<double> timeout)
  {
    auto client = rclcpp_action::create_client<ActionT>(this, action_name);
    bool ready = client->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
    return ready;
  }

  bool checkTopic(const std::string & topic_name)
  {
    // Check if any node is publishing on this topic
    size_t pub_count = this->count_publishers(topic_name);
    return pub_count > 0;
  }
};

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::CheckSystemReadySkill)
