#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "robot_skills_msgs/action/move_to_named_config.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_mock_skill_atoms
{

class MockMoveToNamedConfig
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::MoveToNamedConfig>
{
public:
  using MoveToNamedConfig = robot_skills_msgs::action::MoveToNamedConfig;
  using Base = robot_arm_skills::SkillBase<MoveToNamedConfig>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MockMoveToNamedConfig(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_move_to_named_config_skill", "/skill_atoms/move_to_named_config", options)
  {
    this->declare_parameter("default_planning_group", "arm");
    this->declare_parameter("default_velocity_scaling", 0.3);
    this->declare_parameter("allowed_named_configs",
      std::vector<std::string>{"home", "ready", "stow", "observe"});
    this->declare_parameter("move_to_named_config_delay", 1.0);

    joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mock/joint_target", 10);

    // Named configs from sim_robot.srdf (7 arm joints)
    named_configs_["home"]    = {0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398};
    named_configs_["ready"]   = {0.0, -0.5,      0.0, -2.0,      0.0, 1.570796, 0.785398};
    named_configs_["stow"]    = {0.0, -1.2,      0.0, -2.8,      0.0, 1.5,      0.785398};
    named_configs_["observe"] = {0.0,  0.3,      0.0, -1.5,      0.0, 1.8,      0.785398};
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

    desc.preconditions = {"robot_initialized", "no_estop_active", "planning_scene_valid"};
    desc.postconditions = {"robot_at_named_config"};
    desc.effects = {"(robot-at-config ?config_name)"};
    desc.constraints = {
      "config_name must be defined in robot SRDF",
      "velocity_scaling must be 0.0-1.0",
      "reduce velocity_scaling near humans or obstacles"
    };

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
    const auto allowed = this->get_parameter("allowed_named_configs").as_string_array();
    if (std::find(allowed.begin(), allowed.end(), goal->config_name) == allowed.end()) {
      return {false, "config_name '" + goal->config_name + "' not in allowed list"};
    }
    if (goal->velocity_scaling < 0.01 || goal->velocity_scaling > 1.0) {
      return {false, "velocity_scaling must be between 0.01 and 1.0"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<MoveToNamedConfig::Result>();
    const auto & goal = goal_handle->get_goal();
    const double delay = this->get_parameter("move_to_named_config_delay").as_double();

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] MoveToNamedConfig: config='%s' (simulating %.1fs)",
      goal->config_name.c_str(), delay);

    // Publish joint target for the mock joint state publisher
    auto it = named_configs_.find(goal->config_name);
    if (it != named_configs_.end()) {
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data = it->second;
      joint_target_pub_->publish(msg);
    }

    // Simulate execution with progress feedback
    const int steps = 10;
    const auto step_duration = std::chrono::duration<double>(delay / steps);
    auto feedback = std::make_shared<MoveToNamedConfig::Feedback>();

    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cancelled";
        return result;
      }
      feedback->progress = static_cast<double>(i) / steps;
      feedback->status_message = (i < steps)
        ? "Moving to '" + goal->config_name + "'..."
        : "Reached '" + goal->config_name + "'";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(step_duration);
    }

    result->success = true;
    result->execution_time_sec = delay;
    result->message = "[MOCK] Moved to '" + goal->config_name + "' in " +
      std::to_string(delay) + "s";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
  std::map<std::string, std::vector<double>> named_configs_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockMoveToNamedConfig)
