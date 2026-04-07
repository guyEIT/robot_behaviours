#include "robot_mock_skill_atoms/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/check_system_ready.hpp"

namespace robot_mock_skill_atoms {

class MockCheckSystemReady
  : public robot_skill_atoms::SkillBase<robot_skills_msgs::action::CheckSystemReady>
{
public:
  using Action = robot_skills_msgs::action::CheckSystemReady;
  using Base = robot_skill_atoms::SkillBase<Action>;

  explicit MockCheckSystemReady(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_check_system_ready_skill", "/skill_atoms/check_system_ready", options)
  {
    this->declare_parameter("check_delay", 0.3);
    this->declare_parameter("simulate_failure", false);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "check_system_ready";
    desc.display_name = "Check System Ready";
    desc.description = "Verify that required robot subsystems are available.";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/CheckSystemReady";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("check_delay").as_double();
    bool simulate_failure = this->get_parameter("simulate_failure").as_bool();

    std::vector<std::string> systems = goal->required_systems;
    if (systems.empty()) {
      systems = {"move_group", "joint_states", "arm_controller", "gripper_controller"};
    }

    auto feedback = std::make_shared<Action::Feedback>();
    for (size_t i = 0; i < systems.size(); i++) {
      feedback->current_check = systems[i];
      feedback->progress = static_cast<float>(i) / static_cast<float>(systems.size());
      feedback->status_message = "[MOCK] Checking " + systems[i] + "...";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(
        std::chrono::duration<double>(delay / static_cast<double>(systems.size())));
    }

    if (simulate_failure) {
      result->success = false;
      result->unavailable_systems = systems;
      result->message = "[MOCK] Simulated system check failure";
    } else {
      result->success = true;
      result->available_systems = systems;
      result->message = "[MOCK] All " + std::to_string(systems.size()) + " systems ready";
    }

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockCheckSystemReady)
