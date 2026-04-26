#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <map>
#include <thread>
#include "robot_skills_msgs/action/set_digital_io.hpp"

namespace robot_mock_skill_atoms {

class MockSetDigitalIO
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::SetDigitalIO>
{
public:
  using Action = robot_skills_msgs::action::SetDigitalIO;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockSetDigitalIO(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_set_digital_io_skill", "/skill_atoms/set_digital_io", options)
  {
    this->declare_parameter("io_delay", 0.1);
    pin_states_["vacuum"] = false;
    pin_states_["tool_changer"] = false;
    pin_states_["led"] = false;
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "set_digital_io";
    desc.display_name = "Set Digital I/O";
    desc.description = "Set or read a digital I/O pin.";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/SetDigitalIO";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("io_delay").as_double();
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    if (!goal->read_only) {
      pin_states_[goal->pin_name] = goal->value;
      RCLCPP_INFO(this->get_logger(), "[MOCK] IO '%s' set to %s",
        goal->pin_name.c_str(), goal->value ? "HIGH" : "LOW");
    }

    result->success = true;
    result->current_value = pin_states_[goal->pin_name];
    result->message = "[MOCK] IO " + goal->pin_name + " = " +
      (result->current_value ? "HIGH" : "LOW");
    return result;
  }

private:
  std::map<std::string, bool> pin_states_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockSetDigitalIO)
