#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/robot_enable.hpp"

namespace robot_mock_skill_atoms {

class MockRobotEnable
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::RobotEnable>
{
public:
  using Action = robot_skills_msgs::action::RobotEnable;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockRobotEnable(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_robot_enable_skill", "/skill_atoms/robot_enable", options)
  {
    this->declare_parameter("enable_delay", 0.5);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "robot_enable";
    desc.display_name = "Robot Enable/Disable";
    desc.description = "Enable or disable the robot (servo on/off).";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/RobotEnable";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("enable_delay").as_double();
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    enabled_ = goal->enable;
    result->success = true;
    result->is_enabled = enabled_;
    result->message = enabled_ ? "[MOCK] Robot enabled" : "[MOCK] Robot disabled";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }

private:
  bool enabled_ = false;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockRobotEnable)
