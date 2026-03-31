#include "robot_mock_skill_atoms/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/check_collision.hpp"

namespace robot_mock_skill_atoms {

class MockCheckCollision
  : public robot_skill_atoms::SkillBase<robot_skills_msgs::action::CheckCollision>
{
public:
  using Action = robot_skills_msgs::action::CheckCollision;
  using Base = robot_skill_atoms::SkillBase<Action>;

  explicit MockCheckCollision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_check_collision_skill", "/skill_atoms/check_collision", options)
  {
    this->declare_parameter("check_delay", 0.1);
    this->declare_parameter("mock_collision_result", false);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "check_collision";
    desc.display_name = "Check Collision";
    desc.description = "Check if a configuration is in collision with the environment.";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/CheckCollision";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    double delay = this->get_parameter("check_delay").as_double();
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    result->success = true;
    result->in_collision = this->get_parameter("mock_collision_result").as_bool();
    result->message = result->in_collision
      ? "[MOCK] Configuration is in collision"
      : "[MOCK] Configuration is collision-free";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockCheckCollision)
