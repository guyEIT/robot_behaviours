#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/move_to_joint_config.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_mock_skill_atoms {

class MockMoveToJointConfig
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::MoveToJointConfig>
{
public:
  using Action = robot_skills_msgs::action::MoveToJointConfig;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockMoveToJointConfig(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_move_to_joint_config_skill", "/skill_atoms/move_to_joint_config", options)
  {
    this->declare_parameter("move_to_joint_config_delay", 1.0);
    joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mock/joint_target", 10);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_to_joint_config";
    desc.display_name = "Move to Joint Config";
    desc.description = "Move robot to arbitrary joint angles.";
    desc.category = "motion";
    desc.action_type = "robot_skills_msgs/action/MoveToJointConfig";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("move_to_joint_config_delay").as_double();

    if (goal->joint_positions.size() >= 7) {
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data.assign(goal->joint_positions.begin(), goal->joint_positions.begin() + 7);
      joint_target_pub_->publish(msg);
    }

    auto feedback = std::make_shared<Action::Feedback>();
    int steps = 10;
    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false; result->message = "Cancelled"; return result;
      }
      feedback->progress = static_cast<double>(i) / steps;
      feedback->status_message = "Moving to joint config...";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::duration<double>(delay / steps));
    }

    result->success = true;
    result->execution_time_sec = delay;
    result->message = "[MOCK] Moved to joint config";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockMoveToJointConfig)
