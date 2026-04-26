#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include "robot_skills_msgs/action/move_cartesian_linear.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_mock_skill_atoms {

class MockMoveCartesianLinear
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::MoveCartesianLinear>
{
public:
  using Action = robot_skills_msgs::action::MoveCartesianLinear;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockMoveCartesianLinear(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_move_cartesian_linear_skill", "/skill_atoms/move_cartesian_linear", options)
  {
    this->declare_parameter("move_cartesian_linear_delay", 2.0);
    joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mock/joint_target", 10);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "move_cartesian_linear";
    desc.display_name = "Move Cartesian Linear";
    desc.description = "Move end-effector in a straight line to target pose.";
    desc.category = "motion";
    desc.action_type = "robot_skills_msgs/action/MoveCartesianLinear";
    return desc;
  }

  std::vector<double> approximateIK(double x, double y, double z)
  {
    constexpr double L1 = 0.333, L2 = 0.316, L3 = 0.384, L4 = 0.107;
    double j1 = std::atan2(y, x);
    double r = std::sqrt(x * x + y * y);
    double h = z - L1;
    double d = std::clamp(std::sqrt(r * r + h * h) - L4, 0.1, L2 + L3 - 0.01);
    double cos_j4 = std::clamp((d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3), -1.0, 1.0);
    double j4 = -(M_PI - std::acos(cos_j4));
    double j2 = std::atan2(h, r) - std::atan2(L3 * std::sin(-j4), L2 + L3 * std::cos(-j4));
    return {j1, j2, 0.0, j4, 0.0, M_PI / 4.0, 0.785398};
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("move_cartesian_linear_delay").as_double();
    const auto & pos = goal->target_pose.pose.position;

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] MoveCartesianLinear: [%.3f, %.3f, %.3f] (simulating %.1fs)",
      pos.x, pos.y, pos.z, delay);

    auto joints = approximateIK(pos.x, pos.y, pos.z);
    auto jmsg = std_msgs::msg::Float64MultiArray();
    jmsg.data = joints;
    joint_target_pub_->publish(jmsg);

    auto feedback = std::make_shared<Action::Feedback>();
    int steps = 10;
    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false; result->message = "Cancelled"; return result;
      }
      feedback->progress = static_cast<double>(i) / steps;
      feedback->status_message = "Linear move...";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::duration<double>(delay / steps));
    }

    result->final_pose = goal->target_pose;
    result->success = true;
    result->fraction_achieved = 1.0;
    result->execution_time_sec = delay;
    result->message = "[MOCK] Linear move completed";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockMoveCartesianLinear)
