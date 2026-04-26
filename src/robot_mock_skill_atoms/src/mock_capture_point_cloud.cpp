#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/capture_point_cloud.hpp"

namespace robot_mock_skill_atoms {

class MockCapturePointCloud
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::CapturePointCloud>
{
public:
  using Action = robot_skills_msgs::action::CapturePointCloud;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockCapturePointCloud(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_capture_point_cloud_skill", "/skill_atoms/capture_point_cloud", options)
  {
    this->declare_parameter("capture_delay", 0.5);
    this->declare_parameter("mock_num_points", 15000);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "capture_point_cloud";
    desc.display_name = "Capture Point Cloud";
    desc.description = "Trigger a depth capture from the camera.";
    desc.category = "perception";
    desc.action_type = "robot_skills_msgs/action/CapturePointCloud";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    double delay = this->get_parameter("capture_delay").as_double();

    auto feedback = std::make_shared<Action::Feedback>();
    feedback->status_message = "Capturing point cloud...";
    goal_handle->publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    result->success = true;
    result->num_points = this->get_parameter("mock_num_points").as_int();
    result->message = "[MOCK] Captured " + std::to_string(result->num_points) + " points";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockCapturePointCloud)
