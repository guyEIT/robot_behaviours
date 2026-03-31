#include "robot_mock_skill_atoms/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/record_rosbag.hpp"

namespace robot_mock_skill_atoms {

class MockRecordRosbag
  : public robot_skill_atoms::SkillBase<robot_skills_msgs::action::RecordRosbag>
{
public:
  using Action = robot_skills_msgs::action::RecordRosbag;
  using Base = robot_skill_atoms::SkillBase<Action>;

  explicit MockRecordRosbag(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_record_rosbag_skill", "/skill_atoms/record_rosbag", options)
  {
    this->declare_parameter("mock_messages_per_sec", 100);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "record_rosbag";
    desc.display_name = "Record Rosbag";
    desc.description = "Start or stop rosbag recording of specified topics.";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/RecordRosbag";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    int msg_rate = this->get_parameter("mock_messages_per_sec").as_int();
    double duration = goal->duration_sec > 0 ? goal->duration_sec : 3.0;

    RCLCPP_INFO(this->get_logger(), "[MOCK] Recording rosbag for %.1fs to %s",
      duration, goal->output_path.c_str());

    auto feedback = std::make_shared<Action::Feedback>();
    int steps = 10;
    for (int i = 1; i <= steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = true;
        result->message = "[MOCK] Recording stopped by cancel";
        result->recorded_duration_sec = duration * i / steps;
        result->num_messages = static_cast<int64_t>(result->recorded_duration_sec * msg_rate);
        result->bag_path = goal->output_path;
        return result;
      }
      feedback->elapsed_sec = duration * i / steps;
      feedback->messages_so_far = static_cast<int64_t>(feedback->elapsed_sec * msg_rate);
      feedback->status_message = "Recording...";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::duration<double>(duration / steps));
    }

    result->success = true;
    result->bag_path = goal->output_path;
    result->recorded_duration_sec = duration;
    result->num_messages = static_cast<int64_t>(duration * msg_rate);
    result->message = "[MOCK] Recorded " + std::to_string(result->num_messages) + " messages";
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockRecordRosbag)
