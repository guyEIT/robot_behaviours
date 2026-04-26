#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <thread>
#include "robot_skills_msgs/action/record_rosbag.hpp"

namespace robot_mock_skill_atoms {

class MockRecordRosbag
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::RecordRosbag>
{
public:
  using Action = robot_skills_msgs::action::RecordRosbag;
  using Base = robot_arm_skills::SkillBase<Action>;

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
    double duration = goal->duration_sec > 0 ? goal->duration_sec : 60.0;

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] Recording started: %.0fs to %s (returns immediately)",
      duration, goal->output_path.c_str());

    // Recording is a background task — return immediately to unblock the tree.
    // A short delay simulates the startup overhead.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    result->success = true;
    result->bag_path = goal->output_path;
    result->recorded_duration_sec = duration;
    result->num_messages = static_cast<int64_t>(
      duration * this->get_parameter("mock_messages_per_sec").as_int());
    result->message = "[MOCK] Recording started";
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockRecordRosbag)
