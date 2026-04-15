#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "robot_skills_msgs/action/record_rosbag.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

namespace robot_skill_atoms
{

class RecordRosbagSkill
  : public SkillBase<robot_skills_msgs::action::RecordRosbag>
{
public:
  using RecordRosbag = robot_skills_msgs::action::RecordRosbag;
  using Base = SkillBase<RecordRosbag>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit RecordRosbagSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<RecordRosbag>(
      "record_rosbag_skill",
      "/skill_atoms/record_rosbag",
      options)
  {
    this->declare_parameter("default_output_dir", "/tmp/rosbags");
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "record_rosbag";
    desc.display_name = "Record Rosbag";
    desc.description =
      "Record ROS2 topics to a rosbag file. Spawns 'ros2 bag record' as a "
      "subprocess. Supports duration-based and cancellation-based stop.";
    desc.version = "1.0.0";
    desc.category = "data";
    desc.tags = {"data", "recording", "rosbag"};

    desc.preconditions = {};
    desc.postconditions = {"rosbag_recorded"};

    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "topics": {
          "type": "array",
          "items": {"type": "string"},
          "description": "Topics to record (empty = all)"
        },
        "output_path": {
          "type": "string",
          "description": "Output directory path"
        },
        "duration_sec": {
          "type": "number",
          "description": "Recording duration (0 = until cancelled)",
          "default": 0.0
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/RecordRosbag";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<RecordRosbag::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string output_dir = goal->output_path.empty()
      ? this->get_parameter("default_output_dir").as_string()
      : goal->output_path;

    // Ensure output directory exists
    std::filesystem::create_directories(output_dir);

    // Build the ros2 bag record command
    std::string cmd = "ros2 bag record -o " + output_dir + "/recording";

    if (goal->topics.empty()) {
      cmd += " -a";
    } else {
      for (const auto & topic : goal->topics) {
        cmd += " " + topic;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Executing RecordRosbag: cmd='%s' duration=%.1fs",
      cmd.c_str(), goal->duration_sec);

    auto feedback = std::make_shared<RecordRosbag::Feedback>();
    feedback->status_message = "Starting rosbag recording...";
    feedback->elapsed_sec = 0.0;
    feedback->messages_so_far = 0;
    goal_handle->publish_feedback(feedback);

    try {
      // Start recording subprocess
      FILE * pipe = popen(cmd.c_str(), "r");
      if (!pipe) {
        result->success = false;
        result->message = "Failed to start ros2 bag record subprocess";
        return result;
      }

      const auto start_time = this->now();
      const double duration = goal->duration_sec;

      // Wait for duration or cancellation
      while (rclcpp::ok()) {
        const double elapsed = (this->now() - start_time).seconds();

        // Check for cancellation
        if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Recording cancelled after %.1fs", elapsed);
          break;
        }

        // Check duration (0 = until cancelled)
        if (duration > 0.0 && elapsed >= duration) {
          RCLCPP_INFO(this->get_logger(), "Recording duration reached: %.1fs", elapsed);
          break;
        }

        // Publish feedback
        feedback->elapsed_sec = elapsed;
        feedback->status_message = "Recording... (" +
          std::to_string(static_cast<int>(elapsed)) + "s)";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      // Stop the recording (SIGINT to ros2 bag record)
      pclose(pipe);

      const double total_elapsed = (this->now() - start_time).seconds();

      result->bag_path = output_dir + "/recording";
      result->recorded_duration_sec = total_elapsed;
      result->num_messages = 0;  // not easily available from subprocess
      result->success = true;
      result->message = "Recorded " + std::to_string(total_elapsed) +
        "s to " + result->bag_path;

      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::RecordRosbagSkill)
