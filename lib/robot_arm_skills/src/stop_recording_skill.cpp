#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <csignal>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <sys/types.h>

#include "robot_skills_msgs/action/stop_recording.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

namespace robot_arm_skills
{

/**
 * StopRecording — stop a background rosbag recording previously started
 * by RecordRosbag.
 *
 * Usage in BT XML:
 *   <RecordRosbag start_recording bag_path="{bag}" recording_pid="{pid}"/>
 *   ... do work ...
 *   <StopRecording stop_recording recording_pid="{pid}"/>
 *
 * If recording_pid > 0 we send SIGINT to the process group (the recorder
 * uses setsid() so its PGID == its PID). After timeout_sec we escalate to
 * SIGKILL if the process is still alive. If recording_pid <= 0 we fall
 * back to a `pkill -INT -f` match against bag_path.
 *
 * SIGINT lets `ros2 bag record` flush + close the bag cleanly; SIGKILL
 * leaves a partially-written bag.
 */
class StopRecordingSkill
  : public SkillBase<robot_skills_msgs::action::StopRecording>
{
public:
  using StopRecording = robot_skills_msgs::action::StopRecording;
  using Base = SkillBase<StopRecording>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit StopRecordingSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<StopRecording>(
      "stop_recording_skill",
      "/skill_atoms/stop_recording",
      options)
  {
    this->declare_parameter("default_timeout_sec", 5.0);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "stop_recording";
    desc.display_name = "Stop Recording";
    desc.description =
      "Stop a background rosbag recording started by RecordRosbag. Sends "
      "SIGINT to the recorder's process group, escalates to SIGKILL after "
      "timeout. Pair with RecordRosbag via the recording_pid blackboard key.";
    desc.version = "1.0.0";
    desc.category = "data";
    desc.tags = {"data", "recording", "rosbag", "background"};
    desc.preconditions = {"rosbag_recording_started"};
    desc.postconditions = {"rosbag_recording_stopped"};
    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "recording_pid": {
          "type": "integer",
          "description": "PID/PGID of the rosbag recorder (from RecordRosbag.recording_pid)"
        },
        "bag_path": {
          "type": "string",
          "description": "Fallback: pkill -INT -f match against this path"
        },
        "timeout_sec": {
          "type": "number",
          "description": "Seconds to wait for SIGINT before escalating to SIGKILL",
          "default": 5.0
        }
      }
    })json";
    desc.action_type = "robot_skills_msgs/action/StopRecording";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<StopRecording::Result>();
    const auto & goal = goal_handle->get_goal();

    const double timeout_sec = goal->timeout_sec > 0.0
      ? goal->timeout_sec
      : this->get_parameter("default_timeout_sec").as_double();

    int64_t pid = goal->recording_pid;
    if (pid <= 0) {
      // Fallback: pkill by command-line match on the bag path.
      if (goal->bag_path.empty()) {
        result->success = false;
        result->message =
          "neither recording_pid nor bag_path provided — nothing to stop";
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }
      const std::string cmd =
        "pkill -INT -f 'ros2 bag record .*-o " + goal->bag_path + "'";
      RCLCPP_INFO(this->get_logger(),
        "StopRecording (by path): %s", cmd.c_str());
      int rc = std::system(cmd.c_str());
      result->success = (rc == 0);
      result->stopped_pid = 0;
      result->message = result->success
        ? "Sent SIGINT to processes matching '" + goal->bag_path + "'"
        : "pkill returned non-zero (rc=" + std::to_string(rc) +
          ") — recorder likely already gone";
      return result;
    }

    pid_t pgid = static_cast<pid_t>(pid);
    if (::kill(pgid, 0) != 0) {
      result->success = true;  // already gone — that's fine
      result->stopped_pid = pid;
      result->message = "Recorder pid=" + std::to_string(pid) +
        " already exited";
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    // SIGINT to the whole process group (negative pgid). RecordRosbag uses
    // setsid so the recorder's PGID == its PID.
    if (::kill(-pgid, SIGINT) != 0) {
      // Fall back to single-process SIGINT.
      ::kill(pgid, SIGINT);
    }
    RCLCPP_INFO(this->get_logger(),
      "StopRecording: sent SIGINT to pgid=%lld (timeout=%.1fs)",
      static_cast<long long>(pid), timeout_sec);

    auto feedback = std::make_shared<StopRecording::Feedback>();
    feedback->status_message = "SIGINT sent; waiting for clean exit";
    goal_handle->publish_feedback(feedback);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(timeout_sec));
    while (std::chrono::steady_clock::now() < deadline) {
      if (::kill(pgid, 0) != 0) {
        result->success = true;
        result->stopped_pid = pid;
        result->message = "Recorder pid=" + std::to_string(pid) +
          " exited cleanly after SIGINT";
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Escalate.
    RCLCPP_WARN(this->get_logger(),
      "Recorder pid=%lld did not exit after %.1fs SIGINT — sending SIGKILL",
      static_cast<long long>(pid), timeout_sec);
    if (::kill(-pgid, SIGKILL) != 0) {
      ::kill(pgid, SIGKILL);
    }
    result->success = true;
    result->stopped_pid = pid;
    result->message = "Recorder pid=" + std::to_string(pid) +
      " killed (SIGKILL after SIGINT timeout)";
    return result;
  }
};

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::StopRecordingSkill)
