#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "robot_skills_msgs/action/record_rosbag.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

namespace robot_arm_skills
{

/**
 * RecordRosbag — fire-and-forget background recording.
 *
 * Spawns `ros2 bag record` as a detached child process and returns SUCCESS
 * immediately so the BT can continue. The recording stops on its own when
 * `duration_sec` elapses (via `--max-duration`) or runs until externally
 * killed when `duration_sec <= 0`.
 *
 * Cancellation of the action goal does NOT stop the recording — the goal
 * has already returned. To stop a still-running recording manually:
 *   pkill -INT -f 'ros2 bag record'
 *
 * Why detached and not blocking: behaviour trees that include data capture
 * shouldn't pause for the entire recording window — recording is a side-channel
 * task, not a step in the manipulation sequence.
 */
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
      "Start a rosbag recording in the background. Returns immediately so the "
      "BT continues. Recording stops automatically after duration_sec, or runs "
      "until killed manually if duration_sec <= 0.";
    desc.version = "2.0.0";
    desc.category = "data";
    desc.tags = {"data", "recording", "rosbag", "background"};

    desc.preconditions = {};
    desc.postconditions = {"rosbag_recording_started"};

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
          "description": "Recording duration in seconds (<=0 = until killed)",
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

    // Build the ros2 bag record command. We rely on bash to handle backgrounding
    // and detachment so the child survives even if this node exits.
    //
    // --max-duration stops the recorder cleanly after N seconds. setsid puts
    // the recorder in its own session, detaching it from our process group.
    const std::string bag_path = output_dir + "/recording_" +
      std::to_string(static_cast<int64_t>(this->now().seconds()));

    std::string topics_part;
    if (goal->topics.empty()) {
      topics_part = " -a";
    } else {
      for (const auto & topic : goal->topics) {
        topics_part += " " + topic;
      }
    }

    std::string duration_part;
    if (goal->duration_sec > 0.0) {
      duration_part = " --max-duration " +
        std::to_string(static_cast<int64_t>(goal->duration_sec));
    }

    // Redirect output to a logfile next to the bag so the user can inspect
    // why a recording failed (e.g. ros2 bag plugin missing). Without this,
    // failures are silent because the parent doesn't read the child's pipes.
    const std::string logfile = bag_path + ".log";
    // Spawn detached via setsid so the child has its own session/PGID, then
    // echo the child PID so we can return it for StopRecording to target.
    const std::string cmd = "setsid ros2 bag record -o " + bag_path +
      topics_part + duration_part +
      " > " + logfile + " 2>&1 < /dev/null & echo $!";

    RCLCPP_INFO(this->get_logger(),
      "RecordRosbag (background): %s", cmd.c_str());

    // Publish a single feedback message so action clients see the start signal.
    auto feedback = std::make_shared<RecordRosbag::Feedback>();
    feedback->status_message = "Background recording started";
    feedback->elapsed_sec = 0.0;
    feedback->messages_so_far = 0;
    goal_handle->publish_feedback(feedback);

    // popen captures the echoed PID. The child remains detached because of
    // setsid + I/O redirects; popen's pipe closes when the bash wrapper exits.
    FILE * pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      result->success = false;
      result->message = "Failed to spawn 'ros2 bag record' subprocess (popen)";
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      return result;
    }
    char buf[64] = {0};
    int64_t pid = 0;
    if (fgets(buf, sizeof(buf), pipe) != nullptr) {
      try { pid = std::stoll(buf); } catch (...) { pid = 0; }
    }
    pclose(pipe);

    // Sanity check: did the recorder die immediately? Brief pause + verify the
    // PID is still alive (kill(pid, 0) returns 0 if process exists).
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (pid > 0 && ::kill(static_cast<pid_t>(pid), 0) != 0) {
      result->success = false;
      result->message = "ros2 bag record (pid=" + std::to_string(pid) +
        ") died on startup; check " + logfile;
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    result->bag_path = bag_path;
    result->recorded_duration_sec = goal->duration_sec;
    result->num_messages = 0;  // not tracked from outside
    result->recording_pid = pid;
    result->success = true;
    result->message = "Recording '" + bag_path + "' running in background pid="
      + std::to_string(pid)
      + (goal->duration_sec > 0.0
        ? " (auto-stops after " + std::to_string(static_cast<int64_t>(goal->duration_sec)) + "s)"
        : " (until killed)")
      + ". Log: " + logfile;

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::RecordRosbagSkill)
