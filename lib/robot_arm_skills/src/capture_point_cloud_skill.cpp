#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "robot_skills_msgs/action/capture_point_cloud.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot_arm_skills
{

class CapturePointCloudSkill
  : public SkillBase<robot_skills_msgs::action::CapturePointCloud>
{
public:
  using CapturePointCloud = robot_skills_msgs::action::CapturePointCloud;
  using Base = SkillBase<CapturePointCloud>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit CapturePointCloudSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<CapturePointCloud>(
      "capture_point_cloud_skill",
      "/skill_atoms/capture_point_cloud",
      options)
  {
    this->declare_parameter("default_camera_topic", "/camera/depth/color/points");
    this->declare_parameter("default_timeout_sec", 5.0);

    // Sim mode: skip the camera wait, return a non-empty fake point cloud.
    this->declare_parameter("simulate_perception", false);
    this->declare_parameter("simulate_delay_sec", 0.3);
    this->declare_parameter("simulate_num_points", 1024);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "capture_point_cloud";
    desc.display_name = "Capture Point Cloud";
    desc.description =
      "Capture a single point cloud frame from the depth camera.";
    desc.version = "1.0.0";
    desc.category = "perception";
    desc.tags = {"perception", "point_cloud", "camera", "depth"};

    desc.preconditions = {"camera_running"};
    desc.postconditions = {"point_cloud_available"};

    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "camera_topic": {
          "type": "string",
          "default": "/camera/depth/color/points"
        },
        "timeout_sec": {
          "type": "number",
          "default": 5.0
        },
        "apply_filters": {
          "type": "boolean",
          "default": true
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/CapturePointCloud";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<CapturePointCloud::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string topic = goal->camera_topic.empty()
      ? this->get_parameter("default_camera_topic").as_string()
      : goal->camera_topic;

    const double timeout_sec = goal->timeout_sec > 0.0
      ? goal->timeout_sec
      : this->get_parameter("default_timeout_sec").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Executing CapturePointCloud: topic='%s' timeout=%.1fs",
      topic.c_str(), timeout_sec);

    auto feedback = std::make_shared<CapturePointCloud::Feedback>();
    feedback->status_message = "Waiting for point cloud on " + topic + "...";
    goal_handle->publish_feedback(feedback);

    if (this->get_parameter("simulate_perception").as_bool()) {
      const double delay = this->get_parameter("simulate_delay_sec").as_double();
      const int n_points = static_cast<int>(
        this->get_parameter("simulate_num_points").as_int());
      RCLCPP_INFO(this->get_logger(),
        "[SIM] CapturePointCloud: synthesising %d-point cloud (delay=%.1fs)",
        n_points, delay);
      std::this_thread::sleep_for(std::chrono::duration<double>(delay));
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Capture cancelled";
        return result;
      }
      // Build an empty header-only point cloud — sufficient for trees that
      // only care that something was captured.
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = "world";
      cloud.header.stamp = this->now();
      cloud.height = 1;
      cloud.width = static_cast<uint32_t>(n_points);
      result->point_cloud = cloud;
      result->num_points = n_points;
      result->success = true;
      result->message = "[SIM] Captured " + std::to_string(n_points) + " points";
      feedback->status_message = result->message;
      goal_handle->publish_feedback(feedback);
      return result;
    }

    try {
      // Use a one-shot subscription to capture a single message
      std::mutex mtx;
      std::condition_variable cv;
      sensor_msgs::msg::PointCloud2::SharedPtr captured_cloud;

      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx);
          if (!captured_cloud) {
            captured_cloud = msg;
            cv.notify_one();
          }
        });

      // Wait for the message with timeout
      std::unique_lock<std::mutex> lock(mtx);
      const bool received = cv.wait_for(lock,
        std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0)),
        [&] { return captured_cloud != nullptr; });

      // Destroy subscription immediately
      sub.reset();

      if (!received) {
        result->success = false;
        result->message = "Timed out waiting for point cloud on " + topic;
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        return result;
      }

      result->point_cloud = *captured_cloud;
      result->num_points = static_cast<int32_t>(
        captured_cloud->width * captured_cloud->height);

      result->success = true;
      result->message = "Captured point cloud with " +
        std::to_string(result->num_points) + " points";

      feedback->status_message = result->message;
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_arm_skills

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::CapturePointCloudSkill)
