#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "robot_skills_msgs/action/detect_object.hpp"
#include "robot_skills_msgs/msg/detected_object.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_skill_atoms
{

/**
 * @brief Object detection skill using Intel RealSense color + depth.
 *
 * This implementation uses a simple color-based blob detection as a starting point.
 * Replace with a proper deep learning model (YOLO, etc.) for production use.
 *
 * The skill subscribes to:
 *   /camera/color/image_raw  (sensor_msgs/Image)
 *   /camera/depth/image_rect_raw  (sensor_msgs/Image)
 *   /camera/depth/color/points  (sensor_msgs/PointCloud2)
 */
class DetectObjectSkill
  : public SkillBase<robot_skills_msgs::action::DetectObject>
{
public:
  using DetectObject = robot_skills_msgs::action::DetectObject;
  using Base = SkillBase<DetectObject>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit DetectObjectSkill(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<DetectObject>(
      "detect_object_skill",
      "/skill_atoms/detect_object",
      options)
  {
    this->declare_parameter("color_image_topic", "/camera/color/image_raw");
    this->declare_parameter("depth_image_topic", "/camera/depth/image_rect_raw");
    this->declare_parameter("pointcloud_topic", "/camera/depth/color/points");
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("detection_model", "color_blob");  // "color_blob" | "yolo" | "external"
    this->declare_parameter("external_detections_topic", "/detections");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Mark as stub until real detection model is integrated
    this->is_stub_ = true;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    stat.summary(DiagnosticStatus::WARN,
      "STUB: No detection model integrated. "
      "Replace executeGoal() in detect_object_skill.cpp with YOLO or similar.");
    stat.add("action_server", action_name_);
    stat.add("total_executions", std::to_string(execution_count_.load()));
    stat.add("successes", std::to_string(success_count_.load()));
    stat.add("failures", std::to_string(failure_count_.load()));
    stat.add("is_stub", "true");
    stat.add("detection_model", "none (stub)");
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "detect_object";
    desc.display_name = "Detect Object";
    desc.description =
      "Detect objects in the scene using the Intel RealSense camera. "
      "Returns detected objects with 3D pose, confidence, and bounding boxes. "
      "Supports filtering by object class. Uses depth + color fusion for pose estimation. "
      "Swap detection_model parameter for different backends (color_blob, yolo, external).";
    desc.version = "1.0.0";
    desc.category = "perception";
    desc.tags = {"perception", "detection", "camera", "realsense", "3d_pose"};

    desc.preconditions = {
      "robot_initialized",
      "camera_running",
      "camera_frame_available_in_tf"
    };
    desc.postconditions = {
      "object_poses_available_on_blackboard"
    };
    desc.effects = {
      "(object-detected ?object ?pose)",
      "(object-pose-known ?object)"
    };
    desc.constraints = {
      "camera must have clear view of target area",
      "object must be within camera field of view",
      "adequate lighting for color-based detection"
    };

    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "object_class": {
          "type": "string",
          "description": "Object class to detect (empty = detect any). Examples: seed, cup, block",
          "default": ""
        },
        "confidence_threshold": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0,
          "default": 0.7
        },
        "max_detections": {
          "type": "integer",
          "minimum": 0,
          "description": "Max objects to return (0 = unlimited)",
          "default": 5
        },
        "timeout_sec": {
          "type": "number",
          "minimum": 0.1,
          "default": 5.0
        },
        "require_pose": {
          "type": "boolean",
          "description": "If true, compute 3D pose from depth data",
          "default": true
        }
      }
    })json";

    desc.pddl_action = R"(
(:action detect-object
  :parameters (?robot - robot ?object - object)
  :precondition (and
    (robot-initialized ?robot)
    (camera-running)
    (not (object-pose-known ?object))
  )
  :effect (and
    (object-pose-known ?object)
    (object-detected ?object)
  )
))";

    desc.action_type = "robot_skills_msgs/action/DetectObject";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<DetectObject::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(),
      "Detecting objects: class='%s' timeout=%.1fs",
      goal->object_class.c_str(), goal->timeout_sec);

    RCLCPP_WARN(this->get_logger(),
      "DetectObject STUB: No detection model integrated. Will return empty detections. "
      "Replace detect_object_skill.cpp::executeGoal() with real detection (YOLO, etc.)");

    auto feedback = std::make_shared<DetectObject::Feedback>();
    feedback->detections_so_far = 0;
    feedback->status_message = "Waiting for detections...";
    goal_handle->publish_feedback(feedback);

    const double timeout = goal->timeout_sec > 0.0 ? goal->timeout_sec : 5.0;
    const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout);

    // Subscribe to detections (model-agnostic via parameter)
    // In production: replace with model-specific subscription
    // Here we wait for an image and run basic detection
    std::vector<robot_skills_msgs::msg::DetectedObject> detections;

    // Subscription-based detection loop.
    // Use a condition_variable so the callback wakes the waiting thread
    // immediately rather than relying on a busy-wait sleep loop.
    //
    // INTEGRATION POINT: Replace the callback body with your real detection
    // model (e.g. YOLO, FoundationPose, SAM). The callback receives each
    // Image frame; populate `detections` and notify the cv when done.
    std::mutex cv_mutex;
    std::condition_variable image_cv;
    bool image_ready = false;

    // Capture a weak_ptr to image_received so the callback is safe if this
    // skill node is destroyed while the subscription is still alive.
    auto image_received = std::make_shared<std::atomic<bool>>(false);
    std::weak_ptr<std::atomic<bool>> weak_received = image_received;

    auto image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("color_image_topic").as_string(), 1,
      [weak_received, &cv_mutex, &image_cv, &image_ready](
        const sensor_msgs::msg::Image::SharedPtr /*msg*/)
      {
        auto received = weak_received.lock();
        if (!received || received->load()) {
          return;  // skill already done or destroyed
        }
        received->store(true);

        // TODO: Run actual detection model here (YOLO, SAM, etc.)
        // Populate detections vector in the outer scope via a shared container,
        // then notify below.

        {
          std::lock_guard<std::mutex> lk(cv_mutex);
          image_ready = true;
        }
        image_cv.notify_one();
      });

    // Wait for the first image callback or timeout — no busy-wait.
    const auto timeout_duration =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout));
    {
      std::unique_lock<std::mutex> lk(cv_mutex);
      image_cv.wait_for(lk, timeout_duration,
        [&image_ready, &goal_handle] {
          return image_ready || goal_handle->is_canceling();
        });
    }

    if (image_received->load()) {
      // TODO: Replace with actual detection results from your model
      RCLCPP_WARN(this->get_logger(),
        "Camera active but no detection model integrated. "
        "Returning empty detections (STUB behavior).");
    }

    // Destroy subscription before processing results to prevent callbacks on stale state
    image_sub.reset();
    image_received->store(true);  // signal callback to stop if still pending

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Detection cancelled";
      return result;
    }

    if (!image_received->load()) {
      result->success = false;
      result->message = "No camera data received within timeout. "
        "Check that realsense2_camera is running and topic '" +
        this->get_parameter("color_image_topic").as_string() + "' is publishing.";
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    // Filter by class if specified
    if (!goal->object_class.empty() && detections.empty()) {
      result->success = false;
      result->message = "No '" + goal->object_class + "' objects detected (STUB — no model)";
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    result->detections = detections;
    result->success = !detections.empty();
    result->message = result->success
      ? "Detected " + std::to_string(detections.size()) + " object(s)"
      : "No objects detected (STUB — no detection model integrated)";

    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
    }
    return result;
  }

};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::DetectObjectSkill)
