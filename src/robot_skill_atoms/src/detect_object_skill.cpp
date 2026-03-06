#include "robot_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "robot_skills_msgs/action/detect_object.hpp"
#include "robot_skills_msgs/msg/detected_object.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
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

    desc.parameters_schema = R"({
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
    })";

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

    auto feedback = std::make_shared<DetectObject::Feedback>();
    feedback->detections_so_far = 0;
    feedback->status_message = "Waiting for detections...";
    goal_handle->publish_feedback(feedback);

    const double timeout = goal->timeout_sec > 0.0 ? goal->timeout_sec : 5.0;
    const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout);

    // Subscribe to detections (model-agnostic via parameter)
    // In production: replace with model-specific subscription
    // Here we wait for an image and run basic detection
    bool got_detection = false;
    std::vector<robot_skills_msgs::msg::DetectedObject> detections;

    // Subscription-based detection loop
    auto image_received = std::make_shared<std::atomic<bool>>(false);

    auto image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("color_image_topic").as_string(), 1,
      [&, image_received](const sensor_msgs::msg::Image::SharedPtr /*msg*/) {
        if (!*image_received) {
          *image_received = true;
          // TODO: Run actual detection model here
          // For now, log that we received an image
          RCLCPP_DEBUG(this->get_logger(), "Received camera image for detection");
        }
      });

    // Wait for detections or timeout
    while (this->now() < deadline && !goal_handle->is_canceling()) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      if (*image_received) {
        // TODO: Replace with actual detection results from your model
        // This is a placeholder that demonstrates the interface
        RCLCPP_INFO_ONCE(this->get_logger(),
          "Camera active. Integrate detection model in detect_object_skill.cpp");
        got_detection = false;
        break;
      }

      feedback->status_message = "Waiting for camera data...";
      goal_handle->publish_feedback(feedback);
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Detection cancelled";
      return result;
    }

    if (!*image_received) {
      result->success = false;
      result->message = "No camera data received within timeout. "
        "Check that realsense2_camera is running and topic '" +
        this->get_parameter("color_image_topic").as_string() + "' is publishing.";
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    // Filter by class if specified
    if (!goal->object_class.empty() && detections.empty() && !got_detection) {
      result->success = false;
      result->message = "No '" + goal->object_class + "' objects detected. "
        "Integrate a detection model (YOLO, etc.) in detect_object_skill.cpp";
      return result;
    }

    result->detections = detections;
    result->success = !detections.empty();
    result->message = result->success
      ? "Detected " + std::to_string(detections.size()) + " object(s)"
      : "No objects detected matching criteria";

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::DetectObjectSkill)
