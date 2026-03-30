#include "robot_mock_skill_atoms/skill_base.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "robot_skills_msgs/action/detect_object.hpp"
#include "robot_skills_msgs/msg/detected_object.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_mock_skill_atoms
{

class MockDetectObject
  : public robot_skill_atoms::SkillBase<robot_skills_msgs::action::DetectObject>
{
public:
  using DetectObject = robot_skills_msgs::action::DetectObject;
  using Base = robot_skill_atoms::SkillBase<DetectObject>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MockDetectObject(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_detect_object_skill", "/skill_atoms/detect_object", options)
  {
    this->declare_parameter("detect_object_delay", 0.8);
    this->declare_parameter("mock_detection_x", 0.4);
    this->declare_parameter("mock_detection_y", 0.0);
    this->declare_parameter("mock_detection_z", 0.1);
    this->declare_parameter("mock_detection_class", "block");
    this->declare_parameter("mock_detection_confidence", 0.95);
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
      "robot_initialized", "camera_running", "camera_frame_available_in_tf"
    };
    desc.postconditions = {"object_poses_available_on_blackboard"};
    desc.effects = {"(object-detected ?object ?pose)", "(object-pose-known ?object)"};
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
    const double delay = this->get_parameter("detect_object_delay").as_double();

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] DetectObject: class='%s' (simulating %.1fs)",
      goal->object_class.c_str(), delay);

    auto feedback = std::make_shared<DetectObject::Feedback>();
    feedback->detections_so_far = 0;
    feedback->status_message = "Scanning for objects...";
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      return result;
    }

    // Return a canned detection
    robot_skills_msgs::msg::DetectedObject det;
    det.object_id = "mock_object_1";
    det.class_name = this->get_parameter("mock_detection_class").as_string();
    det.confidence = this->get_parameter("mock_detection_confidence").as_double();

    det.pose.header.frame_id = "world";
    det.pose.header.stamp = this->now();
    det.pose.pose.position.x = this->get_parameter("mock_detection_x").as_double();
    det.pose.pose.position.y = this->get_parameter("mock_detection_y").as_double();
    det.pose.pose.position.z = this->get_parameter("mock_detection_z").as_double();
    det.pose.pose.orientation.w = 1.0;

    det.width = 0.05;
    det.height = 0.05;
    det.depth = 0.05;

    // Apply class filter
    if (!goal->object_class.empty() && goal->object_class != det.class_name) {
      result->success = false;
      result->message = "[MOCK] No '" + goal->object_class + "' objects detected";
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      return result;
    }

    result->detections.push_back(det);
    result->success = true;
    result->message = "[MOCK] Detected 1 object(s)";

    feedback->detections_so_far = 1;
    feedback->status_message = "Detection complete";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return result;
  }
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockDetectObject)
