#include "robot_arm_skills/skill_base.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "robot_skills_msgs/action/gripper_control.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_mock_skill_atoms
{

class MockGripperControl
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::GripperControl>
{
public:
  using GripperControl = robot_skills_msgs::action::GripperControl;
  using Base = robot_arm_skills::SkillBase<GripperControl>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit MockGripperControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_gripper_control_skill", "/skill_atoms/gripper_control", options)
  {
    this->declare_parameter("gripper_action_server", "/gripper_controller/gripper_cmd");
    this->declare_parameter("open_position", 0.085);
    this->declare_parameter("closed_position", 0.0);
    this->declare_parameter("default_force_limit", 50.0);
    this->declare_parameter("default_speed", 0.1);
    this->declare_parameter("gripper_control_delay", 0.5);
    this->declare_parameter("simulate_grasp", true);

    // Publish gripper finger positions to mock joint state publisher
    // Message format: [finger1_pos, finger2_pos] (appended after 7 arm joints)
    joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mock/gripper_target", 10);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "gripper_control";
    desc.display_name = "Gripper Control";
    desc.description =
      "Control the robot gripper: open, close, or set to a specific position. "
      "Close command with force limit allows detecting when an object is grasped "
      "(force-controlled grasping). Object grasped is indicated in result.";
    desc.version = "1.0.0";
    desc.category = "manipulation";
    desc.tags = {"manipulation", "gripper", "grasp", "release"};

    desc.preconditions = {"robot_initialized", "no_estop_active", "gripper_controller_running"};
    desc.postconditions = {"gripper_at_position"};
    desc.effects = {
      "(gripper-open ?robot)",
      "(gripper-closed ?robot)",
      "(object-grasped ?robot ?object)"
    };
    desc.constraints = {
      "position must be 0.0-1.0 (normalized) for set_position command",
      "force_limit in Newtons, 0 = use default",
      "do not close gripper while EEF is inside an object"
    };

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["command"],
      "properties": {
        "command": {
          "type": "string",
          "enum": ["open", "close", "set_position"],
          "description": "Gripper command"
        },
        "position": {
          "type": "number",
          "description": "Target position (0.0=closed, 1.0=open), used with set_position",
          "minimum": 0.0,
          "maximum": 1.0,
          "default": 0.5
        },
        "force_limit": {
          "type": "number",
          "description": "Maximum grip force in Newtons (0 = default)",
          "minimum": 0.0,
          "default": 0.0
        }
      }
    })json";

    desc.pddl_action = R"(
(:action close-gripper
  :parameters (?robot - robot)
  :precondition (and
    (robot-initialized ?robot)
    (not (estop-active))
    (gripper-open ?robot)
  )
  :effect (and
    (gripper-closed ?robot)
    (not (gripper-open ?robot))
  )
))";

    desc.action_type = "robot_skills_msgs/action/GripperControl";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    const std::set<std::string> valid_commands = {"open", "close", "set_position"};
    if (valid_commands.find(goal->command) == valid_commands.end()) {
      return {false, "Invalid command '" + goal->command + "'. Must be: open, close, set_position"};
    }
    if (goal->command == "set_position" && (goal->position < 0.0 || goal->position > 1.0)) {
      return {false, "position must be 0.0-1.0 for set_position command"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<GripperControl::Result>();
    const auto & goal = goal_handle->get_goal();
    const double delay = this->get_parameter("gripper_control_delay").as_double();
    const bool simulate_grasp = this->get_parameter("simulate_grasp").as_bool();

    RCLCPP_INFO(this->get_logger(),
      "[MOCK] GripperControl: command='%s' (simulating %.1fs)",
      goal->command.c_str(), delay);

    // Compute target finger position
    const double open_pos = this->get_parameter("open_position").as_double();
    const double closed_pos = this->get_parameter("closed_position").as_double();
    double target_pos = 0.0;

    if (goal->command == "open") {
      target_pos = open_pos;
    } else if (goal->command == "close") {
      target_pos = closed_pos;
    } else {
      target_pos = closed_pos + goal->position * (open_pos - closed_pos);
    }

    // Publish gripper target for joint state publisher
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {target_pos, target_pos};
    joint_target_pub_->publish(msg);

    // Simulate execution
    auto feedback = std::make_shared<GripperControl::Feedback>();
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      return result;
    }

    feedback->current_position = target_pos;
    goal_handle->publish_feedback(feedback);

    result->object_grasped = (goal->command == "close") && simulate_grasp;
    // When simulating a grasp, the fingers stop partway closed (object between them)
    if (result->object_grasped) {
      target_pos = open_pos * 0.25;  // 25% open — something is gripped
    }
    result->final_position = target_pos;
    result->success = true;
    result->message = result->object_grasped
      ? "[MOCK] Gripper closed - object grasped (simulated)"
      : "[MOCK] Gripper '" + goal->command + "' complete";

    RCLCPP_INFO(this->get_logger(), "%s (pos=%.4f)", result->message.c_str(), target_pos);
    return result;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockGripperControl)
