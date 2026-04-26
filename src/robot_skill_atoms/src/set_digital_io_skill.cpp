#include "robot_skill_atoms/skill_base.hpp"

#include <memory>
#include <string>
#include <unordered_map>

#include "robot_skills_msgs/action/set_digital_io.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

namespace robot_skill_atoms
{

class SetDigitalIOSkill
  : public SkillBase<robot_skills_msgs::action::SetDigitalIO>
{
public:
  using SetDigitalIO = robot_skills_msgs::action::SetDigitalIO;
  using Base = SkillBase<SetDigitalIO>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit SetDigitalIOSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<SetDigitalIO>(
      "set_digital_io_skill",
      "/skill_atoms/set_digital_io",
      options)
  {
    // Pin state is tracked in-memory until the actual Meca500 GPIO interface
    // is wired up. Set/get operations are logged and tracked.
    pin_state_["vacuum"] = false;
    pin_state_["tool_changer"] = false;
    pin_state_["led"] = false;
    pin_state_["workspace_light"] = false;
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "set_digital_io";
    desc.display_name = "Set Digital I/O";
    desc.description =
      "Set or read a digital I/O pin. Tracks pin state in-memory. "
      "Wire to Meca500 GPIO interface when available.";
    desc.version = "1.0.0";
    desc.category = "io";
    desc.tags = {"io", "digital", "gpio"};

    desc.preconditions = {"robot_initialized"};
    desc.postconditions = {"io_state_set"};

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["pin_name"],
      "properties": {
        "pin_name": {
          "type": "string",
          "description": "Logical pin name (vacuum, tool_changer, led, workspace_light)"
        },
        "value": {
          "type": "boolean",
          "description": "true=HIGH, false=LOW"
        },
        "read_only": {
          "type": "boolean",
          "default": false
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/SetDigitalIO";
    desc.is_compound = false;
    return desc;
  }

  robot_skills_msgs::msg::SkillAdvertisement getAdvertisement() override
  {
    // Default PascalCase derivation would yield "SetDigitalIo"; the
    // canonical BT tag uses the all-caps "IO" acronym, so set it explicitly.
    robot_skills_msgs::msg::SkillAdvertisement ad;
    ad.bt_tag = "SetDigitalIO";
    return ad;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    if (goal->pin_name.empty()) {
      return {false, "pin_name must not be empty"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<SetDigitalIO::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(),
      "Executing SetDigitalIO: pin='%s' value=%s read_only=%s",
      goal->pin_name.c_str(),
      goal->value ? "HIGH" : "LOW",
      goal->read_only ? "true" : "false");

    auto feedback = std::make_shared<SetDigitalIO::Feedback>();

    try {
      std::lock_guard<std::mutex> lock(pin_mutex_);

      if (goal->read_only) {
        feedback->status_message = "Reading pin '" + goal->pin_name + "'...";
        goal_handle->publish_feedback(feedback);

        auto it = pin_state_.find(goal->pin_name);
        if (it != pin_state_.end()) {
          result->current_value = it->second;
          result->success = true;
          result->message = "Pin '" + goal->pin_name + "' = " +
            (it->second ? "HIGH" : "LOW");
        } else {
          result->current_value = false;
          result->success = true;
          result->message = "Pin '" + goal->pin_name + "' not found, defaulting to LOW";
        }
      } else {
        feedback->status_message = "Setting pin '" + goal->pin_name + "' to " +
          (goal->value ? "HIGH" : "LOW") + "...";
        goal_handle->publish_feedback(feedback);

        pin_state_[goal->pin_name] = goal->value;
        result->current_value = goal->value;
        result->success = true;
        result->message = "Set pin '" + goal->pin_name + "' = " +
          (goal->value ? "HIGH" : "LOW");
      }

      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }

private:
  std::mutex pin_mutex_;
  std::unordered_map<std::string, bool> pin_state_;
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::SetDigitalIOSkill)
