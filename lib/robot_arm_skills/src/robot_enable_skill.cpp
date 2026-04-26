#include "robot_arm_skills/skill_base.hpp"

#include <memory>
#include <string>
#include <vector>

#include "robot_skills_msgs/action/robot_enable.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"

namespace robot_arm_skills
{

class RobotEnableSkill
  : public SkillBase<robot_skills_msgs::action::RobotEnable>
{
public:
  using RobotEnable = robot_skills_msgs::action::RobotEnable;
  using Base = SkillBase<RobotEnable>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit RobotEnableSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<RobotEnable>(
      "robot_enable_skill",
      "/skill_atoms/robot_enable",
      options)
  {
    this->declare_parameter("controllers_to_activate",
      std::vector<std::string>{"joint_trajectory_controller"});
    this->declare_parameter("switch_controller_service",
      "/controller_manager/switch_controller");
    this->declare_parameter("list_controllers_service",
      "/controller_manager/list_controllers");
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "robot_enable";
    desc.display_name = "Robot Enable/Disable";
    desc.description =
      "Enable (servo on) or disable (servo off) the robot by activating or "
      "deactivating ros2_control controllers.";
    desc.version = "1.0.0";
    desc.category = "system";
    desc.tags = {"system", "enable", "disable", "controller"};

    desc.preconditions = {};
    desc.postconditions = {"robot_enabled_state_set"};

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["enable"],
      "properties": {
        "enable": {
          "type": "boolean",
          "description": "true=enable (servo on), false=disable (servo off)"
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/RobotEnable";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<RobotEnable::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(),
      "Executing RobotEnable: enable=%s", goal->enable ? "true" : "false");

    auto feedback = std::make_shared<RobotEnable::Feedback>();

    try {
      const auto controllers = this->get_parameter("controllers_to_activate")
        .as_string_array();
      const auto switch_srv = this->get_parameter("switch_controller_service")
        .as_string();

      auto client = this->create_client<controller_manager_msgs::srv::SwitchController>(
        switch_srv);

      if (!client->wait_for_service(std::chrono::seconds(5))) {
        result->success = false;
        result->message = "SwitchController service not available at " + switch_srv;
        return result;
      }

      auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

      if (goal->enable) {
        request->activate_controllers = controllers;
        request->strictness =
          controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        feedback->status_message = "Activating controllers...";
      } else {
        request->deactivate_controllers = controllers;
        request->strictness =
          controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        feedback->status_message = "Deactivating controllers...";
      }
      goal_handle->publish_feedback(feedback);

      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future,
            std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS)
      {
        result->success = false;
        result->message = "SwitchController service call timed out";
        return result;
      }

      const auto response = future.get();
      result->is_enabled = goal->enable;

      if (response->ok) {
        result->success = true;
        result->message = goal->enable
          ? "Robot enabled (controllers activated)"
          : "Robot disabled (controllers deactivated)";
      } else {
        result->success = false;
        result->message = "SwitchController returned failure";
      }

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
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::RobotEnableSkill)
