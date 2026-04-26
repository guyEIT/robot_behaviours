#include "robot_arm_skills/skill_base.hpp"

#include <memory>
#include <string>

#include "robot_skills_msgs/action/check_collision.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"

namespace robot_arm_skills
{

class CheckCollisionSkill
  : public SkillBase<robot_skills_msgs::action::CheckCollision>
{
public:
  using CheckCollision = robot_skills_msgs::action::CheckCollision;
  using Base = SkillBase<CheckCollision>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit CheckCollisionSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<CheckCollision>(
      "check_collision_skill",
      "/skill_atoms/check_collision",
      options)
  {
    this->declare_parameter("default_planning_group", "arm");
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "check_collision";
    desc.display_name = "Check Collision";
    desc.description =
      "Check if a given joint configuration or the current robot state is in collision "
      "with the planning scene.";
    desc.version = "1.0.0";
    desc.category = "scene";
    desc.tags = {"scene", "collision", "safety"};

    desc.preconditions = {"planning_scene_valid"};
    desc.postconditions = {"collision_status_known"};

    desc.parameters_schema = R"json({
      "type": "object",
      "properties": {
        "joint_positions": {
          "type": "array",
          "items": {"type": "number"},
          "description": "Joint angles to check (empty = check current state)"
        },
        "planning_group": {
          "type": "string",
          "default": "arm"
        }
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/CheckCollision";
    desc.is_compound = false;
    return desc;
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<CheckCollision::Result>();
    const auto & goal = goal_handle->get_goal();

    const std::string planning_group = goal->planning_group.empty()
      ? this->get_parameter("default_planning_group").as_string()
      : goal->planning_group;

    RCLCPP_INFO(this->get_logger(),
      "Executing CheckCollision: group='%s' joints=%s",
      planning_group.c_str(),
      goal->joint_positions.empty() ? "current_state" : "specified");

    auto feedback = std::make_shared<CheckCollision::Feedback>();
    feedback->status_message = "Fetching planning scene...";
    goal_handle->publish_feedback(feedback);

    try {
      // Get the current planning scene from the service
      auto client = this->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene");

      if (!client->wait_for_service(std::chrono::seconds(5))) {
        result->success = false;
        result->message = "GetPlanningScene service not available";
        return result;
      }

      auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future,
            std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
      {
        result->success = false;
        result->message = "Failed to get planning scene";
        return result;
      }

      // Build the planning scene from the response
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), planning_group);
      auto robot_model = move_group.getRobotModel();

      planning_scene::PlanningScene ps(robot_model);
      ps.setPlanningSceneMsg(future.get()->scene);

      // Get the state to check
      auto & state = ps.getCurrentStateNonConst();
      if (!goal->joint_positions.empty()) {
        const auto * jmg = robot_model->getJointModelGroup(planning_group);
        if (!jmg) {
          result->success = false;
          result->message = "Unknown planning group: " + planning_group;
          return result;
        }
        if (goal->joint_positions.size() != jmg->getVariableCount()) {
          result->success = false;
          result->message = "Expected " + std::to_string(jmg->getVariableCount()) +
            " joint values, got " + std::to_string(goal->joint_positions.size());
          return result;
        }
        state.setJointGroupPositions(jmg, goal->joint_positions);
        state.update();
      }

      feedback->status_message = "Checking collision...";
      goal_handle->publish_feedback(feedback);

      // Check collision
      collision_detection::CollisionRequest collision_request;
      collision_request.contacts = true;
      collision_request.max_contacts = 50;
      collision_detection::CollisionResult collision_result;
      ps.checkCollision(collision_request, collision_result, state);

      result->in_collision = collision_result.collision;

      // Extract colliding link names
      for (const auto & contact_pair : collision_result.contacts) {
        const auto & link1 = contact_pair.first.first;
        const auto & link2 = contact_pair.first.second;
        if (std::find(result->colliding_links.begin(), result->colliding_links.end(),
              link1) == result->colliding_links.end())
        {
          result->colliding_links.push_back(link1);
        }
        if (std::find(result->colliding_links.begin(), result->colliding_links.end(),
              link2) == result->colliding_links.end())
        {
          result->colliding_links.push_back(link2);
        }
      }

      result->success = true;
      result->message = result->in_collision
        ? "State is in collision (" + std::to_string(result->colliding_links.size()) + " links)"
        : "State is collision-free";

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
RCLCPP_COMPONENTS_REGISTER_NODE(robot_arm_skills::CheckCollisionSkill)
