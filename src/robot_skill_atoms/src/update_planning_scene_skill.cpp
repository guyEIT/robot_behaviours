#include "robot_skill_atoms/skill_base.hpp"

#include <memory>
#include <string>

#include "robot_skills_msgs/action/update_planning_scene.hpp"
#include "robot_skills_msgs/msg/skill_description.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_skill_atoms
{

class UpdatePlanningSceneSkill
  : public SkillBase<robot_skills_msgs::action::UpdatePlanningScene>
{
public:
  using UpdatePlanningScene = robot_skills_msgs::action::UpdatePlanningScene;
  using Base = SkillBase<UpdatePlanningScene>;
  using typename Base::GoalHandle;
  using typename Base::Goal;
  using typename Base::Result;
  using typename Base::Feedback;

  explicit UpdatePlanningSceneSkill(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SkillBase<UpdatePlanningScene>(
      "update_planning_scene_skill",
      "/skill_atoms/update_planning_scene",
      options)
  {
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "update_planning_scene";
    desc.display_name = "Update Planning Scene";
    desc.description =
      "Add, remove, or clear collision objects in the MoveIt2 planning scene. "
      "Supports box, cylinder, and sphere shapes.";
    desc.version = "1.0.0";
    desc.category = "scene";
    desc.tags = {"scene", "collision", "planning"};

    desc.preconditions = {"planning_scene_valid"};
    desc.postconditions = {"planning_scene_updated"};

    desc.parameters_schema = R"json({
      "type": "object",
      "required": ["object_id", "operation"],
      "properties": {
        "object_id": {"type": "string"},
        "operation": {"type": "string", "enum": ["add", "remove", "clear_all"]},
        "pose": {"type": "object"},
        "dimensions": {"type": "array", "items": {"type": "number"}},
        "shape_type": {"type": "string", "enum": ["box", "cylinder", "sphere"], "default": "box"}
      }
    })json";

    desc.action_type = "robot_skills_msgs/action/UpdatePlanningScene";
    desc.is_compound = false;
    return desc;
  }

  std::pair<bool, std::string> checkPreconditions(
    const std::shared_ptr<const Goal> goal) override
  {
    const auto & op = goal->operation;
    if (op != "add" && op != "remove" && op != "clear_all") {
      return {false, "operation must be 'add', 'remove', or 'clear_all'"};
    }
    if (op == "add" && goal->object_id.empty()) {
      return {false, "object_id is required for 'add' operation"};
    }
    if (op == "add" && goal->dimensions.size() < 3) {
      return {false, "dimensions must have at least 3 values for 'add' operation"};
    }
    if (op == "remove" && goal->object_id.empty()) {
      return {false, "object_id is required for 'remove' operation"};
    }
    return {true, ""};
  }

  std::shared_ptr<Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<UpdatePlanningScene::Result>();
    const auto & goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(),
      "Executing UpdatePlanningScene: op='%s' id='%s'",
      goal->operation.c_str(), goal->object_id.c_str());

    auto feedback = std::make_shared<UpdatePlanningScene::Feedback>();

    try {
      moveit::planning_interface::PlanningSceneInterface psi;

      if (goal->operation == "add") {
        feedback->status_message = "Adding collision object '" + goal->object_id + "'...";
        goal_handle->publish_feedback(feedback);

        moveit_msgs::msg::CollisionObject co;
        co.id = goal->object_id;
        co.header = goal->pose.header;
        if (co.header.frame_id.empty()) {
          co.header.frame_id = "world";
        }
        co.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        const std::string shape = goal->shape_type.empty() ? "box" : goal->shape_type;

        if (shape == "box") {
          primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
          primitive.dimensions = {goal->dimensions[0], goal->dimensions[1], goal->dimensions[2]};
        } else if (shape == "cylinder") {
          primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          // dimensions: [height, radius]
          primitive.dimensions = {goal->dimensions[2], goal->dimensions[0]};
        } else if (shape == "sphere") {
          primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
          primitive.dimensions = {goal->dimensions[0]};
        } else {
          result->success = false;
          result->message = "Unknown shape_type: " + shape;
          return result;
        }

        co.primitives.push_back(primitive);
        co.primitive_poses.push_back(goal->pose.pose);

        psi.applyCollisionObject(co);

        result->success = true;
        result->message = "Added collision object '" + goal->object_id + "'";

      } else if (goal->operation == "remove") {
        feedback->status_message = "Removing collision object '" + goal->object_id + "'...";
        goal_handle->publish_feedback(feedback);

        psi.removeCollisionObjects({goal->object_id});

        result->success = true;
        result->message = "Removed collision object '" + goal->object_id + "'";

      } else if (goal->operation == "clear_all") {
        feedback->status_message = "Clearing all collision objects...";
        goal_handle->publish_feedback(feedback);

        auto known_objects = psi.getKnownObjectNames();
        if (!known_objects.empty()) {
          psi.removeCollisionObjects(known_objects);
        }

        result->success = true;
        result->message = "Cleared " + std::to_string(known_objects.size()) +
          " collision objects";
      }

      // Return current scene object list
      result->scene_object_ids = psi.getKnownObjectNames();

      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());

    } catch (const std::exception & e) {
      result->success = false;
      result->message = "Exception: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    }

    return result;
  }
};

}  // namespace robot_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_skill_atoms::UpdatePlanningSceneSkill)
