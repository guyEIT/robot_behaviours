#include "robot_arm_skills/skill_base.hpp"
#include <chrono>
#include <set>
#include <thread>
#include "robot_skills_msgs/action/update_planning_scene.hpp"

namespace robot_mock_skill_atoms {

class MockUpdatePlanningScene
  : public robot_arm_skills::SkillBase<robot_skills_msgs::action::UpdatePlanningScene>
{
public:
  using Action = robot_skills_msgs::action::UpdatePlanningScene;
  using Base = robot_arm_skills::SkillBase<Action>;

  explicit MockUpdatePlanningScene(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Base("mock_update_planning_scene_skill", "/skill_atoms/update_planning_scene", options)
  {
    this->declare_parameter("scene_delay", 0.2);
  }

  robot_skills_msgs::msg::SkillDescription getDescription() override
  {
    robot_skills_msgs::msg::SkillDescription desc;
    desc.name = "update_planning_scene";
    desc.display_name = "Update Planning Scene";
    desc.description = "Add or remove collision objects in the planning scene.";
    desc.category = "utility";
    desc.action_type = "robot_skills_msgs/action/UpdatePlanningScene";
    return desc;
  }

  std::shared_ptr<Action::Result> executeGoal(
    const std::shared_ptr<GoalHandle> goal_handle) override
  {
    auto result = std::make_shared<Action::Result>();
    const auto & goal = goal_handle->get_goal();
    double delay = this->get_parameter("scene_delay").as_double();
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    if (goal->operation == "add") {
      scene_objects_.insert(goal->object_id);
      RCLCPP_INFO(this->get_logger(), "[MOCK] Added '%s' to scene", goal->object_id.c_str());
    } else if (goal->operation == "remove") {
      scene_objects_.erase(goal->object_id);
      RCLCPP_INFO(this->get_logger(), "[MOCK] Removed '%s' from scene", goal->object_id.c_str());
    } else if (goal->operation == "clear_all") {
      scene_objects_.clear();
      RCLCPP_INFO(this->get_logger(), "[MOCK] Cleared all scene objects");
    }

    result->success = true;
    result->scene_object_ids.assign(scene_objects_.begin(), scene_objects_.end());
    result->message = "[MOCK] Scene has " + std::to_string(scene_objects_.size()) + " objects";
    return result;
  }

private:
  std::set<std::string> scene_objects_;
};

}  // namespace robot_mock_skill_atoms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_mock_skill_atoms::MockUpdatePlanningScene)
