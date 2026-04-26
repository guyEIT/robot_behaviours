/**
 * @brief Main entry point for all mock robot skill servers.
 *
 * Drop-in replacement for robot_arm_skills/skills_node.
 * Uses rclcpp_components ComponentManager to load mock skills
 * as composable components.
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "robot_mock_skill_atoms_manager");

  executor->add_node(component_manager);

  RCLCPP_INFO(rclcpp::get_logger("mock_skills_node"),
    "Mock Skill Atoms node started (lite sim mode). "
    "Load components via: ros2 component load /robot_mock_skill_atoms_manager "
    "robot_mock_skill_atoms <ComponentName>");

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
