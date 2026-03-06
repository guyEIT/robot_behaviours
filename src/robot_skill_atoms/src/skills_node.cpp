/**
 * @brief Main entry point for all robot skill atom servers.
 *
 * This node uses rclcpp_components ComponentManager to load all skill servers
 * as composable components, enabling shared memory and efficient communication.
 *
 * Alternatively, each skill can be run as an independent node for isolation.
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Use a MultiThreadedExecutor: skill servers run concurrent actions
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Component manager allows loading skills as plugins at runtime
  // Skills are loaded via launch file or ros2 component load commands
  auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "robot_skill_atoms_manager");

  executor->add_node(component_manager);

  RCLCPP_INFO(rclcpp::get_logger("skills_node"),
    "Robot Skill Atoms node started. "
    "Load skill components via: ros2 component load /robot_skill_atoms_manager "
    "robot_skill_atoms <ComponentName>");

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
