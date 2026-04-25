#!/usr/bin/env python3
"""
Main entry point for the Robot Skill Server.

Starts all three components:
  - SkillRegistry: skill catalog + registration services
  - TaskComposer: BT XML generation from skill step lists
  - BtExecutor: BT execution action server

Usage:
  ros2 run robot_skill_server skill_server_node
  ros2 launch robot_skill_server skill_server.launch.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_skill_server.skill_registry import SkillRegistry
from robot_skill_server.task_composer import TaskComposer
from robot_skill_server.bt_executor import BtExecutor
from robot_skill_server.lease_broker import LeaseBroker


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=6)

    # SkillRegistry is shared by TaskComposer (needs skill metadata for XML gen)
    registry = SkillRegistry()
    composer = TaskComposer(skill_registry=registry)
    lease_broker = LeaseBroker()
    bt_executor = BtExecutor()

    executor.add_node(registry)
    executor.add_node(composer)
    executor.add_node(lease_broker)
    executor.add_node(bt_executor)

    try:
        registry.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════════════════╗\n"
            "║         Robot Skills Framework - Skill Server        ║\n"
            "╠══════════════════════════════════════════════════════╣\n"
            "║  Services:                                           ║\n"
            "║    /skill_server/get_skill_descriptions              ║\n"
            "║    /skill_server/compose_task                        ║\n"
            "║    /skill_server/register_compound_skill             ║\n"
            "║    /skill_server/register_skill  (for atoms)         ║\n"
            "║    /skill_server/acquire_lease                       ║\n"
            "║    /skill_server/renew_lease                         ║\n"
            "║    /skill_server/release_lease                       ║\n"
            "║  Actions:                                            ║\n"
            "║    /skill_server/execute_behavior_tree               ║\n"
            "║  Topics:                                             ║\n"
            "║    /skill_server/task_state (monitor)                ║\n"
            "║    /skill_server/available_skills (JSON list)        ║\n"
            "║    /skill_server/leases (JSON list)                  ║\n"
            "║    /skill_server/lease_events                        ║\n"
            "╚══════════════════════════════════════════════════════╝"
        )
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        registry.destroy_node()
        composer.destroy_node()
        lease_broker.destroy_node()
        bt_executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
