cmake_minimum_required(VERSION 3.8)
project(robot_skill_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

# ── Python package ────────────────────────────────────────────────────────────
ament_python_install_package(robot_skill_server)

install(PROGRAMS
  robot_skill_server/skill_server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME skill_server_node
)

# ── C++ tree execution server ─────────────────────────────────────────────────
# Minimal subclass of BT::TreeExecutionServer from BehaviorTree.ROS2.
# BT node plugins are loaded at runtime via the `plugins` parameter.
add_executable(robot_tree_server src/robot_tree_server.cpp)
ament_target_dependencies(robot_tree_server
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
)

install(TARGETS robot_tree_server
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Config files ──────────────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ── Behavior tree XML files ────────────────────────────────────────────────────
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_task_composer test/test_task_composer.py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
    TIMEOUT 60
  )
  ament_add_pytest_test(test_skill_registry test/test_skill_registry.py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
    TIMEOUT 60
  )
endif()

ament_package()
