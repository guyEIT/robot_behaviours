cmake_minimum_required(VERSION 3.8)
project(robot_skill_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Python package ────────────────────────────────────────────────────────────
# Pure Python — tree execution uses rclpy action clients directly.
# No C++ build needed.
ament_python_install_package(robot_skill_server)

install(PROGRAMS
  robot_skill_server/skill_server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME skill_server_node
)

# bt_executor_mcp moved to agent/robot_skill_mcp — it runs on the agent's
# machine, not the orchestrator (see plan section C, "Trust model").

# ── Launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Config files ──────────────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
