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
find_package(robot_skills_msgs REQUIRED)
find_package(robot_bt_nodes REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Python package ────────────────────────────────────────────────────────────
ament_python_install_package(robot_skill_server)

install(PROGRAMS
  robot_skill_server/skill_server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME skill_server_node
)

# ── C++ bt_runner executable ──────────────────────────────────────────────────
add_executable(bt_runner src/bt_runner.cpp)
target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  robot_skills_msgs
  robot_bt_nodes
  geometry_msgs
)

install(TARGETS bt_runner
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
