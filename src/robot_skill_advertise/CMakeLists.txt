cmake_minimum_required(VERSION 3.8)
project(robot_skill_advertise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_skills_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Python module ──────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── C++ header-only library ────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rclcpp robot_skills_msgs builtin_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
