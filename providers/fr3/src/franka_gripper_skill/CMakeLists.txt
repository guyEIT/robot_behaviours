cmake_minimum_required(VERSION 3.8)
project(franka_gripper_skill)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/franka_gripper_skill_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_gripper_skill_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
