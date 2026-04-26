cmake_minimum_required(VERSION 3.8)
project(robot_skill_mcp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mcp_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bt_executor_mcp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
