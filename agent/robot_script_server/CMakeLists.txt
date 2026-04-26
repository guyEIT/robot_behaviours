cmake_minimum_required(VERSION 3.8)
project(robot_script_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/script_action_server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME script_action_server
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
