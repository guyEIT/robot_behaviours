cmake_minimum_required(VERSION 3.8)
project(meca500_skill_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python entry points (manifest publisher node)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/manifest_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME meca500_manifest_publisher
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
