cmake_minimum_required(VERSION 3.8)
project(robot_behaviors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(robot_behaviors)

# Install BT XML tree definitions
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
