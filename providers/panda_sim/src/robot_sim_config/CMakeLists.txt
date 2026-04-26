cmake_minimum_required(VERSION 3.8)
project(robot_sim_config)

find_package(ament_cmake REQUIRED)

# Install all config files so they're accessible via ament_index
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
