cmake_minimum_required(VERSION 3.8)
project(robot_dashboard)

find_package(ament_cmake REQUIRED)

# Install pre-built frontend (run `npm ci && npm run build` in frontend/ before colcon build)
install(DIRECTORY frontend/dist/
  DESTINATION share/${PROJECT_NAME}/www
  OPTIONAL  # Allow colcon build to succeed even if frontend hasn't been built yet
)

# Install launch and config
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
